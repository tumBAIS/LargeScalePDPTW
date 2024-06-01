use std::fs::File;
use std::io::{BufRead, Write};
use std::io::{BufReader, BufWriter};
use std::sync;
use std::sync::Mutex;

use log::trace;
#[cfg(feature = "parallel")]
use rayon::iter::{IndexedParallelIterator, IntoParallelRefIterator, ParallelIterator};
use tinyvec::ArrayVec;
use took::Timer;

use crate::pooling::preprocess::{CalculationResult, CombinationCalculator};
use crate::problem::pdptw::{Node, PDPTWInstance};
use crate::problem::Num;
use crate::refn::{ForwardREF, REFData, REFNode};
use crate::utils::{DefaultSearchTracker, SearchProgressIterationTracker};

pub mod matching;
mod preprocess;

pub const MAX_REQUESTS_PER_POOLING: usize = 4;

pub struct Poolings {
    pub list: Vec<(ArrayVec<[usize; MAX_REQUESTS_PER_POOLING]>, Num)>,
}

pub fn process_requests(instance: &PDPTWInstance) -> Poolings {
    let mut sorted_pickup_nodes: Vec<&Node> = instance.iter_pickups().collect();
    sorted_pickup_nodes.sort_unstable_by_key(|a| a.ready);

    let delta = Num::ZERO;

    let sorted_requests: Vec<usize> = sorted_pickup_nodes
        .iter()
        .map(|p| instance.request_id(p.id))
        .collect();

    #[cfg(feature = "parallel")]
        let iter = sorted_pickup_nodes.par_iter();
    #[cfg(not(feature = "parallel"))]
        let iter = sorted_pickup_nodes.iter();

    let heuristic_ranges = iter
        .take(sorted_pickup_nodes.len())
        .map(|p| instance.delivery_of(p.id))
        .enumerate()
        .map(|(begin, delivery)| {
            let mut iter = sorted_pickup_nodes[(begin + 1)..]
                .iter()
                .enumerate()
                .skip_while(|(_, other_pickup)| other_pickup.ready - delivery.due <= delta);
            let m = iter
                .next()
                .map_or(sorted_pickup_nodes.len() - (begin + 1), |(i, _)| i);
            &sorted_requests[begin..(begin + m + 1)]
        })
        .collect::<Vec<&[usize]>>();

    let timer = Timer::new();
    let poolings = Poolings {
        list: get_combinations_from_ranges_incrementally(instance, &heuristic_ranges),
    };
    log::info!(
        "found {} poolings after {}",
        poolings.list.len(),
        timer.took()
    );

    poolings
}

fn get_combinations_from_ranges_incrementally(
    instance: &PDPTWInstance,
    ranges: &Vec<&[usize]>,
) -> Vec<(ArrayVec<[usize; MAX_REQUESTS_PER_POOLING]>, Num)> {
    let calc = CombinationCalculator::new(MAX_REQUESTS_PER_POOLING, 3);
    check_combinations_from_ranges_incrementally(instance, &calc, ranges)
}

fn check_combinations_from_ranges_incrementally(
    instance: &PDPTWInstance,
    calc: &CombinationCalculator,
    ranges: &Vec<&[usize]>,
) -> Vec<(ArrayVec<[usize; MAX_REQUESTS_PER_POOLING]>, Num)> {
    let pb = sync::Arc::new(Mutex::new(DefaultSearchTracker::new(ranges.len() as u64)));
    let pb_inner = &pb;

    #[cfg(feature = "parallel")]
        let iter = ranges.par_iter();
    #[cfg(not(feature = "parallel"))]
        let iter = ranges.iter();

    let res = iter
        .map(move |it| {
            // start with the first four (or less)
            let mut arr = ArrayVec::<[usize; MAX_REQUESTS_PER_POOLING]>::from_iter(
                it.iter().cloned().take(MAX_REQUESTS_PER_POOLING),
            );
            let mut counters = ArrayVec::<[usize; MAX_REQUESTS_PER_POOLING - 1]>::from_iter(
                (1..MAX_REQUESTS_PER_POOLING).into_iter(),
            );
            let mut k_result = ArrayVec::<[Num; MAX_REQUESTS_PER_POOLING - 1]>::new();

            let mut res = Vec::new();

            'main: loop {
                #[cfg(debug_assertions)]
                for a in arr.windows(2) {
                    debug_assert_ne!(a[0], a[1], "{:?}", arr.as_slice());
                }

                for k in (k_result.len() + 2)
                    ..(MAX_REQUESTS_PER_POOLING.min(it.len()).min(arr.len()) + 1)
                {
                    // check and store
                    match calc.calc_min_any_order_request_ids_with_ub(
                        instance,
                        &arr[0..k],
                        None,
                    ) {
                        CalculationResult::FoundMin(cost) => {
                            res.push((ArrayVec::from_iter(arr.iter().cloned().take(k)), cost));
                            k_result.push(cost);
                        }
                        CalculationResult::WasDominated(cost) => {
                            k_result.push(cost);
                        }
                        CalculationResult::NoneFound => {
                            k_result.push(Num::MAX);
                            break;
                        }
                    }
                }

                for i in (1..(k_result.len() + 1)).rev() {
                    debug_assert_eq!(i, k_result.len());
                    counters[i - 1] += 1;
                    k_result.pop();

                    if counters[i - 1] < it.len() {
                        arr[i] = it[counters[i - 1]];
                        let mut j = i + 1;
                        while j < MAX_REQUESTS_PER_POOLING
                            .min(arr.len())
                            .min(it.len() - counters[i - 1])
                        {
                            counters[j - 1] = counters[j - 2] + 1;
                            arr[j] = it[counters[j - 1]];
                            j += 1;
                        }
                        if j < MAX_REQUESTS_PER_POOLING {
                            arr.truncate(j);
                        }
                        continue 'main;
                    }
                }
                debug_assert_eq!(0, k_result.len());
                break;
            }

            pb_inner.lock().unwrap().inc();
            trace!("{}: {}", it[0], res.len());
            res
        })
        .flatten()
        .collect();
    res
}

pub fn store_poolings<'a>(
    path: impl Into<String>,
    poolings: &Poolings,
) -> anyhow::Result<()> {
    let f = File::create(path.into())?;
    let mut file = BufWriter::new(&f);

    for (request_ids, cost) in &poolings.list {
        write!(
            file,
            "{}",
            request_ids
                .iter()
                .map(|it| format!("{}", it))
                .collect::<Vec<_>>()
                .join("-")
        )?;
        writeln!(file, ",{cost}", cost = cost)?;
    }

    Ok(())
}

pub fn map_poolings_to_itinerary_and_data<'a>(
    instance: &'a PDPTWInstance,
    iter: impl Iterator<Item=&'a (ArrayVec<[usize; MAX_REQUESTS_PER_POOLING]>, Num)> + 'a,
) -> impl Iterator<Item=(Vec<usize>, REFData)> + 'a {
    let calc = CombinationCalculator::new(MAX_REQUESTS_PER_POOLING, 3);
    iter.filter_map(move |(arr, _cost)| {
        if let Some((itinerary, _ref_cost)) =
            calc.get_min_any_order_by_request_ids(instance, arr.as_slice())
        {
            let mut data = REFData::with_node(&REFNode::from(&instance.nodes[itinerary[0]]));
            for i in 1..itinerary.len() {
                data.extend_forward_assign(
                    &REFNode::from(&instance.nodes[itinerary[i]]),
                    instance.distance_and_time(itinerary[i - 1], itinerary[i]),
                );
            }
            Some((itinerary, data))
        } else {
            None
        }
    })
}

#[cfg(feature = "parallel")]
pub fn par_map_poolings_to_itinerary_and_data<'a>(
    instance: &'a PDPTWInstance,
    iter: impl ParallelIterator<Item=&'a (ArrayVec<[usize; MAX_REQUESTS_PER_POOLING]>, Num)> + 'a,
) -> impl ParallelIterator<Item=(Vec<usize>, REFData)> + 'a {
    let calc = CombinationCalculator::new(MAX_REQUESTS_PER_POOLING, 3);
    iter.filter_map(move |(arr, _cost)| {
        if let Some((itinerary, _ref_cost)) =
            calc.get_min_any_order_by_request_ids(instance, arr.as_slice())
        {
            let mut data = REFData::with_node(&REFNode::from(&instance.nodes[itinerary[0]]));
            for i in 1..itinerary.len() {
                data.extend_forward_assign(
                    &REFNode::from(&instance.nodes[itinerary[i]]),
                    instance.distance_and_time(itinerary[i - 1], itinerary[i]),
                );
            }
            Some((itinerary, data))
        } else {
            None
        }
    })
}

pub fn load_poolings<'a>(
    path: impl Into<String>,
) -> anyhow::Result<Poolings> {
    let path = path.into();
    let f = File::open(&path)?;
    let file = BufReader::new(&f);

    log::info!("loading pooling options from {}", path);

    let mut poolings: Vec<(
        ArrayVec<[usize; crate::pooling::MAX_REQUESTS_PER_POOLING]>,
        Num,
    )> = Vec::new();
    let lines = file.lines();
    for line in lines
        .filter_map(|it| it.ok())
        .filter(|it| !it.trim().is_empty())
    {
        let mut split = line.split(",");
        let pooling = split
            .next()
            .unwrap()
            .split("-")
            .map(|it| it.parse::<usize>().unwrap())
            .collect::<ArrayVec<[usize; crate::pooling::MAX_REQUESTS_PER_POOLING]>>();
        let cost = split.next().unwrap().parse::<Num>().unwrap();
        poolings.push((pooling, cost));
    }

    Ok(Poolings { list: poolings })
}
