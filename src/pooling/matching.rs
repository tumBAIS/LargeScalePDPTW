#[cfg(feature = "use-grb")]
use std::cmp::Reverse;

use fixedbitset::FixedBitSet;
#[cfg(feature = "use-grb")]
use grb::expr::{GurobiSum, LinExpr};

#[cfg(feature = "use-grb")]
use rand::Rng;
use tinyvec::ArrayVec;

use crate::pooling::{MAX_REQUESTS_PER_POOLING, Poolings};
use crate::problem::Num;
use crate::problem::pdptw::PDPTWInstance;
use crate::utils::Random;

pub struct Matching {
    pub selected_poolings: Vec<(ArrayVec<[usize; MAX_REQUESTS_PER_POOLING]>, Num)>,
    pub uncovered_requests: FixedBitSet,
}

pub fn greedy_matching(
    instance: &PDPTWInstance,
    poolings: &Poolings,
    weight_fn: PoolingWeightFunction,
) -> Matching {
    // select poolings
    log::info!("select pooling options greedily");
    let gamma_max = MAX_REQUESTS_PER_POOLING;
    let mut sorted_poolings = poolings.list.clone();
    sorted_poolings
        .sort_by_cached_key(|it| Num::from(weight_fn.calculate_weight(instance, it, gamma_max)));
    let mut covered_requests = FixedBitSet::with_capacity(instance.num_requests);
    let mut selected_pooling_entries = FixedBitSet::with_capacity(sorted_poolings.len());
    for (i, pooling) in sorted_poolings.iter().enumerate() {
        if pooling.0.iter().any(|it| covered_requests.contains(*it)) {
            // ignore
        } else {
            selected_pooling_entries.insert(i);
            pooling.0.iter().for_each(|it| covered_requests.insert(*it));
            if covered_requests.count_ones(..) == instance.num_requests {
                break;
            }
        }
    }
    let selected_poolings: Vec<(ArrayVec<[usize; MAX_REQUESTS_PER_POOLING]>, Num)> =
        selected_pooling_entries
            .ones()
            .map(|i| sorted_poolings[i])
            .collect();
    log::info!("subset of {} options selected", selected_poolings.len());

    let mut uncovered_requests = covered_requests;
    uncovered_requests.toggle_range(..);

    Matching {
        selected_poolings,
        uncovered_requests,
    }
}

#[derive(Debug)]
pub enum PoolingWeightFunction {
    NormalizedNumberOfRequestsServed,
    TravelCost,
    TemporalOverlapRatio,
    WeightedTravelCostTemporalOverlapRatio(f64),
    TemporalDeviation,
    WeightedTravelCostTemporalDeviation(f64),
}

impl PoolingWeightFunction {
    fn calculate_weight(
        &self,
        instance: &PDPTWInstance,
        pooling: &(ArrayVec<[usize; MAX_REQUESTS_PER_POOLING]>, Num),
        gamma_max: usize,
    ) -> f64 {
        return match self {
            PoolingWeightFunction::NormalizedNumberOfRequestsServed => 1.0f64,
            PoolingWeightFunction::TravelCost => pooling.1.value() as f64,
            PoolingWeightFunction::TemporalOverlapRatio => {
                1.0f64 - calculate_temporal_overlap_ratio(instance, &pooling.0)
            }
            PoolingWeightFunction::TemporalDeviation => {
                calculate_temporal_deviation(instance, &pooling.0) as f64
            }
            PoolingWeightFunction::WeightedTravelCostTemporalOverlapRatio(rho) => {
                pooling.1.value() as f64 * (1.0f64 - rho)
                    + (1.0f64 - calculate_temporal_overlap_ratio(instance, &pooling.0)) * rho
            }
            PoolingWeightFunction::WeightedTravelCostTemporalDeviation(rho) => {
                pooling.1.value() as f64 * (1.0f64 - rho)
                    + calculate_temporal_deviation(instance, &pooling.0) as f64 * rho
            }
        } * (gamma_max as f64 / pooling.0.len() as f64);
    }

    fn calculate_weight_of_singleton(
        &self,
        instance: &PDPTWInstance,
        request_id: usize,
        gamma_max: usize,
    ) -> f64 {
        let pid = instance.pickup_id_of_request(request_id);
        let did = instance.delivery_of(pid).id;
        let direct = instance.distance(pid, did);
        return match self {
            PoolingWeightFunction::NormalizedNumberOfRequestsServed => 1.0f64,
            PoolingWeightFunction::TravelCost => direct.value() as f64,
            PoolingWeightFunction::TemporalOverlapRatio => 0.0f64,
            PoolingWeightFunction::TemporalDeviation => 0.0f64,
            PoolingWeightFunction::WeightedTravelCostTemporalOverlapRatio(rho) => {
                direct.value() as f64 * (1f64 - rho) + 0f64 * rho
            }
            PoolingWeightFunction::WeightedTravelCostTemporalDeviation(rho) => {
                direct.value() as f64 * (1f64 - rho) + 0f64 * rho
            }
        } * (gamma_max as f64 / 1f64);
    }
}

#[derive(Debug)]
pub enum MatchingProcedure {
    Greedy,
    WeightedSetCover(ModelType),
    WeightedSetPartitioning(ModelType),
}

#[derive(Debug)]
pub enum ModelType {
    Integer,
    Continuous(FractionalSelectionStrategy),
}

#[derive(Debug)]
pub enum FractionalSelectionStrategy {
    GreedyFromSortedFractions,
    StandardRandomizedRounding,
    ConditionalRandomizedRounding,
}

pub struct WeightedSetCoverMatchingParams {
    pub ub: Option<Num>,
    pub time_limit_in_seconds: Option<f64>,
    pub weight_fn: PoolingWeightFunction,
    pub model_type: ModelType,
    pub enforce_partitioning: bool,
    pub enable_solver_stdout_logging: bool,
}

fn calculate_temporal_overlap_ratio(
    instance: &PDPTWInstance,
    pooling: &ArrayVec<[usize; MAX_REQUESTS_PER_POOLING]>,
) -> f64 {
    let (mut tfmin, mut tfmax, mut omin, mut omax) = (Num::MAX.value(), 0, 0, Num::MAX.value());
    for rid in pooling {
        let pid = instance.pickup_id_of_request(*rid);
        let did = instance.delivery_id_of_request(*rid);
        let earliest_start = instance.nodes[pid].ready.value();
        let latest_completion = instance.nodes[did].due.value();
        tfmin = tfmin.min(earliest_start);
        tfmax = tfmax.max(latest_completion);
        omin = omin.max(earliest_start);
        omax = omax.min(latest_completion);
    }
    if omin >= omax {
        0f64
    } else {
        (omax - omin) as f64 / (tfmax - tfmin) as f64
    }
}

fn calculate_temporal_deviation(
    instance: &PDPTWInstance,
    pooling: &ArrayVec<[usize; MAX_REQUESTS_PER_POOLING]>,
) -> usize {
    let (mut tfmin, mut tfmax, mut omin, mut omax) = (Num::MAX.value(), 0, 0, Num::MAX.value());
    for rid in pooling {
        let pid = instance.pickup_id_of_request(*rid);
        let did = instance.delivery_id_of_request(*rid);
        let earliest_start = instance.nodes[pid].ready.value();
        let latest_completion = instance.nodes[did].due.value();
        tfmin = tfmin.min(earliest_start);
        tfmax = tfmax.max(latest_completion);
        omin = omin.max(earliest_start);
        omax = omax.min(latest_completion);
    }
    ((tfmax - tfmin) - (omax - omin)) as usize
}

#[cfg(not(feature = "use-grb"))]
pub fn weighed_set_cover_matching(
    _instance: &PDPTWInstance,
    _poolings: &Poolings,
    _params: WeightedSetCoverMatchingParams,
    _rng: &mut Random,
) -> anyhow::Result<Matching> {
    panic!(
        "this executable was build without gurobi support -- recompile with 'use-grb' to enable."
    );
}

#[cfg(feature = "use-grb")]
pub fn weighed_set_cover_matching(
    instance: &PDPTWInstance,
    poolings: &Poolings,
    params: WeightedSetCoverMatchingParams,
    rng: &mut Random,
) -> anyhow::Result<Matching> {
    let gamma_max = MAX_REQUESTS_PER_POOLING;
    let poolings = &poolings.list;

    // generate environment first so we can disable all output
    let mut env = grb::Env::empty()?;
    env.set(
        grb::parameter::IntParam::OutputFlag,
        params.enable_solver_stdout_logging.into(),
    )?;
    let env = env.start()?;

    // define the weighted set cover model
    let mut model = grb::Model::with_env("weighted-set-cover", &env)?;
    model.set_param(grb::parameter::IntParam::MIPFocus, 1)?;
    model.set_param(grb::parameter::IntParam::Threads, 1)?;
    if let Some(seconds) = params.time_limit_in_seconds {
        model.set_param(grb::parameter::DoubleParam::TimeLimit, seconds)?;
    }
    if let Some(ub) = params.ub {
        model.set_param(grb::parameter::DoubleParam::Cutoff, ub.value() as f64)?;
    }

    let mut x = Vec::with_capacity(poolings.len());

    let mut xs_visiting_request = vec![vec![]; instance.num_requests];

    let mut sum_x = LinExpr::new();
    for i in 0..poolings.len() {
        let var_obj = params
            .weight_fn
            .calculate_weight(instance, &poolings[i], gamma_max);

        let var = match &params.model_type {
            ModelType::Integer => {
                grb::add_intvar!(model, name: format!("x_{}", i).as_str(), obj: var_obj, bounds: 0..1)?
            }
            ModelType::Continuous(_) => {
                grb::add_ctsvar!(model, name: format!("x_{}", i).as_str(), obj: var_obj, bounds: 0..1)?
            }
        };

        sum_x.add_term(1.0, var);
        x.push(var);

        for rid in &poolings[i].0 {
            xs_visiting_request[*rid].push(var);
        }
    }
    for i in 0..instance.num_requests {
        let var_obj = params
            .weight_fn
            .calculate_weight_of_singleton(instance, i, gamma_max);

        let var = match &params.model_type {
            ModelType::Integer => {
                grb::add_intvar!(model, name: format!("x_{}", i).as_str(), obj: var_obj, bounds: 0..1)?
            }
            ModelType::Continuous(_) => {
                grb::add_ctsvar!(model, name: format!("x_{}", i).as_str(), obj: var_obj, bounds: 0..1)?
            }
        };

        sum_x.add_term(1.0, var);
        x.push(var);
        xs_visiting_request[i].push(var);
    }

    if params.enforce_partitioning {
        for r in 0..instance.num_requests {
            model.add_constr(
                format!("a_{}", r).as_str(),
                grb::c!(xs_visiting_request[r].clone().grb_sum() == 1),
            )?;
        }
    } else {
        for r in 0..instance.num_requests {
            model.add_constr(
                format!("a_{}", r).as_str(),
                grb::c!(xs_visiting_request[r].clone().grb_sum() >= 1),
            )?;
        }
    }

    model.set_attr(grb::attr::ModelSense, grb::ModelSense::Minimize)?;
    model.optimize()?;

    let num_solutions_found = model.get_attr(grb::attr::SolCount)?;

    if num_solutions_found > 0 {
        let mut selected_poolings = vec![];
        let mut uncovered_requests = FixedBitSet::with_capacity(instance.num_requests);
        uncovered_requests.insert_range(..);

        match params.model_type {
            ModelType::Integer => {
                for (i, it) in poolings.iter().enumerate() {
                    let val = model.get_obj_attr(grb::attr::X, &x[i]).unwrap();
                    if val > 0.9 {
                        selected_poolings.push(it.clone());
                        it.0.iter().for_each(|j| uncovered_requests.set(*j, false));
                    }
                }
            }
            ModelType::Continuous(strategy) => {
                match strategy {
                    FractionalSelectionStrategy::GreedyFromSortedFractions => {
                        let mut fractionals = poolings
                            .iter()
                            .enumerate()
                            .map(|(i, it)| {
                                let val = model.get_obj_attr(grb::attr::X, &x[i]).unwrap();
                                ((val * 100_000f64) as usize, it)
                            })
                            .collect::<Vec<_>>();
                        fractionals.sort_by_key(|(val, it)| Reverse((*val, it.0.len())));
                        for (_, it) in fractionals {
                            if it.0.iter().any(|rid| !uncovered_requests.contains(*rid)) {
                                continue;
                            }
                            selected_poolings.push(it.clone());
                            it.0.iter().for_each(|j| uncovered_requests.set(*j, false));
                        }
                    }
                    FractionalSelectionStrategy::StandardRandomizedRounding => {
                        for i in 0..poolings.len() {
                            let fractional = model.get_obj_attr(grb::attr::X, &x[i]).unwrap();
                            if rng.gen_bool(fractional.clamp(0.0, 1.0)) {
                                poolings[i]
                                    .0
                                    .iter()
                                    .for_each(|it| uncovered_requests.set(*it, false));
                                selected_poolings.push(poolings[i]);
                            }
                        }
                    }
                    FractionalSelectionStrategy::ConditionalRandomizedRounding => {
                        // https://algnotes.info/on/background/randomized-rounding/
                        let objective = dbg!(model.get_attr(grb::attr::ObjVal)?);
                        let lambda = dbg!((2.0 * instance.num_requests as f64).ln());
                        let ps = x
                            .iter()
                            .map(|var| {
                                1f64.min(model.get_obj_attr(grb::attr::X, &var).unwrap() * lambda)
                            })
                            .collect::<Vec<_>>();

                        for i in 0..poolings.len() {
                            let i_cost: f64 = poolings[i].1.into();
                            let lhs = i_cost / (2.0 * lambda * objective);
                            let rhs =
                                // poolings
                                poolings[i]
                                    .0
                                    .iter()
                                    .filter(|it| uncovered_requests.contains(**it))
                                    .map(|element| {
                                        poolings
                                            .iter()
                                            .skip(i + 1)
                                            .enumerate()
                                            .filter_map(|(j, (arr, _))| {
                                                if arr.contains(element) {
                                                    Some(1f64 - ps[j])
                                                } else {
                                                    None
                                                }
                                            })
                                            .product::<f64>()
                                            * (1f64 - ps[poolings.len() + element])
                                    })
                                    .sum::<_>();

                            if lhs > rhs {
                                poolings[i]
                                    .0
                                    .iter()
                                    .for_each(|it| uncovered_requests.set(*it, false));
                                selected_poolings.push(poolings[i]);
                            }
                        }
                    }
                }

                let mut encountered_requests = FixedBitSet::with_capacity(instance.num_requests);
                let mut i = 0;
                while i < selected_poolings.len() {
                    let (pooling, _) = &mut selected_poolings[i];
                    let mut j = 0;
                    while j < pooling.len() {
                        if encountered_requests.contains(pooling[j]) {
                            pooling.swap_remove(j);
                        }
                        j += 1;
                    }
                    if pooling.len() > 1 {
                        pooling
                            .iter()
                            .for_each(|it| encountered_requests.insert(*it));
                    } else {
                        selected_poolings.swap_remove(i);
                    }
                    i += 1;
                }
            }
        }

        Ok(Matching {
            selected_poolings,
            uncovered_requests,
        })
    } else {
        Err(anyhow::Error::msg(format!(
            "No solutions found (Status: {:?})",
            model.status().unwrap()
        )))
    }
}
