use clap::ValueEnum;
use log::{error, info};
#[cfg(feature = "parallel")]
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};
use took::Timer;

use crate::cli;
use crate::cli::{LPModelType, LPWeightFn, MatchingMode, SolverArguments};
use crate::construction::bin::BestBinInsertion;
use crate::construction::insertion::ParallelInsertion;
use crate::construction::kdsp::{Blocks, ClusterKDSP, KDSPBlockMode};
use crate::io::sintef_solution::load_sintef_solution;
use crate::pooling::matching::{
    greedy_matching, weighed_set_cover_matching, FractionalSelectionStrategy, Matching, ModelType,
    PoolingWeightFunction, WeightedSetCoverMatchingParams,
};
#[cfg(feature = "parallel")]
use crate::pooling::par_map_poolings_to_itinerary_and_data;
#[cfg(not(feature = "parallel"))]
use crate::pooling::map_poolings_to_itinerary_and_data;
use crate::pooling::{load_poolings, process_requests, store_poolings, MAX_REQUESTS_PER_POOLING};
use crate::problem::pdptw::PDPTWInstance;
use crate::problem::Num;
use crate::refn::{ForwardREF, REFData, REFNode};
use crate::solution::blocknode::BlockNode;
use crate::solution::{create_solution_from_sintef, Solution};
use crate::utils::Random;

#[allow(non_camel_case_types)]
#[derive(Clone, ValueEnum, Debug)]
pub enum InitialSolutionGeneration {
    BEST_BIN_INSERTION,
    PARALLEL_INSERTION,
    KDSP,
    MATCHING_KDSP,
    PARALLEL_INSERTION_KDSP,
    WARMSTART,
}

pub fn construct<'a, 'b>(
    instance: &'a PDPTWInstance,
    rng: &mut Random,
    params: &SolverArguments,
) -> Solution<'b>
    where
        'a: 'b,
{
    use InitialSolutionGeneration::*;
    let timer = Timer::new();

    let (name, sol) = match params.init {
        BEST_BIN_INSERTION => (
            "BestBinInsertion".to_string(),
            best_bin_insertion(instance, rng),
        ),
        PARALLEL_INSERTION => (
            "ParallelInsertion".to_string(),
            parallel_insertion(instance, rng),
        ),
        KDSP => (
            "SingleRequestsKDSP".to_string(),
            kdsp_with_single_request_blocks(instance),
        ),
        MATCHING_KDSP => (
            "MatchingKDSP".to_string(),
            matching_kdsp(instance, params, rng),
        ),
        PARALLEL_INSERTION_KDSP => (
            "ParallelInsertionKDSP".to_string(),
            parallel_insertion_kdsp(instance, rng, KDSPBlockMode::Average),
        ),
        WARMSTART => {
            // read SINTEF solution
            match &params.warmstart_solution_file {
                None => {
                    error!("Initial solution generation set to warmstart, but no solution file provided. Using ParallelInsertion instead.");
                    (
                        "ParallelInsertion".to_string(),
                        parallel_insertion(instance, rng),
                    )
                }
                Some(filepath) => match load_sintef_solution(filepath) {
                    Err(e) => {
                        panic!(
                            "error while loading warmstart solution from {} - {}",
                            filepath, e
                        )
                    }
                    Ok(sintef) => {
                        let sol = create_solution_from_sintef(sintef, instance);
                        (format!("Warmstart ({})", filepath), sol)
                    }
                },
            }
        }
    };

    info!(
        "{} - Objective: {}, took: {}",
        name,
        sol.objective(),
        timer.took()
    );

    sol
}

pub fn best_bin_insertion<'a>(instance: &'a PDPTWInstance, rng: &mut Random) -> Solution<'a> {
    let ch = BestBinInsertion::new(instance);
    ch.construct(rng)
}

pub fn parallel_insertion<'a>(instance: &'a PDPTWInstance, rng: &mut Random) -> Solution<'a> {
    let ch = ParallelInsertion::new(instance);
    ch.construct(rng)
}

pub fn parallel_insertion_kdsp<'a>(
    instance: &'a PDPTWInstance,
    rng: &'_ mut Random,
    block_mode: KDSPBlockMode,
) -> Solution<'a> {
    let cluster = ClusterKDSP::new(instance);
    cluster.construct_with_blocks(cluster.extract_tightened_blocks_from_solution(
        &parallel_insertion(instance, rng).to_description(),
        block_mode,
    ))
}

pub fn kdsp_with_single_request_blocks(instance: &PDPTWInstance) -> Solution {
    let cluster = ClusterKDSP::new(instance);
    cluster.construct_with_single_request_blocks()
}

pub fn matching_kdsp<'a>(
    instance: &'a PDPTWInstance,
    params: &SolverArguments,
    rng: &'_ mut Random,
) -> Solution<'a> {
    let pooling_cache_hash = if let Some(filepath) = &params.pooling_cache_filepath {
        filepath.clone()
    } else {
        format!(
            "{}/{}.{}.{}.dat",
            params
                .pooling_cache_directory
                .clone()
                .unwrap_or("cache".into()),
            params
                .pooling_cache_prefix
                .clone()
                .unwrap_or("poolings".into()),
            instance.name,
            MAX_REQUESTS_PER_POOLING
        )
    };

    let poolings = load_poolings(pooling_cache_hash.clone())
        .or_else(|_| {
            log::warn!(
                "pooling options not preprocessed - doing so now - this may take some time."
            );
            let poolings = process_requests(instance);
            store_poolings(pooling_cache_hash.clone(), &poolings)
                .and_then(|_| Ok(poolings))
        })
        .ok()
        .unwrap();

    log::info!(
        "number of available pooling options: {}",
        poolings.list.len()
    );

    let weight_fn = match params.lp_weight_fn {
        LPWeightFn::Cardinality => PoolingWeightFunction::NormalizedNumberOfRequestsServed,
        LPWeightFn::WeightedCost => PoolingWeightFunction::TravelCost,
        LPWeightFn::WeightedTemporalOverlap => PoolingWeightFunction::TemporalOverlapRatio,
        LPWeightFn::WeightedTemporalDeviation => PoolingWeightFunction::TemporalDeviation,
        LPWeightFn::WeightedCostAndTemporalOverlap => {
            PoolingWeightFunction::WeightedTravelCostTemporalOverlapRatio(params.lp_weight_fn_rho)
        }
        LPWeightFn::WeightedCostAndTemporalDeviation => {
            PoolingWeightFunction::WeightedTravelCostTemporalDeviation(params.lp_weight_fn_rho)
        }
    };
    let Matching {
        selected_poolings,
        uncovered_requests,
    } = match params.matching_mode {
        MatchingMode::Greedy => {
            info!("running greedy matching");
            greedy_matching(instance, &poolings, weight_fn)
        }
        MatchingMode::WSC | MatchingMode::WSP => {
            let wsc_params = WeightedSetCoverMatchingParams {
                ub: None,
                time_limit_in_seconds: params.wsc_time_limit_in_seconds.map(|it| it as f64),
                weight_fn,
                model_type: match params.lp_model_type {
                    LPModelType::ILP => ModelType::Integer,
                    LPModelType::LP => {
                        ModelType::Continuous(match params.fractional_selection_strategy {
                            cli::FractionalSelectionStrategy::Greedy => {
                                FractionalSelectionStrategy::GreedyFromSortedFractions
                            }
                            cli::FractionalSelectionStrategy::StandardRandomizedRounding => {
                                FractionalSelectionStrategy::StandardRandomizedRounding
                            }
                            cli::FractionalSelectionStrategy::ConditionalRandomizedRounding => {
                                FractionalSelectionStrategy::ConditionalRandomizedRounding
                            }
                        })
                    }
                },
                enforce_partitioning: match params.matching_mode {
                    MatchingMode::WSP => true,
                    _ => false,
                },
                enable_solver_stdout_logging: false,
            };

            info!(
                "running WSC matching with params: WeightFn {:?}, Model {:?}",
                wsc_params.weight_fn, wsc_params.model_type,
            );
            weighed_set_cover_matching(instance, &poolings, wsc_params, rng)
                .ok()
                .unwrap()
        }
    };

    info!(
        "number of poolings selected: {}/{}",
        selected_poolings
            .iter()
            .filter(|(it, _)| it.len() > 1)
            .count(),
        selected_poolings.len()
    );

    // recalculate min combination to get itinerary
    log::info!("create blocks from options");
    #[cfg(feature = "parallel")]
        let mapped_pooling_iter =
        par_map_poolings_to_itinerary_and_data(instance, selected_poolings.par_iter());
    #[cfg(not(feature = "parallel"))]
        let mapped_pooling_iter =
        map_poolings_to_itinerary_and_data(instance, selected_poolings.iter());

    let mut blocks: Vec<Blocks> = mapped_pooling_iter
        .map(|(itinerary, data)| Blocks {
            earliest_start: data.earliest_start(),
            latest_start: data.latest_start(),
            block: BlockNode::new(itinerary[0], itinerary[itinerary.len() - 1], data),
            path: itinerary,
        })
        .collect();

    blocks.extend(uncovered_requests.ones().map(|rid| {
        let pid = instance.pickup_id_of_request(rid);
        let did = instance.delivery_of(pid).id;
        let mut data = REFData::with_node(&REFNode::from(&instance.nodes[pid]));
        data.extend_forward_assign(
            &REFNode::from(&instance.nodes[did]),
            instance.distance_and_time(pid, did),
        );
        Blocks {
            earliest_start: data.earliest_start(),
            latest_start: data.latest_start(),
            block: BlockNode::new(pid, did, data),
            path: vec![pid, did],
        }
    }));

    // solve kdsp
    let kdsp_mode = params.pooling_kdsp_mode.clone();
    blocks.iter_mut().for_each(|b| {
        let value = match kdsp_mode {
            KDSPBlockMode::Earliest => b.earliest_start,
            KDSPBlockMode::Latest => b.latest_start,
            KDSPBlockMode::Average => {
                (b.latest_start - b.earliest_start) / Num::TWO + b.earliest_start
            }
        };
        b.earliest_start = value;
        b.latest_start = value;
    });

    info!(
        "solving kdsp ({:?}) with {} blocks",
        kdsp_mode,
        blocks.len()
    );
    let clustering = ClusterKDSP::new(&instance);
    let solution = clustering.construct_with_blocks(blocks.clone());

    solution
}
