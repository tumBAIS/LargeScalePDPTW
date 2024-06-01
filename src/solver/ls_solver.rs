use log::info;
use took::Timer;
use crate::ages::{ILSSolutionSelectionStrategy, PenaltyCounterResetStrategy};

use crate::cli::SolverArguments;
use crate::lns::largescale::ages::LargeNeighborhoodAGES;
use crate::lns::largescale::{GroupMatchingMode, SplitSettings, UnassignedMatchingMode};
use crate::problem::pdptw::PDPTWInstance;
use crate::solution::Solution;
use crate::solver::construction::construct;
use crate::solver::SolverResult;
#[cfg(feature = "timed_solution_logger")]
use crate::utils::logging::timed::TimedSolutionLogger;
#[cfg(feature = "progress_tracking")]
use crate::utils::stats::search_progress_stats::SearchProgressTracking;
use crate::utils::{Countdown, Random, TimeLimit};

fn construct_initial_solution<'a, 'b>(
    instance: &'a PDPTWInstance,
    _rng: &mut Random,
) -> Solution<'b>
    where
        'a: 'b,
{
    let timer = Timer::new();
    info!("Construct initial solution with KSDP using single-requests-blocks");
    let sol = super::construction::kdsp_with_single_request_blocks(instance);
    info!(
        "KSDP using single-requests-blocks: solution found {}, took: {} (fleet: {}, unassigned: {})",
        sol.objective(),
        timer.took(),
        sol.number_of_vehicles_used(),
        sol.number_of_unassigned_requests(),
    );
    sol
}

pub fn ls_ages_lns(
    instance: &PDPTWInstance,
    args: &SolverArguments,
    rng: &mut Random,
    #[cfg(feature = "progress_tracking")] progress_tracking: &mut SearchProgressTracking,
    #[cfg(feature = "timed_solution_logger")] timed_solution_logger: TimedSolutionLogger,
) -> SolverResult {
    let timer = Timer::new();
    let countdown = Countdown::new(
        timer.clone(),
        TimeLimit::Seconds(args.time_limit_in_seconds),
    );

    let mut lns_kdsp_ages = LargeNeighborhoodAGES::with_instance(
        &instance,
        crate::lns::largescale::ages::Parameters {
            max_iterations: args.lns_iterations.clone(),
            repair_blink_rate: args.lns_recreate_blink_rate.clone(),
            repair_order_weights: args.lns_recreate_order_weights.clone(),
            repair_insertion_limit: args.lns_recreate_insertion_limit.clone(),
            destroy_adjacent_measure: args.lns_ruin_measure.clone(),
            destroy_adjacent_max_cardinality: args.lns_ruin_max_cardinality.clone(),
            destroy_adjacent_alpha: args.lns_ruin_alpha.clone(),
            destroy_adjacent_beta: args.lns_ruin_beta.clone(),
            nested_iterations: args.nested_iterations.clone(),
            avg_nodes_per_route: args.avg_nodes_per_split_range(),
            init_temp: args.lns_init_aspiration_temp.clone(),
            acceptance_criterion: args.lns_acceptance_criterion_strategy(),
            ls_method: args.lns_ls_mode.clone(),
            ls_probability: args.ls_probability.clone(),
            ls_threshold: None,
            ls_method_on_new_best_sol: args.lns_ls_mode_on_new_best_sol.clone(),
            bs_thickness: args.bs_thickness,
            num_perturbation_ils: args.ages_num_perturbations_ils(instance),
            max_ages_perturbation_phases: args.ages_max_perturbation_phases,
            count_successful_perturbations_only: args.ages_count_successful_perturbations_only,
            num_perturbation_after_ejection_range: args
                .ages_num_perturbations_after_ejection_range(instance),
            perturbation_mode_after_ejection: args.ages_perturbation_mode_ejection_search(),
            perturbation_mode_ils: args.ages_perturbation_mode_ils(),
            recombine_mode: args.get_recombine_mode(),
            #[cfg(feature = "relatedness-measures")]
            split_settings: SplitSettings {
                group_matching_mode: GroupMatchingMode::ByRequestRelatedness,
                unassigned_matching_mode: UnassignedMatchingMode::ByLinkAndRouteHistory {
                    link_weight: 1.0,
                    route_weight: 1.0,
                },
            },
            #[cfg(not(feature = "relatedness-measures"))]
            split_settings: SplitSettings {
                group_matching_mode: GroupMatchingMode::Random,
                unassigned_matching_mode: UnassignedMatchingMode::Random,
            },
            shuffle_stack_after_permutation: args.ages_shuffle_stack_after_permutation.clone(),
            penalty_counter_reset: {
                if args.ages_penalty_counter_decay > 1.0 {
                    PenaltyCounterResetStrategy::ResetOnNewMin
                } else if args.ages_penalty_counter_decay < 0.0 {
                    PenaltyCounterResetStrategy::Ignore
                } else {
                    PenaltyCounterResetStrategy::DecayOnNewMin(args.ages_penalty_counter_decay)
                }
            },
            ils_solution_selection: ILSSolutionSelectionStrategy::NonImprovementBased,
            #[cfg(feature = "timed_solution_logger")]
            timed_solution_logger,
        },
    );

    let desc = construct(instance, rng, &args).to_description();

    let (min_sol, _) = lns_kdsp_ages.run_with_solution_desc(
        desc,
        rng,
        None,
        &countdown,
        #[cfg(feature = "progress_tracking")]
            progress_tracking,
    );

    SolverResult {
        time: timer.took(),
        solution: min_sol,
    }
}
