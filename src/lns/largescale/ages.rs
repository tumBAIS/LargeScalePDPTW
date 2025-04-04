use std::ops::RangeInclusive;

use log::info;
use rand::{Rng, RngCore};
use rand::seq::{IteratorRandom, SliceRandom};
#[cfg(feature = "parallel")]
use rayon::iter::{
    IndexedParallelIterator, IntoParallelIterator, ParallelIterator,
};
#[cfg(feature = "progress_tracking_ls_lns")]
use took::Timer;
use crate::ages::{eject_and_insert, ILSSolutionSelectionStrategy, PenaltyCounterResetStrategy, perform_perturbation, PerturbationMode};

#[cfg(feature = "search_assertions")]
use crate::assert_valid_solution;
use crate::cli::LS_Mode;
use crate::lns::{destroy, repair};
use crate::lns::absence_counter::AbsenceCounter;
use crate::lns::acceptance_criterion::{
    AcceptanceCriterion, AcceptanceCriterionStrategy, ExponentialMetropolisCriteria,
    LinearRecordToRecordCriteria, OnlyImprovementsCriteria,
};
use crate::lns::destroy::adjacent_string_removal::AdjacencyMeasure;
use crate::lns::destroy::DestroyOperators;
use crate::lns::largescale::{
    PartialInstanceStruct, recombine_solution, RecombineMode, split_solution, SplitSettings,
};
use crate::lns::repair::RepairOperators;
#[cfg(feature = "progress_tracking_ls_lns")]
use crate::problem::Num;
use crate::problem::pdptw::PDPTWInstance;
use crate::solution::{Solution, SolutionDescription};
use crate::solution::balassimonetti::ls::BalasSimonettiLS;
use crate::solution::permutation::KEjectionInsertion;
use crate::utils::{Countdown, DefaultSearchTracker, Random, SearchProgressIterationTracker};
#[cfg(feature = "timed_solution_logger")]
use crate::utils::logging::timed::TimedSolutionLogger;
#[cfg(feature = "relatedness-measures")]
use crate::utils::relatedness::RelatednessMeasures;
#[cfg(feature = "progress_tracking_ls_ils")]
use crate::utils::stats::search_progress_stats::{
    ComponentTracking, ILSTrackingInfo, LNSTrackingInfo, SearchProgressTracking,
    SolutionTrackingInfo, SolutionUpdateEvent, TriggeredTerminationCriterion,
};
#[cfg(feature = "progress_tracking_ls_decomposition")]
use crate::utils::stats::search_progress_stats::{DecompositionTrackingInfo, MergeType, SplitType};

pub struct DestroyParameters {
    pub adjacent_measure: AdjacencyMeasure,
    pub adjacent_max_cardinality: usize,
    pub adjacent_alpha: f64,
    pub adjacent_beta: f64,
}

impl DestroyParameters {
    pub fn from(params: &Parameters) -> Self {
        Self {
            adjacent_measure: params.destroy_adjacent_measure.clone(),
            adjacent_max_cardinality: params.destroy_adjacent_max_cardinality,
            adjacent_alpha: params.destroy_adjacent_alpha,
            adjacent_beta: params.destroy_adjacent_beta,
        }
    }
}

pub struct RepairParameters {
    pub order_weights: Vec<usize>,
    pub blink_rate: f64,
    pub insertion_limit: usize,
}

impl RepairParameters {
    pub fn from(params: &Parameters) -> Self {
        Self {
            order_weights: params.repair_order_weights.clone(),
            blink_rate: params.repair_blink_rate,
            insertion_limit: params.repair_insertion_limit,
        }
    }
}

pub struct Parameters {
    pub max_ils_iterations: Option<usize>,
    pub max_lns_iterations: usize,
    pub repair_order_weights: Vec<usize>,
    pub repair_blink_rate: f64,
    pub repair_insertion_limit: usize,
    pub destroy_adjacent_measure: AdjacencyMeasure,
    pub destroy_adjacent_max_cardinality: usize,
    pub destroy_adjacent_alpha: f64,
    pub destroy_adjacent_beta: f64,
    pub nested_iterations: usize,
    pub avg_nodes_per_route: RangeInclusive<usize>,
    pub init_temp: f64,
    pub acceptance_criterion: AcceptanceCriterionStrategy,
    pub ls_method: LS_Mode,
    pub ls_probability: f64,
    pub ls_threshold: Option<f64>,
    pub ls_method_on_new_best_sol: LS_Mode,
    pub bs_thickness: usize,
    pub num_perturbation_after_ejection_range: RangeInclusive<usize>,
    pub max_ages_perturbation_phases: usize,
    pub perturbation_mode_after_ejection: PerturbationMode,
    pub num_perturbation_ils: usize,
    pub perturbation_mode_ils: PerturbationMode,
    pub recombine_mode: RecombineMode,
    pub split_settings: SplitSettings,
    pub shuffle_stack_after_permutation: bool,
    pub count_successful_perturbations_only: bool,
    pub penalty_counter_reset: PenaltyCounterResetStrategy,
    pub ils_solution_selection: ILSSolutionSelectionStrategy,
    #[cfg(feature = "timed_solution_logger")]
    pub timed_solution_logger: TimedSolutionLogger,
}

impl Parameters {}

pub struct LargeNeighborhoodAGES<'a> {
    instance: &'a PDPTWInstance,
    params: Parameters,
    #[cfg(feature = "relatedness-measures")]
    measures: RelatednessMeasures<'a>,
    destroy_operator_gen: fn(&PDPTWInstance, DestroyParameters) -> DestroyOperators,
    repair_operator_gen: fn(&PDPTWInstance, RepairParameters) -> RepairOperators,
}

impl<'a> LargeNeighborhoodAGES<'a> {
    pub fn with_instance(instance: &'a PDPTWInstance, params: Parameters) -> Self {
        Self {
            #[cfg(feature = "relatedness-measures")]
            measures: RelatednessMeasures::with_instance(instance),
            instance,
            params,
            destroy_operator_gen: |instance, params| {
                DestroyOperators::AdjacentStringRemoval(destroy::AdjacentStringRemoval::new(
                    instance,
                    params.adjacent_measure.clone(),
                    params.adjacent_max_cardinality.clone(),
                    params.adjacent_alpha.clone(),
                    params.adjacent_beta.clone(),
                ))
            },
            repair_operator_gen: |instance, params| {
                RepairOperators::GreedyInsertionWithBlinks(
                    #[cfg(not(feature = "classic-pdptw"))]
                        repair::GreedyInsertionWithBlinks::with_weights(
                        instance,
                        params.blink_rate.clone(),
                        true,
                        params.order_weights.clone(),
                        params.insertion_limit.clone(),
                    ),
                    #[cfg(feature = "classic-pdptw")] repair::GreedyInsertionWithBlinks::new(
                        instance,
                        params.blink_rate.clone(),
                        true,
                    ),
                )
            },
        }
    }

    fn reduce_number_of_routes(&self, sol: &mut Solution, abs: &AbsenceCounter) {
        let min_absence_r = sol
            .iter_route_ids()
            .min_by_key(|&r| {
                abs.get_sum_for_requests(
                    &sol.iter_route(r)
                        .filter_map(|it| {
                            if self.instance.is_pickup(it) {
                                Some(self.instance.request_id(it))
                            } else {
                                None
                            }
                        })
                        .collect(),
                )
            })
            .unwrap();

        sol.unassign_complete_route(min_absence_r);
        sol.clamp_max_number_of_vehicles_to_current_fleet_size();
    }

    fn ages_without_splits(
        &mut self,
        sol: &mut Solution<'a>,
        rng: &mut Random,
        abs: Option<AbsenceCounter<'a>>,
        countdown: &Countdown,
    ) -> (SolutionDescription, SolutionDescription, AbsenceCounter<'a>) {
        let mut abs = abs.unwrap_or(AbsenceCounter::new(self.instance));

        let mut min_vehicle_solution = sol.to_description();
        let mut min_unassigned_solution = sol.to_description();
        let mut cnt = 0;
        let mut pb = DefaultSearchTracker::new(self.params.max_ages_perturbation_phases as u64);
        pb.update(Some(&sol.to_description()), None);

        while cnt < self.params.max_ages_perturbation_phases && !countdown.is_finished() {
            match &self.params.penalty_counter_reset {
                PenaltyCounterResetStrategy::ResetOnNewMin => abs.fill(1),
                PenaltyCounterResetStrategy::DecayOnNewMin(factor) => abs.apply_decay(*factor),
                PenaltyCounterResetStrategy::Ignore => {}
            }
            if sol.unassigned_requests.count() == 0 {
                // randomly select a route an un-assign all PD-pairs from it
                if let Some(rid) = sol.iter_route_ids().choose(rng) {
                    sol.unassign_complete_route(rid);
                    sol.clamp_max_number_of_vehicles_to_current_fleet_size();
                } else {
                    break;
                }
            }
            cnt = 0;
            pb = DefaultSearchTracker::new(self.params.max_ages_perturbation_phases as u64);

            let mut stack: Vec<_> = sol
                .unassigned_requests
                .iter_request_ids()
                .map(|it| self.instance.pickup_id_of_request(it))
                .collect();
            stack.shuffle(rng);
            let mut min_unassigned = stack.len();

            // for a fixed number of iterations
            while !stack.is_empty()
                && cnt < self.params.max_ages_perturbation_phases
                && !countdown.is_finished()
            {
                pb.update(Some(&min_vehicle_solution), Some(&sol.to_description()));

                let u = stack.pop().unwrap();
                if let Some(ins) = sol.find_random_insert_for_request(u, rng) {
                    // If request u was successfully reinserted in s', then the algorithm moves to the next
                    //  request in the stack.
                    sol.insert(ins);
                } else {
                    abs.increment_single_request(self.instance.request_id(u));

                    eject_and_insert(sol, u, &mut stack, rng, &mut abs);

                    let num_perturbations_after_ejection =
                        rng.gen_range(self.params.num_perturbation_after_ejection_range.clone());
                    let performed_perturbations = perform_perturbation(
                        sol,
                        rng,
                        num_perturbations_after_ejection,
                        self.params.perturbation_mode_after_ejection,
                    );

                    let counted_perturbations = if self.params.count_successful_perturbations_only {
                        performed_perturbations.max(1)
                    } else {
                        num_perturbations_after_ejection
                    };

                    cnt += counted_perturbations;
                    pb.add(counted_perturbations as u64);

                    if self.params.shuffle_stack_after_permutation {
                        stack.shuffle(rng);
                    }
                }

                if stack.len() < min_unassigned {
                    pb.update(Some(&min_vehicle_solution), Some(&sol.to_description()));
                    cnt = 0;
                    min_unassigned = stack.len();
                    min_unassigned_solution = sol.to_description();
                    pb = DefaultSearchTracker::new(self.params.max_ages_perturbation_phases as u64);
                } else if stack.len() > 50.max(min_unassigned * 2) {
                    // heuristic for early exit
                    cnt = self.params.max_ages_perturbation_phases;
                    break;
                }
            }
            if stack.is_empty() {
                assert_eq!(sol.unassigned_requests.count(), 0);
                min_vehicle_solution = sol.to_description();

                #[cfg(feature = "timed_solution_logger")]
                self.params.timed_solution_logger.check(
                    &countdown.time_elapsed(),
                    &min_vehicle_solution,
                    &self.instance,
                );
            } else {
                sol.set_with(&min_vehicle_solution);
            }
        }
        (min_vehicle_solution, min_unassigned_solution, abs)
    }

    fn ages_only_insert(
        &mut self,
        sol: &mut Solution<'a>,
        rng: &mut Random,
        abs: Option<AbsenceCounter<'a>>,
        countdown: &Countdown,
    ) -> (SolutionDescription, AbsenceCounter<'a>) {
        let mut abs = abs.unwrap_or(AbsenceCounter::new(self.instance));

        let mut min_unassigned_solution = sol.to_description();
        let mut cnt = 0;

        let mut pb = DefaultSearchTracker::new(self.params.max_ages_perturbation_phases as u64);
        pb.update(Some(&sol.to_description()), None);

        while cnt < self.params.max_ages_perturbation_phases && !countdown.is_finished() {
            match &self.params.penalty_counter_reset {
                PenaltyCounterResetStrategy::ResetOnNewMin => abs.fill(1),
                PenaltyCounterResetStrategy::DecayOnNewMin(factor) => abs.apply_decay(*factor),
                PenaltyCounterResetStrategy::Ignore => {}
            }
            if sol.unassigned_requests.count() == 0 {
                break;
            }
            cnt = 0;
            pb = DefaultSearchTracker::new(self.params.max_ages_perturbation_phases as u64);

            let mut stack: Vec<_> = sol
                .unassigned_requests
                .iter_request_ids()
                .map(|it| self.instance.pickup_id_of_request(it))
                .collect();
            stack.shuffle(rng);
            let mut min_unassigned = stack.len();

            // for a fixed number of iterations
            while !stack.is_empty()
                && cnt < self.params.max_ages_perturbation_phases
                && !countdown.is_finished()
            {
                pb.update(Some(&min_unassigned_solution), Some(&sol.to_description()));

                let u = stack.pop().unwrap();
                if let Some(ins) = sol.find_random_insert_for_request(u, rng) {
                    // If request u was successfully reinserted in s', then the algorithm moves to the next
                    //  request in the stack.
                    sol.insert(ins);
                } else {
                    abs.increment_single_request(self.instance.request_id(u));

                    eject_and_insert(sol, u, &mut stack, rng, &mut abs);

                    let num_perturbations_after_ejection =
                        rng.gen_range(self.params.num_perturbation_after_ejection_range.clone());
                    let performed_perturbations = perform_perturbation(
                        sol,
                        rng,
                        num_perturbations_after_ejection,
                        self.params.perturbation_mode_after_ejection,
                    );

                    let counted_perturbations = if self.params.count_successful_perturbations_only {
                        performed_perturbations.max(1)
                    } else {
                        num_perturbations_after_ejection
                    };

                    cnt += counted_perturbations;
                    pb.add(counted_perturbations as u64);

                    if self.params.shuffle_stack_after_permutation {
                        stack.shuffle(rng);
                    }
                }

                if stack.len() < min_unassigned {
                    cnt = 0;
                    min_unassigned = stack.len();
                    min_unassigned_solution = sol.to_description();
                    pb = DefaultSearchTracker::new(self.params.max_ages_perturbation_phases as u64);
                }
            }
            if stack.is_empty() {
                assert_eq!(min_unassigned_solution.unassigned_requests, 0);

                #[cfg(feature = "timed_solution_logger")]
                self.params.timed_solution_logger.check(
                    &countdown.time_elapsed(),
                    &min_unassigned_solution,
                    &self.instance,
                );
            } else {
                sol.set_with(&min_unassigned_solution);
            }
        }
        (min_unassigned_solution, abs)
    }

    pub fn run_with_solution_desc(
        &mut self,
        initial_solution: SolutionDescription,
        rng: &mut Random,
        abs: Option<AbsenceCounter<'a>>,
        countdown: &Countdown,
        #[cfg(feature = "progress_tracking")] tracking: &mut SearchProgressTracking,
    ) -> (SolutionDescription, AbsenceCounter<'a>) {
        #[cfg(feature = "fleet-minimization")]
            let mut abs = abs.unwrap_or(AbsenceCounter::new(self.instance));
        #[cfg(not(feature = "fleet-minimization"))]
            let abs = abs.unwrap_or(AbsenceCounter::new(self.instance));

        let mut sol = Solution::new(self.instance);
        sol.set_with(&initial_solution);
        let mut best_sol = initial_solution;

        #[cfg(feature = "progress_tracking_ls_lns")]
            let timer = Timer::new();

        #[cfg(feature = "progress_tracking_ls_ils")]
        {
            tracking.track_component(ComponentTracking::ILS(ILSTrackingInfo::Start {
                init: SolutionTrackingInfo::with_solution(&sol),
            }));
        }

        let mut current_iteration = 0;
        let mut non_improving_iterations = 0;
        loop {
            #[cfg(feature = "fleet-minimization")]
            {
                let (mut desc, min_desc, _abs) = self.ages_without_splits(
                    &mut sol,
                    rng,
                    Some(abs),
                    countdown,
                );

                if desc.number_of_unassigned_requests() != 0 {
                    desc = min_desc;
                }

                abs = _abs;

                #[cfg(feature = "timed_solution_logger")]
                self.params.timed_solution_logger.check(
                    &countdown.time_elapsed(),
                    &desc,
                    &self.instance,
                );

                info!(
                    "after GES: {}/{}/{}",
                    desc.number_of_unassigned_requests(),
                    desc.number_of_vehicles_used(),
                    desc.objective
                );

                sol.set_with(&desc);
            }
            #[cfg(feature = "search_assertions")]
            assert_valid_solution(self.instance, &sol);

            if !countdown.is_finished() {
                #[cfg(not(feature = "disable-decomposition"))]
                    let desc = match self.params.acceptance_criterion {
                    AcceptanceCriterionStrategy::ExponentialMetropolis {
                        initial_temperature,
                        final_temperature,
                    } => self.ls_lns(
                        &mut sol,
                        ExponentialMetropolisCriteria::new(
                            initial_temperature,
                            final_temperature,
                            self.params.max_lns_iterations,
                        ),
                        rng,
                        countdown,
                        #[cfg(feature = "progress_tracking")]
                            tracking,
                    ),
                    AcceptanceCriterionStrategy::LinearRecordToRecord {
                        initial_temperature,
                        final_temperature,
                    } => self.ls_lns(
                        &mut sol,
                        LinearRecordToRecordCriteria::new(
                            initial_temperature,
                            final_temperature,
                            self.params.max_lns_iterations,
                        ),
                        rng,
                        countdown,
                        #[cfg(feature = "progress_tracking")]
                            tracking,
                    ),
                    AcceptanceCriterionStrategy::None => self.ls_lns(
                        &mut sol,
                        OnlyImprovementsCriteria,
                        rng,
                        countdown,
                        #[cfg(feature = "progress_tracking")]
                            tracking,
                    ),
                };

                #[cfg(feature = "disable-decomposition")]
                    let desc = match self.params.acceptance_criterion {
                    AcceptanceCriterionStrategy::ExponentialMetropolis {
                        initial_temperature,
                        final_temperature,
                    } => self.ls_lns_without_splits(
                        &mut sol,
                        ExponentialMetropolisCriteria::new(
                            initial_temperature,
                            final_temperature,
                            self.params.max_lns_iterations,
                        ),
                        rng,
                        countdown,
                        #[cfg(feature = "progress_tracking")]
                            tracking,
                    ),
                    AcceptanceCriterionStrategy::LinearRecordToRecord {
                        initial_temperature,
                        final_temperature,
                    } => self.ls_lns_without_splits(
                        &mut sol,
                        LinearRecordToRecordCriteria::new(
                            initial_temperature,
                            final_temperature,
                            self.params.max_lns_iterations,
                        ),
                        rng,
                        countdown,
                        #[cfg(feature = "progress_tracking")]
                            tracking,
                    ),
                    AcceptanceCriterionStrategy::None => self.ls_lns_without_splits(
                        &mut sol,
                        crate::lns::acceptance_criterion::OnlyImprovementsCriteria,
                        rng,
                        countdown,
                        #[cfg(feature = "progress_tracking")]
                            tracking,
                    ),
                };

                #[cfg(feature = "timed_solution_logger")]
                self.params.timed_solution_logger.check(
                    &countdown.time_elapsed(),
                    &desc,
                    &self.instance,
                );

                sol.set_with(&desc);
                info!(
                    "after LS-LNS: {}/{}/{}",
                    sol.number_of_unassigned_requests(),
                    sol.number_of_vehicles_used(),
                    sol.objective()
                );
            }

            current_iteration += 1;
            #[cfg(feature = "fleet-minimization")]
                let is_new_best = sol.number_of_vehicles_used() < best_sol.number_of_vehicles_used()
                || (sol.number_of_vehicles_used() == best_sol.number_of_vehicles_used()
                && sol.objective() < best_sol.objective);
            #[cfg(not(feature = "fleet-minimization"))]
                let is_new_best = sol.number_of_unassigned_requests()
                < best_sol.number_of_unassigned_requests()
                || (sol.number_of_unassigned_requests()
                == best_sol.number_of_unassigned_requests()
                && sol.objective() < best_sol.objective());

            if is_new_best {
                best_sol = sol.to_description();
                #[cfg(feature = "timed_solution_logger")]
                self.params.timed_solution_logger.check(
                    &countdown.time_elapsed(),
                    &best_sol,
                    &self.instance,
                );
                non_improving_iterations = 0;
            } else {
                non_improving_iterations += 1;
                match self.params.ils_solution_selection {
                    ILSSolutionSelectionStrategy::AlwaysCurrent => {
                        // stay with current solution from the metaheuristic
                    }
                    ILSSolutionSelectionStrategy::AlwaysBest => sol.set_with(&best_sol),
                    ILSSolutionSelectionStrategy::NonImprovementBased => {
                        if rng.gen_bool(non_improving_iterations as f64 / current_iteration as f64)
                        {
                            sol.set_with(&best_sol);
                        } else {
                            // stay with current solution from the metaheuristic
                        }
                    }
                }
            }

            let iteration_limit_reached = if let Some(max_iterations) = self.params.max_ils_iterations {
                current_iteration >= max_iterations
            } else { false };

            if iteration_limit_reached || countdown.is_finished() {
                #[cfg(feature = "progress_tracking_ls_ils")]
                {
                    tracking.track_component(ComponentTracking::ILS(ILSTrackingInfo::Finished {
                        total_iterations: current_iteration,
                        best: SolutionTrackingInfo::with_description(&best_sol),
                        trigger: if countdown.is_finished() {
                            TriggeredTerminationCriterion::TimeLimit
                        } else {
                            TriggeredTerminationCriterion::IterationLimit
                        },
                        took: timer.took().into_std(),
                    }));
                }

                break (best_sol, abs);
            } else {
                #[cfg(feature = "progress_tracking_ls_ils")]
                    let before_permutation = SolutionTrackingInfo::with_solution(&sol);
                let _num_permutations_performed = perform_perturbation(
                    &mut sol,
                    rng,
                    self.params.num_perturbation_ils,
                    self.params.perturbation_mode_ils,
                );
                log::info!(
                    "after ILS permutation: {}/{}/{}",
                    sol.number_of_unassigned_requests(),
                    sol.number_of_vehicles_used(),
                    sol.objective()
                );
                #[cfg(feature = "progress_tracking_ls_ils")]
                tracking.track_component(ComponentTracking::ILS(ILSTrackingInfo::Iteration {
                    iteration: current_iteration,
                    before_permutation,
                    after_permutation: SolutionTrackingInfo::with_solution(&sol),
                    num_permutations_performed: _num_permutations_performed,
                    best: SolutionTrackingInfo::with_description(&best_sol),
                }));
                #[cfg(feature = "timed_solution_logger")]
                self.params.timed_solution_logger.check(
                    &countdown.time_elapsed(),
                    &sol.to_description(),
                    &self.instance,
                );
            }
        }
    }

    fn ls_lns(
        &mut self,
        solution: &mut Solution,
        mut acceptance_criterion: impl AcceptanceCriterion + std::marker::Sync,
        rng: &mut Random,
        countdown: &Countdown,
        #[cfg(feature = "progress_tracking")] tracking: &mut SearchProgressTracking,
    ) -> SolutionDescription {
        #[cfg(feature = "fleet-minimization")]
            let vehicle_limit = solution.number_of_vehicles_used();
        #[cfg(not(feature = "fleet-minimization"))]
            let vehicle_limit = self.instance.num_vehicles;

        let mut best_sol = solution.to_description();
        let mut current_sol = solution.to_description();

        #[cfg(feature = "progress_tracking_ls_lns")]
            let timer = Timer::new();

        #[cfg(feature = "progress_tracking_ls_lns")]
        tracking.track_component(ComponentTracking::LNS(LNSTrackingInfo::Start {
            init: SolutionTrackingInfo::with_solution(&solution),
            temperature: acceptance_criterion.get_current_temperature(),
        }));

        let mut pb_outer = DefaultSearchTracker::new(self.params.max_lns_iterations as u64);
        pb_outer.update(Some(&best_sol), None);

        let mut _lap = 0;
        let mut iteration = 0;

        #[cfg(feature = "progress_tracking_ls_lns")]
        let mut last_improving_iteration = 0;

        while iteration < self.params.max_lns_iterations && !countdown.is_finished() {
            // split up the solution into several smaller ones
            let avg_nodes_per_split = self.params.avg_nodes_per_route.clone().choose(rng).unwrap();
            let splits = split_solution(
                self.instance,
                &solution,
                avg_nodes_per_split,
                rng,
                &self.params.split_settings,
                #[cfg(feature = "relatedness-measures")]
                    &self.measures,
            );

            #[cfg(feature = "progress_tracking_ls_decomposition")]
            tracking.track_component(ComponentTracking::Decomposition(
                DecompositionTrackingInfo::Split {
                    lap_id: _lap,
                    procedure: SplitType::RoutesOnly,
                    avg_nodes_per_split,
                    num_partitions: splits.len(),
                    init: SolutionTrackingInfo::with_solution(&solution),
                },
            ));

            let until = self
                .params
                .nested_iterations
                .min(self.params.max_lns_iterations - iteration);

            let rngs = (0..splits.len())
                .into_iter()
                .map(|_| Random::new(((rng.next_u64() as u128) << 64) + rng.next_u64() as u128))
                .collect::<Vec<Random>>();

            let part_solutions: Vec<Option<(SolutionDescription, SolutionDescription)>> = {
                #[cfg(feature = "parallel")]
                    let iter = rngs.into_par_iter();
                #[cfg(not(feature = "parallel"))]
                    let iter = rngs.into_iter();

                iter.enumerate()
                    .map(|(s, mut rng)| {
                        let (partial_instance_struct, routes) = &splits[s];
                        if partial_instance_struct.partial.num_requests == 0 {
                            None
                        } else {
                            let res = self.perform(
                                s,
                                &partial_instance_struct.partial,
                                routes,
                                &mut rng,
                                until,
                                acceptance_criterion.clone(),
                                countdown,
                            );
                            Some(res)
                        }
                    })
                    .collect()
            };

            // recombine to single solution
            let parts: Vec<
                Option<(
                    PartialInstanceStruct,
                    SolutionDescription,
                    SolutionDescription,
                )>,
            > = splits
                .into_iter()
                .zip(part_solutions.into_iter())
                .map(|(part, maybe_solutions)| {
                    maybe_solutions.map(|(best, current)| (part.0, best, current))
                })
                .collect();

            #[cfg(feature = "progress_tracking_ls_decomposition")]
                let timer = Timer::new();

            let (mut new_best_sol, new_current_sol, _) = recombine_solution(
                self.params.recombine_mode.into(),
                self.instance,
                parts,
                Some(vehicle_limit),
                0,
            );

            #[cfg(feature = "progress_tracking_ls_decomposition")]
            tracking.track_component(ComponentTracking::Decomposition(
                DecompositionTrackingInfo::Merge {
                    duration: timer.took().into_std(),
                    lap_id: _lap,
                    procedure: MergeType::KDSP,
                    after_merge: SolutionTrackingInfo::with_description(&new_best_sol),
                },
            ));

            iteration += until;
            _lap += 1;
            acceptance_criterion.update(until);

            if new_current_sol.objective() < new_best_sol.objective()
                && new_current_sol.number_of_unassigned_requests()
                <= new_best_sol.number_of_unassigned_requests()
            {
                new_best_sol = new_current_sol.clone();
            } else {
                #[cfg(feature = "relatedness-measures")]
                self.measures.process_solution_description(&new_current_sol);
            }
            #[cfg(feature = "relatedness-measures")]
            self.measures.process_solution_description(&new_best_sol);

            solution.set_with(&new_best_sol);

            if new_best_sol.number_of_unassigned_requests()
                < best_sol.number_of_unassigned_requests()
                || (new_best_sol.number_of_unassigned_requests()
                == best_sol.number_of_unassigned_requests()
                && new_best_sol.objective() < best_sol.objective())
            {
                best_sol = new_best_sol.clone();

                #[cfg(feature = "timed_solution_logger")]
                self.params.timed_solution_logger.check(
                    &countdown.time_elapsed(),
                    &best_sol,
                    &self.instance,
                );

                current_sol = new_best_sol;
                pb_outer.update(Some(&best_sol), Some(&current_sol));
                solution.set_with(&best_sol);

                #[cfg(feature = "progress_tracking_ls_lns")]
                {
                    last_improving_iteration = iteration;
                    tracking.track_component(ComponentTracking::LNS(LNSTrackingInfo::Iteration {
                        iteration,
                        update: SolutionUpdateEvent::NewBestSolution,
                        current: SolutionTrackingInfo::with_description(&current_sol),
                        best: SolutionTrackingInfo::with_description(&best_sol),
                        temperature: acceptance_criterion.get_current_temperature(),
                    }));
                }
            } else if new_current_sol.number_of_unassigned_requests()
                <= current_sol.number_of_unassigned_requests()
                && acceptance_criterion.check_aspiration_criterion(
                new_current_sol.objective,
                best_sol.objective,
                current_sol.objective,
                rng,
            ) {
                #[cfg(feature = "progress_tracking_ls_lns")]
                    let is_improved_solution =
                    new_current_sol.objective() - best_sol.objective() <= Num::NEGATIVE_EPSILON;

                current_sol = new_current_sol;
                pb_outer.update(Some(&best_sol), Some(&current_sol));
                solution.set_with(&current_sol);

                #[cfg(feature = "progress_tracking_ls_lns")]
                tracking.track_component(ComponentTracking::LNS(LNSTrackingInfo::Iteration {
                    iteration,
                    update: if is_improved_solution {
                        SolutionUpdateEvent::ImprovedSolution
                    } else {
                        SolutionUpdateEvent::AspirationCriterion
                    },
                    current: SolutionTrackingInfo::with_description(&current_sol),
                    best: SolutionTrackingInfo::with_description(&best_sol),
                    temperature: acceptance_criterion.get_current_temperature(),
                }));
            } else {
                solution.set_with(&current_sol);
            }

            pb_outer.add(until as u64);

            #[cfg(feature = "search_assertions")]
            assert_valid_solution(self.instance, &solution);
        }

        #[cfg(feature = "progress_tracking_ls_lns")]
        {
            tracking.track_component(ComponentTracking::LNS(LNSTrackingInfo::Finished {
                total_iterations: iteration,
                non_improving_count: iteration - last_improving_iteration,
                best: SolutionTrackingInfo::with_description(&best_sol),
                trigger: if countdown.is_finished() {
                    TriggeredTerminationCriterion::TimeLimit
                } else {
                    TriggeredTerminationCriterion::IterationLimit
                },
                took: timer.took().into_std(),
            }));
        }

        best_sol
    }

    fn ls_lns_without_splits(
        &mut self,
        solution: &mut Solution,
        acceptance_criterion: impl AcceptanceCriterion,
        rng: &mut Random,
        countdown: &Countdown,
        #[cfg(feature = "progress_tracking")] tracking: &mut SearchProgressTracking,
    ) -> SolutionDescription {
        let mut best_sol = solution.to_description();
        let mut current_sol = solution.to_description();

        let temp_start = self.params.init_temp;
        let mut current_temp = temp_start;
        let temp_decrement_step = temp_start / (self.params.max_lns_iterations as f64);

        #[cfg(feature = "progress_tracking_ls_lns")]
            let timer = Timer::new();

        #[cfg(feature = "progress_tracking_ls_lns")]
        tracking.track_component(ComponentTracking::LNS(LNSTrackingInfo::Start {
            init: SolutionTrackingInfo::with_solution(&solution),
            temperature: current_temp,
        }));

        let mut pb_outer = DefaultSearchTracker::new(self.params.max_lns_iterations as u64);
        pb_outer.update(Some(&best_sol), None);

        let mut _lap = 0;
        let mut iteration = 0;

        #[cfg(feature = "progress_tracking_ls_lns")]
        let mut last_improving_iteration = 0;

        while iteration < self.params.max_lns_iterations && !countdown.is_finished() {
            let until = self
                .params
                .nested_iterations
                .min(self.params.max_lns_iterations - iteration);

            let (mut new_best_sol, new_current_sol) = self.perform(
                0,
                self.instance,
                &current_sol.to_routes_vec(self.instance),
                rng,
                until,
                acceptance_criterion.clone(),
                countdown,
            );

            #[cfg(feature = "progress_tracking_ls_decomposition")]
            tracking.track_component(ComponentTracking::Decomposition(
                DecompositionTrackingInfo::Merge {
                    duration: timer.took().into_std(),
                    lap_id: _lap,
                    procedure: MergeType::KDSP,
                    after_merge: SolutionTrackingInfo::with_description(&best_sol),
                },
            ));

            iteration += until;
            _lap += 1;
            current_temp = current_temp - (temp_decrement_step * until as f64);

            if new_current_sol.objective() < new_best_sol.objective()
                && new_current_sol.number_of_unassigned_requests()
                <= new_best_sol.number_of_unassigned_requests()
            {
                new_best_sol = new_current_sol.clone();
            } else {
                #[cfg(feature = "relatedness-measures")]
                self.measures.process_solution_description(&new_current_sol);
            }
            #[cfg(feature = "relatedness-measures")]
            self.measures.process_solution_description(&new_best_sol);
            solution.set_with(&new_best_sol);

            if new_best_sol.number_of_unassigned_requests()
                < best_sol.number_of_unassigned_requests()
                || (new_best_sol.number_of_unassigned_requests()
                == best_sol.number_of_unassigned_requests()
                && new_best_sol.objective() < best_sol.objective())
            {
                best_sol = new_best_sol.clone();

                #[cfg(feature = "timed_solution_logger")]
                self.params.timed_solution_logger.check(
                    &countdown.time_elapsed(),
                    &best_sol,
                    &self.instance,
                );

                current_sol = new_best_sol;
                pb_outer.update(Some(&best_sol), Some(&current_sol));
                solution.set_with(&best_sol);

                #[cfg(feature = "progress_tracking_ls_lns")]
                {
                    last_improving_iteration = iteration;
                    tracking.track_component(ComponentTracking::LNS(LNSTrackingInfo::Iteration {
                        iteration,
                        update: SolutionUpdateEvent::NewBestSolution,
                        current: SolutionTrackingInfo::with_description(&current_sol),
                        best: SolutionTrackingInfo::with_description(&best_sol),
                        temperature: current_temp,
                    }));
                }
            } else if new_current_sol.number_of_unassigned_requests()
                <= current_sol.number_of_unassigned_requests()
                && f64::from(new_current_sol.objective() - best_sol.objective())
                / f64::from(new_current_sol.objective())
                < current_temp
            {
                #[cfg(feature = "progress_tracking_ls_lns")]
                    let is_improved_solution =
                    new_current_sol.objective() - best_sol.objective() <= Num::NEGATIVE_EPSILON;

                current_sol = new_current_sol;
                pb_outer.update(Some(&best_sol), Some(&current_sol));
                solution.set_with(&current_sol);

                #[cfg(feature = "progress_tracking_ls_lns")]
                tracking.track_component(ComponentTracking::LNS(LNSTrackingInfo::Iteration {
                    iteration,
                    update: if is_improved_solution {
                        SolutionUpdateEvent::ImprovedSolution
                    } else {
                        SolutionUpdateEvent::AspirationCriterion
                    },
                    current: SolutionTrackingInfo::with_description(&current_sol),
                    best: SolutionTrackingInfo::with_description(&best_sol),
                    temperature: current_temp,
                }));
            } else {
                solution.set_with(&current_sol);
            }

            pb_outer.add(until as u64);

            #[cfg(feature = "search_assertions")]
            assert_valid_solution(self.instance, &solution);
        }

        #[cfg(feature = "progress_tracking_ls_lns")]
        {
            tracking.track_component(ComponentTracking::LNS(LNSTrackingInfo::Finished {
                total_iterations: iteration,
                non_improving_count: iteration - last_improving_iteration,
                best: SolutionTrackingInfo::with_description(&best_sol),
                trigger: if countdown.is_finished() {
                    TriggeredTerminationCriterion::TimeLimit
                } else {
                    TriggeredTerminationCriterion::IterationLimit
                },
                took: timer.took().into_std(),
            }));
        }

        best_sol
    }

    fn perform(
        &self,
        _id: usize,
        instance: &PDPTWInstance,
        routes: &Vec<Vec<usize>>,
        rng: &mut Random,
        num_iterations: usize,
        mut acceptance_criterion: impl AcceptanceCriterion,
        countdown: &Countdown,
    ) -> (SolutionDescription, SolutionDescription) {
        let remove_operator =
            (self.destroy_operator_gen)(instance, DestroyParameters::from(&self.params));
        let repair_operator =
            (self.repair_operator_gen)(instance, RepairParameters::from(&self.params));

        let mut balas_simonetti_ls = BalasSimonettiLS::new(
            instance,
            instance.num_requests * 2 + 2,
            self.params.bs_thickness,
        );

        let mut sol = Solution::new(instance);
        sol.set(routes);
        sol.clamp_max_number_of_vehicles_to_current_fleet_size();

        let mut best_part_sol = sol.to_description();
        let mut current_part_sol = sol.to_description();

        let mut i = 0;
        while i < num_iterations && countdown.is_time_remaining() {
            #[cfg(feature = "search_assertions")]
            assert_valid_solution(instance, &sol);

            self.handle_destroy_operator(&remove_operator, &mut sol, rng);
            #[cfg(feature = "search_assertions")]
            assert_valid_solution(instance, &sol);

            self.handle_repair_operator(&repair_operator, &mut sol, rng);
            #[cfg(feature = "search_assertions")]
            assert_valid_solution(instance, &sol);

            if sol.is_feasible()
                && sol.number_of_unassigned_requests()
                < best_part_sol.number_of_unassigned_requests()
                || (sol.number_of_unassigned_requests()
                == best_part_sol.number_of_unassigned_requests()
                && sol.objective() < best_part_sol.objective())
            {
                match self.params.ls_method_on_new_best_sol {
                    LS_Mode::DISABLED => (),
                    LS_Mode::BS => {
                        balas_simonetti_ls.improve(&mut sol);
                    }
                }

                current_part_sol = sol.to_description();
                best_part_sol = sol.to_description();
            } else if sol.is_feasible()
                && sol.number_of_unassigned_requests()
                <= current_part_sol.number_of_unassigned_requests()
                && acceptance_criterion.check_aspiration_criterion(
                sol.objective(),
                best_part_sol.objective(),
                current_part_sol.objective(),
                rng,
            ) {
                current_part_sol = sol.to_description();
            } else {
                sol.set_with(&current_part_sol);
            }
            acceptance_criterion.update(1);
            i += 1;
        }
        (best_part_sol, current_part_sol)
    }

    fn perform_ages<'b>(
        instance: &'b PDPTWInstance,
        routes: &'b Vec<Vec<usize>>,
        max_ages_perturbation_phases: usize,
        num_perturbation_after_ejection_range: RangeInclusive<usize>,
        perturbation_mode: PerturbationMode,
        count_successful_perturbations_only: bool,
        rng: &mut Random,
        abs: &mut AbsenceCounter<'b>,
        countdown: &Countdown,
    ) -> (SolutionDescription, SolutionDescription) {
        let mut sol = Solution::new(instance);
        sol.set(routes);

        let mut min_vehicle_solution = sol.to_description();
        let mut min_unassigned_solution = sol.to_description();
        let mut cnt = 0;
        let mut pb = DefaultSearchTracker::new(max_ages_perturbation_phases as u64);
        pb.update(Some(&sol.to_description()), None);

        while cnt < max_ages_perturbation_phases && !countdown.is_finished() {
            let routes: Vec<_> = sol.iter_route_ids().collect();
            abs.apply_decay(0.80);

            // randomly select a route an un-assign all PD-pairs from it
            if sol.unassigned_requests.count() == 0 {
                if let Some(rid) = routes.choose(rng) {
                    sol.unassign_complete_route(*rid);
                    sol.clamp_max_number_of_vehicles_to_current_fleet_size();
                } else {
                    break;
                }
            }

            cnt = 0;
            pb = DefaultSearchTracker::new(max_ages_perturbation_phases as u64);

            let mut stack: Vec<_> = sol
                .unassigned_requests
                .iter_request_ids()
                .map(|it| instance.pickup_id_of_request(it))
                .collect();
            stack.shuffle(rng);
            let mut min_unassigned = stack.len();

            // for a fixed number of iterations
            while !stack.is_empty()
                && cnt < max_ages_perturbation_phases
                && !countdown.is_finished()
            {
                pb.update(Some(&min_vehicle_solution), Some(&sol.to_description()));
                let u = stack.pop().unwrap();
                if let Some(ins) = sol.find_random_insert_for_request(u, rng) {
                    // If request u was successfully reinserted in s', then the algorithm moves to the next
                    //  request in the stack.
                    sol.insert(ins);
                } else {
                    abs.increment_single_request(instance.request_id(u));

                    eject_and_insert(&mut sol, u, &mut stack, rng, abs);

                    let num_perturbations_after_ejection =
                        rng.gen_range(num_perturbation_after_ejection_range.clone());
                    let performed_perturbations = perform_perturbation(
                        &mut sol,
                        rng,
                        num_perturbations_after_ejection,
                        perturbation_mode,
                    );

                    let counted_perturbations = if count_successful_perturbations_only {
                        performed_perturbations
                    } else {
                        num_perturbations_after_ejection
                    };

                    cnt += counted_perturbations;
                    pb.add(counted_perturbations as u64);
                }

                if stack.len() < min_unassigned {
                    pb.update(Some(&min_vehicle_solution), Some(&sol.to_description()));
                    cnt = 0;
                    min_unassigned = stack.len();
                    min_unassigned_solution = sol.to_description();
                    pb = DefaultSearchTracker::new(max_ages_perturbation_phases as u64);
                }
            }
            if stack.is_empty() {
                assert_eq!(sol.unassigned_requests.count(), 0);
                min_vehicle_solution = sol.to_description();
            } else {
                sol.set_with(&min_vehicle_solution);
            }
        }
        (min_vehicle_solution, min_unassigned_solution)
    }


    fn eject_and_insert(
        sol: &mut Solution,
        u: usize,
        stack: &mut Vec<usize>,
        rng: &mut Random,
        abs: &mut AbsenceCounter,
    ) {
        if let Some(KEjectionInsertion {
                        ejections,
                        insertion,
                    }) = sol.find_best_insertion_single_ejection_for_request(u, rng, abs)
        {
            sol.unassign_request(ejections[0].pickup_id);
            stack.push(ejections[0].pickup_id);

            sol.insert(insertion);

            debug_assert!(sol.is_feasible());
        } else if let Some(KEjectionInsertion {
                               ejections,
                               insertion,
                           }) = sol.find_best_insertion_pair_ejection_for_request(u, rng, abs)
        {
            sol.unassign_request(ejections[0].pickup_id);
            stack.push(ejections[0].pickup_id);

            sol.unassign_request(ejections[1].pickup_id);
            stack.push(ejections[1].pickup_id);

            sol.insert(insertion);

            debug_assert!(sol.is_feasible());
        } else {
            stack.push(u);
        }
    }

    fn handle_destroy_operator(
        &self,
        op: &DestroyOperators,
        solution: &mut Solution,
        rng: &mut Random,
    ) {
        match op {
            DestroyOperators::AdjacentStringRemoval(op) => op.destroy(solution, rng, 10),
            DestroyOperators::RandomRouteRemoval(op) => op.destroy(solution, rng, 10),
            _ => unimplemented!(),
        }
    }

    fn handle_repair_operator(
        &self,
        op: &RepairOperators,
        solution: &mut Solution,
        rng: &mut Random,
    ) {
        match op {
            RepairOperators::GreedyInsertionWithBlinks(op) => op.repair(solution, rng),
            _ => unimplemented!(),
        }
    }
}
