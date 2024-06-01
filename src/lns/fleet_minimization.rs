use rand::distributions::uniform::SampleRange;
use rand::Rng;

use crate::cli::LS_Mode;
use crate::lns::absence_counter::AbsenceCounter;
use crate::lns::destroy::adjacent_string_removal::AdjacencyMeasure;
use crate::lns::destroy::{handle_destroy_operator_generic, DestroyOperators};
use crate::lns::repair::{handle_repair_operator_generic, RepairOperators};
use crate::lns::{destroy, repair, Parameters};
use crate::problem::pdptw::PDPTWInstance;
use crate::solution::balassimonetti::ls::BalasSimonettiLS;
use crate::solution::{Solution, SolutionDescription};
use crate::utils::{Countdown, DefaultSearchTracker, Random, SearchProgressIterationTracker};

pub struct FleetMinimizationLNS<'a> {
    instance: &'a PDPTWInstance,
    params: Parameters,
}

impl<'a> FleetMinimizationLNS<'a> {
    pub fn with_instance(instance: &'a PDPTWInstance, params: Parameters) -> Self {
        Self { instance, params }
    }

    pub fn run_with_starting_solution(
        &self,
        mut sol: Solution<'a>,
        rng: &mut Random,
        abs: Option<AbsenceCounter<'a>>,
        countdown: Countdown,
    ) -> (Solution<'a>, Option<SolutionDescription>, AbsenceCounter) {
        let mut abs = abs.unwrap_or_else(|| AbsenceCounter::new(self.instance));
        let mut best_sol: SolutionDescription = sol.to_description();

        if sol.unassigned_requests.count() == 0 {
            self.reduce_number_of_routes(&mut sol, &abs);
        }

        let mut current_sol = sol.to_description();
        let mut currently_unassigned = sol
            .unassigned_requests
            .iter_request_ids()
            .collect::<Vec<usize>>();
        let mut current_num_unassigned = sol.unassigned_requests.count();

        let removal_operators = [DestroyOperators::AdjacentStringRemoval(
            destroy::AdjacentStringRemoval::new(
                self.instance,
                AdjacencyMeasure::Classic,
                10,
                0.5,
                0.01,
            ),
        )];

        let repair_operators = [RepairOperators::GreedyInsertionWithBlinks(
            repair::GreedyInsertionWithBlinks::new(
                self.instance,
                0.01,
                true,
            ),
        )];

        let mut balas_simonetti_ls =
            BalasSimonettiLS::new(self.instance, self.instance.num_requests * 2 + 2, 4);

        let mut pb = DefaultSearchTracker::new(self.params.max_iterations as u64);
        pb.update(Some(&best_sol), None);
        for _i in 0..self.params.max_iterations {
            let num_destroy = self.params.num_destroy_range.clone().sample_single(rng);
            handle_destroy_operator_generic(
                &removal_operators[rng.gen_range(0..removal_operators.len())],
                &mut sol,
                rng,
                num_destroy,
                &abs,
            );

            handle_repair_operator_generic(
                &repair_operators[rng.gen_range(0..repair_operators.len())],
                &mut sol,
                rng,
                &abs,
            );

            if rng.gen_bool(self.params.ls_probability) {
                match self.params.ls_method {
                    LS_Mode::DISABLED => (),
                    LS_Mode::BS => {
                        balas_simonetti_ls.improve(&mut sol);
                    }
                }
            }

            if sol.unassigned_requests.count() < current_num_unassigned
                || abs.get_sum_for_unassigned(&sol)
                < abs.get_sum_for_requests(&currently_unassigned)
            {
                if sol.unassigned_requests.count() == 0 {
                    best_sol = sol.to_description();
                    self.reduce_number_of_routes(&mut sol, &abs);
                }
                current_sol = sol.to_description();
                currently_unassigned = sol
                    .unassigned_requests
                    .iter_request_ids()
                    .collect::<Vec<usize>>();

                current_num_unassigned = sol.unassigned_requests.count();
                abs.increment_for_iter_requests(sol.unassigned_requests.iter_request_ids());
                pb.update(Some(&best_sol), Some(&current_sol));
            } else {
                abs.increment_for_iter_requests(sol.unassigned_requests.iter_request_ids());

                sol.set_with(&current_sol);
            }
            pb.inc();

            if countdown.is_finished() {
                break;
            }
        }

        sol.set_with(&best_sol);
        (sol, Some(best_sol), abs)
    }

    fn reduce_number_of_routes(&self, sol: &mut Solution, abs: &AbsenceCounter) {
        // remove t ∈ T with the lowest sumAbs(t)
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
}
