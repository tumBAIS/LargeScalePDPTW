use std::cmp::Reverse;

use rand::seq::SliceRandom;

use crate::lns::absence_counter::AbsenceCounter;
use crate::problem::pdptw::PDPTWInstance;
use crate::solution::{BestInsertion, Solution};
use crate::utils::Random;

pub struct HardestFirstInsertion<'a> {
    instance: &'a PDPTWInstance,
    use_empty_route: bool,
    blink_rate: f64,
}

impl<'a> HardestFirstInsertion<'a> {
    pub fn new(instance: &'a PDPTWInstance, blink_rate: f64, use_empty_route: bool) -> Self {
        Self {
            instance,
            blink_rate,
            use_empty_route,
        }
    }
    pub fn repair(
        &self,
        solution: &mut Solution,
        absence_counter: &AbsenceCounter,
        rng: &mut Random,
    ) {
        let sorted_pickups = self.get_sorted_unassigned_customers(solution, absence_counter, rng);
        let mut v_ids: Vec<usize> = if self.use_empty_route {
            solution
                .iter_route_ids()
                .chain(solution.iter_empty_route_ids())
                .collect()
        } else {
            solution.iter_route_ids().collect()
        };
        v_ids.shuffle(rng);

        for p_id in sorted_pickups {
            // find best insertion
            let mut best_insertion: BestInsertion = BestInsertion::None;
            for &v_id in &v_ids {
                let best_insertion_in_route = solution
                    .find_best_insert_for_request_in_route_with_blinks(
                        p_id,
                        self.instance.vn_id_of(v_id),
                        rng,
                        self.blink_rate,
                    );
                best_insertion.replace_if_better(best_insertion_in_route)
            }

            if let BestInsertion::Some(ins, _) = best_insertion {
                solution.insert(ins);
                solution.unassigned_requests.remove(p_id);
            }
        }
    }

    fn get_sorted_unassigned_customers(
        &self,
        solution: &mut Solution,
        absence_counter: &AbsenceCounter,
        rng: &mut Random,
    ) -> Vec<usize> {
        let mut pickups: Vec<usize> = solution.unassigned_requests.iter_pickup_ids().collect();
        pickups.shuffle(rng);
        pickups.sort_by_cached_key(|&p_id| {
            Reverse(absence_counter.get_sum_for_request(self.instance.request_id(p_id)))
        });
        pickups
    }
}
