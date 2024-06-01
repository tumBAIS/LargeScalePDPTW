use rand::seq::SliceRandom;

use crate::lns::absence_counter::AbsenceCounter;
use crate::problem::pdptw::PDPTWInstance;
use crate::problem::Num;
use crate::solution::{BestInsertion, PDInsertion, Solution};
use crate::utils::rcl::RCL;
use crate::utils::Random;

pub struct AbsenceBasedRegretInsertion<'a> {
    instance: &'a PDPTWInstance,
    use_empty_route: bool,
}

impl<'a> AbsenceBasedRegretInsertion<'a> {
    pub fn new(instance: &'a PDPTWInstance, use_empty_route: bool) -> Self {
        Self {
            instance,
            use_empty_route,
        }
    }
    pub fn repair(
        &self,
        solution: &mut Solution,
        absence_counter: &AbsenceCounter,
        rng: &mut Random,
    ) {
        let mut v_ids: Vec<usize> = if self.use_empty_route {
            solution
                .iter_route_ids()
                .chain(solution.iter_empty_route_ids())
                .collect()
        } else {
            solution.iter_route_ids().collect()
        };
        v_ids.shuffle(rng);

        let mut rcl = RCL::<PDInsertion, 4>::new();

        loop {
            let max_available = 5;
            let mut best_regret = Num::ZERO;
            let mut best_regret_insertion = None;

            for p_id in solution.unassigned_requests.iter_pickup_ids() {
                rcl.clear();
                // find best insertion
                for &v_id in &v_ids {
                    let best_insertion_in_route = solution
                        .find_best_insert_for_request_in_route_with_blinks(
                            p_id,
                            self.instance.vn_id_of(v_id),
                            rng,
                            0.0,
                        );

                    match best_insertion_in_route {
                        BestInsertion::Some(ins, num) => {
                            rcl.push((
                                num * Num::from(
                                    2.0 - absence_counter
                                        .get_share_of_request(self.instance.request_id(p_id)),
                                ),
                                ins,
                            ));
                        }
                        BestInsertion::None => {}
                    };
                }
                if !rcl.is_empty() {
                    if rcl.len() < max_available {
                        best_regret = rcl.iter().skip(1).map(|it| it.0).sum::<Num>()
                            - (rcl.iter().next().unwrap().0 * Num::from(rcl.len()));
                        best_regret_insertion = Some(rcl.pop_first_and_clear());
                    } else if rcl.len() == max_available {
                        let regret = rcl.iter().map(|it| it.0).skip(1).sum::<Num>()
                            - (rcl.iter().next().unwrap().0 * Num::from(rcl.len()));
                        if regret > best_regret {
                            best_regret = regret;
                            best_regret_insertion = Some(rcl.pop_first_and_clear());
                        }
                    }
                }
            }
            if let Some(ins) = best_regret_insertion {
                solution.insert(ins);
            } else {
                break;
            }
        }
    }
}
