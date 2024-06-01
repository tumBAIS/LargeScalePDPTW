use crate::problem::pdptw::PDPTWInstance;
use crate::problem::Num;
use crate::solution::{PDInsertion, Solution};
use crate::utils::Random;

pub struct BestInsertion<'a> {
    instance: &'a PDPTWInstance,
    use_empty_route: bool,
}

impl<'a> BestInsertion<'a> {
    pub fn new(instance: &'a PDPTWInstance, use_empty_route: bool) -> Self {
        Self {
            instance,
            use_empty_route,
        }
    }
    pub fn repair(&self, solution: &mut Solution, rng: &mut Random) {
        let mut best_insertion: Vec<(usize, Option<(Num, PDInsertion, bool)>)> = solution
            .unassigned_requests
            .iter_pickup_ids()
            .map(|pickup_id| (pickup_id, None))
            .collect();
        let mut num_to_insert = best_insertion.len();

        let mut empty_v_id = solution.get_empty_route();
        let mut v_ids: Vec<usize> = solution.iter_route_ids().collect();

        let mut next_best_insertion: Option<(usize, usize, Num, PDInsertion, bool)> = None;
        for (id, (pickup_id, mut best_ins)) in best_insertion.iter_mut().enumerate() {
            for v_id in v_ids.iter() {
                if let crate::solution::BestInsertion::Some(ins, value) =
                    solution.find_best_insert_for_request_in_route(*pickup_id, v_id * 2, rng)
                {
                    if best_ins.is_none() || value < best_ins.unwrap().0 {
                        best_ins = Some((value, ins, false));
                    }
                }
            }

            if self.use_empty_route {
                if let Some(v_id) = empty_v_id {
                    if let crate::solution::BestInsertion::Some(ins, value) =
                        solution.find_best_insert_for_request_in_route(*pickup_id, v_id * 2, rng)
                    {
                        if best_ins.is_none() || value < best_ins.unwrap().0 {
                            best_ins = Some((value, ins, true));
                        }
                    }
                }
            }

            if let Some(best_ins) = best_ins {
                if next_best_insertion.is_none() || best_ins.0 < next_best_insertion.unwrap().2 {
                    next_best_insertion =
                        Some((id, *pickup_id, best_ins.0, best_ins.1, best_ins.2));
                }
            }
        }

        while num_to_insert > 0 {
            if let Some((id, next_p_id, _, next_ins, empty_route)) = next_best_insertion {
                let changed_vn_id = if empty_route {
                    empty_v_id.unwrap() * 2
                } else {
                    next_ins.vn_id
                };
                v_ids.push(changed_vn_id / 2);
                empty_v_id = solution.get_empty_route();

                solution.insert(next_ins);
                solution.unassigned_requests.remove(next_p_id);

                num_to_insert -= 1;
                if num_to_insert <= 0 {
                    break;
                }

                best_insertion.swap_remove(id);
                next_best_insertion = None;
                for (pickup_id, mut best_ins) in &mut best_insertion {
                    if let Some((_value, ins, _)) = best_ins {
                        if ins.vn_id == changed_vn_id {
                            if empty_route {
                                // no need to recalculate - value is still valid
                            } else {
                                best_ins = None;
                                // reevaluate all routes
                                for v_id in v_ids.iter() {
                                    if let crate::solution::BestInsertion::Some(ins, value) =
                                        solution.find_best_insert_for_request_in_route(
                                            *pickup_id,
                                            v_id * 2,
                                            rng,
                                        )
                                    {
                                        if best_ins.is_none() || value < best_ins.unwrap().0 {
                                            best_ins = Some((value, ins, false));
                                        }
                                    }
                                }
                            }
                        } else {
                            if let crate::solution::BestInsertion::Some(ins, value) = solution
                                .find_best_insert_for_request_in_route(
                                    *pickup_id,
                                    changed_vn_id,
                                    rng,
                                )
                            {
                                if best_ins.is_none() || value < best_ins.unwrap().0 {
                                    best_ins = Some((value, ins, false));
                                }
                            }
                        }
                    }

                    if let Some(best_ins) = best_ins {
                        if next_best_insertion.is_none()
                            || best_ins.0 < next_best_insertion.unwrap().2
                        {
                            next_best_insertion =
                                Some((id, *pickup_id, best_ins.0, best_ins.1, best_ins.2));
                        }
                    }
                }
            } else {
                break;
            }
        }
    }
}
