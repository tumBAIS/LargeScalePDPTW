/// Parallel insertion heuristic based on Malheiros et al. (2021)

use rand::seq::SliceRandom;
use rand::Rng;

use crate::problem::pdptw::{Node, PDPTWInstance};
use crate::problem::Num;
use crate::solution::BestInsertion;
use crate::solution::{PDInsertion, Solution};
use crate::utils::{Random, Tolerance};

pub struct ParallelInsertion<'a> {
    pub instance: &'a PDPTWInstance,
}

impl<'a> ParallelInsertion<'a> {
    pub fn new(instance: &'a PDPTWInstance) -> Self {
        Self {
            instance,
        }
    }
    pub fn construct<'b>(&self, rng: &mut Random) -> Solution<'b>
        where
            'a: 'b,
    {
        let solution = loop {
            let mut solution = Solution::new(self.instance);
            let mut pickup_nodes: Vec<&Node> = self
                .instance
                .nodes
                .iter()
                .skip(self.instance.num_vehicles * 2)
                .filter(|n| n.id % 2 == 0)
                .collect();
            pickup_nodes.shuffle(rng);
            for v_id in 0..self.instance.num_vehicles {
                for idx in 0..pickup_nodes.len() {
                    let pickup_node = pickup_nodes[idx];
                    let travel_time = self.instance.time(v_id * 2, pickup_node.id);
                    // Note: when we have heterogeneous shifts for vehicles:
                    // - check whether the request can be assigned to the vehicle (shift start/end)
                    if self.instance.vehicles[v_id].check_capacity(pickup_node.demand)
                        && travel_time <= pickup_node.due
                    {
                        // reachable in time, take it
                        solution.insert(PDInsertion {
                            vn_id: v_id * 2,
                            pickup_id: pickup_node.id,
                            pickup_after: v_id * 2,
                            delivery_before: (v_id * 2) + 1,
                        });

                        pickup_nodes.swap_remove(idx);
                        break;
                    }
                }
            }
            pickup_nodes.sort_by(|a, b| (a.ready).cmp(&(b.ready)));

            for p_id in pickup_nodes.iter().map(|n| n.id) {
                let mut best_ins = BestInsertion::None;
                let mut num_min = 1;
                for v_id in 0..self.instance.num_vehicles {
                    let ins = solution.find_best_insert_for_request_in_route(p_id, v_id * 2, rng);
                    let replace = match (&ins, &best_ins) {
                        (BestInsertion::None, _) => false,
                        (BestInsertion::Some(_, _), BestInsertion::None) => true,
                        (BestInsertion::Some(_, cost), BestInsertion::Some(_, best_cost)) => {
                            if cost < &(best_cost - Num::tol()) {
                                num_min = 1;
                                true
                            } else if (best_cost - cost).abs() < Num::tol()
                                && rng.gen_range(0..=num_min) == num_min
                            {
                                num_min = num_min + 1;
                                true
                            } else {
                                false
                            }
                        }
                    };
                    if replace {
                        best_ins = ins
                    }
                }
                if let BestInsertion::Some(ins, _) = best_ins {
                    solution.insert(ins);
                } else {
                    solution.track_request_unassigned(p_id);
                }
            }
            break solution;
        };
        solution
    }
}
