use rand::seq::SliceRandom;

use crate::problem::pdptw::{Node, PDPTWInstance};
use crate::problem::Num;
use crate::solution::Solution;
#[cfg(feature = "search_assertions")]
use crate::utils::validator::assert_valid_solution;
use crate::utils::Random;

pub struct BestBinInsertion<'a> {
    pub instance: &'a PDPTWInstance,
}

impl<'a> BestBinInsertion<'a> {
    pub fn new(instance: &'a PDPTWInstance) -> Self {
        Self { instance }
    }
    pub fn construct<'b>(&self, rng: &mut Random) -> Solution<'b>
    where
        'a: 'b,
    {
        let solution = {
            let mut solution = Solution::new(self.instance);
            let mut pickup_nodes: Vec<&Node> = self.instance.iter_pickups().collect();
            pickup_nodes.shuffle(rng);
            pickup_nodes.sort_by(|a, b| (a.ready).cmp(&(b.ready)));

            let mut bins = (0..self.instance.num_vehicles)
                .map(|v_id| vec![v_id * 2])
                .collect::<Vec<Vec<usize>>>();
            let mut earliest_time_of_last_node = (0..self.instance.num_vehicles)
                .map(|_v_id| Num::ZERO)
                .collect::<Vec<Num>>();

            for p in pickup_nodes {
                let mut best_bin = None;
                let mut best_cost = Num::MAX;
                let mut best_time = Num::MAX;
                for bin_id in 0..self.instance.num_vehicles {
                    if earliest_time_of_last_node[bin_id] > p.due {
                        continue;
                    }
                    let last_node = bins[bin_id].last().unwrap().clone();

                    let distance_and_time = self.instance.distance_and_time(last_node, p.id);

                    if distance_and_time.distance < best_cost
                        && earliest_time_of_last_node[bin_id] + distance_and_time.time <= p.due
                    {
                        best_bin = Some(bin_id);
                        best_cost = distance_and_time.distance;
                        best_time = earliest_time_of_last_node[bin_id] + distance_and_time.time
                    }
                }
                if let Some(bin_id) = best_bin {
                    let d = self.instance.delivery_of(p.id);

                    bins[bin_id].push(p.id);
                    bins[bin_id].push(d.id);

                    earliest_time_of_last_node[bin_id] = d.ready.max(
                        best_time.max(p.ready)
                            + p.servicetime
                            + self.instance.distance_and_time(p.id, d.id).time,
                    ) + d.servicetime;
                } else {
                    solution.track_request_unassigned(p.id);
                }
            }
            for v_id in 0..self.instance.num_vehicles {
                bins[v_id].push(v_id * 2 + 1);
            }
            solution.set(&bins);
            solution
        };

        #[cfg(feature = "search_assertions")]
        assert_valid_solution(self.instance, &solution);
        solution
    }
}
