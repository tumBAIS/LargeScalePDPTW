use crate::problem::pdptw::PDPTWInstance;
use crate::problem::Num;
use crate::solution::Solution;
use crate::utils::Random;

pub struct WorstRemoval<'a> {
    instance: &'a PDPTWInstance,
}

impl<'a> WorstRemoval<'a> {
    pub fn with_instance(instance: &'a PDPTWInstance) -> Self {
        Self { instance }
    }

    pub fn destroy(&self, solution: &mut Solution, _rng: &mut Random, num: usize) {
        let rids: Vec<usize> = solution.iter_route_ids().collect();
        let mut worst: Vec<Option<(Num, usize)>> = vec![None; rids.len()];

        for id in 0..rids.len() {
            worst[id] = Self::calculate_worst(self.instance, solution, rids[id]);
        }

        let mut removed = 0;
        loop {
            let mut worst_value = Num::MIN;
            let mut worst_pid = 0;
            let mut worst_rid = 0;
            for i in 0..rids.len() {
                if let Some(value) = worst[i] {
                    if value.0 > worst_value {
                        worst_value = value.0;
                        worst_pid = value.1;
                        worst_rid = i;
                    }
                }
            }
            if worst_value != Num::MIN {
                solution.unassign_request(worst_pid);
                worst[worst_rid] = Self::calculate_worst(self.instance, &solution, rids[worst_rid]);
                removed += 1;
                if removed >= num {
                    break;
                }
            } else {
                break;
            }
        }
    }

    fn calculate_worst(
        instance: &PDPTWInstance,
        solution: &Solution,
        rid: usize,
    ) -> Option<(Num, usize)> {
        let mut iter = solution.iter_route_by_vn_id(rid * 2).skip(1);
        let mut max_gain = (Num::MIN, 0);
        while let Some(node) = iter.next() {
            let node = node;
            if instance.nodes[node].node_type.is_pickup() {
                let gain = solution.calculate_change_on_removing_request_of(node);
                if gain > max_gain.0 {
                    max_gain = (gain, node)
                }
            }
        }
        if max_gain.0 != Num::MIN {
            Some(max_gain)
        } else {
            None
        }
    }
}
