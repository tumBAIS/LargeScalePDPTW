use rand::Rng;

use crate::solution::Solution;
use crate::utils::Random;

pub struct RouteRemoval {}

impl RouteRemoval {
    pub fn destroy(&self, solution: &mut Solution, rng: &mut Random, num: usize) {
        let rids: Vec<usize> = solution.iter_route_ids().collect();
        assert!(rids.len() > 0);

        let mut removed = 0;
        loop {
            let rid = rids[rng.gen_range(0..rids.len())];
            removed += solution.unassign_complete_route(rid);
            if removed >= num {
                break;
            }
        }
    }
}
