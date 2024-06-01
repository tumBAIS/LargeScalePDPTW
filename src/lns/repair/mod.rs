use std::cmp::Reverse;

use rand::seq::SliceRandom;
use rand::Rng;

pub use absence_based_regret_insertion::AbsenceBasedRegretInsertion;
pub use greedy_insertion_with_blinks::GreedyInsertionWithBlinks;
pub use hardest_first_insertion::HardestFirstInsertion;

use crate::lns::absence_counter::AbsenceCounter;
use crate::problem::pdptw::PDPTWInstance;
use crate::solution::Solution;
use crate::utils::Random;

mod absence_based_regret_insertion;
mod best_insertion;
mod greedy_insertion_with_blinks;
mod hardest_first_insertion;
mod regret_insertion;

pub enum RepairOperators<'a> {
    GreedyInsertionWithBlinks(GreedyInsertionWithBlinks<'a>),
    HardestFirstInsertion(HardestFirstInsertion<'a>),
    AbsenceBasedRegretInsertion(AbsenceBasedRegretInsertion<'a>),
}

pub fn handle_repair_operator_generic(
    op: &RepairOperators,
    sol: &mut Solution,
    rng: &mut Random,
    absence_counter: &AbsenceCounter,
) {
    match op {
        RepairOperators::GreedyInsertionWithBlinks(op) => op.repair(sol, rng),
        RepairOperators::AbsenceBasedRegretInsertion(op) => op.repair(sol, &absence_counter, rng),
        RepairOperators::HardestFirstInsertion(op) => op.repair(sol, &absence_counter, rng),
    }
}

fn sort_unassigned_customers(
    instance: &PDPTWInstance,
    solution: &mut Solution,
    rng: &mut Random,
) -> Vec<usize> {
    // In addition to the sorting orders introduced in Section 5.3 (random, demand, far, and close),
    // customers are sorted with respect to increasing time window length, increasing time window
    // start, and decreasing time window end.
    enum Order {
        Random,
        Demand,
        Far,
        Close,
        TimeWindowLength,
        TimeWindowStart,
        TimeWindowEnd,
    }
    use Order::*;
    let orders = [
        Random,
        Demand,
        Far,
        Close,
        TimeWindowLength,
        TimeWindowStart,
        TimeWindowEnd,
    ];
    let weights = [4, 4, 2, 1, 2, 2, 2];
    let accumulated_weights = [4, 8, 10, 11, 13, 15, 17];
    let w = rng.gen_range(0..weights.iter().sum::<usize>());
    let order = &orders[accumulated_weights
        .into_iter()
        .enumerate()
        .skip_while(|(_, it)| *it <= w)
        .next()
        .unwrap()
        .0];

    let mut pickups: Vec<usize> = solution.unassigned_requests.iter_pickup_ids().collect();
    match order {
        Random => pickups.shuffle(rng),
        Demand => pickups.sort_by_key(|&p_id| Reverse(instance.nodes[p_id].demand)),
        Far => pickups.sort_by_cached_key(|&p_id| Reverse(instance.distance(0, p_id))),
        Close => pickups.sort_by_cached_key(|&p_id| instance.distance(0, p_id)),
        TimeWindowLength => pickups
            .sort_by_cached_key(|&p_id| instance.nodes[p_id].due - instance.nodes[p_id].ready),
        TimeWindowStart => pickups.sort_by_cached_key(|&p_id| instance.nodes[p_id].ready),
        TimeWindowEnd => pickups.sort_by_cached_key(|&p_id| Reverse(instance.nodes[p_id].due)),
    }
    pickups
}
