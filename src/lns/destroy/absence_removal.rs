use rand::seq::SliceRandom;

use crate::lns::absence_counter::AbsenceCounter;
use crate::problem::pdptw::PDPTWInstance;
use crate::solution::Solution;
use crate::utils::Random;

pub struct AbsenceRemoval<'a> {
    instance: &'a PDPTWInstance,
}

impl<'a> AbsenceRemoval<'a> {
    pub(crate) fn new(instance: &'a PDPTWInstance) -> Self {
        Self { instance }
    }

    pub fn destroy(
        &self,
        solution: &mut Solution,
        rng: &mut Random,
        absence_counter: &AbsenceCounter,
        num: usize,
    ) {
        let mut pairs: Vec<(usize, &usize)> = absence_counter
            .iter_pairs()
            .filter(|(r_id, _)| !solution.unassigned_requests.contains_request(*r_id))
            .collect();
        pairs.shuffle(rng);
        pairs.sort_by(|a, b| a.1.cmp(&b.1).then(a.0.cmp(&b.0)));

        for (r_id, _absence) in pairs.into_iter().take(num) {
            solution.unassign_request(self.instance.pickup_id_of_request(r_id));
        }
    }
}
