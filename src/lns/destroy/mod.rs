mod absence_removal;
pub mod adjacent_string_removal;
mod route_removal;
mod worst_removal;

use crate::lns::absence_counter::AbsenceCounter;
use crate::solution::Solution;
use crate::utils::Random;
pub use absence_removal::AbsenceRemoval;
pub use adjacent_string_removal::AdjacentStringRemoval;
pub use route_removal::RouteRemoval as RandomRouteRemoval;

pub enum DestroyOperators<'a> {
    AdjacentStringRemoval(AdjacentStringRemoval<'a>),
    AbsenceRemoval(AbsenceRemoval<'a>),
    RandomRouteRemoval(RandomRouteRemoval),
}

pub fn handle_destroy_operator_generic(
    op: &DestroyOperators,
    solution: &mut Solution,
    rng: &mut Random,
    num_destroy: usize,
    absence_counter: &AbsenceCounter,
) {
    match op {
        DestroyOperators::AbsenceRemoval(op) => {
            op.destroy(solution, rng, &absence_counter, num_destroy)
        }
        DestroyOperators::AdjacentStringRemoval(op) => op.destroy(
            solution,
            rng,
            num_destroy,
        ),
        DestroyOperators::RandomRouteRemoval(op) => op.destroy(
            solution,
            rng,
            num_destroy,
        ),
    }
}
