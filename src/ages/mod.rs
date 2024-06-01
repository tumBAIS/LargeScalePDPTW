use rand::Rng;

use crate::lns::absence_counter::AbsenceCounter;
use crate::solution::permutation::{KEjectionInsertion, PDEjection};
use crate::solution::{PDInsertion, Solution};
use crate::utils::Random;

pub(crate) fn eject_and_insert(
    sol: &mut Solution,
    u: usize,
    stack: &mut Vec<usize>,
    rng: &mut Random,
    abs: &mut AbsenceCounter,
) {
    fn perform_k_ejections_and_insertion(
        sol: &mut Solution,
        stack: &mut Vec<usize>,
        ejections: &[PDEjection],
        insertion: PDInsertion,
    ) {
        for ejection in ejections {
            sol.unassign_request(ejection.pickup_id);
            stack.push(ejection.pickup_id);
        }
        sol.insert(insertion);
    }

    if let Some(KEjectionInsertion {
                    ejections,
                    insertion,
                }) = sol.find_best_insertion_k_ejection_for_request::<1>(u, rng, abs)
    {
        perform_k_ejections_and_insertion(sol, stack, &ejections, insertion);
        debug_assert!(sol.is_feasible());
    } else if let Some(KEjectionInsertion {
                           ejections,
                           insertion,
                       }) = sol.find_best_insertion_k_ejection_for_request::<2>(u, rng, abs)
    {
        perform_k_ejections_and_insertion(sol, stack, &ejections, insertion);
        debug_assert!(sol.is_feasible());
    } else {
        stack.push(u);
    }
}

#[derive(Copy, Clone, Debug)]
pub enum PerturbationMode {
    RelocateAndExchange { shift_probability: f64 },
    BiasedRelocation { bias: f64 },
}

pub(crate) fn perform_perturbation(
    solution: &mut Solution,
    rng: &mut Random,
    num_perturbations: usize,
    mode: PerturbationMode,
) -> usize {
    use PerturbationMode::*;
    match mode {
        RelocateAndExchange { shift_probability } => perform_relocate_and_exchange_perturbation(
            solution,
            rng,
            num_perturbations,
            shift_probability,
        ),
        BiasedRelocation { bias } => {
            perform_biased_perturbation(solution, rng, num_perturbations, bias)
        }
    }
}

pub(crate) fn perform_relocate_and_exchange_perturbation(
    solution: &mut Solution,
    rng: &mut Random,
    num_perturbations: usize,
    shift_probability: f64,
) -> usize {
    let mut cnt = 0;
    for _ in 0..num_perturbations {
        if rng.gen_bool(shift_probability) {
            if solution.random_shift(rng) {
                cnt += 1;
            }
        } else {
            if solution.random_exchange(rng) {
                cnt += 1;
            }
        }
    }
    cnt
}

pub(crate) fn perform_biased_perturbation(
    solution: &mut Solution,
    rng: &mut Random,
    num_perturbations: usize,
    bias: f64,
) -> usize {
    let mut cnt = 0;
    for _ in 0..num_perturbations {
        if solution.biased_random_shift(rng, bias) {
            cnt += 1;
        }
    }
    cnt
}

pub enum PenaltyCounterResetStrategy {
    ResetOnNewMin,
    DecayOnNewMin(f64),
    Ignore,
}

pub enum ILSSolutionSelectionStrategy {
    AlwaysCurrent,
    AlwaysBest,
    NonImprovementBased,
}
