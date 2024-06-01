use num_traits::Inv;
use rand::Rng;

use crate::problem::Num;
use crate::utils::Random;

pub enum AcceptanceCriterionStrategy {
    ExponentialMetropolis {
        initial_temperature: f64,
        final_temperature: f64,
    },
    LinearRecordToRecord {
        initial_temperature: f64,
        final_temperature: f64,
    },
    None,
}

#[derive(Clone)]
pub struct ExponentialMetropolisCriteria {
    initial_temperature: f64,
    final_temperature: f64,
    current_temperature: f64,
    exponential_cooling_factor: f64,
}

impl ExponentialMetropolisCriteria {
    pub fn new(initial_temperature: f64, final_temperature: f64, num_iterations: usize) -> Self {
        Self {
            exponential_cooling_factor: (final_temperature / initial_temperature)
                .powf((num_iterations as f64).inv()),
            current_temperature: initial_temperature,
            initial_temperature,
            final_temperature,
        }
    }
}

#[derive(Clone)]
pub struct LinearRecordToRecordCriteria {
    initial_temperature: f64,
    final_temperature: f64,
    current_temperature: f64,
    linear_cooling_constant: f64,
}

impl LinearRecordToRecordCriteria {
    pub fn new(initial_temperature: f64, final_temperature: f64, num_iterations: usize) -> Self {
        Self {
            linear_cooling_constant: (initial_temperature - final_temperature)
                / (num_iterations as f64),
            current_temperature: initial_temperature,
            initial_temperature,
            final_temperature,
        }
    }
}

pub trait AcceptanceCriterion: Clone + Sync + Send {
    fn update(&mut self, num_iterations: usize);
    fn check_aspiration_criterion(
        &self,
        new_solution_objective: Num,
        best_solution_objective: Num,
        current_solution_objective: Num,
        rng: &mut Random,
    ) -> bool;

    fn get_current_temperature(&self) -> f64;
}

impl AcceptanceCriterion for ExponentialMetropolisCriteria {
    fn update(&mut self, num_iterations: usize) {
        self.current_temperature *= self.exponential_cooling_factor.powi(num_iterations as i32);
    }
    fn check_aspiration_criterion(
        &self,
        new_solution_objective: Num,
        _best_solution_objective: Num,
        current_solution_objective: Num,
        rng: &mut Random,
    ) -> bool {
        f64::from(new_solution_objective)
            < (f64::from(current_solution_objective)
                - f64::from(self.current_temperature) * (rng.gen_range(0.0..=1.0f64).ln()))
    }
    fn get_current_temperature(&self) -> f64 {
        self.current_temperature
    }
}

impl AcceptanceCriterion for LinearRecordToRecordCriteria {
    fn update(&mut self, num_iterations: usize) {
        self.current_temperature -= self.linear_cooling_constant * (num_iterations as f64);
    }
    fn check_aspiration_criterion(
        &self,
        new_solution_objective: Num,
        best_solution_objective: Num,
        _current_solution_objective: Num,
        _rng: &mut Random,
    ) -> bool {
        f64::from(new_solution_objective - best_solution_objective)
            / f64::from(new_solution_objective)
            < self.current_temperature
    }
    fn get_current_temperature(&self) -> f64 {
        self.current_temperature
    }
}

#[derive(Clone)]
pub struct OnlyImprovementsCriteria;

impl AcceptanceCriterion for OnlyImprovementsCriteria {
    fn update(&mut self, _num_iterations: usize) {}
    fn check_aspiration_criterion(
        &self,
        new_solution_objective: Num,
        _best_solution_objective: Num,
        current_solution_objective: Num,
        _rng: &mut Random,
    ) -> bool {
        new_solution_objective < current_solution_objective
    }
    fn get_current_temperature(&self) -> f64 {
        0.0
    }
}
