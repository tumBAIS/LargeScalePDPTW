use std::ops::RangeInclusive;

use crate::cli::LS_Mode;
use crate::problem::pdptw::PDPTWInstance;

pub mod absence_counter;
pub(crate) mod acceptance_criterion;
pub mod destroy;
pub mod fleet_minimization;
pub mod largescale;
pub mod repair;

pub struct Parameters {
    pub max_iterations: usize,
    pub max_non_improving_iterations: Option<usize>,
    pub num_destroy_range: RangeInclusive<usize>,
    pub ls_probability: f64,
    pub ls_method: LS_Mode,
}

impl Parameters {
    pub fn default_for_instance(_instance: &PDPTWInstance) -> Self {
        Self {
            max_iterations: 200,
            max_non_improving_iterations: None,
            num_destroy_range: (10..=30),
            ls_probability: 0.0,
            ls_method: LS_Mode::DISABLED,
        }
    }
}
