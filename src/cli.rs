use std::ops::RangeInclusive;

use clap::builder::ArgPredicate;
use clap::{Parser, ValueEnum};
use crate::ages::PerturbationMode;

use crate::construction::kdsp::{KDSPBlockMode, KDSPSettings};
use crate::lns::acceptance_criterion::AcceptanceCriterionStrategy;
use crate::lns::destroy::adjacent_string_removal::AdjacencyMeasure;
use crate::lns::largescale;
use crate::problem::pdptw::PDPTWInstance;
use crate::solver::construction::InitialSolutionGeneration;

#[derive(Parser, Debug)]
#[command(version)]
pub struct ProgramArguments {
    #[arg(long, help = "rng seed")]
    pub seed: Option<i128>,

    #[arg(short, long, help = "instance file path")]
    pub instance: String,

    #[arg(short, long, help = "solution file path")]
    pub solution: Option<String>,

    #[arg(
        long,
        help = "directory to store the solution",
        conflicts_with = "solution"
    )]
    pub solution_directory: Option<String>,

    #[cfg(feature = "progress_tracking")]
    #[arg(long, help = "file to store the track")]
    pub tracking_file: Option<String>,

    #[arg(long, help = "maximum number of vehicles")]
    pub max_vehicles: Option<usize>,

    #[command(flatten)]
    pub solver: SolverArguments,

    #[arg(
        long,
        help = "print objective and time for tuning purposes (e.g., irace)",
        default_value = "false"
    )]
    pub print_for_tuning: bool,

    #[arg(long, help = "print summary to stdout", default_value = "false")]
    pub print_summary_to_stdout: bool,

    #[cfg(feature = "timed_solution_logger")]
    #[arg(long, value_delimiter = ' ', num_args = 0..)]
    pub timed_solution_logging: Vec<u64>,
}

#[allow(non_camel_case_types)]
#[derive(Clone, ValueEnum, Debug)]
pub enum Solver {
    LS_AGES_LNS,
    Construction_Only,
}

#[allow(non_camel_case_types)]
#[derive(Clone, ValueEnum, Debug)]
pub enum LS_Mode {
    DISABLED,
    BS,
}

#[allow(non_camel_case_types)]
#[derive(Clone, ValueEnum, Debug)]
pub enum RecombineMode {
    Naive,
    KDSP_Earliest,
    KDSP_Latest,
    KDSP_Average,
}

#[allow(non_camel_case_types)]
#[derive(Clone, ValueEnum, Debug)]
pub enum AspirationMode {
    Metropolis,
    RecordToRecord,
    None,
}

#[allow(non_camel_case_types)]
#[derive(Clone, ValueEnum, Debug)]
pub enum MatchingMode {
    Greedy,
    WSC,
    WSP,
}

#[allow(non_camel_case_types)]
#[derive(Clone, ValueEnum, Debug)]
pub enum LPWeightFn {
    Cardinality,
    WeightedCost,
    WeightedTemporalOverlap,
    WeightedTemporalDeviation,
    WeightedCostAndTemporalOverlap,
    WeightedCostAndTemporalDeviation,
}

#[allow(non_camel_case_types)]
#[derive(Clone, ValueEnum, Debug)]
pub enum LPModelType {
    ILP,
    LP,
}

#[allow(non_camel_case_types)]
#[derive(Clone, ValueEnum, Debug)]
pub enum FractionalSelectionStrategy {
    Greedy,
    StandardRandomizedRounding,
    ConditionalRandomizedRounding,
}

impl SolverArguments {
    pub(crate) fn get_recombine_mode(&self) -> largescale::RecombineMode {
        match self.recombine_mode {
            RecombineMode::Naive => largescale::RecombineMode::Naive,
            RecombineMode::KDSP_Earliest => largescale::RecombineMode::KDSP(KDSPSettings {
                block_mode: KDSPBlockMode::Earliest,
                maximum_distance_to_next_request: self
                    .kdsp_maximum_distance_to_next_request
                    .map(|it| it.into()),
                maximum_time_to_next_request: self
                    .kdsp_maximum_time_to_next_request
                    .map(|it| it.into()),
            }),
            RecombineMode::KDSP_Latest => largescale::RecombineMode::KDSP(KDSPSettings {
                block_mode: KDSPBlockMode::Latest,
                maximum_distance_to_next_request: self
                    .kdsp_maximum_distance_to_next_request
                    .map(|it| it.into()),
                maximum_time_to_next_request: self
                    .kdsp_maximum_time_to_next_request
                    .map(|it| it.into()),
            }),
            RecombineMode::KDSP_Average => largescale::RecombineMode::KDSP(KDSPSettings {
                block_mode: KDSPBlockMode::Average,
                maximum_distance_to_next_request: self
                    .kdsp_maximum_distance_to_next_request
                    .map(|it| it.into()),
                maximum_time_to_next_request: self
                    .kdsp_maximum_time_to_next_request
                    .map(|it| it.into()),
            }),
        }
    }
}

#[derive(clap::Args, Clone, Debug)]
pub struct SolverArguments {
    #[arg(long = "solver", value_enum, default_value = "ls-ages-lns")]
    pub variant: Solver,
    #[arg(long)]
    pub ils_iterations: Option<usize>,
    #[arg(long, default_value = "5000")]
    pub lns_iterations: usize,
    #[arg(long, default_value = "3600")]
    pub time_limit_in_seconds: u64,
    #[arg(long, value_delimiter = ' ', num_args = 6..=6, default_value = "6 2 1 4 2 2")]
    pub lns_recreate_order_weights: Vec<usize>,
    #[arg(long, default_value = "0.05")]
    pub lns_recreate_blink_rate: f64,
    #[arg(long, default_value = "40")]
    pub lns_recreate_insertion_limit: usize,
    #[arg(long, default_value = "classic")]
    pub lns_ruin_measure: AdjacencyMeasure,
    #[arg(long, default_value = "10")]
    pub lns_ruin_max_cardinality: usize,
    #[arg(long, default_value = "0.75")]
    pub lns_ruin_alpha: f64,
    #[arg(long, default_value = "0.10")]
    pub lns_ruin_beta: f64,

    #[arg(long, value_delimiter = ' ', num_args = 2..=2, default_value = "15 15")]
    pub num_destroy_range: Vec<usize>,
    #[arg(long, default_value = "2500")]
    pub nested_iterations: usize,
    #[arg(long, value_delimiter = ' ', num_args = 2..=2, default_value = "500 500")]
    pub avg_nodes_per_split: Vec<usize>,
    #[arg(long, default_value = "0.0333")]
    pub lns_init_aspiration_temp: f64,
    #[arg(long, default_value = "0.0")]
    pub lns_final_aspiration_temp: f64,
    #[arg(long, default_value = "record-to-record")]
    pub aspiration_mode: AspirationMode,

    #[arg(long, default_value = "0.01")]
    pub ls_probability: f64,
    #[arg(long, default_value = "disabled")]
    pub lns_ls_mode: LS_Mode,
    #[arg(long, default_value = "bs")]
    pub lns_ls_mode_on_new_best_sol: LS_Mode,
    #[arg(long, default_value = "4")]
    pub bs_thickness: usize,
    #[arg(long, default_value = "kdsp-earliest")]
    pub recombine_mode: RecombineMode,
    #[arg(
        long,
        help = "spatial limit in meters for arcs in the k-dSP auxiliary graph (default: no limit)"
    )]
    pub kdsp_maximum_distance_to_next_request: Option<usize>,
    #[arg(
        long,
        help = "temporal limit in seconds for arcs in the k-dSP auxiliary graph (default: no limit)"
    )]
    pub kdsp_maximum_time_to_next_request: Option<usize>,

    #[arg(long, conflicts_with("ages_perturbation_random_relocate_exchange"))]
    pub ages_perturbation_biased_relocate: Option<f64>,
    #[arg(
        long,
        default_value = "0.58",
        default_value_if("ages_perturbation_biased_relocate", ArgPredicate::IsPresent, None),
        conflicts_with("ages_perturbation_biased_relocate")
    )]
    pub ages_perturbation_random_relocate_exchange: Option<f64>,

    #[arg(long, default_value = "1.0")]
    pub ages_penalty_counter_decay: f64,
    #[arg(long, default_value = "false")]
    pub ages_shuffle_stack_after_permutation: bool,
    #[arg(long, default_value = "false")]
    pub ages_count_successful_perturbations_only: bool,
    #[arg(long, default_value = "1000000")]
    pub ages_max_perturbation_phases: usize,
    #[arg(long, default_value = "1.66")]
    pub ages_num_perturbation_ils_rel_requests: f64,

    #[arg(
        long,
        value_delimiter = ' ',
        num_args = 2..=2,
        conflicts_with("ages_num_perturbation_after_ejection_rel_requests")
    )]
    pub ages_num_perturbation_after_ejection_abs: Option<Vec<usize>>,
    #[arg(
        long,
        value_delimiter = ' ',
        num_args = 2..=2,
        default_value = "0.15 0.15", default_value_if("ages_num_perturbation_after_ejection_abs", ArgPredicate::IsPresent, None),
        conflicts_with("ages_num_perturbation_after_ejection_abs")
    )]
    pub ages_num_perturbation_after_ejection_rel_requests: Option<Vec<f64>>,
    #[arg(long, default_value = "parallel-insertion")]
    pub init: InitialSolutionGeneration,

    #[arg(long, help = "solution to warmstart the solver")]
    pub warmstart_solution_file: Option<String>,

    #[arg(long, default_value = "greedy")]
    pub matching_mode: MatchingMode,

    #[arg(long, help = "time limit for solving the WSC model (default: unlimited)")]
    pub wsc_time_limit_in_seconds: Option<u64>,

    #[arg(long, default_value = "cardinality")]
    pub lp_weight_fn: LPWeightFn,

    #[arg(long, default_value = "0.2")]
    pub lp_weight_fn_rho: f64,

    #[arg(long, default_value = "lp")]
    pub lp_model_type: LPModelType,

    #[arg(long, default_value = "greedy")]
    pub fractional_selection_strategy: FractionalSelectionStrategy,

    #[arg(long, default_value = "average")]
    pub pooling_kdsp_mode: KDSPBlockMode,

    #[arg(
        long,
        help = "directory to store cached pooling",
        conflicts_with("pooling_cache_filepath")
    )]
    pub pooling_cache_directory: Option<String>,

    #[arg(
        long,
        help = "custom prefix for cache file",
        conflicts_with("pooling_cache_filepath")
    )]
    pub pooling_cache_prefix: Option<String>,

    #[arg(
        long,
        help = "path to the cache file",
        conflicts_with_all(["pooling_cache_directory", "pooling_cache_prefix"])
    )]
    pub pooling_cache_filepath: Option<String>,
}

impl SolverArguments {
    pub(crate) fn num_destroy_range(&self) -> RangeInclusive<usize> {
        self.num_destroy_range[0].min(self.num_destroy_range[1])
            ..=self.num_destroy_range[1].max(self.num_destroy_range[0])
    }
    pub(crate) fn avg_nodes_per_split_range(&self) -> RangeInclusive<usize> {
        self.avg_nodes_per_split[0].min(self.avg_nodes_per_split[1])
            ..=self.avg_nodes_per_split[1].max(self.avg_nodes_per_split[0])
    }
    pub(crate) fn ages_perturbation_mode_ejection_search(&self) -> PerturbationMode {
        if let Some(ref bias) = self.ages_perturbation_biased_relocate {
            PerturbationMode::BiasedRelocation { bias: bias.clone() }
        } else if let Some(ref prob) = self.ages_perturbation_random_relocate_exchange {
            PerturbationMode::RelocateAndExchange {
                shift_probability: prob.clone(),
            }
        } else {
            panic!("Invalid perturbation mode encountered! (should have been caught by the cli)")
        }
    }
    pub(crate) fn ages_perturbation_mode_ils(&self) -> PerturbationMode {
        if let Some(ref bias) = self.ages_perturbation_biased_relocate {
            PerturbationMode::BiasedRelocation { bias: bias.clone() }
        } else if let Some(ref prob) = self.ages_perturbation_random_relocate_exchange {
            PerturbationMode::RelocateAndExchange {
                shift_probability: prob.clone(),
            }
        } else {
            panic!("Invalid perturbation mode encountered! (should have been caught by the cli)")
        }
    }
    pub(crate) fn ages_num_perturbations_after_ejection_range(
        &self,
        instance: &PDPTWInstance,
    ) -> RangeInclusive<usize> {
        if let Some(ref abs) = self.ages_num_perturbation_after_ejection_abs {
            abs[0]..=abs[1]
        } else if let Some(ref rel) = self.ages_num_perturbation_after_ejection_rel_requests {
            ((instance.num_requests as f64) * rel[0]).floor() as usize
                ..=((instance.num_requests as f64) * rel[1]).floor() as usize
        } else {
            panic!("Invalid perturbation-after-ejection values encountered! (should have been caught by the cli)")
        }
    }
    pub(crate) fn ages_num_perturbations_ils(&self, instance: &PDPTWInstance) -> usize {
        (self.ages_num_perturbation_ils_rel_requests * instance.num_requests as f64).round()
            as usize
    }

    pub(crate) fn lns_acceptance_criterion_strategy(&self) -> AcceptanceCriterionStrategy {
        match self.aspiration_mode {
            AspirationMode::None => AcceptanceCriterionStrategy::None,
            AspirationMode::Metropolis => AcceptanceCriterionStrategy::ExponentialMetropolis {
                initial_temperature: self.lns_init_aspiration_temp,
                final_temperature: self.lns_final_aspiration_temp,
            },
            AspirationMode::RecordToRecord => AcceptanceCriterionStrategy::LinearRecordToRecord {
                initial_temperature: self.lns_init_aspiration_temp,
                final_temperature: self.lns_final_aspiration_temp,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn verify_cli() {
        use clap::CommandFactory;
        ProgramArguments::command().debug_assert()
    }
}
