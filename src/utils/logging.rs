use took::Took;

use crate::solution::{Solution, SolutionDescription};

pub fn format_log_method_solution_timed(method: &str, sol: &Solution, took: Took) -> String {
    format!("{method} - {}, took: {took}", format_log_solution(sol))
}

pub fn format_log_solution(sol: &Solution) -> String {
    format!(
        "{}/{}/{} (feasible: {})",
        sol.number_of_unassigned_requests(),
        sol.number_of_vehicles_used(),
        sol.objective(),
        sol.is_feasible(),
    )
}

pub fn format_log_solution_desc(desc: &SolutionDescription) -> String {
    format!(
        "{}/{}/{}",
        desc.number_of_unassigned_requests(),
        desc.number_of_vehicles_used(),
        desc.objective(),
    )
}

#[cfg(feature = "timed_solution_logger")]
pub mod timed {
    use std::path::Path;
    use std::time::Duration;

    use log::{info, log, warn};

    use crate::cli::ProgramArguments;
    use crate::io::sintef_solution::{write_sintef_solution, SINTEFSolutionBuilder};
    use crate::problem::pdptw::PDPTWInstance;
    use crate::solution::SolutionDescription;

    pub struct TimedSolutionLogger {
        instance_name: String,
        solution_file: Option<String>,
        solution_folder: Option<String>,
        seed_value: i128,
        times: Vec<Duration>,
        active_times_counter: usize,
        current_best: Option<SolutionDescription>,
    }

    impl TimedSolutionLogger {
        pub fn with_args_and_seed(args: &ProgramArguments, seed_value: i128) -> Self {
            let instance_name = Path::new(&args.instance)
                .file_name()
                .unwrap()
                .to_str()
                .unwrap()
                .to_string();

            // ensure one of these are available via cli
            let solution_file = args.solution.clone();
            let solution_folder = args.solution_directory.clone();

            let times = args
                .timed_solution_logging
                .iter()
                .cloned()
                .map(|it| Duration::from_secs(it as u64))
                .collect();

            Self {
                instance_name,
                solution_file,
                solution_folder,
                seed_value,
                times,
                active_times_counter: 0,
                current_best: None,
            }
        }

        pub fn check(
            &mut self,
            time_elapsed: &Duration,
            solution_desc: &SolutionDescription,
            instance: &PDPTWInstance,
        ) {
            if self.active_times_counter < self.times.len() {
                while self.active_times_counter < self.times.len()
                    && time_elapsed > &self.times[self.active_times_counter]
                {
                    // we reached the time -- do what you must
                    info!(
                        "timed logging event at {}s",
                        self.times[self.active_times_counter].as_secs()
                    );
                    if let Some(current_sol) = &self.current_best {
                        match self.write_sintef_solution(
                            instance,
                            current_sol,
                            self.times[self.active_times_counter].as_secs(),
                        ) {
                            Ok(_) => {}
                            Err(e) => {
                                warn!(
                                    "Couldn't write on timed event at {}s | {}",
                                    self.times[self.active_times_counter].as_secs(),
                                    e
                                );
                            }
                        };
                    } else {
                        // otherwise do nothing
                        warn!(
                            "Couldn't write on timed event at {}s | no solution encountered yet",
                            self.times[self.active_times_counter].as_secs(),
                        );
                    }

                    // then increment the counter
                    self.active_times_counter += 1;
                }
                if let Some(current_sol) = &self.current_best {
                    #[cfg(not(feature = "classic-pdptw"))]
                        let is_solution_better_than_current = solution_desc
                        .number_of_unassigned_requests()
                        < current_sol.number_of_unassigned_requests()
                        || (solution_desc.number_of_unassigned_requests()
                        == current_sol.number_of_unassigned_requests()
                        && solution_desc.total_cost() < current_sol.total_cost());
                    #[cfg(feature = "classic-pdptw")]
                        let is_solution_better_than_current = solution_desc.number_of_vehicles_used()
                        < current_sol.number_of_vehicles_used()
                        || (solution_desc.number_of_vehicles_used()
                        == current_sol.number_of_vehicles_used()
                        && solution_desc.total_cost() < current_sol.total_cost());
                    if is_solution_better_than_current {
                        self.current_best.replace(solution_desc.clone());
                    }
                } else {
                    self.current_best = Some(solution_desc.clone());
                }
            }
        }

        pub fn write_sintef_solution(
            &self,
            instance: &PDPTWInstance,
            solution_desc: &SolutionDescription,
            time_in_seconds: u64,
        ) -> anyhow::Result<()> {
            let mut sintef_solution_builder = SINTEFSolutionBuilder::new();
            sintef_solution_builder
                .instance_name(&self.instance_name)
                .routes_from_solution_description(solution_desc, instance);
            let solution_path = if let Some(ref file) = self.solution_file {
                format!("{}_{}s", file, time_in_seconds, )
            } else if let Some(ref folder) = self.solution_folder {
                let solution_filename = format!(
                    "{}.{}_{}_{}.{}.sol_{}s",
                    self.instance_name,
                    solution_desc.unassigned_requests,
                    solution_desc.vehicles_used,
                    solution_desc.total_cost,
                    self.seed_value,
                    time_in_seconds,
                );
                format!("{}/{}", folder, solution_filename)
            } else {
                unreachable!()
            };

            info!("writing solution to {}", &solution_path);
            write_sintef_solution(solution_path, sintef_solution_builder.build(), None)
        }
    }
}
