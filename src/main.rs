#![allow(dead_code)]

use std::path::Path;

use clap::{CommandFactory, FromArgMatches};
use log::info;
use os_str_bytes::OsStrBytesExt;
use rand::random;
use took::Timer;

use crate::cli::Solver;
use crate::io::load_instance;
#[cfg(not(feature = "classic-pdptw"))]
use crate::io::nyc_reader::NYCInstanceElements;
use crate::io::sintef_solution::SINTEFSolutionBuilder;
#[cfg(not(feature = "classic-pdptw"))]
use crate::problem::pdptw::create_instance_with;
#[cfg(not(feature = "classic-pdptw"))]
use crate::problem::travel_matrix::TravelMatrixProxy;
use crate::solution::Solution;
#[cfg(feature = "timed_solution_logger")]
use crate::utils::logging::timed::TimedSolutionLogger;
#[cfg(feature = "progress_tracking")]
use crate::utils::stats::search_progress_stats::SearchProgressTracking;
#[cfg(feature = "search_assertions")]
use crate::utils::validator::{assert_valid_solution, assert_valid_solution_description};
use crate::utils::create_seeded_rng;

mod clustering;
mod construction;
mod io;
mod lns;
mod problem;
mod refn;
mod solution;
mod solver;
mod utils;

mod cli;
mod pooling;
mod ages;

fn main() -> anyhow::Result<()> {
    env_logger::init();

    let args = argfile::expand_args_from(
        std::env::args_os(),
        argfile::parse_fromfile,
        argfile::PREFIX,
    )?;
    let args = cli::ProgramArguments::from_arg_matches(
        &cli::ProgramArguments::command()
            .get_matches_from(args.iter().flat_map(|it| {
                it.split(" ").into_iter().collect::<Vec<_>>()
            }))
    )?;
    info!("{:?}", &args);

    let (seed_value, mut rng) = {
        let seed_value = args.seed.unwrap_or_else(|| random::<i128>().abs());
        info!("seed: {}", seed_value);
        (seed_value, create_seeded_rng(seed_value))
    };
    let max_vehicles = args.max_vehicles;

    #[cfg(not(feature = "classic-pdptw"))]
    {
        let load_timer = Timer::new();
        let NYCInstanceElements {
            name,
            num_vehicles,
            num_requests,
            vehicles,
            nodes,
            travel_matrix,
        } = load_instance(&args.instance, max_vehicles)?;

        // revisit later: how to avoid leaking the matrix?
        let matrix = Box::leak(Box::new(travel_matrix));
        let mapping = nodes.iter().map(|node| node.gid).collect();
        let instance = create_instance_with(
            name,
            num_vehicles,
            num_requests,
            vehicles,
            nodes,
            TravelMatrixProxy::new(
                mapping, matrix,
            ),
        )?;

        #[cfg(feature = "progress_tracking")]
            let mut progress_tracking = SearchProgressTracking::new();

        info!("instance loaded after {}", load_timer.took());

        info!("starting solver");
        let res = match &args.solver.variant {
            Solver::LS_AGES_LNS => solver::ls_solver::ls_ages_lns(
                &instance,
                &args.solver,
                &mut rng,
                #[cfg(feature = "progress_tracking")] &mut progress_tracking,
                #[cfg(
                    feature = "timed_solution_logger"
                )] TimedSolutionLogger::with_args_and_seed(&args, seed_value),
            ),
            Solver::Construction_Only => {
                solver::construction_only(&instance, &args.solver, &mut rng)
            }
        };

        info!("finished after {}", res.time);
        info!(
            "best solution found: {}/{}/{}",
            res.solution.number_of_unassigned_requests(),
            res.solution.number_of_vehicles_used(),
            res.solution.objective()
        );

        #[cfg(feature = "search_assertions")]
        assert_valid_solution_description(&instance, &res.solution);

        if args.print_summary_to_stdout {
            println!(
                "{},{},{},{}",
                res.solution.number_of_unassigned_requests(),
                res.solution.number_of_vehicles_used(),
                res.solution.total_cost(),
                res.time.as_std().as_secs()
            );
        }

        let instance_name = Path::new(&args.instance)
            .file_name()
            .unwrap()
            .to_str()
            .unwrap();

        let solution_out = args
            .solution
            .map(|it| it.to_string())
            .or(args.solution_directory.map(|dir| {
                format!(
                    "{}/{}.{}_{}_{}.{}.sol",
                    dir,
                    instance_name,
                    res.solution.unassigned_requests,
                    res.solution.vehicles_used,
                    res.solution.objective,
                    seed_value
                )
            }));

        if let Some(solution_path) = solution_out {
            let mut sintef_solution_builder = SINTEFSolutionBuilder::new();
            sintef_solution_builder
                .instance_name(instance_name)
                .routes_from_solution_description(&res.solution, &instance);
            io::sintef_solution::write_sintef_solution(
                solution_path.to_string(),
                sintef_solution_builder.build(),
                Some(res),
            )?;
        }

        #[cfg(feature = "progress_tracking")]
        {
            if let Some(tracking_filepath) = args.tracking_file.map(|it| it.to_string()) {
                progress_tracking.write_json(Path::new(tracking_filepath.as_str()))?;
            }
        }

        return Ok(());
    }
    #[cfg(feature = "classic-pdptw")]
    {
        let load_timer = Timer::new();
        let instance = load_instance(&args.instance, max_vehicles)?;

        log::info!("instance loaded after {}", load_timer.took());

        log::info!("starting solver {:?}", &args.solver.variant);

        #[cfg(feature = "progress_tracking")]
            let mut progress_tracking = SearchProgressTracking::new();

        let res = match &args.solver.variant {
            Solver::LS_AGES_LNS => solver::ls_solver::ls_ages_lns(
                &instance,
                &args.solver,
                &mut rng,
                #[cfg(feature = "progress_tracking")] &mut progress_tracking,
                #[cfg(
                    feature = "timed_solution_logger"
                )] TimedSolutionLogger::with_args_and_seed(&args, seed_value),
            ),
            Solver::Construction_Only => {
                solver::construction_only(&instance, &args.solver, &mut rng)
            }
        };
        log::info!("finished after {}", res.time);
        log::info!(
            "best solution found {}/{}/{}",
            res.solution.number_of_unassigned_requests(),
            res.solution.number_of_vehicles_used(),
            res.solution.objective
        );

        if args.print_for_tuning {
            println!(
                "{:01}{:04}{:010} {}",
                res.solution.number_of_unassigned_requests(),
                res.solution.number_of_vehicles_used(),
                res.solution.total_cost().value(),
                res.time.as_std().as_secs()
            );
        }

        #[cfg(feature = "search_assertions")]
        assert_valid_solution_description(&instance, &res.solution);

        let instance_name = Path::new(&args.instance)
            .file_name()
            .unwrap()
            .to_str()
            .unwrap();

        let solution_out = args
            .solution
            .map(|it| it.to_string())
            .or(args.solution_directory.map(|dir| {
                format!(
                    "{}/{}.{}_{}_{}.{}.sol",
                    dir,
                    instance_name,
                    res.solution.unassigned_requests,
                    res.solution.vehicles_used,
                    res.solution.total_cost,
                    seed_value
                )
            }));

        if let Some(solution_path) = solution_out {
            let mut sintef_solution_builder = SINTEFSolutionBuilder::new();
            sintef_solution_builder
                .instance_name(instance_name)
                .routes_from_solution_description(&res.solution, &instance);
            io::sintef_solution::write_sintef_solution(
                solution_path.to_string(),
                sintef_solution_builder.build(),
                Some(res),
            )?;
        }

        #[cfg(feature = "progress_tracking")]
        {
            if let Some(tracking_filepath) = args.tracking_file.map(|it| it.to_string()) {
                progress_tracking.write_json(Path::new(tracking_filepath.as_str()))?;
            }
        }
        Ok(())
    }
}
