# Solver for Large-Scale Pickup and Delivery Problems with Time Windows

This codebase was developed to tackle large-scale ride-sharing optimization problems.
A detailed description of the problem and resolution approach is available online on
[arxiv](https://arxiv.org/abs/2405.00230).

## Project structure

- `./src/` - contains the source code of the solver, with `main.rs` being the main entry point.
- `./Cargo.toml` comprise of configurations and dependencies used by `cargo` to build the project.
- `./libs/` - contains two libraries we previously developed:
    - `fp_decimal_type` is a simple floating point decimal type used in the resource calculations.
    - `kdsp` is a rust re-implementation of a k-disjoint shortest path algorithm used
      in [Schiffer et al. (2021)](https://doi.org/10.1016/j.trb.2020.11.001)
- `./resources/` - contains other files, e.g., instances, solutions and miscellaneous scripts.

## Installation

The solver is implemented in the programming language [Rust](https://www.rust-lang.org/tools/install).
The experiments were run using version 1.70. To compile the project, install the dependencies and
build the project using `cargo build` (with `--release` for non-debug build).

## Usage

We use the commonly used `log` crate for our logging purposes, with `env-logger` to print to the terminal.
On default, the logging is disabled. To enable the logging, you need to set the environmental variable `RUST_LOG` to
the desired level, e.g., `RUST_LOG=info` to output all info-level information.

Parameters are passed by CLI arguments. Run `cargo run -- --help` for a list of available options.

Parameters used in the experiments can be found in the paper, and are
available as text files to be passed by prefixing ``@`` to the corresponding filepath, e.g., 

``cargo run --features=classic-pdptw --release -- --instance ./resources/instances/pdptw/n200/bar-n200-1.txt --time-limit-in-seconds 60 @./resources/parameters/benchmark.args``

The parameter files are located in ``./resources/parameters/``, including
- ``benchmark.args`` - parameters used for the classic-pdptw instances (Sartori&Buriol)
- ``case-study.args`` - parameters used for the case-study (NYC)
- ``matching-only.args`` parameters used to run the matching procedure

### Classic PDPTW

The solver was initially designed to solve a large-scale ride-sharing problem. We applied the approach
to classic PDPTW instances as well, focussing on the larger instance sets introduced in
[Sartori & Buriol (2020)](https://doi.org/10.1016/j.cor.2020.105065), available
[here](https://github.com/cssartori/pdptw-instances).
To run the classic benchmark instances, the project needs to be built with the
`classic-pdptw` feature enabled, e.g., `cargo run --features=classic-pdptw --release`.
Assuming the instances are located in `./resources/instances/pdptw/` the solver can be
run using

``cargo run --features=classic-pdptw --release -- --instance ./resources/instances/pdptw/n200/bar-n200-1.txt --time-limit-in-seconds 60 @./resources/parameters/benchmark.args``

### New instance set based on the NYC data

To run the new instance sets, the default build can be used. In addition to the other parameters, the instance path
needs to point to the instance
main `.toml` file,
which contains the relative paths to the respective requests, vehicles, and distance matrix file paths.

``cargo run --release -- --instance .\resources\instances\nyc\p15\ny_2015_01_day-04\18h\60m\ny_2015_01_day-04_18h_60m_b0m-do_v300_c3.toml --time-limit-in-seconds 3600``

Running the sequential matheuristic approach with matching by solving a weighted-set-covering problem, requires
Gurobi (version 9.x) installed, and the feature `use-grb` enabled.

## Features

We use conditional compilation via [features](https://doc.rust-lang.org/cargo/reference/features.html) specified in the
`Cargo.toml` file. Following features are available:

- `parallel`: enable parallel processing using the [`rayon` crate](https://docs.rs/rayon/latest/rayon/). The maximum
  number of threads can be set by specifying the environmental variable `RAYON_NUM_THREADS`,
  e.g., `RAYON_NUM_THREADS=8` to allow up to 8 threads to be used.
- `progressbar`: enables an optional progressbar for a more active experience
- `classic-pdptw`: replaces IO and other functionality to accommodate for solving the classic PDPTW instances.
  Note that if enabled, you can no longer load and solve instances of the NYC instance set.
- `timed_solution_logger`: enables `--timed-solution-logging` CLI option to output the best found solution so far for
  provided points in time (see `--help` for details).
- `disable-decomposition`: disables the decomposition (split + nested search + merge) component.
- `progress_tracking_all`: enables more detailed tracking information on the search progress to be collected. Note that
  you should specify the filepath `--tracking-file` in the CLI to write the JSON output.

## Licence

MIT
