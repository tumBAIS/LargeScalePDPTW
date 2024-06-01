use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;
use std::time::{Duration, Instant};

use chrono::{DateTime, Utc};
use serde::{Serialize, Serializer};

use crate::problem::Num;
use crate::solution::SolutionDescription;
use crate::Solution;

#[derive(Serialize)]
pub(crate) enum TriggeredTerminationCriterion {
    TimeLimit,
    IterationLimit,
    NonImproving,
}

pub(crate) enum TrackEvent {}

#[derive(Serialize)]
pub(crate) enum SolutionUpdateEvent {
    NewBestSolution,
    ImprovedSolution,
    AspirationCriterion,
    KeepCurrent,
}

trait SolutionTracker {
    fn get_best_solution_info(&self) -> Option<&SolutionTrackingInfo>;
}

#[derive(Serialize)]
pub(crate) enum FleetMinLNSTrackingInfo {
    Start {
        init: SolutionTrackingInfo,
    },
    Iteration {
        iteration: usize,
        update: SolutionUpdateEvent,
        current: SolutionTrackingInfo,
        best: SolutionTrackingInfo,
    },
    Finished {
        took: Duration,
        total_iterations: usize,
        non_improving_count: usize,
        best: SolutionTrackingInfo,
        trigger: TriggeredTerminationCriterion,
    },
}

#[derive(Serialize)]
pub(crate) enum LNSTrackingInfo {
    Start {
        init: SolutionTrackingInfo,
        temperature: f64,
    },
    Iteration {
        iteration: usize,
        // num_destroyed: usize,
        // after_repair: SolutionTrackingInfo,
        update: SolutionUpdateEvent,
        best: SolutionTrackingInfo,
        current: SolutionTrackingInfo,
        temperature: f64,
    },
    Finished {
        took: Duration,
        total_iterations: usize,
        non_improving_count: usize,
        best: SolutionTrackingInfo,
        trigger: TriggeredTerminationCriterion,
    },
}

pub(crate) enum BalasSimonettiTrackingInfo {
    Start {
        init: SolutionTrackingInfo,
    },
    Finished {
        took: Duration,
        routes_modified: usize,
        best: SolutionTrackingInfo,
    },
}

#[derive(Serialize)]
pub(crate) enum ILSTrackingInfo {
    Start {
        init: SolutionTrackingInfo,
    },
    Iteration {
        iteration: usize,
        before_permutation: SolutionTrackingInfo,
        after_permutation: SolutionTrackingInfo,
        num_permutations_performed: usize,
        best: SolutionTrackingInfo,
    },
    Finished {
        took: Duration,
        total_iterations: usize,
        best: SolutionTrackingInfo,
        trigger: TriggeredTerminationCriterion,
    },
}
#[derive(Serialize)]
pub(crate) enum SplitType {
    RoutesOnly,
    TimeThenRoutes,
}
#[derive(Serialize)]
pub(crate) enum MergeType {
    KDSP,
}

#[derive(Serialize)]
pub(crate) enum DecompositionTrackingInfo {
    Split {
        lap_id: usize,
        procedure: SplitType,
        num_partitions: usize,
        avg_nodes_per_split: usize,
        init: SolutionTrackingInfo,
    },
    Iterations {
        lap_id: usize,
        part_id: usize,
        num_assigned: usize,
        num_unassigned: usize,
        part_current: SolutionTrackingInfo,
        part_best: SolutionTrackingInfo,
        update: SolutionUpdateEvent,
    },
    Merge {
        lap_id: usize,
        procedure: MergeType,
        // blocks: usize,
        duration: Duration,
        after_merge: SolutionTrackingInfo,
    },
}

pub(crate) enum ComponentTracking {
    FleetMinLNS(FleetMinLNSTrackingInfo),
    LNS(LNSTrackingInfo),
    AGES,
    ILS(ILSTrackingInfo),
    BalasSimonetti,
    LocalSearch,
    FleetMinDecomposition(DecompositionTrackingInfo),
    Decomposition(DecompositionTrackingInfo),
}

impl SolutionTracker for ComponentTracking {
    fn get_best_solution_info(&self) -> Option<&SolutionTrackingInfo> {
        match &self {
            Self::LNS(tracking) => match tracking {
                LNSTrackingInfo::Start { init, .. } => Some(init),
                LNSTrackingInfo::Finished { best, .. } => Some(best),
                LNSTrackingInfo::Iteration { best, .. } => Some(best),
            },
            _ => unimplemented!(),
        }
    }
}

pub struct Tracking<E> {
    when: Duration,
    what: E,
}

impl<E> Tracking<E> {
    pub fn of(what: E, elapsed: Duration) -> Self {
        Self {
            what,
            when: elapsed,
        }
    }
}

#[derive(Clone, Debug, Serialize)]
pub(crate) struct SolutionTrackingInfo {
    num_unassigned: usize,
    num_vehicles: usize,
    objective: Num,
}

impl Serialize for Num {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        f64::from(self.clone()).serialize(serializer)
    }
}

impl SolutionTrackingInfo {
    pub fn with_description(desc: &SolutionDescription) -> Self {
        Self {
            num_unassigned: desc.number_of_unassigned_requests(),
            num_vehicles: desc.number_of_vehicles_used(),
            objective: desc.objective(),
        }
    }

    pub fn with_solution(sol: &Solution) -> Self {
        Self {
            num_unassigned: sol.number_of_unassigned_requests(),
            num_vehicles: sol.number_of_vehicles_used(),
            objective: sol.objective(),
        }
    }
}

pub struct SearchProgressTracking {
    init_datetime: DateTime<Utc>,
    init_instant: Instant,
    component: Vec<Tracking<ComponentTracking>>,
    best_solution: Vec<Tracking<SolutionTrackingInfo>>,
}

impl SearchProgressTracking {
    pub fn new() -> Self {
        Self {
            init_datetime: Utc::now(),
            init_instant: Instant::now(),
            component: vec![],
            best_solution: vec![],
        }
    }

    pub(crate) fn track_component_at_time(
        &mut self,
        component: ComponentTracking,
        instant: Instant,
    ) {
        self.component.push(Tracking::of(
            component,
            instant.duration_since(self.init_instant),
        ));
    }

    pub(crate) fn track_component(&mut self, component: ComponentTracking) {
        self.track_component_at_time(component, Instant::now());
    }

    fn consider_best_solution<E: SolutionTracker>(&mut self, tracking: &Tracking<E>) {
        if let Some(sol) = tracking.what.get_best_solution_info() {
            let last: Option<&Tracking<SolutionTrackingInfo>> = self.best_solution.last();
            let is_new_best = if let Some(last) = last {
                sol.num_unassigned == 0
                    && sol.num_vehicles <= last.what.num_vehicles
                    && sol.objective < last.what.objective
            } else {
                // no solution yet, so it has to be the best
                true
            };
            if is_new_best {
                self.best_solution.push(Tracking {
                    when: tracking.when.clone(),
                    what: sol.clone(),
                })
            }
        }
    }
}

impl SearchProgressTracking {
    pub fn write_json(&self, path: &Path) -> std::io::Result<()> {
        let f = File::create(path)?;
        let mut file = BufWriter::new(&f);
        write!(file, "{{")?;
        write!(file, "\"datetime\":\"{}\",", self.init_datetime)?;
        write!(file, "\"tracks\":[")?;
        let mut first = None;
        for tracking in self.component.iter() {
            if first.is_none() {
                write!(file, "{{")?;
                first = Some(());
            } else {
                write!(file, ",{{")?;
            }
            write!(file, "\"t\":{}", serde_json::to_string(&tracking.when)?)?;
            write!(file, ",")?;
            match &tracking.what {
                ComponentTracking::FleetMinLNS(info) => {
                    write!(file, "\"FleetMinLNS\":{}", serde_json::to_string(info)?)?
                }
                ComponentTracking::FleetMinDecomposition(info) => write!(
                    file,
                    "\"FleetMinDecomposition\":{}",
                    serde_json::to_string(info)?
                )?,
                ComponentTracking::LNS(info) => {
                    write!(file, "\"LNS\":{}", serde_json::to_string(info)?)?
                }
                ComponentTracking::Decomposition(info) => {
                    write!(file, "\"Decomposition\":{}", serde_json::to_string(info)?)?
                }
                ComponentTracking::ILS(info) => {
                    write!(file, "\"ILS\":{}", serde_json::to_string(info)?)?
                }

                _ => {}
            };
            // match tracking.what {
            //     ComponentTracking::FleetMinLNS(info) => match info {
            //         FleetMinLNSTrackingInfo::Start { init } => {
            //             // init.write_json(file)?;
            //         },
            //         FleetMinLNSTrackingInfo::Iteration {
            //             iteration, update, current, best
            //         } => {
            //
            //         },
            // };
            write!(file, "}}")?;
        }
        write!(file, "]")?;
        write!(file, "}}")?;
        Ok(())
    }
}
