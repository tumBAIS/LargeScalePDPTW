use fixedbitset::FixedBitSet;

use crate::problem::pdptw::PDPTWInstance;
use crate::problem::Num;
use crate::solution::{Solution, SolutionDescription};

#[derive(Debug)]
pub enum Violation {
    Precedence,
    Demand(Num),
    TimeWindow(Num),
}

#[derive(Debug)]
pub enum ValidatorResult {
    Valid(Num),
    ConstraintViolation(Violation),
    ObjectiveMismatch(Num),
}

impl ValidatorResult {
    pub fn is_valid(&self) -> bool {
        match self {
            Self::Valid(_) => true,
            _ => false,
        }
    }

    pub fn assert_valid(&self) {
        match self {
            Self::Valid(_) => {}
            Self::ConstraintViolation(violation) => {
                assert!(false, "{:?}", violation)
            }
            Self::ObjectiveMismatch(num) => {
                assert!(false, "ObjectiveMismatch({})", num)
            }
        }
    }
}

pub fn validate_route(
    instance: &PDPTWInstance,
    route: &Vec<usize>,
    objective: Option<Num>,
) -> ValidatorResult {
    use ValidatorResult::*;
    use Violation::*;

    let capacity = instance.vehicles[route[0] / 2].seats;

    let mut pickups_visited = FixedBitSet::with_capacity(instance.num_requests);
    let mut deliveries_visited = FixedBitSet::with_capacity(instance.num_requests);

    let mut load = instance.nodes[route[0]].demand;
    let mut distance = Num::ZERO;
    let mut time = instance.nodes[route[0]].ready + instance.nodes[route[0]].servicetime;

    for i in 1..route.len() {
        let node = &instance.nodes[route[i]];
        if node.node_type.is_pickup() {
            pickups_visited.insert(instance.request_id(node.id))
        } else if node.node_type.is_delivery() {
            deliveries_visited.insert(instance.request_id(node.id))
        }

        load += node.demand;

        if load > capacity {
            return ConstraintViolation(Demand((load - capacity).into()));
        }

        let distance_and_time = instance.distance_and_time(route[i - 1], route[i]);
        distance += distance_and_time.distance;

        time += distance_and_time.time;

        if time > node.due {
            return ConstraintViolation(TimeWindow(time - node.due));
        } else if time < node.ready {
            time = node.ready;
        }
        time += node.servicetime;
    }

    if pickups_visited.difference(&deliveries_visited).count() > 0 {
        return ConstraintViolation(Precedence);
    }

    if objective.is_some() && distance != objective.unwrap() {
        ObjectiveMismatch(distance)
    } else {
        Valid(distance)
    }
}

pub fn assert_valid_solution_description(instance: &PDPTWInstance, desc: &SolutionDescription) {
    let mut solution = Solution::new(instance);
    solution.set_with(desc);
    assert_valid_solution(instance, &solution)
}

pub fn assert_valid_solution(instance: &PDPTWInstance, solution: &Solution) {
    use ValidatorResult::*;
    use Violation::*;

    let mut total_distances = Num::ZERO;
    for route in solution
        .to_description()
        .to_routes_vec(instance)
        .into_iter()
    {
        match validate_route(instance, &route, None) {
            Valid(distance) => total_distances += distance,
            ConstraintViolation(violation) => match violation {
                Precedence => {
                    panic!("precedence violation")
                }
                Demand(excess) => {
                    panic!("demand violation (excess: {})", excess)
                }
                TimeWindow(excess) => {
                    panic!("time window violation (tardiness: {})", excess)
                }
            },
            _ => {
                unreachable!()
            }
        }
    }
    assert_eq!(total_distances, solution.total_cost());
}
