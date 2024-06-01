use fixedbitset::FixedBitSet;

pub use counter::LinkCounter;

use crate::problem::pdptw::PDPTWInstance;
use crate::solution::{Solution, SolutionDescription};
use crate::utils::relatedness::counter::SimpleCounter;

mod counter;

pub struct RelatednessMeasures<'a> {
    instance: &'a PDPTWInstance,
    pub link_counter: LinkCounter,
    pub same_route_counter: SimpleCounter,
}

impl<'a> RelatednessMeasures<'a> {
    pub fn with_instance(instance: &'a PDPTWInstance) -> Self {
        Self {
            instance,
            link_counter: LinkCounter::with_instance(instance),
            same_route_counter: SimpleCounter::with_instance(instance),
        }
    }

    pub fn process_solution_description(&mut self, desc: &SolutionDescription) {
        let mut requests_in_route = FixedBitSet::with_capacity(self.instance.num_requests);
        let num_vehicles = self.instance.num_vehicles;
        for v in 0..num_vehicles {
            requests_in_route.clear();
            let first = desc.successors[v * 2];
            if first != v * 2 + 1 {
                let mut prev_request = self.instance.request_id(first);
                let mut node_id = desc.successors[first];
                while node_id != v * 2 + 1 {
                    if self.instance.is_pickup(node_id) {
                        let request_id = self.instance.request_id(node_id);
                        requests_in_route.put(request_id);

                        self.link_counter.inc_predecessor(request_id, prev_request);
                        self.link_counter.inc_successor(prev_request, request_id);

                        prev_request = request_id;
                    }
                    node_id = desc.successors[node_id];
                }
            }

            // same route counter
            for r1 in requests_in_route.ones() {
                for r2 in requests_in_route.ones() {
                    if r1 == r2 {
                        continue;
                    } else {
                        self.same_route_counter.inc(r1, r2);
                    }
                }
            }
        }
    }

    pub fn process_solution(&mut self, sol: &Solution) {
        for route_id in sol.iter_route_ids() {
            let mut iter = sol.iter_requests_of_route(route_id);
            let mut prev = iter.next();
            for request_id in iter {
                self.link_counter.inc_predecessor(request_id, prev.unwrap());
                self.link_counter.inc_successor(prev.unwrap(), request_id);
                prev.replace(request_id);
            }
            // same route counter
            for r1 in sol.iter_requests_of_route(route_id) {
                for r2 in sol.iter_requests_of_route(route_id) {
                    if r1 == r2 {
                        continue;
                    } else {
                        self.same_route_counter.inc(r1, r2);
                    }
                }
            }
        }
    }

    pub fn get_requests_relation_value(&self, rs1: &Vec<usize>, rs2: &Vec<usize>) -> usize {
        rs1.iter()
            .cloned()
            .map(|r1| {
                rs2.iter()
                    .cloned()
                    .map(|r2| self.same_route_counter.get_count(r1, r2))
                    .sum::<usize>()
            })
            .sum()
    }

    pub fn get_requests_relation_value_from_itineraries(
        &self,
        it1: &Vec<usize>,
        it2: &Vec<usize>,
    ) -> usize {
        it1.iter()
            .cloned()
            .flat_map(|n1| {
                if self.instance.is_pickup(n1) {
                    Some(self.instance.request_id(n1))
                } else {
                    None
                }
            })
            .map(|r1| {
                it2.iter()
                    .cloned()
                    .flat_map(|n2| {
                        if self.instance.is_pickup(n2) {
                            Some(self.instance.request_id(n2))
                        } else {
                            None
                        }
                    })
                    .map(|r2| self.same_route_counter.get_count(r1, r2))
                    .sum::<usize>()
            })
            .sum::<usize>()
    }
}
