use crate::problem::pdptw::PDPTWInstance;
use crate::solution::{Solution, SolutionDescription};

#[derive(Clone)]
pub struct AbsenceCounter<'a> {
    instance: &'a PDPTWInstance,
    counters: Vec<usize>,
}

impl<'a> AbsenceCounter<'a> {
    pub fn new(instance: &'a PDPTWInstance) -> Self {
        Self {
            instance,
            counters: vec![1usize; instance.num_requests],
        }
    }

    pub fn clear(&mut self) {
        self.fill(0);
    }

    pub fn fill(&mut self, value: usize) {
        self.counters.fill(value);
    }

    pub fn apply_decay(&mut self, rate: f64) {
        self.counters
            .iter_mut()
            .for_each(|it| *it = (*it as f64 * rate).ceil() as usize);
    }

    pub fn get_sum_for_request(&self, request: usize) -> usize {
        self.counters[request]
    }

    pub fn get_sum_for_request_by_pickup_id(&self, pickup_id: usize) -> usize {
        self.counters[pickup_id / 2 - self.instance.num_vehicles]
    }

    pub fn get_share_of_request(&self, request: usize) -> f64 {
        self.counters[request] as f64 / self.counters.iter().sum::<usize>() as f64
    }

    pub fn get_sum_for_requests(&self, requests: &Vec<usize>) -> usize {
        requests.iter().map(|&r| self.counters[r]).sum()
    }

    pub fn get_sum_for_iter_requests_by_pickup_id(
        &self,
        iter: impl Iterator<Item=usize>,
    ) -> usize {
        iter.map(|r| self.counters[r / 2 - self.instance.num_vehicles])
            .sum()
    }

    pub fn get_sum_for_unassigned(&self, solution: &Solution) -> usize {
        solution
            .unassigned_requests
            .iter_request_ids()
            .map(|r| self.counters[r])
            .sum()
    }

    pub fn get_sum_for_unassigned_from_desc(&self, desc: &SolutionDescription) -> usize {
        desc.successors
            .iter()
            .enumerate()
            .filter(|(idx, it)| idx == *it && self.instance.is_pickup(**it))
            .map(|(_, n_id)| self.counters[n_id / 2 - self.instance.num_vehicles])
            .sum()
    }

    pub fn increment_single_request(&mut self, request: usize) {
        self.counters[request] += 1;
    }
    pub fn increment_for_iter_requests(&mut self, iter: impl Iterator<Item=usize>) {
        for r in iter {
            self.counters[r] += 1;
        }
    }
    pub fn set_single_request(&mut self, request: usize, value: usize) {
        self.counters[request] = value;
    }
    pub fn iter_pairs(&self) -> impl Iterator<Item=(usize, &usize)> {
        self.counters.iter().enumerate()
    }
}
