use fixedbitset::FixedBitSet;

use crate::problem::pdptw::PDPTWInstance;
use crate::problem::Num;

/// stores unassigned request (ids)
#[derive(Clone)]
pub struct RequestBank<'a> {
    instance: &'a PDPTWInstance,
    requests: FixedBitSet,
    pub penalty_per_entry: Num,
}

impl<'a> RequestBank<'a> {
    pub fn with_instance(instance: &'a PDPTWInstance) -> Self {
        let mut requests = FixedBitSet::with_capacity(instance.num_requests);
        requests.set_range(.., true);
        Self {
            instance,
            requests,
            penalty_per_entry: Num::from(10_000),
        }
    }

    pub fn iter_request_ids(&self) -> impl Iterator<Item = usize> + '_ {
        self.requests.ones()
    }

    pub fn iter_pickup_ids(&self) -> impl Iterator<Item = usize> + '_ {
        self.requests
            .ones()
            .map(move |it| self.request_to_pickup_id(it))
    }
}

impl RequestBank<'_> {
    fn pickup_to_request_id(&self, pickup_id: usize) -> usize {
        (pickup_id / 2) - self.instance.num_vehicles
    }
    fn request_to_pickup_id(&self, request_id: usize) -> usize {
        (request_id + self.instance.num_vehicles) * 2
    }
    pub fn insert_pickup_id(&mut self, pickup_id: usize) {
        self.requests.insert(self.pickup_to_request_id(pickup_id));
    }
    pub fn remove(&mut self, pickup_id: usize) {
        self.requests
            .set(self.pickup_to_request_id(pickup_id), false)
    }
    pub fn contains(&self, pickup_id: usize) -> bool {
        self.requests.contains(self.pickup_to_request_id(pickup_id))
    }
    pub fn contains_request(&self, request_id: usize) -> bool {
        self.requests.contains(request_id)
    }
    pub fn count(&self) -> usize {
        self.requests.count_ones(..)
    }
    pub fn clear(&mut self) {
        self.requests.clear()
    }
    pub fn set_all(&mut self) {
        self.requests.insert_range(..)
    }
    pub fn is_subset(&self, other: &RequestBank) -> bool {
        self.requests.is_subset(&other.requests)
    }
}
