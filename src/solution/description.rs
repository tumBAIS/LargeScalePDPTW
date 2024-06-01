use crate::problem::pdptw::PDPTWInstance;
use crate::problem::Num;

#[derive(Clone, Debug)]
pub struct SolutionDescription {
    pub(crate) successors: Vec<usize>,
    pub(crate) vehicles_used: usize,
    pub(crate) unassigned_requests: usize,
    pub(crate) objective: Num,
    pub(crate) total_cost: Num,
}

impl SolutionDescription {
    pub fn to_routes_vec(&self, instance: &PDPTWInstance) -> Vec<Vec<usize>> {
        let num_vehicles = instance.num_vehicles;
        let mut routes = Vec::with_capacity(num_vehicles);
        for v in 0..num_vehicles {
            let mut route = Vec::with_capacity(instance.num_requests * 2);
            let mut node_id = v * 2;
            while node_id != v * 2 + 1 {
                debug_assert_ne!(node_id, self.successors[node_id]);
                route.push(node_id);
                node_id = self.successors[node_id];
            }
            route.push(node_id);
            route.shrink_to_fit();
            routes.push(route);
        }
        routes
    }
    pub fn objective(&self) -> Num {
        self.objective
    }
    pub fn total_cost(&self) -> Num {
        self.total_cost
    }
    pub fn number_of_vehicles_used(&self) -> usize {
        self.vehicles_used
    }
    pub fn number_of_unassigned_requests(&self) -> usize {
        self.unassigned_requests
    }

    pub fn get_summary_string(&self) -> String {
        format!(
            "{}/{}/{}",
            self.number_of_unassigned_requests(),
            self.number_of_vehicles_used(),
            self.objective()
        )
    }
}
