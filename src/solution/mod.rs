use fixedbitset::FixedBitSet;

use crate::io::sintef_solution::SINTEFSolution;
use crate::problem::pdptw::{NodeType, PDPTWInstance, Vehicle};
use crate::problem::Num;
use crate::refn::REFData;
use crate::solution::blocknode::BlockNodes;
pub use crate::solution::description::SolutionDescription;
use crate::solution::misc::REFNodeVec;
use crate::solution::requestbank::RequestBank;

pub mod blocknode;
pub mod datastructure;
mod description;
pub mod balassimonetti;
mod misc;
pub mod permutation;
mod requestbank;

impl REFData {
    pub fn check_feasible(&self, vehicle: &Vehicle) -> bool {
        self.tw_feasible
            && vehicle.check_capacity(self.max_load)
            && self.earliest_completion - self.latest_start <= vehicle.shift_length
    }
}

pub struct Solution<'a> {
    pub(crate) instance: &'a PDPTWInstance,
    pub fw_data: REFNodeVec,
    pub bw_data: REFNodeVec,
    pub blocks: BlockNodes,
    pub empty_route_ids: FixedBitSet,
    pub unassigned_requests: RequestBank<'a>,

    objective: Num,
    max_num_vehicles_available: usize,
    num_requests: usize,
}

impl<'a> Solution<'a> {
    pub fn extract_itinerary_and_data(&self, route_id: usize) -> (Vec<usize>, REFData) {
        let itinerary = self
            .iter_route_by_vn_id(route_id * 2)
            .collect::<Vec<usize>>();
        let data = self.fw_data[(route_id * 2) + 1].data.clone();
        (itinerary, data)
    }

    pub fn new(instance: &'a PDPTWInstance) -> Self {
        let num_requests = instance.num_requests;
        let num_vehicles = instance.num_vehicles;
        debug_assert_eq!(instance.nodes.len(), (num_vehicles + num_requests) * 2);

        let mut empty_route_ids = FixedBitSet::with_capacity(instance.num_vehicles);
        empty_route_ids.insert_range(..);

        Self {
            instance,
            objective: Num::max_value(),
            max_num_vehicles_available: num_vehicles,
            num_requests,

            fw_data: REFNodeVec::with_instance(&instance),
            bw_data: REFNodeVec::with_instance(&instance),
            blocks: BlockNodes::with_instance(&instance),
            empty_route_ids,
            unassigned_requests: RequestBank::with_instance(&instance),
        }
    }

    pub fn set_with(&mut self, desc: &SolutionDescription) {
        self.set(&desc.to_routes_vec(self.instance));
    }

    pub fn objective(&self) -> Num {
        let mut objective = self.unassigned_requests.penalty_per_entry
            * Num::from(self.unassigned_requests.count());
        objective += self.total_cost();
        objective
    }

    pub fn total_cost(&self) -> Num {
        let mut cost = Num::ZERO;
        for i in 0..self.instance.num_vehicles {
            cost += self.fw_data[(i * 2) + 1].data.distance;
        }
        cost
    }

    pub fn total_waiting_time(&self) -> Num {
        let mut waiting_time = Num::ZERO;
        for i in 0..self.instance.num_vehicles {
            waiting_time += Num::ZERO.max(
                self.fw_data[(i * 2) + 1].data.duration() - self.fw_data[(i * 2) + 1].data.time,
            );
        }
        waiting_time
    }

    pub fn is_feasible(&self) -> bool {
        for i in 0..self.instance.num_vehicles {
            if !self.fw_data[(i * 2) + 1].data.tw_feasible {
                dbg!(&self.fw_data[(i * 2) + 1].data);
                return false;
            }
        }
        true
    }
}

#[derive(Copy, Clone, Debug)]
pub struct PDExchange {
    pub(crate) vn1_id: usize,
    pub(crate) vn2_id: usize,
    pub(crate) p1_id: usize,
    pub(crate) p2_id: usize,
}

#[derive(Copy, Clone, Debug)]
pub struct PDReplacement {
    pub(crate) vn_id: usize,
    pub(crate) pickup_id: usize,
    pub(crate) replaced_pickup: usize,
}

#[derive(Copy, Clone, Debug)]
pub struct PDInsertion {
    pub(crate) vn_id: usize,
    pub(crate) pickup_id: usize,
    pub(crate) pickup_after: usize,
    pub(crate) delivery_before: usize,
}

pub enum PDInsertionCheckResult {
    Feasible(PDInsertion, Num),
    Infeasible,
}

impl PDInsertionCheckResult {
    fn is_feasible(&self) -> bool {
        match self {
            Self::Feasible(_, _) => true,
            _ => false,
        }
    }
    fn is_infeasible(&self) -> bool {
        !self.is_feasible()
    }
}

impl<'a> Solution<'a> {
    fn check_feasible(&mut self) -> bool {
        (0..self.instance.num_vehicles).all(|i| self.is_route_feasible(i))
    }

    fn check_feasibility(&self, data: &REFData, vn_id: usize) -> bool {
        data.check_feasible(&self.instance.vehicle_from_vn_id(vn_id))
    }

    pub fn check_precedence(&mut self, vn_id: usize) -> bool {
        let mut prev = vn_id;
        let mut open_pickups = FixedBitSet::with_capacity(self.instance.nodes.len());
        loop {
            let next = self.succ(prev);
            if next == vn_id + 1 {
                break;
            }
            match self.instance.node_type(next) {
                NodeType::Pickup => open_pickups.insert(next),
                NodeType::Delivery => {
                    if !open_pickups.contains(next - 1) {
                        return false;
                    }
                    open_pickups.set(next - 1, false);
                }
                _ => {}
            }
            prev = next;
        }
        open_pickups.count_ones(..) == 0
    }
}

impl Solution<'_> {
    pub fn to_description(&self) -> SolutionDescription {
        SolutionDescription {
            successors: self
                .fw_data
                .iter()
                .map(|it| it.succ)
                .collect::<Vec<usize>>(),
            objective: self.objective(),
            total_cost: self.total_cost(),
            vehicles_used: self.number_of_vehicles_used(),
            unassigned_requests: self.number_of_unassigned_requests(),
        }
    }
}

pub fn create_solution_from_sintef(solution: SINTEFSolution, instance: &PDPTWInstance) -> Solution {
    let mut sol = Solution::new(&instance);
    sol.set(
        &solution
            .routes
            .iter()
            .enumerate()
            .map(|(i, it)| {
                let mut r = Vec::with_capacity(it.len());
                r.push(instance.nodes[i * 2].id);
                for u in it {
                    let i = instance
                        .nodes
                        .iter()
                        .find(|it| instance.is_request(it.id) && it.oid == *u)
                        .unwrap();
                    r.push(i.id)
                }
                r.push(instance.nodes[(i * 2) + 1].id);
                r
            })
            .collect::<Vec<Vec<usize>>>(),
    );
    sol
}


#[derive(Debug)]
pub enum BestInsertion {
    Some(PDInsertion, Num),
    None,
}

impl BestInsertion {
    pub fn is_none(&self) -> bool {
        match self {
            Self::None => true,
            _ => false,
        }
    }
    pub fn is_some(&self) -> bool {
        !self.is_none()
    }

    pub fn replace_if_better(&mut self, other: BestInsertion) {
        match (&self, other) {
            (BestInsertion::None, x) => *self = x,
            (BestInsertion::Some(_, cost_a), BestInsertion::Some(ins_b, cost_b)) => {
                if cost_b < *cost_a {
                    *self = BestInsertion::Some(ins_b, cost_b)
                }
            }
            _ => {}
        }
    }
}


#[cfg(feature = "classic-pdptw")]
#[cfg(feature = "test-with-sartoriburiol-solutions")]
#[cfg(test)]
pub mod tests {
    use crate::problem::Num;
    use crate::solution::create_solution_from_sintef;

    #[test]
    fn test_with_solutions_n100() -> anyhow::Result<()> {
        use crate::io::sintef_solution::tests::sartori_buriol::INSTANCE_DIR_N100;
        use crate::io::sintef_solution::tests::sartori_buriol::N100;
        use crate::io::sintef_solution::tests::sartori_buriol::SOLUTION_DIR_N100;

        for (instance_name, solution_name, ref_vehicles, ref_obj) in N100.iter() {
            let instance_path = format!("{}/{}", INSTANCE_DIR_N100, instance_name);
            let solution_path = format!("{}/{}", SOLUTION_DIR_N100, solution_name);

            let instance = crate::io::sartori_buriol_reader::load_instance(
                instance_path,
                Some(*ref_vehicles),
            )?;
            let solution = crate::io::sintef_solution::load_sintef_solution(solution_path)?;

            let sol = create_solution_from_sintef(solution, &instance);

            assert_eq!(Num::from(*ref_obj), sol.objective());
        }

        Ok(())
    }

    #[test]
    fn test_with_solutions_n200() -> anyhow::Result<()> {
        use crate::io::sintef_solution::tests::sartori_buriol::INSTANCE_DIR_N200;
        use crate::io::sintef_solution::tests::sartori_buriol::N200;
        use crate::io::sintef_solution::tests::sartori_buriol::SOLUTION_DIR_N200;

        for (instance_name, solution_name, ref_vehicles, ref_obj) in N200.iter() {
            let instance_path = format!("{}/{}", INSTANCE_DIR_N200, instance_name);
            let solution_path = format!("{}/{}", SOLUTION_DIR_N200, solution_name);

            let instance = crate::io::sartori_buriol_reader::load_instance(
                instance_path,
                Some(*ref_vehicles),
            )?;
            let solution = crate::io::sintef_solution::load_sintef_solution(solution_path)?;

            let sol = create_solution_from_sintef(solution, &instance);

            assert_eq!(Num::from(*ref_obj), sol.objective());
        }

        Ok(())
    }

    #[test]
    fn test_with_solutions_n400() -> anyhow::Result<()> {
        use crate::io::sintef_solution::tests::sartori_buriol::INSTANCE_DIR_N400;
        use crate::io::sintef_solution::tests::sartori_buriol::N400;
        use crate::io::sintef_solution::tests::sartori_buriol::SOLUTION_DIR_N400;

        for (instance_name, solution_name, ref_vehicles, ref_obj) in N400.iter() {
            let instance_path = format!("{}/{}", INSTANCE_DIR_N400, instance_name);
            let solution_path = format!("{}/{}", SOLUTION_DIR_N400, solution_name);

            let instance = crate::io::sartori_buriol_reader::load_instance(
                instance_path,
                Some(*ref_vehicles),
            )?;
            let solution = crate::io::sintef_solution::load_sintef_solution(solution_path)?;

            let sol = create_solution_from_sintef(solution, &instance);

            assert_eq!(Num::from(*ref_obj), sol.objective());
        }

        Ok(())
    }

    #[test]
    fn test_with_solutions_n1000() -> anyhow::Result<()> {
        use crate::io::sintef_solution::tests::sartori_buriol::INSTANCE_DIR_N1000;
        use crate::io::sintef_solution::tests::sartori_buriol::N1000;
        use crate::io::sintef_solution::tests::sartori_buriol::SOLUTION_DIR_N1000;

        for (instance_name, solution_name, ref_vehicles, ref_obj) in N1000.iter() {
            let instance_path = format!("{}/{}", INSTANCE_DIR_N1000, instance_name);
            let solution_path = format!("{}/{}", SOLUTION_DIR_N1000, solution_name);

            let instance = crate::io::sartori_buriol_reader::load_instance(
                instance_path,
                Some(*ref_vehicles),
            )?;
            let solution = crate::io::sintef_solution::load_sintef_solution(solution_path)?;

            let sol = create_solution_from_sintef(solution, &instance);

            assert_eq!(Num::from(*ref_obj), sol.objective());
        }

        Ok(())
    }
}
