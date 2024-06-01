use crate::problem::pdptw::{Node, PDPTWInstance};
use crate::refn::{ForwardREF, REFData, REFNode};

pub struct REFCalculator<'a> {
    ref_nodes: Vec<REFNode>,
    instance: &'a PDPTWInstance,
}

pub struct REFDataBuilder<'a> {
    calc: &'a REFCalculator<'a>,
    last_node: &'a Node,
    data: REFData,
}

impl<'a> REFDataBuilder<'a> {
    pub fn then(mut self, to: &'a Node) -> Self {
        self.data.extend_forward_assign(
            &self.calc.ref_nodes[to.id],
            self.calc
                .instance
                .distance_and_time(self.last_node.id, to.id),
        );
        self.last_node = to;
        self
    }
    pub fn finished(self) -> REFData {
        self.data
    }
}

impl<'a> REFCalculator<'a> {
    pub fn with_instance(instance: &'a PDPTWInstance) -> Self {
        Self {
            instance,
            ref_nodes: instance.nodes.iter().map(|n| REFNode::from(n)).collect(),
        }
    }
    pub fn first(&'a self, node: &'a Node) -> REFDataBuilder<'a> {
        REFDataBuilder {
            calc: self,
            last_node: node,
            data: REFData::with_node(&self.ref_nodes[node.id]),
        }
    }
    pub fn for_iter(&'a self, mut iter: impl Iterator<Item=&'a Node>) -> REFData {
        if let Some(node) = iter.next() {
            let mut builder = self.first(node);
            for n in iter {
                builder = builder.then(n);
            }
            builder.finished()
        } else {
            REFData::default()
        }
    }
}

#[cfg(feature = "test-with-sartoriburiol-solutions")]
#[cfg(test)]
mod tests {
    use crate::problem::Num;

    use super::*;

    #[test]
    #[allow(non_snake_case)]
    fn test_with_solutions_N100() -> anyhow::Result<()> {
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

            let calc = REFCalculator::with_instance(&instance);

            let obj: Num = solution
                .routes
                .iter()
                .enumerate()
                .map(|(i, it)| {
                    let mut data_builder = calc.first(&instance.nodes[i * 2]);
                    for u in it {
                        let node = instance.nodes.iter().find(|it| it.oid == *u).unwrap();
                        data_builder = data_builder.then(node);
                    }
                    data_builder = data_builder.then(&instance.nodes[i * 2 + 1]);
                    data_builder.finished().distance
                })
                .sum();

            assert_eq!(Num::from(*ref_obj), obj);
        }

        Ok(())
    }

    #[test]
    #[allow(non_snake_case)]
    fn test_with_solutions_N200() -> anyhow::Result<()> {
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

            let calc = REFCalculator::with_instance(&instance);

            let obj: Num = solution
                .routes
                .iter()
                .enumerate()
                .map(|(i, it)| {
                    let mut data_builder = calc.first(&instance.nodes[i * 2]);
                    for u in it {
                        let node = instance.nodes.iter().find(|it| it.oid == *u).unwrap();
                        data_builder = data_builder.then(node);
                    }
                    data_builder = data_builder.then(&instance.nodes[i * 2 + 1]);
                    data_builder.finished().distance
                })
                .sum();

            assert_eq!(Num::from(*ref_obj), obj);
        }

        Ok(())
    }
}
