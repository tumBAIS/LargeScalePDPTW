use crate::problem::pdptw::PDPTWInstance;
use crate::solution::balassimonetti::BalasSimonetti;
use crate::Solution;

pub struct BalasSimonettiLS<'a> {
    graph: BalasSimonetti<'a>,
}

impl<'a> BalasSimonettiLS<'a> {
    pub fn new(instance: &'a PDPTWInstance, max_len: usize, thickness: usize) -> Self {
        Self {
            graph: BalasSimonetti::new(instance, max_len, thickness),
        }
    }

    pub fn improve(&mut self, sol: &mut Solution) -> bool {
        let r_ids: Vec<_> = sol.iter_route_ids().collect();
        let mut some_route_changed = false;
        for r_id in r_ids {
            if !sol.is_route_empty(r_id) {
                some_route_changed |= self.graph.improve(sol, r_id, 4);
            }
        }
        some_route_changed
    }
}

#[cfg(feature = "classic-pdptw")]
#[cfg(test)]
mod tests {
    use crate::io::sartori_buriol_reader::load_instance;
    use crate::io::sintef_solution::tests::sartori_buriol::INSTANCE_DIR_N1000;

    use super::*;

    #[test]
    fn test_with_instance() -> anyhow::Result<()> {
        let instance_path = format!("{}/{}", INSTANCE_DIR_N1000, "nyc-n1000-1.txt");
        let instance = load_instance(instance_path, None)?;

        let mut sol = Solution::new(&instance);
        let routes = vec![vec![388, 1204, 1286, 1287, 548, 902, 903, 1205, 549, 389]];
        sol.set(&routes);

        let mut balas_simonetti_ls =
            BalasSimonettiLS::new(&instance, instance.num_requests * 2 + 2, 4);
        assert!(balas_simonetti_ls.improve(&mut sol));
        Ok(())
    }
}
