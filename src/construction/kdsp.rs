use clap::ValueEnum;
use fixedbitset::FixedBitSet;
use kdsp::{Arc, Vertex};
use kdsp::node_disjoint::{SourceSinkGraph, SourceSinkGraphBuilder};
use log::warn;

#[cfg(feature = "search_assertions")]
use crate::assert_valid_solution;
use crate::clustering::create_block_from_single_request;
use crate::construction::insertion::ParallelInsertion;
use crate::io::kdsp_writer::write_instance;
use crate::problem::Num;
use crate::problem::pdptw::PDPTWInstance;
use crate::problem::travel_matrix::TravelMatrix;
use crate::refn::{ForwardREF, REFData};
use crate::solution::{Solution, SolutionDescription};
use crate::solution::blocknode::BlockNode;
use crate::utils::Random;
use crate::utils::refn::REFCalculator;
use crate::utils::suurballe;

type EdgeWeight = i64;

pub struct ClusterKDSP<'a> {
    pub instance: &'a PDPTWInstance,
}

#[derive(Default, Clone, Debug)]
pub struct Blocks {
    pub block: BlockNode,
    pub path: Vec<usize>,
    pub earliest_start: Num,
    pub latest_start: Num,
}

#[derive(Copy, Clone, Debug)]
pub struct KDSPSettings {
    pub block_mode: KDSPBlockMode,
    pub maximum_distance_to_next_request: Option<Num>,
    pub maximum_time_to_next_request: Option<Num>,
}

#[derive(Copy, Clone, Debug, ValueEnum)]
pub enum KDSPBlockMode {
    Earliest,
    Latest,
    Average,
}

impl<'a> ClusterKDSP<'a> {
    pub fn new(instance: &'a PDPTWInstance) -> Self {
        Self { instance }
    }

    fn extract_graph_properties_from_blocks(
        &self,
        clusters: &mut Vec<Blocks>,
        maximum_distance_to_next_block: Option<Num>,
        maximum_time_to_next_block: Option<Num>,
    ) -> (Vec<Arc<EdgeWeight>>, usize, usize) {
        clusters.sort_by_cached_key(|c| c.earliest_start);

        let maximum_distance_to_next_block = maximum_distance_to_next_block.unwrap_or(Num::MAX);
        let maximum_time_to_next_block = maximum_time_to_next_block.unwrap_or(Num::MAX);

        let profit_per_node = Num::from(self.instance.travel_matrix.max_distance()) + Num::ONE;
        let cost_per_vehicle = Num::ZERO;

        let mut arcs = vec![];
        let offset_vehicles = 2;
        let offset_blocks = offset_vehicles + self.instance.num_vehicles * 2;
        for v in 0..self.instance.num_vehicles {
            arcs.push(Arc {
                from: 0,
                to: (v * 2 + offset_vehicles) as Vertex,
                w: Num::ZERO.value() as EdgeWeight,
            });
            // allow vehicles not being used
            arcs.push(Arc {
                from: (v * 2 + offset_vehicles) as Vertex,
                to: 1 as Vertex,
                w: Num::ZERO.value() as EdgeWeight,
            });
        }
        for (idx, b1) in clusters.iter().enumerate() {
            for v in 0..self.instance.num_vehicles {
                // check if vehicle v can reach the block in time
                let vn_id = self.instance.vn_id_of(v);
                let dist_time_vn_b1 = self
                    .instance
                    .distance_and_time(vn_id, b1.block.first_node_id);

                // spatial cut
                if dist_time_vn_b1.distance > maximum_distance_to_next_block {
                    continue;
                }

                if self.instance.nodes[vn_id].ready + dist_time_vn_b1.time <= b1.latest_start {
                    // temporal cut
                    if b1.earliest_start - self.instance.nodes[vn_id].ready
                        <= maximum_time_to_next_block
                    {
                        arcs.push(Arc {
                            from: (v * 2 + offset_vehicles) as Vertex,
                            to: (idx + offset_blocks) as Vertex,
                            w: (cost_per_vehicle + dist_time_vn_b1.distance
                                - (profit_per_node * Num::from(b1.path.len())))
                                .value() as EdgeWeight,
                        });
                    }
                }
            }

            for (idx2, b2) in clusters.iter().enumerate() {
                if idx >= idx2 {
                    continue;
                }
                let dist_time_b1_b2 = self
                    .instance
                    .distance_and_time(b1.block.last_node_id, b2.block.first_node_id);

                // spatial cut
                if dist_time_b1_b2.distance > maximum_distance_to_next_block {
                    continue;
                }

                let earliest_departure_from_b1 = b1.earliest_start + b1.block.data.duration();
                let earliest_arrival_at_b2 = earliest_departure_from_b1 + dist_time_b1_b2.time;
                let earliest_start_b2 = b2.latest_start.min(b2.earliest_start);

                if earliest_arrival_at_b2 > earliest_start_b2 {
                    continue;
                }

                // temporal cut
                if earliest_start_b2 - earliest_departure_from_b1 > maximum_time_to_next_block {
                    continue;
                }

                arcs.push(Arc {
                    from: (idx + offset_blocks) as Vertex,
                    to: (idx2 + offset_blocks) as Vertex,
                    w: (self
                        .instance
                        .distance(b1.block.last_node_id, b2.block.first_node_id)
                        - (profit_per_node * Num::from(b2.path.len())))
                        .value() as EdgeWeight,
                });
            }

            let distance_time = self.instance.distance_and_time(b1.block.last_node_id, 1);
            if b1.latest_start + b1.block.data.duration() + distance_time.time
                > self.instance.nodes[1].due
            {
                warn!(
                    "exceeding route duration! {:?} {} {}",
                    b1, distance_time.time, self.instance.nodes[1].due
                );
            } else {
                arcs.push(Arc {
                    from: (idx + offset_blocks) as Vertex,
                    to: 1 as Vertex,
                    w: self.instance.distance(b1.block.last_node_id, 1).value() as EdgeWeight,
                });
            }
        }

        (arcs, offset_vehicles, offset_blocks)
    }

    pub fn construct_with_single_request_blocks<'b>(&self) -> Solution<'b>
        where
            'a: 'b,
    {
        let solution = Solution::new(self.instance);
        self.construct_with_blocks(
            solution
                .unassigned_requests
                .iter_pickup_ids()
                .map(|p_id| {
                    let block = BlockNode::new(
                        p_id,
                        p_id + 1,
                        REFData::with_node(solution.node(p_id)).extend_forward(
                            solution.node(p_id + 1),
                            self.instance.distance_and_time(p_id, p_id + 1),
                        ),
                    );
                    Blocks {
                        earliest_start: block.data.earliest_start(),
                        latest_start: block.data.latest_start,
                        block,
                        path: vec![p_id, p_id + 1],
                    }
                })
                .collect(),
        )
    }

    pub fn construct_with_blocks<'b>(&self, mut clusters: Vec<Blocks>) -> Solution<'b>
        where
            'a: 'b,
    {
        let (arcs, offset_vehicles, offset_blocks) =
            self.extract_graph_properties_from_blocks(&mut clusters, None, None);

        let graph: SourceSinkGraph<EdgeWeight> = SourceSinkGraphBuilder::new()
            .set_num_nodes((clusters.len() + offset_blocks) as Vertex)
            .set_source(0)
            .set_sink(1)
            .add_arcs(arcs)
            .build();

        let (_, sp_arcs) = suurballe::solve(graph, Some(self.instance.num_vehicles), None);

        let mut successors = vec![0; clusters.len() + offset_blocks];
        for arc in sp_arcs.iter() {
            successors[arc.0 as usize] = arc.1;
        }
        let mut routes = vec![];
        for v_id in 0..self.instance.num_vehicles {
            let vn = offset_vehicles + v_id * 2;
            let succ = successors[vn];
            debug_assert!(sp_arcs
                .iter()
                .any(|(from, to)| *from == vn as Vertex && *to == succ));
            let mut route = vec![v_id * 2];
            let mut block = succ;

            while block >= (offset_blocks as Vertex) {
                route.extend(clusters[(block as usize) - offset_blocks].path.iter());
                let tmp = block;
                block = successors[block as usize];
                debug_assert!(sp_arcs
                    .iter()
                    .any(|(from, to)| *from == tmp && *to == block));
            }
            route.push(v_id * 2 + 1);
            routes.push(route);
        }

        let mut solution = Solution::new(self.instance);
        solution.set(&routes);

        solution
    }

    pub fn construct_with_blocks_and_vehicle_limit(
        &self,
        mut clusters: Vec<Blocks>,
        vehicle_limit: Option<usize>,
        maximum_distance_to_next_block: Option<Num>,
        maximum_time_to_next_block: Option<Num>,
    ) -> Solution {
        let (arcs, offset_vehicles, offset_blocks) = self.extract_graph_properties_from_blocks(
            &mut clusters,
            maximum_distance_to_next_block,
            maximum_time_to_next_block,
        );

        let graph: SourceSinkGraph<EdgeWeight> = SourceSinkGraphBuilder::new()
            .set_num_nodes((clusters.len() + offset_blocks) as Vertex)
            .set_source(0)
            .set_sink(1)
            .add_arcs(arcs)
            .build();

        let mut nodes_to_visit = FixedBitSet::with_capacity(graph.num_nodes() as usize);
        nodes_to_visit.insert_range(offset_blocks..);
        let (_, sp_arcs) = suurballe::solve(graph, vehicle_limit, Some(nodes_to_visit));

        let mut successors = vec![0; clusters.len() + offset_blocks];
        for arc in sp_arcs.iter() {
            successors[arc.0 as usize] = arc.1;
        }
        let mut routes = vec![];
        for v_id in 0..self.instance.num_vehicles {
            let vn = offset_vehicles + v_id * 2;
            let succ = successors[vn];
            let mut route = vec![v_id * 2];
            let mut block = succ;

            while block >= (offset_blocks as Vertex) {
                route.extend(clusters[(block as usize) - offset_blocks].path.iter());
                let tmp = block;
                block = successors[block as usize];
                debug_assert!(
                    sp_arcs
                        .iter()
                        .any(|(from, to)| *from == tmp && *to == block),
                    "{} -> {} not in sp_arcs",
                    vn,
                    succ
                );
            }
            route.push(v_id * 2 + 1);
            routes.push(route);
        }

        let mut solution = Solution::new(self.instance);
        solution.set(&routes);

        solution
    }

    pub fn construct(&self, rng: &mut Random) -> Solution {
        let blocks = self.create_clusters(rng);
        self.construct_with_blocks(blocks)
    }

    fn create_clusters(&self, rng: &mut Random) -> Vec<Blocks> {
        let ins = ParallelInsertion::new(self.instance);
        let sol = ins.construct(rng);

        self.extract_simple_blocks_from_solution_description(sol.to_description())
    }

    // extracts blocks with the earliest/latest starting time possible
    pub fn extract_simple_blocks_from_solution_description(
        &self,
        s: SolutionDescription,
    ) -> Vec<Blocks> {
        self.extract_simple_blocks_directly_from_solution(s)
    }

    fn extract_simple_blocks_directly_from_solution(&self, s: SolutionDescription) -> Vec<Blocks> {
        let mut solution = Solution::new(self.instance);
        solution.set(&s.to_routes_vec(self.instance));

        let mut blocks = vec![];
        for i in 0..self.instance.num_vehicles {
            let vn = i * 2;

            let mut block_start: usize = vn;
            let mut vec = vec![];
            let mut node = vn;
            while node != vn + 1 {
                node = solution.succ(node);
                if node == vn + 1 || solution.blocks.is_block_start(node) {
                    if block_start != vn {
                        debug_assert!(vec.len() % 2 == 0, "{:?}", vec);
                        blocks.push(Blocks {
                            block: solution.blocks[block_start].clone(),
                            path: vec,
                            earliest_start: solution.blocks[block_start].data.earliest_completion
                                - solution.blocks[block_start].data.duration(),
                            latest_start: solution.blocks[block_start].data.latest_start,
                        });
                    }
                    if node == vn + 1 {
                        break;
                    }
                    // otherwise begin a new block
                    block_start = node;
                    vec = vec![node];
                } else {
                    vec.push(node);
                }
            }
        }

        blocks.append(&mut self.extract_unassigned_requests_as_blocks(&solution));

        blocks
    }

    pub fn extract_tightened_blocks_from_solution(
        &self,
        s: &SolutionDescription,
        block_mode: KDSPBlockMode,
    ) -> Vec<Blocks> {
        self.experimental_extract_tightened_blocks_directly_from_solution(s, block_mode)
    }

    pub fn extract_tightened_blocks_directly_from_solution(
        &self,
        sol: &SolutionDescription,
    ) -> Vec<Blocks> {
        let mut solution = Solution::new(self.instance);
        solution.set(&sol.to_routes_vec(self.instance));

        #[cfg(feature = "search_assertions")]
        assert_valid_solution(self.instance, &solution);

        let mut blocks = vec![];
        for i in 0..self.instance.num_vehicles {
            let vn = i * 2;

            let mut block_start: usize = vn;
            let mut vec = vec![];
            let mut node = vn;
            while node != vn + 1 {
                node = solution.succ(node);
                if node == vn + 1 || solution.blocks.is_block_start(node) {
                    if block_start != vn {
                        debug_assert!(vec.len() % 2 == 0, "{:?}", vec);

                        let tightest_earliest_start =
                            solution.blocks[block_start].data.earliest_start().max(
                                solution.fw_data[block_start].data.earliest_completion
                                    - self.instance.nodes[block_start].servicetime,
                            );

                        // extract block
                        blocks.push(Blocks {
                            block: solution.blocks[block_start].clone(),
                            path: vec,
                            earliest_start: tightest_earliest_start,
                            latest_start: tightest_earliest_start,
                        });
                    }
                    if node == vn + 1 {
                        break;
                    }
                    // otherwise begin a new block
                    block_start = node;
                    vec = vec![node];
                } else {
                    vec.push(node);
                }
            }
        }

        blocks.append(&mut self.extract_unassigned_requests_as_blocks(&solution));

        blocks
    }

    pub fn experimental_extract_tightened_blocks_directly_from_solution(
        &self,
        sol: &SolutionDescription,
        block_mode: KDSPBlockMode,
    ) -> Vec<Blocks> {
        let mut solution = Solution::new(self.instance);
        solution.set(&sol.to_routes_vec(self.instance));

        #[cfg(feature = "search_assertions")]
        assert_valid_solution(self.instance, &solution);

        let mut blocks = vec![];
        for i in 0..self.instance.num_vehicles {
            let vn = i * 2;

            let mut block_start: usize = vn;
            let mut vec = vec![];
            let mut node = vn;
            while node != vn + 1 {
                node = solution.succ(node);
                if node == vn + 1 || solution.blocks.is_block_start(node) {
                    if block_start != vn {
                        debug_assert!(vec.len() % 2 == 0, "{:?}", vec);

                        let tightest_earliest_start =
                            solution.blocks[block_start].data.earliest_start().max(
                                solution.fw_data[block_start].data.earliest_completion
                                    - self.instance.nodes[block_start].servicetime,
                            );

                        // extract block
                        blocks.push(Blocks {
                            block: solution.blocks[block_start].clone(),
                            path: vec,
                            earliest_start: tightest_earliest_start,
                            latest_start: solution.bw_data[block_start].data.latest_start(),
                        });
                    }
                    if node == vn + 1 {
                        break;
                    }
                    // otherwise begin a new block
                    block_start = node;
                    vec = vec![node];
                } else {
                    vec.push(node);
                }
            }
        }

        blocks.append(&mut self.extract_unassigned_requests_as_blocks(&solution));

        blocks.iter_mut().for_each(|b| {
            let value = match block_mode {
                KDSPBlockMode::Earliest => b.earliest_start,
                KDSPBlockMode::Latest => b.latest_start,
                KDSPBlockMode::Average => {
                    (b.latest_start - b.earliest_start) / Num::TWO + b.earliest_start
                }
            };
            b.earliest_start = value;
            b.latest_start = value;
        });

        blocks
    }

    fn extract_unassigned_requests_as_blocks(&self, solution: &Solution) -> Vec<Blocks> {
        let calc = REFCalculator::with_instance(self.instance);
        solution
            .unassigned_requests
            .iter_pickup_ids()
            .map(|pickup_id| create_block_from_single_request(pickup_id, &calc, self.instance))
            .collect()
    }

    pub fn store_kdsp_graph(
        &'a self,
        mut clusters: Vec<Blocks>,
        name: impl Into<String>,
        path: impl Into<String>,
        solution: Option<(&'a PDPTWInstance, SolutionDescription)>,
    ) -> anyhow::Result<()> {
        let (arcs, offset_vehicles, offset_blocks) =
            self.extract_graph_properties_from_blocks(&mut clusters, None, None);

        let kdsp = solution.map(|(instance, sol)| {
            let mut paths = vec![];
            for v in sol.to_routes_vec(instance) {
                if v.len() > 2 {
                    let mut path = vec![0, v[0] + offset_vehicles];
                    let mut i = 1;
                    while i < v.len() - 1 {
                        let node = v[i];
                        // find corresponding block
                        for (idx, it) in clusters.iter().enumerate() {
                            if it.path[0] == node {
                                // found the block - add node to the path
                                path.push(idx + offset_blocks);
                                i += it.path.len();
                                break;
                            }
                        }
                    }
                    // not necessary, we do not consider multi-depot
                    path.push(1);
                    paths.push(path);
                }
            }
            paths
        });

        write_instance(path, name, offset_blocks + clusters.len(), arcs, kdsp)?;
        Ok(())
    }
}
