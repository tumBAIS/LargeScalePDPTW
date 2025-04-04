use fixedbitset::FixedBitSet;
use rand::Rng;
use rand::seq::{IteratorRandom, SliceRandom};
use std::cmp::Reverse;
use std::collections::HashMap;
use std::ops::RangeInclusive;

#[cfg(feature = "search_assertions")]
use crate::assert_valid_solution;
use crate::construction::kdsp::{Blocks, ClusterKDSP, KDSPBlockMode, KDSPSettings};
use crate::problem::Num;
use crate::problem::pdptw::{Node, NodeType, PDPTWInstance};
use crate::solution::{Solution, SolutionDescription};
use crate::utils::Random;
#[cfg(feature = "relatedness-measures")]
use crate::utils::relatedness::RelatednessMeasures;

pub mod ages;

pub(crate) struct PartialInstanceStruct<'a> {
    pub(crate) full: &'a PDPTWInstance,
    pub(crate) partial_to_full_mapping: Vec<usize>,
    pub(crate) partial: PDPTWInstance,
}

#[derive(Copy, Clone, Debug)]
pub enum RecombineMode {
    Naive,
    KDSP(KDSPSettings),
}

fn recombine_solution(
    recombine_mode: RecombineMode,
    instance: &PDPTWInstance,
    parts: Vec<
        Option<(
            PartialInstanceStruct,
            SolutionDescription,
            SolutionDescription,
        )>,
    >,
    vehicle_limit: Option<usize>,
    vehicles_limit_lower_relative_to_best: usize,
) -> (SolutionDescription, SolutionDescription, usize) {
    match recombine_mode {
        RecombineMode::Naive => {
            let (a, b) = recombine_solution_naive(
                instance,
                &parts,
                vehicle_limit,
                vehicles_limit_lower_relative_to_best,
            );
            (a, b, 0)
        }
        RecombineMode::KDSP(settings) => {
            recombine_solution_kdsp(
                instance,
                &parts,
                vehicle_limit,
                vehicles_limit_lower_relative_to_best,
                settings,
            )
        }
    }
}

fn recombine_solution_kdsp(
    instance: &PDPTWInstance,
    parts: &Vec<
        Option<(
            PartialInstanceStruct,
            SolutionDescription,
            SolutionDescription,
        )>,
    >,
    vehicle_limit: Option<usize>,
    vehicles_limit_lower_relative_to_best: usize,
    kdsp_settings: KDSPSettings,
) -> (SolutionDescription, SolutionDescription, usize) {
    let best_blocks = parts
        .iter()
        .filter_map(|maybe_part| {
            maybe_part
                .as_ref()
                .map(|part| extract_blocks_from_part(&part.0, &part.1, kdsp_settings.block_mode))
        })
        .flatten()
        .collect::<Vec<Blocks>>();

    let num_best_blocks = best_blocks.len();
    let kdsp = ClusterKDSP::new(instance);
    let best_sol = kdsp.construct_with_blocks_and_vehicle_limit(
        best_blocks,
        vehicle_limit.or(Some(instance.num_vehicles)),
        kdsp_settings.maximum_distance_to_next_request,
        kdsp_settings.maximum_time_to_next_request,
    );

    #[cfg(feature = "search_assertions")]
    assert_valid_solution(instance, &best_sol);

    let current_blocks = parts
        .iter()
        .filter_map(|maybe_part| {
            maybe_part
                .as_ref()
                .map(|part| extract_blocks_from_part(&part.0, &part.2, kdsp_settings.block_mode))
        })
        .flatten()
        .collect::<Vec<Blocks>>();

    let current_sol = kdsp.construct_with_blocks_and_vehicle_limit(
        current_blocks,
        Some(best_sol.number_of_vehicles_used() - vehicles_limit_lower_relative_to_best),
        kdsp_settings.maximum_distance_to_next_request,
        kdsp_settings.maximum_time_to_next_request,
    );

    #[cfg(feature = "search_assertions")]
    assert_valid_solution(instance, &current_sol);

    (
        best_sol.to_description(),
        current_sol.to_description(),
        num_best_blocks,
    )
}

fn recombine_solution_naive(
    instance: &PDPTWInstance,
    parts: &Vec<
        Option<(
            PartialInstanceStruct,
            SolutionDescription,
            SolutionDescription,
        )>,
    >,
    _vehicle_limit: Option<usize>,
    _vehicles_limit_lower_relative_to_best: usize,
) -> (SolutionDescription, SolutionDescription) {
    fn map_routes(
        partial_instance_struct: &PartialInstanceStruct,
        partial_solution: &SolutionDescription,
    ) -> Vec<Vec<usize>> {
        let mut routes = partial_solution.to_routes_vec(&partial_instance_struct.partial);
        for r in &mut routes {
            for i in 0..r.len() {
                r[i] = partial_instance_struct.partial_to_full_mapping[r[i]];
            }
        }
        routes
    }

    let best_sol = {
        let routes: Vec<Vec<usize>> = parts
            .iter()
            .filter_map(|it| if let Some(x) = it { Some(x) } else { None })
            .map(|it| map_routes(&it.0, &it.1))
            .flatten()
            .collect();
        let mut sol = Solution::new(instance);
        sol.set(&routes);
        sol
    };

    #[cfg(feature = "search_assertions")]
    assert_valid_solution(instance, &best_sol);

    let current_sol = {
        let routes: Vec<Vec<usize>> = parts
            .iter()
            .filter_map(|it| if let Some(x) = it { Some(x) } else { None })
            .map(|it| map_routes(&it.0, &it.2))
            .flatten()
            .collect();
        let mut sol = Solution::new(instance);
        sol.set(&routes);
        sol
    };

    #[cfg(feature = "search_assertions")]
    assert_valid_solution(instance, &current_sol);

    (best_sol.to_description(), current_sol.to_description())
}

fn extract_blocks_from_part(
    part: &PartialInstanceStruct,
    desc: &SolutionDescription,
    block_mode: KDSPBlockMode,
) -> Vec<Blocks> {
    let kdsp = ClusterKDSP::new(&part.partial);
    let mut blocks = kdsp.extract_tightened_blocks_from_solution(desc, block_mode);
    for block in &mut blocks {
        block.block.first_node_id = part.partial_to_full_mapping[block.block.first_node_id];
        block.block.last_node_id = part.partial_to_full_mapping[block.block.last_node_id];
        for i in 0..block.path.len() {
            block.path[i] = part.partial_to_full_mapping[block.path[i]];
        }
    }
    blocks
}

#[derive(Copy, Clone, Debug)]
pub enum GroupMatchingMode {
    Random,
    #[cfg(feature = "relatedness-measures")]
    ByRequestRelatedness,
}

#[derive(Copy, Clone, Debug)]
pub enum UnassignedMatchingMode {
    Random,
    TimeWindows,
    #[cfg(feature = "relatedness-measures")]
    ByLinkHistory,
    #[cfg(feature = "relatedness-measures")]
    ByRouteHistory,
    #[cfg(feature = "relatedness-measures")]
    ByLinkAndRouteHistory {
        link_weight: f64,
        route_weight: f64,
    },
}

#[derive(Copy, Clone, Debug)]
pub struct SplitSettings {
    pub group_matching_mode: GroupMatchingMode,
    pub unassigned_matching_mode: UnassignedMatchingMode,
}

fn split_solution<'a>(
    instance: &'a PDPTWInstance,
    solution: &Solution,
    avg_nodes_per_route: usize,
    rng: &mut Random,
    settings: &SplitSettings,
    #[cfg(feature = "relatedness-measures")] measures: &RelatednessMeasures,
) -> Vec<(PartialInstanceStruct<'a>, Vec<Vec<usize>>)> {
    split_only_routes(
        instance,
        solution,
        avg_nodes_per_route,
        rng,
        settings,
        #[cfg(feature = "relatedness-measures")]
            measures,
    )
}

fn split_time<'a>(
    instance: &'a PDPTWInstance,
    solution: &Solution,
    rng: &mut Random,
    split_settings: &SplitSettings,
    #[cfg(feature = "relatedness-measures")] measures: &RelatednessMeasures,
) -> Vec<(PartialInstanceStruct<'a>, Vec<Vec<usize>>)> {
    // select pivot route
    let mut route_ids = solution
        .iter_route_ids()
        .filter(|it| !solution.is_route_empty(*it))
        .collect::<Vec<usize>>();
    route_ids.shuffle(rng);

    let pivot_route = loop {
        if let Some(t) = route_ids.pop() {
            break t;
        } else {
            break solution
                .iter_route_ids()
                .filter(|it| !solution.is_route_empty(*it))
                .choose(rng)
                .unwrap();
        }
    };

    // determine a single, balanced split point
    let start = solution.fw_data[solution.succ(instance.vn_id_of(pivot_route) + 1)]
        .data
        .earliest_start();
    let end = solution.fw_data[solution.pred(instance.vn_id_of(pivot_route) + 1)]
        .data
        .latest_start();
    let duration = solution.fw_data[solution.pred(instance.vn_id_of(pivot_route) + 1)]
        .data
        .duration();
    let midpoint = (end - start + duration) / Num::TWO + start;

    let pivot_block_node_start =
        get_nearest_block_start_node(instance, solution, pivot_route, midpoint);

    let split_before_nodes = solution
        .iter_route_ids()
        .filter(|it| !solution.is_route_empty(*it))
        .map(|r_id| {
            (
                r_id,
                get_nearest_block_start_node(
                    instance,
                    solution,
                    r_id,
                    solution.fw_data[pivot_block_node_start]
                        .data
                        .earliest_start(),
                ),
            )
        })
        .collect::<Vec<(usize, usize)>>();

    // split solution using the identified split points
    let routes_part1 = split_before_nodes
        .iter()
        .map(|(r_id, split_node_id)| {
            let ready = instance.nodes[*r_id * 2].ready;
            let due = (solution.bw_data[*split_node_id].data.latest_start()
                - solution.bw_data[*split_node_id].data.earliest_start())
                + solution.bw_data[*split_node_id].data.earliest_start();
            let sub_sequence = solution
                .iter_route(*r_id)
                .take_while(|it| it != split_node_id)
                .chain([split_node_id.clone()].into_iter())
                .collect::<Vec<usize>>();
            (*r_id, sub_sequence, (ready, due))
        })
        .collect::<Vec<(usize, Vec<usize>, (Num, Num))>>();

    let routes_part2 = split_before_nodes
        .iter()
        .map(|(r_id, split_node_id)| {
            let (vehicle_node, ready) = if *split_node_id == instance.vn_id_of(*r_id) + 1 {
                let pred = solution.pred(*split_node_id);
                (
                    pred,
                    solution.fw_data[pred].data.earliest_completion
                        - instance.nodes[pred].servicetime,
                )
            } else {
                (
                    *split_node_id,
                    solution.fw_data[*split_node_id].data.earliest_completion
                        - instance.nodes[*split_node_id].servicetime,
                )
            };
            let due = instance.nodes[*r_id * 2 + 1].due;
            (
                *r_id,
                [vehicle_node]
                    .into_iter()
                    .chain(
                        solution
                            .iter_route(*r_id)
                            .skip_while(|it| it != split_node_id),
                    )
                    .collect::<Vec<usize>>(),
                (ready, due),
            )
        })
        .collect::<Vec<(usize, Vec<usize>, (Num, Num))>>();

    let mut unassigned_requests = FixedBitSet::with_capacity(instance.num_requests);
    solution
        .unassigned_requests
        .iter_request_ids()
        .for_each(|request_id| unassigned_requests.insert(request_id));

    create_partial_instances_and_mapped_routes(
        instance,
        vec![routes_part1, routes_part2],
        &mut unassigned_requests,
        rng,
        split_settings,
        #[cfg(feature = "relatedness-measures")]
            measures,
    )
}

fn split_time_then_routes<'a>(
    instance: &'a PDPTWInstance,
    solution: &Solution,
    avg_nodes_per_route: usize,
    rng: &mut Random,
    split_settings: &SplitSettings,
    #[cfg(feature = "relatedness-measures")] measures: &RelatednessMeasures,
) -> Vec<(PartialInstanceStruct<'a>, Vec<Vec<usize>>)> {
    // select pivot route
    let mut route_ids = solution
        .iter_route_ids()
        .filter(|it| !solution.is_route_empty(*it))
        .collect::<Vec<usize>>();
    route_ids.shuffle(rng);

    let pivot_route = loop {
        if let Some(t) = route_ids.pop() {
            break t;
        } else {
            break solution
                .iter_route_ids()
                .filter(|it| !solution.is_route_empty(*it))
                .choose(rng)
                .unwrap();
        }
    };

    // determine a single, balanced split point
    let start = solution.fw_data[solution.succ(instance.vn_id_of(pivot_route) + 1)]
        .data
        .earliest_start();
    let end = solution.fw_data[solution.pred(instance.vn_id_of(pivot_route) + 1)]
        .data
        .latest_start();
    let duration = solution.fw_data[solution.pred(instance.vn_id_of(pivot_route) + 1)]
        .data
        .duration();
    let midpoint = (end - start + duration) / Num::TWO + start;

    let pivot_block_node_start =
        get_nearest_block_start_node(instance, solution, pivot_route, midpoint);

    let split_before_nodes = solution
        .iter_route_ids()
        .filter(|it| !solution.is_route_empty(*it))
        .map(|r_id| {
            (
                r_id,
                get_nearest_block_start_node(
                    instance,
                    solution,
                    r_id,
                    solution.fw_data[pivot_block_node_start]
                        .data
                        .earliest_start(),
                ),
            )
        })
        .collect::<Vec<(usize, usize)>>();

    // split solution using the identified split points
    let routes_part1 = split_before_nodes
        .iter()
        .map(|(r_id, split_node_id)| {
            let ready = instance.nodes[*r_id * 2].ready;
            let due = (solution.bw_data[*split_node_id].data.latest_start()
                - solution.bw_data[*split_node_id].data.earliest_start())
                + solution.bw_data[*split_node_id].data.earliest_start();
            let sub_sequence = solution
                .iter_route(*r_id)
                .take_while(|it| it != split_node_id)
                .chain([split_node_id.clone()].into_iter())
                .collect::<Vec<usize>>();
            (*r_id, sub_sequence, (ready, due))
        })
        .collect::<Vec<(usize, Vec<usize>, (Num, Num))>>();

    let routes_part2 = split_before_nodes
        .iter()
        .map(|(r_id, split_node_id)| {
            let (vehicle_node, ready) = if *split_node_id == instance.vn_id_of(*r_id) + 1 {
                let pred = solution.pred(*split_node_id);
                (
                    pred,
                    solution.fw_data[pred].data.earliest_completion
                        - instance.nodes[pred].servicetime,
                )
            } else {
                (
                    *split_node_id,
                    solution.fw_data[*split_node_id].data.earliest_completion
                        - instance.nodes[*split_node_id].servicetime,
                )
            };
            let due = instance.nodes[*r_id * 2 + 1].due;
            (
                *r_id,
                [vehicle_node]
                    .into_iter()
                    .chain(
                        solution
                            .iter_route(*r_id)
                            .skip_while(|it| it != split_node_id),
                    )
                    .collect::<Vec<usize>>(),
                (ready, due),
            )
        })
        .collect::<Vec<(usize, Vec<usize>, (Num, Num))>>();

    let mut unassigned_requests = FixedBitSet::with_capacity(instance.num_requests);
    solution
        .unassigned_requests
        .iter_request_ids()
        .for_each(|request_id| unassigned_requests.insert(request_id));

    create_partial_instances_and_mapped_routes(
        instance,
        split_by_routes(
            vec![routes_part1, routes_part2],
            avg_nodes_per_route,
            rng,
            split_settings,
            #[cfg(feature = "relatedness-measures")]
                measures,
        ),
        &mut unassigned_requests,
        rng,
        split_settings,
        #[cfg(feature = "relatedness-measures")]
            measures,
    )
}

fn draw_avg_nodes_per_route_normalized_by_num_unassigned(
    instance: &PDPTWInstance,
    avg_nodes_per_route_range: RangeInclusive<usize>,
    num_unassigned: usize,
    rng: &mut Random,
) -> usize {
    rng.gen_range(avg_nodes_per_route_range) / (1 - (num_unassigned / instance.num_requests))
}

fn split_only_routes<'a>(
    instance: &'a PDPTWInstance,
    solution: &Solution,
    avg_nodes_per_route: usize,
    rng: &mut Random,
    settings: &SplitSettings,
    #[cfg(feature = "relatedness-measures")] measures: &RelatednessMeasures,
) -> Vec<(PartialInstanceStruct<'a>, Vec<Vec<usize>>)> {
    let mut unassigned_requests = FixedBitSet::with_capacity(instance.num_requests);
    solution
        .unassigned_requests
        .iter_request_ids()
        .for_each(|request_id| unassigned_requests.insert(request_id));
    create_partial_instances_and_mapped_routes(
        instance,
        split_by_routes(
            vec![solution
                .iter_route_ids()
                .filter_map(|r_id| {
                    if solution.is_route_empty(r_id) {
                        None
                    } else {
                        Some((
                            r_id,
                            solution.iter_route(r_id).collect::<Vec<usize>>(),
                            (
                                instance.nodes[instance.vn_id_of(r_id)].ready,
                                instance.nodes[instance.vn_id_of(r_id) + 1].due,
                            ),
                        ))
                    }
                })
                .collect()],
            avg_nodes_per_route,
            rng,
            settings,
            #[cfg(feature = "relatedness-measures")]
                measures,
        ),
        &mut unassigned_requests,
        rng,
        settings,
        #[cfg(feature = "relatedness-measures")]
            measures,
    )
}

fn split_by_routes(
    groups_to_split: Vec<Vec<(usize, Vec<usize>, (Num, Num))>>,
    avg_nodes_per_route: usize,
    rng: &mut Random,
    split_settings: &SplitSettings,
    #[cfg(feature = "relatedness-measures")] measures: &RelatednessMeasures,
) -> Vec<Vec<(usize, Vec<usize>, (Num, Num))>> {
    let avg_nodes = avg_nodes_per_route;
    let min_routes = 2;

    groups_to_split
        .into_iter()
        .map(|mut group_to_split| {
            group_to_split.sort_by_key(|entry| Reverse(entry.1.len()));
            let first_empty_idx = group_to_split
                .iter()
                .enumerate()
                .find(|(_, entry)| entry.1.len() <= 2)
                .map(|(i, _)| i);
            let mut empty_routes = if let Some(idx) = first_empty_idx {
                group_to_split
                    .drain(idx..)
                    .collect::<Vec<(usize, Vec<usize>, (Num, Num))>>()
            } else {
                vec![]
            };

            group_to_split.shuffle(rng);

            let avg_route_length: usize = (group_to_split
                .iter()
                .map(|(_, route, _)| ((route.len() - 2) as f64))
                .sum::<f64>()
                / group_to_split.iter().count() as f64)
                .round() as usize;

            let mut groups: Vec<Vec<(usize, Vec<usize>, (Num, Num))>> = vec![];
            while group_to_split.len() > 1 {
                if let Some(entry) = group_to_split.pop() {
                    let (t, route, tw) = entry;
                    let mut group: Vec<(usize, Vec<usize>, (Num, Num))> = vec![];
                    let mut num_nodes = route.len() - 2;
                    while group_to_split.len() > 0
                        && (group.len() + 1 < min_routes
                        || num_nodes < avg_nodes - (avg_route_length / 2))
                    {
                        // find proper other route
                        let other = match split_settings.group_matching_mode {
                            GroupMatchingMode::Random => {
                                group_to_split.swap_remove(rng.gen_range(0..group_to_split.len()))
                            }
                            #[cfg(feature = "relatedness-measures")]
                            GroupMatchingMode::ByRequestRelatedness => group_to_split.swap_remove(
                                group_to_split
                                    .iter()
                                    .map(|r2| {
                                        group
                                            .iter()
                                            .map(|r1| {
                                                measures
                                                    .get_requests_relation_value_from_itineraries(
                                                        &r1.1, &r2.1,
                                                    )
                                            })
                                            .sum::<usize>()
                                    })
                                    .enumerate()
                                    .max_by_key(|(_, it)| *it)
                                    .map(|(index, _)| index)
                                    .unwrap(),
                            ),
                        };
                        group.push(other);
                        num_nodes += route.len() - 2;
                    }
                    group.push((t, route, tw));
                    groups.push(group);
                } else {
                    break;
                }
            }
            if let Some(last_group) = group_to_split.pop() {
                if let Some(group) = groups.iter_mut().min_by_key(|it| it.len()) {
                    group.push(last_group);
                }
            }

            let num_groups = groups.len();
            for (i, group) in groups.iter_mut().enumerate() {
                for _ in 0..(empty_routes.len() / (num_groups - i)) {
                    group.push(empty_routes.pop().unwrap())
                }
            }
            groups
        })
        .flatten()
        .collect::<Vec<Vec<(usize, Vec<usize>, (Num, Num))>>>()
}

fn get_random_block_start_node(
    instance: &PDPTWInstance,
    solution: &Solution,
    pivot_route: usize,
    rng: &mut Random,
) -> usize {
    let mut prev = instance.vn_id_of(pivot_route);

    let mut block_starts = FixedBitSet::with_capacity(instance.num_requests);

    loop {
        let next = solution.succ(prev);
        if next == instance.vn_id_of(pivot_route) + 1 {
            break;
        }
        if solution.fw_data[next].data.current_load == 0 {
            block_starts.insert(instance.request_id(next));
        }
        prev = next;
    }

    if rng.gen_bool(1.0 / block_starts.count_ones(..) as f64) {
        instance.vn_id_of(pivot_route) + 1
    } else {
        instance.pickup_id_of_request(block_starts.ones().choose(rng).unwrap())
    }
}

fn get_nearest_block_start_node(
    instance: &PDPTWInstance,
    solution: &Solution,
    pivot_route: usize,
    time: Num,
) -> usize {
    let mut prev = instance.vn_id_of(pivot_route);
    let mut nearest_time = Num::max_value();
    let mut nearest_start_block_node = instance.vn_id_of(pivot_route);
    loop {
        let next = solution.succ(prev);
        if next == instance.vn_id_of(pivot_route) + 1 {
            let start = solution.fw_data[next].data.earliest_completion();
            let delta = (start - time).abs();
            if delta < nearest_time {
                nearest_start_block_node = next;
            }
            break;
        } else {
            {
                if solution.fw_data[prev].data.current_load == 0 {
                    let start = solution.bw_data[next].data.latest_start;
                    let delta = (start - time).abs();
                    if delta < nearest_time {
                        nearest_start_block_node = next;
                        nearest_time = delta;
                    }
                }
                prev = next;
            }
        }
    }
    return nearest_start_block_node;
}

#[cfg(feature = "relatedness-measures")]
fn count_link_history<'a>(
    instance: &'a PDPTWInstance,
    request_id: usize,
    group: &Vec<(usize, Vec<usize>, (Num, Num))>,
    measures: &RelatednessMeasures,
) -> usize {
    group
        .iter()
        .flat_map(|it| it.1.iter())
        .map(|node_id| {
            if instance.nodes[*node_id].node_type.is_request() {
                let other_request_id = instance.request_id(*node_id);
                measures
                    .link_counter
                    .predecessor_and_successor_count(request_id, other_request_id)
            } else {
                0usize
            }
        })
        .sum::<usize>()
}

#[cfg(feature = "relatedness-measures")]
fn count_route_history<'a>(
    instance: &'a PDPTWInstance,
    request_id: usize,
    group: &Vec<(usize, Vec<usize>, (Num, Num))>,
    measures: &RelatednessMeasures,
) -> usize {
    group
        .iter()
        .flat_map(|it| it.1.iter())
        .map(|node_id| {
            if instance.nodes[*node_id].node_type.is_request() {
                let other_request_id = instance.request_id(*node_id);
                measures
                    .same_route_counter
                    .get_count(request_id, other_request_id)
            } else {
                0usize
            }
        })
        .sum::<usize>()
}

#[cfg(feature = "relatedness-measures")]
fn count_link_and_route_history<'a>(
    instance: &'a PDPTWInstance,
    request_id: usize,
    group: &Vec<(usize, Vec<usize>, (Num, Num))>,
    measures: &RelatednessMeasures,
) -> (usize, usize) {
    (
        count_link_history(instance, request_id, group, measures),
        count_route_history(instance, request_id, group, measures),
    )
}

fn create_partial_instances_and_mapped_routes<'a>(
    instance: &'a PDPTWInstance,
    grouped_routes: Vec<Vec<(usize, Vec<usize>, (Num, Num))>>,
    distributable_unassigned_requests: &mut FixedBitSet,
    rng: &mut Random,
    split_settings: &SplitSettings,
    #[cfg(feature = "relatedness-measures")] measures: &RelatednessMeasures,
) -> Vec<(PartialInstanceStruct<'a>, Vec<Vec<usize>>)> {
    let mut assigned_requests_to_group = Vec::with_capacity(grouped_routes.len());
    let mut overlap_with_groups = Vec::with_capacity(grouped_routes.len());

    for group in &grouped_routes {
        assigned_requests_to_group.push(vec![]);

        let max_timeframe = match split_settings.unassigned_matching_mode {
            UnassignedMatchingMode::TimeWindows => Some(
                group
                    .iter()
                    .map(|it| it.2)
                    .fold((Num::MAX, Num::MIN), |acc, x| {
                        (acc.0.min(x.0).clone(), acc.1.max(x.1).clone())
                    }),
            ),
            _ => None,
        };

        let mut overlap_with_group = vec![0.0f64; instance.num_requests];
        for request_id in distributable_unassigned_requests.ones() {
            let pid = instance.pickup_id_of_request(request_id);
            let pickup_node = &instance.nodes[pid];

            overlap_with_group[request_id] = match split_settings.unassigned_matching_mode {
                UnassignedMatchingMode::Random => 1.0,
                UnassignedMatchingMode::TimeWindows => {
                    let max_timeframe = max_timeframe.unwrap();
                    if pickup_node.due >= max_timeframe.0 && pickup_node.due <= max_timeframe.0 {
                        1.0f64
                    } else {
                        let len_timeframe = pickup_node.due - pickup_node.ready;
                        let len_overlap = pickup_node.due.min(max_timeframe.1)
                            - pickup_node.ready.max(max_timeframe.0);
                        f64::from(len_overlap.max(Num::ZERO)) / f64::from(len_timeframe)
                    }
                }
                #[cfg(feature = "relatedness-measures")]
                UnassignedMatchingMode::ByLinkHistory => {
                    count_link_history(instance, request_id, &group, measures) as f64
                }
                #[cfg(feature = "relatedness-measures")]
                UnassignedMatchingMode::ByRouteHistory => {
                    count_route_history(instance, request_id, &group, measures) as f64
                }
                #[cfg(feature = "relatedness-measures")]
                UnassignedMatchingMode::ByLinkAndRouteHistory {
                    link_weight,
                    route_weight,
                } => {
                    let (count_pred_succ, count_in_route) =
                        count_link_and_route_history(instance, request_id, &group, measures);
                    count_pred_succ as f64 * link_weight + count_in_route as f64 * route_weight
                }
            };
        }
        overlap_with_groups.push(overlap_with_group);
    }

    for request_id in distributable_unassigned_requests.ones() {
        let sum = overlap_with_groups
            .iter()
            .map(|it| it[request_id])
            .sum::<f64>();

        // does it overlap with any group?
        if sum > 0.0 {
            let mut rand = rng.gen_range(0.0..sum);
            for (g_id, overlap_with_group) in overlap_with_groups.iter().enumerate() {
                rand -= overlap_with_group[request_id];
                if rand <= 0.0 {
                    assigned_requests_to_group[g_id].push(request_id);
                    break;
                }
            }
        } else {
            assigned_requests_to_group[rng.gen_range(0..grouped_routes.len())].push(request_id);
        }
    }

    grouped_routes
        .into_iter()
        .enumerate()
        .map(|(g_id, routes_of_group)| {
            let unassigned_requests_considered_for_group = assigned_requests_to_group[g_id].clone(); // avoid cloning?
            create_partial_instance_and_mapped_routes_with_unassigned_requests(
                instance,
                routes_of_group,
                unassigned_requests_considered_for_group,
                rng,
            )
        })
        .collect()
}

fn create_partial_instance_and_mapped_routes_with_unassigned_requests<'a>(
    instance: &'a PDPTWInstance,
    routes: Vec<(usize, Vec<usize>, (Num, Num))>,
    unassigned_requests: Vec<usize>,
    _rng: &mut Random,
) -> (PartialInstanceStruct<'a>, Vec<Vec<usize>>) {
    let num_vehicles = routes.len();
    let num_nodes =
        routes.iter().map(|(_, r, _)| r.len()).sum::<usize>() + unassigned_requests.len() * 2;
    let num_requests = num_nodes / 2 - num_vehicles;
    let mut vehicles = Vec::with_capacity(num_vehicles);

    let mut partial_to_full_mapping = Vec::with_capacity(num_nodes);
    let mut full_to_partial_mapping = HashMap::new();
    let mut nodes = Vec::with_capacity(num_nodes);
    let mut cnt = 0;
    for (r_id, route, ready_due) in &routes {
        vehicles.push(instance.vehicles[*r_id].clone());

        let first = route[0];
        let first_node = &instance.nodes[first];
        let last = route[route.len() - 1];
        let last_node = &instance.nodes[last];

        nodes.push(Node {
            id: cnt,
            oid: first_node.oid,
            gid: first_node.gid,
            node_type: NodeType::Depot,
            x: first_node.x,
            y: first_node.y,
            demand: 0,
            ready: ready_due.0,
            due: ready_due.1,
            servicetime: Num::ZERO,
        });
        partial_to_full_mapping.push(first_node.id);
        full_to_partial_mapping.insert(first_node.id, cnt);
        cnt += 1;

        nodes.push(Node {
            id: cnt,
            oid: last_node.oid,
            gid: last_node.gid,
            node_type: NodeType::Depot,
            x: last_node.x,
            y: last_node.y,
            demand: 0,
            ready: ready_due.0,
            due: ready_due.1,
            servicetime: Num::ZERO,
        });
        partial_to_full_mapping.push(last_node.id);
        full_to_partial_mapping.insert(last_node.id, cnt);
        cnt += 1;
    }

    for (_, route, _) in &routes {
        for visit in (1..route.len() - 1).map(|it| route[it]) {
            let node = &instance.nodes[visit];
            if node.node_type.is_pickup() {
                let pickup_node = node;
                let delivery_node = instance.delivery_of(node.id);

                nodes.push(Node {
                    id: cnt,
                    oid: pickup_node.oid,
                    gid: pickup_node.gid,
                    node_type: pickup_node.node_type.clone(),
                    x: pickup_node.x,
                    y: pickup_node.y,
                    demand: pickup_node.demand,
                    ready: pickup_node.ready,
                    due: pickup_node.due,
                    servicetime: pickup_node.servicetime,
                });
                partial_to_full_mapping.push(pickup_node.id);
                full_to_partial_mapping.insert(pickup_node.id, cnt);
                cnt += 1;

                nodes.push(Node {
                    id: cnt,
                    oid: delivery_node.oid,
                    gid: delivery_node.gid,
                    node_type: delivery_node.node_type.clone(),
                    x: delivery_node.x,
                    y: delivery_node.y,
                    demand: delivery_node.demand,
                    ready: delivery_node.ready,
                    due: delivery_node.due,
                    servicetime: delivery_node.servicetime,
                });
                partial_to_full_mapping.push(delivery_node.id);
                full_to_partial_mapping.insert(delivery_node.id, cnt);
                cnt += 1;
            }
        }
    }

    for request_id in unassigned_requests {
        let node = &instance.nodes[instance.pickup_id_of_request(request_id)];
        if node.node_type.is_pickup() {
            let pickup_node = node;
            let delivery_node = instance.delivery_of(node.id);

            nodes.push(Node {
                id: cnt,
                oid: pickup_node.oid,
                gid: pickup_node.gid,
                node_type: pickup_node.node_type.clone(),
                x: pickup_node.x,
                y: pickup_node.y,
                demand: pickup_node.demand,
                ready: pickup_node.ready,
                due: pickup_node.due,
                servicetime: pickup_node.servicetime,
            });
            partial_to_full_mapping.push(pickup_node.id);
            full_to_partial_mapping.insert(pickup_node.id, cnt);
            cnt += 1;

            nodes.push(Node {
                id: cnt,
                oid: delivery_node.oid,
                gid: delivery_node.gid,
                node_type: delivery_node.node_type.clone(),
                x: delivery_node.x,
                y: delivery_node.y,
                demand: delivery_node.demand,
                ready: delivery_node.ready,
                due: delivery_node.due,
                servicetime: delivery_node.servicetime,
            });
            partial_to_full_mapping.push(delivery_node.id);
            full_to_partial_mapping.insert(delivery_node.id, cnt);
            cnt += 1;
        }
    }

    let new_routes = routes
        .iter()
        .enumerate()
        .map(|(id, (_, it, _))| {
            it.iter()
                .enumerate()
                .map(|(it, n)| {
                    // we duplicate the first node, therefore it will be overridden; however
                    // the node_id of the first node must always the vn_id
                    if it == 0 {
                        id * 2
                    } else {
                        full_to_partial_mapping[n]
                    }
                })
                .collect::<Vec<usize>>()
        })
        .collect::<Vec<Vec<usize>>>();

    let travel_matrix = instance
        .travel_matrix
        .relabeled_subset(&partial_to_full_mapping);

    (
        PartialInstanceStruct {
            full: instance,
            partial_to_full_mapping,
            partial: PDPTWInstance {
                name: instance.name.clone(),
                num_requests,
                num_vehicles,
                nodes,
                vehicles,
                travel_matrix,
            },
        },
        new_routes,
    )
}
