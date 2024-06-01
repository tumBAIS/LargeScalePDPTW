use std::cmp::Reverse;

use enum_map::{enum_map, Enum, EnumMap};
use fixedbitset::FixedBitSet;

use crate::construction::kdsp::Blocks;
use crate::problem::pdptw::{Node, PDPTWInstance};
use crate::problem::Num;
use crate::refn::REFData;
use crate::solution::blocknode::BlockNode;
use crate::utils::refn::REFCalculator;

#[derive(Enum)]
pub enum RelationType {
    Distance,
    MinWaitingTime,
    Gain,
}

pub struct RequestsStats<'a> {
    instance: &'a PDPTWInstance,
    pub of: Vec<Relations>,
}

pub struct Relation {
    pub node: usize,
    pub values: EnumMap<RelationType, Num>,
    pub combinations: EnumMap<Combinations, Option<REFData>>,
}

pub struct Relations {
    pub node: usize,
    pub with: Vec<Relation>,
}

impl<'a> RequestsStats<'a> {
    pub fn process(instance: &'a PDPTWInstance) -> Self {
        let mut sorted_nodes: Vec<&Node> = instance.iter_pickups().collect();
        sorted_nodes.sort_by(|a, b| (a.ready).cmp(&(b.ready)));

        Self {
            instance,
            of: sorted_nodes
                .iter()
                .map(|n| {
                    let mut nearest: Vec<&&Node> = sorted_nodes
                        .iter()
                        .filter(|u| {
                            u.id != n.id
                                && n.ready + n.servicetime + instance.time(n.id, u.id) <= u.due
                                && u.ready
                                + u.servicetime
                                + instance.time(u.id, instance.delivery_of(n.id).id)
                                <= instance.delivery_of(n.id).due
                        })
                        .collect();
                    nearest.sort_by_cached_key(|u| instance.time(n.id, u.id));
                    let mut relations = Relations {
                        node: n.id,
                        with: nearest
                            .iter()
                            .take(10)
                            .filter_map(|u| {
                                let gain_of = gain(instance, n, u);
                                if gain_of < Num::EPSILON {
                                    return None;
                                }
                                let combinations = combinations(instance, n, u);
                                if combinations.values().all(|it| it.is_none()) {
                                    return None;
                                }

                                let min_wt = min_wt(instance, n, u);
                                Some(Relation {
                                    node: u.id,
                                    values: enum_map![
                                        RelationType::Distance => distance(instance, n, u),
                                        RelationType::MinWaitingTime => min_wt,
                                        RelationType::Gain => gain_of
                                    ],
                                    combinations,
                                })
                            })
                            .collect(),
                    };
                    relations
                        .with
                        .sort_by_cached_key(|u| Reverse(u.values[RelationType::Gain]));
                    relations
                        .with
                        .sort_by_cached_key(|u| u.values[RelationType::MinWaitingTime]);
                    relations
                })
                .collect(),
        }
    }

    pub fn print(&self) {
        for n in &self.of {
            println!("{:?}", self.instance.nodes[n.node]);
            println!("├─ nearest reachable successors:", );
            for u in &n.with {
                println!(
                    "   ├─ id: {} | dist: {} | min_wt: {}, gain: {}",
                    u.node,
                    u.values[RelationType::Distance],
                    u.values[RelationType::MinWaitingTime],
                    u.values[RelationType::Gain]
                );
            }
        }
    }
}

#[derive(Enum)]
pub enum Combinations {
    P1P2D1D2,
    P1P2D2D1,
}

fn combinations(
    instance: &PDPTWInstance,
    r: &Node,
    o: &Node,
) -> EnumMap<Combinations, Option<REFData>> {
    let r_d = instance.delivery_of(r.id);
    let o_d = instance.delivery_of(o.id);

    let calc = REFCalculator::with_instance(instance);

    enum_map! {
        Combinations::P1P2D1D2 => Some(calc.first(r).then(o).then(r_d).then(o_d).finished())
            .filter(|it| it.tw_feasible),
        Combinations::P1P2D2D1 => Some(calc.first(r).then(o).then(o_d).then(r_d).finished())
            .filter(|it| it.tw_feasible),
    }
}

fn distance(instance: &PDPTWInstance, r: &Node, o: &Node) -> Num {
    instance.distance(r.id, o.id)
}

fn min_wt(instance: &PDPTWInstance, r: &Node, o: &Node) -> Num {
    Num::ZERO
        .max(o.ready - (r.due + r.servicetime + instance.time(r.id, o.id)))
        .clone()
}

fn gain(instance: &PDPTWInstance, r_p: &Node, o_p: &Node) -> Num {
    let r_d = instance.delivery_of(r_p.id);
    let o_d = instance.delivery_of(o_p.id);

    let dist_p1d1 = instance.distance(r_p.id, r_d.id);
    let dist_p2d2 = instance.distance(o_p.id, o_d.id);
    let dist_d1p2 = instance.distance(r_d.id, o_p.id);
    let dist_p1p2 = instance.distance(r_p.id, o_p.id);
    let dist_p2d1 = instance.distance(o_p.id, r_d.id);
    let dist_d1d2 = instance.distance(r_d.id, o_d.id);
    let dist_d2d1 = instance.distance(o_d.id, r_d.id);

    let separate = dist_p1d1 + dist_d1p2 + dist_p2d2;
    let p1p2d1d2 = dist_p1p2 + dist_p2d1 + dist_d1d2;
    let p1p2d2d1 = dist_p1p2 + dist_p2d2 + dist_d2d1;
    let gain = separate - p1p2d1d2.min(p1p2d2d1);

    gain
}

pub fn extract_promising_blocks(stats: &RequestsStats) -> Vec<Blocks> {
    stats.print();
    let calc = REFCalculator::with_instance(stats.instance);

    let mut candidate_blocks = vec![];
    for p1 in &stats.of {
        candidate_blocks.extend(p1.with.iter().filter_map(|p2| {
            if p2.values[RelationType::Gain] >= Num::ZERO {
                Some((
                    p2.values[RelationType::Gain],
                    p2.values[RelationType::MinWaitingTime],
                    p1.node,
                    p2.node,
                    Some(&p2.combinations),
                ))
            } else {
                None
            }
        }));
        candidate_blocks.push((Num::ZERO, Num::ZERO, p1.node, p1.node, None));
    }
    let mut used = FixedBitSet::with_capacity(stats.instance.nodes.len());
    candidate_blocks.sort_by_cached_key(|(gain, _, _, _, _)| Reverse(gain.clone()));

    let mut blocks = vec![];
    for (_gain, _min_wt, p1, p2, combinations) in candidate_blocks {
        if used.contains(p1) || used.contains(p2) {
            continue;
        }
        if p1 == p2 {
            used.insert(p1);
            blocks.push(create_block_from_single_request(p1, &calc, stats.instance));
        } else if let Some(combinations) = combinations {
            used.insert(p1);
            used.insert(p2);

            blocks.push(create_block_from_combination(
                p1,
                p2,
                combinations,
                stats.instance,
            ));
        };
    }

    blocks
}

pub fn extract_blocks(stats: &RequestsStats) -> Vec<Blocks> {
    let calc = REFCalculator::with_instance(stats.instance);

    let mut candidate_blocks = vec![];
    for p1 in &stats.of {
        candidate_blocks.extend(p1.with.iter().filter_map(|p2| {
            if p2.values[RelationType::Gain] >= Num::ZERO {
                Some((
                    p2.values[RelationType::Gain],
                    p2.values[RelationType::MinWaitingTime],
                    p1.node,
                    p2.node,
                    Some(&p2.combinations),
                ))
            } else {
                None
            }
        }));
        candidate_blocks.push((Num::ZERO, Num::ZERO, p1.node, p1.node, None));
    }
    let mut used = FixedBitSet::with_capacity(stats.instance.nodes.len());
    candidate_blocks.sort_by_cached_key(|(gain, _, _, _, _)| Reverse(gain.clone()));

    let mut blocks = vec![];
    for (_gain, _min_wt, p1, p2, combinations) in candidate_blocks {
        if p1 == p2 {
            used.insert(p1);
            blocks.push(create_block_from_single_request(p1, &calc, stats.instance));
        } else if let Some(combinations) = combinations {
            used.insert(p1);
            used.insert(p2);

            blocks.push(create_block_from_combination(
                p1,
                p2,
                combinations,
                stats.instance,
            ));
        };
    }

    blocks
}

pub fn create_block_from_combination(
    p1: usize,
    p2: usize,
    combinations: &EnumMap<Combinations, Option<REFData>>,
    instance: &PDPTWInstance,
) -> Blocks {
    // test combination
    let r_p = &instance.nodes[p1];
    let r_d = instance.delivery_of(r_p.id);
    let o_p = &instance.nodes[p2];
    let o_d = instance.delivery_of(o_p.id);

    let p1p2d1d2 = combinations[Combinations::P1P2D1D2].as_ref();
    let p1p2d2d1 = combinations[Combinations::P1P2D2D1].as_ref();

    let to_use = match (p1p2d1d2, p1p2d2d1) {
        (None, None) => unreachable!(),
        (Some(r1), None) => (Combinations::P1P2D1D2, r1),
        (None, Some(r2)) => (Combinations::P1P2D2D1, r2),
        (Some(r1), Some(r2)) => {
            if r1.distance < r2.distance {
                (Combinations::P1P2D1D2, r1)
            } else {
                (Combinations::P1P2D2D1, r2)
            }
        }
    };

    match to_use {
        (Combinations::P1P2D1D2, data) => Blocks {
            earliest_start: data.earliest_completion - data.duration(),
            latest_start: data.latest_start,
            path: vec![p1, p2, r_d.id, o_d.id],
            block: BlockNode {
                first_node_id: p1,
                last_node_id: o_d.id,
                data: combinations[Combinations::P1P2D1D2]
                    .as_ref()
                    .unwrap()
                    .clone(),
                #[cfg(feature = "trace-refdata")]
                trace: vec![p1, p2, r_d.id, o_d.id],
            },
        },
        (Combinations::P1P2D2D1, data) => Blocks {
            earliest_start: data.earliest_completion - data.duration(),
            latest_start: data.latest_start,
            path: vec![p1, p2, o_d.id, r_d.id],
            block: BlockNode {
                first_node_id: p1,
                last_node_id: r_d.id,
                data: combinations[Combinations::P1P2D2D1]
                    .as_ref()
                    .unwrap()
                    .clone(),
                #[cfg(feature = "trace-refdata")]
                trace: vec![p1, p2, o_d.id, r_d.id],
            },
        },
    }
}

pub fn create_block_from_single_request(
    p: usize,
    calc: &REFCalculator,
    instance: &PDPTWInstance,
) -> Blocks {
    let data = calc
        .first(&instance.nodes[p])
        .then(instance.delivery_of(p))
        .finished();
    Blocks {
        earliest_start: data.earliest_completion - data.duration(),
        latest_start: data.latest_start,
        path: vec![p, instance.delivery_of(p).id],
        block: BlockNode {
            first_node_id: p,
            last_node_id: instance.delivery_of(p).id,
            data,
            #[cfg(feature = "trace-refdata")]
            trace: vec![p, instance.delivery_of(p).id],
        },
    }
}
