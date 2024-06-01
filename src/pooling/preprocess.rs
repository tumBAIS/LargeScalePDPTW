use fixedbitset::FixedBitSet;
use itertools::Itertools;
use tinyvec::ArrayVec;

use crate::pooling::MAX_REQUESTS_PER_POOLING;
use crate::problem::pdptw::{Node, PDPTWInstance};
use crate::problem::Num;

fn check_tw_feasible_from_iter<'a>(
    instance: &PDPTWInstance,
    mut iter: impl Iterator<Item=&'a Node>,
) -> bool {
    let mut u = iter.next().unwrap();
    let mut time = u.ready;
    while let Some(v) = iter.next() {
        time += u.servicetime + instance.time(u.id, v.id);
        if time > v.due {
            return false;
        } else {
            time = time.max(v.ready);
            u = v;
        }
    }
    true
}

fn check_tw_feasible_from_node_id_iter(
    instance: &PDPTWInstance,
    iter: impl Iterator<Item=usize>,
) -> bool {
    check_tw_feasible_from_iter(instance, iter.map(|id| &instance.nodes[id]))
}

fn calc_distance(instance: &PDPTWInstance, sequence: &[&Node]) -> Num {
    sequence.windows(2).fold(Num::ZERO, |acc, uv| {
        acc + instance.distance(uv[0].id, uv[1].id)
    })
}

fn calc_distance_from_iter<'a>(
    instance: &PDPTWInstance,
    mut iter: impl Iterator<Item=&'a Node>,
) -> Num {
    let mut u = iter.next().unwrap();
    let mut distance = Num::ZERO;
    while let Some(v) = iter.next() {
        distance += instance.distance(u.id, v.id);
        u = v;
    }
    distance
}

fn calc_distance_from_node_id_iter(
    instance: &PDPTWInstance,
    iter: impl Iterator<Item=usize>,
) -> Num {
    calc_distance_from_iter(instance, iter.map(|id| &instance.nodes[id]))
}

pub struct CombinationCalculator {
    request_permutations: Vec<Vec<Vec<usize>>>,
    pd_permutations: Vec<Vec<Vec<usize>>>,
}

#[derive(Debug, Copy, Clone)]
pub enum CalculationResult {
    FoundMin(Num),
    WasDominated(Num),
    NoneFound,
}

impl CombinationCalculator {
    pub fn new(max_requests: usize, max_concurrent_requests: usize) -> Self {
        let request_permutations = (2..=max_requests)
            .map(|k| (0..k).permutations(k).collect())
            .collect();
        let pd_permutations = (2..=max_requests)
            .map(|k| {
                let mut bitset = FixedBitSet::with_capacity(2 * k);
                (0..k)
                    .into_iter()
                    .chain((k..2 * k).into_iter())
                    .permutations(2 * k)
                    .unique()
                    .filter(|it| {
                        if it[0] < k {
                            bitset.clear();
                            bitset.insert(it[0]);
                            let mut i = 0;
                            loop {
                                i += 1;
                                if i < it.len() {
                                    let current_active_requests = bitset.count_ones(..);
                                    if (it[i - 1] + k == it[i]) && (current_active_requests == 1) {
                                        break false;
                                    } else if it[i] < k {
                                        if current_active_requests >= max_concurrent_requests {
                                            break false;
                                        } else {
                                            bitset.toggle(it[i]);
                                        }
                                    } else {
                                        if !bitset.contains(it[i] - k) {
                                            break false;
                                        } else {
                                            bitset.toggle(it[i] - k);
                                        }
                                    }
                                } else {
                                    break true;
                                }
                            }
                        } else {
                            false
                        }
                    })
                    .collect()
            })
            .collect();
        Self {
            request_permutations,
            pd_permutations,
        }
    }

    pub fn calc_min_any_order_request_ids_with_ub(
        &self,
        instance: &PDPTWInstance,
        requests: &[usize],
        ub: Option<Num>,
    ) -> CalculationResult {
        let k = requests.len();

        let pd_direct_distances = (0..k)
            .map(|i| {
                instance.distance(
                    instance.pickup_id_of_request(requests[i]),
                    instance.delivery_id_of_request(requests[i]),
                )
            })
            .sum::<Num>();

        let direct_combinations = self.request_permutations[k - 2]
            .iter()
            .map(|combination| {
                combination
                    .windows(2)
                    .map(|a| {
                        instance.distance(
                            instance.delivery_id_of_request(requests[a[0]]),
                            instance.pickup_id_of_request(requests[a[1]]),
                        )
                    })
                    .sum::<Num>()
            })
            .min()
            .expect("each combination has at least two entries");

        let direct = pd_direct_distances + direct_combinations;
        let bound = ub.unwrap_or(Num::MAX).min(direct);

        let combinations = &self.pd_permutations[k - 2];

        let map_request = |i: &usize| {
            if *i < k {
                instance.pickup_id_of_request(requests[*i])
            } else {
                instance.delivery_id_of_request(requests[*i - k])
            }
        };
        let mut distances: Vec<(usize, Num)> = combinations
            .iter()
            .map(|sequence| {
                let cost =
                    calc_distance_from_node_id_iter(instance, sequence.iter().map(map_request));
                cost
            })
            .enumerate()
            .collect();

        distances.sort_unstable_by(|(_, cost_a), (_, cost_b)| cost_a.cmp(cost_b));
        for (id, cost) in distances {
            if cost >= bound {
                return CalculationResult::WasDominated(bound);
            } else if check_tw_feasible_from_node_id_iter(
                instance,
                combinations[id].iter().map(map_request),
            ) {
                return CalculationResult::FoundMin(cost);
            }
        }
        return CalculationResult::NoneFound;
    }

    pub fn calc_min_any_order_request_ids(
        &self,
        instance: &PDPTWInstance,
        requests: &[usize],
    ) -> Option<Num> {
        let k = requests.len();

        let pd_direct_distances = (0..k)
            .map(|i| {
                instance.distance(
                    instance.pickup_id_of_request(requests[i]),
                    instance.delivery_id_of_request(requests[i]),
                )
            })
            .sum::<Num>();

        let direct_combinations = self.request_permutations[k - 2]
            .iter()
            .map(|combination| {
                combination
                    .windows(2)
                    .map(|a| {
                        instance.distance(
                            instance.delivery_id_of_request(requests[a[0]]),
                            instance.pickup_id_of_request(requests[a[1]]),
                        )
                    })
                    .sum::<Num>()
            })
            .min()
            .expect("each combination has at least two entries");

        let direct = pd_direct_distances + direct_combinations;

        let combinations = &self.pd_permutations[k - 2];

        let map_request = |i: &usize| {
            if *i < k {
                instance.pickup_id_of_request(requests[*i])
            } else {
                instance.delivery_id_of_request(requests[*i - k])
            }
        };
        let mut distances: Vec<(usize, Num)> = combinations
            .iter()
            .map(|sequence| {
                let cost =
                    calc_distance_from_node_id_iter(instance, sequence.iter().map(map_request));
                cost
            })
            .enumerate()
            .collect();

        distances.sort_unstable_by(|(_, cost_a), (_, cost_b)| cost_a.cmp(cost_b));
        for (id, cost) in distances {
            if cost >= direct {
                return None;
            } else if check_tw_feasible_from_node_id_iter(
                instance,
                combinations[id].iter().map(map_request),
            ) {
                return Some(cost);
            }
        }
        return None;
    }

    pub fn get_min_any_order_by_request_ids(
        &self,
        instance: &PDPTWInstance,
        requests: &[usize],
    ) -> Option<(Vec<usize>, Num)> {
        let k = requests.len();
        let combinations = &self.pd_permutations[k - 2];

        let map_request = |i: &usize| {
            if *i < k {
                instance.pickup_id_of_request(requests[*i])
            } else {
                instance.delivery_id_of_request(requests[*i - k])
            }
        };
        let mut distances: Vec<(usize, Num)> = combinations
            .iter()
            .map(|sequence| {
                let cost =
                    calc_distance_from_node_id_iter(instance, sequence.iter().map(map_request));
                cost
            })
            .enumerate()
            .collect();

        distances.sort_unstable_by(|(_, cost_a), (_, cost_b)| cost_a.cmp(cost_b));
        for (id, cost) in distances {
            if check_tw_feasible_from_node_id_iter(
                instance,
                combinations[id].iter().map(map_request),
            ) {
                return Some((combinations[id].iter().map(map_request).collect(), cost));
            }
        }
        return None;
    }

    pub fn calc_min_any_order_pickup_nodes(
        &self,
        instance: &PDPTWInstance,
        nodes: &[&Node],
    ) -> Option<Num> {
        let k = nodes.len();

        let pd_direct_distances = (0..k)
            .map(|i| instance.distance(nodes[i].id, instance.delivery_of(nodes[i].id).id))
            .sum::<Num>();

        let direct_combinations = self.request_permutations[k - 2]
            .iter()
            .map(|combination| {
                combination
                    .windows(2)
                    .map(|a| {
                        instance.distance(instance.delivery_of(nodes[a[0]].id).id, nodes[a[1]].id)
                    })
                    .sum::<Num>()
            })
            .min()
            .expect("each combination has at least two entries");

        let direct = pd_direct_distances + direct_combinations;

        let combinations = &self.pd_permutations[k - 2];

        let mut distances: Vec<(usize, Num)> = combinations
            .iter()
            .map(|sequence| {
                let cost = calc_distance_from_iter(
                    instance,
                    sequence.iter().map(|i| {
                        if *i < k {
                            nodes[*i]
                        } else {
                            instance.delivery_of(nodes[*i - k].id)
                        }
                    }),
                );
                cost
            })
            .enumerate()
            .collect();

        distances.sort_unstable_by(|(_, cost_a), (_, cost_b)| cost_a.cmp(cost_b));
        for (id, cost) in distances {
            if cost >= direct {
                return None;
            } else if check_tw_feasible_from_iter(
                instance,
                combinations[id].iter().map(|i| {
                    if *i < k {
                        nodes[*i]
                    } else {
                        instance.delivery_of(nodes[*i - k].id)
                    }
                }),
            ) {
                return Some(cost);
            }
        }
        return None;
    }

    pub fn get_min_any_order(
        &self,
        instance: &PDPTWInstance,
        nodes: &[&Node],
    ) -> Option<Vec<usize>> {
        let k = nodes.len();

        let pd_direct_distances = (0..k)
            .map(|i| instance.distance(nodes[i].id, instance.delivery_of(nodes[i].id).id))
            .sum::<Num>();

        let direct_combinations = self.request_permutations[k - 2]
            .iter()
            .map(|combination| {
                combination
                    .windows(2)
                    .map(|a| {
                        instance.distance(
                            instance.delivery_of(nodes[a[0]].id).id,
                            instance.delivery_of(nodes[a[1]].id).id,
                        )
                    })
                    .sum::<Num>()
            })
            .min()
            .expect("each combination has at least two entries");

        let direct = pd_direct_distances + direct_combinations;

        let combinations = &self.pd_permutations[k - 2];

        let mut distances: Vec<(usize, Num)> = combinations
            .iter()
            .map(|sequence| {
                calc_distance_from_iter(
                    instance,
                    sequence.iter().map(|i| {
                        if *i < k {
                            nodes[*i]
                        } else {
                            instance.delivery_of(nodes[*i - k].id)
                        }
                    }),
                )
            })
            .enumerate()
            .collect();

        distances.sort_unstable_by(|(_, cost_a), (_, cost_b)| cost_a.cmp(cost_b));
        for (id, cost) in distances {
            if cost >= direct {
                return None;
            } else if check_tw_feasible_from_iter(
                instance,
                combinations[id].iter().map(|i| {
                    if *i < k {
                        nodes[*i]
                    } else {
                        instance.delivery_of(nodes[*i - k].id)
                    }
                }),
            ) {
                return Some(
                    combinations[id]
                        .iter()
                        .map(|i| {
                            if *i < k {
                                nodes[*i].id
                            } else {
                                instance.delivery_of(nodes[*i - k].id).id
                            }
                        })
                        .collect(),
                );
            }
        }
        return None;
    }

    pub fn calc_min_any_order_request_ids_plain(
        &self,
        instance: &PDPTWInstance,
        requests: &[usize],
    ) -> Option<Num> {
        let k = requests.len();

        let pd_direct_distances = (0..k)
            .map(|i| {
                instance.distance(
                    instance.pickup_id_of_request(requests[i]),
                    instance.delivery_id_of_request(requests[i]),
                )
            })
            .sum::<Num>();

        let direct_combinations = self.request_permutations[k - 2]
            .iter()
            .map(|combination| {
                combination
                    .windows(2)
                    .map(|a| {
                        instance.distance(
                            instance.delivery_id_of_request(requests[a[0]]),
                            instance.pickup_id_of_request(requests[a[1]]),
                        )
                    })
                    .sum::<Num>()
            })
            .min()
            .expect("each combination has at least two entries");

        let direct = pd_direct_distances + direct_combinations;

        let combinations = &self.pd_permutations[k - 2];

        let map_request = |i: &usize| {
            if *i < k {
                instance.pickup_id_of_request(requests[*i])
            } else {
                instance.delivery_id_of_request(requests[*i - k])
            }
        };
        let mut distances: Vec<(usize, Num)> = combinations
            .iter()
            .map(|sequence| {
                let cost =
                    calc_distance_from_node_id_iter(instance, sequence.iter().map(map_request));
                cost
            })
            .enumerate()
            .collect();

        distances.sort_unstable_by(|(_, cost_a), (_, cost_b)| cost_a.cmp(cost_b));
        for (id, cost) in distances {
            if cost >= direct {
                return None;
            } else if check_tw_feasible_from_node_id_iter(
                instance,
                combinations[id].iter().map(map_request),
            ) {
                return Some(cost);
            }
        }
        return None;
    }
}

fn permutations(requests: &ArrayVec<[usize; MAX_REQUESTS_PER_POOLING]>) -> Vec<Vec<usize>> {
    assert!(requests.len() > 0);
    let permutations = requests.iter().cloned().permutations(requests.len());
    permutations.collect()
}

#[cfg(test)]
mod tests {
    use tinyvec::array_vec;

    use super::*;

    #[test]
    fn permutations_pair() {
        itertools::assert_equal(
            permutations(&array_vec!([usize; MAX_REQUESTS_PER_POOLING] => 0, 2)),
            vec![vec![0, 2], vec![2, 0]],
        );
    }

    #[test]
    fn combinations_pair() {
        let calc = CombinationCalculator::new(2, 2);
        dbg!(&calc.request_permutations);
        println!("combinations:");
        dbg!(&calc.pd_permutations);
        for perm in &calc.pd_permutations[0] {
            println!("{:?}", perm);
        }
    }
}
