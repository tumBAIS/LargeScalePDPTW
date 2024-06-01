//! Balas & Simonetti (2001) dynamic programming implementation in Rust

use fixedbitset::FixedBitSet;

use crate::problem::pdptw::PDPTWInstance;
use crate::problem::Num;
use crate::refn::{ForwardREF, REFData};
use crate::solution::Solution;

pub mod ls;

pub struct BalasSimonetti<'a> {
    instance: &'a PDPTWInstance,
    nodes: Vec<Vec<BSNodeEntryList>>,
    visits: Vec<usize>,
}

impl<'a> BalasSimonetti<'a> {
    pub fn new(instance: &PDPTWInstance, max_len: usize, thickness: usize) -> BalasSimonetti {
        BalasSimonetti {
            instance,
            nodes: (0..max_len + 1)
                .map(|_| {
                    (0..H)
                        .map(|_| BSNodeEntryList::new(thickness, instance, max_len))
                        .collect::<Vec<_>>()
                })
                .collect::<Vec<_>>(),
            visits: vec![0; max_len + 1],
        }
    }

    pub fn improve(&mut self, solution: &mut Solution<'_>, v_id: usize, k: usize) -> bool {
        debug_assert_eq!(true, k > 1);
        debug_assert_eq!(true, k <= K);
        debug_assert_eq!(
            true,
            solution.iter_route_by_vn_id(v_id * 2).count() < self.nodes.len()
        );

        // set self.visits with the corresponding route visits
        let n = self
            .visits
            .iter_mut()
            .zip(solution.iter_route_by_vn_id(v_id * 2))
            .map(|(v, s)| *v = s)
            .count();
        let ref vehicle = self.instance.vehicles[v_id];

        // reset
        for i in 0..self.nodes.len() {
            for j in 0..H {
                self.nodes[i][j as usize].clear();
            }
        }

        // first iteration
        for next in V[0].succ.iter() {
            if *next >= HK[k - 1] {
                break;
            }

            let to = {
                let offset = V[*next as usize].rel_pi as i64;
                if offset < 0 || 1 as i64 + offset >= -1 + n as i64 {
                    continue;
                }
                self.visits[(1 + offset) as usize]
            };

            if self.instance.is_delivery(to) {
                // do not expand
            } else {
                let idx = self.nodes[1][*next as usize]
                    .check_dominance_and_return_entry(
                        solution.fw_data[self.visits[0]].data.extend_forward(
                            solution.node(to),
                            self.instance.distance_and_time(self.visits[0], to),
                        ),
                    )
                    .unwrap();
                self.nodes[1][*next as usize].data[idx]
                    .visits
                    .insert(self.instance.request_id(to));
            }
        }

        // main loop
        for i in 1..n {
            // expand all bs-nodes
            for x in 0..HK[k - 1] {
                for j in 0..self.nodes[i][x as usize].size {
                    // get current visit
                    let from = {
                        let offset = V[x as usize].rel_pi as i64;
                        if -offset > i as i64 || i as i64 + offset >= n as i64 {
                            continue;
                        }
                        self.visits[(i as i64 + offset) as usize]
                    };

                    // calculate next bs-node to expand to
                    for next in if i == (n - 1) {
                        [0].iter()
                    } else {
                        V[x as usize].succ.iter()
                    } {
                        if *next >= HK[k - 1] {
                            break;
                        }
                        let to = if i == (n - 1) {
                            self.visits[n - 1]
                        } else {
                            let offset = V[*next as usize].rel_pi as i64;
                            if (i + 1) as i64 + offset < 0 || (i + 1) as i64 + offset >= n as i64 {
                                continue;
                            }
                            let to_idx = ((i + 1) as i64 + offset) as usize;
                            if *next > 0 && to_idx == n - 1 {
                                // ignore arcs leading to the final node but not the final state
                                continue;
                            }
                            self.visits[to_idx]
                        };

                        if self.instance.is_delivery(to) {
                            // check if we visited the corresponding pickup yet
                            if self.nodes[i][x as usize].data[j]
                                .visits
                                .contains(self.instance.request_id(to))
                            {
                                // if so, it is a feasible expansion
                            } else {
                                // otherwise, we stop this path
                                continue;
                            }
                        }

                        debug_assert!(
                            (x != 0 || *next != 0)
                                || (self.nodes[i][x as usize].data[j].cost.tw_feasible),
                        );

                        let d = self.instance.distance_and_time(from, to);

                        let tmp = self.nodes[i][x as usize].data[j]
                            .cost
                            .extend_forward(solution.node(to), d);

                        if tmp.check_feasible(vehicle) {
                            if let Some(idx) = self.nodes[i + 1][*next as usize]
                                .check_dominance_and_return_entry(tmp)
                            {
                                self.nodes[i + 1][*next as usize].data[idx].prev = [x, j as u8];
                                if to != self.visits[n - 1] {
                                    self.nodes[i + 1][*next as usize].data[idx].visits =
                                        self.nodes[i][x as usize].data[j].visits.clone();
                                    self.nodes[i + 1][*next as usize].data[idx]
                                        .visits
                                        .insert(self.instance.request_id(to));
                                }
                            }
                        }
                    }
                }
            }
        }

        if self.nodes[n - 1][0].data[0].cost.distance
            < solution.fw_data[self.visits[n - 1]].data.distance
        {
            let last_prev = self.nodes[n - 1][0].data[0].prev;

            // update backwards
            let mut last_changed = None;
            let mut first_changed = None;

            let [mut h, mut j] = last_prev;
            let mut current = self.visits[n - 1];
            let mut current_rel_pi = 0;
            for i in 1..n - 1 {
                let prev =
                    self.visits[((n - 1 - i) as i64 + (V[h as usize].rel_pi as i64)) as usize];

                solution.link_nodes(prev, current);
                if V[h as usize].rel_pi != 0 || current_rel_pi != 0 {
                    // this one has changed
                    if last_changed.is_none() {
                        last_changed.replace(prev);
                    }
                    first_changed.replace(prev);
                }
                current = prev;
                current_rel_pi = V[h as usize].rel_pi;
                [h, j] = self.nodes[n - 1 - i][h as usize].data[j as usize].prev;
            }
            if current_rel_pi != 0 {
                solution.link_nodes(self.visits[0], current);
            }

            if last_changed.is_some() {
                solution.validate_between(
                    solution.pred(first_changed.unwrap()),
                    solution.succ(last_changed.unwrap()),
                );
                true
            } else {
                false
            }
        } else {
            false
        }
    }
}

const K: usize = 4;
// maximum distance of moving nodes in a sequence
const H: usize = (K + 1) * (2usize).pow(K as u32 - 2);

// Node in the Balas-Simonetti Graph
struct BSNode {
    rel_pi: i8,
    succ: [u8; K],
}

const V: [BSNode; H] = [
    // lvl 1
    BSNode {
        rel_pi: 0,
        succ: [0, 1, 3, 8],
    },
    // lvl 2
    BSNode {
        rel_pi: 1,
        succ: [2, 4, 9, u8::MAX],
    },
    BSNode {
        rel_pi: -1,
        succ: [0, 1, 3, 8],
    },
    // lvl 3
    BSNode {
        rel_pi: 2,
        succ: [5, 6, 10, u8::MAX],
    },
    BSNode {
        rel_pi: 1,
        succ: [7, 14, u8::MAX, u8::MAX],
    },
    BSNode {
        rel_pi: -1,
        succ: [2, 4, 9, u8::MAX],
    },
    BSNode {
        rel_pi: 0,
        succ: [7, 14, u8::MAX, u8::MAX],
    },
    BSNode {
        rel_pi: -2,
        succ: [0, 1, 3, 8],
    },
    // lvl 4
    BSNode {
        rel_pi: 3,
        succ: [11, 12, 13, u8::MAX],
    },
    BSNode {
        rel_pi: 2,
        succ: [15, 16, u8::MAX, u8::MAX],
    },
    BSNode {
        rel_pi: 2,
        succ: [17, 18, u8::MAX, u8::MAX],
    },
    BSNode {
        rel_pi: -1,
        succ: [5, 6, 10, u8::MAX],
    },
    BSNode {
        rel_pi: 0,
        succ: [15, 16, u8::MAX, u8::MAX],
    },
    BSNode {
        rel_pi: 1,
        succ: [17, 18, u8::MAX, u8::MAX],
    },
    BSNode {
        rel_pi: 1,
        succ: [19, u8::MAX, u8::MAX, u8::MAX],
    },
    BSNode {
        rel_pi: -2,
        succ: [2, 4, 9, u8::MAX],
    },
    BSNode {
        rel_pi: 0,
        succ: [19, u8::MAX, u8::MAX, u8::MAX],
    },
    BSNode {
        rel_pi: -1,
        succ: [19, u8::MAX, u8::MAX, u8::MAX],
    },
    BSNode {
        rel_pi: -2,
        succ: [7, 14, u8::MAX, u8::MAX],
    },
    BSNode {
        rel_pi: -3,
        succ: [0, 1, 3, 8],
    },
];

const HK: [u8; 4] = [1, 3, 8, 20]; // exclusive index marking the end in V for a given K

struct BSNodeEntry {
    pub cost: REFData,
    pub visits: FixedBitSet,
    pub prev: [u8; 2],
}

impl BSNodeEntry {
    pub fn new(_instance: &PDPTWInstance, max_len: usize) -> Self {
        Self {
            cost: REFData::default(),
            visits: FixedBitSet::with_capacity(max_len),
            prev: [0, 0],
        }
    }

    pub fn clear(&mut self) {
        self.cost.tw_feasible = false;
        self.cost.distance = Num::MAX;
        self.visits.clear();
        self.prev = [0, 0];
    }
}

struct BSNodeEntryList {
    data: Vec<BSNodeEntry>,
    size: usize,
}

impl BSNodeEntryList {
    pub fn new(thickness: usize, instance: &PDPTWInstance, max_len: usize) -> Self {
        Self {
            data: (0..thickness)
                .map(|_| BSNodeEntry::new(instance, max_len))
                .collect::<Vec<_>>(),
            size: 0,
        }
    }

    pub fn clear(&mut self) {
        for it in self.data.iter_mut() {
            it.clear();
        }
        self.size = 0;
    }

    pub fn check_dominance_and_return_entry(&mut self, s: REFData) -> Option<usize> {
        let mut i = 0;
        while i < self.size {
            if !self.data[i].cost.tw_feasible || s.distance < self.data[i].cost.distance {
                break;
            } else if BSNodeEntryList::dominates(&self.data[i].cost, &s) {
                return None;
            } else if self.data[i].cost.distance == s.distance {
                break;
            } else {
                i += 1;
            }
        }

        let s_idx = i;
        let mut tmp = if i == self.data.len() {
            return None;
        } else if i == self.size {
            self.data[i].cost = s;
            self.size += 1;
            return Some(i);
        } else if BSNodeEntryList::dominates(&s, &self.data[i].cost) {
            self.data[i].cost = s;
            None
        } else {
            let mut entry = BSNodeEntry {
                cost: s,
                visits: FixedBitSet::with_capacity(self.data[i].visits.len()),
                prev: [0, 0],
            };
            std::mem::swap(&mut self.data[i], &mut entry);

            Some(entry)
        };

        let mut removed = 0;
        i += 1;
        while i < self.size {
            if BSNodeEntryList::dominates(&self.data[s_idx].cost, &self.data[i].cost) {
                if tmp.is_some() {
                    let val = tmp.take().unwrap();
                    self.data[i] = val;
                } else {
                    removed += 1;
                }
            } else {
                if tmp.is_some() {
                    std::mem::swap(&mut self.data[i], tmp.as_mut().unwrap());
                } else if removed > 0 {
                    self.data.swap(i, i - removed);
                }
            }
            i += 1;
        }

        if i < self.data.len() {
            if let Some(entry) = tmp {
                self.size = i + 1;
                let _ = std::mem::replace(&mut self.data[i], entry);
            } else if removed > 0 {
                self.size -= removed;
            }
        } else if removed > 0 {
            self.size -= removed;
        }

        Some(s_idx)
    }

    pub fn dominates(a: &REFData, b: &REFData) -> bool {
        a.distance <= b.distance && a.earliest_completion <= b.earliest_completion
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

        sol.set(&routes); // <- required by balas_simonetti (route data used to identify an improvement)

        let mut balas_simonetti = BalasSimonetti::new(&instance, instance.num_requests * 2 + 2, 4);
        assert!(balas_simonetti.improve(&mut sol, routes[0][0] / 2, 4));

        Ok(())
    }
}
