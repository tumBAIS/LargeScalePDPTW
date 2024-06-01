//! Modified version of the k-disjoint shortest path problem solver where
//! terminate early in the classic PDPTW (with fleet-minimization goal)
//! if all nodes are part of any shortest path. Note that we do only return
//! the shortest paths found when terminating to reduce the memory overhead.

use std::cmp::Reverse;
use std::collections::{BinaryHeap, VecDeque};
use std::marker::PhantomData;

use fixedbitset::FixedBitSet;
use kdsp::node_disjoint::SourceSinkGraph;
use kdsp::Vertex;
use num_traits::bounds::Bounded;
use num_traits::{NumOps, Zero};

use crate::utils::NumIndexVec;

fn bellman_ford<W>(
    graph: &SourceSinkGraph<W>,
    p: &mut NumIndexVec<Vertex>,
    d: &mut NumIndexVec<W>,
) -> bool
    where
        W: Zero + Bounded + NumOps + Copy + PartialOrd + Ord,
{
    let source = graph.source();
    for (idx, entry) in &mut p.iter_mut().enumerate() {
        *entry = idx as Vertex;
    }
    for entry in &mut d.iter_mut() {
        *entry = W::max_value();
    }
    d[source] = W::zero();

    let mut on_queue = FixedBitSet::with_capacity(d.len());
    let mut queue = VecDeque::with_capacity(d.len());
    queue.push_back(source);
    on_queue.insert(source as usize);

    while let Some(from) = queue.pop_front() {
        on_queue.set(from as usize, false);
        for arc in graph.iter_outgoing_arcs(from) {
            if d[from] + arc.w < d[arc.to] {
                d[arc.to] = d[from] + arc.w;
                p[arc.to] = from;
                if !on_queue[arc.to as usize] {
                    on_queue.insert(arc.to as usize);
                    queue.push_back(arc.to);
                }
            }
        }
    }

    #[cfg(debug_assertions)]
    {
        // check for negative-cost cycles only enabled in debug build (with debug assertions on)
        for arc in graph.iter_arcs() {
            debug_assert!(
                d[arc.from] + arc.w >= d[arc.to],
                "Graph contains at least one negative-cost cycle"
            )
        }
    }

    return true;
}

struct Uninitialized;

struct Initialized;

#[allow(non_snake_case)]
struct Suurballe<W, Stage = Uninitialized>
    where
        W: Zero + Bounded + NumOps + Copy + PartialOrd + Ord,
{
    stage: std::marker::PhantomData<Stage>,

    graph: SourceSinkGraph<W>,

    p: NumIndexVec<Vertex>,
    p_in_S: FixedBitSet,
    p_t: NumIndexVec<Vertex>,
    s_t: NumIndexVec<Vertex>,
    w_t: NumIndexVec<W>,
    d: NumIndexVec<W>,

    A: NumIndexVec<Vertex>,
    Z: NumIndexVec<Vertex>,
    L: NumIndexVec<W>,

    C: BinaryHeap<Reverse<(W, Vertex)>>,
    in_C: FixedBitSet,

    D: W,
    M: usize,

    Aj_in_Pm: FixedBitSet,
    nodes_to_visit: Option<FixedBitSet>,
}

impl<W> Suurballe<W, Uninitialized>
    where
        W: Zero + Bounded + NumOps + Copy + PartialOrd + Ord + std::fmt::Display,
{
    pub fn new(graph: SourceSinkGraph<W>, nodes_to_visit: Option<FixedBitSet>) -> Self {
        let mut p_t = fix_sized_vec![0; graph.num_nodes()];
        for j in 0..graph.num_nodes() {
            p_t[j] = j as Vertex;
        }
        Self {
            stage: Default::default(),
            p: fix_sized_vec![0; graph.num_nodes()],
            p_in_S: FixedBitSet::with_capacity(graph.num_nodes() as usize),
            p_t,
            s_t: fix_sized_vec![0; graph.num_nodes()],
            w_t: fix_sized_vec![W::zero(); graph.num_nodes()],
            d: fix_sized_vec![W::zero(); graph.num_nodes()],
            A: fix_sized_vec![0; graph.num_nodes()],
            Z: fix_sized_vec![0; graph.num_nodes()],
            L: fix_sized_vec![W::zero(); graph.num_nodes()],
            Aj_in_Pm: FixedBitSet::with_capacity(graph.num_nodes() as usize),
            C: BinaryHeap::new(),
            in_C: FixedBitSet::with_capacity(graph.num_nodes() as usize),
            D: W::zero(),
            M: 0,
            graph,
            nodes_to_visit,
        }
    }

    pub fn first(mut self) -> Suurballe<W, Initialized> {
        let source = self.graph.source();
        let sink = self.graph.sink();

        bellman_ford(&self.graph, &mut self.p, &mut self.d);

        // Reweigh the edges to remove negatives
        for arc in self.graph.iter_arcs_mut() {
            if self.d[arc.from] == W::max_value() {
                arc.w = W::max_value()
            } else {
                arc.w = self.d[arc.from] - self.d[arc.to] + arc.w;
            }
        }

        // store initial shortest path
        let mut next = self.p[sink];
        self.Z[0usize] = next;

        loop {
            let prev = self.p[next];
            self.p_t[next] = prev;
            self.s_t[prev] = next;
            self.w_t[next] = self.graph.get_arc(prev, next).w;
            // in P_1 there are no i64ed nodes
            if prev == source {
                self.A[0usize] = next;
                self.Aj_in_Pm.insert(next as usize);
                break;
            } else {
                next = prev;
            }
        }

        // initialize labels
        for j in 0..self.graph.num_nodes() {
            self.L[j] = if self.d[sink] < self.d[j] {
                self.d[sink]
            } else {
                self.d[j]
            };
        }

        self.initialize_candidate_list();
        self.M = 1;

        Suurballe::<W, Initialized> {
            stage: PhantomData::default(),
            graph: self.graph,
            p: self.p,
            p_in_S: self.p_in_S,
            p_t: self.p_t,
            s_t: self.s_t,
            w_t: self.w_t,
            d: self.d,
            A: self.A,
            Z: self.Z,
            L: self.L,
            C: self.C,
            in_C: self.in_C,
            D: self.D,
            M: self.M,
            Aj_in_Pm: self.Aj_in_Pm,
            nodes_to_visit: self.nodes_to_visit,
        }
    }
}

impl<W> Suurballe<W, Initialized>
    where
        W: Zero + Bounded + NumOps + Copy + PartialOrd + Ord + std::fmt::Display,
{
    #[allow(non_snake_case)]
    pub fn next(&mut self) -> Option<usize> {
        // check whether we visited all the required nodes
        #[cfg(feature = "fleet-minimization")]
        if let Some(ref nodes_to_visit) = self.nodes_to_visit {
            let mut tmp = FixedBitSet::with_capacity(nodes_to_visit.len());
            for (idx, v) in self.p_t.iter().cloned().enumerate() {
                if idx == v as usize {
                    continue;
                }
                tmp.insert(v as usize);
            }
            for i in 0..self.M {
                tmp.insert(self.Z[i] as usize);
            }
            if nodes_to_visit.is_subset(&tmp) {
                return None;
            }
        }

        let source = self.graph.source();
        let sink = self.graph.sink();

        // Interlacing Construction
        // find I = node in C with the minimal distance
        while let Some(Reverse((weight, mut I))) = self.C.pop() {
            if self.d[I] < weight || !self.in_C[I as usize] {
                continue;
            }
            self.L[I] = self.L[I] + self.d[I];
            self.in_C.set(I as usize, false);

            if I == sink {
                // go to P_M+1
                while I != source {
                    // p[I] cannot be negative at this point
                    let mut J = self.p[I]; // J = node in P_M where S last leaves P_M prior to I
                    let mut Q = I; // Q = node following J on S
                    self.p_t[Q] = J;
                    self.w_t[Q] = self.graph.get_arc(J, Q).w;
                    while self.p_t[J] == J && J != source {
                        self.s_t[J] = Q;
                        // p[Q] cannot be negative at this point
                        Q = J;
                        J = self.p[Q];
                        self.p_t[Q] = J;
                        self.w_t[Q] = self.graph.get_arc(J, Q).w;
                    }

                    if J != source {
                        let T = self.s_t[J]; // find T = node following J on P_M
                        self.s_t[J] = Q;

                        if self.p_in_S[T as usize] {
                            // going further than T, reset ts along the way
                            I = self.p[T];
                            let mut i = I;
                            while i != J {
                                let prev_t = self.p_t[i];
                                self.p_t[i] = i;
                                i = prev_t;
                            }
                        } else {
                            I = T;
                        }
                    } else {
                        self.s_t[J] = Q;
                        // p[sink] cannot be negative at this point
                        self.A[self.M] = Q;
                        self.Z[self.M] = self.p[sink];
                        self.Aj_in_Pm.set(Q as usize, true);

                        self.D = self.D + self.L[sink];

                        self.C.clear();
                        self.M += 1;

                        let dZ = self.d[sink];
                        // New Initial Condition
                        // step 1: For each node j in C, add d_z to L_j
                        for j in 1..self.graph.num_nodes() {
                            if self.in_C.contains(j as usize) && !self.Aj_in_Pm.contains(j as usize)
                            {
                                self.L[j] = self.L[j] + dZ;
                            }
                        }

                        // step 2 + 3
                        self.reinitialize_candidate_list();
                        return Some(self.M);
                    }
                }
            } else if self.p_t[I] != I {
                // I in P_M
                // go to negative interlacing
                let mut J = self.p_t[I];
                let mut Q = I;
                let T = if self.p_in_S[I as usize] {
                    self.p[I]
                } else {
                    I
                };
                while J != source {
                    if Q != I {
                        // remove from C all nodes between and different from I and J in P
                        self.in_C.set(Q as usize, false);
                    }
                    if !self.in_C.contains(J as usize) || self.L[J] + self.w_t[Q] < self.L[Q] {
                        break;
                    }
                    Q = J;
                    J = self.p_t[Q];
                }

                // step 3
                let mut s = self.p_t[I];
                while s != J {
                    self.L[s] = self.L[s] + self.d[I];
                    self.p[s] = T;
                    self.p_in_S.insert(s as usize);
                    self.expand_to_neighbors(s, self.L[s]);
                    s = self.p_t[s];
                }
                // step 4
                let Lj = self.L[Q] - self.graph.get_arc(J, Q).w;
                self.expand_to_neighbors(J, Lj);

                // step 5
                if self.in_C.contains(J as usize) {
                    let delta = Lj - self.L[J];
                    if self.d[J] > delta {
                        self.d[J] = delta;
                        self.p[J] = T;
                        self.p_in_S.insert(J as usize);
                        self.C.push(Reverse((delta, J)));
                    }
                }
                // go to Interlacing Construction
            } else {
                // just expand to neighbors
                self.expand_to_neighbors(I, self.L[I]);
            }
        }

        None
    }
}

impl<W, Stage> Suurballe<W, Stage>
    where
        W: Zero + Bounded + NumOps + Copy + PartialOrd + Ord,
{
    #[allow(non_snake_case)]
    #[inline(always)]
    fn initialize_candidate_list(&mut self) {
        for j in 0..self.graph.num_nodes() {
            self.d[j] = W::max_value();
        }
        for arc in self.graph.iter_outgoing_arcs(self.graph.source()) {
            let j = arc.to;
            if j != self.A[self.graph.source()] {
                self.d[j] = arc.w - self.L[j];
                self.p[j] = self.graph.source();
                self.C.push(Reverse((self.d[j], arc.to)));
                self.in_C.insert(j as usize);
            } else {
                self.Aj_in_Pm.insert(j as usize);
            }
        }
        for j in 1..self.graph.num_nodes() {
            if !self.Aj_in_Pm.contains(j as usize) {
                self.in_C.insert(j as usize);
            }
        }
    }

    #[allow(non_snake_case)]
    #[inline(always)]
    fn expand_to_neighbors(&mut self, i: Vertex, L_i: W) {
        for arc in self.graph.iter_outgoing_arcs(i) {
            let j = arc.to;
            if self.in_C.contains(j as usize) {
                let delta = L_i + arc.w - self.L[j];
                if self.d[j] > delta {
                    self.d[j] = delta;
                    self.p[j] = i;
                    self.p_in_S.set(j as usize, false);
                    self.C.push(Reverse((delta, arc.to)));
                }
            }
        }
    }

    #[allow(non_snake_case)]
    #[inline(always)]
    fn reinitialize_candidate_list(&mut self) {
        self.in_C.clear();
        self.initialize_candidate_list();
    }

    fn extract_all_shortest_path_edges_sorted(&self) -> Vec<(Vertex, Vertex)> {
        // extract all edges of shortest paths
        let mut sp_edges = Vec::new();
        for i in 0..self.M {
            sp_edges.push((self.Z[i], self.graph.sink()));
            let mut j = self.Z[i];
            while j != self.graph.source() {
                let pred_j = self.p_t[j];
                sp_edges.push((pred_j, j));
                j = pred_j;
            }
        }
        sp_edges.sort();
        sp_edges
    }
}

#[allow(non_snake_case)]
#[inline(never)]
pub fn solve<W>(
    graph: SourceSinkGraph<W>,
    k: Option<usize>,
    nodes_to_visit: Option<FixedBitSet>,
) -> (usize, Vec<(Vertex, Vertex)>)
    where
        W: Zero + Bounded + NumOps + Copy + PartialOrd + Ord + std::fmt::Display,
{
    let alg = Suurballe::new(graph, nodes_to_visit);
    let mut alg = alg.first();
    for _ in 1..if k.is_some() {
        k.unwrap()
    } else {
        alg.graph.iter_outgoing_arcs(alg.graph.source()).count()
    } {
        if let None = alg.next() {
            break;
        }
    }

    // extract all edges of shortest paths
    let sp_edges = alg.extract_all_shortest_path_edges_sorted();

    (alg.M, sp_edges)
}

#[cfg(test)]
mod tests {
    use std::iter::FromIterator;
    use std::ops::RangeInclusive;

    use kdsp::node_disjoint::SourceSinkGraphBuilder;
    use kdsp::Arc;
    use rand::seq::SliceRandom;
    use rand::{Rng, SeedableRng};
    use rand_pcg::Pcg64;

    use super::*;

    fn create_random_graph(
        rand: &mut Pcg64,
        width: usize,
        length: usize,
        link_density_range: RangeInclusive<usize>,
        weight_range: RangeInclusive<i32>,
    ) -> SourceSinkGraph<i32> {
        let num_vertices = width * length + 2;

        let min_links = (*link_density_range.start()).min(width);
        let max_links = (*link_density_range.end()).min(width);
        let next_num_links = |rand: &mut Pcg64| rand.gen_range(min_links..=max_links);

        let min_weight = *weight_range.start();
        let max_weight = *weight_range.end();
        let next_weight = |rand: &mut Pcg64| rand.gen_range(min_weight..=max_weight);

        let mut arcs: Vec<Arc<i32>> = vec![];
        // arcs from the source
        for i in 0..width {
            arcs.push(Arc {
                from: 0,
                to: (i + 1) as Vertex,
                w: 0,
            });
        }

        let mut shuffled: Vec<usize> = (0..width).collect();
        for j in 0..length - 1 {
            shuffled.shuffle(rand);
            let mut uncovered_next_vertices = FixedBitSet::from_iter(0..width);
            for i in 0..width {
                for l in 0..next_num_links(rand) {
                    arcs.push(Arc {
                        from: (1 + width * j + i) as Vertex,
                        to: (1 + width * (j + 1) + shuffled[l]) as Vertex,
                        w: next_weight(rand),
                    });
                    uncovered_next_vertices.set(shuffled[l], false);
                }
            }
            for v in uncovered_next_vertices.ones() {
                // nodes missing -> just connect the i->i
                arcs.push(Arc {
                    from: (1 + width * j + v) as Vertex,
                    to: (1 + width * (j + 1) + v) as Vertex,
                    w: next_weight(rand),
                });
            }
        }

        for i in 0..width {
            arcs.push(Arc {
                from: (i + 1 + width * (length - 1)) as Vertex,
                to: (num_vertices - 1) as Vertex,
                w: 0,
            });
        }

        SourceSinkGraphBuilder::new()
            .set_num_nodes(num_vertices as Vertex)
            .add_arcs(arcs.into_iter())
            .build()
    }

    #[test]
    fn generic_10x10_graph_with_seed_842() {
        let mut rand = Pcg64::seed_from_u64(842);
        let graph = create_random_graph(&mut rand, 10, 10, 2..=7, -250..=25);
        let (_, _) = solve(graph, Some(10), None);
    }

    #[test]
    fn generic_10x10_graph_with_seed_84() {
        let mut rand = Pcg64::seed_from_u64(84);
        let graph = create_random_graph(&mut rand, 10, 10, 2..=7, -250..=25);
        let (_, _) = solve(graph, Some(10), None);
    }

    #[test]
    fn generic_10x10_graph_with_seed_42() {
        let mut rand = Pcg64::seed_from_u64(42);
        let graph = create_random_graph(&mut rand, 10, 10, 2..=7, -250..=25);
        let (_, _) = solve(graph, Some(10), None);
    }

    #[test]
    fn suurballe() {
        // # example from Suurballe (1974) paper with k = 3 and D = 25
        // A,B,1.0
        // B,C,1.0
        // C,D,1.0
        // D,E,1.0
        // E,Z,1.0
        // A,E,8.0
        // A,F,1.0
        // B,Z,8.0
        // D,G,2.0
        // F,C,2.0
        // F,G,6.0
        // G,Z,1.0

        #[allow(non_snake_case)]
        struct Vertices {
            A: Vertex,
            B: Vertex,
            C: Vertex,
            D: Vertex,
            E: Vertex,
            F: Vertex,
            G: Vertex,
            Z: Vertex,
        }
        let vertices = Vertices {
            A: 0 as Vertex,
            B: 1 as Vertex,
            C: 2 as Vertex,
            D: 3 as Vertex,
            E: 4 as Vertex,
            F: 5 as Vertex,
            G: 6 as Vertex,
            Z: 7 as Vertex,
        };
        let arcs = vec![
            Arc {
                from: vertices.A,
                to: vertices.B,
                w: 1,
            },
            Arc {
                from: vertices.B,
                to: vertices.C,
                w: 1,
            },
            Arc {
                from: vertices.C,
                to: vertices.D,
                w: 1,
            },
            Arc {
                from: vertices.D,
                to: vertices.E,
                w: 1,
            },
            Arc {
                from: vertices.E,
                to: vertices.Z,
                w: 1,
            },
            Arc {
                from: vertices.A,
                to: vertices.E,
                w: 8,
            },
            Arc {
                from: vertices.A,
                to: vertices.F,
                w: 1,
            },
            Arc {
                from: vertices.B,
                to: vertices.Z,
                w: 8,
            },
            Arc {
                from: vertices.D,
                to: vertices.G,
                w: 2,
            },
            Arc {
                from: vertices.F,
                to: vertices.C,
                w: 2,
            },
            Arc {
                from: vertices.F,
                to: vertices.G,
                w: 6,
            },
            Arc {
                from: vertices.G,
                to: vertices.Z,
                w: 1,
            },
        ];

        let graph = SourceSinkGraphBuilder::new()
            .set_num_nodes(8)
            .set_source(vertices.A)
            .set_sink(vertices.Z)
            .add_arcs(arcs.into_iter())
            .build();

        let (_, sp_arcs) = solve(graph, Some(3), None);

        let sp_arcs_contains = |from: Vertex, to: Vertex| -> bool {
            sp_arcs
                .iter()
                .find(|(a, b)| from == *a && to == *b)
                .is_some()
        };

        assert!(sp_arcs_contains(vertices.A, vertices.B));
        assert!(sp_arcs_contains(vertices.B, vertices.Z));

        assert!(sp_arcs_contains(vertices.A, vertices.E));
        assert!(sp_arcs_contains(vertices.E, vertices.Z));

        assert!(sp_arcs_contains(vertices.A, vertices.F));
        assert!(sp_arcs_contains(vertices.F, vertices.C));
        assert!(sp_arcs_contains(vertices.C, vertices.D));
        assert!(sp_arcs_contains(vertices.D, vertices.G));
        assert!(sp_arcs_contains(vertices.G, vertices.Z));
    }

    #[test]
    fn mid_example() {
        let arcs = vec![
            Arc {
                from: 0,
                to: 2,
                w: 0,
            },
            Arc {
                from: 2,
                to: 1,
                w: -10000000,
            },
            Arc {
                from: 0,
                to: 4,
                w: 0,
            },
            Arc {
                from: 4,
                to: 1,
                w: -10000000,
            },
            Arc {
                from: 2,
                to: 6,
                w: -39004297,
            },
            Arc {
                from: 4,
                to: 6,
                w: -39004297,
            },
            Arc {
                from: 6,
                to: 8,
                w: -39646728,
            },
            Arc {
                from: 6,
                to: 9,
                w: -17933421,
            },
            Arc {
                from: 6,
                to: 10,
                w: -19648898,
            },
            Arc {
                from: 6,
                to: 11,
                w: -18426874,
            },
            Arc {
                from: 6,
                to: 12,
                w: -18806123,
            },
            Arc {
                from: 6,
                to: 13,
                w: -38470531,
            },
            Arc {
                from: 6,
                to: 14,
                w: -18353779,
            },
            Arc {
                from: 6,
                to: 15,
                w: -38299118,
            },
            Arc {
                from: 6,
                to: 16,
                w: -18912909,
            },
            Arc {
                from: 6,
                to: 1,
                w: -9003643,
            },
            Arc {
                from: 2,
                to: 7,
                w: -39738971,
            },
            Arc {
                from: 4,
                to: 7,
                w: -39738971,
            },
            Arc {
                from: 7,
                to: 8,
                w: -38703229,
            },
            Arc {
                from: 7,
                to: 9,
                w: -19196388,
            },
            Arc {
                from: 7,
                to: 10,
                w: -18769212,
            },
            Arc {
                from: 7,
                to: 11,
                w: -19302587,
            },
            Arc {
                from: 7,
                to: 12,
                w: -18766710,
            },
            Arc {
                from: 7,
                to: 13,
                w: -39592925,
            },
            Arc {
                from: 7,
                to: 14,
                w: -19133199,
            },
            Arc {
                from: 7,
                to: 15,
                w: -38916854,
            },
            Arc {
                from: 7,
                to: 16,
                w: -19231529,
            },
            Arc {
                from: 7,
                to: 1,
                w: -9420187,
            },
            Arc {
                from: 2,
                to: 8,
                w: -39254337,
            },
            Arc {
                from: 4,
                to: 8,
                w: -39254337,
            },
            Arc {
                from: 8,
                to: 9,
                w: -18980446,
            },
            Arc {
                from: 8,
                to: 10,
                w: -18647872,
            },
            Arc {
                from: 8,
                to: 11,
                w: -19027197,
            },
            Arc {
                from: 8,
                to: 12,
                w: -18510794,
            },
            Arc {
                from: 8,
                to: 13,
                w: -39317278,
            },
            Arc {
                from: 8,
                to: 14,
                w: -19363180,
            },
            Arc {
                from: 8,
                to: 15,
                w: -39144620,
            },
            Arc {
                from: 8,
                to: 16,
                w: -18990759,
            },
            Arc {
                from: 8,
                to: 1,
                w: -9306072,
            },
            Arc {
                from: 2,
                to: 9,
                w: -18786321,
            },
            Arc {
                from: 4,
                to: 9,
                w: -18786321,
            },
            Arc {
                from: 9,
                to: 11,
                w: -19612324,
            },
            Arc {
                from: 9,
                to: 12,
                w: -18906572,
            },
            Arc {
                from: 9,
                to: 13,
                w: -39651573,
            },
            Arc {
                from: 9,
                to: 14,
                w: -18542134,
            },
            Arc {
                from: 9,
                to: 15,
                w: -38324256,
            },
            Arc {
                from: 9,
                to: 16,
                w: -19184789,
            },
            Arc {
                from: 9,
                to: 1,
                w: -9004232,
            },
            Arc {
                from: 2,
                to: 10,
                w: -19338561,
            },
            Arc {
                from: 4,
                to: 10,
                w: -19338561,
            },
            Arc {
                from: 10,
                to: 13,
                w: -38513341,
            },
            Arc {
                from: 10,
                to: 14,
                w: -19316677,
            },
            Arc {
                from: 10,
                to: 15,
                w: -39370557,
            },
            Arc {
                from: 10,
                to: 16,
                w: -18608333,
            },
            Arc {
                from: 10,
                to: 1,
                w: -9118392,
            },
            Arc {
                from: 2,
                to: 11,
                w: -19157964,
            },
            Arc {
                from: 4,
                to: 11,
                w: -19157964,
            },
            Arc {
                from: 11,
                to: 13,
                w: -38386661,
            },
            Arc {
                from: 11,
                to: 14,
                w: -18461409,
            },
            Arc {
                from: 11,
                to: 15,
                w: -38434773,
            },
            Arc {
                from: 11,
                to: 16,
                w: -18782193,
            },
            Arc {
                from: 11,
                to: 1,
                w: -8981758,
            },
            Arc {
                from: 2,
                to: 12,
                w: -18999286,
            },
            Arc {
                from: 4,
                to: 12,
                w: -18999286,
            },
            Arc {
                from: 12,
                to: 13,
                w: -39798706,
            },
            Arc {
                from: 12,
                to: 14,
                w: -18550356,
            },
            Arc {
                from: 12,
                to: 15,
                w: -38339665,
            },
            Arc {
                from: 12,
                to: 16,
                w: -19461089,
            },
            Arc {
                from: 12,
                to: 1,
                w: -9181474,
            },
            Arc {
                from: 2,
                to: 13,
                w: -39344871,
            },
            Arc {
                from: 4,
                to: 13,
                w: -39344871,
            },
            Arc {
                from: 13,
                to: 15,
                w: -39085432,
            },
            Arc {
                from: 13,
                to: 16,
                w: -19334319,
            },
            Arc {
                from: 13,
                to: 1,
                w: -9837158,
            },
            Arc {
                from: 2,
                to: 14,
                w: -19096300,
            },
            Arc {
                from: 4,
                to: 14,
                w: -19096300,
            },
            Arc {
                from: 14,
                to: 15,
                w: -38009104,
            },
            Arc {
                from: 14,
                to: 16,
                w: -19583369,
            },
            Arc {
                from: 14,
                to: 1,
                w: -9053274,
            },
            Arc {
                from: 2,
                to: 15,
                w: -38936438,
            },
            Arc {
                from: 4,
                to: 15,
                w: -38936438,
            },
            Arc {
                from: 15,
                to: 1,
                w: -8866440,
            },
            Arc {
                from: 2,
                to: 16,
                w: -19469886,
            },
            Arc {
                from: 4,
                to: 16,
                w: -19469886,
            },
            Arc {
                from: 16,
                to: 1,
                w: -9051199,
            },
        ];

        let graph = SourceSinkGraphBuilder::new()
            .set_num_nodes(17)
            .set_source(0)
            .set_sink(1)
            .add_arcs(arcs.into_iter())
            .build();

        let (_, sp_arcs) = solve(graph, Some(3), None);

        let sp_arcs_contains = |from: Vertex, to: Vertex| -> bool {
            sp_arcs
                .iter()
                .find(|(a, b)| from == *a && to == *b)
                .is_some()
        };

        assert!(sp_arcs_contains(0, 2));
        assert!(sp_arcs_contains(2, 6));
        assert!(sp_arcs_contains(6, 8));
        assert!(sp_arcs_contains(8, 10));
        assert!(sp_arcs_contains(10, 14));
        assert!(sp_arcs_contains(14, 16));
        assert!(sp_arcs_contains(16, 1));

        assert!(sp_arcs_contains(0, 4));
        assert!(sp_arcs_contains(4, 7));
        assert!(sp_arcs_contains(7, 9));
        assert!(sp_arcs_contains(9, 12));
        assert!(sp_arcs_contains(12, 13));
        assert!(sp_arcs_contains(13, 15));
        assert!(sp_arcs_contains(15, 1));
    }
}
