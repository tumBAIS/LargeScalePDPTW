use crate::node_disjoint::SourceSinkGraph;
use crate::utils::NumIndexVec;
use crate::Vertex;
use fixedbitset::FixedBitSet;
use num_traits::bounds::Bounded;
use num_traits::{NumOps, Zero};
use std::cmp::Reverse;
use std::collections::{BinaryHeap, VecDeque};

fn bellman_ford<W>(
    graph: &SourceSinkGraph<W>,
    p: &mut NumIndexVec<Vertex>,
    d: &mut NumIndexVec<W>,
) -> bool
    where
        W: Zero + Bounded + NumOps + Copy + PartialOrd + Ord,
{
    let source = graph.source_node;
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

#[allow(non_snake_case)]
pub fn solve<W>(mut graph: SourceSinkGraph<W>, k: Option<usize>) -> (usize, Vec<(Vertex, Vertex)>)
    where
        W: Zero + Bounded + NumOps + Copy + PartialOrd + Ord,
{
    let source = graph.source_node;
    let sink = graph.sink_node;

    let mut p = num_index_vec![0; graph.num_nodes()];
    let mut p_in_S = FixedBitSet::with_capacity(graph.num_nodes as usize);
    let mut p_t = num_index_vec![0; graph.num_nodes()];
    let mut s_t = num_index_vec![0; graph.num_nodes()];
    let mut w_t = num_index_vec![W::zero(); graph.num_nodes()];
    let mut d = num_index_vec![W::zero(); graph.num_nodes()];
    for j in 0..graph.num_nodes() {
        p_t[j] = j as Vertex
    }

    let mut A = num_index_vec![0; graph.num_nodes()];
    let mut Z = num_index_vec![0; graph.num_nodes()];
    let mut L = num_index_vec![W::zero(); graph.num_nodes()];

    let mut Aj_in_Pm = FixedBitSet::with_capacity(graph.num_nodes as usize);

    {
        let mut p_1 = num_index_vec![0; graph.num_nodes()];
        bellman_ford(&graph, &mut p_1, &mut d);

        // Reweigh the edges to remove negatives
        for arc in graph.iter_arcs_mut() {
            arc.w = d[arc.from] - d[arc.to] + arc.w;
        }

        // store initial shortest path
        let mut next = p_1[sink];
        Z[0usize] = next;

        loop {
            let prev = p_1[next];
            p_t[next] = prev;
            s_t[prev] = next;
            w_t[next] = graph.get_arc(prev, next).w;
            // in P_1 there are no i64ed nodes
            if prev == source {
                A[0usize] = next;
                Aj_in_Pm.insert(next as usize);
                break;
            } else {
                next = prev;
            }
        }
    }
    // initialize labels
    for j in 0..graph.num_nodes() {
        L[j] = if d[sink] < d[j] { d[sink] } else { d[j] };
    }

    let mut C = BinaryHeap::new();
    let mut in_C = FixedBitSet::with_capacity(graph.num_nodes() as usize);

    initialize_candidate_list(
        &graph,
        &A,
        &mut Aj_in_Pm,
        &mut C,
        &mut in_C,
        &mut p,
        &mut d,
        &L,
    );

    let k = k.unwrap_or(graph.iter_outgoing_arcs(source).count());
    let mut M = 1;

    // Interlacing Construction
    // find I = node in C with the minimal distance
    while let Some(Reverse((weight, mut I))) = C.pop() {
        if d[I] < weight || !in_C[I as usize] {
            continue;
        }
        L[I] = L[I] + d[I];
        in_C.set(I as usize, false);

        if I == sink {
            // go to P_M+1
            while I != 0 {
                // p[I] cannot be negative at this point
                let mut J = p[I]; // J = node in P_M where S last leaves P_M prior to I
                let mut Q = I; // Q = node following J on S
                p_t[Q] = J;
                w_t[Q] = graph.get_arc(J, Q).w;
                while p_t[J] == J && J != 0 {
                    s_t[J] = Q;
                    // p[Q] cannot be negative at this point
                    Q = J;
                    J = p[Q];
                    p_t[Q] = J;
                    w_t[Q] = graph.get_arc(J, Q).w;
                }

                if J != 0 {
                    let T = s_t[J];
                    s_t[J] = Q;

                    if p_in_S[T as usize] {
                        // going further than T, reset ts along the way
                        I = p[T];
                        let mut i = I;
                        while i != J {
                            let prev_t = p_t[i];
                            p_t[i] = i;
                            i = prev_t;
                        }
                    } else {
                        I = T;
                    }
                } else {
                    s_t[J] = Q;
                    // p[sink] cannot be negative at this point
                    A[M] = Q;
                    Z[M] = p[sink];
                    Aj_in_Pm.set(Q as usize, true);

                    // D += L[sink];

                    C.clear();
                    M += 1;

                    if k > M {
                        let dZ = d[sink];
                        // New Initial Condition
                        // step 1: For each node j in C, add d_z to L_j
                        for j in 1..graph.num_nodes() {
                            if in_C.contains(j as usize) && !Aj_in_Pm.contains(j as usize) {
                                L[j] = L[j] + dZ;
                            }
                        }

                        // step 2 + 3
                        reinitialize_candidate_list(
                            &graph,
                            &A,
                            &mut Aj_in_Pm,
                            &mut C,
                            &mut in_C,
                            &mut p,
                            &mut d,
                            &L,
                        );
                    } else {
                        // Terminate
                    }

                    break;
                }
            }
        } else if p_t[I] != I {
            // I in P_M
            // go to negative interlacing
            let mut J = p_t[I];
            let mut Q = I;
            let T = if p_in_S[I as usize] { p[I] } else { I };
            while J != 0 {
                if Q != I {
                    // remove from C all nodes between and different from I and J in P
                    in_C.set(Q as usize, false);
                }
                if !in_C.contains(J as usize) || L[J] + w_t[Q] < L[Q] {
                    break;
                }
                Q = J;
                J = p_t[Q];
            }

            // step 3
            let mut s = p_t[I];
            while s != J {
                L[s] = L[s] + d[I];
                p[s] = T;
                p_in_S.insert(s as usize);
                expand_to_neighbors(
                    s,
                    L[s],
                    &graph,
                    &mut C,
                    &in_C,
                    &mut p,
                    &mut p_in_S,
                    &mut d,
                    &L,
                );
                s = p_t[s];
            }
            // step 4
            let Lj = L[Q] - graph.get_arc(J, Q).w;
            expand_to_neighbors(
                J,
                Lj,
                &graph,
                &mut C,
                &in_C,
                &mut p,
                &mut p_in_S,
                &mut d,
                &L,
            );

            // step 5
            if in_C.contains(J as usize) {
                let delta = Lj - L[J];
                if d[J] > delta {
                    d[J] = delta;
                    p[J] = T;
                    p_in_S.insert(J as usize);
                    C.push(Reverse((delta, J)));
                }
            }
            // go to Interlacing Construction
        } else {
            // just expand to neighbors
            expand_to_neighbors(
                I,
                L[I],
                &graph,
                &mut C,
                &in_C,
                &mut p,
                &mut p_in_S,
                &mut d,
                &L,
            );
        }
    }

    // extract all edges of shortest paths
    let mut sp_edges = Vec::new();
    for i in 0..M {
        sp_edges.push((Z[i], sink));
        let mut j = Z[i];
        while j != 0 {
            let pred_j = p_t[j];
            sp_edges.push((pred_j, j));
            j = pred_j;
        }
    }
    sp_edges.sort();

    (M, sp_edges)
}

#[allow(non_snake_case)]
#[inline(always)]
fn expand_to_neighbors<W>(
    i: Vertex,
    L_i: W,
    graph: &SourceSinkGraph<W>,
    C: &mut BinaryHeap<Reverse<(W, Vertex)>>,
    in_C: &FixedBitSet,
    p: &mut NumIndexVec<Vertex>,
    is_p_in_S: &mut FixedBitSet,
    d: &mut NumIndexVec<W>,
    L: &NumIndexVec<W>,
) where
    W: Zero + Bounded + NumOps + Copy + PartialOrd + Ord,
{
    for arc in graph.iter_outgoing_arcs(i) {
        let j = arc.to;
        if in_C.contains(j as usize) {
            let delta = L_i + arc.w - L[j];
            if d[j] > delta {
                d[j] = delta;
                p[j] = i;
                is_p_in_S.set(j as usize, false);
                C.push(Reverse((delta, arc.to)));
            }
        }
    }
}

#[allow(non_snake_case)]
#[inline(always)]
fn initialize_candidate_list<W>(
    graph: &SourceSinkGraph<W>,
    A: &NumIndexVec<Vertex>,
    Aj_in_Pm: &mut FixedBitSet,
    C: &mut BinaryHeap<Reverse<(W, Vertex)>>,
    in_C: &mut FixedBitSet,
    p: &mut NumIndexVec<Vertex>,
    d: &mut NumIndexVec<W>,
    L: &NumIndexVec<W>,
) where
    W: Zero + Bounded + NumOps + Copy + PartialOrd + Ord,
{
    for j in 0..graph.num_nodes() {
        d[j] = W::max_value();
    }
    for arc in graph.iter_outgoing_arcs(graph.source_node) {
        let j = arc.to;
        if j != A[0usize] {
            d[j] = arc.w - L[j];
            p[j] = 0;
            C.push(Reverse((d[j], arc.to)));
            in_C.insert(j as usize);
        } else {
            Aj_in_Pm.insert(j as usize);
        }
    }
    for j in 1..graph.num_nodes() {
        if !Aj_in_Pm.contains(j as usize) {
            in_C.insert(j as usize);
        }
    }
}

#[allow(non_snake_case)]
#[inline(always)]
fn reinitialize_candidate_list<W>(
    graph: &SourceSinkGraph<W>,
    A: &NumIndexVec<Vertex>,
    Aj_in_Pm: &mut FixedBitSet,
    C: &mut BinaryHeap<Reverse<(W, Vertex)>>,
    in_C: &mut FixedBitSet,
    p: &mut NumIndexVec<Vertex>,
    d: &mut NumIndexVec<W>,
    L: &NumIndexVec<W>,
) where
    W: Zero + Bounded + NumOps + Copy + PartialOrd + Ord,
{
    in_C.clear();
    initialize_candidate_list(graph, A, Aj_in_Pm, C, in_C, p, d, L);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::node_disjoint::tests::{read_instance, ReadInstanceError, TEST_GRAPHS, TEST_GRAPHS_DIR};
    use crate::node_disjoint::SourceSinkGraphBuilder;
    use crate::Arc;
    use rand::seq::SliceRandom;
    use rand::{Rng, SeedableRng};
    use rand_pcg::Pcg64;
    use std::iter::FromIterator;
    use std::ops::RangeInclusive;

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
        let (_, _) = solve(graph, Some(10));
    }

    #[test]
    fn generic_10x10_graph_with_seed_84() {
        let mut rand = Pcg64::seed_from_u64(84);
        let graph = create_random_graph(&mut rand, 10, 10, 2..=7, -250..=25);
        let (_, _) = solve(graph, Some(10));
    }

    #[test]
    fn generic_10x10_graph_with_seed_42() {
        let mut rand = Pcg64::seed_from_u64(42);
        let graph = create_random_graph(&mut rand, 10, 10, 2..=7, -250..=25);
        let (_, _) = solve(graph, Some(10));
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

        let (_, sp_arcs) = solve(graph, Some(3));

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

        let (_, sp_arcs) = solve(graph, Some(3));

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

    #[test]
    fn test_graphs() -> Result<(), ReadInstanceError> {
        for path in TEST_GRAPHS {
            let (graph, _sol) = read_instance(format!("{}/{}", TEST_GRAPHS_DIR, path))?;
            let (_, sp) = solve(graph, Some(100));

            let (graph, sol) = read_instance(format!("{}/{}", TEST_GRAPHS_DIR, path))?;

            if let Some(solution) = sol {
                // first check the costs
                let mut obj = 0;
                for a in sp {
                    obj += graph.get_arc(a.0, a.1).w;
                }
                let mut ref_obj = 0;
                for ref_path in solution.paths {
                    for a in ref_path.windows(2) {
                        ref_obj += graph.get_arc(a[0], a[1]).w;
                    }
                }
                assert_eq!(ref_obj, obj);
            }
        }
        Ok(())
    }
}
