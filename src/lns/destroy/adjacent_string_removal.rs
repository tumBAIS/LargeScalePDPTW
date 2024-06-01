/// Based on from Christiaens, J., & Vanden Berghe, G. (2020)

use clap::ValueEnum;
use fixedbitset::{FixedBitSet, Union};
use rand::seq::IteratorRandom;
use rand::Rng;

use crate::problem::pdptw::{NodeType, PDPTWInstance};
use crate::problem::Num;
use crate::solution::Solution;
#[cfg(feature = "search_assertions")]
use crate::utils::validator::assert_valid_solution;
use crate::utils::Random;

#[derive(ValueEnum, Clone, Copy, Debug)]
pub enum AdjacencyMeasure {
    Classic,
    DistanceBetweenPickupsAndBetweenDeliveries,
    Detour,
    StaticRelatedness,
}

pub struct AdjacentStringRemoval<'a> {
    instance: &'a PDPTWInstance,
    adj: Vec<Vec<usize>>,
    alpha: f64,
    beta: f64,
    max_string_cardinality: usize,
}

impl<'a> AdjacentStringRemoval<'a> {
    pub fn new(
        instance: &'a PDPTWInstance,
        adj_measure: AdjacencyMeasure,
        max_string_cardinality: usize,
        alpha: f64,
        beta: f64,
    ) -> Self {
        let adj = match adj_measure {
            AdjacencyMeasure::Classic => instance
                .iter_pickups()
                .map(|from| {
                    let mut related: Vec<usize> = instance.iter_pickups().map(|to| to.id).collect();
                    related.sort_by_cached_key(|to| {
                        instance.distance(from.id, *to)
                    });
                    related
                })
                .collect(),
            AdjacencyMeasure::DistanceBetweenPickupsAndBetweenDeliveries => instance
                .iter_pickups()
                .map(|from| {
                    let mut related: Vec<usize> = instance.iter_pickups().map(|to| to.id).collect();
                    related.sort_by_cached_key(|to| {
                        instance.distance(from.id, *to) + instance.distance(from.id + 1, *to + 1)
                    });
                    related
                })
                .collect(),
            AdjacencyMeasure::Detour => {
                let relatedness = |p1: usize, p2: usize| -> Num {
                    let d1 = instance.nodes[p1 + 1].id;
                    let d2 = instance.nodes[p2 + 1].id;

                    let sequential = instance.distance(p1, d1)
                        + instance.distance(d1, p2)
                        + instance.distance(p2, d2);

                    let p1p2d1d2 = instance.distance(p1, p2)
                        + instance.distance(p2, d1)
                        + instance.distance(d1, d2);

                    let p1p2d2d1 = instance.distance(p1, p2)
                        + instance.distance(p2, d2)
                        + instance.distance(d2, d1);

                    sequential.min(p1p2d1d2).min(p1p2d2d1)
                };

                instance
                    .iter_pickups()
                    .map(|from| {
                        let mut related: Vec<usize> =
                            instance.iter_pickups().map(|to| to.id).collect();
                        related.sort_by_cached_key(|to| relatedness(from.id, *to));
                        related
                    })
                    .collect()
            }
            AdjacencyMeasure::StaticRelatedness => {
                let relatedness = |p1: usize, p2: usize| -> Num {
                    let gamma = Num::from(9.0);
                    let xi = Num::from(3.0);
                    let phi = Num::from(2.0);
                    let _omega = Num::from(5.0);

                    let distance_and_time_p1_p2 = instance.distance_and_time(p1, p2);
                    let distance_and_time_d1_d2 = instance.distance_and_time(p1 + 1, p2 + 1);

                    let rel_distance =
                        distance_and_time_p1_p2.distance + distance_and_time_d1_d2.distance;
                    let rel_time = (instance.nodes[p1].ready - instance.nodes[p1].due).abs()
                        + (instance.nodes[p1 + 1].ready - instance.nodes[p1 + 1].due);
                    let rel_capacity =
                        Num::from((instance.nodes[p1].demand - instance.nodes[p1].demand).abs());

                    gamma * rel_distance + xi * rel_time + phi * rel_capacity
                };

                instance
                    .iter_pickups()
                    .map(|from| {
                        let mut related: Vec<usize> =
                            instance.iter_pickups().map(|to| to.id).collect();
                        related.sort_by_cached_key(|to| relatedness(from.id, *to));
                        related
                    })
                    .collect()
            }
        };
        Self {
            instance,
            adj,
            alpha,
            beta,
            max_string_cardinality,
        }
    }

    pub fn destroy(&self, solution: &mut Solution, rng: &mut Random, num: usize) {
        self.remove_adjacent_strings_of(
            self.select_random_pickup(solution, rng),
            solution,
            rng,
            num,
            self.max_string_cardinality, // maximum string cardinality
        );
    }
    //
    // pub fn destroy_and_return_affected_routes(
    //     &self,
    //     solution: &mut Solution,
    //     rng: &mut Random,
    //     num: usize,
    // ) -> FixedBitSet {
    //     self.remove_adjacent_strings_of(
    //         self.select_random_pickup(solution, rng),
    //         solution,
    //         rng,
    //         num,
    //         10, // maximum string cardinality
    //     )
    // }

    #[allow(non_snake_case)]
    pub fn remove_adjacent_strings_of(
        &self,
        p: usize,
        solution: &mut Solution,
        rng: &mut Random,
        c_bar: usize, // average number of removed customers ̄c (10)
        L_max: usize, // maximum string cardinality
    ) -> FixedBitSet {
        // from Christiaens, J., & Vanden Berghe, G. (2020)
        // let L_max = 10; // maximum string cardinality
        // let L_max = 10;
        // let L_max = 25;
        // let alpha = 0.25;

        // maximum string cardinality l max
        // s is initially set to the minimum of Lmax and the solution’s
        // average tour cardinality |t ∈ T|
        let average_tour_cardinality = solution
            .iter_route_ids()
            .map(|route_id| solution.number_customer_nodes_assigned_to_route(route_id))
            .sum::<usize>() as f64
            / solution.iter_route_ids().count() as f64;
        let l_s_max = (L_max as f64).min(average_tour_cardinality);
        let k_s_max = ((4.0 * c_bar as f64) / (1.0 + l_s_max)) - 1.0;
        let k_s = rng.gen_range(0.0..=k_s_max).floor() as usize + 1;

        let is_unassigned = solution.unassigned_requests.contains(p);
        let mut R = FixedBitSet::with_capacity(self.instance.num_vehicles);
        let mut R_len = 0;

        for adj_p in self.adj[p / 2 - self.instance.num_vehicles]
            .iter()
            .skip(if is_unassigned { 1 } else { 0 })
        {
            let t = solution.fw_data[*adj_p].vn_id / 2;
            if solution.unassigned_requests.contains(*adj_p) || R.contains(t) {
                continue;
            } else {
                // (1) (5−) Strings are removed as described in Section 5.2. In addition, the associated pickup/delivery locations that were not included by the
                // string are also removed.
                let l_t_max =
                    l_s_max.min(solution.number_customer_nodes_assigned_to_route(t) as f64);
                let l_t = rng.gen_range(0.0..=l_t_max).floor() as usize + 1;
                if l_t_max.floor() as usize == l_t || rng.gen_range(0.0..1.0) < self.alpha {
                    // if rng.gen_range(0.0..1.0) < alpha {
                    self.string_procedure(solution, rng, *adj_p, t, l_t);
                } else {
                    self.split_string_procedure(solution, rng, *adj_p, t, l_t);
                }
                R.insert(t);
                R_len += 1;
            }

            if R_len >= k_s {
                break;
            }
        }

        R
    }

    fn select_random_pickup(&self, _solution: &Solution, rng: &mut Random) -> usize {
        // let i = rng.gen_range(0..self.instance.num_requests - solution.unassigned_requests.count());
        self.instance
            .iter_pickups()
            // .filter(|pickup| !solution.unassigned_requests.contains(pickup.id))
            .choose(rng)
            .unwrap()
            .id
        // self.instance.iter_pickups().choose(rng).unwrap().id
        // if solution.number_of_unassigned_requests() > 0 {
        // if rng.gen_range(0..self.instance.num_requests) < solution.unassigned_requests.count() {
        //     solution
        //         .unassigned_requests
        //         .iter_pickup_ids()
        //         .choose(rng)
        //         .unwrap()
        // } else {
        //     self.instance.iter_pickups().choose(rng).unwrap().id
        // }
    }

    fn string_procedure(
        &self,
        solution: &mut Solution,
        rng: &mut Random,
        p_id: usize,
        t: usize,
        l_t: usize,
    ) -> usize {
        // string procedure - removes a randomly selected string of cardinality l t
        //  in tour t, which includes customer c∗
        let mut request_parts_removed = RemovedRequestParts::new(self.instance.num_requests);
        let vn_id = t * 2;
        let mut num_nodes_removed = 0;
        let mut node = p_id;
        let (first, last) = loop {
            request_parts_removed.remove(
                self.instance.request_id(node),
                self.instance.node_type(node),
            );
            let (pred, succ) = solution.relink_gap_when_removing_node(node);
            num_nodes_removed += 1;
            if pred == vn_id {
                node = succ;
            } else if succ == vn_id + 1 {
                node = pred
            } else {
                if rng.gen_range(0..=1) == 0 {
                    node = pred
                } else {
                    node = succ
                }
            }
            if num_nodes_removed >= l_t {
                break (pred, succ);
            }
        };

        let union = FixedBitSet::from_iter(request_parts_removed.union());

        let (mut requests_deliveries_missing, mut requests_pickups_missing) = (
            &mut request_parts_removed.requests_pickups_removed,
            &mut request_parts_removed.requests_delivery_removed,
        );

        {
            let tmp = requests_deliveries_missing.clone();
            requests_deliveries_missing.difference_with(&requests_pickups_missing);
            requests_pickups_missing.difference_with(&tmp);
        }

        // search forward for missing deliveries
        let last =
            self.process_second_part_forward(solution, &mut requests_deliveries_missing, last);

        // search backward for missing pickups
        let first =
            self.process_first_part_backwards(solution, &mut requests_pickups_missing, first);

        let num_requests_removed = union.count_ones(..);
        // self.track_unassigned_requests(solution, &request_parts_removed.requests_pickups_removed);
        // self.track_unassigned_requests(solution, &request_parts_removed.requests_delivery_removed);
        self.track_unassigned_requests(solution, &union);

        solution.validate_between(first, last);
        // solution.validate_between(t * 2, t * 2 + 1);

        num_requests_removed
    }

    fn split_string_procedure(
        &self,
        solution: &mut Solution,
        rng: &mut Random,
        p_id: usize,
        t: usize,
        l_t: usize,
    ) -> usize {
        // from Christiaens, J., & Vanden Berghe, G. (2020) - Table 5
        let cardinality_t = solution.number_customer_nodes_assigned_to_route(t);
        // let m = if cardinality_t > 0 {
        //     Self::calculate_m(beta, cardinality_t - l_t, rng)
        // } else {
        //     0
        // };
        let m = Self::calculate_m(self.beta, cardinality_t - l_t, rng);
        if m == 0 {
            return self.string_procedure(solution, rng, p_id, t, l_t);
        }
        // from hereon out we assume that m > 0
        // println!("split_string_procedure with m = {}", m);
        // println!("starting string: {}", solution.route_as_string(t));

        // split string procedure - randomly selecting a string of cardinality l + m
        //  that includes customer c∗; the ruin phase bypasses and preserves a random
        //  substring of m intervening customers.

        // first find the subset of l_t + m nodes around c
        let vn_id = t * 2;
        let (first_exclusive, last_exclusive) =
            Self::select_string_from_initial_node(solution, rng, l_t, m, vn_id, p_id);
        // println!(
        //     "string to remove selected: {}",
        //     solution.subsequence_as_string(
        //         solution.succ(first_exclusive),
        //         solution.pred(last_exclusive)
        //     )
        // );

        let start_of_substring_m = rng.gen_range(0..l_t);
        let (
            first_in_m,
            first_half_request_parts_removed,
            last_in_m,
            second_half_request_parts_removed,
        ) = self.process_split_string(
            solution,
            first_exclusive,
            last_exclusive,
            l_t,
            start_of_substring_m,
        );

        // println!("after first removal: {}", solution.route_as_string(t));

        let all_requests_removed = FixedBitSet::from_iter(
            first_half_request_parts_removed
                .union()
                .chain(second_half_request_parts_removed.union()),
        );

        // go forward first through the m-string
        let mut requests_deliveries_missing = first_half_request_parts_removed
            .requests_pickups_removed
            .clone();
        requests_deliveries_missing
            .difference_with(&first_half_request_parts_removed.requests_delivery_removed);

        // search forward for missing deliveries
        self.process_first_part_forward(
            solution,
            &mut requests_deliveries_missing,
            first_in_m,
            last_in_m,
        );

        let closed_m_completely = solution.succ(first_exclusive) == last_exclusive;

        // add the pickups from the second half
        requests_deliveries_missing
            .union_with(&second_half_request_parts_removed.requests_pickups_removed);
        requests_deliveries_missing
            .difference_with(&second_half_request_parts_removed.requests_delivery_removed);

        // now continue with the remain of the route
        let last = self.process_second_part_forward(
            solution,
            &mut requests_deliveries_missing,
            last_exclusive,
        );

        // now in reverse
        // go backwards through the m-string
        let mut requests_pickup_missing = second_half_request_parts_removed
            .requests_delivery_removed
            .clone();
        requests_pickup_missing
            .difference_with(&second_half_request_parts_removed.requests_pickups_removed);

        // check whether m was removed (only contained deliveries from the first part)
        if closed_m_completely {
            // nothing to be done
        } else {
            // search backwards for missing pickups
            self.process_second_part_backward(
                solution,
                &mut requests_pickup_missing,
                first_exclusive,
                last_in_m,
            )
        }
        // add the pickups from the second half
        requests_pickup_missing
            .union_with(&first_half_request_parts_removed.requests_delivery_removed);
        requests_pickup_missing
            .difference_with(&first_half_request_parts_removed.requests_pickups_removed);

        // now continue with the remain of the route string
        // println!("now continue with the remain of the route string");
        let first = self.process_first_part_backwards(
            solution,
            &mut requests_pickup_missing,
            first_exclusive,
        );

        self.track_unassigned_requests(solution, &all_requests_removed);

        let num_requests_removed = first_half_request_parts_removed
            .requests_pickups_removed
            .count_ones(..)
            + second_half_request_parts_removed
            .requests_pickups_removed
            .count_ones(..)
            + requests_pickup_missing.count_ones(..);

        solution.validate_between(first, last);

        #[cfg(feature = "search_assertions")]
        assert_valid_solution(self.instance, &solution);

        num_requests_removed
    }

    fn process_first_part_backwards(
        &self,
        solution: &mut Solution,
        requests_pickup_missing: &mut FixedBitSet,
        mut first: usize,
    ) -> usize {
        while requests_pickup_missing.count_ones(..) != 0 {
            let node = first;
            first = solution.pred(node);
            debug_assert_ne!(node, first);
            if self.instance.is_pickup(node)
                && requests_pickup_missing.contains(self.instance.request_id(node))
            {
                // if self.instance.is_pickup(node) {
                //     solution.track_request_unassigned(node);
                // }
                solution.relink_gap_when_removing_node(node);
                requests_pickup_missing.set(self.instance.request_id(node), false);
            }
        }
        first
    }

    fn process_second_part_backward(
        &self,
        solution: &mut Solution,
        requests_pickup_missing: &mut FixedBitSet,
        first: usize,
        last_in_m: usize,
    ) {
        let mut next = last_in_m;
        while next != first && requests_pickup_missing.count_ones(..) != 0 {
            let node = next;
            next = solution.pred(node);
            debug_assert_ne!(node, next);
            if self.instance.is_pickup(node)
                && requests_pickup_missing.contains(self.instance.request_id(node))
            {
                // if self.instance.is_pickup(node) {
                //     solution.track_request_unassigned(node);
                // }
                solution.relink_gap_when_removing_node(node);
                requests_pickup_missing.set(self.instance.request_id(node), false);
            }
        }
    }

    fn process_second_part_forward(
        &self,
        solution: &mut Solution,
        requests_deliveries_missing: &mut FixedBitSet,
        mut last: usize,
    ) -> usize {
        while requests_deliveries_missing.count_ones(..) != 0 {
            let node = last;
            last = solution.succ(node);
            debug_assert_ne!(
                node,
                last,
                "{:?}",
                requests_deliveries_missing
                    .ones()
                    .map(|it| self.instance.pickup_id_of_request(it))
                    .collect::<Vec<usize>>()
            );
            if self.instance.is_delivery(node)
                && requests_deliveries_missing.contains(self.instance.request_id(node))
            {
                solution.relink_gap_when_removing_node(node);
                requests_deliveries_missing.set(self.instance.request_id(node), false);
            }
        }
        last
    }

    fn process_first_part_forward(
        &self,
        solution: &mut Solution,
        requests_deliveries_missing: &mut FixedBitSet,
        first_in_m: usize,
        last_in_m: usize,
    ) {
        let mut next = first_in_m;
        while requests_deliveries_missing.count_ones(..) != 0 {
            let node = next;
            next = solution.succ(node);
            debug_assert_ne!(node, next);
            if self.instance.is_delivery(node)
                && requests_deliveries_missing.contains(self.instance.request_id(node))
            {
                solution.relink_gap_when_removing_node(node);
                requests_deliveries_missing.set(self.instance.request_id(node), false);
            }
            // first_in_m == last_in_m -> consider only a single node
            if node == last_in_m {
                break;
            }
        }
    }

    fn process_split_string(
        &self,
        solution: &mut Solution,
        first: usize,
        last: usize,
        l_t: usize,
        start_of_substring_m: usize,
    ) -> (usize, RemovedRequestParts, usize, RemovedRequestParts) {
        // println!("m starts at {}", start_of_substring_m);
        // process the first part of the substring before m (could be empty)
        let (first_in_m, first_half_request_parts_removed) =
            self.initialize_first_part(solution, first, start_of_substring_m);

        // if start_of_substring_m == 0 -> ``first`` and ``first_in_m`` are already connected
        debug_assert!(start_of_substring_m != 0 || solution.succ(first) == first_in_m);
        if solution.succ(first) != first_in_m {
            // dbg!(first, first_in_m);
            solution.link_nodes(first, first_in_m);
        }

        // process the second part of the substring after m (could be empty as well)
        let (last_in_m, second_half_request_parts_removed) =
            self.initialize_second_part(solution, l_t, last, start_of_substring_m);

        if solution.pred(last) != last_in_m {
            // dbg!(last, last_in_m);
            solution.link_nodes(last_in_m, last);
        }
        (
            first_in_m,
            first_half_request_parts_removed,
            last_in_m,
            second_half_request_parts_removed,
        )
    }

    fn initialize_second_part(
        &self,
        solution: &mut Solution,
        l_t: usize,
        last: usize,
        start_of_substring_m: usize,
    ) -> (usize, RemovedRequestParts) {
        let mut second_half_request_parts_removed =
            RemovedRequestParts::new(self.instance.num_requests);
        let mut last_in_m = solution.pred(last);

        for _ in start_of_substring_m..l_t {
            second_half_request_parts_removed.remove(
                self.instance.request_id(last_in_m),
                self.instance.node_type(last_in_m),
            );
            last_in_m = solution.pred(last_in_m);
        }
        (last_in_m, second_half_request_parts_removed)
    }

    fn initialize_first_part(
        &self,
        solution: &mut Solution,
        first: usize,
        start_of_substring_m: usize,
    ) -> (usize, RemovedRequestParts) {
        let mut first_half_request_parts_removed =
            RemovedRequestParts::new(self.instance.num_requests);
        let mut first_in_m = solution.succ(first);

        for _ in 0..start_of_substring_m {
            first_half_request_parts_removed.remove(
                self.instance.request_id(first_in_m),
                self.instance.node_type(first_in_m),
            );
            first_in_m = solution.succ(first_in_m);
        }
        (first_in_m, first_half_request_parts_removed)
    }

    fn select_string_from_initial_node(
        solution: &mut Solution,
        rng: &mut Random,
        l_t: usize,
        m: usize,
        vn_id: usize,
        initial_node: usize,
    ) -> (usize, usize) {
        let mut selected = 1;
        let (mut first, mut last) = solution.pred_succ_pair(initial_node);

        loop {
            if first == vn_id {
                last = solution.succ(last);
            } else if last == vn_id + 1 {
                first = solution.pred(first);
            } else {
                if rng.gen_range(0..=1) == 0 {
                    first = solution.pred(first);
                } else {
                    last = solution.succ(last);
                }
            }
            selected += 1;
            if selected >= l_t + m {
                break;
            }
        }
        (first, last)
    }

    fn calculate_m(beta: f64, m_max: usize, rng: &mut Random) -> usize {
        // The number of preserved customers m is determined as follows. Initially
        // m = 1, and its current value is maintained either if a random number
        // U(0, 1) is larger than β, which is referred to as the split depth, or once
        // the maximum value for m is reached (m = m^max = |t| − l). If neither of these
        // conditions is satisfied, m is incremented (m = m + 1), and the process repeats
        if m_max > 0 {
            let mut m = 1;
            while m < m_max && rng.gen_bool(1.0 - beta) {
                m += 1;
            }
            m
        } else {
            0
        }
    }

    fn track_unassigned_requests(
        &self,
        solution: &mut Solution,
        request_ids_removed: &FixedBitSet,
    ) {
        for request_id in request_ids_removed.ones() {
            solution.track_request_unassigned(self.instance.pickup_id_of_request(request_id));
        }
    }
}

struct RemovedRequestParts {
    pub requests_pickups_removed: FixedBitSet,
    pub requests_delivery_removed: FixedBitSet,
}

impl RemovedRequestParts {
    fn new(num_requests: usize) -> Self {
        Self {
            requests_pickups_removed: FixedBitSet::with_capacity(num_requests),
            requests_delivery_removed: FixedBitSet::with_capacity(num_requests),
        }
    }
    fn pickup_removed(&mut self, request_id: usize) {
        self.requests_pickups_removed.insert(request_id);
    }
    fn delivery_removed(&mut self, request_id: usize) {
        self.requests_delivery_removed.insert(request_id);
    }

    fn remove(&mut self, request_id: usize, node_type: &NodeType) {
        match node_type {
            NodeType::Pickup => {
                self.pickup_removed(request_id);
            }
            NodeType::Delivery => {
                self.delivery_removed(request_id);
            }
            _ => panic!(),
        }
    }

    fn union(&self) -> Union {
        self.requests_pickups_removed
            .union(&self.requests_delivery_removed)
    }
}
