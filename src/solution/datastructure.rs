use rand::Rng;

use crate::problem::Num;
use crate::refn::{ConcatREF, ForwardREF, REFNode};
use crate::solution::{BestInsertion, PDInsertion, Solution};
#[cfg(feature = "search_assertions")]
use crate::utils::validator::{validate_route, ValidatorResult};
use crate::utils::Random;

pub struct UpdateBounds {
    pub vn: usize,
    pub pred_first: usize,
    pub succ_last: usize,
}

impl UpdateBounds {
    pub fn complete_route(vn: usize) -> Self {
        Self {
            vn,
            pred_first: vn,
            succ_last: vn + 1,
        }
    }
}

impl<'a> Solution<'a> {
    pub fn node(&self, node_id: usize) -> &REFNode {
        &self.fw_data[node_id].node
    }
    pub fn pred(&self, node_id: usize) -> usize {
        self.fw_data[node_id].pred
    }
    pub fn succ(&self, node_id: usize) -> usize {
        self.fw_data[node_id].succ
    }
    pub fn pred_succ_pair(&self, node_id: usize) -> (usize, usize) {
        let n = &self.fw_data[node_id];
        (n.pred, n.succ)
    }
    pub fn vn_id(&self, node_id: usize) -> usize {
        self.fw_data[node_id].vn_id
    }

    pub fn relink(&mut self, vn_id: usize, node_id: usize, pred: usize, succ: usize) {
        self.fw_data.relink(vn_id, node_id, pred, succ);
        self.bw_data.relink(vn_id, node_id, pred, succ);
    }

    pub(crate) fn link_nodes(&mut self, n1: usize, n2: usize) {
        self.fw_data[n1].succ = n2;
        self.fw_data[n2].pred = n1;
        self.bw_data[n1].succ = n2;
        self.bw_data[n2].pred = n1;
    }

    pub(crate) fn relink_when_inserting_pd(
        &mut self,
        vn_id: usize,
        pickup_id: usize,
        pickup_after: usize,
        delivery_before: usize,
    ) -> (usize, usize) {
        let delivery_id = pickup_id + 1;
        if self.fw_data[pickup_after].succ == delivery_before {
            self.fw_data
                .relink(vn_id, pickup_id, pickup_after, delivery_id);
            self.fw_data
                .relink(vn_id, delivery_id, pickup_id, delivery_before);
        } else {
            self.fw_data
                .relink(vn_id, pickup_id, pickup_after, self.succ(pickup_after));
            self.fw_data.relink(
                vn_id,
                delivery_id,
                self.pred(delivery_before),
                delivery_before,
            );
        }
        (pickup_after, delivery_before)
    }

    pub(crate) fn relink_gap_when_removing_node(&mut self, node: usize) -> (usize, usize) {
        let (pred, succ) = self.pred_succ_pair(node);
        self.link_nodes(pred, succ);
        (pred, succ)
    }

    pub(crate) fn relink_gap_when_removing_pd(&mut self, pickup_id: usize) -> (usize, usize) {
        let p_pred = self.fw_data[pickup_id].pred;
        let d_succ = self.fw_data[pickup_id + 1].succ;
        if self.fw_data[pickup_id].succ == pickup_id + 1 {
            self.link_nodes(p_pred, d_succ);
        } else {
            self.link_nodes(p_pred, self.fw_data[pickup_id].succ);
            self.link_nodes(self.fw_data[pickup_id + 1].pred, d_succ);
        }
        (p_pred, d_succ)
    }

    pub fn clear(&mut self) {
        self.fw_data.reset(self.instance);
        self.bw_data.reset(self.instance);
    }

    pub fn set(&mut self, itineraries: &Vec<Vec<usize>>) {
        // relink vehicle nodes
        for i in 0..self.instance.num_vehicles {
            self.link_nodes(i * 2, i * 2 + 1);
        }

        self.empty_route_ids.insert_range(..);
        self.unassigned_requests.set_all();
        self.blocks.invalidate_all();

        for route in itineraries {
            let vehicle_node = route[0];
            let vn_id = self.fw_data[vehicle_node].vn_id;
            self.empty_route_ids.set(vn_id / 2, false);

            let mut prev_id = route[0];
            for i in 1..(route.len() - 1) {
                let node_id = route[i];
                debug_assert_eq!(self.fw_data[node_id].node.id, node_id);
                self.fw_data[node_id].relink(vn_id, prev_id, route[i + 1]);
                self.fw_data
                    .extend_forward_unchecked(prev_id, node_id, self.instance);

                if node_id % 2 == 0 {
                    self.unassigned_requests.remove(node_id);
                }
                prev_id = node_id;
            }

            prev_id = vn_id + 1;
            for i in (1..(route.len() - 1)).rev() {
                let node_id = route[i];
                self.bw_data[node_id].relink(vn_id, route[i - 1], prev_id);
                self.bw_data
                    .extend_backward_unchecked(prev_id, node_id, self.instance);
                prev_id = node_id;
            }

            // first/last node
            debug_assert_eq!(*route.last().unwrap(), route[0] + 1);

            self.fw_data[vn_id].succ = route[1];
            self.fw_data[vn_id + 1].pred = route[route.len() - 2];
            self.fw_data
                .extend_forward_unchecked(route[route.len() - 2], vn_id + 1, self.instance);

            self.bw_data[vn_id].succ = route[1];
            self.bw_data[vn_id + 1].pred = route[route.len() - 2];
            self.revalidate_blocks(vn_id);
        }
    }

    // this function assumes that the nodes of the route do not change!
    pub fn update_route_sequence(&mut self, route: &Vec<usize>) {
        let vn_id = route[0];

        let mut prev_id = route[0];
        for i in 1..(route.len() - 1) {
            let node_id = route[i];
            debug_assert_eq!(self.fw_data[node_id].node.id, node_id);
            self.fw_data[node_id].relink(vn_id, prev_id, route[i + 1]);
            self.fw_data
                .extend_forward_unchecked(prev_id, node_id, self.instance);

            prev_id = node_id;
        }

        prev_id = vn_id + 1;
        for i in (1..(route.len() - 1)).rev() {
            let node_id = route[i];
            self.bw_data[node_id].relink(vn_id, route[i - 1], prev_id);
            self.bw_data
                .extend_backward_unchecked(prev_id, node_id, self.instance);
            prev_id = node_id;
        }

        // first/last node
        debug_assert_eq!(*route.last().unwrap(), route[0] + 1);

        self.fw_data[vn_id].succ = route[1];
        self.fw_data[vn_id + 1].pred = route[route.len() - 2];
        self.fw_data
            .extend_forward_unchecked(route[route.len() - 2], vn_id + 1, self.instance);

        self.bw_data[vn_id].succ = route[1];
        self.bw_data[vn_id + 1].pred = route[route.len() - 2];

        // init blocks
        self.revalidate_blocks(vn_id);
    }

    pub fn validate_between(&mut self, pickup_after: usize, delivery_before: usize) {
        let vn_id = self.fw_data[pickup_after].vn_id;
        self.partially_validate_between(
            pickup_after,
            delivery_before,
            &UpdateBounds {
                vn: vn_id,
                pred_first: vn_id,
                succ_last: vn_id + 1,
            },
        )
    }

    pub fn partially_validate_between(&mut self, first: usize, last: usize, bound: &UpdateBounds) {
        let vn_id = self.fw_data[first].vn_id;
        let mut prev_id = first;
        while prev_id != bound.succ_last {
            let node_id = self.fw_data[prev_id].succ;
            debug_assert_ne!(prev_id, node_id);
            self.fw_data
                .extend_forward_unchecked(prev_id, node_id, self.instance);
            self.fw_data[node_id].vn_id = vn_id;
            prev_id = node_id;
        }
        // update bw
        // prev_id == delivery_before
        let until = self.fw_data[bound.pred_first].succ;
        let mut next_id = last;
        while next_id != until {
            let node_id = self.bw_data[next_id].pred;
            self.bw_data
                .extend_backward_unchecked(next_id, node_id, self.instance);
            self.bw_data[node_id].vn_id = vn_id;
            next_id = node_id;
        }

        self.empty_route_ids
            .set(vn_id / 2, self.succ(vn_id) == vn_id + 1);

        self.revalidate_blocks(vn_id);
    }

    pub(crate) fn revalidate_blocks(&mut self, vn_id: usize) {
        let mut block_start = self.succ(vn_id);
        while block_start != vn_id + 1 {
            self.blocks.set_block_valid(block_start);
            self.blocks[block_start].first_node_id = block_start;
            self.blocks[block_start]
                .data
                .reset_with_node(&self.fw_data[block_start].node);
            let mut open_pickups = 1;

            let mut prev_id = block_start;
            while open_pickups != 0 {
                let node_id = self.succ(prev_id);
                self.blocks.invalidate_block(node_id);
                let dist_time = self.instance.distance_and_time(prev_id, node_id);
                self.blocks[block_start]
                    .data
                    .extend_forward_assign(&self.fw_data[node_id].node, dist_time);

                if self.instance.is_pickup(node_id) {
                    open_pickups += 1;
                } else {
                    debug_assert!(self.instance.is_delivery(node_id));
                    open_pickups -= 1;
                }

                prev_id = node_id;
                // we assume that the predecessor constraint is never violated, therefore we will
                //  reach the corresponding delivery for every pickup encountered, reaching a state
                //  where the vehicle is empty before encountering (vn_id + 1)-
                debug_assert_ne!(prev_id, vn_id + 1);
            }

            // prev is now the last delivery node of the block
            self.blocks[block_start].last_node_id = prev_id;

            #[cfg(feature = "trace-refdata")]
            {
                self.blocks[block_start].trace = self.blocks[block_start].data.trace.clone();
            }

            block_start = self.succ(prev_id);
        }
        self.blocks[vn_id]
            .data
            .reset_with_node(&self.fw_data[vn_id].node);
        self.blocks[vn_id + 1]
            .data
            .reset_with_node(&self.fw_data[vn_id + 1].node);
    }

    pub fn is_route_feasible(&self, r_id: usize) -> bool {
        self.fw_data[self.instance.vn_id_of(r_id)].data.tw_feasible
    }

    pub fn is_route_empty(&self, r_id: usize) -> bool {
        self.fw_data[self.instance.vn_id_of(r_id)].succ == self.instance.vn_id_of(r_id) + 1
    }

    pub(crate) fn assert_links_aligned(&self, vn_id: usize) {
        let mut node_id = vn_id;
        while node_id != vn_id + 1 {
            assert_eq!(self.fw_data[node_id].vn_id, vn_id);
            assert_eq!(self.fw_data[node_id].vn_id, self.bw_data[node_id].vn_id);
            assert_eq!(self.fw_data[node_id].succ, self.bw_data[node_id].succ);
            assert_eq!(self.fw_data[node_id].pred, self.bw_data[node_id].pred);
            node_id = self.fw_data[node_id].succ;
        }
    }

    pub(crate) fn assert_sane(&self) {
        for v_id in 0..self.instance.num_vehicles {
            self.assert_links_aligned(v_id * 2);
        }
        for node in &self.instance.nodes {
            if node.node_type.is_pickup() && !self.unassigned_requests.contains(node.id) {
                assert!(
                    self.route_contains_node_id(self.fw_data[node.id].vn_id / 2, node.id),
                    "node {} not found in route {}",
                    node.id,
                    self.fw_data[node.id].vn_id / 2
                );
                assert!(
                    self.route_contains_node_id(self.fw_data[node.id].vn_id / 2, node.id + 1),
                    "node {} not found in route {}",
                    node.id + 1,
                    self.fw_data[node.id].vn_id / 2
                );
            }
        }
    }

    pub fn route_contains_node_id(&self, route_id: usize, node_id: usize) -> bool {
        self.iter_route(route_id)
            .find(|it| *it == node_id)
            .is_some()
    }
}

impl Solution<'_> {
    #[inline(always)]
    pub fn find_best_insert_for_request_in_route(
        &self,
        p_id: usize,
        vn_id: usize,
        rng: &mut Random,
    ) -> BestInsertion {
        self.find_best_insert_for_request_in_route_with_blinks(p_id, vn_id, rng, 0.0)
    }

    pub fn find_best_insert_for_request_in_route_with_blinks(
        &self,
        p_id: usize,
        vn_id: usize,
        rng: &mut Random,
        blink_rate: f64,
    ) -> BestInsertion {
        let mut best_insertion: Option<PDInsertion> = None;
        let mut best_obj = Num::max_value(); // some large number

        // filtering - only try relocate positions which can be reached in time
        let v = self.instance.vehicle_from_vn_id(vn_id);
        let p = &self.fw_data[p_id];
        let d = &self.fw_data[p_id + 1];

        let mut next_ins_pos = vn_id;
        while next_ins_pos != vn_id + 1 {

            // position in v2 after which we try to relocate p1
            let insert_pos = next_ins_pos;
            let before = &self.fw_data[insert_pos];
            next_ins_pos = before.succ;

            debug_assert_ne!(insert_pos, next_ins_pos);

            let dist_time = self.instance.distance_and_time(insert_pos, p_id);

            // can we reach p1 in time?
            if before.data.earliest_completion + dist_time.time > p.node.due {
                continue;
            }

            let mut tmp = self.fw_data[insert_pos]
                .data
                .extend_forward(&p.node, dist_time);

            if !v.check_capacity(tmp.current_load) {
                continue;
            }

            let mut prev = p;
            let mut next = &self.fw_data[before.succ];
            // we now check whether we can insert d1 before the next node, iterating
            //  forward until we hit the urt limit of p1-d1
            while prev.node.id != vn_id + 1 {
                debug_assert_ne!(prev.node.id, next.node.id);

                // try blinking
                if blink_rate > 0.0 && rng.gen_bool(blink_rate) {
                    // ignore insertion point
                } else {
                    // try to insert d1 and evaluate the route
                    let dist_time_tmp_d1 = self.instance.distance_and_time(prev.node.id, p_id + 1);
                    let dist_time_d1_next = self.instance.distance_and_time(p_id + 1, next.node.id);
                    let new_v2_data = tmp
                        .extend_forward(&d.node, dist_time_tmp_d1)
                        .concat(&self.bw_data[next.node.id].data, dist_time_d1_next);

                    if new_v2_data.check_feasible(v) {
                        // calculate change, see whether the move makes sense objective-wise,
                        //  i.e., could it be the best improving move?
                        if new_v2_data.distance < best_obj {
                            // it is the best one found yet, so take it
                            let ins = PDInsertion {
                                vn_id,
                                pickup_id: p_id,
                                pickup_after: insert_pos,
                                delivery_before: next.node.id,
                            };

                            best_insertion = Some(ins);
                            best_obj = new_v2_data.distance;
                        }
                    }
                }
                // then extend by the next node and continue with the one after
                let dist_time = self.instance.distance_and_time(prev.node.id, next.node.id);
                tmp.extend_forward_assign(&next.node, dist_time);
                if !tmp.check_feasible(v) {
                    break;
                }

                prev = next;
                next = &self.fw_data[prev.succ];
            }
        }

        match best_insertion {
            Some(ins) => BestInsertion::Some(ins, best_obj),
            _ => BestInsertion::None,
        }
    }

    pub fn insert(&mut self, insertion: PDInsertion) {
        let PDInsertion {
            pickup_id,
            pickup_after,
            delivery_before,
            ..
        } = insertion;
        debug_assert!(self.unassigned_requests.contains(insertion.pickup_id));

        debug_assert_eq!(
            self.fw_data[pickup_after].vn_id,
            self.fw_data[delivery_before].vn_id
        );

        let vn_id = self.fw_data[pickup_after].vn_id;
        let delivery_id = pickup_id + 1;

        let succ_pickup = self.fw_data[pickup_after].succ;
        self.relink(vn_id, pickup_id, pickup_after, succ_pickup);

        let pred_delivery = if delivery_before == succ_pickup {
            pickup_id
        } else {
            self.fw_data[delivery_before].pred
        };
        self.relink(vn_id, delivery_id, pred_delivery, delivery_before);

        self.unassigned_requests.remove(pickup_id);

        // recalculate auxiliary data for constant time feasibility check
        self.validate_between(pickup_after, delivery_before);

        #[cfg(feature = "search_assertions")]
        match validate_route(self.instance, &self.iter_route(vn_id / 2).collect(), None) {
            ValidatorResult::Valid(_) => {}
            ValidatorResult::ConstraintViolation(violation) => {
                println!("\n{:?}", self.iter_route(vn_id / 2).collect::<Vec<usize>>());
                panic!("{:?}", violation)
            }
            ValidatorResult::ObjectiveMismatch(_) => {}
        }
    }

    pub fn track_request_unassigned(&mut self, pickup_id: usize) {
        self.unassigned_requests.insert_pickup_id(pickup_id);
        self.blocks.invalidate_block(pickup_id);
        self.fw_data[pickup_id].succ = pickup_id;
        self.fw_data[pickup_id + 1].succ = pickup_id + 1;
    }

    pub fn unassign_request(&mut self, pickup_id: usize) {
        if !self.unassigned_requests.contains(pickup_id) {
            let (pred, succ) = self.relink_gap_when_removing_pd(pickup_id);
            self.track_request_unassigned(pickup_id);
            self.validate_between(pred, succ);
        }
    }

    pub fn unassign_complete_route(&mut self, route: usize) -> usize {
        let mut requests_unassigned = 0;
        let vn_id = route * 2;
        let mut next = self.succ(vn_id);
        while next != vn_id + 1 {
            let succ = self.succ(next);
            debug_assert_ne!(next, self.succ(next));
            // check for delivery as we have then already passed the pickup
            if self.instance.nodes[next].node_type.is_delivery() {
                self.track_request_unassigned(next - 1);
                requests_unassigned += 1;
            }
            next = succ;
        }
        self.link_nodes(vn_id, vn_id + 1);

        self.validate_between(vn_id, vn_id + 1);

        requests_unassigned
    }

    pub fn get_max_number_of_vehicles(&self) -> usize {
        self.max_num_vehicles_available
    }

    pub fn clamp_max_number_of_vehicles_to_current_fleet_size(&mut self) {
        self.max_num_vehicles_available = self.number_of_vehicles_used();
        self.empty_route_ids.set_range(.., false); // forcing vehicles no longer available
    }

    pub fn iter_route_ids<'a>(&'a self) -> impl Iterator<Item=usize> + 'a {
        (0..self.instance.num_vehicles).filter(|it| self.succ(it * 2) != it * 2 + 1)
    }

    pub fn iter_empty_route_ids<'a>(&'a self) -> impl Iterator<Item=usize> + 'a {
        self.empty_route_ids.ones().into_iter()
    }

    pub fn number_of_vehicles_used(&self) -> usize {
        self.iter_route_ids().count()
    }

    pub fn get_empty_route<'a>(&'a self) -> Option<usize> {
        self.empty_route_ids.ones().next()
    }

    pub fn calculate_change_on_removing_request_of(&self, pickup_id: usize) -> Num {
        let delivery_id = pickup_id + 1;
        if self.fw_data[pickup_id].succ == delivery_id {
            self.fw_data[self.succ(delivery_id)].data.distance
                - self.fw_data[self.pred(pickup_id)].data.distance
        } else {
            self.fw_data[self.succ(pickup_id)].data.distance
                - self.fw_data[self.pred(pickup_id)].data.distance
                + self.fw_data[self.succ(delivery_id)].data.distance
                - self.fw_data[self.pred(delivery_id)].data.distance
        }
    }
}

pub struct RouteIterator<'a> {
    ads: &'a Solution<'a>,
    next: Option<usize>,
    last: usize,
}

impl<'a> Iterator for RouteIterator<'a> {
    type Item = usize;
    fn next(&mut self) -> Option<Self::Item> {
        if let Some(u) = self.next {
            if u != self.last {
                self.next = Some(self.ads.succ(u));
            } else {
                self.next = None;
            }
            return Some(u);
        }
        None
    }
}

impl<'a> RouteIterator<'a> {
    fn new(ads: &'a Solution, first: usize, last: usize) -> Self {
        Self {
            ads,
            next: Some(first),
            last,
        }
    }
    fn empty(ads: &'a Solution) -> Self {
        Self {
            ads,
            next: None,
            last: 0,
        }
    }

    pub(crate) fn to_vec(self) -> Vec<usize> {
        self.collect()
    }
}

impl<'a> Solution<'a> {
    pub fn iter_route(&self, route_id: usize) -> RouteIterator {
        self.iter_route_by_vn_id(route_id * 2)
    }
    pub fn iter_customers_of_route(&self, route_id: usize) -> RouteIterator {
        RouteIterator::new(self, self.succ(route_id * 2), self.pred(route_id * 2 + 1))
    }
    pub fn iter_pickups_of_route(&self, route_id: usize) -> impl Iterator<Item=usize> + '_ {
        RouteIterator::new(self, self.succ(route_id * 2), self.pred(route_id * 2 + 1))
            .filter(|it| self.instance.is_pickup(*it))
    }
    pub fn iter_requests_of_route(&self, route_id: usize) -> impl Iterator<Item=usize> + '_ {
        self.iter_pickups_of_route(route_id)
            .map(|it| self.instance.request_id(it))
    }
    pub fn iter_route_by_vn_id(&self, vn_id: usize) -> RouteIterator {
        RouteIterator::new(self, vn_id, vn_id + 1)
    }
    pub fn number_customer_nodes_assigned_to_route(&self, route_id: usize) -> usize {
        self.iter_route(route_id).count() - 2
    }
    pub fn number_of_unassigned_requests(&self) -> usize {
        self.unassigned_requests.count()
    }
    pub fn route_as_string(&self, route_id: usize) -> String {
        format!(
            "{}",
            self.iter_route(route_id)
                .map(|it| format!("{}", it))
                .collect::<Vec<String>>()
                .join(",")
        )
    }
    pub fn iter_subsequence(&self, start_inclusive: usize, end_inclusive: usize) -> RouteIterator {
        RouteIterator::new(self, start_inclusive, end_inclusive)
    }
    pub fn subsequence_as_string(&self, start_inclusive: usize, end_inclusive: usize) -> String {
        format!(
            "{}",
            RouteIterator::new(self, start_inclusive, end_inclusive)
                .map(|it| format!("{}", it))
                .collect::<Vec<String>>()
                .join(",")
        )
    }
}


#[cfg(test)]
mod tests {}
