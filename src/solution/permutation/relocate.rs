use rand::seq::{IteratorRandom, SliceRandom};
use rand::Rng;

use crate::refn::{ConcatREF, ForwardREF};
use crate::solution::datastructure::UpdateBounds;
use crate::solution::permutation::moves::InterRelocateMove;
use crate::solution::permutation::ReservoirSampling;
use crate::solution::Solution;
use crate::utils::Random;

impl Solution<'_> {
    pub fn random_shift(&mut self, rng: &mut Random) -> bool {
        // the random shift selects a request u at random and moves it to another
        // route r_2 at random.
        let p = self
            .instance
            .iter_pickups()
            .filter(|p| !self.unassigned_requests.contains(p.id))
            .choose(rng);

        if let Some(p) = p {
            let p_id = p.id;
            let vn1_id = self.vn_id(p_id);
            let r_id = self
                .iter_route_ids()
                .filter(|r_id| *r_id != vn1_id / 2)
                .choose(rng);

            if let Some(r_id) = r_id {
                let insert = self
                    .find_random_insert_for_request_in_route_using_reservoir_sampling(
                        p_id,
                        r_id,
                        rng,
                        ReservoirSampling::new(),
                    )
                    .take();

                if let Some(insert) = insert {
                    self.apply_inter_relocate_move(InterRelocateMove {
                        vn1: vn1_id,
                        vn2: insert.vn_id,
                        p1: insert.pickup_id,
                        p1_after_node2: insert.pickup_after,
                        d1_before_node2: insert.delivery_before,
                    });
                    return true;
                }
            }
        }
        false
    }

    pub fn biased_random_shift(&mut self, rng: &mut Random, bias: f64) -> bool {
        // the random shift selects a request u at random and moves it to another
        // route r_2 at random.
        let p = self
            .instance
            .iter_pickups()
            .filter(|p| !self.unassigned_requests.contains(p.id))
            .choose(rng);

        if let Some(p) = p {
            let p_id = p.id;
            let vn1_id = self.vn_id(p_id);
            let r_id = self
                .iter_route_ids()
                .filter(|r_id| *r_id != vn1_id / 2)
                .choose(rng);

            if let Some(r_id) = r_id {
                let mut inserts = self.find_all_inserts_for_request_in_route(p_id, r_id);
                inserts.shuffle(rng);
                let num_inserts = inserts.len();
                if let Some((insert, _cost)) = inserts
                    .iter()
                    // From all possible insertions of u
                    // in r, a set of \mu% of them is taken at random and the best position
                    // is selected for the insertion.
                    .take((num_inserts as f64 * bias).ceil() as usize)
                    .min_by_key(|(_, cost)| cost)
                {
                    self.apply_inter_relocate_move(InterRelocateMove {
                        vn1: vn1_id,
                        vn2: insert.vn_id,
                        p1: insert.pickup_id,
                        p1_after_node2: insert.pickup_after,
                        d1_before_node2: insert.delivery_before,
                    });
                    return true;
                }
            }
        }
        false
    }

    pub fn any_biased_random_shift(&mut self, rng: &mut Random, bias: f64) -> bool {
        // any shift tries all request u
        let mut ps = self
            .instance
            .iter_pickups()
            .filter(|p| !self.unassigned_requests.contains(p.id))
            .collect::<Vec<_>>();
        ps.shuffle(rng);

        for p in ps.into_iter() {
            let p_id = p.id;
            let vn1_id = self.vn_id(p_id);
            let rs: Vec<usize> = self
                .iter_route_ids()
                .filter(|r_id| *r_id != vn1_id / 2)
                .collect();

            for r_id in rs.into_iter() {
                let mut inserts = self.find_all_inserts_for_request_in_route(p_id, r_id);
                inserts.shuffle(rng);
                let num_inserts = inserts.len();
                if let Some((insert, _)) = inserts
                    .iter()
                    // From all possible insertions of u
                    // in r, a set of \mu% of them is taken at random and the best position
                    // is selected for the insertion.
                    .take((num_inserts as f64 * bias).ceil() as usize)
                    .min_by_key(|(_, cost)| cost)
                {
                    self.apply_inter_relocate_move(InterRelocateMove {
                        vn1: vn1_id,
                        vn2: insert.vn_id,
                        p1: insert.pickup_id,
                        p1_after_node2: insert.pickup_after,
                        d1_before_node2: insert.delivery_before,
                    });
                    return true;
                }
            }
        }
        false
    }

    pub fn random_shift_to_all_routes(&mut self, rng: &mut Random) -> bool {
        // the random shift selects a request u at random and moves it to another
        // route r_2 at random.
        let p = self
            .instance
            .iter_pickups()
            .filter(|p| !self.unassigned_requests.contains(p.id))
            .choose(rng);


        if let Some(p) = p {
            let mut relocate: Option<InterRelocateMove> = None;
            let mut feasible_relocations = 0;

            let p1 = &self.fw_data[p.id];
            let d1 = &self.fw_data[p.id + 1];
            let vn1_id = p1.vn_id;
            let mut r_ids = self
                .iter_route_ids()
                .filter(|r_id| *r_id != vn1_id / 2)
                .collect::<Vec<usize>>();
            r_ids.shuffle(rng);

            for r_id in r_ids {
                // now check inserting p1 and d1 to v2
                let v2 = &self.instance.vehicles[r_id];
                let vn2_id = r_id * 2;

                // filtering - only try relocate positions which could be reached in time
                let mut next_ins_pos_p1 = vn2_id;
                while next_ins_pos_p1 != vn2_id + 1 {
                    // position in v2 after which we try to relocate p1
                    let insert_pos_p1 = next_ins_pos_p1;
                    let before = &self.fw_data[insert_pos_p1];
                    debug_assert_ne!(next_ins_pos_p1, before.succ);
                    next_ins_pos_p1 = before.succ;

                    let dist_time = self.instance.distance_and_time(insert_pos_p1, p.id);

                    // can we reach p1 in time?
                    if before.data.earliest_completion + dist_time.time > p1.node.due {
                        continue;
                    }

                    let mut tmp = self.fw_data[insert_pos_p1]
                        .data
                        .extend_forward(&p1.node, dist_time);
                    if !tmp.check_feasible(v2) {
                        continue;
                    }

                    let mut prev = p1;
                    let mut next = &self.fw_data[before.succ];
                    // we now check whether we can insert d1 before the next node, iterating
                    //  forward until we hit the urt limit of p1-d1

                    while prev.node.id != vn2_id + 1 {
                        debug_assert_ne!(
                            prev.node.id,
                            next.node.id,
                            "{:?}",
                            self.iter_route(r_id).collect::<Vec<_>>()
                        );
                        // try to insert d1 and evaluate the route
                        let dist_time_tmp_d1 =
                            self.instance.distance_and_time(prev.node.id, d1.node.id);
                        let dist_time_d1_next =
                            self.instance.distance_and_time(d1.node.id, next.node.id);
                        let new_v2_data = tmp
                            .extend_forward(&d1.node, dist_time_tmp_d1)
                            .concat(&self.bw_data[next.node.id].data, dist_time_d1_next);

                        if new_v2_data.check_feasible(v2) {
                            // calculate change, see whether the move makes sense objective-wise,
                            //  i.e., could it be the best improving move?
                            feasible_relocations += 1;
                            if relocate.is_none() || rng.gen_bool(1.0 / feasible_relocations as f64)
                            {
                                relocate = Some(InterRelocateMove {
                                    vn1: vn1_id,
                                    vn2: vn2_id,
                                    p1: p.id,
                                    p1_after_node2: insert_pos_p1,
                                    d1_before_node2: next.node.id,
                                });
                            }
                        }

                        // then extend by the next node and continue with the one after
                        let dist_time = self.instance.distance_and_time(prev.node.id, next.node.id);
                        tmp.extend_forward_assign(&next.node, dist_time);
                        if !tmp.check_feasible(v2) {
                            break;
                        }
                        prev = next;
                        next = &self.fw_data[prev.succ];
                    }
                }
            }
            if let Some(mv) = relocate {
                self.apply_inter_relocate_move(mv);
                return true;
            }
        }
        false
    }

    pub fn any_random_shift(&mut self, rng: &mut Random) -> bool {
        // any shift tries all request u
        let mut ps = self
            .instance
            .iter_pickups()
            .filter(|p| !self.unassigned_requests.contains(p.id))
            .collect::<Vec<_>>();
        ps.shuffle(rng);

        for p in ps.into_iter() {
            let mut relocate: Option<InterRelocateMove> = None;
            let mut feasible_relocations = 0;

            let p1 = &self.fw_data[p.id];
            let d1 = &self.fw_data[p.id + 1];
            let vn1_id = p1.vn_id;
            let mut r_ids = self
                .iter_route_ids()
                .filter(|r_id| *r_id != vn1_id / 2)
                .collect::<Vec<usize>>();
            r_ids.shuffle(rng);

            for r_id in r_ids {
                // now check inserting p1 and d1 to v2
                let v2 = &self.instance.vehicles[r_id];
                let vn2_id = r_id * 2;

                // filtering - only try relocate positions which could be reached in time
                let mut next_ins_pos_p1 = vn2_id;
                while next_ins_pos_p1 != vn2_id + 1 {
                    // position in v2 after which we try to relocate p1
                    let insert_pos_p1 = next_ins_pos_p1;
                    let before = &self.fw_data[insert_pos_p1];
                    debug_assert_ne!(next_ins_pos_p1, before.succ);
                    next_ins_pos_p1 = before.succ;

                    let dist_time = self.instance.distance_and_time(insert_pos_p1, p.id);

                    // can we reach p1 in time?
                    if before.data.earliest_completion + dist_time.time > p1.node.due {
                        continue;
                    }

                    let mut tmp = self.fw_data[insert_pos_p1]
                        .data
                        .extend_forward(&p1.node, dist_time);
                    if !tmp.check_feasible(v2) {
                        continue;
                    }

                    let mut prev = p1;
                    let mut next = &self.fw_data[before.succ];
                    // we now check whether we can insert d1 before the next node, iterating
                    //  forward until we hit the urt limit of p1-d1

                    while prev.node.id != vn2_id + 1 {
                        debug_assert_ne!(
                            prev.node.id,
                            next.node.id,
                            "{:?}",
                            self.iter_route(r_id).collect::<Vec<_>>()
                        );
                        // try to insert d1 and evaluate the route
                        let dist_time_tmp_d1 =
                            self.instance.distance_and_time(prev.node.id, d1.node.id);
                        let dist_time_d1_next =
                            self.instance.distance_and_time(d1.node.id, next.node.id);
                        let new_v2_data = tmp
                            .extend_forward(&d1.node, dist_time_tmp_d1)
                            .concat(&self.bw_data[next.node.id].data, dist_time_d1_next);

                        if new_v2_data.check_feasible(v2) {
                            // calculate change, see whether the move makes sense objective-wise,
                            //  i.e., could it be the best improving move?
                            feasible_relocations += 1;
                            if relocate.is_none() || rng.gen_bool(1.0 / feasible_relocations as f64)
                            {
                                relocate = Some(InterRelocateMove {
                                    vn1: vn1_id,
                                    vn2: vn2_id,
                                    p1: p.id,
                                    p1_after_node2: insert_pos_p1,
                                    d1_before_node2: next.node.id,
                                });
                            }
                        }

                        // then extend by the next node and continue with the one after
                        let dist_time = self.instance.distance_and_time(prev.node.id, next.node.id);
                        tmp.extend_forward_assign(&next.node, dist_time);
                        if !tmp.check_feasible(v2) {
                            break;
                        }
                        prev = next;
                        next = &self.fw_data[prev.succ];
                    }
                }
            }
            if let Some(mv) = relocate {
                self.apply_inter_relocate_move(mv);
                return true;
            } else {}
        }
        false
    }


    pub fn apply_inter_relocate_move(&mut self, mv: InterRelocateMove) {
        let vn1_id = mv.vn1;
        let vn2_id = mv.vn2;
        self.apply_inter_relocate_move_bounded(
            mv,
            &UpdateBounds::complete_route(vn1_id),
            &UpdateBounds::complete_route(vn2_id),
        )
    }

    pub fn apply_inter_relocate_move_bounded(
        &mut self,
        mv: InterRelocateMove,
        bound1: &UpdateBounds,
        bound2: &UpdateBounds,
    ) {
        assert_eq!(mv.vn1, bound1.vn);
        assert_eq!(self.fw_data[mv.p1].vn_id, bound1.vn);
        assert_eq!(mv.vn2, bound2.vn);
        debug_assert_eq!(self.fw_data[mv.p1_after_node2].vn_id, mv.vn2);
        debug_assert_eq!(
            self.fw_data[mv.p1_after_node2].vn_id,
            self.fw_data[mv.d1_before_node2].vn_id
        );
        self.assert_links_aligned(mv.vn1);

        // close gaps from removing request
        let (p1_pred, d1_succ) = self.relink_gap_when_removing_pd(mv.p1);

        let delivery_id = mv.p1 + 1;

        let succ_pickup = self.fw_data[mv.p1_after_node2].succ;
        self.relink(mv.vn2, mv.p1, mv.p1_after_node2, succ_pickup);

        let pred_delivery = if mv.d1_before_node2 == succ_pickup {
            mv.p1
        } else {
            self.fw_data[mv.d1_before_node2].pred
        };
        self.relink(mv.vn2, delivery_id, pred_delivery, mv.d1_before_node2);

        // recalculate auxiliary data for constant time feasibility check
        self.partially_validate_between(p1_pred, d1_succ, bound1);
        self.partially_validate_between(mv.p1_after_node2, mv.d1_before_node2, bound2);

        #[cfg(feature = "move-asserts")]
        {
            self.assert_links_aligned(mv.vn1);
            self.assert_links_aligned(mv.vn2);
        }
    }
}
