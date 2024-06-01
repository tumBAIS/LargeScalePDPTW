use rand::Rng;

use crate::problem::Num;
use crate::refn::{ConcatREF, ForwardREF, REFData};
use crate::solution::permutation::moves::InterExchangeMove;
use crate::solution::{PDInsertion, Solution};
use crate::solution::datastructure::UpdateBounds;
use crate::utils::Random;

impl Solution<'_> {
    pub fn random_exchange(&mut self, rng: &mut Random) -> bool {
        // random exchange, which
        // selects two routes r1; r2 at random and one request at random from
        // each route u \in r1; v \in r2, then it moves u to r2 and v to r1 placing
        // them in the best position.
        let pair: Option<(usize, usize)> = {
            let mut ps = self
                .instance
                .iter_pickups()
                .filter(|p| !self.unassigned_requests.contains(p.id))
                .collect::<Vec<_>>();
            if ps.len() > 0 {
                let p1 = ps.swap_remove(rng.gen_range(0..ps.len())).id;
                let p1_vn_id = self.fw_data[p1].vn_id;
                let p2 = loop {
                    if ps.len() > 0 {
                        let p = ps.swap_remove(rng.gen_range(0..ps.len())).id;
                        if self.fw_data[p].vn_id == p1_vn_id {
                            continue;
                        }
                        break Some(p);
                    }
                    break None;
                };
                if p2.is_some() {
                    p2.map(|p2| (p1, p2))
                } else {
                    None
                }
            } else {
                None
            }
        };

        if let Some((p1, p2)) = pair {
            if let Some(insertion_1) = self.compute_eject_and_best_insert(p1, p2) {
                if let Some(insertion_2) = self.compute_eject_and_best_insert(p2, p1) {
                    self.unassign_request(p1);
                    self.unassign_request(p2);

                    self.insert(insertion_1);
                    self.insert(insertion_2);

                    return true;
                }
            }
        }
        false
    }

    pub fn random_swap(&mut self, rng: &mut Random) -> bool {
        // random swap
        // selects two routes r1; r2 at random and one request at random from
        // each route u \in r1; v \in r2, swap their positions.
        let (p, ps) = {
            let mut ps = self
                .instance
                .iter_pickups()
                .filter(|p| !self.unassigned_requests.contains(p.id))
                .collect::<Vec<_>>();

            if ps.len() > 0 {
                (Some(ps.swap_remove(rng.gen_range(0..ps.len()))), ps)
            } else {
                (None, vec![])
            }
        };

        if let Some(p1) = p {
            let mut exchange: Option<InterExchangeMove> = None;
            let mut feasible_exchanges = 0;

            let p1_node = &self.fw_data[p1.id];
            let vn1_id = p1_node.vn_id;

            for p2 in ps {
                let vn2_id = self.vn_id(p2.id);

                if vn2_id == vn1_id {
                    // ignore intra-route exchanges
                    continue;
                }

                // exchange nodes at the current positions
                let _new_v1_data = if let Some(it) = self.compute_replacement(p1.id, p2.id) {
                    it
                } else {
                    continue;
                };

                // now check vn2
                let _new_v2_data = if let Some(it) = self.compute_replacement(p2.id, p1.id) {
                    it
                } else {
                    continue;
                };

                // calculate change, see whether the move makes sense objective-wise,
                //  i.e., could it be the best improving move?
                feasible_exchanges += 1;
                if exchange.is_none() || rng.gen_bool(1.0 / feasible_exchanges as f64) {
                    exchange = Some(InterExchangeMove {
                        vn1: vn1_id,
                        vn2: vn2_id,
                        p1: p1.id,
                        p2: p2.id,
                    });
                    break;
                }
            }

            if let Some(mv) = exchange {
                self.apply_inter_exchange_move(mv);
                return true;
            }
        }
        false
    }

    fn compute_replacement(&self, p_id: usize, other_p_id: usize) -> Option<REFData> {
        let d_id = p_id + 1;
        let other_d_id = other_p_id + 1;
        let v = self.instance.vehicle_from_vn_id(self.vn_id(p_id));

        // start with vn1
        let mut v1_tmp = self.fw_data[self.pred(p_id)].data.extend_forward(
            &self.node(other_p_id),
            self.instance.distance_and_time(self.pred(p_id), other_p_id),
        );

        if !v1_tmp.check_feasible(v) {
            return None;
        }

        let mut pred_id = other_p_id;
        let mut next_id = self.succ(p_id);
        while next_id != d_id {
            v1_tmp.extend_forward_assign(
                self.node(next_id),
                self.instance.distance_and_time(pred_id, next_id),
            );
            pred_id = next_id;
            next_id = self.succ(pred_id);
        }

        v1_tmp.extend_forward_assign(
            self.node(other_d_id),
            self.instance.distance_and_time(pred_id, other_d_id),
        );

        v1_tmp.concat_assign(
            &self.bw_data[self.succ(d_id)].data,
            self.instance.distance_and_time(other_d_id, self.succ(d_id)),
        );

        if v1_tmp.check_feasible(v) {
            Some(v1_tmp)
        } else {
            None
        }
    }

    fn compute_eject_and_best_insert(&self, eject_p: usize, p_id: usize) -> Option<PDInsertion> {
        let mut min_insertion_cost = Num::MAX;
        let mut insertion: Option<PDInsertion> = None;

        let vn_id = self.fw_data[eject_p].vn_id;
        let v = self.instance.vehicle_from_vn_id(vn_id);
        let eject_d = eject_p + 1;

        let p = &self.fw_data[p_id].node;
        let d_id = p_id + 1;
        let d = &self.fw_data[d_id].node;

        // find insertion where we can feasibly insert the request defined by insert_p
        let mut next_ins_pos = vn_id;
        let mut next_prev_pos = vn_id;
        let mut eject_p_before_p = false;
        let mut eject_d_before_p = false;
        let mut data_before_p: Option<REFData> = None;

        while next_ins_pos != vn_id + 1 {
            let prev_pos = next_prev_pos;
            let insert_pos = next_ins_pos;
            let before = &self.fw_data[insert_pos];
            next_prev_pos = insert_pos;
            next_ins_pos = before.succ;

            if let Some(ref mut data) = data_before_p {
                // we did already encounter the ejected node and store the modified
                //  data already in 'data_before_p' - now update accordingly
                debug_assert!(eject_p_before_p);

                data.extend_forward_assign(
                    &before.node,
                    self.instance.distance_and_time(prev_pos, insert_pos),
                )
            }

            // is the next node the to-be-ejected pickup?
            if next_ins_pos == eject_p {
                // if so, skip this position - handle this next iteration
                eject_p_before_p = true;
                next_ins_pos = self.succ(next_ins_pos);
                data_before_p = Some(self.fw_data[insert_pos].data.clone());
            }
            // is the next node the to-be-ejected delivery?
            if next_ins_pos == eject_d {
                // if so, skip this position - handle this next iteration
                eject_d_before_p = true;
                next_ins_pos = self.succ(next_ins_pos);
            }

            let dist_time = self.instance.distance_and_time(insert_pos, p_id);

            // can we reach p1 in time?
            let before_earliest_completion = if let Some(ref data) = data_before_p {
                data.earliest_completion
            } else {
                before.data.earliest_completion
            };

            if before_earliest_completion + dist_time.time > p.due {
                continue;
            }

            // if so, ref extend forward
            let mut data_with_p = if let Some(ref mut data) = data_before_p {
                debug_assert!(eject_p_before_p);
                data.extend_forward(p, dist_time)
            } else {
                self.fw_data[insert_pos].data.extend_forward(p, dist_time)
            };

            // now continue with the delivery
            let mut eject_d_before_d = eject_d_before_p;

            let mut ins_pos_d = p_id;
            let mut next_pos_d = next_ins_pos;
            // we now check whether we can insert d1 before the next node
            while ins_pos_d != vn_id + 1 {
                if next_pos_d == eject_p {
                    // if so, skip this position - handle this next iteration
                    next_pos_d = self.succ(next_pos_d);
                }
                // is the next node the to-be-ejected delivery?
                if next_pos_d == eject_d {
                    // if so, skip this position - handle this next iteration
                    eject_d_before_d = true;
                    next_pos_d = self.succ(next_pos_d);
                }

                let dist_time = self.instance.distance_and_time(ins_pos_d, d_id);

                // can we reach d1 in time?
                if data_with_p.earliest_completion + dist_time.time > d.due {
                    // no, and we never will
                    break;
                }

                let mut new_v2_data = data_with_p.extend_forward(d, dist_time);
                let mut eject_d_encountered = eject_d_before_d;

                let mut prev = d_id;
                let mut next = next_pos_d;
                while !eject_d_encountered {
                    if next == eject_p {
                        // if so, skip this position - handle this next iteration
                        next = self.succ(next);
                    }
                    // is the next node the to-be-ejected delivery?
                    if next == eject_d {
                        // if so, skip this position - handle this next iteration
                        eject_d_encountered = true;
                        next = self.succ(next);
                    } else {
                        new_v2_data.extend_forward_assign(
                            &self.fw_data[next].node,
                            self.instance.distance_and_time(prev, next),
                        );
                        prev = next;
                        next = self.succ(next);
                    }
                }
                new_v2_data.concat_assign(
                    &self.bw_data[next].data,
                    self.instance.distance_and_time(prev, next),
                );

                if new_v2_data.check_feasible(v) {
                    if insertion.is_none() || new_v2_data.distance < min_insertion_cost {
                        insertion = Some(PDInsertion {
                            vn_id,
                            pickup_id: p_id,
                            pickup_after: insert_pos,
                            delivery_before: next_pos_d,
                        });
                        min_insertion_cost = new_v2_data.distance;
                    }
                }

                // then extend by the next node and continue with the one after
                let dist_time = self.instance.distance_and_time(ins_pos_d, next_pos_d);
                data_with_p.extend_forward_assign(&self.fw_data[next_pos_d].node, dist_time);
                ins_pos_d = next_pos_d;
                next_pos_d = self.succ(next_pos_d);
            }
        }
        insertion
    }

    pub fn apply_inter_exchange_move(&mut self, mv: InterExchangeMove) {
        let vn1_id = mv.vn1;
        let vn2_id = mv.vn2;
        self.apply_inter_exchange_move_bounded(
            mv,
            &UpdateBounds::complete_route(vn1_id),
            &UpdateBounds::complete_route(vn2_id),
        )
    }

    pub fn apply_inter_exchange_move_bounded(
        &mut self,
        mv: InterExchangeMove,
        bound1: &UpdateBounds,
        bound2: &UpdateBounds,
    ) {
        debug_assert_eq!(self.fw_data[mv.p1].vn_id, mv.vn1);
        debug_assert_eq!(self.fw_data[mv.p2].vn_id, mv.vn2);

        let pred_p1 = self.pred(mv.p1);
        let succ_p1 = self.succ(mv.p1);
        let pred_d1 = self.pred(mv.p1 + 1);
        let succ_d1 = self.succ(mv.p1 + 1);

        let pred_p2 = self.pred(mv.p2);
        let succ_p2 = self.succ(mv.p2);
        let pred_d2 = self.pred(mv.p2 + 1);
        let succ_d2 = self.succ(mv.p2 + 1);

        if succ_p1 == mv.p1 + 1 {
            self.relink(mv.vn1, mv.p2, pred_p1, mv.p2 + 1);
            self.relink(mv.vn1, mv.p2 + 1, mv.p2, succ_d1);
        } else {
            self.relink(mv.vn1, mv.p2, pred_p1, succ_p1);
            self.relink(mv.vn1, mv.p2 + 1, pred_d1, succ_d1);
        }

        if succ_p2 == mv.p2 + 1 {
            self.relink(mv.vn2, mv.p1, pred_p2, mv.p1 + 1);
            self.relink(mv.vn2, mv.p1 + 1, mv.p1, succ_d2);
        } else {
            self.relink(mv.vn2, mv.p1, pred_p2, succ_p2);
            self.relink(mv.vn2, mv.p1 + 1, pred_d2, succ_d2);
        }

        self.partially_validate_between(pred_p1, succ_d1, bound1);
        self.partially_validate_between(pred_p2, succ_d2, bound2);

        #[cfg(feature = "move-asserts")]
        {
            self.assert_links_aligned(mv.vn1);
            self.assert_links_aligned(mv.vn2);
        }
    }
}
