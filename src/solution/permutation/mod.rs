use fixedbitset::FixedBitSet;
use rand::seq::SliceRandom;
use rand::Rng;

use crate::lns::absence_counter::AbsenceCounter;
use crate::problem::pdptw::{Node, PDPTWInstance};
use crate::problem::Num;
use crate::refn::{BackwardREF, ConcatREF, ForwardREF, REFData, REFNode};
use crate::solution::{PDInsertion, Solution};
use crate::utils::Random;

mod exchange;
mod relocate;
pub mod moves;

pub struct ReservoirSampling<U> {
    sample: Option<U>,
    count: usize,
}

impl<T> ReservoirSampling<T> {
    pub fn new() -> Self {
        Self {
            sample: None,
            count: 0,
        }
    }

    pub fn next<F>(&mut self, sample: F, rng: &mut Random)
        where
            F: FnOnce() -> T,
    {
        if self.sample.is_none() || rng.gen_bool(1.0 / self.count as f64) {
            self.sample = Some(sample());
        }
        self.count += 1;
    }

    pub fn take(self) -> Option<T> {
        self.sample
    }
}

impl<'a> Solution<'a> {
    pub fn find_random_insert_for_request_in_route(
        &self,
        p_id: usize,
        r_id: usize,
        rng: &mut Random,
    ) -> Option<PDInsertion> {
        self.find_random_insert_for_request_in_route_using_reservoir_sampling(
            p_id,
            r_id,
            rng,
            ReservoirSampling::new(),
        )
            .take()
    }

    pub fn find_random_insert_for_request_in_route_using_reservoir_sampling(
        &self,
        p_id: usize,
        r_id: usize,
        rng: &mut Random,
        previous_sampling: ReservoirSampling<PDInsertion>,
    ) -> ReservoirSampling<PDInsertion> {
        let mut sampling = previous_sampling;

        let vn_id = r_id * 2;
        // filtering - only try positions which can be reached in time
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

            let mut forward_with_p = self.fw_data[insert_pos]
                .data
                .extend_forward(&p.node, dist_time);

            if !v.check_capacity(forward_with_p.current_load) {
                continue;
            }

            let mut prev = p;
            let mut next = &self.fw_data[before.succ];
            // we now check whether we can insert d1 before the next node, iterating
            //  forward until we hit the urt limit of p1-d1
            while prev.node.id != vn_id + 1 {
                debug_assert_ne!(prev.node.id, next.node.id);
                // try to insert d1 and evaluate the route
                let dist_time_tmp_d1 = self.instance.distance_and_time(prev.node.id, p_id + 1);
                let dist_time_d1_next = self.instance.distance_and_time(p_id + 1, next.node.id);

                let new_v2_data = forward_with_p
                    .extend_forward(&d.node, dist_time_tmp_d1)
                    .concat(&self.bw_data[next.node.id].data, dist_time_d1_next);

                if new_v2_data.check_feasible(v) {
                    sampling.next(
                        || PDInsertion {
                            vn_id,
                            pickup_id: p_id,
                            pickup_after: insert_pos,
                            delivery_before: next.node.id,
                        },
                        rng,
                    );
                }
                // then extend by the next node and continue with the one after
                let dist_time = self.instance.distance_and_time(prev.node.id, next.node.id);
                forward_with_p.extend_forward_assign(&next.node, dist_time);
                if !forward_with_p.check_feasible(v) {
                    break;
                }

                prev = next;
                next = &self.fw_data[prev.succ];
            }
        }

        sampling
    }

    pub fn find_random_insert_for_request(
        &self,
        p_id: usize,
        rng: &mut Random,
    ) -> Option<PDInsertion> {
        let mut reservoir_sampling = ReservoirSampling::new();

        for r_id in self
            .iter_route_ids()
            .chain(self.iter_empty_route_ids().take(1))
        {
            reservoir_sampling = self
                .find_random_insert_for_request_in_route_using_reservoir_sampling(
                    p_id,
                    r_id,
                    rng,
                    reservoir_sampling,
                );
        }

        reservoir_sampling.take()
    }

    pub fn find_all_inserts_for_request_in_route(
        &self,
        p_id: usize,
        r_id: usize,
    ) -> Vec<(PDInsertion, Num)> {
        let mut inserts = Vec::new();
        inserts.reserve(32);

        let vn_id = r_id * 2;
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

            let mut prev = p;
            let mut next = &self.fw_data[before.succ];
            // we now check whether we can insert d1 before the next node, iterating
            //  forward until we hit the urt limit of p1-d1
            while prev.node.id != vn_id + 1 {
                debug_assert_ne!(prev.node.id, next.node.id);
                // try to insert d1 and evaluate the route
                let dist_time_tmp_d1 = self.instance.distance_and_time(prev.node.id, p_id + 1);
                let dist_time_d1_next = self.instance.distance_and_time(p_id + 1, next.node.id);

                let new_v2_data = tmp
                    .extend_forward(&d.node, dist_time_tmp_d1)
                    .concat(&self.bw_data[next.node.id].data, dist_time_d1_next);

                if new_v2_data.check_feasible(v) {
                    inserts.push((
                        PDInsertion {
                            vn_id,
                            pickup_id: p_id,
                            pickup_after: insert_pos,
                            delivery_before: next.node.id,
                        },
                        new_v2_data.distance - self.fw_data[vn_id + 1].data.distance,
                    ));
                }
                // then extend by the next node and continue with the one after
                let dist_time = self.instance.distance_and_time(prev.node.id, next.node.id);
                tmp.extend_forward_assign(&next.node, dist_time);
                prev = next;
                next = &self.fw_data[prev.succ];
            }
        }
        inserts
    }
}

#[derive(Copy, Clone, Debug)]
pub struct PDEjection {
    pub(crate) vn_id: usize,
    pub(crate) pickup_id: usize,
}

impl<'a> Solution<'a> {
    pub fn find_best_insertion_triple_ejection_for_request_alt(
        &self,
        p_id: usize,
        rng: &mut Random,
        abs: &AbsenceCounter<'a>,
    ) -> Option<KEjectionInsertion<3>> {
        let mut ejection_insertion: Option<KEjectionInsertion<3>> = None;
        let mut feasible_min_positions = 0;

        let mut min_cnt = usize::MAX;

        let mut route_ids = self.iter_route_ids().collect::<Vec<usize>>();
        route_ids.shuffle(rng);

        for r_id in route_ids {
            let vn_id = r_id * 2;
            let route_len = self.iter_route(vn_id / 2).count();

            // find first request to eject
            let mut eject_p1 = self.succ(vn_id);
            while eject_p1 != vn_id + 1 {
                let cnt_p1 = abs.get_sum_for_request(self.instance.request_id(eject_p1));
                if self.instance.is_delivery(eject_p1) || cnt_p1 > min_cnt {
                    eject_p1 = self.succ(eject_p1);
                    continue;
                }
                // ejection of p1 might make sense
                let eject_d1 = eject_p1 + 1;

                let mut eject_p2 = self.succ(eject_p1);
                while eject_p2 != vn_id + 1 {
                    let cnt_p2 = abs.get_sum_for_request(self.instance.request_id(eject_p2));
                    if self.instance.is_delivery(eject_p2) || cnt_p1 + cnt_p2 > min_cnt {
                        eject_p2 = self.succ(eject_p2);
                        continue;
                    }

                    // ejection of p1&p2 might make sense
                    let eject_d2 = eject_p2 + 1;

                    let mut eject_p3 = self.succ(eject_p2);
                    while eject_p3 != vn_id + 1 {
                        let cnt_p3 = abs.get_sum_for_request(self.instance.request_id(eject_p3));
                        if self.instance.is_delivery(eject_p2) || cnt_p1 + cnt_p2 + cnt_p3 > min_cnt
                        {
                            eject_p3 = self.succ(eject_p3);
                            continue;
                        }

                        // ejection of p1&p2 might make sense
                        let eject_d3 = eject_p3 + 1;

                        let route = REFRoute::for_iter(
                            self.instance,
                            self.iter_route(vn_id / 2)
                                .filter(|it| *it != eject_p1)
                                .filter(|it| *it != eject_p2)
                                .filter(|it| *it != eject_p3)
                                .filter(|it| *it != eject_d1)
                                .filter(|it| *it != eject_d2)
                                .filter(|it| *it != eject_d3),
                            Some(route_len - 6),
                        );
                        if let Some((insertion, feasible_positions)) =
                            route.find_best_insertion_for_request(p_id, rng)
                        {
                            if cnt_p1 + cnt_p2 + cnt_p3 < min_cnt {
                                // just take it and update feasible positions
                                ejection_insertion = Some(KEjectionInsertion {
                                    ejections: [
                                        PDEjection {
                                            vn_id,
                                            pickup_id: eject_p1,
                                        },
                                        PDEjection {
                                            vn_id,
                                            pickup_id: eject_p2,
                                        },
                                        PDEjection {
                                            vn_id,
                                            pickup_id: eject_p3,
                                        },
                                    ],
                                    insertion,
                                });
                                feasible_min_positions = feasible_positions;
                                min_cnt = cnt_p1 + cnt_p2 + cnt_p3;
                            } else {
                                feasible_min_positions += feasible_positions;
                                if rng.gen_bool(
                                    feasible_positions as f64 / feasible_min_positions as f64,
                                ) {
                                    ejection_insertion = Some(KEjectionInsertion {
                                        ejections: [
                                            PDEjection {
                                                vn_id,
                                                pickup_id: eject_p1,
                                            },
                                            PDEjection {
                                                vn_id,
                                                pickup_id: eject_p2,
                                            },
                                            PDEjection {
                                                vn_id,
                                                pickup_id: eject_p3,
                                            },
                                        ],
                                        insertion,
                                    });
                                }
                            }
                        }
                        // consider next p2
                        eject_p3 = self.succ(eject_p3);
                    }
                    // consider next p2
                    eject_p2 = self.succ(eject_p2);
                }
                // consider next p1
                eject_p1 = self.succ(eject_p1);
            }
        }
        ejection_insertion
    }

    pub fn find_best_insertion_k_ejection_for_request<const K: usize>(
        &self,
        p_id: usize,
        rng: &mut Random,
        abs: &AbsenceCounter<'a>,
    ) -> Option<KEjectionInsertion<K>> {
        let mut ejection_insertion: Option<KEjectionInsertion<K>> = None;
        let mut feasible_min_positions = 0;

        let mut min_cnt = usize::MAX;

        let mut route_ids = self.iter_route_ids().collect::<Vec<usize>>();
        route_ids.shuffle(rng);

        let mut stack: [usize; K] = [0; K];
        let mut stack_cnt_sum: [usize; K] = [0; K];
        let mut requests_on_stack = FixedBitSet::with_capacity(self.instance.num_requests);

        for r_id in route_ids.into_iter() {
            let vn_id = r_id * 2;
            let route_len = self.iter_route(r_id).count();

            stack[0] = self.succ(vn_id);
            stack_cnt_sum[0] = abs.get_sum_for_request(self.instance.request_id(stack[0]));

            let mut stack_height = 1;

            let mut working_route =
                REFRouteView::init(self.instance, &self, r_id, route_len - (K * 2));

            // stack is filled with the first set of requests (pickups) to be ejected
            'main: while stack_height > 0 {
                for i in stack_height..K {
                    let mut next = self.succ(stack[i - 1]);
                    while self.instance.is_delivery(next) {
                        next = self.succ(next);
                    }
                    if next == vn_id + 1 {
                        break 'main;
                    } else {
                        stack_cnt_sum[i] = stack_cnt_sum[i - 1]
                            + abs.get_sum_for_request(self.instance.request_id(next));
                        stack[i] = next;
                        stack_height = i + 1;
                    }
                }

                if stack_cnt_sum[stack_height - 1] <= min_cnt {
                    requests_on_stack.clear();
                    for it in stack.iter().cloned() {
                        requests_on_stack.insert(self.instance.request_id(it))
                    }

                    if let Some((insertion, feasible_positions)) = working_route
                        .find_best_insertion_for_request_while_ignoring_requests(
                            p_id,
                            &requests_on_stack,
                            rng,
                        )
                    {
                        if stack_cnt_sum[K - 1] < min_cnt {
                            // just take it and update feasible positions
                            ejection_insertion = Some(KEjectionInsertion {
                                ejections: core::array::from_fn(|i| PDEjection {
                                    vn_id,
                                    pickup_id: stack[i].clone(),
                                }),
                                insertion,
                            });
                            feasible_min_positions = feasible_positions;
                            min_cnt = stack_cnt_sum[K - 1];
                        } else {
                            feasible_min_positions += feasible_positions;
                            if rng
                                .gen_bool(feasible_positions as f64 / feasible_min_positions as f64)
                            {
                                ejection_insertion = Some(KEjectionInsertion {
                                    ejections: core::array::from_fn(|i| PDEjection {
                                        vn_id,
                                        pickup_id: stack[i].clone(),
                                    }),
                                    insertion,
                                });
                            }
                        }
                    }
                }
                // now progress the stack
                // 1. starting with K-1, update the value on the stack by progressing forward
                // 2. if we find a pickup node stop and continue the main loop
                // 3. if we encounter vn_id + 1, we reached the end of the route, decrease the stack height and go to 1
                while stack_height > 0 {
                    let i = stack_height - 1;
                    let mut next = self.succ(stack[i]);
                    while self.instance.is_delivery(next) {
                        next = self.succ(next);
                    }
                    if next == vn_id + 1 {
                        stack_height -= 1;
                    } else {
                        stack_cnt_sum[i] = abs.get_sum_for_request(self.instance.request_id(next))
                            + if i > 0 { stack_cnt_sum[i - 1] } else { 0 };
                        stack[i] = next;

                        if stack_cnt_sum[i] <= min_cnt {
                            // we found a pickup node which may be an equal or better option
                            //  -> stop and continue the main loop
                            break;
                        }
                    }
                }
            }
        }

        ejection_insertion
    }

    pub fn find_best_insertion_pair_ejection_for_request_alt(
        &self,
        p_id: usize,
        rng: &mut Random,
        abs: &AbsenceCounter<'a>,
    ) -> Option<KEjectionInsertion<2>> {
        let mut ejection_insertion: Option<KEjectionInsertion<2>> = None;
        let mut feasible_min_positions = 0;

        let mut min_cnt = usize::MAX;

        let mut route_ids = self.iter_route_ids().collect::<Vec<usize>>();
        route_ids.shuffle(rng);

        for r_id in route_ids {
            let vn_id = r_id * 2;
            let route_len = self.iter_route(vn_id / 2).count();

            // find first request to eject
            let mut eject_p1 = self.succ(vn_id);
            while eject_p1 != vn_id + 1 {
                let cnt_p1 = abs.get_sum_for_request(self.instance.request_id(eject_p1));
                if self.instance.is_delivery(eject_p1) || cnt_p1 > min_cnt {
                    eject_p1 = self.succ(eject_p1);
                    continue;
                }
                // ejection of p1 might make sense
                let eject_d1 = eject_p1 + 1;

                let mut eject_p2 = self.succ(eject_p1);
                while eject_p2 != vn_id + 1 {
                    let cnt_p2 = abs.get_sum_for_request(self.instance.request_id(eject_p2));
                    if self.instance.is_delivery(eject_p2) || cnt_p1 + cnt_p2 > min_cnt {
                        eject_p2 = self.succ(eject_p2);
                        continue;
                    }

                    // ejection of p1&p2 might make sense
                    let eject_d2 = eject_p2 + 1;

                    let route = REFRoute::for_iter(
                        self.instance,
                        self.iter_route(vn_id / 2)
                            .filter(|it| *it != eject_p1)
                            .filter(|it| *it != eject_p2)
                            .filter(|it| *it != eject_d1)
                            .filter(|it| *it != eject_d2),
                        Some(route_len - 4),
                    );
                    if let Some((insertion, feasible_positions)) =
                        route.find_best_insertion_for_request(p_id, rng)
                    {
                        if cnt_p1 + cnt_p2 < min_cnt {
                            // just take it and update feasible positions
                            ejection_insertion = Some(KEjectionInsertion {
                                ejections: [
                                    PDEjection {
                                        vn_id,
                                        pickup_id: eject_p1,
                                    },
                                    PDEjection {
                                        vn_id,
                                        pickup_id: eject_p2,
                                    },
                                ],
                                insertion,
                            });
                            feasible_min_positions = feasible_positions;
                            min_cnt = cnt_p1 + cnt_p2;
                        } else {
                            feasible_min_positions += feasible_positions;
                            if rng
                                .gen_bool(feasible_positions as f64 / feasible_min_positions as f64)
                            {
                                ejection_insertion = Some(KEjectionInsertion {
                                    ejections: [
                                        PDEjection {
                                            vn_id,
                                            pickup_id: eject_p1,
                                        },
                                        PDEjection {
                                            vn_id,
                                            pickup_id: eject_p2,
                                        },
                                    ],
                                    insertion,
                                });
                            }
                        }
                    }
                    // consider next p2
                    eject_p2 = self.succ(eject_p2);
                }
                // consider next p1
                eject_p1 = self.succ(eject_p1);
            }
        }
        ejection_insertion
    }

    pub fn find_best_insertion_single_ejection_for_request_alt(
        &self,
        p_id: usize,
        rng: &mut Random,
        abs: &AbsenceCounter<'a>,
    ) -> Option<KEjectionInsertion<1>> {
        let mut ejection_insertion: Option<KEjectionInsertion<1>> = None;
        let mut min_cnt = usize::MAX;
        let mut feasible_min_positions = 0;

        let mut route_ids = self.iter_route_ids().collect::<Vec<usize>>();
        route_ids.shuffle(rng);

        for r_id in route_ids {
            let vn_id = r_id * 2;
            let route_len = self.iter_route(vn_id / 2).count();

            // first find request to eject
            let mut eject_p = self.succ(vn_id);
            while eject_p != vn_id + 1 {
                let current_cnt = abs.get_sum_for_request(self.instance.request_id(eject_p));
                if self.instance.is_delivery(eject_p) || current_cnt > min_cnt {
                    eject_p = self.succ(eject_p);
                    continue;
                }
                // ejection of p might make sense
                let eject_d = eject_p + 1;

                // 2. find insertion where we can feasibly insert the request defined by p_id
                let route = REFRoute::for_iter(
                    self.instance,
                    self.iter_route(vn_id / 2)
                        .filter(|it| *it != eject_p)
                        .filter(|it| *it != eject_d),
                    Some(route_len - 2),
                );

                if let Some((insertion, feasible_positions_in_route)) =
                    route.find_best_insertion_for_request(p_id, rng)
                {
                    if current_cnt < min_cnt {
                        // just take it and update feasible positions
                        ejection_insertion = Some(KEjectionInsertion {
                            ejections: [PDEjection {
                                vn_id,
                                pickup_id: eject_p,
                            }],
                            insertion,
                        });
                        feasible_min_positions = feasible_positions_in_route;
                        min_cnt = current_cnt;
                    } else {
                        feasible_min_positions += feasible_positions_in_route;
                        if rng.gen_bool(
                            feasible_positions_in_route as f64 / feasible_min_positions as f64,
                        ) {
                            ejection_insertion = Some(KEjectionInsertion {
                                ejections: [PDEjection {
                                    vn_id,
                                    pickup_id: eject_p,
                                }],
                                insertion,
                            });
                        }
                    }
                }
                // consider next p
                eject_p = self.succ(eject_p);
            }
        }
        ejection_insertion
    }

    pub fn find_best_insertion_pair_ejection_for_request(
        &self,
        p_id: usize,
        rng: &mut Random,
        abs: &AbsenceCounter<'a>,
    ) -> Option<KEjectionInsertion<2>> {
        let mut ejection_insertion: Option<KEjectionInsertion<2>> = None;
        let mut feasible_min_positions = 1;

        let mut min_cnt = usize::MAX;

        let mut route_ids = self.iter_route_ids().collect::<Vec<usize>>();
        route_ids.shuffle(rng);

        for r_id in route_ids {
            let vn_id = r_id * 2;
            let v = self.instance.vehicle_from_vn_id(vn_id);
            let p = &self.fw_data[p_id];
            let d_id = p_id + 1;
            let d = &self.fw_data[p_id + 1];

            // find first request to eject
            let mut eject_p1 = self.succ(vn_id);
            while eject_p1 != vn_id + 1 {
                let cnt_p1 = abs.get_sum_for_request(self.instance.request_id(eject_p1));
                if self.instance.is_delivery(eject_p1) {
                    eject_p1 = self.succ(eject_p1);
                    continue;
                }
                // ejection of p1 might make sense
                let eject_d1 = eject_p1 + 1;

                let mut eject_p2 = self.succ(eject_p1);
                while eject_p2 != vn_id + 1 {
                    let cnt_p2 = abs.get_sum_for_request(self.instance.request_id(eject_p2));
                    if self.instance.is_delivery(eject_p2) {
                        eject_p2 = self.succ(eject_p2);
                        continue;
                    }

                    // ejection of p1&p2 might make sense
                    let eject_d2 = eject_p2 + 1;

                    // 2. find insertion where we can feasibly insert the request defined by p_id
                    // starting right at the first position
                    let mut next_ins_pos = vn_id;
                    let mut next_prev_pos = vn_id;
                    let mut eject_p1_before_p = false;
                    let mut eject_d1_before_p = false;
                    let mut eject_d2_before_p = false;
                    let mut data_before_p: Option<REFData> = None;

                    'insert_loop: while next_ins_pos != vn_id + 1 {
                        let prev_pos = next_prev_pos;
                        let insert_pos = next_ins_pos;
                        let before = &self.fw_data[insert_pos];
                        next_prev_pos = insert_pos;
                        next_ins_pos = before.succ;

                        if let Some(ref mut data) = data_before_p {
                            // we did already encounter the ejected node and store the modified
                            //  data already in 'data_before_p' - now update accordingly
                            debug_assert!(eject_p1_before_p);

                            data.extend_forward_assign(
                                &before.node,
                                self.instance.distance_and_time(prev_pos, insert_pos),
                            )
                        }

                        // is the next node the to-be-ejected pickup1?
                        if next_ins_pos == eject_p1 {
                            // if so, skip this position - handle this next iteration
                            eject_p1_before_p = true;
                            next_ins_pos = self.succ(next_ins_pos);
                            data_before_p = Some(self.fw_data[insert_pos].data.clone());
                        }
                        // is the next node the to-be-ejected pickup2, delivery1, or delivery2?
                        while next_ins_pos == eject_d1
                            || next_ins_pos == eject_p2
                            || next_ins_pos == eject_d2
                        {
                            eject_d1_before_p |= next_ins_pos == eject_d1;
                            eject_d2_before_p |= next_ins_pos == eject_d2;
                            next_ins_pos = self.succ(next_ins_pos);
                        }

                        let dist_time = self.instance.distance_and_time(insert_pos, p_id);

                        // can we reach p1 in time?
                        let before_earliest_completion = if let Some(ref data) = data_before_p {
                            data.earliest_completion
                        } else {
                            before.data.earliest_completion
                        };

                        if before_earliest_completion + dist_time.time > p.node.due {
                            continue;
                        }

                        // if so, ref extend forward
                        let mut data_with_p = if let Some(ref mut data) = data_before_p {
                            debug_assert!(eject_p1_before_p);
                            data.extend_forward(&p.node, dist_time)
                        } else {
                            self.fw_data[insert_pos]
                                .data
                                .extend_forward(&p.node, dist_time)
                        };

                        // now continue with the delivery
                        let mut eject_d1_before_d = eject_d1_before_p;
                        let mut eject_d2_before_d = eject_d2_before_p;

                        let mut ins_pos_d = p.node.id;
                        let mut next_pos_d = next_ins_pos;

                        // we now check whether we can insert d1 before the next node
                        while ins_pos_d != vn_id + 1 {
                            // is the next node the to-be-ejected a pickup/delivery?
                            //  then update flags and go to the next node
                            while next_pos_d == eject_p1
                                || next_pos_d == eject_d1
                                || next_pos_d == eject_p2
                                || next_pos_d == eject_d2
                            {
                                eject_d1_before_d |= next_pos_d == eject_d1;
                                eject_d2_before_d |= next_pos_d == eject_d2;

                                next_pos_d = self.succ(next_pos_d);
                            }

                            let dist_time = self.instance.distance_and_time(ins_pos_d, d_id);

                            // can we reach d1 in time?
                            if data_with_p.earliest_completion + dist_time.time > d.node.due {
                                // no, and we never will
                                break;
                            }

                            let mut new_v2_data = data_with_p.extend_forward(&d.node, dist_time);

                            let mut eject_d1_encountered = eject_d1_before_d;
                            let mut eject_d2_encountered = eject_d2_before_d;

                            let mut prev = d_id;
                            let mut next = next_pos_d;
                            loop {
                                while next == eject_p1
                                    || next == eject_d1
                                    || next == eject_p2
                                    || next == eject_d2
                                {
                                    eject_d1_encountered |= next == eject_d1;
                                    eject_d2_encountered |= next == eject_d2;

                                    // if so, skip this position
                                    next = self.succ(next);
                                }

                                if eject_d1_encountered && eject_d2_encountered {
                                    break;
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
                                if cnt_p1 + cnt_p2 < min_cnt {
                                    ejection_insertion = None;
                                    feasible_min_positions = 1;
                                } else {
                                    feasible_min_positions += 1;
                                }

                                if ejection_insertion.is_none()
                                    || rng.gen_bool(1.0 / feasible_min_positions as f64)
                                {
                                    min_cnt = cnt_p1 + cnt_p2;
                                    debug_assert_ne!(eject_p1, eject_p2);

                                    ejection_insertion = Some(KEjectionInsertion {
                                        ejections: [
                                            PDEjection {
                                                vn_id,
                                                pickup_id: eject_p1,
                                            },
                                            PDEjection {
                                                vn_id,
                                                pickup_id: eject_p2,
                                            },
                                        ],
                                        insertion: PDInsertion {
                                            vn_id,
                                            pickup_id: p_id,
                                            pickup_after: insert_pos,
                                            delivery_before: next_pos_d,
                                        },
                                    });
                                }
                            }
                            // then extend by the next node and continue with the one after
                            let dist_time = self.instance.distance_and_time(ins_pos_d, next_pos_d);
                            data_with_p
                                .extend_forward_assign(&self.fw_data[next_pos_d].node, dist_time);
                            if !data_with_p.check_feasible(v) {
                                continue 'insert_loop;
                            }

                            ins_pos_d = next_pos_d;
                            next_pos_d = self.succ(next_pos_d);
                        }
                    }
                    // consider next p2
                    eject_p2 = self.succ(eject_p2);
                }
                // consider next p1
                eject_p1 = self.succ(eject_p1);
            }
        }
        ejection_insertion
    }

    pub fn find_best_insertion_single_ejection_for_request(
        &self,
        p_id: usize,
        rng: &mut Random,
        abs: &AbsenceCounter<'a>,
    ) -> Option<KEjectionInsertion<1>> {
        let mut ejection_insertion: Option<KEjectionInsertion<1>> = None;
        let mut min_cnt = usize::MAX;
        let mut feasible_min_positions = 0;

        let mut route_ids = self.iter_route_ids().collect::<Vec<usize>>();
        route_ids.shuffle(rng);

        for r_id in route_ids {
            let vn_id = r_id * 2;
            let v = self.instance.vehicle_from_vn_id(vn_id);
            let p = &self.fw_data[p_id];
            let d_id = p_id + 1;
            let d = &self.fw_data[p_id + 1];

            // first find request to eject
            let mut eject_p = self.succ(vn_id);
            while eject_p != vn_id + 1 {
                let current_cnt = abs.get_sum_for_request(self.instance.request_id(eject_p));
                if self.instance.is_delivery(eject_p) || current_cnt > min_cnt {
                    eject_p = self.succ(eject_p);
                    continue;
                }
                // ejection of p might make sense
                let eject_d = eject_p + 1;

                // 2. find insertion where we can feasibly insert the request defined by p_id
                // starting right at the first position
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

                    if before_earliest_completion + dist_time.time > p.node.due {
                        continue;
                    }

                    // if so, ref extend forward
                    let mut data_with_p = if let Some(ref mut data) = data_before_p {
                        debug_assert!(eject_p_before_p);
                        data.extend_forward(&p.node, dist_time)
                    } else {
                        self.fw_data[insert_pos]
                            .data
                            .extend_forward(&p.node, dist_time)
                    };

                    // now continue with the delivery
                    let mut eject_d_before_d = eject_d_before_p;

                    let mut ins_pos_d = p.node.id;
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
                        if data_with_p.earliest_completion + dist_time.time > d.node.due {
                            // no, and we never will
                            break;
                        }

                        let mut new_v2_data = data_with_p.extend_forward(&d.node, dist_time);
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
                            if current_cnt < min_cnt {
                                ejection_insertion = None;
                                feasible_min_positions = 1;
                            } else {
                                feasible_min_positions += 1;
                            }

                            if ejection_insertion.is_none()
                                || rng.gen_bool(1.0 / feasible_min_positions as f64)
                            {
                                min_cnt = current_cnt;

                                ejection_insertion = Some(KEjectionInsertion {
                                    ejections: [PDEjection {
                                        vn_id,
                                        pickup_id: eject_p,
                                    }],
                                    insertion: PDInsertion {
                                        vn_id,
                                        pickup_id: p_id,
                                        pickup_after: insert_pos,
                                        delivery_before: next_pos_d,
                                    },
                                });
                            }
                        }
                        // then extend by the next node and continue with the one after
                        let dist_time = self.instance.distance_and_time(ins_pos_d, next_pos_d);
                        data_with_p
                            .extend_forward_assign(&self.fw_data[next_pos_d].node, dist_time);
                        ins_pos_d = next_pos_d;
                        next_pos_d = self.succ(next_pos_d);
                    }
                }
                // consider next p
                eject_p = self.succ(eject_p);
            }
        }
        ejection_insertion
    }
}

pub struct KEjectionInsertion<const K: usize> {
    pub ejections: [PDEjection; K],
    pub insertion: PDInsertion,
}

pub struct SingleEjectionInsertion {
    pub ejection: PDEjection,
    pub insertion: PDInsertion,
}

pub struct DoubleEjectionInsertion {
    pub ejection1: PDEjection,
    pub ejection2: PDEjection,
    pub insertion: PDInsertion,
}

pub struct TripleEjectionInsertion {
    pub ejection1: PDEjection,
    pub ejection2: PDEjection,
    pub ejection3: PDEjection,
    pub insertion: PDInsertion,
}

struct REFRoute<'a> {
    instance: &'a PDPTWInstance,
    data: Vec<(REFNode, REFData, REFData)>,
}

impl<'a> REFRoute<'a> {
    fn for_iter(
        instance: &'a PDPTWInstance,
        mut iter: impl Iterator<Item=usize>,
        iter_length: Option<usize>,
    ) -> Self {
        let mut data = iter_length
            .map(|len| Vec::with_capacity(len))
            .unwrap_or(Vec::new());
        if let Some(mut prev) = iter.next() {
            let node = REFNode::from(&instance.nodes[prev]);
            let fw_data = REFData::with_node(&node);
            let bw_data = REFData::with_node(&node);
            data.push((node, fw_data, bw_data));

            while let Some(next) = iter.next() {
                let node = REFNode::from(&instance.nodes[next]);
                let fw_data = data
                    .last()
                    .unwrap()
                    .1
                    .extend_forward(&node, instance.distance_and_time(prev, next));
                let bw_data = REFData::with_node(&node);
                data.push((node, fw_data, bw_data));
                prev = next;
            }

            for i in (1..data.len() - 1).rev() {
                let next = data[i + 1].0.id;
                let prev = data[i].0.id;
                unsafe {
                    let next_entry_data = &(*data.as_ptr().add(i + 1)).2;
                    let prev_entry_node = &(*data.as_ptr().add(i)).0;
                    let prev_entry_data = &mut (*data.as_mut_ptr().add(i)).2;

                    next_entry_data.extend_backward_into_target(
                        prev_entry_node,
                        prev_entry_data,
                        instance.distance_and_time(prev, next),
                    );
                }
            }
        }

        Self { instance, data }
    }

    pub fn find_best_insertion_for_request(
        &self,
        pickup_id: usize,
        rng: &mut Random,
    ) -> Option<(PDInsertion, usize)> {
        let v = self.instance.vehicle_from_vn_id(self.data[0].0.id);
        let delivery_id = self.instance.delivery_of(pickup_id).id;
        let ref_pickup = REFNode::from(&self.instance.nodes[pickup_id]);
        let ref_delivery = REFNode::from(&self.instance.nodes[delivery_id]);

        let mut feasible_min_positions = 0;
        let mut insertion: Option<PDInsertion> = None;

        for p_pos in 1..self.data.len() {
            let prev_id = self.data[p_pos - 1].0.id;
            let dt_from_prev = self.instance.distance_and_time(prev_id, pickup_id);
            // if the due time of the pickup to be inserted is smaller than the earliest arrival from
            //  the previous visit we can ignore any subsequent visits as we will always have a
            //  feasibility issue.
            if ref_pickup.due < self.data[p_pos - 1].1.earliest_completion() + dt_from_prev.time {
                break;
            }

            let mut seq_with_p = self.data[p_pos - 1]
                .1
                .extend_forward(&ref_pickup, dt_from_prev);
            let mut last_in_seq_with_p = pickup_id;

            if !&seq_with_p.check_feasible(v) {
                continue;
            }

            for d_pos in p_pos..self.data.len() {
                let next_id = self.data[d_pos].0.id;
                let dt_to_delivery = self
                    .instance
                    .distance_and_time(last_in_seq_with_p, delivery_id);

                if ref_delivery.due < seq_with_p.earliest_completion() + dt_to_delivery.time {
                    break;
                }

                let dt_to_next = self.instance.distance_and_time(delivery_id, next_id);

                let new_data = seq_with_p
                    .extend_forward(&ref_delivery, dt_to_delivery)
                    .concat(&self.data[d_pos].2, dt_to_next);

                if new_data.check_feasible(v) {
                    feasible_min_positions += 1;
                    if insertion.is_none() || rng.gen_bool(1.0 / feasible_min_positions as f64) {
                        insertion = Some(PDInsertion {
                            vn_id: self.data[0].0.id,
                            pickup_id,
                            pickup_after: self.data[p_pos - 1].0.id,
                            delivery_before: self.data[d_pos].0.id,
                        });
                    }
                }

                // extend seq_with_p for next iteration
                let dt_to_next = self.instance.distance_and_time(last_in_seq_with_p, next_id);
                seq_with_p.extend_forward_assign(&self.data[d_pos].0, dt_to_next);
                last_in_seq_with_p = next_id;

                if !&seq_with_p.check_feasible(v) {
                    break;
                }
            }
        }

        insertion.map(|it| (it, feasible_min_positions))
    }
}

struct REFRouteView<'a, 'b> {
    instance: &'a PDPTWInstance,
    solution: &'b Solution<'a>,
    route_id: usize,
    data: Vec<(REFNode, REFData, REFData)>,
}

impl<'a, 'b> REFRouteView<'a, 'b> {
    fn init(
        instance: &'a PDPTWInstance,
        solution: &'b Solution<'a>,
        route_id: usize,
        max_route_length: usize,
    ) -> Self {
        let data = Vec::with_capacity(max_route_length);

        Self {
            instance,
            solution,
            route_id,
            data,
        }
    }

    fn update_data_ignoring_requests(&mut self, ignore_requests: &FixedBitSet) {
        // init from scratch
        if self.data.is_empty() {
            let mut iter = self.solution.iter_route(self.route_id);
            if let Some(mut prev) = iter.next() {
                let prev_node: &Node = &self.instance.nodes[prev];
                let node = REFNode::from(prev_node);
                let fw_data = REFData::with_node(&node);
                let bw_data = REFData::with_node(&node);
                self.data.push((node, fw_data, bw_data));

                let mut update_from_here_forward = false;
                let mut last_idx_different = 0;

                while let Some(next) = iter.next() {
                    if !ignore_requests.contains(self.instance.request_id(next)) {
                        if update_from_here_forward {
                            let node = REFNode::from(&self.instance.nodes[next]);
                            let fw_data =
                                self.data.last().unwrap().1.extend_forward(
                                    &node,
                                    self.instance.distance_and_time(prev, next),
                                );
                            let bw_data = REFData::with_node(&node);
                            self.data.push((node, fw_data, bw_data));
                        } else {
                            // copy from solution
                            let node = REFNode::from(&self.instance.nodes[next]);
                            let fw_data = self.solution.fw_data[next].data.clone();
                            let bw_data = REFData::with_node(&node);
                            self.data.push((node, fw_data, bw_data));
                        }
                        prev = next;
                    } else {
                        last_idx_different = self.data.len();
                        update_from_here_forward = true;
                    }
                }

                for i in (1..self.data.len() - 1).rev() {
                    let prev = self.data[i].0.id;
                    if i > last_idx_different {
                        unsafe {
                            (*self.data.as_mut_ptr().add(i)).2 =
                                self.solution.bw_data[prev].data.clone();
                        }
                    } else {
                        let next = self.data[i + 1].0.id;
                        unsafe {
                            let next_entry_data = &(*self.data.as_ptr().add(i + 1)).2;
                            let prev_entry_node = &(*self.data.as_ptr().add(i)).0;
                            let prev_entry_data = &mut (*self.data.as_mut_ptr().add(i)).2;

                            next_entry_data.extend_backward_into_target(
                                prev_entry_node,
                                prev_entry_data,
                                self.instance.distance_and_time(prev, next),
                            );
                        }
                    }
                }
            }
        } else {
            // update only
            // Assumption: the first and last element (vehicle node) do not change!
            let mut iter = self.solution.iter_route(self.route_id);
            let mut prev = iter.next().unwrap();

            let mut update_from_here_forward = false;
            let mut last_idx_different = 0;
            let mut idx = 1;
            while let Some(next) = iter.next() {
                if !ignore_requests.contains(self.instance.request_id(next)) {
                    let next_node: &Node = &self.instance.nodes[next];
                    if self.data[idx].0.id != next_node.id {
                        last_idx_different = idx;
                        update_from_here_forward = true;
                    }

                    if update_from_here_forward {
                        let node = REFNode::from(next_node);
                        unsafe {
                            let prev_entry_data = &(*self.data.as_ptr().add(idx - 1)).1;
                            let next_entry_data = &mut (*self.data.as_mut_ptr().add(idx)).1;

                            prev_entry_data.extend_forward_into_target(
                                &node,
                                next_entry_data,
                                self.instance.distance_and_time(prev, next),
                            );
                        }
                        self.data[idx].0 = node;
                    }
                    idx += 1;
                    prev = next;
                }
            }

            for i in (1..=last_idx_different).rev() {
                let next = self.data[i + 1].0.id;
                let prev = self.data[i].0.id;
                unsafe {
                    let next_entry_data = &(*self.data.as_ptr().add(i + 1)).2;
                    let prev_entry_node = &(*self.data.as_ptr().add(i)).0;
                    let prev_entry_data = &mut (*self.data.as_mut_ptr().add(i)).2;

                    next_entry_data.extend_backward_into_target(
                        prev_entry_node,
                        prev_entry_data,
                        self.instance.distance_and_time(prev, next),
                    );
                }
            }
        }
    }

    pub fn find_best_insertion_for_request_while_ignoring_requests(
        &mut self,
        pickup_id: usize,
        ignore_requests: &FixedBitSet,
        rng: &mut Random,
    ) -> Option<(PDInsertion, usize)> {
        // can we quickly scan if there is a feasible insertion point before we update the REFRouteView?

        self.update_data_ignoring_requests(ignore_requests);

        let v = self.instance.vehicle_from_vn_id(self.data[0].0.id);
        let delivery_id = self.instance.delivery_of(pickup_id).id;
        let ref_pickup = REFNode::from(&self.instance.nodes[pickup_id]);
        let ref_delivery = REFNode::from(&self.instance.nodes[delivery_id]);

        let mut feasible_min_positions = 0;
        let mut insertion: Option<PDInsertion> = None;

        for p_pos in 1..self.data.len() {
            let prev_id = self.data[p_pos - 1].0.id;
            let dt_from_prev = self.instance.distance_and_time(prev_id, pickup_id);
            // if the due time of the pickup to be inserted is smaller than the earliest arrival from
            //  the previous visit we can ignore any subsequent visits as we will always have a
            //  feasibility issue.
            if ref_pickup.due < self.data[p_pos - 1].1.earliest_completion() + dt_from_prev.time {
                break;
            }

            let mut seq_with_p = self.data[p_pos - 1]
                .1
                .extend_forward(&ref_pickup, dt_from_prev);
            let mut last_in_seq_with_p = pickup_id;

            if !&seq_with_p.check_feasible(v) {
                continue;
            }

            for d_pos in p_pos..self.data.len() {
                let next_id = self.data[d_pos].0.id;
                let dt_to_delivery = self
                    .instance
                    .distance_and_time(last_in_seq_with_p, delivery_id);

                if ref_delivery.due < seq_with_p.earliest_completion() + dt_to_delivery.time {
                    break;
                }

                let dt_to_next = self.instance.distance_and_time(delivery_id, next_id);

                let new_data = seq_with_p
                    .extend_forward(&ref_delivery, dt_to_delivery)
                    .concat(&self.data[d_pos].2, dt_to_next);

                if new_data.check_feasible(v) {
                    feasible_min_positions += 1;
                    if insertion.is_none() || rng.gen_bool(1.0 / feasible_min_positions as f64) {
                        insertion = Some(PDInsertion {
                            vn_id: self.data[0].0.id,
                            pickup_id,
                            pickup_after: self.data[p_pos - 1].0.id,
                            delivery_before: self.data[d_pos].0.id,
                        });
                    }
                }

                // extend seq_with_p for next iteration
                let dt_to_next = self.instance.distance_and_time(last_in_seq_with_p, next_id);
                seq_with_p.extend_forward_assign(&self.data[d_pos].0, dt_to_next);
                last_in_seq_with_p = next_id;

                if !&seq_with_p.check_feasible(v) {
                    break;
                }
            }
        }

        insertion.map(|it| (it, feasible_min_positions))
    }
}
