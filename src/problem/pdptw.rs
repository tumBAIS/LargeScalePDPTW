use std::fmt::{Debug, Formatter};

use anyhow::Result;
use log::warn;

#[cfg(feature = "classic-pdptw")]
use crate::problem::travel_matrix::FixSizedTravelMatrix;
use crate::problem::travel_matrix::TravelMatrix;
#[cfg(not(feature = "classic-pdptw"))]
use crate::problem::travel_matrix::TravelMatrixProxy;
use crate::problem::Num;

pub(crate) type Capacity = i16;

#[derive(Debug, Clone)]
pub struct Vehicle {
    pub seats: Capacity,
    pub shift_length: Num,
}

impl Vehicle {
    pub fn check_capacity(&self, demand: Capacity) -> bool {
        demand <= self.seats
    }
}

#[derive(Clone, Debug)]
pub enum NodeType {
    Depot,
    Pickup,
    Delivery,
}

impl NodeType {
    pub fn is_depot(&self) -> bool {
        match self {
            Self::Depot => true,
            _ => false,
        }
    }
    pub fn is_pickup(&self) -> bool {
        match self {
            Self::Pickup => true,
            _ => false,
        }
    }
    pub fn is_delivery(&self) -> bool {
        match self {
            Self::Delivery => true,
            _ => false,
        }
    }
    pub fn is_request(&self) -> bool {
        match self {
            Self::Pickup | Self::Delivery => true,
            _ => false,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Node {
    pub id: usize,
    pub oid: usize,
    pub gid: usize,
    pub node_type: NodeType,
    pub x: f64,
    pub y: f64,
    pub demand: Capacity,
    pub ready: Num,
    pub due: Num,
    pub servicetime: Num,
}

pub struct PDPTWInstance {
    pub name: String,
    pub num_requests: usize,
    pub num_vehicles: usize,
    pub nodes: Vec<Node>,
    pub vehicles: Vec<Vehicle>,
    #[cfg(feature = "classic-pdptw")]
    pub(crate) travel_matrix: FixSizedTravelMatrix,
    #[cfg(not(feature = "classic-pdptw"))]
    pub(crate) travel_matrix: TravelMatrixProxy,
}

impl Debug for PDPTWInstance {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "PDPTW instance:")
            .and(writeln!(
                f,
                "{} vehicles: {:?}",
                self.vehicles.len(),
                self.vehicles
            ))
            .and(write!(
                f,
                "{} requests: {:?}",
                self.num_requests,
                self.nodes[self.vehicles.len()..(self.vehicles.len() + self.num_requests * 2)]
                    .iter()
                    .collect::<Vec<&Node>>()
            ))
    }
}

#[derive(Clone, Copy, Default)]
pub struct DistanceAndTime {
    pub distance: Num,
    pub time: Num,
}

impl PDPTWInstance {
    pub fn distance(&self, from: usize, to: usize) -> Num {
        self.travel_matrix.distance(from, to).into()
    }
    pub fn time(&self, from: usize, to: usize) -> Num {
        self.travel_matrix.time(from, to).into()
    }
    pub fn distance_and_time(&self, from: usize, to: usize) -> DistanceAndTime {
        let arc = self.travel_matrix.arc(from, to);
        DistanceAndTime {
            distance: arc.distance.into(),
            time: arc.time.into(),
        }
    }
    pub fn vehicle_from_vn_id(&self, vn_id: usize) -> &Vehicle {
        &self.vehicles[vn_id / 2]
    }
    pub fn vn_id_of(&self, v_id: usize) -> usize {
        v_id * 2
    }
    pub fn iter_vn(&self) -> impl Iterator<Item=&Node> {
        self.nodes
            .iter()
            .take(self.num_vehicles * 2)
            .filter(|n| n.id % 2 == 0)
    }
    pub fn iter_pickups(&self) -> impl Iterator<Item=&Node> {
        self.nodes
            .iter()
            .skip(self.num_vehicles * 2)
            .filter(|n| n.id % 2 == 0)
    }
    pub fn node_type(&self, id: usize) -> &NodeType {
        &self.nodes[id].node_type
    }
    pub fn is_request(&self, node_id: usize) -> bool {
        node_id >= self.num_vehicles * 2
    }
    pub fn is_pickup(&self, node_id: usize) -> bool {
        self.is_request(node_id) && node_id % 2 == 0
    }
    pub fn is_delivery(&self, node_id: usize) -> bool {
        self.is_request(node_id) && node_id % 2 == 1
    }
    pub fn pickup_of(&self, delivery_id: usize) -> &Node {
        &self.nodes[delivery_id - 1]
    }
    pub fn delivery_of(&self, pickup_id: usize) -> &Node {
        &self.nodes[pickup_id + 1]
    }
    pub fn pair_of(&self, node_id: usize) -> &Node {
        match self.nodes[node_id].node_type {
            NodeType::Pickup => &self.nodes[node_id + 1],
            NodeType::Delivery => &self.nodes[node_id - 1],
            _ => panic!(),
        }
    }
    pub fn request_id(&self, node_id: usize) -> usize {
        (node_id / 2) - self.vehicles.len()
    }
    pub fn pickup_id_of_request(&self, request_id: usize) -> usize {
        (request_id + self.vehicles.len()) * 2
    }
    pub fn delivery_id_of_request(&self, request_id: usize) -> usize {
        (request_id + self.vehicles.len()) * 2 + 1
    }
}

pub fn create_instance_with(
    name: String,
    num_vehicles: usize,
    num_requests: usize,
    vehicles: Vec<Vehicle>,
    mut nodes: Vec<Node>,
    #[cfg(feature = "classic-pdptw")] travel_matrix: FixSizedTravelMatrix,
    #[cfg(not(feature = "classic-pdptw"))] travel_matrix: TravelMatrixProxy,
) -> Result<PDPTWInstance> {
    // Preprocessing - tighten time windows
    for i in 0..num_requests {
        let p_id = (num_vehicles * 2) + (i * 2);
        let d_id = p_id + 1;

        // ensure that we can reach at least one vehicle time window feasibly
        let (earliest_arrival, latest_departure) =
            (0..vehicles.len())
                .into_iter()
                .fold((Num::MAX, Num::MIN), |(e, l), v_id| {
                    let travel_time_v_p: Num = travel_matrix.time(v_id * 2, p_id).into();
                    let travel_time_d_v: Num = travel_matrix.time(d_id, v_id * 2 + 1).into();
                    (
                        e.min(nodes[v_id * 2].ready + travel_time_v_p),
                        l.max(nodes[v_id * 2 + 1].due - travel_time_d_v),
                    )
                });

        nodes[p_id].ready = nodes[p_id].ready.max(earliest_arrival).min(nodes[p_id].due);

        #[cfg(
            not(feature = "classic-pdptw")
        )] // nodes may not be reachable - remove them from the instance
        if nodes[p_id].ready > nodes[d_id].due - (Num::from(travel_matrix.time(p_id, d_id))) {
            warn!(
                "p_id: {} is not reachable in time to deliver (rdy: {}, due: {}, tt: {})",
                p_id,
                nodes[p_id].ready,
                nodes[d_id].due,
                travel_matrix.time(p_id, d_id)
            );
            if p_id < d_id {
                nodes.remove(d_id);
                nodes.remove(p_id);
            } else {
                nodes.remove(p_id);
                nodes.remove(d_id);
            }
        }

        #[cfg(feature = "classic-pdptw")] // ensure that all nodes are reachable
        assert!(
            nodes[p_id].ready <= nodes[d_id].due - (Num::from(travel_matrix.time(p_id, d_id))),
            "p_id: {} is not reachable in time to deliver (rdy: {}, due: {}, tt: {})",
            p_id,
            nodes[p_id].ready,
            nodes[d_id].due,
            travel_matrix.time(p_id, d_id)
        );

        nodes[d_id].due = nodes[d_id]
            .due
            .min(latest_departure - nodes[d_id].servicetime);

        let tt = travel_matrix.time(p_id, d_id).into();
        // tighten delivery node tw
        let p_rdy = nodes[p_id].ready;
        let p_st = nodes[p_id].servicetime;
        nodes[d_id].ready = nodes[d_id].ready.max(p_rdy + p_st + tt);

        // tighten pickup node tw
        let d_due = nodes[d_id].due;
        if tt + p_st > d_due {
            warn!("tt + p_st > d_due ({} + {} > {})", tt, p_st, d_due);
            warn!("nodes[p_id].oid: {}, nodes[d_id].oid: {}", nodes[p_id].oid, nodes[d_id].oid);
            warn!("nodes[p_id].x: {}, nodes[p_id].y: {}", nodes[p_id].x, nodes[p_id].y);
            warn!("nodes[d_id].x: {}, nodes[d_id].y: {}", nodes[d_id].x, nodes[d_id].y);
            warn!("p_rdy: {}, p_due: {}", p_rdy, d_due);
            warn!("travel_matrix.distance(p_id, d_id): {}", travel_matrix.distance(p_id, d_id));
        }
        nodes[p_id].due = nodes[p_id].due.min(d_due - tt - p_st);
    }

    Ok(PDPTWInstance {
        name,
        num_requests,
        num_vehicles,
        nodes,
        vehicles,
        travel_matrix,
    })
}

#[cfg(test)]
mod tests {}
