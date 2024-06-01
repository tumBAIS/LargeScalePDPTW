use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

use anyhow::Context;
use log::info;

use crate::problem::pdptw::{Capacity, Node, NodeType, Vehicle};
use crate::problem::travel_matrix::{FixSizedTravelMatrix, FixSizedTravelMatrixBuilder};
use crate::problem::{Arc, Num};

mod toml {
    use std::fs::File;
    use std::io::Read;

    use serde::Deserialize;

    #[derive(Debug, Deserialize)]
    pub struct Config {
        pub instance: Instance,
        pub vehicles: Vehicles,
        pub requests: Requests,
        pub distance_matrix: DistanceMatrix,
    }

    #[derive(Debug, Deserialize)]
    pub struct Instance {
        pub start_date: String,
        pub start_time: String,
        pub start_timestamp: u64,
        pub end_date: String,
        pub end_time: String,
        pub end_timestamp: u64,
    }

    #[derive(Debug, Deserialize)]
    pub struct Vehicles {
        pub num_vehicles: usize,
        pub csv: String,
    }

    #[derive(Debug, Deserialize)]
    pub struct Requests {
        pub num_request_nodes: usize,
        pub csv: String,
    }

    #[derive(Debug, Deserialize)]
    pub struct DistanceMatrix {
        pub num_nodes: usize,
        pub csv: String,
    }

    pub fn read_config(path: impl Into<String>) -> anyhow::Result<Config> {
        let mut s = String::new();
        File::open(path.into())?.read_to_string(&mut s)?;
        Ok(toml::from_str(s.as_str())?)
    }
}

#[derive(Debug)]
pub struct IOVehicle {
    pub uid: usize,
    pub gid: usize,
    pub lat: f64,
    pub lon: f64,
    pub ready: usize,
    pub due: usize,
    pub capacity: usize,
}

pub fn read_vehicles(
    directory: impl Into<String>,
    config: toml::Vehicles,
) -> anyhow::Result<Vec<IOVehicle>> {
    let file = File::open(format!("{}/{}", directory.into(), config.csv))?;
    let reader = BufReader::new(file);
    let lines = reader.lines();
    let mut vehicles = Vec::with_capacity(config.num_vehicles);
    for next_line in lines {
        let line = next_line?;
        let trimed_line = line.trim();
        // id,lon,lat,ready,due,capacity
        let mut split = trimed_line.split(",");
        let uid = split
            .next()
            .context("expected next split: uid")?
            .parse::<usize>()?;
        let lon = split
            .next()
            .context("expected next split: lon")?
            .parse::<f64>()?;
        let lat = split
            .next()
            .context("expected next split: lat")?
            .parse::<f64>()?;
        let ready = split
            .next()
            .context("expected next split: ready")?
            .parse::<usize>()?;
        let due = split
            .next()
            .context("expected next split: due")?
            .parse::<usize>()?;
        let capacity = split
            .next()
            .context("expected next split: capacity")?
            .parse::<usize>()?;
        let gid = split
            .next()
            .context("expected next split: gid")?
            .parse::<usize>()?;
        vehicles.push(IOVehicle {
            uid,
            lat,
            lon,
            ready,
            due,
            capacity,
            gid,
        });
    }
    Ok(vehicles)
}

#[derive(Debug)]
pub enum IORequestPart {
    Pickup,
    Delivery,
}

#[derive(Debug)]
pub struct IORequestNode {
    pub uid: usize,
    pub rid: usize,
    pub gid: usize,
    pub lat: f64,
    pub lon: f64,
    pub ready: usize,
    pub due: usize,
    pub demand: isize,
    pub part: IORequestPart,
}

pub fn read_nodes(
    directory: impl Into<String>,
    config: toml::Requests,
) -> anyhow::Result<Vec<IORequestNode>> {
    let file = File::open(format!("{}/{}", directory.into(), config.csv))?;
    let reader = BufReader::new(file);
    let lines = reader.lines();
    let mut request_nodes = Vec::with_capacity(config.num_request_nodes);
    for next_line in lines {
        let line = next_line?;
        let trimed_line = line.trim();
        // uid,rid,lon,lat,ready,due,demand,type
        let mut split = trimed_line.split(",");
        let uid = split
            .next()
            .context("expected next split: uid")?
            .parse::<usize>()?;
        let rid = split
            .next()
            .context("expected next split: rid")?
            .parse::<usize>()?;
        let lon = split
            .next()
            .context("expected next split: lon")?
            .parse::<f64>()?;
        let lat = split
            .next()
            .context("expected next split: lat")?
            .parse::<f64>()?;
        let ready = split
            .next()
            .context("expected next split: ready")?
            .parse::<usize>()?;
        let due = split
            .next()
            .context("expected next split: due")?
            .parse::<usize>()?;
        let demand = split
            .next()
            .context("expected next split: capacity")?
            .parse::<isize>()?;
        let part = match split.next().context("expected next split: part")? {
            "p" => IORequestPart::Pickup,
            "d" => IORequestPart::Delivery,
            x => {
                return Err(anyhow::Error::msg(format!(
                    "unrecognized requests-part type {} -- expected 'p' or 'd'",
                    x
                )));
            }
        };
        let gid = split
            .next()
            .context("expected next split: gid")?
            .parse::<usize>()?;
        request_nodes.push(IORequestNode {
            uid,
            rid,
            lat,
            lon,
            ready,
            due,
            demand,
            part,
            gid,
        });
    }
    Ok(request_nodes)
}

pub struct NYCInstanceElements {
    pub name: String,
    pub num_vehicles: usize,
    pub num_requests: usize,
    pub vehicles: Vec<Vehicle>,
    pub nodes: Vec<Node>,
    pub travel_matrix: FixSizedTravelMatrix,
}

/**

 */
pub(crate) fn load_instance(toml_path: impl Into<String>) -> anyhow::Result<NYCInstanceElements> {
    let toml_path = toml_path.into();
    let config = toml::read_config(toml_path.clone())?;
    let name = Path::new(toml_path.as_str())
        .file_name()
        .unwrap()
        .to_string_lossy();
    let directory = Path::new(toml_path.as_str())
        .parent()
        .unwrap()
        .to_str()
        .unwrap();
    let io_vehicles = read_vehicles(directory, config.vehicles)?;
    let io_request_nodes = read_nodes(directory, config.requests)?;

    let num_vehicles = io_vehicles.len();
    let num_requests = io_request_nodes.len() / 2;
    let route_time = (config.instance.end_timestamp - config.instance.start_timestamp) as usize;

    let vehicles: Vec<Vehicle> = io_vehicles
        .iter()
        .map(|it| Vehicle {
            seats: it.capacity as i16,
            shift_length: route_time.into(),
        })
        .collect();

    let num_streetnetwork_nodes = config.distance_matrix.num_nodes;
    let travel_matrix = read_distances_and_create_travel_matrix(directory, config.distance_matrix)?;

    let nodes = transform_to_pdptw_nodes(io_vehicles, io_request_nodes, num_streetnetwork_nodes)?;

    Ok(NYCInstanceElements {
        name: name.to_string(),
        num_vehicles,
        num_requests,
        vehicles,
        nodes,
        travel_matrix,
    })
}

fn transform_to_pdptw_nodes(
    io_vehicles: Vec<IOVehicle>,
    io_nodes: Vec<IORequestNode>,
    num_streetnetwork_nodes: usize,
) -> anyhow::Result<Vec<Node>> {
    let num_nodes = io_nodes.len() + (io_vehicles.len()) * 2;
    let mut nodes = Vec::with_capacity(num_nodes);

    // first do the vehicle nodes
    // for id in 0..num_vehicles {
    for vehicle in io_vehicles {
        nodes.push(Node {
            id: vehicle.uid * 2,
            oid: vehicle.uid,
            gid: vehicle.gid,
            x: vehicle.lon,
            y: vehicle.lat,
            demand: 0,
            ready: Num::from(vehicle.ready),
            due: Num::from(vehicle.due),
            servicetime: Num::ZERO,
            node_type: NodeType::Depot,
        });
        nodes.push(Node {
            id: vehicle.uid * 2 + 1,
            oid: vehicle.uid,
            gid: num_streetnetwork_nodes,
            x: vehicle.lon,
            y: vehicle.lat,
            demand: 0,
            ready: Num::from(vehicle.ready),
            due: Num::from(vehicle.due),
            servicetime: Num::ZERO,
            node_type: NodeType::Depot,
        });
    }

    let mut id = nodes.len();
    for node in io_nodes {
        nodes.push(Node {
            id,
            oid: node.uid,
            gid: node.gid,
            x: node.lon,
            y: node.lat,
            demand: node.demand as Capacity,
            ready: node.ready.into(),
            due: node.due.into(),
            servicetime: Num::ZERO,
            node_type: match node.part {
                IORequestPart::Pickup => NodeType::Pickup,
                IORequestPart::Delivery => NodeType::Delivery,
            },
        });
        id += 1;
    }

    Ok(nodes)
}

fn haversine(lon1: f64, lat1: f64, lon2: f64, lat2: f64) -> f64 {
    let d_lat = (lat2 - lat1).to_radians();
    let d_lon = (lon2 - lon1).to_radians();

    let a = (d_lat / 2.0).powf(2.0).sin()
        + (d_lon / 2.0).powf(2.0).sin() * lat1.to_radians().cos() * lat2.to_radians().cos();
    let dist_in_km = 6371.0 * (2.0 * a.sqrt().asin());
    dist_in_km
}

fn distance_to_time(distance: f64) -> f64 {
    let kmh = 20;
    (distance as f64 / (kmh as f64 * (1f64 / 60f64 / 60f64))).ceil()
}

fn create_traveltime_matrix_from_scratch(
    io_vehicles: &Vec<IOVehicle>,
    io_nodes: &Vec<IORequestNode>,
) -> anyhow::Result<FixSizedTravelMatrix> {
    info!("calculating arcs");
    let mut builder =
        FixSizedTravelMatrixBuilder::with_num_nodes(io_vehicles.len() * 2 + io_nodes.len());

    #[cfg(feature = "progressbar")]
        let mut pb = pbr::ProgressBar::new(io_vehicles.len() * 2 * io_nodes.len() + io_nodes.len() * 2 as u64);
    for v in io_vehicles {
        for n in io_nodes {
            let distance = haversine(v.lon, v.lat, n.lon, n.lat);
            let time = distance_to_time(distance);
            builder.set_arc(Arc {
                from: v.uid * 2,
                to: n.uid,
                distance: distance.into(),
                time: time.into(),
            });
            builder.set_arc(Arc {
                from: n.uid,
                to: v.uid * 2 + 1,
                distance: Num::ZERO,
                time: Num::ZERO,
            });
            #[cfg(feature = "progressbar")]
            pb.add(2);
        }
    }

    for n1 in io_nodes {
        for n2 in io_nodes {
            if n1.uid != n2.uid {
                let distance = haversine(n1.lon, n1.lat, n2.lon, n2.lat);
                let time = distance_to_time(distance);
                builder.set_arc(Arc {
                    from: n1.uid,
                    to: n2.uid,
                    distance: distance.into(),
                    time: time.into(),
                });
            }
            #[cfg(feature = "progressbar")]
            pb.inc();
        }
    }

    Ok(builder.build())
}

pub fn read_distances_and_create_travel_matrix(
    directory: impl Into<String>,
    config: toml::DistanceMatrix,
) -> anyhow::Result<FixSizedTravelMatrix> {
    let file = File::open(format!("{}/{}", directory.into(), config.csv))?;
    let reader = BufReader::new(file);
    let lines = reader.lines();
    let mut travel_matrix_builder =
        FixSizedTravelMatrixBuilder::with_num_nodes(config.num_nodes + 1);
    for (i, next_line) in lines.enumerate() {
        let line = next_line?;
        let trimed_line = line.trim();
        // 0,21,2,34,23,124
        let mut split = trimed_line.split(",");
        for j in 0..config.num_nodes {
            let value: usize = split.next().unwrap().parse()?;
            let distance = value as f64;
            travel_matrix_builder.set_arc(Arc {
                from: i,
                to: j,
                distance: distance.into(),
                time: (distance_to_time(distance / 1000f64)).into(),
            });
        }
    }
    for i in 0..config.num_nodes {
        travel_matrix_builder.set_arc(Arc {
            from: i,
            to: config.num_nodes,
            distance: Num::ZERO.into(),
            time: Num::ZERO.into(),
        });
    }

    Ok(travel_matrix_builder.build())
}
