use std::fs::File;
use std::io::{BufRead, BufReader, Lines};
#[cfg(feature = "classic-pdptw")]
use std::path::Path;

use anyhow::Context;

#[cfg(feature = "classic-pdptw")]
use crate::problem::pdptw::{create_instance_with, Vehicle};
use crate::problem::pdptw::{Capacity, Node, NodeType, PDPTWInstance};
use crate::problem::travel_matrix::{FixSizedTravelMatrix, FixSizedTravelMatrixBuilder};
use crate::problem::Arc;

/**
From the how_to_read.txt in https://github.com/cssartori/pdptw-instances in the /instance folder

----------

## Structure of instance files (or how to read them).

"Instances for the Pickup and Delivery Problem with Time Windows
based on open data"

===============================================================================
===============================================================================

The first 10 lines of each file contain general information about the
instance. These are:

NAME:          <unique instance identification>
LOCATION:      <city where it was generated>
COMMENT:       <a general reference>
TYPE:          <the type is always PDPTW>
SIZE:          <number of nodes including the depot>
DISTRIBUTION:  <the type of distribution>
DEPOT:         <type of depot location: either 'central' or 'random'>
ROUTE-TIME:    <time horizon>
TIME-WINDOW:   <time window width>
CAPACITY:      <maximum vehicle capacity>

Then, a line containing the word NODES is followed by SIZE lines
containing the complete information of each location (node) in the
instance file. For each line, there are 9 fields separated by a
single white space character as below:

<id> <lat> <long> <dem> <etw> <ltw> <dur> <p> <d>

The fields are:
<id>:   the unique node identifier ( node 0 is the unique depot )
<lat>:  latitude of this location
<lon>:  longitude of this location
<dem>:  the demand of this node ( dem > 0 for pickup, dem < 0 for delivery )
<etw>:  earliest possible time to start service (Time Window)
<ltw>:  latest possible time to start service (Time Window)
<dur>:  the service duration at this location
<p>:    the pickup pair if <id> is a delivery, and 0 otherwise
<d>:    the delivery pair if <id> is a pickup, and 0 otherwise


The <p> and <d> are for completeness reasons only. In all instances,
for a pickup location <id> its delivery is given by (<id>+((SIZE-1)/2)).
For a delivery location <id>, its pickup is given by (<id>-((SIZE-1)/2)).


After all NODES, there is a line containing the word EDGES followed by SIZE
lines, each one with SIZE integer values separated by a single white space
character. These integers represent the travel times between each pair of
locations in the instance, measured in minutes and computed using the
Open Source Routing Machine (OSRM)[1] tool.

All instances end with a line containing the word EOF.

===============================================================================
===============================================================================

NOTES:
(1): For cluster and cluster-random distributions, (x / y) indicate
that x seeds were chosen to create clusters and the density used
in the selection of locations was y.

(2): There are only two float values in an instance: <lat> and <lon>.
All other values are integers.
(except for the cluster density when applicable)

(3): The only information that is strictly necessary to solve an
instance is included in the following fields: CAPACITY, NODES, EDGES.
All the remaining values are either auxiliary (e.g., SIZE) or simply
additional information about the instance.

===============================================================================
===============================================================================

References:
[1]: Luxen, D., Vetter, C., 2011. Real-time routing with openstreetmap data.
In: Proceedings of the 19th ACM SIGSPATIAL International Conference
on Advances in Geographic Information Systems. GIS ’11. ACM, New York,
NY, USA, pp. 513–516.

===============================================================================
===============================================================================

# Carlo Sartori and Luciana Buriol (2019).
 */
#[cfg(feature = "classic-pdptw")]
pub(crate) fn load_instance(
    path: impl Into<String>,
    max_vehicles: Option<usize>,
) -> anyhow::Result<PDPTWInstance> {
    let path = path.into();
    let f = File::open(&path)?;
    let file = BufReader::new(&f);

    // line 0: 9 72
    // line 1: 480 1 6 0 1
    // line 10: 1	-4.927	9.670	3	30	1	1	0	0	0	1440

    let mut lines = file.lines();

    let (num_nodes, route_time, tw_width, capacity) = read_properties(&mut lines)?;
    let io_nodes = read_nodes(num_nodes, tw_width, &mut lines)?;

    let num_vehicles = max_vehicles.unwrap_or((num_nodes / 4).min(1000));
    let vehicles: Vec<Vehicle> = (0..num_vehicles)
        .map(|_| Vehicle {
            seats: capacity,
            shift_length: route_time.into(),
        })
        .collect();
    let nodes = transform_to_pdptw_nodes(io_nodes, vehicles.len())?;

    let travel_matrix =
        read_edges_and_create_travel_matrix(num_vehicles, num_nodes, &nodes, &mut lines)?;

    create_instance_with(
        Path::new(&path)
            .file_name()
            .unwrap()
            .to_string_lossy()
            .to_string(),
        num_vehicles,
        (num_nodes - 1) / 2,
        vehicles,
        nodes,
        travel_matrix,
    )
}

#[cfg(not(feature = "classic-pdptw"))]
pub(crate) fn load_instance(
    path: impl Into<String>,
    max_vehicles: Option<usize>,
) -> anyhow::Result<PDPTWInstance> {
    Err(anyhow::Error::msg(
        "not available with the active feature set (requires 'classic-pdptw' feature)",
    ))
}

fn parse_property_value_string(line: String) -> anyhow::Result<String> {
    let mut split = line.split(": ");
    split.next(); // discard the first part
    Ok(split
        .next()
        .context("line should contain a colon")?
        .to_string())
}

fn read_properties(
    lines: &mut Lines<BufReader<&File>>,
) -> anyhow::Result<(usize, usize, usize, Capacity)> {
    // NAME:          <unique instance identification>
    lines.next().unwrap()?;
    // LOCATION:      <city where it was generated>
    lines.next().unwrap()?;
    // COMMENT:       <a general reference>
    lines.next().unwrap()?;
    // TYPE:          <the type is always PDPTW>
    lines.next().unwrap()?;
    // SIZE:          <number of nodes including the depot>
    let size = parse_property_value_string(lines.next().unwrap()?)?.parse::<usize>()?;
    // DISTRIBUTION:  <the type of distribution>
    lines.next().unwrap()?;
    // DEPOT:         <type of depot location: either 'central' or 'random'>
    lines.next().unwrap()?;
    // ROUTE-TIME:    <time horizon>
    let route_time = parse_property_value_string(lines.next().unwrap()?)?.parse::<usize>()?;
    // TIME-WINDOW:   <time window width>
    let time_window = parse_property_value_string(lines.next().unwrap()?)?.parse::<usize>()?;
    // CAPACITY:      <maximum vehicle capacity>
    let capacity = parse_property_value_string(lines.next().unwrap()?)?.parse::<Capacity>()?;

    Ok((size, route_time, time_window, capacity))
}

struct IONode {
    /// <id>:   the unique node identifier ( node 0 is the unique depot )
    id: usize,
    /// <lat>:  latitude of this location
    lat: f64,
    /// <lon>:  longitude of this location
    lon: f64,
    /// <dem>:  the demand of this node ( dem > 0 for pickup, dem < 0 for delivery )
    demand: Capacity,
    /// <etw>:  earliest possible time to start service (Time Window)
    earliest: usize,
    /// <ltw>:  latest possible time to start service (Time Window)
    latest: usize,
    /// <dur>:  the service duration at this location
    service: usize,
    /// <p>:    the pickup pair if <id> is a delivery, and 0 otherwise
    p: usize,
    /// <d>:    the delivery pair if <id> is a pickup, and 0 otherwise
    d: usize,
}

fn read_nodes(
    num_nodes: usize,
    _tw_width: usize,
    lines: &mut Lines<BufReader<&File>>,
) -> anyhow::Result<Vec<IONode>> {
    // NODES
    lines.next().unwrap()?;

    let mut nodes = Vec::with_capacity(num_nodes);
    for _i in 0..num_nodes {
        let line = lines.next().unwrap()?;
        let mut split = line.split(" ");
        // <id> <lat> <long> <dem> <etw> <ltw> <dur> <p> <d>
        let id: usize = split.next().unwrap().parse()?;
        let lat: f64 = split.next().unwrap().parse()?;
        let lon: f64 = split.next().unwrap().parse()?;
        let demand: Capacity = split.next().unwrap().parse()?;
        let earliest: usize = split.next().unwrap().parse()?;
        let latest: usize = split.next().unwrap().parse()?;
        let service: usize = split.next().unwrap().parse()?;
        let p: usize = split.next().unwrap().parse()?;
        let d: usize = split.next().unwrap().parse()?;
        nodes.push(IONode {
            id,
            lat,
            lon,
            demand,
            earliest,
            latest,
            service,
            p,
            d,
        })
    }

    Ok(nodes)
}

fn read_edges_and_create_travel_matrix(
    num_vehicles: usize,
    num_nodes: usize,
    nodes: &Vec<Node>,
    lines: &mut Lines<BufReader<&File>>,
) -> anyhow::Result<FixSizedTravelMatrix> {
    let mut travel_matrix_builder = FixSizedTravelMatrixBuilder::with_num_nodes(nodes.len());

    // EDGES
    lines.next().unwrap()?;

    let num_requests = (num_nodes - 1) / 2;
    let oid_to_id = |oid: usize| -> usize {
        (num_vehicles * 2)
            + if oid > num_requests {
            // delivery
            (oid - 1 - num_requests) * 2 + 1
        } else {
            (oid - 1) * 2
        }
    };

    for i in 0..num_nodes {
        // each one with SIZE integer values separated by a single white space
        // character. These integers represent the travel times between each pair of
        // locations in the instance, measured in minutes
        let line = lines.next().unwrap()?;
        let mut split = line.split(" ");
        for j in 0..num_nodes {
            let value: usize = split.next().unwrap().parse()?;

            // we know the offset of vehicle nodes
            // so for 0 or to 0, we have special rules
            // for from=0 && to=0 we can assume that this is zero which the builder already
            // initialize with
            if i == j {
                if i == 0 {
                    for u in (0..num_vehicles * 2).step_by(2) {
                        travel_matrix_builder.set_arc(Arc {
                            from: u,
                            to: u + 1,
                            distance: 0.into(),
                            time: 0.into(),
                        });
                    }
                }
            } else if i == 0 || j == 0 {
                // push that value to all vehicle nodes
                if i == 0 {
                    // but j > 0
                    for u in (0..num_vehicles * 2).step_by(2) {
                        debug_assert_eq!(
                            oid_to_id(j),
                            nodes.iter().find(|n| n.oid == j).unwrap().id
                        );
                        travel_matrix_builder.set_arc(Arc {
                            from: u,
                            to: oid_to_id(j),
                            distance: value.into(),
                            time: value.into(),
                        });
                    }
                } else {
                    // i > 0 => j == 0
                    for v in (1..num_vehicles * 2).step_by(2) {
                        debug_assert_eq!(
                            oid_to_id(i),
                            nodes.iter().find(|n| n.oid == i).unwrap().id
                        );
                        travel_matrix_builder.set_arc(Arc {
                            from: oid_to_id(i),
                            to: v,
                            distance: value.into(),
                            time: value.into(),
                        });
                    }
                }
            } else {
                debug_assert_eq!(oid_to_id(i), nodes.iter().find(|n| n.oid == i).unwrap().id);
                debug_assert_eq!(oid_to_id(j), nodes.iter().find(|n| n.oid == j).unwrap().id);
                travel_matrix_builder.set_arc(Arc {
                    from: oid_to_id(i),
                    to: oid_to_id(j),
                    distance: value.into(),
                    time: value.into(),
                });
            }
        }
    }

    Ok(travel_matrix_builder.build())
}

fn transform_to_pdptw_nodes(
    io_nodes: Vec<IONode>,
    num_vehicles: usize,
) -> anyhow::Result<Vec<Node>> {
    // depot node is duplicated for each vehicle
    let mut nodes = Vec::with_capacity(io_nodes.len() - 1 + num_vehicles);

    // first do the vehicle nodes
    for id in 0..num_vehicles {
        let depot = &io_nodes[0];
        nodes.push(Node {
            id: (id * 2),
            oid: depot.id,
            gid: depot.id,
            x: depot.lon,
            y: depot.lat,
            demand: depot.demand,
            ready: depot.earliest.into(),
            due: depot.latest.into(),
            servicetime: depot.service.into(),
            node_type: NodeType::Depot,
        });
        nodes.push(Node {
            id: (id * 2) + 1,
            oid: depot.id,
            gid: depot.id,
            x: depot.lon,
            y: depot.lat,
            demand: depot.demand,
            ready: depot.earliest.into(),
            due: depot.latest.into(),
            servicetime: depot.service.into(),
            node_type: NodeType::Depot,
        })
    }

    let mut id = nodes.len();
    let num_requests = (io_nodes.len() - 1) / 2;
    for i in 0..num_requests {
        // pickup
        let node = &io_nodes[i + 1];
        nodes.push(Node {
            id,
            oid: node.id,
            gid: node.id,
            x: node.lon,
            y: node.lat,
            demand: node.demand,
            ready: node.earliest.into(),
            due: node.latest.into(),
            servicetime: node.service.into(),
            node_type: NodeType::Pickup,
        });

        // delivery
        let node = &io_nodes[(i + num_requests) + 1];
        nodes.push(Node {
            id: id + 1,
            oid: node.id,
            gid: node.id,
            x: node.lon,
            y: node.lat,
            demand: node.demand,
            ready: node.earliest.into(),
            due: node.latest.into(),
            servicetime: node.service.into(),
            node_type: NodeType::Delivery,
        });

        id += 2;
    }

    Ok(nodes)
}

#[cfg(test)]
mod tests {
    use crate::io::sartori_buriol_reader::load_instance;

    #[test]
    fn read_100_instance() -> anyhow::Result<()> {
        let instance = load_instance(
            "resources/instances/SartoriBuriol/n100/bar-n100-1.txt",
            None,
        )
            .expect("Instance not loaded");
        println!("{:?}", &instance);
        assert_eq!(instance.num_requests, 50);
        Ok(())
    }

    #[test]
    fn test_travel_matrix_with_solutions() -> anyhow::Result<()> {
        use crate::io::sintef_solution::tests::sartori_buriol::INSTANCE_DIR_N100;
        use crate::io::sintef_solution::tests::sartori_buriol::N100;

        for (instance_name, _solution_name, ref_vehicles, _ref_obj) in N100.iter() {
            let instance_path = format!("{}/{}", INSTANCE_DIR_N100, instance_name);

            crate::io::sartori_buriol_reader::load_instance(
                instance_path,
                Some(*ref_vehicles),
            )?;
        }

        Ok(())
    }
}
