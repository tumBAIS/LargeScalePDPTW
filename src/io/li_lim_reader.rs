use std::fs::File;
use std::io::{BufRead, BufReader, Lines};
#[cfg(feature = "classic-pdptw")]
use std::path::Path;

#[cfg(feature = "classic-pdptw")]
use crate::problem::pdptw::{create_instance_with, PDPTWInstance, Vehicle};
use crate::problem::pdptw::{Capacity, Node, NodeType};
use crate::problem::travel_matrix::FixSizedTravelMatrix;

/**
From https://www.sintef.no/projectweb/top/pdptw/documentation/

----------

Below is an explanation of the format of the instance definitions (text files). Note that tabulator is used as field separator rather than spaces.

NUMBER OF VEHICLES  VEHICLE CAPACITY    SPEED(not used)
K   Q   S
TASK NO.    X   Y   DEMAND  EARLIEST PICKUP/DELIVERY TIME   LATEST PICKUP/DELIVERY TIME SERVICE TIME    PICKUP(index to sibling)    DELIVERY(index to sibling)
0   x0  y0  q0  e0  l0  s0  p0  d0
1   x1  y1  q1  e1  l1  s1  p1  d1
...

Task 0 specifies the coordinates of the depot. For pickup tasks, the PICKUP index is 0, whereas
the DELIVERY sibling gives the index of the corresponding delivery task. For delivery tasks, the
PICKUP index gives the index of the corresponding pickup task. The value of travel time is equal
to the value of distance.
 */
#[cfg(feature = "classic-pdptw")]
pub(crate) fn load_instance(
    path: impl Into<String>,
    max_vehicles: Option<usize>,
) -> anyhow::Result<PDPTWInstance> {
    let path: String = path.into();
    let f = File::open(&path)?;
    let file = BufReader::new(&f);

    // line 0: K Q S
    // line 1: 0 x0 y0 q0 e0 l0 s0 p0 d0
    // line 2: 1 x1 y1 q1 e1 l1 s1 p1 d1
    // ...

    let mut lines = file.lines();

    let (_num_vehicles, capacity, _speed) = read_properties(&mut lines)?;
    let io_nodes = read_nodes(&mut lines)?;
    let num_nodes = io_nodes.len();

    let num_vehicles = max_vehicles.unwrap_or((num_nodes - 1) / 2);

    let vehicles: Vec<Vehicle> = (0..num_vehicles)
        .map(|_| Vehicle {
            seats: capacity.into(),
            shift_length: io_nodes[0].latest.into(),
        })
        .collect();
    let nodes = transform_to_pdptw_nodes(io_nodes, vehicles.len())?;

    let travel_matrix = create_travel_matrix(&nodes)?;

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

fn parse_property_value_string(line: String) -> anyhow::Result<String> {
    let mut split = line.split(": ");
    split.next().unwrap(); // discard the first part
    Ok(split.next().unwrap().to_string())
}

fn read_properties(
    lines: &mut Lines<BufReader<&File>>,
) -> anyhow::Result<(usize, Capacity, usize)> {
    // K Q S
    let line = lines.next().unwrap()?;
    let mut split = line.split("\t");
    let num_vehicles = split.next().unwrap().parse::<usize>()?;
    let capacity = split.next().unwrap().parse::<Capacity>()?;
    let speed = split.next().unwrap().parse::<usize>()?;

    Ok((num_vehicles, capacity, speed))
}

#[derive(Debug)]
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

fn read_nodes(lines: &mut Lines<BufReader<&File>>) -> anyhow::Result<Vec<IONode>> {
    let mut nodes = Vec::new();
    while let Some(line) = lines.next() {
        let line = line.unwrap();
        let line = line.trim();
        if line.is_empty() {
            break;
        }
        let mut split = line.split("\t");
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

fn create_travel_matrix(nodes: &Vec<Node>) -> anyhow::Result<FixSizedTravelMatrix> {
    Ok(FixSizedTravelMatrix::with_euclidean_distances(
        &nodes.iter().map(|it| (it.x, it.y)).collect(),
    ))
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
    for i in 1..io_nodes.len() {
        if io_nodes[i].p == 0 {
            // pickup
            let pickup = &io_nodes[i];
            nodes.push(Node {
                id,
                oid: pickup.id,
                gid: pickup.id,
                x: pickup.lon,
                y: pickup.lat,
                demand: pickup.demand,
                ready: pickup.earliest.into(),
                due: pickup.latest.into(),
                servicetime: pickup.service.into(),
                node_type: NodeType::Pickup,
            });

            // delivery
            let delivery = &io_nodes[pickup.d];
            nodes.push(Node {
                id: id + 1,
                oid: delivery.id,
                gid: pickup.id,
                x: delivery.lon,
                y: delivery.lat,
                demand: delivery.demand,
                ready: delivery.earliest.into(),
                due: delivery.latest.into(),
                servicetime: delivery.service.into(),
                node_type: NodeType::Delivery,
            });

            debug_assert_eq!(pickup.d, delivery.id);
            debug_assert_eq!(delivery.p, pickup.id);

            id += 2;
        }
    }

    Ok(nodes)
}

#[cfg(test)]
mod tests {
    use crate::io::li_lim_reader::load_instance;

    #[test]
    fn read_100_instance() -> anyhow::Result<()> {
        let instance = load_instance("resources/instances/LiLim/pdp_100/lr108.txt", None)
            .expect("Instance not loaded");
        println!("{:?}", &instance);
        assert_eq!(instance.num_requests, 50);
        Ok(())
    }

    #[test]
    fn read_another_100_instance() -> anyhow::Result<()> {
        let instance = load_instance("resources/instances/LiLim/pdp_100/lc103.txt", None)
            .expect("Instance not loaded");
        println!("{:?}", &instance);
        assert_eq!(instance.num_requests, 52);
        Ok(())
    }
}
