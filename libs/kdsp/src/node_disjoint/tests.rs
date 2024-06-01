use crate::node_disjoint::{SourceSinkGraph, SourceSinkGraphBuilder};
use crate::Vertex;
use std::fs::File;
use std::io::{BufRead, BufReader, Error};
use std::fmt::{Display, Debug, Formatter};
use std::num::ParseIntError;

pub struct Solution {
    pub paths: Vec<Vec<Vertex>>,
}

#[derive(Debug)]
pub struct ReadInstanceError {
    message: String
}
impl Display for ReadInstanceError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "Error while reading the instance: {}", self.message)
    }
}

impl From<Error> for ReadInstanceError {
    fn from(error: Error) -> Self {
        ReadInstanceError {
            message: error.to_string(),
        }
    }
}
impl From<ParseIntError> for ReadInstanceError {
    fn from(error: ParseIntError) -> Self {
        ReadInstanceError {
            message: error.to_string(),
        }
    }
}

pub fn read_instance(
    path: impl Into<String>,
) -> Result<(SourceSinkGraph<i32>, Option<Solution>), ReadInstanceError> {
    let f = File::open(path.into())?;
    let file = BufReader::new(&f);

    let mut lines = file.lines();

    let _name = lines.next().unwrap()?;
    let num_nodes = lines.next().unwrap()?.parse::<Vertex>()?;
    let num_edges = lines.next().unwrap()?.parse::<usize>()?;

    lines.next().unwrap()?; // empty line

    let mut arcs = Vec::with_capacity(num_edges);
    for _ in 0..num_edges {
        let line = lines.next().unwrap()?;

        let mut split = line.split(",");
        let from = split.next().unwrap().parse::<Vertex>()?;
        let to = split.next().unwrap().parse::<Vertex>()?;
        let w = split.next().unwrap().parse::<i32>()?;

        let arc = crate::Arc { from, to, w };

        arcs.push(arc);
    }

    lines.next().unwrap()?; // empty line

    let sol = {
        let mut paths = vec![];
        while let Some(res) = lines.next() {
            let line = res?;
            if line.trim().is_empty() {
                break;
            }

            let mut sp: Vec<Vertex> = vec![];
            for it in line.split(",") {
                sp.push(it.parse()?);
            }
            paths.push(sp);
        }
        if paths.is_empty() {
            None
        } else {
            Some(Solution { paths })
        }
    };

    let graph = SourceSinkGraphBuilder::new()
        .set_num_nodes(num_nodes)
        .set_source(0)
        .set_sink(1)
        .add_arcs(arcs.into_iter())
        .build();

    Ok((graph, sol))
}

pub const TEST_GRAPHS_DIR: &str = "./resources/test_graphs/";
pub const TEST_GRAPHS: [&str; 12] = [
    "a2-16hetARX.txt",
    "a2-20hetARX.txt",
    "a2-24hetARX.txt",
    "a3-18hetARX.txt",
    "a3-24hetARX.txt",
    "a3-30hetARX.txt",
    "a3-36hetARX.txt",
    "a4-16hetARX.txt",
    "a4-24hetARX.txt",
    "a4-32hetARX.txt",
    "a4-40hetARX.txt",
    "a4-48hetARX.txt",
];
