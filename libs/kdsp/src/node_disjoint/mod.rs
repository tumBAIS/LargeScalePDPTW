use crate::utils::NumIndexVec;
use crate::{Arc, Vertex};
use num_traits::{Bounded, NumOps, Zero};

pub mod suurballe;

#[cfg(test)]
mod tests;

#[derive(Clone)]
pub struct SourceSinkGraph<W> {
    num_nodes: Vertex,
    source_node: Vertex,
    sink_node: Vertex,
    arcs: NumIndexVec<Arc<W>>,
    fw_row_ptr: NumIndexVec<usize>,
}

impl<W> SourceSinkGraph<W>
where
    W: Zero + Bounded + NumOps + Copy + PartialOrd + Ord,
{
    pub fn num_nodes(&self) -> Vertex {
        self.num_nodes
    }
    pub fn source(&self) -> Vertex {
        self.source_node
    }
    pub fn sink(&self) -> Vertex {
        self.sink_node
    }

    pub fn arc_exists(&self, from: Vertex, to: Vertex) -> bool {
        for i in self.fw_row_ptr[from]..self.fw_row_ptr[from + 1] {
            if self.arcs[i].to == to {
                return true;
            }
        }
        return false;
    }

    pub fn get_arc_id(&self, from: Vertex, to: Vertex) -> usize {
        for i in self.fw_row_ptr[from]..self.fw_row_ptr[from + 1] {
            if self.arcs[i].to == to {
                return i;
            }
        }
        panic!("expected arc from {} to {}, but none found", from, to);
    }

    pub fn get_arc_by_id(&self, id: usize) -> &Arc<W> {
        &self.arcs[id]
    }

    pub fn get_arc(&self, from: Vertex, to: Vertex) -> &Arc<W> {
        &self.arcs[self.get_arc_id(from, to)]
    }

    pub fn iter_outgoing_arcs(&self, from: Vertex) -> impl Iterator<Item = &Arc<W>> {
        self.arcs[self.fw_row_ptr[from]..self.fw_row_ptr[from + 1]].iter()
    }

    #[allow(unused)]
    pub fn iter_arcs(&self) -> impl Iterator<Item = &Arc<W>> {
        self.arcs.iter()
    }

    pub fn iter_arcs_mut(&mut self) -> impl Iterator<Item = &mut Arc<W>> {
        self.arcs.iter_mut()
    }
}

pub struct SourceSinkGraphBuilder<W>
where
    W: Zero + Bounded + NumOps + Copy + PartialOrd + Ord,
{
    num_nodes: Vertex,
    source_node: Option<Vertex>,
    sink_node: Option<Vertex>,
    arcs: Vec<Arc<W>>,
}

impl<W> SourceSinkGraphBuilder<W>
where
    W: Zero + Bounded + NumOps + Copy + PartialOrd + Ord,
{
    pub fn new() -> Self {
        Self {
            num_nodes: 0,
            source_node: None,
            sink_node: None,
            arcs: vec![],
        }
    }
    pub fn set_num_nodes(mut self, num: Vertex) -> Self {
        self.num_nodes = num;
        self
    }
    pub fn set_source(mut self, source: Vertex) -> Self {
        self.source_node = Some(source);
        self
    }
    pub fn set_sink(mut self, sink: Vertex) -> Self {
        self.sink_node = Some(sink);
        self
    }
}

impl<W> SourceSinkGraphBuilder<W>
where
    W: Zero + Bounded + NumOps + Copy + PartialOrd + Ord,
{
    pub fn add_arcs(mut self, arcs: impl IntoIterator<Item = Arc<W>>) -> Self {
        for arc in arcs {
            self.arcs.push(arc);
        }
        self
    }
}

impl<W> SourceSinkGraphBuilder<W>
where
    W: Zero + Bounded + NumOps + Copy + PartialOrd + Ord,
{
    pub fn build(mut self) -> SourceSinkGraph<W> {
        self.arcs
            .sort_by(|a, b| a.from.cmp(&b.from).then(a.to.cmp(&b.to)));
        let arcs = NumIndexVec::from_vec(self.arcs);

        debug_assert!(self.num_nodes < Vertex::MAX);

        let mut fw_row_ptr = num_index_vec![0; self.num_nodes + 1];
        let mut fw_row_cnt = 0;

        for (i, arc) in arcs.iter().enumerate() {
            if fw_row_cnt < arc.from + 1 {
                fw_row_ptr[arc.from] = i;
                fw_row_cnt = arc.from + 1;
            }
        }
        fw_row_ptr[self.num_nodes] = arcs.len();

        // fill uninitialized row_ptr when graph not strongly connected
        let first_row = arcs[0usize].from;
        let mut next_row_ptr = arcs.len();
        for i in (first_row + 1..self.num_nodes).rev() {
            if fw_row_ptr[i] == 0 {
                fw_row_ptr[i] = next_row_ptr;
            } else {
                next_row_ptr = fw_row_ptr[i];
            }
        }
        let mut rev_arc_ids: Vec<usize> = (0..arcs.len()).collect();
        rev_arc_ids.sort_by(|a, b| {
            arcs[*a]
                .to
                .cmp(&arcs[*b].to)
                .then(arcs[*a].from.cmp(&arcs[*b].from))
        });

        SourceSinkGraph {
            num_nodes: self.num_nodes,
            source_node: if let Some(s) = self.source_node { s } else { 0 },
            sink_node: if let Some(s) = self.sink_node {
                s
            } else {
                self.num_nodes - 1
            },
            arcs,
            fw_row_ptr,
        }
    }
}

impl<W> SourceSinkGraph<W>
where
    W: Zero + Bounded + NumOps + Copy + PartialOrd + Ord,
{
    pub fn builder() -> SourceSinkGraphBuilder<W> {
        SourceSinkGraphBuilder::new()
    }
}
