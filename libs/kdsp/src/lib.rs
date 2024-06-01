#[macro_use]
mod utils;
pub mod node_disjoint;

pub type Vertex = u32;
// trait aliases are experimental (rust-lang/rfcs#1733)
// pub trait Weight = Zero + Bounded + NumOps + Copy + PartialOrd + Ord;

#[derive(Clone, Debug)]
pub struct Arc<W> {
    pub from: Vertex,
    pub to: Vertex,
    pub w: W,
}
