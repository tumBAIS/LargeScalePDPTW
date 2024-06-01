pub mod pdptw;
pub mod travel_matrix;

#[cfg(not(feature = "classic-pdptw"))]
pub type Num = crate::utils::num::NumI64P2;

#[cfg(feature = "classic-pdptw")]
pub type Num = crate::utils::num::NumI32P3;

pub type RequestId = usize;

pub struct Arc {
    pub from: usize,
    pub to: usize,
    pub distance: Num,
    pub time: Num,
}
