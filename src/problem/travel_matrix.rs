use num_traits::{Pow, Zero};

use crate::problem::Arc;

#[cfg(not(feature = "classic-pdptw"))]
pub type DistanceNumType = crate::utils::num::NumU16P0;

#[cfg(not(feature = "classic-pdptw"))]
pub type TimeNumType = crate::utils::num::NumU16P0;

#[cfg(feature = "classic-pdptw")]
pub type DistanceNumType = crate::utils::num::NumI32P3;

#[cfg(feature = "classic-pdptw")]
pub type TimeNumType = crate::utils::num::NumI32P3;

#[derive(Debug, Clone)]
pub struct ArcValues {
    pub distance: DistanceNumType,
    pub time: TimeNumType,
}

static ZERO_ARC_VALUES: ArcValues = ArcValues {
    distance: DistanceNumType::ZERO,
    time: TimeNumType::ZERO,
};

impl Default for ArcValues {
    fn default() -> Self {
        ArcValues {
            distance: DistanceNumType::max_value(),
            time: TimeNumType::max_value(),
        }
    }
}

pub trait TravelMatrix {
    fn distance(&self, from: usize, to: usize) -> DistanceNumType;
    fn time(&self, from: usize, to: usize) -> TimeNumType;
    fn arc(&self, from: usize, to: usize) -> &ArcValues;
    fn max_distance(&self) -> DistanceNumType;
    fn max_time(&self) -> TimeNumType;
}

#[derive(Debug)]
pub struct FixSizedTravelMatrix {
    n: usize,
    data: Vec<ArcValues>,
    max_distance: DistanceNumType,
    max_time: TimeNumType,
}

impl FixSizedTravelMatrix {
    pub fn with_euclidean_distances(coords: &Vec<(f64, f64)>) -> Self {
        let n = coords.len();
        let num_arcs = n * n;
        let mut data = vec![ArcValues::default(); num_arcs];

        let mut max_distance = DistanceNumType::ZERO;
        let mut max_time = TimeNumType::ZERO;
        for i in 0..n {
            for j in 0..n {
                let idx = i * n + j;
                if i == j {
                    data[idx] = ArcValues {
                        distance: DistanceNumType::zero(),
                        time: TimeNumType::zero(),
                    };
                } else {
                    let (xi, yi) = &coords[i];
                    let (xj, yj) = &coords[j];
                    let euclidean_squared: f64 = (xi - xj).pow(2) + (yi - yj).pow(2);
                    let euclidean = euclidean_squared.sqrt();
                    let distance = euclidean.into();
                    let time = euclidean.into();
                    if distance > max_distance {
                        max_distance = distance;
                    }
                    if time > max_time {
                        max_time = time;
                    }
                    data[idx] = ArcValues { distance, time };
                }
            }
        }

        Self {
            n,
            data,
            max_distance,
            max_time,
        }
    }

    pub fn relabeled_subset(&self, to_original_mapping: &Vec<usize>) -> Self {
        let n = to_original_mapping.len();
        let mut data = vec![ArcValues::default(); n * n];
        let mut max_distance = DistanceNumType::ZERO;
        let mut max_time = TimeNumType::ZERO;

        for i in 0..n {
            let from = to_original_mapping[i];
            for j in 0..n {
                let to = to_original_mapping[j];
                if from == to {
                    data[i * n + j].distance = DistanceNumType::ZERO;
                    data[i * n + j].time = TimeNumType::ZERO;
                } else {
                    data[i * n + j] = self.data[self.idx(from, to)].clone();
                    if data[i * n + j].distance > max_distance {
                        max_distance = data[i * n + j].distance;
                    }
                    if data[i * n + j].distance > max_time {
                        max_time = data[i * n + j].distance;
                    }
                }
            }
        }

        Self {
            n,
            data,
            max_distance,
            max_time,
        }
    }

    #[inline(always)]
    fn idx(&self, from: usize, to: usize) -> usize {
        debug_assert!(from < self.n);
        debug_assert!(to < self.n);
        from * self.n + to
    }
}

impl TravelMatrix for FixSizedTravelMatrix {
    #[inline(always)]
    fn distance(&self, from: usize, to: usize) -> DistanceNumType {
        self.data[self.idx(from, to)].distance
    }
    #[inline(always)]
    fn time(&self, from: usize, to: usize) -> TimeNumType {
        self.data[self.idx(from, to)].time
    }
    #[inline(always)]
    fn arc(&self, from: usize, to: usize) -> &ArcValues {
        &self.data[self.idx(from, to)]
    }
    #[inline(always)]
    fn max_distance(&self) -> DistanceNumType {
        self.max_distance
    }
    #[inline(always)]
    fn max_time(&self) -> DistanceNumType {
        self.max_time
    }
}

pub struct FixSizedTravelMatrixBuilder {
    n: usize,
    data: Vec<ArcValues>,
    max_distance: DistanceNumType,
    max_time: TimeNumType,
}

impl FixSizedTravelMatrixBuilder {
    pub fn with_num_nodes(num_nodes: usize) -> Self {
        let mut data = vec![ArcValues::default(); num_nodes * num_nodes];
        for i in 0..num_nodes {
            let idx = i * num_nodes + i;
            data[idx] = ArcValues {
                distance: DistanceNumType::zero(),
                time: TimeNumType::zero(),
            };
        }
        Self {
            n: num_nodes,
            data,
            max_distance: DistanceNumType::ZERO,
            max_time: TimeNumType::ZERO,
        }
    }
    pub fn set_arc(&mut self, arc: Arc) -> &mut Self {
        if DistanceNumType::try_from(arc.distance).is_err() {
            panic!("{} does not fit in DistanceNumType", arc.distance);
        }
        if TimeNumType::try_from(arc.time).is_err() {
            panic!("{} does not fit in TimeNumType", arc.time);
        }
        let distance = arc.distance.try_into().unwrap();
        if distance > self.max_distance {
            self.max_distance = distance;
        }
        let time = arc.time.try_into().unwrap();
        if time > self.max_time {
            self.max_time = time;
        }
        self.data[arc.from * self.n + arc.to] = ArcValues { distance, time };

        self
    }
    pub fn build(self) -> FixSizedTravelMatrix {
        FixSizedTravelMatrix {
            n: self.n,
            data: self.data,
            max_distance: self.max_distance,
            max_time: self.max_time,
        }
    }
}

#[derive(Debug)]
pub struct TravelMatrixProxy {
    pub map: Vec<usize>,
    matrix: &'static FixSizedTravelMatrix,
}

impl TravelMatrix for TravelMatrixProxy {
    #[inline(always)]
    fn distance(&self, from: usize, to: usize) -> DistanceNumType {
        self.matrix.distance(self.map[from], self.map[to])
    }
    #[inline(always)]
    fn time(&self, from: usize, to: usize) -> TimeNumType {
        self.matrix.time(self.map[from], self.map[to])
    }
    #[inline(always)]
    fn arc(&self, from: usize, to: usize) -> &ArcValues {
        self.matrix.arc(self.map[from], self.map[to])
    }

    #[inline(always)]
    fn max_distance(&self) -> DistanceNumType {
        self.matrix.max_distance()
    }
    #[inline(always)]
    fn max_time(&self) -> DistanceNumType {
        self.matrix.max_time()
    }
}

impl TravelMatrixProxy {
    pub fn new(
        map: Vec<usize>,
        matrix: &'static FixSizedTravelMatrix,
    ) -> Self {
        Self {
            map,
            matrix,
        }
    }

    pub fn relabeled_subset(
        &self,
        to_original_mapping: &Vec<usize>,
    ) -> Self {
        Self {
            map: to_original_mapping.iter().map(|it| self.map[*it]).collect(),
            matrix: self.matrix,
        }
    }
}
