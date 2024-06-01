#[cfg(test)]
use std::fmt::Debug;
#[cfg(feature = "progressbar")]
use std::io::Stdout;
use std::iter::FromIterator;
use std::mem::transmute;
use std::ops::{Index, IndexMut, Range};
use std::time::Duration;

use rand::{RngCore, SeedableRng};
use rand_pcg::Pcg64Mcg;
use took::Timer;

use crate::problem::Num;
use crate::solution::SolutionDescription;

pub mod num;
pub mod rcl;
pub mod refn;

pub mod stats;
pub mod validator;

pub trait Tolerance {
    fn tol() -> Self;
}

impl Tolerance for f64 {
    fn tol() -> Self {
        0.001
    }
}

impl Tolerance for Num {
    fn tol() -> Self {
        Num::from(0.001)
    }
}

pub type Random = Pcg64Mcg;

pub fn create_seeded_rng(seed: i128) -> Random {
    let raw_bytes: [u8; 16] = unsafe { transmute(seed) };
    let mut rng = Pcg64Mcg::from_seed(raw_bytes);
    // discard the first three
    rng.next_u64();
    rng.next_u64();
    rng.next_u64();
    rng
}

pub struct NumIndexVec<T> {
    data: Vec<T>,
}

impl<T: Clone> NumIndexVec<T> {
    pub fn with_default(size: usize, default: T) -> Self {
        Self {
            data: vec![default; size],
        }
    }
}

impl<T> NumIndexVec<T> {
    pub fn from_vec(data: Vec<T>) -> Self {
        Self { data }
    }

    #[inline(always)]
    pub fn iter(&self) -> impl Iterator<Item=&T> {
        self.data.iter()
    }

    #[inline(always)]
    pub fn iter_mut(&mut self) -> impl Iterator<Item=&mut T> {
        self.data.iter_mut()
    }

    #[inline(always)]
    pub fn len(&self) -> usize {
        self.data.len()
    }
}

macro_rules! impl_index_t {
    ($t:ty) => {
        impl<T> Index<$t> for NumIndexVec<T> {
            type Output = T;

            #[inline(always)]
            fn index(&self, index: $t) -> &Self::Output {
                debug_assert!((index as usize) < self.data.len());
                unsafe { self.data.get_unchecked(index as usize) }
            }
        }

        impl<T> IndexMut<$t> for NumIndexVec<T> {
            #[inline(always)]
            fn index_mut(&mut self, index: $t) -> &mut Self::Output {
                debug_assert!((index as usize) < self.data.len());
                unsafe { self.data.get_unchecked_mut(index as usize) }
            }
        }

        impl<T> Index<Range<$t>> for NumIndexVec<T> {
            type Output = [T];

            #[inline(always)]
            fn index(&self, index: Range<$t>) -> &Self::Output {
                self.data.index(index.start as usize..index.end as usize)
            }
        }
    };
}

impl<T> FromIterator<T> for NumIndexVec<T> {
    fn from_iter<E: IntoIterator<Item=T>>(iter: E) -> Self {
        Self {
            data: Vec::from_iter(iter),
        }
    }
}

impl_index_t!(usize);
impl_index_t!(u64);
impl_index_t!(u32);
impl_index_t!(u16);
impl_index_t!(u8);

macro_rules! fix_sized_vec {
    [$default:expr; $size:expr] => {
        NumIndexVec::with_default($size as usize, $default)
    };
}

pub mod logging;
pub mod relatedness;
pub mod suurballe;

pub enum TimeLimit {
    Seconds(u64),
    None,
}

impl TimeLimit {
    pub fn as_seconds(&self) -> u64 {
        match self {
            Self::Seconds(seconds) => *seconds,
            Self::None => u64::MAX,
        }
    }
    pub fn is_none(&self) -> bool {
        match self {
            Self::None => true,
            _ => false,
        }
    }
}

pub struct Countdown {
    start: Timer,
    time_limit: TimeLimit,
}

impl Countdown {
    pub fn new(start: Timer, limit: TimeLimit) -> Self {
        Self {
            start,
            time_limit: limit,
        }
    }

    pub fn empty() -> Self {
        Self {
            start: Timer::new(),
            time_limit: TimeLimit::None,
        }
    }

    pub fn time_remaining(&self) -> u64 {
        if let TimeLimit::Seconds(value) = self.time_limit {
            let duration = self.start.took().as_std().as_secs();
            if duration > value {
                0
            } else {
                value - duration
            }
        } else {
            u64::MAX
        }
    }

    pub fn is_finished(&self) -> bool {
        self.time_remaining() == 0
    }

    pub fn is_time_remaining(&self) -> bool {
        self.time_remaining() != 0
    }

    pub fn time_elapsed(&self) -> Duration {
        self.start.took().clone().into_std()
    }
}

pub trait SearchProgressIterationTracker {
    fn update(&mut self, best: Option<&SolutionDescription>, current: Option<&SolutionDescription>);
    fn inc(&mut self);
    fn add(&mut self, i: u64);
}

pub struct DisabledSearchTracker {}

impl DisabledSearchTracker {
    pub fn new(_total: u64) -> Self {
        Self {}
    }
}

impl SearchProgressIterationTracker for DisabledSearchTracker {
    fn update(&mut self, _: Option<&SolutionDescription>, _: Option<&SolutionDescription>) {}
    fn inc(&mut self) {}
    fn add(&mut self, _: u64) {}
}

#[cfg(feature = "progressbar")]
pub struct PBRSearchTracker {
    progressbar: pbr::ProgressBar<Stdout>,
}

#[cfg(feature = "progressbar")]
impl PBRSearchTracker {
    pub fn new(total: u64) -> Self {
        Self {
            progressbar: pbr::ProgressBar::new(total),
        }
    }
}

#[cfg(feature = "progressbar")]
impl Drop for PBRSearchTracker {
    fn drop(&mut self) {
        self.progressbar.finish_println("");
    }
}

#[cfg(feature = "progressbar")]
impl SearchProgressIterationTracker for PBRSearchTracker {
    fn update(
        &mut self,
        best_sol: Option<&SolutionDescription>,
        current_sol: Option<&SolutionDescription>,
    ) {
        self.progressbar.message(
            format!(
                "{} | {} | ",
                best_sol
                    .map(|it| format!(
                        "best {}/{}/{}",
                        it.number_of_unassigned_requests(),
                        it.number_of_vehicles_used(),
                        it.objective
                    ))
                    .unwrap_or("best -/-/-".to_string()),
                current_sol
                    .map(|it| format!(
                        "current {}/{}/{}",
                        it.number_of_unassigned_requests(),
                        it.number_of_vehicles_used(),
                        it.objective
                    ))
                    .unwrap_or("current -/-/-".to_string())
            )
                .as_str(),
        );
    }

    fn inc(&mut self) {
        self.progressbar.inc();
    }

    fn add(&mut self, i: u64) {
        self.progressbar.add(i);
    }
}

#[cfg(feature = "progressbar")]
pub type DefaultSearchTracker = PBRSearchTracker;

#[cfg(not(feature = "progressbar"))]
pub type DefaultSearchTracker = DisabledSearchTracker;

#[cfg(test)]
pub fn assert_vec_eq<T: PartialEq + Eq + Debug>(expect: &Vec<T>, actual: &Vec<T>) {
    assert_eq!(
        expect.len(),
        actual.len(),
        "sizes of the vecs differ (expect: {}, actual: {})",
        expect.len(),
        actual.len()
    );
    for (idx, (x, y)) in expect.iter().zip(actual.iter()).enumerate() {
        assert_eq!(
            x, y,
            "vecs differ at index {} ({:?} != {:?})\n expect: {:?}\n actual: {:?}",
            idx, x, y, &expect, &actual
        );
    }
}
