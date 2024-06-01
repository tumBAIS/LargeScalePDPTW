pub use num_traits::{One, SaturatingAdd, SaturatingMul, SaturatingSub, Zero};
pub use std::fmt::{Debug, Display, Error, Formatter};
pub use std::iter::Sum;
pub use std::num::ParseFloatError;
pub use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};
pub use std::str::FromStr;

pub const POW_10: [usize; 10] = [
    1,
    10,
    100,
    1_000,
    10_000,
    100_000,
    1_000_000,
    10_000_000,
    100_000_000,
    1_000_000_000,
];

#[macro_export]
macro_rules! add_from_impl_float {
    ($id:ident: $v:ty, $($t:ty)*) => ($(
        impl From<$t> for $id {
            fn from(o: $t) -> Self {
                Self { v: (o as f64 * Self::PREC_POW_10).round() as $v }
            }
        }
    )*)
}

#[macro_export]
macro_rules! add_from_impl_int {
    ($id:ident: $v:ty, $($t:ty)*) => ($(
        impl From<$t> for $id {
            fn from(o: $t) -> Self {
                Self { v: (o as $v * Self::PREC_POW_10 as $v)}
            }
        }
    )*)
}

#[macro_export]
macro_rules! define_unsigned_fpd_type {

    (name: $id:ident, type: $t:ty, precision: $prec:expr) => (
        #[derive(Default, Copy, Clone, PartialOrd, Ord, PartialEq, Eq)]
        pub struct $id { v: $t }

        impl $id {
            pub const PRECISION: usize = $prec;
            const PREC_POW_10: f64 = POW_10[$prec] as f64;
            pub const EPSILON: Self = Self { v: 1 };
            pub const MAX: Self = Self { v: <$t>::max_value() };
            pub const MIN: Self = Self { v: <$t>::min_value() };
            pub const ZERO: Self = Self { v: 0 as $t };
            pub const HALF: Self = Self { v: 1 * (Self::PREC_POW_10 as $t) / 2 };
            pub const ONE: Self = Self { v: 1 * (Self::PREC_POW_10 as $t) };
            pub const ONE_HALF: Self = Self { v: 3 * (Self::PREC_POW_10 as $t) / 2 };
            pub const TWO: Self = Self { v: 2 * (Self::PREC_POW_10 as $t) };

            pub const fn max_value() -> Self { Self::MAX }

            pub fn min<'a>(&'a self, other: impl Into<&'a Self>) -> &'a Self {
                let b = other.into();
                if b.v < self.v { b } else { self }
            }
            pub fn max<'a>(&'a self, other: impl Into<&'a Self>) -> &'a Self {
                let b = other.into();
                if b.v > self.v { b } else { self }
            }
        }

        add_from_impl_int!($id:$t, i8 i16 i32 i64 i128 u8 u16 u32 u64 u128 usize);
        add_from_impl_float!($id:$t, f32 f64);

        impl $id {
            pub const fn from_i64(o: i64) -> $id { Self { v: (o * POW_10[$prec] as i64) as $t } }
        }

        impl Add for $id {
            type Output = Self;
            fn add(self, rhs: Self) -> Self {
                Self { v: self.v + rhs.v }
            }
        }

        impl<'a> Add<$id> for &'a $id {
            type Output = $id;

            fn add(self, rhs: $id) -> $id {
                $id { v: self.v + rhs.v }
            }
        }

        impl<'a, 'b> Add<&'b $id> for &'a $id {
            type Output = $id;

            fn add(self, rhs: &'b $id) -> $id {
                $id { v: self.v + rhs.v }
            }
        }

        impl AddAssign for $id {
            fn add_assign(&mut self, rhs: Self) {
                self.v += rhs.v
            }
        }

        impl Sub for $id {
            type Output = Self;
            fn sub(self, rhs: Self) -> Self::Output {
                Self { v: self.v - rhs.v }
            }
        }

        impl<'a> Sub<$id> for &'a $id {
            type Output = $id;

            fn sub(self, rhs: $id) -> $id {
                $id { v: self.v - rhs.v }
            }
        }

        impl<'b> Sub<&'b $id> for $id {
            type Output = $id;

            fn sub(self, rhs: &'b $id) -> $id {
                $id { v: self.v - rhs.v }
            }
        }

        impl<'a, 'b> Sub<&'b $id> for &'a $id {
            type Output = $id;

            fn sub(self, rhs: &'b $id) -> $id {
                $id { v: self.v - rhs.v }
            }
        }

        impl SubAssign for $id {
            fn sub_assign(&mut self, rhs: Self) {
                self.v -= rhs.v
            }
        }

        impl Mul for $id {
            type Output = Self;
            fn mul(self, rhs: Self) -> Self::Output {
                Self { v: ((self.v as f64 * rhs.v as f64) / $id::PREC_POW_10) as $t }
            }
        }

        impl<'a, 'b> Mul<&'b $id> for &'a $id {
            type Output = $id;

            fn mul(self, rhs: &'b $id) -> $id {
                $id { v: ((self.v as f64 * rhs.v as f64) / $id::PREC_POW_10) as $t }
            }
        }

        impl MulAssign for $id {
            fn mul_assign(&mut self, rhs: Self) {
                self.v = ((self.v as f64 * rhs.v as f64) / $id::PREC_POW_10) as $t
            }
        }

        impl Div for $id {
            type Output = Self;
            fn div(self, rhs: Self) -> Self::Output {
                Self { v: (self.v as f64 / rhs.v as f64 * Self::PREC_POW_10) as $t }
            }
        }

        impl<'a, 'b> Div<&'b $id> for &'a $id {
            type Output = $id;

            fn div(self, rhs: &'b $id) -> $id {
                $id { v: (self.v as f64 / rhs.v as f64 * $id::PREC_POW_10) as $t }
            }
        }

        impl DivAssign for $id {
            fn div_assign(&mut self, rhs: Self) {
                self.v = (self.v as f64 / rhs.v as f64 * Self::PREC_POW_10) as $t
            }
        }

        impl Display for $id {
            fn fmt(&self, f: &mut Formatter<'_>) -> Result<(), Error> {
                write!(f, "{:.*}", Self::PRECISION, self.v as f64 / Self::PREC_POW_10)
            }
        }

        impl Debug for $id {
            fn fmt(&self, f: &mut Formatter<'_>) -> Result<(), Error> {
                write!(f, "{}", self.v as f64 / Self::PREC_POW_10)
            }
        }

        impl Sum for $id {
            fn sum<I: Iterator<Item=$id>>(iter: I) -> $id {
                $id { v: iter.fold(0, |sum, rhs| { sum + rhs.v }) }
            }
        }

        impl Zero for $id {
            fn zero() -> Self {
                Self::ZERO
            }
            fn is_zero(&self) -> bool {
                *self == Self::ZERO
            }
        }

        impl One for $id {
           fn one() -> Self {
                Self::ONE
            }
        }

        impl SaturatingAdd for $id {
            fn saturating_add(&self, rhs: &$id) -> $id {
                Self { v: self.v.saturating_add(rhs.v) }
            }
        }

        impl SaturatingSub for $id {
            fn saturating_sub(&self, &rhs: &$id) -> $id {
                Self { v: self.v.saturating_sub(rhs.v) }
            }
        }

        impl SaturatingMul for $id {
            fn saturating_mul(&self, rhs: &$id) -> $id {
                Self { v: (((self.v as f64) * (rhs.v as f64)) / $id::PREC_POW_10) as $t }
            }
        }

        impl FromStr for $id {
            type Err = ParseFloatError;
            fn from_str(s: &str) -> Result<Self, Self::Err> {
                s.parse::<f64>().map(|v| v.into())
            }
        }

        impl $id {
            pub fn value(&self) -> $t {
                self.v
            }
        }

        impl From<$id> for f64 {
            fn from(o: $id) -> Self {
                (o.v as f64 / $id::PREC_POW_10)
            }
        }
    )
}

#[macro_export]
macro_rules! define_fpd_type {
    (name: $id:ident, type: $t:ty, precision: $prec:expr) => (
        define_unsigned_fpd_type!(name: $id, type: $t, precision: $prec);

        impl $id {
            pub const NEGATIVE_EPSILON: Self = Self { v: -1 * Self::EPSILON.v };
            pub const NEGATIVE_ONE: Self = Self { v: -1 * (Self::PREC_POW_10 as $t) };
            pub fn abs(&self) -> $id {
                Self { v: self.v.abs() }
            }
        }

        impl Neg for $id {
            type Output = Self;
            fn neg(self) -> Self::Output { Self { v: -self.v } }
        }
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn precision_test() {
        define_fpd_type!(name: TestNum4, type: i64, precision: 4);

        let num: TestNum4 = 1.001.into();
        assert_eq!(num.v, 10010);

        define_fpd_type!(name: TestNum2, type: i64, precision: 2);

        let num: TestNum2 = 1.001.into();
        assert_eq!(num.v, 100);
    }

    #[test]
    fn arithmetics() {
        define_fpd_type!(name: TestNum, type: i64, precision: 2);
        let a: TestNum = 1.01.into();
        let b: TestNum = 0.95.into();
        assert_eq!((a.clone() + b.clone()).v, 101 + 95);
        assert_eq!((a.clone() - b.clone()).v, 101 - 95);
        assert_eq!(
            (a.clone() * b.clone()).v,
            ((1.01 * 0.95) * TestNum::PREC_POW_10) as i64
        );
        assert_eq!(
            (a.clone() / b.clone()).v,
            (101 as f64 / 95 as f64 * TestNum::PREC_POW_10) as i64
        );

        assert_eq!(
            {
                let mut c = a.clone();
                c += b.clone();
                c
            }
            .v,
            101 + 95
        );
        assert_eq!(
            {
                let mut c = a.clone();
                c -= b.clone();
                c
            }
            .v,
            101 - 95
        );
        assert_eq!(
            {
                let mut c = a.clone();
                c *= b.clone();
                c
            }
            .v,
            ((1.01 * 0.95) * TestNum::PREC_POW_10) as i64
        );
        assert_eq!(
            {
                let mut c = a.clone();
                c /= b.clone();
                c
            }
            .v,
            (101 as f64 / 95 as f64 * TestNum::PREC_POW_10) as i64
        );
    }
}
