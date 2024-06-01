use std::convert::Infallible;

use fp_decimal_type::*;
use num_traits::Zero;

define_fpd_type!(name: NumI64P2, type: i64, precision: 2);
define_fpd_type!(name: NumI32P1, type: i32, precision: 1);
define_fpd_type!(name: NumI32P2, type: i32, precision: 2);
define_fpd_type!(name: NumI32P3, type: i32, precision: 3);
define_fpd_type!(name: NumI32P4, type: i32, precision: 4);
define_fpd_type!(name: NumI32P5, type: i32, precision: 5);
define_unsigned_fpd_type!(name: NumU16P2, type: u16, precision: 2);
define_unsigned_fpd_type!(name: NumU16P1, type: u16, precision: 1);
define_unsigned_fpd_type!(name: NumU16P0, type: u16, precision: 0);

impl From<NumU16P0> for NumI32P1 {
    fn from(other: NumU16P0) -> Self {
        Self {
            v: other.v as i32 * 10,
        }
    }
}

impl From<NumU16P1> for NumI32P1 {
    fn from(other: NumU16P1) -> Self {
        Self { v: other.v as i32 }
    }
}

impl From<NumU16P2> for NumI32P1 {
    fn from(other: NumU16P2) -> Self {
        Self {
            v: other.v as i32 / 10,
        }
    }
}

impl TryFrom<NumI32P1> for NumU16P0 {
    type Error = std::num::TryFromIntError;
    fn try_from(other: NumI32P1) -> Result<Self, Self::Error> {
        let v = (other.v / 10).try_into();
        v.map(|it| Self { v: it })
    }
}

impl TryFrom<NumI32P1> for NumU16P1 {
    type Error = std::num::TryFromIntError;
    fn try_from(other: NumI32P1) -> Result<Self, Self::Error> {
        let v = other.v.try_into();
        v.map(|it| Self { v: it })
    }
}

impl TryFrom<NumI32P1> for NumU16P2 {
    type Error = std::num::TryFromIntError;
    fn try_from(other: NumI32P1) -> Result<Self, Self::Error> {
        let v = (other.v * 10).try_into();
        v.map(|it| Self { v: it })
    }
}

impl From<NumU16P0> for NumI32P2 {
    fn from(other: NumU16P0) -> Self {
        Self {
            v: other.v as i32 * 100,
        }
    }
}

impl From<NumU16P1> for NumI32P2 {
    fn from(other: NumU16P1) -> Self {
        Self {
            v: other.v as i32 * 10,
        }
    }
}

impl From<NumU16P2> for NumI32P2 {
    fn from(other: NumU16P2) -> Self {
        Self { v: other.v as i32 }
    }
}

impl TryFrom<NumI32P2> for NumU16P0 {
    type Error = std::num::TryFromIntError;
    fn try_from(other: NumI32P2) -> Result<Self, Self::Error> {
        let v = (other.v / 100).try_into();
        v.map(|it| Self { v: it })
    }
}

impl TryFrom<NumI32P2> for NumU16P1 {
    type Error = std::num::TryFromIntError;
    fn try_from(other: NumI32P2) -> Result<Self, Self::Error> {
        let v = (other.v / 10).try_into();
        v.map(|it| Self { v: it })
    }
}

impl TryFrom<NumI32P2> for NumU16P2 {
    type Error = std::num::TryFromIntError;
    fn try_from(other: NumI32P2) -> Result<Self, Self::Error> {
        let v = other.v.try_into();
        v.map(|it| Self { v: it })
    }
}

impl From<NumU16P0> for NumI64P2 {
    fn from(other: NumU16P0) -> Self {
        Self {
            v: other.v as i64 * 100,
        }
    }
}

impl From<NumU16P1> for NumI64P2 {
    fn from(other: NumU16P1) -> Self {
        Self {
            v: other.v as i64 * 10,
        }
    }
}

impl From<NumU16P2> for NumI64P2 {
    fn from(other: NumU16P2) -> Self {
        Self { v: other.v as i64 }
    }
}

impl TryFrom<NumI64P2> for NumU16P0 {
    type Error = std::num::TryFromIntError;
    fn try_from(other: NumI64P2) -> Result<Self, Self::Error> {
        let v = (other.v / 100).try_into();
        v.map(|it| Self { v: it })
    }
}

impl TryFrom<NumI64P2> for NumU16P1 {
    type Error = std::num::TryFromIntError;
    fn try_from(other: NumI64P2) -> Result<Self, Self::Error> {
        let v = (other.v / 10).try_into();
        v.map(|it| Self { v: it })
    }
}

impl TryFrom<NumI64P2> for NumU16P2 {
    type Error = std::num::TryFromIntError;
    fn try_from(other: NumI64P2) -> Result<Self, Self::Error> {
        let v = other.v.try_into();
        v.map(|it| Self { v: it })
    }
}

impl From<NumI32P3> for NumI32P5 {
    fn from(other: NumI32P3) -> Self {
        Self {
            v: other.v as i32 * 100,
        }
    }
}

impl TryFrom<NumI32P5> for NumI32P3 {
    type Error = Infallible;
    fn try_from(other: NumI32P5) -> Result<Self, Self::Error> {
        let v = (other.v / 100).try_into();
        v.map(|it| Self { v: it })
    }
}
