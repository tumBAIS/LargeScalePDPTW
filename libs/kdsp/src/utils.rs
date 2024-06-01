use std::iter::FromIterator;
use std::ops::{Index, IndexMut, Range};

/// Wrapper around Vec<T> to use non-usize number types as index for readability
#[derive(Clone)]
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
                &self.data[index.start as usize..index.end as usize]
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

macro_rules! num_index_vec {
    [$default:expr; $size:expr] => {
        NumIndexVec::with_default($size as usize, $default)
    };
}
