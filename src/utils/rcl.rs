use crate::problem::Num;

pub struct RCL<T, const SIZE: usize> {
    list: Vec<(Num, T)>,
}

impl<T, const SIZE: usize> RCL<T, SIZE> {
    pub fn new() -> Self {
        Self {
            list: Vec::with_capacity(SIZE),
        }
    }
    pub fn is_empty(&self) -> bool {
        self.list.is_empty()
    }
    pub fn clear(&mut self) {
        self.list.clear();
    }
    pub fn len(&self) -> usize {
        self.list.len()
    }
    pub fn iter(&self) -> impl Iterator<Item=&(Num, T)> {
        self.list.iter()
    }
    pub fn push(&mut self, element: (Num, T)) {
        let mut i = 0;
        while i < self.list.len() {
            if element.0 < self.list[i].0 {
                break;
            } else {
                i += 1;
            }
        }
        if i == SIZE {
            return;
        } else {
            if self.list.len() < SIZE {
                self.list.insert(i, element);
            } else if i == SIZE - 1 {
                self.list[SIZE - 1] = element;
            } else {
                self.list.remove(SIZE - 1);
                self.list.insert(i, element);
            }
        }
    }
    pub fn pop_first_and_clear(&mut self) -> T {
        let (_num, item) = self.list.swap_remove(0);
        self.list.clear();
        item
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn newly_created_rcl_is_empty() {
        let rcl: RCL<usize, 5> = RCL::new();
        assert!(rcl.is_empty());
    }

    #[test]
    fn cleared_rcl_is_empty() {
        let mut rcl: RCL<usize, 5> = RCL::new();
        rcl.clear();
        assert!(rcl.is_empty());
    }

    #[test]
    fn iter_rcl_is_empty_after_cleared() {
        let mut rcl: RCL<usize, 5> = RCL::new();
        rcl.clear();
        let mut iter = rcl.iter();
        assert!(iter.next().is_none());
    }

    #[test]
    fn len_is_zero_after_cleared() {
        let mut rcl: RCL<usize, 5> = RCL::new();
        rcl.clear();
        assert_eq!(0, rcl.len());
    }

    #[test]
    fn after_insert_no_longer_empty() {
        let mut rcl: RCL<usize, 5> = RCL::new();
        rcl.push((Num::ZERO, 1));
        assert!(!rcl.is_empty());
    }

    #[test]
    fn after_insert_len_is_one() {
        let mut rcl: RCL<usize, 5> = RCL::new();
        rcl.push((Num::ZERO, 1));
        assert_eq!(1, rcl.len());
    }

    #[test]
    fn iter_rcl_returns_each_element_sorted_by_num() {
        let insertions: [(Num, usize); 3] =
            [(Num::from(10), 10), (Num::from(1), 1), (Num::from(5), 5)];
        let mut rcl: RCL<usize, 5> = RCL::new();
        for it in insertions {
            rcl.push(it);
        }

        let mut iter = rcl.iter();

        assert_eq!(iter.next().unwrap().1, 1);
        assert_eq!(iter.next().unwrap().1, 5);
        assert_eq!(iter.next().unwrap().1, 10);
    }

    #[test]
    fn pushing_entries_do_not_exceed_rcl_size() {
        let insertions: [(Num, usize); 5] = [
            (Num::from(10), 10),
            (Num::from(1), 1),
            (Num::from(5), 5),
            (Num::from(7), 7),
            (Num::from(2), 2),
        ];
        let mut rcl: RCL<usize, 1> = RCL::new();
        for it in insertions {
            rcl.push(it);
        }
        assert_eq!(1, rcl.iter().count());

        let mut rcl: RCL<usize, 2> = RCL::new();
        for it in insertions {
            rcl.push(it);
        }
        assert_eq!(2, rcl.iter().count());

        let mut rcl: RCL<usize, 3> = RCL::new();
        for it in insertions {
            rcl.push(it);
        }
        assert_eq!(3, rcl.iter().count());

        let mut rcl: RCL<usize, 4> = RCL::new();
        for it in insertions {
            rcl.push(it);
        }
        assert_eq!(4, rcl.iter().count());

        let mut rcl: RCL<usize, 5> = RCL::new();
        for it in insertions {
            rcl.push(it);
        }
        assert_eq!(5, rcl.iter().count());
    }
}
