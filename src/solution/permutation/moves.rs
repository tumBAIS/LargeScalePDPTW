use enum_map::Enum;

use crate::problem::Num;
use crate::utils::Tolerance;

#[derive(Copy, Clone, Debug, Enum)]
pub enum MoveType {
    InterRelocate,
    InterExchange,
}

#[derive(Clone, Debug)]
pub enum Move {
    InterRelocate(InterRelocateMove),
    InterExchange(InterExchangeMove),
}

impl From<&Move> for MoveType {
    fn from(mv: &Move) -> Self {
        match mv {
            Move::InterRelocate(_) => MoveType::InterRelocate,
            Move::InterExchange(_) => MoveType::InterExchange,
        }
    }
}

#[derive(Clone, Debug)]
pub struct InterRelocateMove {
    pub vn1: usize,
    pub vn2: usize,
    pub p1: usize,
    pub p1_after_node2: usize,
    pub d1_before_node2: usize,
}

#[derive(Clone, Debug)]
pub struct InterExchangeMove {
    pub vn1: usize,
    pub vn2: usize,
    pub p1: usize,
    pub p2: usize,
}

#[derive(Debug, Clone)]
pub enum BestMove {
    Some(Move, Num),
    None,
}

impl BestMove {
    pub(crate) fn is_none(&self) -> bool {
        match self {
            Self::None => true,
            _ => false,
        }
    }
    pub(crate) fn is_some(&self) -> bool {
        !self.is_none()
    }
    pub(crate) fn unwrap(self) -> (Move, Num) {
        match self {
            Self::Some(mv, v) => (mv, v),
            Self::None => panic!("Move is None"),
        }
    }

    pub(crate) fn replace_if_better(&mut self, other: BestMove) {
        let replace = match (&self, &other) {
            (_, BestMove::None) => false,
            (BestMove::None, BestMove::Some(_, _)) => true,
            (BestMove::Some(_, ref c1), BestMove::Some(_, c2)) => {
                if c2 < &(c1 - Num::tol()) {
                    true
                } else {
                    false
                }
            }
        };
        if replace {
            *self = other
        }
    }
}
