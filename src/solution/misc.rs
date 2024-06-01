use std::ops::{Index, IndexMut};

use crate::problem::pdptw::PDPTWInstance;
use crate::refn::{BackwardREF, ForwardREF, REFData, REFNode};

pub struct REFListNode {
    pub node: REFNode,
    pub succ: usize,
    pub pred: usize,
    pub vn_id: usize,
    pub data: REFData,
}

impl REFListNode {
    pub fn with_node(node: REFNode) -> Self {
        Self {
            data: REFData::with_node(&node),
            succ: node.id,
            pred: node.id,
            vn_id: usize::MAX,
            node,
        }
    }

    pub fn relink(&mut self, vn_id: usize, pred: usize, succ: usize) {
        self.vn_id = vn_id;
        self.pred = pred;
        self.succ = succ;
    }
}

pub struct REFNodeVec {
    data: Vec<REFListNode>,
}

impl REFNodeVec {
    pub fn with_instance(instance: &PDPTWInstance) -> Self {
        let mut data = Vec::with_capacity(instance.nodes.len());
        for node in instance.nodes.iter() {
            data.push(REFListNode::with_node(node.into()));
        }
        // ensure that vehicle nodes point to the corresponding end node initially
        for i in 0..instance.num_vehicles {
            data[i * 2].vn_id = i * 2;
            data[i * 2].succ = i * 2 + 1;
            data[i * 2 + 1].vn_id = i * 2;
            data[i * 2 + 1].pred = i * 2;
        }
        Self { data }
    }

    pub fn len(&self) -> usize {
        self.data.len()
    }

    pub fn reset(&mut self, instance: &PDPTWInstance) {
        // ensure that vehicle nodes point to the corresponding end node initially
        for i in 0..instance.num_vehicles {
            self.data[i * 2].vn_id = i * 2;
            self.data[i * 2].succ = i * 2 + 1;
            self.data[i * 2 + 1].vn_id = i * 2;
            self.data[i * 2 + 1].pred = i * 2;
        }
        for i in instance.num_vehicles..self.data.len() {
            self.data[i].vn_id = i;
            self.data[i].succ = i;
        }
    }

    pub fn relink(&mut self, vn_id: usize, node_id: usize, pred: usize, succ: usize) {
        self.data[node_id].vn_id = vn_id;
        self.data[node_id].pred = pred;
        self.data[node_id].succ = succ;
        self.data[pred].succ = node_id;
        self.data[succ].pred = node_id;
    }

    #[inline(always)]
    fn get_pair_internal(&mut self, from: usize, to: usize) -> (&REFListNode, &mut REFListNode) {
        debug_assert!(
            from < self.data.len(),
            "from index larger or equal to vec length"
        );
        debug_assert!(
            to < self.data.len(),
            "to index larger or equal to vec length"
        );
        debug_assert_ne!(from, to);
        unsafe {
            (
                &*self.data.as_ptr().add(from),
                &mut *self.data.as_mut_ptr().add(to),
            )
        }
    }

    pub fn extend_forward_unchecked(&mut self, from: usize, to: usize, instance: &PDPTWInstance) {
        let (a, b) = self.get_pair_internal(from, to);
        REFData::extend_forward_into_target(
            &a.data,
            &b.node,
            &mut b.data,
            instance.distance_and_time(from, to),
        );
    }
    pub fn extend_backward_unchecked(&mut self, from: usize, to: usize, instance: &PDPTWInstance) {
        let (a, b) = self.get_pair_internal(from, to);

        REFData::extend_backward_into_target(
            &a.data,
            &b.node,
            &mut b.data,
            instance.distance_and_time(to, from),
        );
    }
}

impl Index<usize> for REFNodeVec {
    type Output = REFListNode;
    fn index(&self, index: usize) -> &Self::Output {
        &self.data[index]
    }
}

impl IndexMut<usize> for REFNodeVec {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.data[index]
    }
}

impl REFNodeVec {
    pub(crate) fn iter<'a>(&'a self) -> impl Iterator<Item=&'a REFListNode> {
        self.data.iter()
    }
}
