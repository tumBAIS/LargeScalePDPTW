use crate::problem::pdptw::PDPTWInstance;
use crate::refn::{ConcatREF, REFData};
use fixedbitset::FixedBitSet;
use std::ops::{Index, IndexMut};

#[derive(Default, Clone, Debug)]
pub struct BlockNode {
    pub first_node_id: usize,
    pub last_node_id: usize,
    pub data: REFData,
    #[cfg(feature = "trace-refdata")]
    pub trace: Vec<usize>,
}

impl BlockNode {
    pub fn new(first_node_id: usize, last_node_id: usize, data: REFData) -> Self {
        Self {
            #[cfg(feature = "trace-refdata")]
            trace: data.trace.clone(),

            first_node_id,
            last_node_id,
            data,
        }
    }
}

impl<'a> ConcatREF<'a> for BlockNode {
    type Param = &'a PDPTWInstance;
    fn concat_into_target(&self, b: &Self, into: &mut Self, instance: Self::Param) {
        into.first_node_id = self.first_node_id;
        into.last_node_id = b.last_node_id;
        self.data.concat_into_target(
            &b.data,
            &mut into.data,
            instance.distance_and_time(self.last_node_id, b.first_node_id),
        )
    }
}

pub struct BlockNodes {
    is_block_start: FixedBitSet,
    data: Vec<BlockNode>,
}

impl BlockNodes {
    #[inline(always)]
    pub fn is_block_start(&self, node_id: usize) -> bool {
        self.is_block_start.contains(node_id)
    }

    pub fn with_instance(instance: &PDPTWInstance) -> Self {
        let mut data = vec![BlockNode::default(); instance.nodes.len()];
        for i in 0..instance.nodes.len() {
            data[i].first_node_id = i;
            data[i].last_node_id = i;
        }
        Self {
            is_block_start: FixedBitSet::with_capacity(instance.nodes.len()),
            data,
        }
    }

    #[inline(always)]
    pub fn set_block_valid(&mut self, node_id: usize) {
        self.is_block_start.insert(node_id)
    }

    #[inline(always)]
    pub fn invalidate_block(&mut self, node_id: usize) {
        self.is_block_start.set(node_id, false)
    }

    pub(crate) fn invalidate_all(&mut self) {
        self.is_block_start.clear();
    }

    #[inline(always)]
    fn get_pair_internal(&mut self, from: usize, to: usize) -> (&BlockNode, &mut BlockNode) {
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

    pub fn iter_blocks(&self) -> impl Iterator<Item=&BlockNode> {
        self.is_block_start.ones().map(|it| &self.data[it])
    }

    pub fn get_block(&self, node: usize) -> &BlockNode {
        &self.data[node]
    }
}

impl Index<usize> for BlockNodes {
    type Output = BlockNode;
    fn index(&self, index: usize) -> &Self::Output {
        &self.data[index]
    }
}

impl IndexMut<usize> for BlockNodes {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.data[index]
    }
}

