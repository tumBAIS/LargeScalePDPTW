use crate::problem::pdptw::PDPTWInstance;
use crate::solution::SolutionDescription;

struct FixedSizedVecMatrix<T> {
    data: Vec<T>,
    width: usize,
    height: usize,
}

impl<T> FixedSizedVecMatrix<T>
    where
        T: Default + Clone,
{
    fn new(width: usize, height: usize) -> Self {
        Self {
            data: vec![T::default(); height * width],
            width,
            height,
        }
    }
}

impl<T> FixedSizedVecMatrix<T> {
    fn get(&self, i: usize, j: usize) -> &T {
        &self.data[j * self.width + i]
    }
    fn get_mut(&mut self, i: usize, j: usize) -> &mut T {
        &mut self.data[j * self.width + i]
    }
    fn set(&mut self, i: usize, j: usize, value: T) {
        *self.get_mut(i, j) = value;
    }
}

pub struct LinkCounter {
    predecessor: FixedSizedVecMatrix<usize>,
    successor: FixedSizedVecMatrix<usize>,
    sum_predecessors_recorded: usize,
    sum_successors_recorded: usize,
}

impl LinkCounter {
    pub fn with_instance(instance: &PDPTWInstance) -> Self {
        let num_requests = instance.num_requests;
        Self {
            predecessor: FixedSizedVecMatrix::new(num_requests, num_requests),
            successor: FixedSizedVecMatrix::new(num_requests, num_requests),
            sum_predecessors_recorded: 0,
            sum_successors_recorded: 0,
        }
    }
    pub fn inc(&mut self, prev: usize, next: usize) {
        self.inc_predecessor(next, prev);
        self.inc_successor(prev, next);
    }
    pub fn inc_predecessor(&mut self, of: usize, pred: usize) {
        self.sum_predecessors_recorded += 1;
        *self.predecessor.get_mut(of, pred) += 1;
    }
    pub fn inc_successor(&mut self, of: usize, succ: usize) {
        self.sum_successors_recorded += 1;
        *self.successor.get_mut(of, succ) += 1;
    }
    pub fn predecessor_count(&self, of: usize, pred: usize) -> usize {
        self.predecessor.get(of, pred).clone()
    }
    pub fn successor_count(&self, of: usize, succ: usize) -> usize {
        self.successor.get(of, succ).clone()
    }
    pub(crate) fn predecessor_and_successor_count(&self, pred: usize, succ: usize) -> usize {
        self.predecessor_count(succ, pred) + self.successor_count(pred, succ)
    }
    pub fn sum_predecessors_counted(&self) -> usize {
        self.sum_predecessors_recorded
    }
    pub fn sum_successors_counted(&self) -> usize {
        self.sum_successors_recorded
    }
    pub fn sum_predecessors_and_successors_counted(&self) -> usize {
        self.sum_predecessors_counted() + self.sum_successors_counted()
    }

    pub fn get_sum_successors_encounted_in(
        &self,
        desc: &SolutionDescription,
        instance: &PDPTWInstance,
    ) -> usize {
        let mut sum = 0;
        for (node, its_succ) in desc.successors.iter().enumerate() {
            if instance.nodes[node].node_type.is_request()
                && instance.nodes[*its_succ].node_type.is_request()
            {
                let request_id = instance.request_id(node);
                let its_successor_request_id = instance.request_id(*its_succ);
                sum += self.successor_count(request_id, its_successor_request_id);
            }
        }
        sum
    }
}

pub struct SimpleCounter {
    data: FixedSizedVecMatrix<usize>,
    sum: Vec<usize>,
    cnt: usize,
}

impl SimpleCounter {
    pub fn with_instance(instance: &PDPTWInstance) -> Self {
        let num_requests = instance.num_requests;
        Self {
            data: FixedSizedVecMatrix::new(num_requests, num_requests),
            sum: vec![0usize; num_requests],
            cnt: 0usize,
        }
    }
    pub fn inc(&mut self, of: usize, with: usize) {
        self.cnt += 1;
        self.sum[of] += 1;
        *self.data.get_mut(of, with) += 1;
    }

    pub fn get_count(&self, of: usize, with: usize) -> usize {
        self.data.get(of, with).clone()
    }

    pub fn get_counted(&self, of: usize) -> usize {
        self.sum[of]
    }

    pub fn get_counted_ratio(&self, of: usize, with: usize) -> f64 {
        self.data.get(of, with).clone() as f64 / self.sum[of] as f64
    }

    pub fn total_counted(&self) -> usize {
        self.cnt
    }
}
