use crate::problem::pdptw::{Capacity, DistanceAndTime, Node, PDPTWInstance};
use crate::problem::Num;

#[derive(Clone)]
pub struct REFNode {
    pub id: usize,
    pub demand: Capacity,
    pub ready: Num,
    pub due: Num,
    pub servicetime: Num,
}

impl From<&Node> for REFNode {
    fn from(node: &Node) -> Self {
        Self {
            id: node.id,
            demand: node.demand.clone(),
            ready: node.ready,
            due: node.due,
            servicetime: node.servicetime,
        }
    }
}

#[derive(Clone, Default, Debug)]
pub struct REFData {
    pub current_load: Capacity,
    pub max_load: Capacity,
    pub distance: Num,

    pub time: Num,
    pub earliest_completion: Num,
    pub latest_start: Num,
    pub tw_feasible: bool,

    #[cfg(feature = "trace-refdata")]
    pub trace: Vec<usize>,
}

impl REFData {
    pub fn duration(&self) -> Num {
        self.time.max(self.earliest_completion - self.latest_start)
    }
    pub fn earliest_start(&self) -> Num {
        self.earliest_completion - self.duration()
    }
    pub fn latest_start(&self) -> Num {
        self.latest_start
    }
    pub fn earliest_completion(&self) -> Num {
        self.earliest_completion
    }
    pub fn latest_completion(&self) -> Num {
        self.latest_start + self.duration()
    }
}

impl REFData {
    pub fn with_node(node: &REFNode) -> Self {
        return Self {
            current_load: node.demand.clone(),
            max_load: node.demand.clone(),
            distance: Num::ZERO,
            time: node.servicetime,
            earliest_completion: node.ready + node.servicetime,
            latest_start: node.due,
            tw_feasible: true,
            #[cfg(feature = "trace-refdata")]
            trace: vec![node.id],
        };
    }

    pub fn reset_with_node(&mut self, node: &REFNode) {
        *self = Self::with_node(node);
    }
}

pub trait ForwardREF<'a, RHS = Self>: Default {
    type Param;
    fn extend_forward_into_target(&self, node: &RHS, into: &mut Self, param: Self::Param);
    fn extend_forward(&self, node: &RHS, param: Self::Param) -> Self {
        let mut tmp = Self::default();
        self.extend_forward_into_target(node, &mut tmp, param);
        tmp
    }
    fn extend_forward_assign(&mut self, node: &RHS, param: Self::Param) {
        *self = self.extend_forward(node, param);
    }
}

impl<'a> ForwardREF<'a, REFNode> for REFData {
    type Param = DistanceAndTime;

    fn extend_forward_into_target(&self, node: &REFNode, into: &mut Self, param: Self::Param) {
        into.max_load = self.max_load.max(self.current_load + node.demand);
        into.current_load = self.current_load + node.demand;

        let DistanceAndTime { distance, time } = param;

        into.tw_feasible = self.tw_feasible && self.earliest_completion + time <= node.due;

        into.earliest_completion =
            (self.earliest_completion + time).max(node.ready) + node.servicetime;
        into.latest_start = self.latest_start.min(node.due - self.time - time);

        into.distance = self.distance + distance;
        into.time = self.time + time + node.servicetime;

        #[cfg(feature = "trace-refdata")]
        {
            into.trace = self.trace.clone();
            into.trace.push(node.id);
        }
    }
}

pub trait BackwardREF<'a, RHS = Self>: Default {
    type Param;
    fn extend_backward_into_target(&self, node: &RHS, target: &mut Self, param: Self::Param);
    fn extend_backward(&self, node: &RHS, param: Self::Param) -> Self {
        let mut tmp = Self::default();
        self.extend_backward_into_target(node, &mut tmp, param);
        tmp
    }
    fn extend_backward_assign(&mut self, node: &RHS, param: Self::Param) {
        *self = self.extend_backward(node, param)
    }
}

impl<'a> BackwardREF<'a, REFNode> for REFData {
    type Param = DistanceAndTime;
    fn extend_backward_into_target(&self, node: &REFNode, into: &mut Self, param: Self::Param) {
        into.max_load = node.demand.max(node.demand + self.current_load);
        into.current_load = node.demand + self.current_load;

        let DistanceAndTime { distance, time } = param;

        into.tw_feasible =
            self.tw_feasible && node.ready + node.servicetime + time <= self.latest_start;
        into.earliest_completion =
            (node.ready + node.servicetime + time + self.time).max(self.earliest_completion);
        into.latest_start = node.due.min(self.latest_start - time - node.servicetime);

        into.distance = distance + self.distance;
        into.time = node.servicetime + time + self.time;

        #[cfg(feature = "trace-refdata")]
        {
            into.trace = self.trace.clone();
            into.trace.insert(0, node.id);
        }
    }
}

pub trait ConcatREF<'a, RHS = Self>: Default {
    type Param;
    fn concat_into_target(&self, b: &RHS, into: &mut Self, param: Self::Param);
    fn concat(&self, rhs: &RHS, param: Self::Param) -> Self {
        let mut tmp = Self::default();
        self.concat_into_target(rhs, &mut tmp, param);
        tmp
    }
    fn concat_assign(&mut self, rhs: &RHS, param: Self::Param) {
        *self = self.concat(rhs, param)
    }
}

impl<'a> ConcatREF<'a> for REFData {
    type Param = DistanceAndTime;

    fn concat_into_target(&self, b: &Self, into: &mut Self, param: Self::Param) {
        into.max_load = self.max_load.max(self.current_load + b.max_load);
        into.current_load = self.current_load + b.current_load;

        let DistanceAndTime { distance, time } = param;

        into.tw_feasible =
            self.tw_feasible && b.tw_feasible && self.earliest_completion + time <= b.latest_start;
        into.earliest_completion =
            (self.earliest_completion + time + b.time).max(b.earliest_completion);
        into.latest_start = self.latest_start.min(b.latest_start - time - self.time);

        into.distance = self.distance + distance + b.distance;
        into.time = self.time + time + b.time;

        #[cfg(feature = "trace-refdata")]
        {
            into.trace = self.trace.clone();
            into.trace.extend(b.trace.iter());
        }
    }
}

impl REFData {
    pub(crate) fn extend_forward_node(
        &mut self,
        last: &REFNode,
        to: &REFNode,
        instance: &PDPTWInstance,
    ) {
        self.extend_forward_assign(to, instance.distance_and_time(last.id, to.id));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test() {
        let nodes = vec![
            REFNode {
                id: 0,
                demand: Capacity::default(),
                ready: 0.0.into(),
                due: 720.0.into(),
                servicetime: 0.0.into(),
            },
            REFNode {
                id: 1,
                demand: Capacity::default(),
                ready: 0.0.into(),
                due: 720.0.into(),
                servicetime: 3.0.into(),
            },
            REFNode {
                id: 2,
                demand: Capacity::default(),
                ready: 0.0.into(),
                due: 200.0.into(),
                servicetime: 3.0.into(),
            },
            REFNode {
                id: 3,
                demand: Capacity::default(),
                ready: 100.0.into(),
                due: 720.0.into(),
                servicetime: 3.0.into(),
            },
            REFNode {
                id: 4,
                demand: Capacity::default(),
                ready: 0.0.into(),
                due: 720.0.into(),
                servicetime: 3.0.into(),
            },
            REFNode {
                id: 5,
                demand: Capacity::default(),
                ready: 0.0.into(),
                due: 720.0.into(),
                servicetime: 0.0.into(),
            },
        ];

        fn get_distance_and_time(from: &REFNode, to: &REFNode) -> DistanceAndTime {
            let distance = Num::from((from.id as i64 - to.id as i64).abs());
            let time = distance * Num::from(10);
            DistanceAndTime { distance, time }
        }

        let mut fw: REFData = REFData::with_node(&nodes[0]);
        fw.extend_forward_assign(&nodes[1], get_distance_and_time(&nodes[0], &nodes[1]));
        fw.extend_forward_assign(&nodes[2], get_distance_and_time(&nodes[1], &nodes[2]));
        fw.extend_forward_assign(&nodes[3], get_distance_and_time(&nodes[2], &nodes[3]));
        fw.extend_forward_assign(&nodes[4], get_distance_and_time(&nodes[3], &nodes[4]));
        fw.extend_forward_assign(&nodes[5], get_distance_and_time(&nodes[4], &nodes[5]));

        assert_eq!(Num::from(5.0), fw.distance);
        assert_eq!(Num::from(50.0 + 12.0), fw.time);

        assert_eq!(Num::from(126.0), fw.earliest_completion);
        assert_eq!(Num::from(177.0), fw.latest_start);

        let mut bw: REFData = REFData::with_node(&nodes[5]);
        bw.extend_backward_assign(&nodes[4], get_distance_and_time(&nodes[4], &nodes[5]));
        bw.extend_backward_assign(&nodes[3], get_distance_and_time(&nodes[3], &nodes[4]));
        bw.extend_backward_assign(&nodes[2], get_distance_and_time(&nodes[2], &nodes[3]));
        bw.extend_backward_assign(&nodes[1], get_distance_and_time(&nodes[1], &nodes[2]));
        bw.extend_backward_assign(&nodes[0], get_distance_and_time(&nodes[0], &nodes[1]));

        assert_eq!(bw.time, fw.time);
        assert_eq!(bw.earliest_completion, fw.earliest_completion);
        assert_eq!(bw.latest_start, fw.latest_start);
        assert_eq!(bw.tw_feasible, fw.tw_feasible);
        assert_eq!(bw.distance, fw.distance);

        let ref_data = fw;
        for i in 1..nodes.len() - 1 {
            let mut fw: REFData = REFData::with_node(&nodes[0]);
            for j in 1..i {
                fw.extend_forward_assign(
                    &nodes[j],
                    get_distance_and_time(&nodes[j - 1], &nodes[j]),
                );
            }
            let mut bw: REFData = REFData::with_node(&nodes[nodes.len() - 1]);
            for j in (i..nodes.len() - 1).rev() {
                bw.extend_backward_assign(
                    &nodes[j],
                    get_distance_and_time(&nodes[j], &nodes[j + 1]),
                );
            }
            let mut concat = REFData::default();
            fw.concat_into_target(
                &bw,
                &mut concat,
                get_distance_and_time(&nodes[i - 1], &nodes[i]),
            );

            assert_eq!(concat.time, ref_data.time);
            assert_eq!(concat.earliest_completion, ref_data.earliest_completion);
            assert_eq!(concat.latest_start, ref_data.latest_start);
            assert_eq!(concat.tw_feasible, ref_data.tw_feasible);
            assert_eq!(concat.distance, ref_data.distance);
        }
    }
}
