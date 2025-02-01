use super::{Edge as InitEdge, Node, NodeIdx};

use rustc_hash::{FxHashMap, FxHashSet};
use std::collections::BinaryHeap;
use unfolding::{AllEdges, Predecessor, Predecessors, ShortcutVia};

mod storage;

#[cfg(not(feature = "path-unfolding"))]
mod no_unfolding;
#[cfg(not(feature = "path-unfolding"))]
use no_unfolding as unfolding;

#[cfg(feature = "path-unfolding")]
mod unfolding;

type HashMap<K, V> = FxHashMap<K, V>;
type HashSet<K> = FxHashSet<K>;

#[derive(Copy, Clone)]
struct Edge {
    to: NodeIdx,
    weight: f32,
    shortcut_via: ShortcutVia,
}

pub struct ContractionHierarchy {
    forward_edges: Vec<Vec<Edge>>,
    backward_edges: Vec<Vec<Edge>>,
    all_edges: AllEdges,
}

struct NodeImportance {
    node_idx: NodeIdx,
    importance: u32,
}

impl Eq for NodeImportance {}
impl PartialEq for NodeImportance {
    fn eq(&self, other: &Self) -> bool {
        self.importance == other.importance
    }
}

impl PartialOrd for NodeImportance {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(&other))
    }
}

impl Ord for NodeImportance {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Inverted for min-heap
        other.importance.cmp(&self.importance)
    }
}

impl ContractionHierarchy {
    pub fn new(nodes: &[Node], edges: &[InitEdge], max_speed: f32) -> Self {
        let mut forward_edges = vec![vec![]; nodes.len()];
        let mut backward_edges = vec![vec![]; nodes.len()];

        for edge in edges {
            forward_edges[edge.from as usize].push(Edge {
                to: edge.to,
                weight: edge.weight,
                shortcut_via: ShortcutVia::none(),
            });
            backward_edges[edge.to as usize].push(Edge {
                to: edge.from,
                weight: edge.weight,
                shortcut_via: ShortcutVia::none(),
            });
        }

        let mut this = Self {
            all_edges: AllEdges::new(&forward_edges, &backward_edges),
            forward_edges,
            backward_edges,
        };

        let mut contracted = vec![false; nodes.len()];
        let mut node_importance = vec![0; nodes.len()];

        let mut remaining_nodes: BinaryHeap<_> = (0..(contracted.len() as NodeIdx))
            .map(|node_idx| NodeImportance {
                importance: this
                    .get_required_shortcuts(&nodes, node_idx, &contracted, max_speed)
                    .len() as u32,
                node_idx,
            })
            .collect();

        let mut next_importance = 0;

        while let Some(node_imp) = remaining_nodes.pop() {
            let node_idx = node_imp.node_idx;
            let required_shortcuts =
                this.get_required_shortcuts(&nodes, node_idx, &contracted, max_speed);
            let importance = required_shortcuts.len() as u32;

            if importance > node_imp.importance {
                remaining_nodes.push(NodeImportance {
                    node_idx,
                    importance,
                });
                continue;
            }

            node_importance[node_idx as usize] = next_importance;
            next_importance += 1;

            for (from, to, weight) in required_shortcuts {
                this.add_shortcut(from, to, node_idx, weight);
            }

            contracted[node_idx as usize] = true;
        }

        this.prune_edges(&node_importance);

        this
    }

    pub fn distance(&self, start: NodeIdx, target: NodeIdx) -> Option<f32> {
        Router::new(self).distance(start, target)
    }

    #[cfg(feature = "path-unfolding")]
    pub fn route(&self, start: NodeIdx, target: NodeIdx) -> Option<Route> {
        Router::new(self).route(start, target)
    }

    pub fn node_count(&self) -> u32 {
        self.forward_edges.len() as u32
    }

    pub fn create_hot_group(&self, nodes: &[NodeIdx]) -> HotGroup {
        Router::new(self).create_hot_group(nodes)
    }

    fn prune_edges(&mut self, node_importance: &[u32]) {
        for (i, edges) in self.forward_edges.iter_mut().enumerate() {
            edges.retain(|edge| node_importance[i] < node_importance[edge.to as usize]);
        }

        for (i, edges) in self.backward_edges.iter_mut().enumerate() {
            edges.retain(|edge| node_importance[edge.to as usize] > node_importance[i]);
        }
    }

    fn get_required_shortcuts(
        &self,
        nodes: &[Node],
        node_idx: NodeIdx,
        contracted: &[bool],
        max_speed: f32,
    ) -> Vec<(NodeIdx, NodeIdx, f32)> {
        let incoming = &self.backward_edges[node_idx as usize];
        let outgoing = &self.forward_edges[node_idx as usize];

        let mut required_shortcuts = vec![];

        let mut pairs = HashMap::default();

        for in_edge in incoming {
            for out_edge in outgoing {
                if contracted[in_edge.to as usize] || contracted[out_edge.to as usize] {
                    continue;
                }

                let path_length = in_edge.weight + out_edge.weight;
                let current = pairs
                    .entry((in_edge.to, out_edge.to))
                    .or_insert(path_length);
                *current = current.min(path_length);
            }
        }

        for ((from, to), path_length) in pairs {
            if !self.witness_path_exists(
                &nodes,
                from,
                to,
                node_idx,
                path_length,
                max_speed,
                contracted,
            ) {
                required_shortcuts.push((from, to, path_length));
            }
        }

        required_shortcuts
    }

    fn witness_path_exists(
        &self,
        nodes: &[Node],
        from: NodeIdx,
        to: NodeIdx,
        via: NodeIdx,
        max_length: f32,
        max_speed: f32,
        contracted: &[bool],
    ) -> bool {
        struct State {
            cost: f32,
            h_cost: f32,
            idx: NodeIdx,
        }

        impl Eq for State {}
        impl PartialEq for State {
            fn eq(&self, other: &Self) -> bool {
                self.h_cost == other.h_cost
            }
        }

        impl PartialOrd for State {
            fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
                Some(self.cmp(other))
            }
        }

        impl Ord for State {
            fn cmp(&self, other: &Self) -> std::cmp::Ordering {
                other.h_cost.partial_cmp(&self.h_cost).unwrap()
            }
        }

        let target = nodes[to as usize];
        let straight_line_distance = |from: NodeIdx| {
            let start = nodes[from as usize];
            ((target.x - start.x).powi(2) + (target.y - start.y).powi(2)).sqrt() / max_speed
        };

        let mut distances = HashMap::default();
        let mut heap = BinaryHeap::new();

        distances.insert(from, 0.0);
        heap.push(State {
            cost: 0.0,
            h_cost: straight_line_distance(from),
            idx: from,
        });

        while let Some(State { cost, idx, h_cost }) = heap.pop() {
            if cost > distances[&idx] {
                continue;
            }

            if h_cost > max_length {
                return false;
            }

            if idx == to {
                return true;
            }

            for edge in &self.forward_edges[idx as usize] {
                if edge.to == via || contracted[edge.to as usize] {
                    continue;
                }

                let cost = cost + edge.weight;
                let next = State {
                    h_cost: cost + straight_line_distance(edge.to),
                    idx: edge.to,
                    cost,
                };

                let dist = distances.entry(next.idx).or_insert(f32::MAX);

                if cost < *dist {
                    *dist = cost;
                    heap.push(next);
                }
            }
        }

        false
    }

    fn add_shortcut(&mut self, from: NodeIdx, to: NodeIdx, via: NodeIdx, weight: f32) {
        let forward = &mut self.forward_edges[from as usize];
        let backward = &mut self.backward_edges[to as usize];

        forward.retain(|edge| edge.to != to);
        backward.retain(|edge| edge.to != from);

        forward.push(Edge {
            to,
            weight,
            shortcut_via: ShortcutVia::node(via),
        });
        backward.push(Edge {
            to: from,
            weight,
            shortcut_via: ShortcutVia::node(via),
        });
    }
}

struct SearchState {
    node: NodeIdx,
    distance: f32,
    pred: Predecessor,
}

impl Eq for SearchState {}
impl PartialEq for SearchState {
    fn eq(&self, other: &Self) -> bool {
        self.distance == other.distance
    }
}

impl Ord for SearchState {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other.distance.partial_cmp(&self.distance).unwrap()
    }
}

impl PartialOrd for SearchState {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

#[cfg(feature = "path-unfolding")]
pub struct Route {
    pub path: Vec<NodeIdx>,
    pub distance: f32,
}

pub struct Router<'ch> {
    ch: &'ch ContractionHierarchy,
    queue: BinaryHeap<SearchState>,
    forward_distances: HashMap<NodeIdx, f32>,
    backward_distances: HashMap<NodeIdx, f32>,
    forward_settled: HashSet<NodeIdx>,
    predecessors: Predecessors,
}

impl<'ch> Router<'ch> {
    pub fn new(ch: &'ch ContractionHierarchy) -> Self {
        Self {
            ch,
            queue: BinaryHeap::new(),
            forward_distances: HashMap::default(),
            backward_distances: HashMap::default(),
            forward_settled: HashSet::default(),
            predecessors: Predecessors::new(),
        }
    }

    pub fn distance(&mut self, start: NodeIdx, target: NodeIdx) -> Option<f32> {
        self.bidirectional_dijkstra(start, target)
            .map(|(_, distance)| distance)
    }

    #[cfg(feature = "path-unfolding")]
    pub fn route(&mut self, start: NodeIdx, target: NodeIdx) -> Option<Route> {
        let (node, distance) = self.bidirectional_dijkstra(start, target)?;

        Some(Route {
            path: self
                .predecessors
                .unfold(&self.ch.all_edges, start, target, node),
            distance,
        })
    }

    pub fn create_hot_group(&mut self, nodes: &[NodeIdx]) -> HotGroup {
        let forward_meeting_nodes: Vec<_> = nodes
            .iter()
            .map(|&node| {
                self.queue.push(SearchState {
                    distance: 0.0,
                    node,
                    pred: Predecessor::new(node),
                });

                let mut forward_settled = vec![];

                self.forward_distances.insert(node, 0.0);

                while let Some(SearchState { node, distance, .. }) = self.queue.pop() {
                    if let Some(&current) = self.forward_distances.get(&node) {
                        if distance > current {
                            continue;
                        }
                    }

                    forward_settled.push(MeetingNode { node, distance });

                    for edge in &self.ch.forward_edges[node as usize] {
                        let next = SearchState {
                            distance: distance + edge.weight,
                            node: edge.to,
                            pred: Predecessor::new(node),
                        };

                        let forward = self.forward_distances.entry(next.node).or_insert(f32::MAX);
                        if next.distance < *forward {
                            *forward = next.distance;
                            self.queue.push(next);
                        }
                    }
                }

                self.forward_distances.clear();
                forward_settled.sort_by_key(|mn| mn.node);
                forward_settled
            })
            .collect();

        let backward_meeting_nodes: Vec<_> = nodes
            .iter()
            .map(|&node| {
                self.queue.push(SearchState {
                    distance: 0.0,
                    node,
                    pred: Predecessor::new(node),
                });

                let mut backward_settled = vec![];

                self.backward_distances.insert(node, 0.0);

                while let Some(SearchState { node, distance, .. }) = self.queue.pop() {
                    if let Some(&current) = self.backward_distances.get(&node) {
                        if distance > current {
                            continue;
                        }
                    }

                    backward_settled.push(MeetingNode { node, distance });

                    for edge in &self.ch.backward_edges[node as usize] {
                        let next = SearchState {
                            distance: distance + edge.weight,
                            node: edge.to,
                            pred: Predecessor::new(node),
                        };

                        let backward = self.backward_distances.entry(next.node).or_insert(f32::MAX);

                        if next.distance < *backward {
                            *backward = next.distance;
                            self.queue.push(next);
                        }
                    }
                }

                self.backward_distances.clear();
                backward_settled.sort_by_key(|mn| mn.node);
                backward_settled
            })
            .collect();

        self.forward_distances.clear();
        self.backward_distances.clear();

        HotGroup {
            forward_meeting_nodes,
            backward_meeting_nodes,
        }
    }

    fn bidirectional_dijkstra(
        &mut self,
        start: NodeIdx,
        target: NodeIdx,
    ) -> Option<(NodeIdx, f32)> {
        let mut best_distance = f32::MAX;
        let mut best_meeting_node = None;

        self.queue.push(SearchState {
            distance: 0.0,
            node: start,
            pred: Predecessor::new(start),
        });

        self.forward_distances.insert(start, 0.0);
        self.backward_distances.insert(target, 0.0);

        while let Some(SearchState {
            node,
            distance,
            pred,
        }) = self.queue.pop()
        {
            if let Some(&current) = self.forward_distances.get(&node) {
                if distance > current {
                    continue;
                }
            }

            self.forward_settled.insert(node);
            self.predecessors.insert_forward(node, pred);

            for edge in &self.ch.forward_edges[node as usize] {
                let next = SearchState {
                    distance: distance + edge.weight,
                    node: edge.to,
                    pred: Predecessor::new(node),
                };

                let forward = self.forward_distances.entry(next.node).or_insert(f32::MAX);
                if next.distance < *forward {
                    *forward = next.distance;
                    self.queue.push(next);
                }
            }
        }

        self.queue.push(SearchState {
            distance: 0.0,
            node: target,
            pred: Predecessor::new(target),
        });

        while let Some(SearchState {
            node,
            distance,
            pred,
        }) = self.queue.pop()
        {
            if let Some(&current) = self.backward_distances.get(&node) {
                if distance > current {
                    continue;
                }
            }

            self.predecessors.insert_backward(node, pred);

            if self.forward_settled.contains(&node) {
                let forward_distance = self.forward_distances[&node];
                let total_distance = distance + forward_distance;
                if total_distance < best_distance {
                    best_distance = total_distance;
                    best_meeting_node = Some(node);
                }
            }

            for edge in &self.ch.backward_edges[node as usize] {
                let next = SearchState {
                    distance: distance + edge.weight,
                    node: edge.to,
                    pred: Predecessor::new(node),
                };

                let backward = self.backward_distances.entry(next.node).or_insert(f32::MAX);

                if next.distance < *backward {
                    *backward = next.distance;
                    self.queue.push(next);
                }
            }
        }

        self.forward_distances.clear();
        self.backward_distances.clear();

        self.forward_settled.clear();

        best_meeting_node.map(|node| (node, best_distance))
    }
}

pub struct HotGroup {
    forward_meeting_nodes: Vec<Vec<MeetingNode>>,
    backward_meeting_nodes: Vec<Vec<MeetingNode>>,
}

struct MeetingNode {
    node: NodeIdx,
    distance: f32,
}

impl HotGroup {
    pub fn distance(&self, start: NodeIdx, end: NodeIdx) -> Option<f32> {
        let forward_nodes = &self.forward_meeting_nodes[start as usize];
        let backward_nodes = &self.backward_meeting_nodes[end as usize];

        let mut i = 0;
        let mut j = 0;

        let mut min_distance = f32::MAX;
        let mut found = false;

        while i < forward_nodes.len() && j < backward_nodes.len() {
            let forward = &forward_nodes[i];
            let backward = &backward_nodes[j];

            if forward.node == backward.node {
                i += 1;
                j += 1;

                min_distance = min_distance.min(forward.distance + backward.distance);
                found = true;
            } else if forward.node < backward.node {
                i += 1;
            } else {
                j += 1;
            }
        }

        if found {
            Some(min_distance)
        } else {
            None
        }
    }

    pub fn node_count(&self) -> u32 {
        self.forward_meeting_nodes.len() as u32
    }
}

#[cfg(test)]
mod tests {
    use crate::*;

    fn create_ch() -> ContractionHierarchy {
        let nodes = [
            Node { x: 0.0, y: 0.0 },
            Node { x: 8.0, y: 0.0 },
            Node { x: 4.0, y: 1.0 },
            Node { x: 2.0, y: 4.0 },
            Node { x: 6.0, y: 4.0 },
            Node { x: 4.0, y: 7.0 },
            Node { x: 0.0, y: 8.0 },
            Node { x: 8.0, y: 8.0 },
        ];

        let edges = [
            Edge {
                from: 0,
                to: 3,
                weight: 2.0,
            },
            Edge {
                from: 1,
                to: 2,
                weight: 1.6,
            },
            Edge {
                from: 1,
                to: 4,
                weight: 2.5,
            },
            Edge {
                from: 2,
                to: 0,
                weight: 1.3,
            },
            Edge {
                from: 2,
                to: 3,
                weight: 1.7,
            },
            Edge {
                from: 2,
                to: 4,
                weight: 1.5,
            },
            Edge {
                from: 3,
                to: 0,
                weight: 2.0,
            },
            Edge {
                from: 3,
                to: 5,
                weight: 1.3,
            },
            Edge {
                from: 3,
                to: 2,
                weight: 1.7,
            },
            Edge {
                from: 4,
                to: 5,
                weight: 1.2,
            },
            Edge {
                from: 4,
                to: 1,
                weight: 2.5,
            },
            Edge {
                from: 5,
                to: 6,
                weight: 1.9,
            },
            Edge {
                from: 5,
                to: 7,
                weight: 1.2,
            },
            Edge {
                from: 5,
                to: 3,
                weight: 1.3,
            },
            Edge {
                from: 5,
                to: 4,
                weight: 1.2,
            },
            Edge {
                from: 6,
                to: 3,
                weight: 3.2,
            },
            Edge {
                from: 6,
                to: 5,
                weight: 1.9,
            },
            Edge {
                from: 7,
                to: 4,
                weight: 3.0,
            },
        ];

        let max_speed = edges
            .iter()
            .map(|e| {
                let from = nodes[e.from as usize];
                let to = nodes[e.to as usize];
                let straight_line_distance =
                    ((from.x - to.x).powi(2) + (from.y - to.y).powi(2)).sqrt();
                straight_line_distance / e.weight
            })
            .fold(f32::MIN, f32::max);

        ContractionHierarchy::new(&nodes, &edges, max_speed)
    }

    fn check_distances(ch: &ContractionHierarchy) {
        let mut router = Router::new(ch);

        assert_eq!(router.distance(0, 1), Some(7.0));
        assert_eq!(router.distance(1, 6), Some(5.6));
        assert_eq!(router.distance(7, 2), Some(7.1));
    }

    #[cfg(feature = "path-unfolding")]
    fn check_routing(ch: &ContractionHierarchy) {
        let mut router = Router::new(&ch);

        let route = router.route(0, 1).unwrap();
        assert_eq!(route.path, vec![0, 3, 5, 4, 1]);
        assert_eq!(route.distance, 7.0);
    }

    #[test]
    fn test_distance() {
        check_distances(&create_ch());
    }

    #[cfg(feature = "path-unfolding")]
    #[test]
    fn test_route() {
        check_routing(&create_ch());
    }

    #[test]
    fn test_load_distance() {
        let mut ch_data = vec![];
        let ch = create_ch();
        ch.write(&mut ch_data).unwrap();
        let ch = ContractionHierarchy::read(&mut &*ch_data).unwrap();
        check_distances(&ch);
    }

    #[cfg(feature = "path-unfolding")]
    #[test]
    fn test_load_route() {
        let mut ch_data = vec![];
        let ch = create_ch();
        ch.write(&mut ch_data).unwrap();
        let ch = ContractionHierarchy::read(&mut &*ch_data).unwrap();
        check_routing(&ch);
    }
}
