use std::io;

use crate::{ch::Edge, NodeIdx};

use super::HashMap;

#[derive(Copy, Clone)]
pub struct ShortcutVia {
    inner: Option<NodeIdx>,
}

impl ShortcutVia {
    pub fn node(idx: NodeIdx) -> Self {
        Self { inner: Some(idx) }
    }

    pub fn none() -> Self {
        Self { inner: None }
    }

    pub fn read(buf: &mut [u8; 4], reader: &mut impl io::Read) -> io::Result<Self> {
        reader.read_exact(buf)?;
        let value = u32::from_le_bytes(*buf);

        Ok(Self {
            inner: match value {
                0 => None,
                i => Some(i - 1),
            },
        })
    }

    pub fn write(&self, writer: &mut impl io::Write) -> io::Result<()> {
        writer.write_all(&(self.inner.map(|idx| idx + 1).unwrap_or(0).to_le_bytes()))
    }
}

pub struct Predecessor {
    node_idx: NodeIdx,
}

impl Predecessor {
    pub fn new(node_idx: NodeIdx) -> Self {
        Self { node_idx }
    }
}

pub struct AllEdges {
    forward: Vec<Vec<Edge>>,
    backward: Vec<Vec<Edge>>,
}

impl AllEdges {
    pub fn new(forward: &[Vec<Edge>], backward: &[Vec<Edge>]) -> Self {
        Self {
            forward: forward.to_vec(),
            backward: backward.to_vec(),
        }
    }

    pub fn read(buf: &mut [u8; 4], reader: &mut impl io::Read) -> io::Result<Self> {
        reader.read_exact(buf)?;

        let nodes_len = u32::from_le_bytes(*buf);

        let mut forward = Vec::with_capacity(nodes_len as usize);
        let mut backward = vec![vec![]; nodes_len as usize];

        for from in 0..nodes_len {
            reader.read_exact(buf)?;
            let edges_len = u32::from_le_bytes(*buf);

            let mut forward_edges = Vec::with_capacity(edges_len as usize);

            for _ in 0..edges_len {
                reader.read_exact(buf)?;
                let to = u32::from_le_bytes(*buf);

                reader.read_exact(buf)?;
                let weight = f32::from_le_bytes(*buf);

                let shortcut_via = ShortcutVia::read(buf, reader)?;

                forward_edges.push(Edge {
                    to,
                    weight,
                    shortcut_via,
                });

                backward[to as usize].push(Edge {
                    to: from,
                    weight,
                    shortcut_via,
                });
            }

            forward.push(forward_edges);
        }

        Ok(Self { forward, backward })
    }

    pub fn write(&self, writer: &mut impl io::Write) -> io::Result<()> {
        writer.write_all(&(self.forward.len() as u32).to_le_bytes())?;

        for forward_edges in &self.forward {
            writer.write_all(&(forward_edges.len() as u32).to_le_bytes())?;

            for edge in forward_edges {
                writer.write_all(&edge.to.to_le_bytes())?;
                writer.write_all(&edge.weight.to_le_bytes())?;
                edge.shortcut_via.write(writer)?;
            }
        }

        Ok(())
    }
}

pub struct Predecessors {
    forward: HashMap<NodeIdx, NodeIdx>,
    backward: HashMap<NodeIdx, NodeIdx>,
}

impl Predecessors {
    pub fn new() -> Self {
        Self {
            forward: HashMap::default(),
            backward: HashMap::default(),
        }
    }

    pub fn insert_forward(&mut self, from: NodeIdx, to: Predecessor) {
        self.forward.insert(from, to.node_idx);
    }

    pub fn insert_backward(&mut self, from: NodeIdx, to: Predecessor) {
        self.backward.insert(from, to.node_idx);
    }

    pub fn unfold(
        &mut self,
        edges: &AllEdges,
        from: NodeIdx,
        to: NodeIdx,
        meeting_node: NodeIdx,
    ) -> Vec<NodeIdx> {
        let mut segments = vec![meeting_node];

        let mut node = meeting_node;
        while node != from {
            let pred = self.forward.get(&node).copied().unwrap();
            push_segment(&mut segments, &edges.forward, node, pred);
            node = pred;
        }

        segments.reverse();

        let mut node = meeting_node;
        while node != to {
            let pred = self.backward.get(&node).copied().unwrap();
            push_segment(&mut segments, &edges.backward, node, pred);
            node = pred;
        }

        self.forward.clear();
        self.backward.clear();

        segments
    }
}

fn push_segment(segments: &mut Vec<NodeIdx>, edges: &[Vec<Edge>], node: NodeIdx, pred: NodeIdx) {
    let via = edges[pred as usize]
        .iter()
        .find(|edge| edge.to == node)
        .unwrap()
        .shortcut_via;

    if let Some(via) = via.inner {
        push_segment(segments, edges, node, via);
        push_segment(segments, edges, via, pred);
    } else {
        segments.push(pred);
    }
}
