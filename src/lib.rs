mod ch;

pub use ch::{ContractionHierarchy, HotGroup, Router};

pub type NodeIdx = u32;

#[derive(Copy, Clone)]
pub struct Node {
    pub x: f32,
    pub y: f32,
}

#[derive(Copy, Clone)]
pub struct Edge {
    pub from: NodeIdx,
    pub to: NodeIdx,
    pub weight: f32,
}
