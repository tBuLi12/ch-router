use std::io;

use crate::NodeIdx;

use super::Edge;

#[derive(Copy, Clone)]
pub struct ShortcutVia {}

impl ShortcutVia {
    pub fn node(_: NodeIdx) -> Self {
        Self {}
    }

    pub fn none() -> Self {
        Self {}
    }

    pub fn read(_: &mut [u8; 4], _: &mut impl io::Read) -> io::Result<Self> {
        Ok(Self {})
    }

    pub fn write(&self, _: &mut impl io::Write) -> io::Result<()> {
        Ok(())
    }
}

pub struct Predecessor {}

impl Predecessor {
    pub fn new(_: NodeIdx) -> Self {
        Self {}
    }
}

pub struct AllEdges {}

impl AllEdges {
    pub fn new(_: &[Vec<Edge>], _: &[Vec<Edge>]) -> Self {
        Self {}
    }

    pub fn read(_: &mut [u8; 4], _: &mut impl io::Read) -> io::Result<Self> {
        Ok(Self {})
    }

    pub fn write(&self, _: &mut impl io::Write) -> io::Result<()> {
        Ok(())
    }
}

pub struct Predecessors {}

impl Predecessors {
    pub fn new() -> Self {
        Self {}
    }

    pub fn insert_forward(&mut self, _: NodeIdx, _: Predecessor) {}

    pub fn insert_backward(&mut self, _: NodeIdx, _: Predecessor) {}
}
