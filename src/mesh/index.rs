use std::ops::{Deref, DerefMut};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MeshNodeIndex(pub(super) usize);

impl std::fmt::Display for MeshNodeIndex {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Node index: {}", self.0)
    }
}

impl Deref for MeshNodeIndex {
    type Target = usize;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for MeshNodeIndex {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MeshEdgeIndex(pub(super) usize);

impl std::fmt::Display for MeshEdgeIndex {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Edge index: {}", self.0)
    }
}
impl Deref for MeshEdgeIndex {
    type Target = usize;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for MeshEdgeIndex {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}



#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MeshTriangleIndex(pub(super) usize);

impl std::fmt::Display for MeshTriangleIndex {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Triangle index: {}", self.0)
    }
}
impl Deref for MeshTriangleIndex {
    type Target = usize;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for MeshTriangleIndex {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
