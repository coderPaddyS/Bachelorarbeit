mod list;
mod mesh;
mod builder;

use builder::MeshBuilderError;
pub use mesh::*;
pub use list::*;

// Vec of EdgeIndex -> No sorting, shuffling, preallocation of array, but bad cache effiency.
// Array start + length -> Good cache effiency, but preallocation and relocation needed.
#[derive(Clone, Debug)]
pub struct Node {
    // pub label: String,
    pub coordinates: [f32; 3],
    pub outgoing: Vec<Index<Edge>>,
}

impl Node {
    pub fn new(coordinates: [f32; 3]) -> Self {
        Self { outgoing: vec![], coordinates }
    }
}

#[derive(Clone, Debug)]
pub struct Edge {
    pub source: Index<Node>,
    pub target: Index<Node>,
    pub opposite: Index<Node>,
    pub previous: Index<Edge>,
    pub next: Index<Edge>,
    pub triangle: Index<Triangle>,
}

#[derive(Clone, Debug)]
pub struct Triangle {
    pub corners: [Index<Node>; 3],
}

#[derive(Debug, thiserror::Error)]
pub enum MeshError {
    #[error("The requested node '{label}' does not exist. The requsted index was {index}")]
    NodeDoesNotExit { label: &'static str, index: Index<Node> },
    #[error("The requested edge '{label}' does not exist. The requsted index was {index}")]
    EdgeDoesNotExit { label: &'static str, index: Index<Edge> },
    #[error("Error while building Mesh: {error}")]
    MeshBuildError { error: MeshBuilderError }
}