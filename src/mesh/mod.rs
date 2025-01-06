mod list;
mod mesh;
mod builder;

use std::collections::HashSet;

pub use builder::*;
pub use mesh::*;
pub use list::*;

// Vec of EdgeIndex -> No sorting, shuffling, preallocation of array, but bad cache effiency.
// Array start + length -> Good cache effiency, but preallocation and relocation needed.
#[derive(Clone, Debug, PartialEq)]
pub struct Node {
    // pub label: String,
    pub coordinates: [f32; 3],
    pub outgoing: Vec<Index<Edge>>,
}

impl Eq for Node {}
impl std::hash::Hash for Node {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.outgoing.hash(state)
    }
}

impl Node {
    pub fn new(coordinates: [f32; 3]) -> Self {
        Self { outgoing: vec![], coordinates }
    }
}

#[derive(Clone, Debug, Hash)]
pub struct Edge {
    pub source: Index<Node>,
    pub target: Index<Node>,
    pub opposite: Index<Edge>,
    pub previous: Index<Edge>,
    pub next: Index<Edge>,
    pub facette: Index<Facette>,
}

impl PartialEq for Edge {
    fn eq(&self, other: &Self) -> bool {
        return self.source == other.source && self.target == other.target;
    }
}

impl Eq for Edge {}

#[derive(Clone, Debug, Default)]
pub struct Facette {
    pub corners: Vec<Index<Node>>,
}

impl std::hash::Hash for Facette {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.corners.iter().for_each(|node| node.hash(state));
    }
}

impl PartialEq for Facette {
    fn eq(&self, other: &Self) -> bool {
        if self.corners.len() != other.corners.len() {
            return false
        }
        for node in &self.corners {
            if !other.corners.contains(node) {
                return false
            }
        }
        return true
    }
}

#[derive(Debug, thiserror::Error)]
pub enum MeshError {
    #[error("The requested node '{label}' does not exist. The requsted index was {index}")]
    NodeDoesNotExit { label: &'static str, index: Index<Node> },
    #[error("The requested edge '{label}' does not exist. The requsted index was {index}")]
    EdgeDoesNotExit { label: &'static str, index: Index<Edge> },
    #[error("The requested edge '{label}' does already exist")]
    EdgeAlreadyExists { label: &'static str, index: Index<Edge> },
    // #[error("Error while building Mesh: {error}")]
    // MeshBuildError { error: MeshBuilderError }
}