mod list;
mod mesh;
mod builder;
mod reduction;

use std::collections::HashSet;

pub use builder::*;
pub use mesh::*;
pub use list::*;

// Vec of EdgeIndex -> No sorting, shuffling, preallocation of array, but bad cache effiency.
// Array start + length -> Good cache effiency, but preallocation and relocation needed.
#[derive(Clone, Debug, PartialEq)]
#[derive(serde::Deserialize, serde::Serialize)]
pub struct Node {
    // pub label: String,
    pub coordinates: [f64; 3],
    pub outgoing: Vec<Index<Edge>>,
}

impl Eq for Node {}
impl std::hash::Hash for Node {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.outgoing.hash(state)
    }
}

impl Node {
    pub fn new(coordinates: [f64; 3]) -> Self {
        Self { outgoing: vec![], coordinates }
    }
}

#[derive(Clone, Debug, Hash)]
#[derive(serde::Deserialize, serde::Serialize)]
pub struct Edge {
    pub source: Index<Node>,
    pub target: Index<Node>,
    pub opposite: Index<Edge>,
    pub previous: Index<Edge>,
    pub next: Index<Edge>,
    pub triangle: Index<Triangle>,
}

impl PartialEq for Edge {
    fn eq(&self, other: &Self) -> bool {
        return self.source == other.source && self.target == other.target;
    }
}

impl Eq for Edge {}
#[derive(Clone, Debug)]
#[derive(serde::Deserialize, serde::Serialize)]
pub struct Triangle {
    pub corners: [Index<Node>; 3],
}

impl std::hash::Hash for Triangle {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.corners.iter().for_each(|node| node.hash(state));
    }
}

impl PartialEq for Triangle {
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