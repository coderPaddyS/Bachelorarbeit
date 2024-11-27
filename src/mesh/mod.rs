mod index;
mod mesh;

pub use mesh::*;
pub use index::*;

#[derive(Debug, thiserror::Error)]
pub enum MeshError {
    #[error("The requested node '{label}' does not exist. The requsted index was {index}")]
    NodeDoesNotExit { label: &'static str, index: MeshNodeIndex },
    #[error("The requested edge '{label}' does not exist. The requsted index was {index}")]
    EdgeDoesNotExit { label: &'static str, index: MeshEdgeIndex }
}
