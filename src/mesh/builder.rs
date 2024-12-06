use bevy::{color::{Color, ColorToComponents}, log::error, render::{mesh::Indices, render_asset::RenderAssetUsages}};
use log::{debug, info};
use nalgebra::Vector3;
use thiserror::Error;

use super::{ClosedTriangleMesh, Edge, Index, List, MeshError, Node, Facette};

#[derive(Debug, Error)]
pub enum MeshBuilderError {
    #[error("The requested node '{label}' does not exist. The requsted index was {index}")]
    NodeDoesNotExit { label: &'static str, index: Index<UnfinishedNode> },
    #[error("The requested edge '{label}' does not exist. The requsted index was {index}")]
    UnfinishedHalfEdgeDoesNotExit { label: &'static str, index: Index<UnfinishedHalfEdge> },
}

pub trait FromMeshBuilder where Self: Sized {
    fn build(builder: MeshBuilder) -> Result<Self, MeshBuilderError>;
}

#[derive(Clone, Debug, PartialEq)]
pub struct UnfinishedNode {
    // pub label: String,
    pub coordinates: [f32; 3],
    pub outgoing: Vec<Index<UnfinishedHalfEdge>>,
}

impl Into<Node> for UnfinishedNode {
    fn into(self) -> Node {
        Node {
            coordinates: self.coordinates,
            outgoing: self.outgoing.into_iter().map(|idx| Index::new(*idx)).collect()
        }
    }
}

impl UnfinishedNode {
    pub fn new(coordinates: [f32; 3]) -> Self {
        Self { outgoing: vec![], coordinates }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct UnfinishedHalfEdge {
    pub source: Index<UnfinishedNode>,
    pub target: Index<UnfinishedNode>,
    pub opposite: Index<UnfinishedHalfEdge>,
    pub previous: Option<Index<UnfinishedHalfEdge>>,
    pub next: Option<Index<UnfinishedHalfEdge>>,
    pub facette: Option<Index<UnfinishedTriangle>>,
}

impl Into<Edge> for UnfinishedHalfEdge {
    fn into(self) -> Edge {
        Edge {
            source: (*self.source).into(),
            target: (*self.target).into(),
            opposite: (*self.opposite).into(),
            previous: (*self.previous.unwrap()).into(),
            next: (*self.next.unwrap()).into(),
            facette: (*self.facette.unwrap()).into()
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct UnfinishedTriangle {
    pub corners: [Index<UnfinishedNode>; 3],
}

impl Into<Facette> for UnfinishedTriangle {
    fn into(self) -> Facette {
        Facette {
            corners: self.corners.map(|idx| Index::new(*idx)).to_vec()
        }
    }
}

#[derive(Default)]
pub struct MeshBuilder {
    pub nodes: List<UnfinishedNode>,
    pub edges: List<UnfinishedHalfEdge>, 
    pub facettes: List<UnfinishedTriangle>
}


impl core::ops::Index<Index<UnfinishedNode>> for MeshBuilder {
    type Output = <List<UnfinishedNode> as core::ops::Index<Index<UnfinishedNode>>>::Output;

    fn index(&self, index: Index<UnfinishedNode>) -> &Self::Output {
        &self.nodes[index]
    }
}

impl core::ops::IndexMut<Index<UnfinishedNode>> for MeshBuilder {
    fn index_mut(&mut self, index: Index<UnfinishedNode>) -> &mut Self::Output {
        &mut self.nodes[index]
    }
}

impl core::ops::Index<Index<UnfinishedHalfEdge>> for MeshBuilder {
    type Output = <List<UnfinishedHalfEdge> as core::ops::Index<Index<UnfinishedHalfEdge>>>::Output;

    fn index(&self, index: Index<UnfinishedHalfEdge>) -> &Self::Output {
        &self.edges[index]
    }
}

impl core::ops::IndexMut<Index<UnfinishedHalfEdge>> for MeshBuilder {
    fn index_mut(&mut self, index: Index<UnfinishedHalfEdge>) -> &mut Self::Output {
        &mut self.edges[index]
    }
}

impl core::ops::Index<Index<UnfinishedTriangle>> for MeshBuilder {
    type Output = <List<UnfinishedTriangle> as core::ops::Index<Index<UnfinishedTriangle>>>::Output;

    fn index(&self, index: Index<UnfinishedTriangle>) -> &Self::Output {
        &self.facettes[index]
    }
}

impl core::ops::IndexMut<Index<UnfinishedTriangle>> for MeshBuilder {
    fn index_mut(&mut self, index: Index<UnfinishedTriangle>) -> &mut Self::Output {
        &mut self.facettes[index]
    }
}

impl MeshBuilder {
    pub fn add_node(&mut self, node: UnfinishedNode) -> Index<UnfinishedNode> {
        self.nodes.push(Some(node));
        (self.nodes.len() - 1).into()
    }

    /// Adds two directional edges: a -> b and a <- b.
    /// If `a` or `b` does not exist, will return an error.   
    pub fn add_edges(&mut self, a: Index<UnfinishedNode>, b: Index<UnfinishedNode>) -> Result<(Index<UnfinishedHalfEdge>, Index<UnfinishedHalfEdge>), MeshBuilderError> {
        self.check_nodes_exist(vec![("a", a), ("b", b)])?;

        let idx_ab = Index::<UnfinishedHalfEdge>::from(self.edges.len());
        let idx_ba = Index::<UnfinishedHalfEdge>::from(self.edges.len() + 1);

        let edge_ab = UnfinishedHalfEdge {
            source: a,
            target: b,
            opposite: idx_ba,
            facette: None,
            next: None,
            previous: None
        };
        let edge_ba = UnfinishedHalfEdge {
            source: b,
            target: a,
            opposite: idx_ab,
            facette: None,
            next: None,
            previous: None
        };
        self.edges.push(Some(edge_ab));
        self.edges.push(Some(edge_ba));

        self[a].as_mut().unwrap().outgoing.push(idx_ab);
        self[b].as_mut().unwrap().outgoing.push(idx_ba);

        info!("Successfully created edges {a}<->{b} with indeces ->{idx_ab} and <-{idx_ba}");
        Ok((idx_ab, idx_ba))
    }

    pub fn has_node(&self, node: Index<UnfinishedNode>) -> bool {
        if (0..self.nodes.len()).contains(&node) {
            self[node].is_some()
        } else {
            false
        }
    }

    // TODO: correct error type
    fn check_nodes_exist<I: IntoIterator<Item = (&'static str, Index<UnfinishedNode>)> + std::fmt::Debug>(&self, nodes: I) -> Result<(), MeshBuilderError> {
        debug!(nodes:?; "Checking if nodes exist");
        nodes.into_iter().try_for_each(|(name, node)| {
            if !self.has_node(node) {
                error!("Node {node} does not exist!");
                debug!("Nodes: {:?}", self.nodes);
                Err(MeshBuilderError::NodeDoesNotExit { label: name, index: node })
            } else {
                Ok(())
            }
        })
    }

    // TODO: correct error type
    fn check_edges_exist<I: IntoIterator<Item = (&'static str, Index<UnfinishedHalfEdge>)>>(&self, edges: I) -> Result<(), MeshBuilderError> {
        edges.into_iter().try_for_each(|(name, edge)| {
            if !self.has_edge(edge) {
                Err(MeshBuilderError::UnfinishedHalfEdgeDoesNotExit { label: name, index: edge })
            } else {
                Ok(())
            }
        })
    }

    pub fn has_edge(&self, edge: Index<UnfinishedHalfEdge>) -> bool {
        if (0..self.edges.len()).contains(&edge) {
            self[edge].is_some()
        } else {
            false
        }
    }

    pub fn add_triangle_by_nodes(&mut self, a: Index<UnfinishedNode>, b: Index<UnfinishedNode>, c: Index<UnfinishedNode>) -> Result<Index<UnfinishedTriangle>, MeshBuilderError> {
        info!("Adding triangle with nodes: a: {a}, b: {b}, c: {c}");
        self.check_nodes_exist([("a", a), ("b", b), ("c", c)])?;
        let tri_idx = self.facettes.len().into();
        self.facettes.push(Some(UnfinishedTriangle { corners: [a,b,c] }));

        let invert = [(a,b), (b,c), (c,a)].iter().any(|(s,t)| {
            match self[*s].as_ref().unwrap().outgoing.iter().find(|node| self[**node].as_ref().unwrap().target == *t) {
                Some(&edge) => self[edge].as_ref().unwrap().facette.is_some(),
                None => false
            }
        });

        let cycle = if invert {
            info!("triangle {a}->{b}->{c} is out of order. using {a}->{c}->{b}.");
            [(a,c), (c,b), (b,a)]
        } else {
            [(a,b), (b,c), (c,a)]
        };

        let edges = cycle.try_map(|(s,t)| {
            match self[s].as_ref().unwrap().outgoing.iter().find(|node| self[**node].as_ref().unwrap().target == t) {
                Some(&edge) => Ok(edge),
                None => {
                    info!("edge {s}->{t} not found in mesh. Attempting to create it.");
                    self.add_edges(s, t).map(|(s,_)| s)
                },
            }
        })?;
        for idx in 0..edges.len() {
            self[edges[(idx + 1) % 3]].as_mut().unwrap().previous = Some(edges[idx]);
            self[edges[idx]].as_mut().unwrap().next = Some(edges[(idx +1) % 3]);
            self[edges[idx]].as_mut().unwrap().facette = Some(tri_idx);
        }

        Ok(tri_idx)
    }

    pub fn add_triangle_by_edges(&mut self, ab: Index<UnfinishedHalfEdge>, bc: Index<UnfinishedHalfEdge>, ca: Index<UnfinishedHalfEdge>) -> Result<Index<UnfinishedTriangle>, MeshBuilderError> {
        self.check_edges_exist([("a->b", ab), ("b->c", bc), ("c->a", ca)])?;
        let tri_idx = Index::<UnfinishedTriangle>::from(self.facettes.len() + 1);
        let corners = [ab,bc,ca].map(|edge| {
            let edge = self[edge].as_mut().unwrap();
            edge.facette = Some(tri_idx);
            edge.source
        });
        self.facettes.push(Some(UnfinishedTriangle { 
            corners
        }));
        Ok(tri_idx)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn simple_cube_triangle_nodes() {
        let mut mesh = MeshBuilder::default();
        
        let nodes: Vec<_> = vec![
            [1.0f32,1.0f32,1.0f32],
            [1.0f32,1.0f32,-1.0f32],
            [1.0f32,-1.0f32,1.0f32],
            [1.0f32,-1.0f32,-1.0f32],
            [-1.0f32,1.0f32,1.0f32],
            [-1.0f32,1.0f32,-1.0f32],
            [-1.0f32,-1.0f32,1.0f32],
            [-1.0f32,-1.0f32,-1.0f32],
        ].into_iter().map(|node| {
            mesh.add_node(UnfinishedNode::new(node))
        }).collect();

        vec![
            [0, 1, 2], [2, 1, 3],
            [1, 0, 5], [5, 0, 4],
            [0, 2, 4], [2, 6, 4],
            [4, 6, 5], [5, 6, 7],
            [2, 3, 6], [3, 7, 6],
            [3, 1, 7], [1, 5, 7]
        ].into_iter()
            .for_each(|[a,b,c]| { 
                mesh.add_triangle_by_nodes(nodes[a], nodes[b], nodes[c]).unwrap(); 
            });

        assert_eq!(8, mesh.nodes.len());
        assert!(mesh.nodes.iter().filter(|node| node.is_none()).collect::<Vec<_>>().is_empty());
        assert_eq!(36, mesh.edges.len());
        assert!(mesh.edges.iter().filter(|edge| edge.is_none()).collect::<Vec<_>>().is_empty());
        assert_eq!(12, mesh.facettes.len());
        assert!(mesh.edges.iter().filter(|edge| edge.as_ref().unwrap().facette.is_none()).collect::<Vec<_>>().is_empty())
    }

    #[test]
    fn simple_cube_triangle_nodes_false_order() {
        let mut mesh = MeshBuilder::default();
        
        let nodes: Vec<_> = vec![
            [1.0f32,1.0f32,1.0f32],
            [1.0f32,1.0f32,-1.0f32],
            [1.0f32,-1.0f32,1.0f32],
            [1.0f32,-1.0f32,-1.0f32],
            [-1.0f32,1.0f32,1.0f32],
            [-1.0f32,1.0f32,-1.0f32],
            [-1.0f32,-1.0f32,1.0f32],
            [-1.0f32,-1.0f32,-1.0f32],
        ].into_iter().map(|node| {
            mesh.add_node(UnfinishedNode::new(node))
        }).collect();

        vec![
            [0, 1, 2], [2, 1, 3],
            [1, 5, 0], [5, 4, 0], // out of order
            [0, 2, 4], [2, 6, 4],
            [4, 6, 5], [5, 6, 7],
            [2, 6, 3], [3, 7, 6], // out of order
            [3, 1, 7], [1, 7, 5]  // out of order
        ].into_iter()
            .for_each(|[a,b,c]| { 
                mesh.add_triangle_by_nodes(nodes[a], nodes[b], nodes[c]).unwrap(); 
            });

        assert_eq!(8, mesh.nodes.len());
        assert!(mesh.nodes.iter().filter(|node| node.is_none()).collect::<Vec<_>>().is_empty());
        assert_eq!(36, mesh.edges.len());
        assert!(mesh.edges.iter().filter(|edge| edge.is_none()).collect::<Vec<_>>().is_empty());
        assert_eq!(12, mesh.facettes.len());
        assert!(mesh.edges.iter().filter(|edge| edge.as_ref().unwrap().facette.is_none()).collect::<Vec<_>>().is_empty())
    }
}