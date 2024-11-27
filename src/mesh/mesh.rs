use std::ops::{Index, IndexMut};

use bevy::{prelude::MeshBuilder, render::{mesh::Indices, render_asset::RenderAssetUsages}};

use super::{MeshEdgeIndex, MeshError, MeshNodeIndex, MeshTriangleIndex};

// Vec of EdgeIndex -> No sorting, shuffling, preallocation of array, but bad cache effiency.
// Array start + length -> Good cache effiency, but preallocation and relocation needed.
#[derive(Clone, Debug)]
pub struct MeshNode {
    // pub label: String,
    pub coordinates: [f32; 3],
    pub outgoing: Vec<MeshEdgeIndex>,
}

impl MeshNode {
    pub fn new(coordinates: [f32; 3]) -> Self {
        Self { outgoing: vec![], coordinates }
    }
}

#[derive(Clone, Debug)]
pub struct MeshEdge {
    pub source: MeshNodeIndex,
    pub target: MeshNodeIndex,
    pub opposite: MeshEdgeIndex,
    pub triangle: Option<MeshTriangleIndex>,
}

#[derive(Clone, Debug)]
pub struct MeshTriangle {
    pub corners: [MeshNodeIndex; 3],
}

#[derive(Default)]
pub struct Mesh {
    pub nodes: Vec<Option<MeshNode>>,
    edges: Vec<Option<MeshEdge>>, 
    pub triangles: Vec<Option<MeshTriangle>>
}


impl Index<MeshNodeIndex> for Mesh {
    type Output = Option<MeshNode>;

    fn index(&self, index: MeshNodeIndex) -> &Self::Output {
        &self.nodes[*index]
    }
}

impl IndexMut<MeshNodeIndex> for Mesh {
    fn index_mut(&mut self, index: MeshNodeIndex) -> &mut Self::Output {
        &mut self.nodes[*index]
    }
}

impl Index<MeshEdgeIndex> for Mesh {
    type Output = Option<MeshEdge>;

    fn index(&self, index: MeshEdgeIndex) -> &Self::Output {
        &self.edges[*index]
    }
}

impl IndexMut<MeshEdgeIndex> for Mesh {
    fn index_mut(&mut self, index: MeshEdgeIndex) -> &mut Self::Output {
        &mut self.edges[*index]
    }
}

impl Mesh {
    pub fn add_node(&mut self, node: MeshNode) -> MeshNodeIndex {
        self.nodes.push(Some(node));
        MeshNodeIndex(self.nodes.len() - 1)
    }

    /// Adds two directional edges: a -> b and a <- b.
    /// If `a` or `b` does not exist, will return an error.   
    pub fn add_edges(&mut self, a: MeshNodeIndex, b: MeshNodeIndex) -> Result<(MeshEdgeIndex, MeshEdgeIndex), MeshError> {
        self.check_nodes_exist(vec![("a", a), ("b", b)])?;

        let idx_ab = MeshEdgeIndex(self.edges.len());
        let idx_ba = MeshEdgeIndex(self.edges.len() + 1);

        let edge_ab = MeshEdge {
            source: a,
            target: b,
            opposite: idx_ba,
            triangle: None
        };
        let edge_ba = MeshEdge {
            source: b,
            target: a,
            opposite: idx_ab,
            triangle: None
        };
        self.edges.push(Some(edge_ab));
        self.edges.push(Some(edge_ba));

        self[a].as_mut().unwrap().outgoing.push(idx_ab);
        self[b].as_mut().unwrap().outgoing.push(idx_ba);
        Ok((idx_ab, idx_ba))
    }

    pub fn has_node(&self, node: MeshNodeIndex) -> bool {
        if (0..self.nodes.len()).contains(&node.0) {
            self[node].is_some()
        } else {
            false
        }
    }

    // TODO: correct error type
    fn check_nodes_exist<I: IntoIterator<Item = (&'static str, MeshNodeIndex)>>(&self, nodes: I) -> Result<(), MeshError> {
        nodes.into_iter().try_for_each(|(name, node)| {
            if !self.has_node(node) {
                Err(MeshError::NodeDoesNotExit { label: name, index: node })
            } else {
                Ok(())
            }
        })
    }

    // TODO: correct error type
    fn check_edges_exist<I: IntoIterator<Item = (&'static str, MeshEdgeIndex)>>(&self, edges: I) -> Result<(), MeshError> {
        edges.into_iter().try_for_each(|(name, edge)| {
            if !self.has_edge(edge) {
                Err(MeshError::EdgeDoesNotExit { label: name, index: edge })
            } else {
                Ok(())
            }
        })
    }

    pub fn has_edge(&self, edge: MeshEdgeIndex) -> bool {
        if (0..self.edges.len()).contains(&edge.0) {
            self[edge].is_some()
        } else {
            false
        }
    }

    pub fn add_triangle_by_nodes(&mut self, a: MeshNodeIndex, b: MeshNodeIndex, c: MeshNodeIndex) -> Result<MeshTriangleIndex, MeshError> {
        println!("a: {a}, b: {b}, c: {c}");
        self.check_nodes_exist([("a", a), ("b", b), ("c", c)])?;
        let tri_idx = MeshTriangleIndex(self.triangles.len());
        self.triangles.push(Some(MeshTriangle { corners: [a,b,c] }));

        for (s,t) in  [(a,b),(b,c),(c,a)] {
            let idx = match self[s].as_ref().unwrap().outgoing.iter().find(|node| self[**node].as_ref().unwrap().target == t) {
                Some(&edge) => edge,
                None => self.add_edges(s, t)?.0,
            };
            self[idx].as_mut().unwrap().triangle = Some(tri_idx)
        }

        Ok(tri_idx)
    }

    pub fn add_triangle_by_edges(&mut self, ab: MeshEdgeIndex, bc: MeshEdgeIndex, ca: MeshEdgeIndex) -> Result<MeshTriangleIndex, MeshError> {
        self.check_edges_exist([("a->b", ab), ("b->c", bc), ("c->a", ca)])?;
        let tri_idx = MeshTriangleIndex(self.triangles.len() + 1);
        let corners = [ab,bc,ca].map(|edge| {
            let edge = self[edge].as_mut().unwrap();
            edge.triangle = Some(tri_idx);
            edge.source
        });
        self.triangles.push(Some(MeshTriangle { 
            corners
        }));
        Ok(tri_idx)
    }
}

impl MeshBuilder for Mesh {
    fn build(&self) -> bevy::prelude::Mesh {
        println!("edges: {}", self.edges.iter().map(|e| format!("{}->{}, tri: {:?}", e.clone().unwrap().source.0, e.clone().unwrap().target.0, e.clone().unwrap().triangle)).collect::<Vec<String>>().join("\n"));

        fn relocate<T: Clone,I,F: Fn(usize) -> I>(items: &Vec<Option<T>>, transform: F) -> (Vec<T>, Vec<I>) {
            let mut _items: Vec<T> = Vec::with_capacity(items.len());
            let mut transformations: Vec<I> = (0..items.len()).map(&transform).collect();
            let (mut i, mut j) = (0, items.len() - 1);
            while i <= j {
                match &items[i] {
                    Some(node) => {
                        _items.push(node.clone());
                        transformations[i] = transform(i);
                    },
                    None => {
                        while items[j].is_none() {
                            j -= 1
                        } 
                        _items.push(items[j].clone().unwrap());
                        transformations[j] = transform(j);
                        j -= 1
                    }
                }
                i += 1;
            }
            (_items, transformations)
        }

        let (mut nodes, node_transformations) = relocate(&self.nodes, MeshNodeIndex);
        let (mut edges, edge_transformations) = relocate(&self.edges, MeshEdgeIndex);
        let (triangles, triangle_transformations) = relocate(&self.triangles, MeshTriangleIndex);

        // println!("original: {:#?}", self.triangles.clone());
        // println!("relocated: {:#?}", triangles.clone());
        for node in &mut nodes {
            for edge in &mut node.outgoing {
                *edge =  edge_transformations[edge.0]
            }
        }
        for i in 0..edges.len() {
            edges[i].source = node_transformations[edges[i].source.0];
            edges[i].target = node_transformations[edges[i].target.0];
            edges[i].opposite = edge_transformations[edges[i].opposite.0];
            edges[i].triangle = Some(triangle_transformations[edges[i].triangle.unwrap().0]);
        }

        bevy::prelude::Mesh::new(
            bevy::render::mesh::PrimitiveTopology::TriangleList,
            RenderAssetUsages::default()
        ).with_inserted_attribute(bevy::prelude::Mesh::ATTRIBUTE_POSITION, nodes.into_iter().map(|n| n.coordinates).collect::<Vec<_>>())
        .with_inserted_indices(Indices::U32(triangles.into_iter().flat_map(|tri| tri.corners).map(|i| i.0 as u32).collect()))


    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn simple_cube_triangle_nodes() {
        let mut mesh = Mesh::default();
        
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
            mesh.add_node(MeshNode::new(node))
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
        assert_eq!(12, mesh.triangles.len());
        assert!(mesh.edges.iter().filter(|edge| edge.as_ref().unwrap().triangle.is_none()).collect::<Vec<_>>().is_empty())
    }
}