use bevy::{color::{Color, ColorToComponents}, render::{mesh::Indices, render_asset::RenderAssetUsages}};
use log::{debug, info, error};
use nalgebra::Vector3;

use super::{Edge, Facette, FromMeshBuilder, Index, List, MeshBuilder, MeshError, Node};

// impl From<TriangleMeshEdge> for ClosedTriangleMeshEdge {
//     fn from(value: TriangleMeshEdge) -> Self {
//         Self {
//             source: value.source,
//             target: value.target,
//             opposite: value.opposite,
//             previous: value.previous.unwrap(),
//             next: value.next.unwrap(),
//             triangle: value.triangle.unwrap()
//         }
//     }
// }

pub struct ClosedTriangleMesh {
    pub nodes: List<Node>,
    edges: List<Edge>, 
    pub triangles: List<Facette>,
}

impl core::ops::Index<Index<Node>> for ClosedTriangleMesh {
    type Output = <List<Node> as core::ops::Index<Index<Node>>>::Output;

    fn index(&self, index: Index<Node>) -> &Self::Output {
        &self.nodes[index]
    }
}

impl core::ops::IndexMut<Index<Node>> for ClosedTriangleMesh {
    fn index_mut(&mut self, index: Index<Node>) -> &mut Self::Output {
        &mut self.nodes[index]
    }
}

impl core::ops::Index<Index<Edge>> for ClosedTriangleMesh {
    type Output = <List<Edge> as core::ops::Index<Index<Edge>>>::Output;

    fn index(&self, index: Index<Edge>) -> &Self::Output {
        &self.edges[index]
    }
}

impl core::ops::IndexMut<Index<Edge>> for ClosedTriangleMesh {
    fn index_mut(&mut self, index: Index<Edge>) -> &mut Self::Output {
        &mut self.edges[index]
    }
}

impl core::ops::Index<Index<Facette>> for ClosedTriangleMesh {
    type Output = <List<Facette> as core::ops::Index<Index<Facette>>>::Output;

    fn index(&self, index: Index<Facette>) -> &Self::Output {
        &self.triangles[index]
    }
}

impl core::ops::IndexMut<Index<Facette>> for ClosedTriangleMesh {
    fn index_mut(&mut self, index: Index<Facette>) -> &mut Self::Output {
        &mut self.triangles[index]
    }
}

impl FromMeshBuilder for ClosedTriangleMesh {
    fn build(builder: super::MeshBuilder) -> Result<super::ClosedTriangleMesh, super::MeshBuilderError> {
        // builder.triangles.iter().enumerate().for_each(|(idx, triangle)| debug!("Triangle index: {idx}, Triangle: {triangle:?}"));
        let MeshBuilder { nodes, edges, facettes } = builder;
        let mesh = Self { 
            nodes: List::new(nodes.take().into_iter().map(|node| node.map(|node| Into::<Node>::into(node))).collect()), 
            edges: List::new(edges.take().into_iter().map(|edge| edge.map(|edge| edge.into())).collect()), 
            triangles:  List::new(facettes.take().into_iter().map(|facette| facette.map(|facette| facette.into())).collect())
        };
        Ok(mesh)
    }
}

impl ClosedTriangleMesh {

    pub fn has_node(&self, node: Index<Node>) -> bool {
        if (0..self.nodes.len()).contains(&node) {
            self[node].is_some()
        } else {
            false
        }
    }

    // TODO: correct error type
    fn check_nodes_exist<I: IntoIterator<Item = (&'static str, Index<Node>)> + std::fmt::Debug>(&self, nodes: I) -> Result<(), MeshError> {
        nodes.into_iter().try_for_each(|(name, node)| {
            if !self.has_node(node) {
                error!("Node {node} does not exist!");
                debug!("Nodes: {:?}", self.nodes);
                Err(MeshError::NodeDoesNotExit { label: name, index: node })
            } else {
                Ok(())
            }
        })
    }

    // TODO: correct error type
    fn check_edges_exist<I: IntoIterator<Item = (&'static str, Index<Edge>)>>(&self, edges: I) -> Result<(), MeshError> {
        edges.into_iter().try_for_each(|(name, edge)| {
            if !self.has_edge(edge) {
                Err(MeshError::EdgeDoesNotExit { label: name, index: edge })
            } else {
                Ok(())
            }
        })
    }

    pub fn has_edge(&self, edge: Index<Edge>) -> bool {
        if (0..self.edges.len()).contains(&edge) {
            self[edge].is_some()
        } else {
            false
        }
    }

    pub fn contract_edge(&mut self, index: Index<Edge>) {
        let edge = std::mem::replace(&mut self[index], None).unwrap();
        let opposite = std::mem::replace(&mut self[edge.opposite], None).unwrap();
        let mut source = std::mem::replace(&mut self[edge.source], None).unwrap();
        let target = edge.target;

        // let edge_facette = self[edge.triangle].as_ref().cloned().unwrap();
        // let opposite_facette = self[opposite.triangle].as_ref().cloned().unwrap();

        let edge_facette = std::mem::replace(&mut self[edge.facette], None).unwrap();
        let opposite_facette = std::mem::replace(&mut self[opposite.facette], None).unwrap_or(edge_facette.clone());

        source.outgoing = source.outgoing.into_iter().filter_map(|outgoing| {
            if index == outgoing {
                return None;
            }
            let opposite = {
                let edge_target = self[outgoing].as_mut().unwrap().target;
                if edge_facette.corners.contains(&edge.target) || opposite_facette.corners.contains(&edge.target) {
                    let edge = std::mem::replace(&mut self[outgoing], None).unwrap();
                    std::mem::replace(&mut self[edge.opposite], None);
                    return None;
                }
                let edge = self[outgoing].as_mut().unwrap();
                edge.source = target;
                edge.opposite
            };
            Some(outgoing)
        }).collect();
    }

    pub fn build_many(&self) -> Vec<bevy::prelude::Mesh> {
        self.triangles.iter().filter_map(|tri| {
            match tri {
                None => None,
                Some(tri) => {
                    debug!("Facette: {tri:?}");
                    let color = Color::srgba_u8(rand::random(), rand::random(), rand::random(), 128).to_srgba().to_vec4();
                    let nodes: Vec<_> = tri.corners.iter().map(|idx| self[*idx].as_ref().unwrap().coordinates.clone()).collect();
                    let normal = (Vector3::from(nodes[1]) - Vector3::from(nodes[0])).cross(&(Vector3::from(nodes[2]) - Vector3::from(nodes[0])));
                    let (normal, inv_normal) = (normal.data.0[0], (normal * -1.0).data.0[0]);

                    Some(
                        bevy::prelude::Mesh::new(
                            bevy::render::mesh::PrimitiveTopology::TriangleList,
                            RenderAssetUsages::default()
                        ).with_inserted_attribute(bevy::prelude::Mesh::ATTRIBUTE_POSITION, vec![nodes.clone(), nodes].into_iter().flatten().collect::<Vec<_>>())
                        .with_inserted_indices(Indices::U32([0,1,2,3,4,5].to_vec()))
                        .with_inserted_attribute(bevy::prelude::Mesh::ATTRIBUTE_COLOR, vec![color, color, color])
                        .with_inserted_attribute(bevy::prelude::Mesh::ATTRIBUTE_NORMAL, [[normal; 3].to_vec(), [inv_normal; 3].to_vec()].into_iter().flatten().collect::<Vec<_>>())
                    )
                }
            }
            // .with_inserted_attribute(bevy::prelude::Mesh::ATTRIBUTE_POSITION, tri.corners.into_iter().map(|n| n.coordinates).collect::<Vec<_>>())
            // .with_inserted_indices(Indices::U32(triangles.into_iter().flat_map(|tri| tri.corners).map(|i| i.0 as u32).collect()))
            // // .with_inserted_attribute(bevy::prelude::Mesh::ATTRIBUTE_COLOR, colors)
            // .with_computed_smooth_normals()
        }).collect()
    }
}

// impl MeshBuilder for ClosedTriangleMesh {
//     fn build(&self) -> bevy::prelude::Mesh {
//         println!("edges: \t{}", self.edges.iter().map(|e| format!("{}->{}, tri: {:?}", e.clone().unwrap().source.0, e.clone().unwrap().target.0, e.clone().unwrap().triangle)).collect::<Vec<String>>().join("\n\t"));

//         fn relocate<T: Clone,I,F: Fn(usize) -> I>(items: &Vec<Option<T>>, transform: F) -> (Vec<T>, Vec<I>) {
//             let mut _items: Vec<T> = Vec::with_capacity(items.len());
//             let mut transformations: Vec<I> = (0..items.len()).map(&transform).collect();
//             let (mut i, mut j) = (0, items.len() - 1);
//             while i <= j {
//                 match &items[i] {
//                     Some(node) => {
//                         _items.push(node.clone());
//                         transformations[i] = transform(i);
//                     },
//                     None => {
//                         while items[j].is_none() {
//                             j -= 1
//                         } 
//                         _items.push(items[j].clone().unwrap());
//                         transformations[j] = transform(j);
//                         j -= 1
//                     }
//                 }
//                 i += 1;
//             }
//             (_items, transformations)
//         }

//         let (mut nodes, node_transformations) = relocate(&self.nodes, MeshNodeIndex);
//         let (mut edges, edge_transformations) = relocate(&self.edges, MeshEdgeIndex);
//         let (triangles, triangle_transformations) = relocate(&self.triangles, MeshTriangleIndex);

//         // println!("original: {:#?}", self.triangles.clone());
//         // println!("relocated: {:#?}", triangles.clone());
//         for node in &mut nodes {
//             for edge in &mut node.outgoing {
//                 *edge =  edge_transformations[edge.0]
//             }
//         }
//         for i in 0..edges.len() {
//             edges[i].source = node_transformations[edges[i].source.0];
//             edges[i].target = node_transformations[edges[i].target.0];
//             edges[i].opposite = edge_transformations[edges[i].opposite.0];
//             edges[i].triangle = Some(triangle_transformations[edges[i].triangle.unwrap().0]);
//         }

//         let normals: Vec<[f32; 3]> = triangles.iter().map(|tri| {
//             let [a,b,c] = tri.corners.map(|v| nalgebra::Vector3::from(self[v].clone().unwrap().coordinates));
//             (b - a).cross(&(c - a)).into()
//         }).collect::<Vec<_>>();

//         let mut palette: [u8; 32] = [
//             255, 102, 159, 255, 255, 159, 102, 255, 236, 255, 102, 255, 121, 255, 102, 255, 102, 255,
//             198, 255, 102, 198, 255, 255, 121, 102, 255, 255, 236, 102, 255, 255,
//         ];

//         // let nodes = nodes.into_iter().map(|n| n.coordinates).collect::<Vec<_>>();
//         // let colors = 

//         // let triangles = triangles.into_iter().flat_map(|tri| tri.corners).map(|i| [i.0 as u32, (i.0 + nodes.len()) as u32]).flatten().collect();
//         // let nodes = [nodes.clone(), nodes].concat();


//         let colors: Vec<_> = (0..triangles.len()).map(|_| Color::srgba_u8(rand::random(), rand::random(), rand::random(), 128).to_srgba().to_vec4()).collect();

//         bevy::prelude::Mesh::new(
//             bevy::render::mesh::PrimitiveTopology::TriangleList,
//             RenderAssetUsages::default()
//         ).with_inserted_attribute(bevy::prelude::Mesh::ATTRIBUTE_POSITION, nodes.into_iter().map(|n| n.coordinates).collect::<Vec<_>>())
//         .with_inserted_indices(Indices::U32(triangles.into_iter().flat_map(|tri| tri.corners).map(|i| i.0 as u32).collect()))
//         .with_inserted_attribute(bevy::prelude::Mesh::ATTRIBUTE_COLOR, colors)
//         .with_computed_smooth_normals()


//     }
// }
