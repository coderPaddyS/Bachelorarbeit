use log::{debug, info, error};
use nalgebra::Vector3;

use super::{Edge, Index, List, MeshError, Node, Triangle};

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

#[derive(Default)]
pub struct ClosedTriangleMesh {
    pub nodes: List<Node>,
    edges: List<Edge>, 
    pub triangles: List<Triangle>,
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

impl core::ops::Index<Index<Triangle>> for ClosedTriangleMesh {
    type Output = <List<Triangle> as core::ops::Index<Index<Triangle>>>::Output;

    fn index(&self, index: Index<Triangle>) -> &Self::Output {
        &self.triangles[index]
    }
}

impl core::ops::IndexMut<Index<Triangle>> for ClosedTriangleMesh {
    fn index_mut(&mut self, index: Index<Triangle>) -> &mut Self::Output {
        &mut self.triangles[index]
    }
}

impl ClosedTriangleMesh {
    pub fn add_node(&mut self, node: Node) -> Index<Node> {
        self.nodes.push(Some(node))
    }

    /// Adds two directional edges: a -> b and a <- b.
    /// If `a` or `b` does not exist, will return an error.   
    // pub fn add_edges(&mut self, a: MeshNodeIndex, b: MeshNodeIndex) -> Result<(MeshEdgeIndex, MeshEdgeIndex), MeshError> {
    //     self.check_nodes_exist(vec![("a", a), ("b", b)])?;

    //     let idx_ab = MeshEdgeIndex(self.edges.len());
    //     let idx_ba = MeshEdgeIndex(self.edges.len() + 1);

    //     let edge_ab = ClosedTriangleMeshEdge {
    //         source: a,
    //         target: b,
    //         opposite: idx_ba,
    //         triangle: None,
    //         next: None,
    //         previous: None
    //     };
    //     let edge_ba = ClosedTriangleMeshEdge {
    //         source: b,
    //         target: a,
    //         opposite: idx_ab,
    //         triangle: None,
    //         next: None,
    //         previous: None
    //     };
    //     self.edges.push(Some(edge_ab));
    //     self.edges.push(Some(edge_ba));

    //     self[a].as_mut().unwrap().outgoing.push(idx_ab);
    //     self[b].as_mut().unwrap().outgoing.push(idx_ba);

    //     info!("Successfully created edges {a}<->{b} with indeces ->{idx_ab} and <-{idx_ba}");
    //     Ok((idx_ab, idx_ba))
    // }

    pub fn has_node(&self, node: Index<Node>) -> bool {
        if (0..self.nodes.len()).contains(&node) {
            self[node].is_some()
        } else {
            false
        }
    }

    // TODO: correct error type
    fn check_nodes_exist<I: IntoIterator<Item = (&'static str, Index<Node>)> + std::fmt::Debug>(&self, nodes: I) -> Result<(), MeshError> {
        debug!(nodes:?; "Checking if nodes exist");
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

    // pub fn build_many(&self) -> Vec<bevy::prelude::Mesh> {
    //     self.triangles.iter().filter_map(|tri| {
    //         match tri {
    //             None => None,
    //             Some(tri) => {
    //                 let color = Color::srgba_u8(rand::random(), rand::random(), rand::random(), 128).to_srgba().to_vec4();
    //                 let nodes: Vec<_> = tri.corners.iter().map(|idx| self[*idx].as_ref().unwrap().coordinates.clone()).collect();
    //                 let normal = (Vector3::from(nodes[1]) - Vector3::from(nodes[0])).cross(&(Vector3::from(nodes[2]) - Vector3::from(nodes[0])));
    //                 let (normal, inv_normal) = (normal.data.0[0], (normal * -1.0).data.0[0]);

    //                 Some(
    //                     bevy::prelude::Mesh::new(
    //                         bevy::render::mesh::PrimitiveTopology::TriangleList,
    //                         RenderAssetUsages::default()
    //                     ).with_inserted_attribute(bevy::prelude::Mesh::ATTRIBUTE_POSITION, vec![nodes.clone(), nodes].into_iter().flatten().collect::<Vec<_>>())
    //                     .with_inserted_indices(Indices::U32([0,1,2,3,4,5].to_vec()))
    //                     .with_inserted_attribute(bevy::prelude::Mesh::ATTRIBUTE_COLOR, vec![color, color, color])
    //                     .with_inserted_attribute(bevy::prelude::Mesh::ATTRIBUTE_NORMAL, [[normal; 3].to_vec(), [inv_normal; 3].to_vec()].into_iter().flatten().collect::<Vec<_>>())
    //                 )
    //             }
    //         }
    //         // .with_inserted_attribute(bevy::prelude::Mesh::ATTRIBUTE_POSITION, tri.corners.into_iter().map(|n| n.coordinates).collect::<Vec<_>>())
    //         // .with_inserted_indices(Indices::U32(triangles.into_iter().flat_map(|tri| tri.corners).map(|i| i.0 as u32).collect()))
    //         // // .with_inserted_attribute(bevy::prelude::Mesh::ATTRIBUTE_COLOR, colors)
    //         // .with_computed_smooth_normals()
    //     }).collect()
    // }
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