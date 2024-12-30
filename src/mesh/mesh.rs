use apply::{Also, Apply};
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

pub struct TriangleMeshHalfEdgeContraction {
    pub source: (Index<Node>, Node),
    pub target: Index<Node>,
    pub facettes: Vec<Index<Facette>>,
    pub removed_edges: Vec<(Index<Edge>, Edge)>,
    pub removed_facettes: [(Index<Facette>, Option<Facette>); 2],
}

#[derive(Clone)]
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
        let triangles = facettes
            .take()
            .into_iter()
            .map(|facette| facette.map(|facette| facette.into()))
            .collect();
        let edges: Vec<Option<Edge>> = edges
            .take()
            .into_iter()
            .map(|edge| edge.map(|edge| edge.into()))
            .collect();
        let edges = List::new(edges);
        let nodes = nodes
            .take()
            .into_iter()
            .map(|node| node.map(|node| Into::<Node>::into(node).also(|it| {
                it.outgoing = Self::collect_outgoing_edges_with_edges(&edges, it);
            })))
            .collect();
        let mesh = Self { 
            nodes: List::new(nodes), 
            edges, 
            triangles:  List::new(triangles)
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

    fn next_edge(&self, index: Index<Edge>) -> &Edge {
        self[self[index].as_ref().unwrap().next].as_ref().unwrap()
    }

    fn next_edge_mut(&mut self, index: Index<Edge>) -> Option<&mut Edge> {
        let next = self[index].as_ref().unwrap().next;
        self[next].as_mut()
    }

    fn previous_edge(&self, index: Index<Edge>) -> &Edge {
        self[self[index].as_ref().unwrap().previous].as_ref().unwrap()
    }
    
    fn previous_edge_mut(&mut self, index: Index<Edge>) -> Option<&mut Edge> {
        let previous = self[index].as_ref().unwrap().previous;
        self[previous].as_mut()
    }
    
    fn opposite_edge(&self, index: Index<Edge>) -> Index<Edge> {
        self[index].as_ref().unwrap().opposite
    }


    fn opposite_edge_mut(&mut self, index: Index<Edge>) -> Option<&mut Edge> {
        let opposite = self[index].as_ref().unwrap().opposite;
        self[opposite].as_mut()
    }

    fn next_outgoing(&self, index: Index<Edge>) -> Index<Edge> {
        Self::next_outgoing_with_edges(&self.edges, index)
    }

    fn next_outgoing_with_edges(edges: &List<Edge>, index: Index<Edge>) -> Index<Edge> {
        edges[index].as_ref().unwrap().apply(|it| {
            edges[it.opposite].as_ref().unwrap().next
        })
    }

    fn collect_outgoing_edges(&self, index: Index<Node>) -> Vec<Index<Edge>> {
        Self::collect_outgoing_edges_with_edges(&self.edges, self[index].as_ref().unwrap())
    }

    fn collect_outgoing_edges_with_edges(edges: &List<Edge>, node: &Node) -> Vec<Index<Edge>> {
        let mut edge = Self::next_outgoing_with_edges(&edges, node.outgoing[0]);
        let mut outgoing = vec![edge];
        while edge != node.outgoing[0] {
            edge = Self::next_outgoing_with_edges(&edges, edge);
            outgoing.push(edge);
        }
        outgoing
    }

    pub fn contract_edge(&mut self, index: Index<Edge>) -> TriangleMeshHalfEdgeContraction {

        // Update the edges attached to the facettes edge_facette/opposite_facette.
        // Those facettes are removed and attached to the next facettes counterclockwise/clockwise with respect to the source node.
        // The edges of those facettes also need to be updated to maintain integrity of the HES.
        // 
        //   b === a
        //    \\ // \\
        //      s === t
        //    // \\ //
        //   c === d
        //
        // Here, edge is the edge st (the upper one) and opposite ts (so the lower one).
        // The edges st, ts, sa, as, sd and ds need to be removed.
        // The resulting graph would look like the following
        //
        //  b === a
        //   \\ //
        //     t
        //   // \\
        //  c === d
        //
        let st = std::mem::take(&mut self[index]).unwrap();
        let ts = std::mem::take(&mut self[st.opposite]).unwrap();
        let source = std::mem::take(&mut self[st.source]).unwrap();

        let mut facettes = vec![];
        let outgoing = source.outgoing.clone().into_iter().filter(|index| self[*index].is_some()).collect::<Vec<_>>();
        for outgoing in outgoing {
            let index = self[outgoing].as_ref().unwrap().facette;

            // Remember all indices of facettes which had s as corner but not t.
            // Those are later needed to uncontract the mesh.
            if index != st.facette && index != ts.facette {
                facettes.push(index)
            } 

            let facette = self[index].as_mut().unwrap();
            println!("HEC: Facette: {facette:?}");
            for corner in &mut facette.corners {
                if *corner == st.source {
                    debug!("HEC: Replacing corner {corner} with target {}", st.target);
                   *corner = st.target 
                }
            }
            self[st.target].as_mut().unwrap().outgoing.push(outgoing);
            self[outgoing].as_mut().unwrap().source = st.target;
            let opposite = self[outgoing].as_ref().unwrap().opposite;
            self[opposite].as_mut().unwrap().target = st.target;
        }
        
        let st_facette = std::mem::take(&mut self[st.facette]);
        let ts_facette = std::mem::take(&mut self[ts.facette]);
        
        let ta = st.next;
        let dt = ts.previous;
        let td = self[dt].as_ref().unwrap().opposite;
    
        let _as = std::mem::take(&mut self[st.previous]).unwrap();
        let sa = std::mem::take(&mut self[_as.opposite]).unwrap();

        let ds = std::mem::take(&mut self[ts.next]).unwrap();
        let sd = std::mem::take(&mut self[ds.opposite]).unwrap();

        let (ab, bs, facette_sa) = (sa.next, sa.previous, sa.facette);
        let (sc, cd, facette_sd) = (sd.next, sd.previous, sd.facette);

        self[ab].as_mut().map(|edge| edge.previous = ta);
        self[ab].as_mut().map(|edge| edge.previous = ta);
        self[bs].as_mut().map(|edge| edge.next = ta);
        
        let previous = if self[sa.previous].as_ref().is_some() { sa.previous } else { dt };
        self[ta].as_mut().map(|edge| edge.next = ab);
        self[ta].as_mut().map(|edge| edge.previous = previous);
        self[ta].as_mut().map(|edge| edge.facette = facette_sa);

        let (next, previous) = if let Some(edge) = self[sd.next].as_ref() {
            (sd.next, edge.next)
        } else {
            let t_outgoing = self[st.target].as_ref().unwrap().outgoing.clone().also(|it| {
                let rotate_by = it.iter().enumerate().find(|(_, edge)| **edge == td).unwrap().0;
                it.rotate_left(rotate_by);
                it.remove(0);
            });

            println!("Looking at edge: {index}. Searching dt: {dt}, so td: {td}");
            println!("Outgoing at t: {t_outgoing:?}");

            // for outgoing in t_outgoing {
            //     if let Some(next) = self[outgoing].as_ref().map(|it| it.next) {
            //         println!("Selected");
            //         self[dt].as_mut().unwrap().next = outgoing;
            //         self[dt].as_mut().unwrap().previous = next;
            //     }
            // }

            t_outgoing.into_iter().find_map(|outgoing| {
                self[outgoing].as_ref().map(|it| (outgoing, it.next))
            }).unwrap()
        };

        self[sc].as_mut().map(|edge| edge.previous = dt);
        self[cd].as_mut().map(|edge| edge.next = dt);
        self[dt].as_mut().map(|edge| edge.next = next);
        self[dt].as_mut().map(|edge| edge.previous = previous);
        self[dt].as_mut().map(|edge| edge.facette = facette_sd);

        self[ts.source].as_mut().unwrap().outgoing = self.collect_outgoing_edges(ts.source);

        TriangleMeshHalfEdgeContraction {
            source: (st.source, source.clone()),
            target: st.target,
            facettes,
            removed_facettes: [(st.facette, st_facette), (ts.facette, ts_facette)],
            removed_edges: vec![(_as.opposite, sa), (st.previous, _as), (ds.opposite, sd), (ts.next, ds), (st.opposite, ts), (index, st)],
        }
    }

    pub fn uncontract_edge(
        &mut self, 
        TriangleMeshHalfEdgeContraction { 
            source, 
            target, 
            facettes, 
            removed_edges, 
            removed_facettes }: TriangleMeshHalfEdgeContraction,
        ) {

        let (idx_source, source) = source;
        let (idx_st, st) = removed_edges.last().as_deref().cloned().unwrap();

        let removed_edge_indices = removed_edges.iter().map(|(i, _)| i).cloned().collect::<Vec<_>>();
        for (index, edge) in removed_edges {
            self[index] = Some(edge)
        }

        for (index, facette) in removed_facettes {
            self[index] = facette
        }

        // Correct all edges which were originally outgoing from s
        for outgoing in source.outgoing.clone() {
            self[outgoing].as_mut().unwrap().source = idx_source;
            let opposite = self[outgoing].as_ref().unwrap().opposite;
            self[opposite].as_mut().unwrap().target = idx_source;
        }
        self[idx_source] = Some(source.clone());

        self[st.next].as_mut().unwrap().apply(|it| {
            it.previous = idx_st;
            it.next = st.previous;
            it.facette = st.facette;
        });
        let ts = self[st.opposite].as_ref().cloned().unwrap();
        self[ts.previous].as_mut().unwrap().apply(|it| {
            it.next = st.opposite;
            it.previous = ts.next;
            it.facette = ts.facette;
        });

        for index in facettes {
            let facette = self[index].as_mut().unwrap();
            for corner in &mut facette.corners {
                if *corner == target {
                    *corner = idx_source
                } 
            }
        }
        self[ts.source].as_mut().unwrap().outgoing = self.collect_outgoing_edges(ts.source);

        // Restore the next and previous relations
        let indices = (0..(source.outgoing.len())).collect::<Vec<_>>().also(|it| it.rotate_left(1));
        for i in 0..(source.outgoing.len()) {
            let edge = source.outgoing[i];
            let opposite = self[edge].as_ref().unwrap().opposite;
            self[opposite].as_mut().unwrap().next = source.outgoing[indices[i]];
            self[source.outgoing[indices[i]]].as_mut().unwrap().previous = opposite;
            let (a, next, previous, facette) = self[edge].as_ref().unwrap().apply(|it| (it.source, it.next, it.previous, it.facette));
            let (b, c) = self[next].as_mut().unwrap().apply(|it| {
                it.next = previous;
                it.previous = edge;
                it.facette = facette;
                (it.source, it.target)
            });
            self[facette].as_mut().unwrap().corners = vec![a, b, c];
        }
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
                        ).with_inserted_attribute(bevy::prelude::Mesh::ATTRIBUTE_POSITION, nodes)
                        .with_inserted_indices(Indices::U32([0,1,2,3,4,5].to_vec()))
                        .with_inserted_attribute(bevy::prelude::Mesh::ATTRIBUTE_COLOR, vec![color, color, color])
                        .with_inserted_attribute(bevy::prelude::Mesh::ATTRIBUTE_NORMAL, [normal; 3].to_vec())
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

#[cfg(test)]
mod tests {
    use crate::mesh::{builder, MeshBuilderError, UnfinishedNode};

    use super::*;

    fn tetraeder() -> ClosedTriangleMesh {
        let mut builder = MeshBuilder::default();
        let nodes: Vec<_> = vec![
            [1.0f32,1.0f32,1.0f32],
            [1.0f32,1.0f32,-1.0f32],
            [1.0f32,-1.0f32,1.0f32],
            [1.0f32,-1.0f32,-1.0f32],
        ].into_iter().map(|node| {
            builder.add_node(UnfinishedNode::new(node))
        }).collect();
        vec![
            [0, 1, 2], [2, 1, 3],
            [0, 3, 1], [0, 2, 3] 
        ].into_iter().for_each(|[a, b, c]| {
            builder.add_triangle_by_nodes(nodes[a], nodes[b], nodes[c]).unwrap();
        });
        ClosedTriangleMesh::build(builder).unwrap()
    }

    fn cube() -> ClosedTriangleMesh {
        let mut builder = MeshBuilder::default();
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
            builder.add_node(UnfinishedNode::new(node))
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
                builder.add_triangle_by_nodes(nodes[a], nodes[b], nodes[c]).unwrap();
            });

        ClosedTriangleMesh::build(builder).unwrap()
    }

    #[test]
    fn outgoing_edges_sorted() -> Result<(), MeshBuilderError> {
        let mesh = cube();
        for node in mesh.nodes.take().into_iter().map(|node| node.unwrap()) {
            for (index, edge) in node.outgoing.iter().cloned().enumerate() {
                assert_eq!(ClosedTriangleMesh::next_outgoing_with_edges(&mesh.edges, edge), node.outgoing[(index + 1) % node.outgoing.len()]);
            }
        }

        Ok(())
    }

    #[test]
    fn tetraeder_hec() -> Result<(), MeshBuilderError> {
        let mut mesh = tetraeder();
        let target = mesh.edges[0.into()].as_ref().unwrap().target;
        let outgoing_edges = mesh[mesh.edges[0.into()].as_ref().unwrap().source].as_ref().unwrap().outgoing.clone();

        mesh.contract_edge(0.into());
        assert_eq!(2, mesh.triangles.iter().filter(|tri| tri.is_some()).count());
        assert_eq!(6, mesh.edges.iter().filter(|edge| edge.is_some()).count());
        assert_eq!(3, mesh.nodes.iter().filter(|node| node.is_some()).count());

        for edge in outgoing_edges {
            if let Some(edge) = mesh[edge].as_ref() {
                assert_eq!(target, edge.source);
                assert_eq!(target, mesh[edge.opposite].as_ref().unwrap().target);
            }
        }

        for [idx_ab, idx_bc, idx_ca] in [[2.into(), 9.into(), 7.into()], [3.into(), 6.into(), 8.into()]] as [[Index<Edge>; 3]; 2] {
            let (ab, bc, ca) = (
                mesh[idx_ab].as_ref().unwrap(),
                mesh[idx_bc].as_ref().unwrap(),
                mesh[idx_ca].as_ref().unwrap()
            );

            assert_eq!(ab.next, idx_bc);
            assert_eq!(ab.previous, idx_ca);
            assert_eq!(bc.next, idx_ca);
            assert_eq!(bc.previous, idx_ab);
            assert_eq!(ca.next, idx_ab);
            assert_eq!(ca.previous, idx_bc);

            assert_eq!(ab.facette, bc.facette);
            assert_eq!(bc.facette, ca.facette);
        }

        Ok(())
    }

    #[test]
    fn tetraeder_hec_undo() {
        let mut mesh = tetraeder();
        let copy = mesh.clone();
        let contraction = mesh.contract_edge(0.into());
        mesh.uncontract_edge(contraction);
        copy.nodes.iter().zip(mesh.nodes.iter()).for_each(|(orig, node)| {
            assert_eq!(orig.is_some(), node.is_some());
            if orig.is_some() {
                let (orig, node) = (orig.clone().unwrap(), node.clone().unwrap());
                assert_eq!(orig.coordinates, node.coordinates);
                for outgoing in &orig.outgoing {
                    assert!(node.outgoing.contains(outgoing))
                }
                for outgoing in &node.outgoing {
                    assert!(orig.outgoing.contains(outgoing))
                }
            }
        });
        for (index, (orig, copy)) in copy.edges.iter().zip(mesh.edges.iter()).enumerate() {
            assert_eq!(orig.is_some(), copy.is_some(), "edges are different at {index}");
            let (orig, copy) = (orig.clone().unwrap(), copy.clone().unwrap());
            assert_eq!(orig, copy, "edges are different at {index}");
        }
        assert_eq!(copy.triangles, mesh.triangles);
    }

    #[test]
    fn cube_single_hec() -> Result<(), MeshBuilderError> {
        let mut mesh = cube();
        let source = mesh.edges[0.into()].as_ref().unwrap().source;
        let target = mesh.edges[0.into()].as_ref().unwrap().target;
        let st: Index<Edge> = 0.into();
        let ts: Index<Edge> = mesh[st].as_ref().unwrap().opposite;
        let outgoing_edges = mesh[mesh.edges[st].as_ref().unwrap().source].as_ref().unwrap().outgoing.clone();

        mesh.contract_edge(0.into());
        assert_eq!(10, mesh.triangles.iter().filter(|tri| tri.is_some()).count());
        assert_eq!(30, mesh.edges.iter().filter(|edge| edge.is_some()).count());
        assert_eq!(7, mesh.nodes.iter().filter(|node| node.is_some()).count());

        for edge in outgoing_edges {
            if let Some(edge) = mesh[edge].as_ref() {
                assert_eq!(target, edge.source);
                assert_eq!(target, mesh[edge.opposite].as_ref().unwrap().target);
            }
        }

        assert_eq!(mesh.edges[14.into()].as_ref().unwrap().source, 1.into());
        assert_eq!(mesh.edges[15.into()].as_ref().unwrap().target, 1.into());

        assert!(mesh.edges[0.into()].as_ref().is_none());
        assert!(mesh.edges[1.into()].as_ref().is_none());
        assert!(mesh.edges[4.into()].as_ref().is_none());
        assert!(mesh.edges[5.into()].as_ref().is_none());
        assert!(mesh.edges[10.into()].as_ref().is_none());
        assert!(mesh.edges[11.into()].as_ref().is_none());

        // Check that the modified edges and their adjacent facette edges are correct
        let pairings: Vec<(usize, usize, usize)> = vec![
            (2, 1, 2), (3, 2, 1), 
            (6, 1, 3), (7, 3, 1),
            (8, 3, 2), (9, 2, 3),
            (12, 5, 1), (13, 1, 5),
            (14, 1, 4), (15, 4, 1),
            (16, 4, 5), (17, 5, 4),
            (18, 2, 4), (19, 4, 2)
        ];
        for (edge, source, target) in pairings {
            let edge = mesh.edges[edge.into()].as_ref().unwrap();
            assert_eq!(edge.source, source.into());
            assert_eq!(edge.target, target.into());
        }

        // Check that the edges surround each modified facette
        // Check that the facettes are correct
        for [idx_ab, idx_bc, idx_ca] in [[2.into(), 18.into(), 15.into()], [3.into(), 6.into(), 8.into()], [12.into(), 14.into(), 16.into()]] as [[Index<Edge>; 3]; 3] {
            let (ab, bc, ca) = (
                mesh[idx_ab].as_ref().unwrap(),
                mesh[idx_bc].as_ref().unwrap(),
                mesh[idx_ca].as_ref().unwrap()
            );

            assert_eq!(ab.next, idx_bc);
            assert_eq!(ab.previous, idx_ca);
            assert_eq!(bc.next, idx_ca);
            assert_eq!(bc.previous, idx_ab);
            assert_eq!(ca.next, idx_ab);
            assert_eq!(ca.previous, idx_bc);

            assert_eq!(ab.facette, bc.facette);
            assert_eq!(bc.facette, ca.facette);

            let facette = mesh[ab.facette].as_ref().unwrap();
            assert_eq!(facette.corners.len(), 3);
            for edge in [ab, bc, ca] {
                assert!(facette.corners.contains(&edge.source))
            }
        }

        Ok(())
    }

    #[test]
    fn cube_hec_undo() {
        let mut mesh = cube();
        let copy = mesh.clone();
        let contraction = mesh.contract_edge(0.into());
        mesh.uncontract_edge(contraction);
        copy.nodes.iter().zip(mesh.nodes.iter()).for_each(|(orig, node)| {
            assert_eq!(orig.is_some(), node.is_some());
            if orig.is_some() {
                let (orig, node) = (orig.clone().unwrap(), node.clone().unwrap());
                assert_eq!(orig.coordinates, node.coordinates);
                for outgoing in &orig.outgoing {
                    assert!(node.outgoing.contains(outgoing))
                }
                for outgoing in &node.outgoing {
                    assert!(orig.outgoing.contains(outgoing))
                }
            }
        });
        for (index, (orig, copy)) in copy.edges.iter().zip(mesh.edges.iter()).enumerate() {
            assert_eq!(orig.is_some(), copy.is_some(), "edges are different at {index}");
            let (orig, copy) = (orig.clone().unwrap(), copy.clone().unwrap());
            assert_eq!(orig, copy, "edges are different at {index}");
        }
        assert_eq!(copy.triangles, mesh.triangles);
    }


    #[test]
    fn cube_hec() -> Result<(), MeshBuilderError> {
        let mut mesh = cube();
        let contraction = mesh.contract_edge(0.into());

        assert_eq!(contraction.source.0, 0.into());
        assert_eq!(contraction.target, 1.into());
        for i in [0,1,4,5,10,11] {
            assert_eq!(1, contraction.removed_edges.iter().filter(|(index,_)| *index == Index::<Edge>::new(i)).count())
        }
        assert_ne!([None, None], contraction.removed_facettes.clone().map(|(_, f)| f));
        assert_eq!(None, mesh[contraction.removed_facettes[0].0]);
        assert_eq!(None, mesh[contraction.removed_facettes[1].0]);

        assert_eq!(mesh.edges[0.into()], None);
        assert_eq!(mesh.edges[1.into()], None);
        assert_eq!(mesh.edges[4.into()], None);
        assert_eq!(mesh.edges[5.into()], None);
        assert_eq!(mesh.edges[10.into()], None);
        assert_eq!(mesh.edges[11.into()], None);

        assert_eq!(30, mesh.edges.iter().filter(|edge| edge.is_some()).count());
        assert_eq!(10, mesh.triangles.iter().filter(|facette| facette.is_some()).count());
        let facettes_with_zero =  mesh.triangles
            .iter()
            .enumerate()
            .filter(|(_, facette)| facette.as_ref().cloned().map(|facette| facette.corners.contains(&0.into())).unwrap_or(false))
            .collect::<Vec<_>>();
        assert!(facettes_with_zero.is_empty(), "facettes containing zero: {:?}", facettes_with_zero);
        Ok(())
    }
}