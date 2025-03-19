use std::{collections::{BinaryHeap, HashMap, HashSet}, io::BufRead, marker::PhantomData};

use apply::{Also, Apply};
use bevy::ecs::system::IntoSystem;
use itertools::Itertools;
use log::{debug, info, error};
use priority_queue::{DoublePriorityQueue, PriorityQueue};

use super::{reduction::ContractionOrder, Edge, FromMeshBuilder, Index, List, MeshBuilder, MeshError, Node, Triangle};
use super::reduction::DihedralAngle;

pub struct MeshTriangleInfo {
    pub nodes: Vec<[f32; 3]>,
    pub triangles: Vec<(usize, [u8; 4])>,
}

pub trait MeshCollapseInfo: Sized + Clone {
    fn source(&self) -> (Index<Node>, Node);
    fn target(&self) -> Index<Node>;
    fn facettes(&self) -> Vec<Index<Triangle>>;
    fn removed_edges(&self) -> Vec<(Index<Edge>, Edge)>;
    fn removed_facettes(&self) -> [(Index<Triangle>, Option<Triangle>); 2];
}

pub trait MeshData: Sized + Clone {
    type CollapseInfoInput: MeshCollapseInfo;
    type CollapseInfoOutput: MeshCollapseInfo;
    fn build_mesh<TM: TriangleMesh<Data = Self> + ?Sized>(mesh: &TM) -> Option<MeshTriangleInfo> { None }
    fn contract_edge(mesh: &mut ClosedTriangleMesh<Self>, collapse_info: Self::CollapseInfoInput) -> Self::CollapseInfoOutput;
    fn uncontract_edge(mesh: &mut ClosedTriangleMesh<Self>, collapse_info: Self::CollapseInfoOutput) {

    }
}

impl MeshData for () {
    type CollapseInfoInput = TriangleMeshHalfEdgeCollapse;
    type CollapseInfoOutput = TriangleMeshHalfEdgeCollapse;
    fn contract_edge(_: &mut ClosedTriangleMesh<Self>, collapse_info: Self::CollapseInfoInput) -> Self::CollapseInfoOutput {
        println!("info");
        collapse_info
    }
}

#[derive(Clone, PartialEq, Debug, serde::Deserialize, serde::Serialize)]
pub struct TriangleMeshHalfEdgeCollapse {
    pub source: (Index<Node>, Node),
    pub target: Index<Node>,
    pub facettes: Vec<Index<Triangle>>,
    pub removed_edges: Vec<(Index<Edge>, Edge)>,
    pub removed_facettes: [(Index<Triangle>, Option<Triangle>); 2],
}

impl MeshCollapseInfo for TriangleMeshHalfEdgeCollapse {
    fn source(&self) -> (Index<Node>, Node) {
        self.source.clone()
    }
    fn target(&self) -> Index<Node> {
        self.target.clone()
    }
    fn facettes(&self) -> Vec<Index<Triangle>> {
        self.facettes.clone()
    }
    fn removed_edges(&self) -> Vec<(Index<Edge>, Edge)> {
        self.removed_edges.clone()
    }
    fn removed_facettes(&self) -> [(Index<Triangle>, Option<Triangle>); 2] {
        self.removed_facettes.clone()
    }
}

pub enum HalfEdgeExpansionEvent {
    FacetteMoved,
    FacetteInserted
}

pub trait TriangleMesh: 
      core::ops::Index<Index<Node>, Output = <List<Node> as core::ops::Index<Index<Node>>>::Output> 
    + core::ops::Index<Index<Edge>, Output = <List<Edge> as core::ops::Index<Index<Edge>>>::Output> 
    + core::ops::Index<Index<Triangle>, Output = <List<Triangle> as core::ops::Index<Index<Triangle>>>::Output> {
    type Data: MeshData;
    fn nodes(&self) -> &List<Node>;
    fn current_nodes(&self) -> Vec<(Index<Node>, Node)>;
    fn edges(&self) -> &List<Edge>;
    fn current_edges(&self) -> Vec<(Index<Edge>, Edge)>;
    fn current_edges_undirected(&self) -> Vec<(Index<Edge>, Edge)>;
    fn triangles(&self) -> &List<Triangle>;
    fn current_triangles(&self) -> Vec<(Index<Triangle>, Triangle)>;
    fn triangle_data(&self) -> &List<Self::Data, Triangle>;

    fn build_mesh(&self) -> MeshTriangleInfo {
        if let Some(info) = Self::Data::build_mesh(self) {
            return info;
        }

        let (colors, nodes): (Vec<[u8; 4]>, Vec<[f32; 3]>) = self.triangles()
            .iter()
            .cloned()
            .filter_map(|tri| {
                match tri {
                    None => None,
                    Some(tri) => {
                        let color: [u8; 4] = [rand::random(), rand::random(), rand::random(), 128];
                        let nodes: Vec<_> = tri.corners.iter().map(|idx| self[*idx].as_ref().unwrap().coordinates.clone()).collect();
                        // let normal = (Vector3::from(nodes[1]) - Vector3::from(nodes[0])).cross(&(Vector3::from(nodes[2]) - Vector3::from(nodes[0])));

                        Some((color, nodes))
                    }
                }
            })
            .map(|(color, nodes)| {
                nodes.into_iter().map(move |node| (color, node.map(|n| n as f32)))
            })
            .flatten()
            .unzip();

        let triangles = (0..nodes.len()).into_iter().zip(colors).collect();
        MeshTriangleInfo { nodes, triangles }
    }

    fn find_edge(&self, source: Index<Node>, target: Index<Node>) -> Option<Index<Edge>> {
        self[source].as_ref().unwrap().outgoing.iter().find(|edge| self[**edge].as_ref().unwrap().target == target).copied()
    }
    fn next_outgoing(&self, index: Index<Edge>) -> Index<Edge> {
        self[index].as_ref().unwrap().apply(|it| {
            self[it.opposite].as_ref().unwrap().next
        })
    }
    fn collect_outgoing_edges(&self, index: Index<Node>) -> Vec<Index<Edge>> {
        self.collect_outgoing_edges_starting_with(self[index].as_ref().unwrap().outgoing[0])
    }
    fn collect_outgoing_edges_starting_with(&self, starting_edge: Index<Edge>) -> Vec<Index<Edge>> {
        let mut edge = self.next_outgoing(starting_edge);
        let mut outgoing = vec![edge];
        while edge != starting_edge {
            edge = self.next_outgoing(edge);
            outgoing.push(edge);
        }
        outgoing
    }
}

pub trait UncontractableMesh: TriangleMesh {
    fn uncontract_edge(&mut self, info: <<Self as TriangleMesh>::Data as MeshData>::CollapseInfoOutput); 
    fn uncontract_next_edge<F: FnOnce(&Self, &<<Self as TriangleMesh>::Data as MeshData>::CollapseInfoOutput)>(&mut self, before: F);
}

pub trait ContractableMesh: TriangleMesh {
    fn contract_edge(&mut self, index: Index<Edge>) -> &<<Self as TriangleMesh>::Data as MeshData>::CollapseInfoOutput;
    fn contract_next_edge(&mut self) -> &<<Self as TriangleMesh>::Data as MeshData>::CollapseInfoOutput;
}

#[derive(Clone)]
pub struct SimulatedEdgeCollapse<'m, TD: MeshData, TM: TriangleMesh> {
    collapsed_edges: HashMap<Index<Edge>, Option<Edge>>,
    collapsed_nodes: HashMap<Index<Node>, Index<Node>>,
    collapsed_triangles: HashMap<Index<Triangle>, Option<Triangle>>,
    original: &'m TM,
    _data: PhantomData<TD>
}

impl<'m, TD: MeshData, TM: TriangleMesh<Data = TD>> TriangleMesh for SimulatedEdgeCollapse<'m, TD, TM> {
    type Data = TD;
    fn nodes(&self) -> &List<Node> {
        self.original.nodes()
    }
    fn current_nodes(&self) -> Vec<(Index<Node>, Node)> {
        self.original.current_nodes()
        .into_iter()
        .filter(|node| self.collapsed_nodes.get(&node.0).is_some())
        .collect()
    }
    fn edges(&self) -> &List<Edge> {
        self.original.edges()
    }
    fn current_edges(&self) -> Vec<(Index<Edge>, Edge)> {
        self.original.current_edges()
        .into_iter()
        .filter(|edge| self.collapsed_edges.get(&edge.0).is_some())
        .collect()
    }
    fn current_edges_undirected(&self) -> Vec<(Index<Edge>, Edge)> {
        self.original.current_edges_undirected()
        .into_iter()
        .filter(|edge| self.collapsed_edges.get(&edge.0).is_some())
        .collect()
    }
    fn triangles(&self) -> &List<Triangle> {
        self.original.triangles()
    }
    fn current_triangles(&self) -> Vec<(Index<Triangle>, Triangle)> {
        self.original.current_triangles()
        .into_iter()
        .filter(|triangle| self.collapsed_triangles.get(&triangle.0).is_some())
        .collect()
    }
    fn triangle_data(&self) -> &List<Self::Data, Triangle> {
        self.original.triangle_data()
    }
}
impl<'m, TD: MeshData, TM: TriangleMesh> core::ops::Index<Index<Node>> for SimulatedEdgeCollapse<'m, TD, TM> {
    type Output = <TM as core::ops::Index<Index<Node>>>::Output;

    fn index(&self, index: Index<Node>) -> &Self::Output {
        &self.original[*self.collapsed_nodes.get(&index).or(Some(&index)).unwrap()]
    }
}

impl<'m, TD: MeshData, TM: TriangleMesh> core::ops::Index<Index<Edge>> for SimulatedEdgeCollapse<'m, TD, TM> {
    type Output = <TM as core::ops::Index<Index<Edge>>>::Output;

    fn index(&self, index: Index<Edge>) -> &Self::Output {
        self.collapsed_edges.get(&index).or(Some(&self.original[index])).unwrap()
    }
}

impl<'m, TD: MeshData, TM: TriangleMesh> core::ops::Index<Index<Triangle>> for SimulatedEdgeCollapse<'m, TD, TM> {
    type Output = <TM as core::ops::Index<Index<Triangle>>>::Output;

    fn index(&self, index: Index<Triangle>) -> &Self::Output {
        self.collapsed_triangles.get(&index).or(Some(&self.original[index])).unwrap()
    }
}

#[derive(Clone)]
pub struct ClosedTriangleMesh<TriangleData = ()>
where 
    TriangleData: MeshData    
{
    pub nodes: List<Node>,
    pub edges: List<Edge>, 
    pub triangles: List<Triangle>,
    pub triangle_data: List<TriangleData, Triangle>,
    pub contraction_order: DoublePriorityQueue<Index<Edge>, usize>,
    pub undo_contraction: Vec<TriangleData::CollapseInfoOutput>
}

impl<TD: MeshData> TriangleMesh for ClosedTriangleMesh<TD> {
    type Data = TD;

    fn nodes(&self) -> &List<Node> {
        &self.nodes
    }
    fn current_nodes(&self) -> Vec<(Index<Node>, Node)> {
        self.nodes.enumerate_some()
            .map(|(index, node)| (index, node.clone()))
            .collect()
    }

    fn edges(&self) -> &List<Edge> {
        &self.edges
    }
    fn current_edges(&self) -> Vec<(Index<Edge>, Edge)> {
        self.edges.iter()
            .enumerate()
            .filter_map(|(index, edge)| edge.clone().map(|edge| (Into::<Index<Edge>>::into(index), edge)))
            .collect()
    }
    fn current_edges_undirected(&self) -> Vec<(Index<Edge>, Edge)> {
        let mut contained = HashSet::<(Index<Node>, Index<Node>)>::with_capacity(self.edges.len());
        self.edges.iter()
            .enumerate()
            .filter_map(|(index, edge)| edge.clone().map(|edge| (Into::<Index<Edge>>::into(index), edge)))
            .filter(|(_, edge)| {
                if contained.contains(&(edge.source, edge.target)) || contained.contains(&(edge.target, edge.source)) {
                    false
                } else {
                    contained.insert((edge.source, edge.target));
                    true
                }
            })
            .collect()
    }
    fn triangles(&self) -> &List<Triangle> {
        &self.triangles
    }
    fn current_triangles(&self) -> Vec<(Index<Triangle>, Triangle)> {
        self.triangles.iter()
            .enumerate()
            .filter_map(|(index, triangle)| triangle.clone().map(|triangle| (Into::<Index<Triangle>>::into(index), triangle)))
            .collect()
    }
    fn triangle_data(&self) -> &List<Self::Data, Triangle> {
        &self.triangle_data
    }
}
impl<TD: MeshData> core::ops::Index<Index<Node>> for ClosedTriangleMesh<TD> {
    type Output = <List<Node> as core::ops::Index<Index<Node>>>::Output;

    fn index(&self, index: Index<Node>) -> &Self::Output {
        &self.nodes[index]
    }
}

impl<TD: MeshData> core::ops::IndexMut<Index<Node>> for ClosedTriangleMesh<TD> {
    fn index_mut(&mut self, index: Index<Node>) -> &mut Self::Output {
        &mut self.nodes[index]
    }
}

impl<TD: MeshData> core::ops::Index<Index<Edge>> for ClosedTriangleMesh<TD> {
    type Output = <List<Edge> as core::ops::Index<Index<Edge>>>::Output;

    fn index(&self, index: Index<Edge>) -> &Self::Output {
        &self.edges[index]
    }
}

impl<TD: MeshData> core::ops::IndexMut<Index<Edge>> for ClosedTriangleMesh<TD> {
    fn index_mut(&mut self, index: Index<Edge>) -> &mut Self::Output {
        &mut self.edges[index]
    }
}

impl<TD: MeshData> core::ops::Index<Index<Triangle>> for ClosedTriangleMesh<TD> {
    type Output = <List<Triangle> as core::ops::Index<Index<Triangle>>>::Output;

    fn index(&self, index: Index<Triangle>) -> &Self::Output {
        &self.triangles[index]
    }
}

impl<TD: MeshData> core::ops::IndexMut<Index<Triangle>> for ClosedTriangleMesh<TD> {
    fn index_mut(&mut self, index: Index<Triangle>) -> &mut Self::Output {
        &mut self.triangles[index]
    }
}

impl<TD: MeshData> FromMeshBuilder for ClosedTriangleMesh<TD> {
    fn build(builder: super::MeshBuilder) -> Result<super::ClosedTriangleMesh<TD>, super::MeshBuilderError> {
        // builder.triangles.iter().enumerate().for_each(|(idx, triangle)| debug!("Triangle index: {idx}, Triangle: {triangle:?}"));
        let MeshBuilder { nodes, edges, facettes } = builder;
        let triangles: Vec<_> = facettes
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
        let mut mesh = Self { 
            contraction_order: DoublePriorityQueue::with_capacity(edges.len()),
            nodes: List::new(nodes), 
            edges, 
            triangle_data: List::with_capacity(triangles.len()),
            triangles:  List::new(triangles),
            undo_contraction: Vec::new()
        };
        mesh.generate_contraction_order();
        Ok(mesh)
    }
}

impl<TD> ClosedTriangleMesh<TD>
where 
    TD: MeshData<CollapseInfoInput = TriangleMeshHalfEdgeCollapse, CollapseInfoOutput = TriangleMeshHalfEdgeCollapse>,
{
    pub fn drop_data(&self) -> ClosedTriangleMesh<()> {
        let ClosedTriangleMesh { nodes, edges, triangles, triangle_data, contraction_order, undo_contraction } = self.clone();
        ClosedTriangleMesh { nodes, edges, triangle_data: List::with_defaults(self.triangles.len()), triangles, contraction_order, undo_contraction }
    }
}

impl<TD: MeshData> ClosedTriangleMesh<TD> {

    pub fn has_node(&self, node: Index<Node>) -> bool {
        if (0..self.nodes.len()).contains(&node) {
            self[node].is_some()
        } else {
            false
        }
    }

    fn generate_contraction_order(&mut self) {
        self.contraction_order.clear();

        for (index, edge) in self.edges.iter().enumerate().filter(|edge| edge.1.is_some()) {
            println!("angle: {}", (&*self, edge.as_ref().unwrap()).dihedral_angle().to_degrees());
            // TODO: Generate a good contraction order
            let index: Index<Edge> = index.into();
            if let Some(order) = (self as &ClosedTriangleMesh<TD>, index.into()).calculate_contraction_order() {
                self.contraction_order.push(index.into(), order);
            }
        }
    }

    fn update_contraction_order<I: Iterator<Item = Index<Edge>>>(&mut self, affected: I) {
        // TODO: correctly recompute contraction order
        for edge in affected {
            if self[edge].is_none() {
                self.contraction_order.remove(&edge);
            } else {
                println!("angle: {}", (&*self, edge).dihedral_angle().to_degrees());
                if let Some(order) = (self as &ClosedTriangleMesh<TD>, edge).calculate_contraction_order() {
                    self.contraction_order.push(edge, order);
                }
            }
        }
    }

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
        Self::collect_outgoing_edges_with_edges_starting_with(edges, node, node.outgoing[0])
    }

    fn collect_outgoing_edges_starting_with(&self, index: Index<Node>, edge: Index<Edge>) -> Vec<Index<Edge>> {
        Self::collect_outgoing_edges_with_edges_starting_with(&self.edges, self[index].as_ref().unwrap(), edge)
    }

    fn collect_outgoing_edges_with_edges_starting_with(edges: &List<Edge>, node: &Node, starting_edge: Index<Edge>) -> Vec<Index<Edge>> {
        let mut edge = Self::next_outgoing_with_edges(&edges, starting_edge);
        let mut outgoing = vec![edge];
        while edge != starting_edge {
            edge = Self::next_outgoing_with_edges(&edges, edge);
            outgoing.push(edge);
        }
        outgoing
    }

    /// Create a view into the structure of the mesh after the edge at `index` is contracted.
    /// This does not change any underlying data such as any attribute of edges, nodes or triangles.
    /// If `SimulatedEdgeCollapse`'s access on any field returns `None` this may be due to a) the original mesh has a `None` value or b) the field will be removed during the contraction. 
    /// Any access to the source of the edge will be mapped to its target.
    pub fn simulate_contract_edge<'m>(&'m self, index: Index<Edge>) -> SimulatedEdgeCollapse<'m, TD, Self> {
        let mut collapsed_edges: HashMap<Index<Edge>, Option<Edge>> = HashMap::new();
        let mut collapsed_nodes: HashMap<Index<Node>, Index<Node>> = HashMap::new();
        let mut collapsed_triangles: HashMap<Index<Triangle>, Option<Triangle>> = HashMap::new();

        let st = self[index].as_ref().cloned().unwrap();
        let ts = self[st.opposite].as_ref().cloned().unwrap();
        let _as = self[st.previous].as_ref().cloned().unwrap();
        let sd = self[ts.next].as_ref().cloned().unwrap();

        collapsed_nodes.insert(st.source, st.target);

        collapsed_edges.insert(index, None);
        collapsed_edges.insert(st.opposite, None);
        collapsed_edges.insert(st.previous, None);
        collapsed_edges.insert(_as.opposite, None);
        collapsed_edges.insert(ts.next, None);
        collapsed_edges.insert(sd.opposite, None);

        collapsed_triangles.insert(st.triangle, None);
        collapsed_triangles.insert(ts.triangle, None);

        SimulatedEdgeCollapse { 
            collapsed_edges, 
            collapsed_nodes, 
            collapsed_triangles, 
            original: &self, 
            _data: PhantomData 
        }
    }
}

impl<TD> ClosedTriangleMesh<TD>
where
    TD: MeshData<CollapseInfoOutput = TriangleMeshHalfEdgeCollapse>,
{
    pub fn save_to_dir(self, dir: &mut std::path::PathBuf, name: &str) {
        if dir.exists() && !dir.is_dir() {
            panic!("{} should be a directory to save to", dir.display());
        }
        if !dir.exists() {
            std::fs::create_dir_all(&dir).unwrap();
        }

        dir.push(name);

        let nodes = dir.clone().also(|dir| { dir.add_extension("ver"); } );
        let nodes_full = dir.clone().also(|dir| { dir.add_extension("ver_full"); } );
        let triangles = dir.clone().also(|dir| { dir.add_extension("tri"); } );
        let edges = dir.clone().also(|dir| { dir.add_extension("edg"); } );
        let map_nodes = dir.clone().also(|dir| { dir.add_extension("ver_map"); } );
        let map_triangles = dir.clone().also(|dir| { dir.add_extension("tri_map"); } );
        let map_edges = dir.clone().also(|dir| { dir.add_extension("edg_map"); } );
        let uncontraction = dir.clone().also(|dir| { dir.add_extension("hec"); } );

        let (idx_nodes, _nodes): (Vec<_>, Vec<_>) = self.nodes.enumerate_some()
            .into_iter()
            .enumerate()
            .map(|(i, (idx, node))| ((i + 1, idx), (idx, node)))
            .unzip();
        let (idx_edges, _edges): (Vec<_>, Vec<_>) = self.edges.enumerate_some()
            .into_iter()
            .enumerate()
            .map(|(i, (idx, edge))| ((i + 1, idx), (idx, edge)))
            .unzip();
        let (idx_triangles, _triangles): (Vec<_>, Vec<_>) = self.triangles.enumerate_some()
            .into_iter()
            .enumerate()
            .map(|(i, (idx, triangle))| ((i + 1, idx), (idx, triangle)))
            .unzip();
        let map_node_idx = move |id: Index<Node>| {
            idx_nodes.iter().find(|(_, idx)| *idx == id).unwrap().0
        };
        let map_edge_idx = move |id: Index<Edge>| {
            idx_edges.iter().find(|(_, idx)| *idx == id).unwrap().0
        };
        let map_triangle_idx = move |id: Index<Triangle>| {
            idx_triangles.iter().find(|(_, idx)| *idx == id).unwrap().0
        };
        _nodes.into_iter()
            .map(|(idx, node)| (idx, node.coordinates))
            .unzip()
            .apply(|(idx, n): (Vec<Index<Node>>, Vec<_>)| {
                std::fs::write(nodes, n.into_iter().map(|it| it.map(|i| std::format!("{i:.15}")).join(",")).join("\n")).unwrap();
                std::fs::write(map_nodes, idx.into_iter().map(|idx| *idx).join("\n")).unwrap();
            });
        std::fs::write(nodes_full, serde_json::to_string_pretty(&self.nodes).unwrap()).unwrap();

        self.triangles.enumerate_some()
            .into_iter()
            .map(|(idx, tri)| (idx, tri.corners))
            .unzip()
            .apply(|(idx, t): (Vec<_>, Vec<_>)| {
                std::fs::write(triangles, t.into_iter().map(|it| it.map(|i| std::format!("{}", map_node_idx(i))).join(",")).join("\n")).unwrap();
                std::fs::write(map_triangles, idx.into_iter().map(|idx| *idx).join("\n")).unwrap();
            });

        self.edges.enumerate_some()
            .into_iter()
            .unzip()
            .apply(|(idx, e): (Vec<_>, Vec<_>)| {
                std::fs::write(edges, e.into_iter().map(|it| std::format!("{},{},{},{},{},{}", map_node_idx(it.source), map_node_idx(it.target), map_edge_idx(it.opposite), map_edge_idx(it.previous), map_edge_idx(it.next), map_triangle_idx(it.triangle))).join("\n")).unwrap();
                std::fs::write(map_edges, idx.into_iter().map(|idx| *idx).join("\n")).unwrap();
            });

        self.undo_contraction.into_iter()
            // .map(|info: TriangleMeshHalfEdgeCollapse| {
            //     TriangleMeshHalfEdgeCollapse {
            //         source: ((*info.source.0 + 1).into(), info.source.1),
            //         target: (*info.target + 1).into(),
            //         facettes: info.facettes.into_iter()
            //             .map(|face| (*face + 1).into())
            //             .collect_vec(),
            //         removed_edges: info.removed_edges.into_iter()
            //             .map(|(idx, edge)| {
            //                 let edge = Edge {
            //                     next: (*edge.next + 1).into(),
            //                     previous: (*edge.previous + 1).into(),
            //                     opposite: (*edge.opposite + 1).into(),
            //                     source: (*edge.source + 1).into(),
            //                     target: (*edge.target + 1).into(),
            //                     triangle: (*edge.triangle + 1).into(),
            //                 };
            //                 (idx, edge)
            //             })
            //             .collect_vec(),
            //         removed_facettes: info.removed_facettes.map(|(idx, face)| {
            //             let face = face.map(|f| {
            //                 Triangle {
            //                     corners: f.corners.map(|c| (*c + 1).into())
            //                 }
            //             });
            //             (idx, face)
            //         } )
            //     }
            // })
            .collect_vec()
            .apply(|it| {
                std::fs::write(uncontraction, serde_json::to_string_pretty(&it).unwrap()).unwrap();
            });
    }

    pub fn restore_from_dir(dir: &mut std::path::PathBuf, name: &str) -> Self {
        dir.push(name);

        let nodes = dir.clone().also(|dir| { dir.add_extension("ver_full"); } );
        let triangles = dir.clone().also(|dir| { dir.add_extension("tri"); } );
        let edges = dir.clone().also(|dir| { dir.add_extension("edg"); } );
        let map_nodes = dir.clone().also(|dir| { dir.add_extension("ver_map"); } );
        let map_triangles = dir.clone().also(|dir| { dir.add_extension("tri_map"); } );
        let map_edges = dir.clone().also(|dir| { dir.add_extension("edg_map"); } );
        let uncontraction: Vec<TriangleMeshHalfEdgeCollapse> = serde_json::from_slice(&std::fs::read(dir.clone().also(|dir| { dir.add_extension("hec"); } )).unwrap()).unwrap();

        let map_nodes = csv::ReaderBuilder::new().has_headers(false)
            .from_path(map_nodes)
            .unwrap()
            .deserialize()
            .into_iter()
            .map(|c| c.unwrap())
            .enumerate()
            .map(|(i, idx): (usize, usize)| {
                (i, idx.into()) as (_, Index<Node>)
            })
            .collect_vec();
        let map_edges = csv::ReaderBuilder::new().has_headers(false)
            .from_path(map_edges)
            .unwrap()
            .deserialize()
            .into_iter()
            .map(|c| c.unwrap())
            .enumerate()
            .map(|(i, idx): (usize, usize)| {
                (i, idx.into()) as (_, Index<Edge>)
            })
            .collect_vec();
        let map_triangles = csv::ReaderBuilder::new().has_headers(false)
            .from_path(map_triangles)
            .unwrap()
            .deserialize()
            .into_iter()
            .map(|c| c.unwrap())
            .enumerate()
            .map(|(i, idx): (usize, usize)| {
                (i, idx.into()) as (_, Index<Triangle>)
            })
            .collect_vec();

        let nodes: List<Node> = serde_json::from_slice(&std::fs::read(nodes).unwrap()).unwrap();
        // let nodes = csv::ReaderBuilder::new().has_headers(false)
        //     .from_path(nodes)
        //     .unwrap()
        //     .deserialize()
        //     .into_iter()
        //     .map(|c| c.unwrap())
        //     .enumerate()
        //     .map(|(idx, coordinates): (usize, [f64; 3])| {
        //         (map_nodes[idx].1, Node::new(coordinates))
        //     })
        //     .collect_vec();
        let edges = csv::ReaderBuilder::new().has_headers(false)
            .from_path(edges)
            .unwrap()
            .deserialize()
            .into_iter()
            .map(|c| c.unwrap())
            .enumerate()
            .map(|(idx, (source, target, opposite, previous, next, triangle)): (usize, (usize, usize, usize, usize, usize, usize))| {
                (map_edges[idx].1, Edge {
                    source: map_nodes[source - 1].1,
                    target: map_nodes[target - 1].1,
                    opposite: map_edges[opposite - 1].1,
                    previous: map_edges[previous - 1].1,
                    next: map_edges[next - 1].1,
                    triangle: map_triangles[triangle - 1].1,
                })
            })
            .collect_vec();
        let triangles = csv::ReaderBuilder::new().has_headers(false)
            .from_path(triangles)
            .unwrap()
            .deserialize()
            .into_iter()
            .map(|c| c.unwrap())
            .enumerate()
            .map(|(idx, (a,b,c)): (usize, (usize, usize, usize))| {
                (map_triangles[idx].1, Triangle { corners: [a,b,c].map(|i| map_nodes[i - 1].1) })
            })
            .collect_vec();

        // let nodes = {
        //     let max = nodes.len() + uncontraction.len();
        //     let mut list = List::with_defaults(max);
        //     nodes.into_iter().for_each(|(idx, node)| { list[idx] = Some(node) });
        //     list
        // };
        let edges = {
            let max = edges.len() + 6 * uncontraction.len();
            let mut list = List::with_defaults(max);
            edges.into_iter().for_each(|(idx, edge)| { list[idx] = Some(edge) });
            list
        };
        let triangles = {
            let max = triangles.len() + 2 * uncontraction.len();
            let mut list = List::with_defaults(max);
            triangles.into_iter().for_each(|(idx, triangle)| { list[idx] = Some(triangle) });
            list
        };
        let undo_contraction = uncontraction.into_iter()
            // .map(|info| {
            //     TriangleMeshHalfEdgeCollapse {
            //         source: ((*info.source.0 - 1).into(), info.source.1),
            //         target: (*info.target - 1).into(),
            //         facettes: info.facettes.into_iter()
            //             .map(|face| (*face - 1).into())
            //             .collect_vec(),
            //         removed_edges: info.removed_edges.into_iter()
            //             .map(|(idx, edge)| {
            //                 let edge = Edge {
            //                     next: (*edge.next - 1).into(),
            //                     previous: (*edge.previous - 1).into(),
            //                     opposite: (*edge.opposite - 1).into(),
            //                     source: (*edge.source - 1).into(),
            //                     target: (*edge.target - 1).into(),
            //                     triangle: (*edge.triangle - 1).into(),
            //                 };
            //                 (idx, edge)
            //             })
            //             .collect_vec(),
            //         removed_facettes: info.removed_facettes.map(|(idx, face)| {
            //             let face = face.map(|f| {
            //                 Triangle {
            //                     corners: f.corners.map(|c| (*c - 1).into())
            //                 }
            //             });
            //             (idx, face)
            //         } )
            //     }
            // })
            .collect();

        let mut mesh = Self {
            contraction_order: DoublePriorityQueue::with_capacity(edges.len()),
            triangle_data: List::with_capacity(triangles.len()),
            nodes,
            edges,
            triangles,
            undo_contraction
        };
        // mesh.generate_contraction_order();
        mesh
    }
}

impl<TD> ContractableMesh for ClosedTriangleMesh<TD>
where 
    TD: MeshData<CollapseInfoInput = TriangleMeshHalfEdgeCollapse>,
{
    fn contract_edge(&mut self, index: Index<Edge>) -> &TD::CollapseInfoOutput {

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
        for outgoing in outgoing.clone() {
            let index = self[outgoing].as_ref().unwrap().triangle;

            // Remember all indices of facettes which had s as corner but not t.
            // Those are later needed to uncontract the mesh.
            if index != st.triangle && index != ts.triangle {
                facettes.push(index)
            } 

            let facette = self[index].as_mut().unwrap();
            debug!("HEC: Triangle: {facette:?}");
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
        
        let st_facette = std::mem::take(&mut self[st.triangle]);
        let ts_facette = std::mem::take(&mut self[ts.triangle]);
        
        let ta = st.next;
        let at = self[ta].as_ref().unwrap().opposite;
        let dt = ts.previous;
        let td = self[dt].as_ref().unwrap().opposite;
    
        let _as = std::mem::take(&mut self[st.previous]).unwrap();
        let sa = std::mem::take(&mut self[_as.opposite]).unwrap();

        let sd = std::mem::take(&mut self[ts.next]).unwrap();
        let ds = std::mem::take(&mut self[sd.opposite]).unwrap();

        let (ab, bs, facette_sa) = (sa.next, sa.previous, sa.triangle);
        let (sc, cd, facette_ds) = (ds.next, ds.previous, ds.triangle);

        self[ab].as_mut().map(|edge| edge.previous = ta);
        self[ab].as_mut().map(|edge| edge.previous = ta);
        self[bs].as_mut().map(|edge| edge.next = ta);
        
        let previous = if self[sa.previous].as_ref().is_some() { sa.previous } else { dt };
        self[ta].as_mut().map(|edge| edge.next = ab);
        self[ta].as_mut().map(|edge| edge.previous = previous);
        self[ta].as_mut().map(|edge| edge.triangle = facette_sa);

        let (next, previous) = if let Some(edge) = self[ds.next].as_ref() {
            (ds.next, edge.next)
        } else {
            let t_outgoing = self[st.target].as_ref().unwrap().outgoing.clone().also(|it| {
                let rotate_by = it.iter().enumerate().find(|(_, edge)| **edge == td).unwrap().0;
                it.rotate_left(rotate_by);
                it.remove(0);
            });

            debug!("Looking at edge: {index}. Searching dt: {dt}, so td: {td}");
            debug!("Outgoing at t: {t_outgoing:?}");

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
        self[dt].as_mut().map(|edge| edge.triangle = facette_ds);

        // Rescan all nodes who had at least one edge removed
        self[ts.source].as_mut().unwrap().outgoing = self.collect_outgoing_edges_starting_with(ts.source, td);
        self[sa.target].as_mut().unwrap().outgoing = self.collect_outgoing_edges_starting_with(sa.target, at);
        self[sd.target].as_mut().unwrap().outgoing = self.collect_outgoing_edges_starting_with(sd.target, dt);

        let info = TriangleMeshHalfEdgeCollapse {
            source: (st.source, source.clone()),
            target: st.target,
            facettes,
            removed_facettes: [(st.triangle, st_facette), (ts.triangle, ts_facette)],
            removed_edges: vec![(_as.opposite, sa), (st.previous, _as), (sd.opposite, ds), (ts.next, sd), (st.opposite, ts), (index, st)],
        };
        let info = TD::contract_edge(self, info);

        let removed_edges = &info.removed_edges();
        let affected_edges = outgoing
            .into_iter()
            .filter_map(|idx| self[idx].as_ref().map(|edge| [idx, edge.opposite]))
            .flatten()
            .collect_vec()
            .into_iter()
            .chain(removed_edges.iter().map(|(i,_)| *i));
        self.update_contraction_order(affected_edges);
        self.undo_contraction.push(info);
        &self.undo_contraction.last().unwrap()
    }

    fn contract_next_edge(&mut self) -> &TD::CollapseInfoOutput {
        let next = self.contraction_order.pop_min();
        self.contract_edge(next.unwrap().0)
    }
}

impl<TD> UncontractableMesh for ClosedTriangleMesh<TD> 
where 
    TD: MeshData<CollapseInfoInput = TriangleMeshHalfEdgeCollapse>,
{
    fn uncontract_edge(&mut self, info: TD::CollapseInfoOutput) {
        let source = info.source(); 
        let target = info.target(); 
        let facettes = info.facettes(); 
        let removed_edges = info.removed_edges(); 
        let removed_facettes = info.removed_facettes();

        let (idx_source, source) = source;
        let (idx_st, st) = removed_edges.last().as_deref().cloned().unwrap();

        let removed_edge_indices = removed_edges.iter().map(|(i, _)| i).cloned().collect::<Vec<_>>();
        for (index, edge) in removed_edges {
            self[index] = Some(edge);
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
            it.triangle = st.triangle;
        });
        let ts = self[st.opposite].as_ref().cloned().unwrap();
        self[ts.previous].as_mut().unwrap().apply(|it| {
            it.next = st.opposite;
            it.previous = ts.next;
            it.triangle = ts.triangle;
        });

        for index in facettes {
            let facette = self[index].as_mut().unwrap();
            for corner in &mut facette.corners {
                if *corner == target {
                    *corner = idx_source
                } 
            }
        }

        // Restore the next and previous relations
        let indices = (0..(source.outgoing.len())).collect::<Vec<_>>().also(|it| it.rotate_left(1));
        for i in 0..(source.outgoing.len()) {
            let edge = source.outgoing[i];
            let opposite = self[edge].as_ref().unwrap().opposite;
            self[opposite].as_mut().unwrap().next = source.outgoing[indices[i]];
            self[source.outgoing[indices[i]]].as_mut().unwrap().previous = opposite;
            let (a, next, previous, facette) = self[edge].as_ref().unwrap().apply(|it| (it.source, it.next, it.previous, it.triangle));
            let (b, c) = self[next].as_mut().unwrap().apply(|it| {
                it.next = previous;
                it.previous = edge;
                it.triangle = facette;
                (it.source, it.target)
            });
            self[facette].as_mut().unwrap().corners = [a, b, c];
        }
        for removed_edge in &removed_edge_indices {
            self.previous_edge_mut(*removed_edge).unwrap().next = *removed_edge;
            self.next_edge_mut(*removed_edge).unwrap().previous = *removed_edge;
        }
        let (ta, a) = (st.next, self[st.next].as_ref().unwrap().target);
        let (dt, d) = self[st.opposite].as_ref().unwrap().apply(|it| (it.previous, self[it.previous].as_ref().unwrap().source));
        // self[ts.source].as_mut().unwrap().outgoing = self.collect_outgoing_edges_starting_with(ts.source, self[self[ta].as_ref().unwrap().opposite].as_ref().unwrap().next);
        self[ts.source].as_mut().unwrap().outgoing = self.collect_outgoing_edges_starting_with(ts.source, ta);
        self[a].as_mut().unwrap().outgoing = self.collect_outgoing_edges_starting_with(a, self[ta].as_ref().unwrap().opposite);
        self[d].as_mut().unwrap().outgoing = self.collect_outgoing_edges_starting_with(d, dt);

        let affected_edges = source.outgoing
            .into_iter()
            .filter_map(|idx| self[idx].as_ref().map(|edge| [idx, edge.opposite]))
            .flatten()
            .collect_vec()
            .into_iter()
            .chain(removed_edge_indices.into_iter());
        self.update_contraction_order(affected_edges);
    }

    fn uncontract_next_edge<F: FnOnce(&Self, &<<Self as TriangleMesh>::Data as MeshData>::CollapseInfoOutput)>(&mut self, before: F) {
        let info = self.undo_contraction.pop().unwrap();
        before(self, &info);
        self.uncontract_edge(info.clone());
    }
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

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
                assert_eq!(ClosedTriangleMesh::<()>::next_outgoing_with_edges(&mesh.edges, edge), node.outgoing[(index + 1) % node.outgoing.len()]);
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

            assert_eq!(ab.triangle, bc.triangle);
            assert_eq!(bc.triangle, ca.triangle);
        }

        Ok(())
    }

    #[test]
    fn tetraeder_hec_undo() {
        let mut mesh = tetraeder();
        let copy = mesh.clone();
        let contraction = mesh.contract_edge(0.into()).clone();
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

            assert_eq!(ab.triangle, bc.triangle);
            assert_eq!(bc.triangle, ca.triangle);

            let facette = mesh[ab.triangle].as_ref().unwrap();
            assert_eq!(facette.corners.len(), 3);
            for edge in [ab, bc, ca] {
                assert!(facette.corners.contains(&edge.source))
            }
        }

        // Check that each edge has the correct opposite edge
        for [edge, opposite] in [[2.into(), 3.into()], [14.into(), 15.into()], [18.into(), 19.into()]] as [[Index<Edge>; 2]; 3] {
            assert_eq!(mesh[edge].as_ref().unwrap().opposite, opposite);
            assert_eq!(mesh[opposite].as_ref().unwrap().opposite, edge);
        }

        Ok(())
    }

    #[test]
    fn cube_hec_undo() {
        let mut mesh = cube();
        let copy = mesh.clone();
        let contraction = mesh.contract_edge(0.into()).clone();
        mesh.uncontract_edge(contraction.clone());
        assert_eq!(copy.nodes.len(), mesh.nodes.len());
        assert_eq!(copy.edges.len(), mesh.edges.len());
        assert_eq!(copy.triangles.len(), mesh.triangles.len());
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
            assert_eq!(orig.next, copy.next, "edges are different at {index}: next: {}, {}", orig.next, copy.next);
            assert_eq!(orig.previous, copy.previous, "edges are different at {index}: previous: {}, {}", orig.previous, copy.previous);
            assert_eq!(orig.opposite, copy.opposite, "edges are different at {index}: opposite: {}, {}", orig.opposite, copy.opposite);
            assert_eq!(orig.triangle, copy.triangle, "edges are different at {index}: facette: {}, {}", orig.triangle, copy.triangle);
        }
        assert_eq!(copy.triangles, mesh.triangles);
    }

    #[test]
    fn cube_hec_redo() {
        let mut mesh = cube();
        let copy = mesh.clone();
        let contraction = mesh.contract_edge(0.into()).clone();
        mesh.uncontract_edge(contraction.clone());
        let other_contraction = mesh.contract_edge(0.into()).clone();
        assert_eq!(contraction, other_contraction);
        mesh.uncontract_edge(other_contraction.clone());
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
            assert_eq!(orig.next, copy.next, "edges are different at {index}: next: {}, {}", orig.next, copy.next);
            assert_eq!(orig.previous, copy.previous, "edges are different at {index}: previous: {}, {}", orig.previous, copy.previous);
            assert_eq!(orig.opposite, copy.opposite, "edges are different at {index}: opposite: {}, {}", orig.opposite, copy.opposite);
            assert_eq!(orig.triangle, copy.triangle, "edges are different at {index}: facette: {}, {}", orig.triangle, copy.triangle);
        }
        assert_eq!(copy.triangles, mesh.triangles);
    }

    #[test]
    fn cube_next_hec_twice() {
        let mut mesh = cube();
        let copy = mesh.clone();
        let first = mesh.contract_next_edge();
        let next = mesh.contraction_order.peek_min();
        println!("next: {:?}", mesh.contraction_order.peek_min());
        let second = mesh.contract_next_edge();
        mesh.uncontract_next_edge(|_, _| {});
        mesh.uncontract_next_edge(|_, _| {});
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
            assert_eq!(orig.next, copy.next, "edges are different at {index}: next: {}, {}", orig.next, copy.next);
            assert_eq!(orig.previous, copy.previous, "edges are different at {index}: previous: {}, {}", orig.previous, copy.previous);
            assert_eq!(orig.opposite, copy.opposite, "edges are different at {index}: opposite: {}, {}", orig.opposite, copy.opposite);
            assert_eq!(orig.triangle, copy.triangle, "edges are different at {index}: facette: {}, {}", orig.triangle, copy.triangle);
        }
        assert_eq!(copy.triangles, mesh.triangles);
    }


    #[test]
    fn cube_hec() -> Result<(), MeshBuilderError> {
        let mut mesh = cube();
        let contraction = mesh.contract_edge(0.into()).clone();

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

    #[test]
    fn save_restore_cube() {
        let mesh = cube();
        mesh.clone().save_to_dir(&mut PathBuf::from("./test_output/"), "orbifold");
        let copy = ClosedTriangleMesh::<()>::restore_from_dir(&mut PathBuf::from("./test_output/"), "orbifold");
        
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
        for (index, (copy, orig)) in copy.edges.iter().zip(mesh.edges.iter()).enumerate() {
            assert_eq!(orig.is_some(), copy.is_some(), "edges are different at {index}");
            let (orig, copy) = (orig.clone().unwrap(), copy.clone().unwrap());
            assert_eq!(orig, copy, "edges are different at {index}");
            assert_eq!(orig.next, copy.next, "edges are different at {index}: next: {}, {}", orig.next, copy.next);
            assert_eq!(orig.previous, copy.previous, "edges are different at {index}: previous: {}, {}", orig.previous, copy.previous);
            assert_eq!(orig.opposite, copy.opposite, "edges are different at {index}: opposite: {}, {}", orig.opposite, copy.opposite);
            assert_eq!(orig.triangle, copy.triangle, "edges are different at {index}: facette: {}, {}", orig.triangle, copy.triangle);
        }
        assert_eq!(copy.triangles, mesh.triangles);
    }

    #[test]
    fn save_restore_cube_hec() {
        let mut mesh = cube();
        mesh.contract_edge(0.into());
        mesh.clone().save_to_dir(&mut PathBuf::from("./test_output/"), "orbifold");
        let copy = ClosedTriangleMesh::<()>::restore_from_dir(&mut PathBuf::from("./test_output/"), "orbifold");
        
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
        for (index, (copy, orig)) in copy.edges.iter().zip(mesh.edges.iter()).enumerate() {
            assert_eq!(orig.is_some(), copy.is_some(), "edges are different at {index}");
            if orig.is_some() {
                let (orig, copy) = (orig.clone().unwrap(), copy.clone().unwrap());
                assert_eq!(orig, copy, "edges are different at {index}");
                assert_eq!(orig.next, copy.next, "edges are different at {index}: next: {}, {}", orig.next, copy.next);
                assert_eq!(orig.previous, copy.previous, "edges are different at {index}: previous: {}, {}", orig.previous, copy.previous);
                assert_eq!(orig.opposite, copy.opposite, "edges are different at {index}: opposite: {}, {}", orig.opposite, copy.opposite);
                assert_eq!(orig.triangle, copy.triangle, "edges are different at {index}: facette: {}, {}", orig.triangle, copy.triangle);
            }
        }
        assert_eq!(copy.triangles, mesh.triangles);
    }
}