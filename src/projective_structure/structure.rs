use std::path::PathBuf;

use apply::{Also, Apply};
use itertools::Itertools;
use nalgebra::{Const, DMatrix, DVector, Dyn, Matrix3, Matrix4, Matrix4x3, SMatrix, SVector, Vector, Vector3, Vector4};

use crate::{mesh::{UncontractableMesh, Edge, Index, List, MeshCollapseInfo, MeshData, Node, Triangle, TriangleMesh}, ClosedTriangleMesh};

use super::{visualisation::VisualiseProjectiveStructure, CEquationParameters, CEquations, Quad};

#[derive(Clone)]
pub struct ProjectiveStructure<TM: UncontractableMesh> {
    mesh: TM,
    nodes: List<Node>,
    pub coefficients: List<SVector<f64, 4>, Edge>
}

impl<'M, TM: UncontractableMesh + 'M> core::ops::Index<Index<Node>> for ProjectiveStructure<TM> {
    type Output = <List<Node> as core::ops::Index<Index<Node>>>::Output;

    fn index(&self, index: Index<Node>) -> &Self::Output {
        &self.nodes[index]
    }
}

impl<'M, TM: UncontractableMesh + 'M> core::ops::IndexMut<Index<Node>> for ProjectiveStructure<TM> {
    fn index_mut(&mut self, index: Index<Node>) -> &mut Self::Output {
        &mut self.nodes[index]
    }
}

impl<'M, TM: UncontractableMesh + 'M> core::ops::Index<Index<Edge>> for ProjectiveStructure<TM> {
    type Output = <List<Edge> as core::ops::Index<Index<Edge>>>::Output;

    fn index(&self, index: Index<Edge>) -> &Self::Output {
        &self.mesh[index]
    }
}

impl<'M, TM: UncontractableMesh + 'M> core::ops::Index<Index<Triangle>> for ProjectiveStructure<TM> {
    type Output = <List<Triangle> as core::ops::Index<Index<Triangle>>>::Output;

    fn index(&self, index: Index<Triangle>) -> &Self::Output {
        &self.mesh[index]
    }
}

impl<'M, TM: UncontractableMesh + 'M> TriangleMesh for ProjectiveStructure<TM> {
    type Data = <TM as TriangleMesh>::Data;

    fn nodes(&self) -> &List<Node> {
        &self.nodes
    }

    fn current_nodes(&self) -> Vec<(Index<Node>, Node)> {
        self.mesh.current_nodes().into_iter().map(|(index, _)| (index, self[index].as_ref().unwrap().clone())).collect_vec()
    }

    fn edges(&self) -> &List<Edge> {
        &self.mesh.edges()
    }

    fn current_edges(&self) -> Vec<(Index<Edge>, Edge)> {
        self.mesh.current_edges()
    }

    fn current_edges_undirected(&self) -> Vec<(Index<Edge>, Edge)> {
        self.mesh.current_edges()
    }

    fn triangles(&self) -> &List<Triangle> {
        self.mesh.triangles()
    }

    fn current_triangles(&self) -> Vec<(Index<Triangle>, Triangle)> {
        self.mesh.current_triangles()
    }

    fn triangle_data(&self) -> &List<Self::Data, Triangle> {
        self.mesh.triangle_data()
    }

    fn build_mesh(&self) -> crate::mesh::MeshTriangleInfo {
        Self::visualise_coefficients(&self)
    }
}

impl<'M, TM: UncontractableMesh + 'M> UncontractableMesh for ProjectiveStructure<TM> {
    fn uncontract_edge(&mut self, contraction: <<Self as TriangleMesh>::Data as MeshData>::CollapseInfoOutput) {
        let source = contraction.source();
        let target = contraction.target();
         
        let triangle = ProjectiveStructure::<TM>::find_triangle(&self.mesh, source.1.coordinates, target).unwrap();
        let [a,b,c] = self.mesh[triangle].as_ref().unwrap().corners.map(|c| (c, self.mesh[c].as_ref().unwrap().coordinates));
        let edge = self.mesh.find_edge(a.0, b.0).unwrap();

        let (a, c, prev) = self.mesh[edge].as_ref().unwrap().apply(|it| (it.source, it.target, it.previous));
        let b = self.mesh[prev].as_ref().unwrap().source;
        let bar = ProjectiveStructure::<TM>::barycentric_coordinates_orig(&self.mesh, edge, source.1.coordinates);
        let translated_source: Vector3<f64> = self.from_barycentric(bar, [a,b,c]).apply(|coordinates| {
            self.nodes[source.0] = Some(Node { coordinates, outgoing: source.1.outgoing });
            coordinates.into()
        });

        self.mesh.uncontract_edge(contraction);

        // TODO: start with edge opposite of s of deleted face, recalculate conditions using new source, keep attention on choosing first edge correct
        // then go around in a circle and update next and opposite edge of s at face
        // And of course, ensure that faces are valid, nodes updated and uncontract original mesh.


        // Recalculate the coefficients for each edge based on the projective structure.
        // This will automatically delete the old coefficients of the edge and its opposite.
        let st = self.mesh.find_edge(source.0, target).unwrap();
        let mut index = st;

        loop {
            let (next, opposite) = self.mesh[index].as_ref().unwrap().apply(|edge| (edge.next, edge.opposite));
            self.recalculate_coefficients(self.mesh[next].as_ref().unwrap().opposite, &translated_source);

            let _index = self.mesh[opposite].as_ref().unwrap().next;
            let d = self.nodes[self.mesh[_index].as_ref().unwrap().target].as_ref().unwrap();
            self.recalculate_coefficients(index, &d.coordinates.into());

            index = _index;

            if index == st {
                break;
            }
        }
    }

    fn uncontract_next_edge<F: FnOnce(&Self, &<<Self as TriangleMesh>::Data as MeshData>::CollapseInfoOutput)>(&mut self, before: F) {
        let mut source = None;
        let mut target= None; 
        let mut bar= None;
        let mut edge = None;
        let mut basis = None;

        self.mesh.uncontract_next_edge(|mesh, contraction| {
            let _source = contraction.source();
            let _target = contraction.target();
             
            let triangle = ProjectiveStructure::<TM>::find_triangle(mesh, _source.1.coordinates, _target).unwrap();
            let [a,b,c] = mesh[triangle].as_ref().unwrap().corners.map(|c| (c, mesh[c].as_ref().unwrap().coordinates));
            let _edge = mesh.find_edge(a.0, b.0).unwrap();

            let (a, c, prev) = mesh[_edge].as_ref().unwrap().apply(|it| (it.source, it.target, it.previous));
            let b = mesh[prev].as_ref().unwrap().source;
            basis = Some([a, b, c]);
            println!("b: {b}");
            println!("b: {:?}", mesh[b].as_ref().unwrap().coordinates);
            bar = Some(ProjectiveStructure::<TM>::barycentric_coordinates_orig(mesh, _edge, _source.1.coordinates));
            source = Some(_source);
            target = Some(_target);
            edge = Some(_edge)
        });

        let source = source.unwrap();
        let target = target.unwrap();
        let edge = edge.unwrap();
        let bar = bar.unwrap();
        let basis = basis.unwrap();
        println!("b: {:?}", self.mesh[basis[1]].as_ref().unwrap().coordinates);
        let translated: Vector3<f64> = self.from_barycentric(bar, basis).apply(|coordinates| {
            self.nodes[source.0] = Some(Node { coordinates, outgoing: source.1.outgoing });
            coordinates.into()
        });
        println!("translated: {translated}, bar: {bar}");

        // TODO: start with edge opposite of s of deleted face, recalculate conditions using new source, keep attention on choosing first edge correct
        // then go around in a circle and update next and opposite edge of s at face
        // And of course, ensure that faces are valid, nodes updated and uncontract original mesh.


        // Recalculate the coefficients for each edge based on the projective structure.
        // This will automatically delete the old coefficients of the edge and its opposite.
        let st = self.mesh.find_edge(source.0, target).unwrap();
        let mut index = st;

        loop {
            let (next, opposite) = self.mesh[index].as_ref().unwrap().apply(|edge| (edge.next, edge.opposite));
            self.recalculate_coefficients(self.mesh[next].as_ref().unwrap().opposite, &translated);

            let _index = self.mesh[opposite].as_ref().unwrap().next;
            let d = self.nodes[self.mesh[_index].as_ref().unwrap().target].as_ref().unwrap();
            self.recalculate_coefficients(index, &d.coordinates.into());

            index = _index;

            if index == st {
                break;
            }
        }
    }
}

impl<'M, TM: UncontractableMesh + Clone> ProjectiveStructure<TM> {
    pub fn new(equations: CEquations<'M, TM>) -> Self {
        let mut coefficients = List::with_defaults(equations.mesh.edges().len());
        let mut nodes = List::with_defaults(equations.mesh.nodes().len());
        let mut triangles = List::with_defaults(equations.mesh.triangles().len());
        equations.mapping
            .into_iter()
            .enumerate()
            .for_each(|(i, (edge,_))| {
                coefficients[edge] = Some(equations.parameters.coefficients[i])
            });
        equations.mesh.current_nodes()
            .into_iter()
            .for_each(|(index, node)| nodes[index] = Some(node));
        equations.mesh.current_triangles()
            .into_iter()
            .for_each(|(index, triangle)| triangles[index] = Some(triangle.corners));
        Self {
            mesh: equations.mesh.clone(),
            coefficients,
            nodes,
        }
    }

    pub fn replace(&mut self, parameters: CEquationParameters, mapping: &Vec<(Index<Edge>, Index<Edge>)>) {
        let mut coefficients = List::with_defaults(self.mesh.edges().len());
        let mut nodes = List::with_defaults(self.mesh.nodes().len());
        let mut triangles = List::with_defaults(self.mesh.triangles().len());
        mapping
            .into_iter()
            .enumerate()
            .for_each(|(i, (edge,_))| {
                coefficients[*edge] = Some(parameters.coefficients[i])
            });
        self.mesh.current_nodes()
            .into_iter()
            .for_each(|(index, node)| nodes[index] = Some(node));
        self.mesh.current_triangles()
            .into_iter()
            .for_each(|(index, triangle)| triangles[index] = Some(triangle.corners));
        self.nodes = nodes;
        self.coefficients = coefficients;
    }
}

impl ProjectiveStructure<ClosedTriangleMesh> {
    pub fn save_to_dir(self, dir: &mut PathBuf, name: &str) {
        
        let (idx_nodes, _nodes): (Vec<_>, Vec<_>) = self.mesh.nodes.enumerate_some()
            .into_iter()
            .enumerate()
            .map(|(i, (idx, node))| ((i + 1, idx), (idx, node)))
            .unzip();
        let map_node_idx = move |id: Index<Node>| {
            idx_nodes.iter().find(|(_, idx)| *idx == id).unwrap().0
        };
        let cof = dir.clone().also(|dir| { 
            dir.push(name);
            dir.add_extension("cof"); 
        } );
        println!("saving cof: {}", cof.display());

        self.mesh.current_edges_undirected()
            .into_iter()
            .map(|(idx, edge)| {
                let [alpha, beta, gamma, _] = self.get_coefficients(idx).data.0[0];
                let b = self.mesh[edge.previous].as_ref().unwrap().source;
                (map_node_idx(edge.source), map_node_idx(b), map_node_idx(edge.target), alpha, beta, gamma)
            })
            .map(|(a,b,c, alpha, beta, gamma)| 
                std::format!("{a},{b},{c},{alpha:.15},{beta:.15},{gamma:.15}")
            )
            .join("\n")
            .apply(|content| {
                std::fs::write(cof, content).unwrap();
            });

        self.mesh.save_to_dir(dir, name);
    }

    pub fn restore_from_dir(dir: &mut PathBuf, name: &str) -> Self {
        let mesh = ClosedTriangleMesh::restore_from_dir(dir, name);
        let map_nodes = dir.clone().also(|dir| { dir.add_extension("ver_map"); } );
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
        let coefficients = dir.clone().also(|dir| { dir.add_extension("cof"); } );
        let coefficients = csv::ReaderBuilder::new().has_headers(false)
            .from_path(coefficients)
            .unwrap()
            .deserialize()
            .into_iter()
            .map(|c| c.unwrap())
            .map(|(a,b, c,alpha,beta,gamma): (usize, usize, usize, f64, f64, f64)| {
                let [a, b, c]: [Index<Node>; 3] = [a, b, c].map(|i| (*map_nodes[i - 1].1).into());
                let edge = mesh.find_edge(a, c).unwrap();
                if mesh[mesh[edge].as_ref().unwrap().previous].as_ref().unwrap().source == b {
                    // The given coefficient triple matches the given triangle and is not for the opposite
                    // Should always be the case
                    (edge, SVector::<f64, 4>::new(alpha, beta, gamma, -1f64)) 
                } else {
                    // panic!("False parameters; Did not match expected order")
                    (mesh[edge].as_ref().unwrap().opposite, SVector::<f64, 4>::new(alpha, beta, gamma, -1f64)) 
                }
            })
            .collect_vec();
        let coefficients = List::with_defaults(mesh.edges.len()).also(|list| {
            coefficients.into_iter()
                .for_each(|(idx, param)| list[idx] = Some(param));
            // Make sure that every half-edge has the coefficents calculated
            mesh.edges.enumerate_some().for_each(|(idx, _)| {
                list[idx] = Some(Self::get_coefficients_using(&list, &mesh, idx));
            });
        });

        Self::from_bare(coefficients, mesh)
    }

    pub fn restore_from_dir_by_cp(dir: &mut PathBuf, name: &str, dg: u8) -> Self {
        let mesh = ClosedTriangleMesh::restore_from_dir(dir, name);
        let map_nodes = dir.clone().also(|dir| { dir.add_extension("ver_map"); } );
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
        let controlpoints = dir.clone().also(|dir| { dir.add_extension("cp"); } );
        let weights = dir.clone().also(|dir| { dir.add_extension("w"); } );
        let indices = dir.clone().also(|dir| { dir.add_extension("ind"); } );
        
        let controlpoints: Vec<(f64, f64, f64)> = csv::ReaderBuilder::new().has_headers(false)
            .from_path(controlpoints)
            .unwrap()
            .deserialize()
            .into_iter()
            .map(|c| c.unwrap())
            .collect_vec();
        let weights: Vec<f64> = csv::ReaderBuilder::new().has_headers(false)
            .from_path(weights)
            .unwrap()
            .deserialize()
            .into_iter()
            .map(|c| c.unwrap())
            .map(|c: (f64, f64, f64)| c.0)
            .collect_vec();
        let indices = csv::ReaderBuilder::new().has_headers(false)
            .from_path(indices)
            .unwrap()
            .deserialize()
            .into_iter()
            .map(|c| c.unwrap())
            .map(|(s, t, u, a, b, c, d): (usize, usize, usize, usize, usize, usize, usize)| {
                let [s, t, u]: [Index<Node>; 3] = [s, t, u].map(|i| (*map_nodes[i - 1].1).into());
                let edge = mesh.find_edge(s, t).unwrap();
                if u == mesh[mesh[edge].as_ref().unwrap().previous].as_ref().unwrap().source {
                    (edge, [a - 1, b - 1, c - 1, d - 1])
                } else {
                    (mesh[edge].as_ref().unwrap().opposite, [a - 1, b - 1, c - 1, d - 1])
                    // (mesh[edge].as_ref().unwrap().opposite, [c - 1, d - 1, a - 1, b - 1])
                }
            })
            .chunk_by(|element| element.0)
            .into_iter()
            .map(|(idx, data)| {
                let data = data.into_iter()
                    .map(|it| it.1)
                    .collect_vec();
                (idx, data)
            })
            .collect_vec();

        let coefficients = List::with_defaults(mesh.edges.len()).also(|list| {
            indices.into_iter()
                .for_each(|(edge, indices)| {
                    let (matrices, ds): (Vec<_>, Vec<_>) = indices.iter()
                        // .map(|ind| ind.map(|i| controlpoints[i].apply(|it| [weights[i], it.0 * weights[i], it.1 * weights[i], it.2 * weights[i]])))
                        .map(|ind| ind.map(|i| controlpoints[i].apply(|it| [1f64, it.0, it.1, it.2])))
                        .map(|[a, b, c, d]| (Matrix4x3::from_column_slice([a,b,c].as_flattened()), d))
                        .unzip();
                    let matrix = nalgebra::Matrix::<f64, Dyn, Const<3>, _>::from_row_iterator(matrices.len() * 4, matrices.into_iter().flat_map(|m| m.row_iter().flatten().cloned().collect_vec()));
                    let coefficent: SVector<f64, 3> = matrix.svd(true, true).solve(&DVector::from_column_slice(ds.as_flattened()), 1e-9).unwrap();
                    list[edge] = Some([coefficent[0],coefficent[1], coefficent[2], -1f64].into())
                });
            // Make sure that every half-edge has the coefficents calculated
            mesh.edges.enumerate_some().for_each(|(idx, _)| {
                list[idx] = Some(Self::get_coefficients_using(&list, &mesh, idx));
            });
        });

        Self::from_bare(coefficients, mesh)
    }
}

impl<'M, TM: UncontractableMesh + 'M>  ProjectiveStructure<TM>{

    pub fn from_bare(coefficients: List<SVector<f64, 4>, Edge>, mesh: TM) -> Self {
        let mut nodes = List::with_defaults(mesh.nodes().len());

        let mut triangles = List::<[(Index<Node>, Vector3<f64>); 3], Triangle>::with_defaults(mesh.triangles().len());

        fn generate_triangles<TM: TriangleMesh>(mesh: &TM, coefficients: &List<SVector<f64, 4>, Edge>, triangle: Index<Triangle>, mut triangles: List<[(Index<Node>, Vector3<f64>); 3], Triangle>) -> List<[(Index<Node>, Vector3<f64>); 3], Triangle> {
            let [a, b, c] = triangles[triangle].as_ref().unwrap().map(|(i, n)| (i, n));
            
            let (index, edge) = mesh.find_edge(a.0, b.0).unwrap().apply(|it| (it, mesh[it].as_ref().unwrap()));
            let mut next_triangles = Vec::with_capacity(3);
            for (index, (idx_s, s), (idx_t, t), (idx_o, o)) in [(index, a, b, c), (edge.next, b, c, a), (edge.previous, c, a, b)] {
                let edge = mesh[index].as_ref().unwrap();
                let idx_d = mesh[edge.opposite].as_ref().unwrap().apply(|opp| mesh[opp.next].as_ref().unwrap().target);
                let opp = mesh[edge.opposite].as_ref().unwrap().triangle; 
                if triangles[opp].is_some() {
                    continue;
                }
                let cs = coefficients[index].as_ref().unwrap().fixed_rows::<3>(0);
                // let matrix = SMatrix::<f64, 3, 3>::zeros().also(|it| {
                //     // it.view_mut((0,0), (1,3)).set_row(0, &RowVector3::from_element(1f64));
                //     it.view_mut((0,0), (3,1)).set_column(0, &Vector3::from_column_slice((&s).into()));
                //     it.view_mut((0,1), (3,1)).set_column(0, &Vector3::from_column_slice((&o).into()));
                //     it.view_mut((0,2), (3,1)).set_column(0, &Vector3::from_column_slice((&t).into()));
                // });
                let matrix = Matrix3::from_iterator(s.into_iter().chain(o.into_iter()).chain(t.into_iter()).cloned());
                let d = matrix * cs;
                let coefficients = coefficients.clone();
                println!("edge: {index}, d: {}", d);
                triangles[opp] = Some([(idx_s, s), (idx_d, d), (idx_t, t)]);
                next_triangles.push(opp);
            }

            for triangle in next_triangles {
                triangles = generate_triangles(mesh, coefficients, triangle, triangles);
            }
            triangles
        }
        let triangle = mesh.current_triangles()[0].clone();
        triangles[triangle.0] = Some(triangle.1.corners.map(|it| (it, mesh[it].as_ref().unwrap().coordinates.into())));
        let triangles = generate_triangles(&mesh, &coefficients, triangle.0, triangles);
        triangles.take().into_iter().filter_map(|it| it).flatten().for_each(|(index, node)| nodes[index] = Some(Node { coordinates: node.into(), outgoing: mesh[index].as_ref().unwrap().outgoing.clone() })); 

        Self {
            mesh,
            coefficients,
            nodes,
        }
    }

    fn expand(&self) -> List<SVector<f64, 4>, Edge> {
        let mut coefficients = List::with_defaults(self.mesh.edges().len());
        self.mesh.edges()
            .enumerate_some()
            .for_each(|(index, _)| {
                coefficients[index] = Some(self.get_coefficients(index));
            });
        coefficients
    }

    pub fn get_coefficients(&self, index: Index<Edge>) -> SVector<f64, 4> {
        let coefficients = self.coefficients[index].unwrap_or_else(|| {
            // Situation:   
            //
            //  d ----- c 
            //  |     ↗ |
            //  |   e   |
            //  | ↗     |
            //  a ----- b
            //
            // We have a,b,c -> d given by the edge e, but we only calculated and stored the conditions for c,d,a->b.
            // TODO: Validate if calculation is true.
            
            let [_a,_b,_c,_d] = self.coefficients[self.mesh[index].as_ref().unwrap().opposite].unwrap().data.0[0];
            SVector::<f64, 4>::new(-_c/_b, _d/_b, -_a/_b,-1f64)
        });
        coefficients
    }

    fn get_coefficients_using(coefficients: &List<SVector<f64, 4>, Edge>, mesh: &'M TM, index: Index<Edge>) -> SVector<f64, 4> {
        let coefficients = coefficients[index].unwrap_or_else(|| {
            // Situation:   
            //
            //  d ----- c 
            //  |     ↗ |
            //  |   e   |
            //  | ↗     |
            //  a ----- b
            //
            // We have a,b,c -> d given by the edge e, but we only calculated and stored the conditions for c,d,a->b.
            // TODO: Validate if calculation is true.
            
            let [_a,_b,_c,_d] = coefficients[mesh[index].as_ref().unwrap().opposite].unwrap().data.0[0];
            SVector::<f64, 4>::new(-_c/_b, 1f64/_b, -_a/_b,_d)
        });
        coefficients
    }

    fn calculate_opposite(&self, index: Index<Edge>) -> [f64; 3]{
        let edge = self.mesh[index].as_ref().unwrap();
        let (a,c) = (edge.source, edge.target);
        let b = self.mesh[edge.previous].as_ref().unwrap().source;
        let coefficients = self.get_coefficients(index);
        let columns = [a,b,c].map(|it| self.mesh[it].as_ref().unwrap().coordinates);
        let matrix = SMatrix::<f64, 3,3>::from_column_slice(columns.as_flattened());
        (matrix * coefficients.fixed_rows(0)).into()
    }

    fn recalculate_coefficients(&mut self, index: Index<Edge>, d: &SVector<f64, 3>) {
        let edge = self.mesh[index].as_ref().unwrap();
        let (a,c) = (edge.source, edge.target);
        let b = self.mesh[edge.previous].as_ref().unwrap().source;
        let columns = [a,b,c].map(|it| self.nodes[it].as_ref().unwrap().coordinates);
        let matrix = SMatrix::<f64, 3,3>::from_column_slice(columns.as_flattened());
        println!("matrix: {matrix}");
        println!("d: {d}");
        let coefficients = matrix.qr().solve(d).unwrap();
        self.coefficients[index] = Some(coefficients.insert_row(3, -1f64));
        self.coefficients[edge.opposite] = None;
    }

    fn find_triangle(mesh: &TM, coordinates: [f64; 3], node: Index<Node>) -> Option<Index<Triangle>> {
        for triangle in mesh.collect_outgoing_edges(node).into_iter().map(|edge| mesh[edge].as_ref().unwrap().triangle) {
            let cols = mesh[triangle].as_ref().unwrap().corners.map(|c| mesh[c].as_ref().unwrap().coordinates);
            if let Some(coordinates) = SMatrix::<f64, 3,3>::from_column_slice(cols.as_flattened()).qr().solve(&coordinates.into()) as Option<Vector3<f64>> {
                if coordinates.into_iter().filter(|i| **i < 0f64).collect_vec().is_empty() {
                    return Some(triangle)
                } 
            }
        }
        return None;
    }

    fn barycentric_coordinates_orig(mesh: &TM, dir: Index<Edge>, x: [f64; 3]) -> SVector<f64, 3> {
        let (a, c, prev) = mesh[dir].as_ref().unwrap().apply(|it| (it.source, it.target, it.previous));
        let b = mesh[prev].as_ref().unwrap().source;
        SMatrix::<f64, 3, 3>::from_column_slice([a,b,c].map(|c| mesh[c].as_ref().unwrap().coordinates).as_flattened()).qr().solve(&x.into()).unwrap()
    }

    fn from_barycentric(&self, bar: SVector<f64, 3>, [a,b,c]: [Index<Node>; 3]) -> [f64; 3] {
        println!("basis: {}", SMatrix::<f64, 3, 3>::from_column_slice([a,b,c].map(|c| self.nodes[c].as_ref().unwrap().coordinates).as_flattened()));
        let d = SMatrix::<f64, 3, 3>::from_column_slice([a,b,c].map(|c| self.nodes[c].as_ref().unwrap().coordinates).as_flattened()) * bar;
        d.data.0[0]
    }

    pub fn visualise_coefficients(&self) -> crate::mesh::MeshTriangleInfo {
        (&self.mesh, self.expand()).visualise()
    }
}


#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use apply::Apply;
    use approx::assert_relative_eq;
    use nalgebra::{SMatrix, SVector};

    use crate::{mesh::{ContractableMesh, FromMeshBuilder, List, MeshBuilder, TriangleMesh, UnfinishedNode}, projective_structure::{structure::ProjectiveStructure, CalculateProjectiveStructure}, ClosedTriangleMesh};

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
    fn save_restore_cube_hec() {
        let mut mesh = cube();
        mesh.contract_edge(0.into());
        let m = mesh.clone().apply(|it| (it.clone(), it.calculate_projective_structure())).apply(|(mut mesh, structure)| {
            let mut coefficients = List::with_defaults(mesh.edges.len());
            structure.into_iter().for_each(|((index, _), coefficient)| coefficients[index] = Some(coefficient));
            ProjectiveStructure::from_bare(coefficients, mesh)
        });
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

    #[test]
    fn restore_orbifold_beccari_c_equations_fulfilled() {
        let structure: ProjectiveStructure<ClosedTriangleMesh> = ProjectiveStructure::restore_from_dir_by_cp(&mut PathBuf::from("./test_output/"), "orbifold", 4);
        for (index, edge) in structure.mesh.current_edges_undirected() {
            let (a, c) = (edge.source, edge.target);
            let b = structure.mesh[edge.previous].as_ref().unwrap().source;
            let d = structure.mesh[edge.opposite].as_ref().unwrap().next.apply(|edge| structure.mesh[edge].as_ref().unwrap().target);
            let [a,b,c,d] = [a,b,c,d].map(|it| structure.mesh[it].as_ref().unwrap().coordinates);
            let matrix = SMatrix::<f64, 3, 4>::from_column_slice([a,b,c,d].as_flattened()).insert_row(0,1f64);
            let coefficients = structure.get_coefficients(index);
            assert_relative_eq!(SVector::zeros(), matrix * coefficients, max_relative = 1e-9)
        }
    }
}
// impl<'M, TM: UncontractableMesh +'M> VisualiseProjectiveStructure for ProjectiveStructure<'M, TM> {
//     fn visualise(&self) -> crate::mesh::MeshTriangleInfo {
        
//     }
// }