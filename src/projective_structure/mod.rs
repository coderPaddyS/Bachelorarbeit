use std::collections::HashMap;

use apply::Apply;
use bevy::{ecs::system::IntoSystem, math::Vec3};
use itertools::Itertools;
use nalgebra::{zero, ArrayStorage, Const, DMatrix, DVector, Matrix4, Matrix4x1, Matrix4x3, MatrixMN, RowVector, SMatrix, SVector, ToTypenum, UninitVector, Vector2, Vector3, Vector4, Vector6, VectorN};

use crate::{mesh::{Edge, Index, List, MeshData, MeshTriangleInfo, Node, Triangle, TriangleMesh, TriangleMeshHalfEdgeCollapse}, ClosedTriangleMesh};

#[derive(Debug, Clone, Copy)]
pub enum RealNode {
    A, C
}

pub struct Quad {
    pub real_node: RealNode,
    pub a: Vector4<f64>,
    pub b: Vector4<f64>,
    pub c: Vector4<f64>,
    pub d: Vector4<f64>,
}

impl Quad {
    fn from_coordinates(real_node: RealNode, a: [f64; 3], b: [f64; 3], c: [f64; 3], d: [f64; 3]) -> Self {
        Self {
            real_node,
            a: Vector3::from(a).insert_row(0, 1f64),
            b: Vector3::from(b).insert_row(0, 1f64),
            c: Vector3::from(c).insert_row(0, 1f64),
            d: Vector3::from(d).insert_row(0, 1f64),
        }
    }

    fn from(real_node: RealNode, a: Vector3<f64>, b: Vector3<f64>, c: Vector3<f64>, d: Vector3<f64>) -> Self {
        Self {
            real_node,
            a: Vector3::from(a).insert_row(0, 1f64),
            b: Vector3::from(b).insert_row(0, 1f64),
            c: Vector3::from(c).insert_row(0, 1f64),
            d: Vector3::from(d).insert_row(0, 1f64),
        }
    }

    fn set_weights(&mut self, a: f64, b: f64, c: f64, d: f64) {
        self.a[0] = a;
        self.b[0] = b;
        self.c[0] = c;
        self.d[0] = d;
    }

    fn normalized(&self) -> Self {
        Self {
            real_node: self.real_node,
            a: self.a / self.a[0],
            b: self.b / self.b[0],
            c: self.c / self.c[0],
            d: self.d / self.d[0],
        }
    }

    fn contains(&self, x: &Vector3<f64>) -> bool {
        self.a.fixed_rows::<3>(1) == *x || self.b.fixed_rows::<3>(1) == *x || self.c.fixed_rows::<3>(1) == *x || self.d.fixed_rows::<3>(1) == *x
    }

    fn a_weightless(&self) -> Vector3<f64> { self.a.fixed_rows(1) / self.a[0] }
    fn b_weightless(&self) -> Vector3<f64> { self.b.fixed_rows(1) / self.b[0] }
    fn c_weightless(&self) -> Vector3<f64> { self.c.fixed_rows(1) / self.c[0] }
    fn d_weightless(&self) -> Vector3<f64> { self.d.fixed_rows(1) / self.d[0] }
}

trait CalculateCCoefficients {
    fn calculate_c_coefficients(&self) -> Vector3<f64>;
}

impl CalculateCCoefficients for (Quad, Quad) {
    fn calculate_c_coefficients(&self) -> Vector3<f64> {
        let upper = Matrix4x3::from_columns(&[self.0.a, self.0.b, self.0.c]);
        let lower = Matrix4x3::from_columns(&[self.1.a, self.1.b, self.1.c]);
        let b = -SMatrix::<f64, 8, 1>::from_row_iterator(self.0.d.into_iter().chain(self.1.d.into_iter()).cloned());
        println!("{upper}");
        println!("{lower}");
        
        let matrix =  SMatrix::<f64, 8, 3>::from_row_iterator(upper.transpose().into_iter().chain(lower.transpose().into_iter()).cloned());
        println!("{matrix}");
        let svd = matrix.svd(true, true);
        let coefficient = svd .solve(&b, 10e-8).unwrap();
        // let b = matrix * coefficient;
        // println!("{coefficient}, error: {b}");
        coefficient
    }
}

pub struct SubdividedTriangle {
    pub a: Vector4<f64>,
    pub b: Vector4<f64>,
    pub c: Vector4<f64>,
    pub d: Vector4<f64>,
}

pub trait SubdivideConnectedTriangles {
    fn subdivide(&self) -> [SubdividedTriangle; 2];
}    

impl SubdivideConnectedTriangles for (ClosedTriangleMesh, Index<Triangle>, Index<Triangle>) {
    fn subdivide(&self) -> [SubdividedTriangle; 2] {
        let (left, right) = (self.1.clone(), self.2.clone());
        let (l_corners, r_corners) = (&self.0[left].as_ref().unwrap().corners, &self.0[right].as_ref().unwrap().corners);
        let [lower, upper, left, right] = {
            let left = l_corners.iter().find(|it| !r_corners.contains(*it)).expect("there to subdivide connected inequal triangles.");
            let [upper, lower] = l_corners.iter().filter(|it| *it != left).collect::<Vec<_>>()[0..1] else {
                panic!("expected there to be exactly two common nodes");
            };
            let right = r_corners.iter().find(|it| *it != upper && *it != lower).expect("there to subdivide connected inequal trianles.");
            [lower, upper, left, right].map(|node| self.0[*node].as_ref().unwrap().coordinates).map(|it| Vector3::from(it))
        };
        let lower_triangle = [(lower, 1f64), (lower + left / 3f64, 1.1f64), (lower + upper / 3f64, 1.1f64), (lower + right / 3f64, 1.1f64)].map(|(vec, weight)| vec.insert_row(0, weight));
        let upper_triangle = [(upper, 1f64), (upper + right / 3f64, 1.1f64), (upper + lower / 3f64, 1.1f64), (upper + left / 3f64, 1.1f64)].map(|(vec, weight)| vec.insert_row(0, weight));

        [lower_triangle, upper_triangle].map(|[a,b,c,d]| SubdividedTriangle { a, b, c, d })
    }
}

type SubdividedTriangleData = [[[f64; 3]; 3]; 3];

impl MeshData for SubdividedTriangleData {
    type CollapseInfoInput = TriangleMeshHalfEdgeCollapse;
    type CollapseInfoOutput = TriangleMeshHalfEdgeCollapse;
    fn build_mesh(mesh: &ClosedTriangleMesh<Self>) -> Option<crate::mesh::MeshTriangleInfo> {
        let (nodes, colors): (Vec<[f32; 3]>, Vec<[u8; 4]>) = mesh.triangle_data.clone().take()
            .into_iter()
            .filter(|it| it.is_some())
            .flatten()
            .flatten()
            .map(|tri| {
                let nodes: Vec<_> = tri.into_iter().map(|it| (it.map(|n| n as f32), [rand::random(), rand::random(), rand::random(), 128])).collect::<Vec<_>>();
                // let normal = (Vector3::from(nodes[1]) - Vector3::from(nodes[0])).cross(&(Vector3::from(nodes[2]) - Vector3::from(nodes[0])));

                nodes
            })
            .flatten()
            .unzip();

        let triangles = (0..nodes.len()).into_iter().zip(colors).collect();
        Some(MeshTriangleInfo { nodes, triangles })
    }
    fn contract_edge(mesh: &mut ClosedTriangleMesh<Self>, collapse_info: Self::CollapseInfoInput) -> Self::CollapseInfoOutput {
        collapse_info
    }
}
pub trait SubdiviveMesh {
    fn subdivide(self) -> ClosedTriangleMesh<SubdividedTriangleData>;
}

impl SubdiviveMesh for ClosedTriangleMesh<()> {
    fn subdivide(self) -> ClosedTriangleMesh<SubdividedTriangleData> {
        let ClosedTriangleMesh { nodes, edges, triangles, contraction_order, undo_contraction, ..} = self;
        let mut triangle_data: List<SubdividedTriangleData, Triangle> = List::with_defaults(triangles.len());
        for (index, triangle) in triangles.iter().enumerate().filter_map(|it| it.1.as_ref().map(|t| (it.0, t))) {
            let [a,b,c] = triangle.corners.map(|corner| nodes[corner].as_ref().unwrap());
            let divide = |a: &Node, b: &Node, c: &Node| {
                let [a,b,c] = [Vector3::from(a.coordinates), Vector3::from(b.coordinates), Vector3::from(c.coordinates)];
                let points: [[f64; 3]; 3]  = [a, a + (b - a)/3f64, a + (c - a)/3f64].map(|it| [it.x, it.y, it.z]);
                points
            };
            triangle_data[index.into()] = Some([divide(a,b,c), divide(b,c,a), divide(c, a, b)]);
        }
        ClosedTriangleMesh::<SubdividedTriangleData> {
            nodes,
            edges,
            triangles,
            triangle_data,
            contraction_order,
            undo_contraction
        }
    }
}

impl ClosedTriangleMesh<SubdividedTriangleData> {
    pub fn calculate_c_conditions_of_edge(&self, index: Index<Edge>) -> Vector4<f64> {
        let edge = self[index].as_ref().unwrap();
        let opposite = self[edge.opposite].as_ref().unwrap();
        let (s, t) = (edge.source, edge.target);
        let (upper_quad, lower_quad) = {
            // Identify the correct triangle at each side of the halfedge
            // The subdivided triangles are in the same order as the corners of the big triangle
            let triangle_data_edge = {
                let idx_s = self[edge.triangle].as_ref().unwrap().corners.iter().enumerate().find(|i| *i.1 == s).unwrap().0;
                self.triangle_data[edge.triangle].as_ref().unwrap().apply(|it| (it[idx_s], it[(idx_s + 1) % it.len()]))
            };
            let triangle_data_opposite = {
                let idx_s = self[opposite.triangle].as_ref().unwrap().corners.iter().enumerate().find(|i| *i.1 == t).unwrap().0;
                self.triangle_data[opposite.triangle].as_ref().unwrap().apply(|it| (it[idx_s], it[(idx_s + 1) % it.len()]))
            };

            // Connect each triangle of edge with the attached triangle of the opposite side.

            let upper_triangle = (triangle_data_edge.0, triangle_data_opposite.1);
            let lower_triangle = (triangle_data_edge.1, triangle_data_opposite.0);

            assert_eq!(upper_triangle.0[0], upper_triangle.1[0]);
            assert_eq!(lower_triangle.0[0], lower_triangle.1[0]);
            let u_a = upper_triangle.0[0];
            let l_c = lower_triangle.0[0];
            // TODO: Continue fixing triangl_data_edge -> upper_triangle and other

            // The other points of the triangle are always build in the same direction such that two connected points have opposites direction in the entries [1] and [2]
            let (u_b, u_c, u_d, l_b, l_a, l_d) = if upper_triangle.0[1] == upper_triangle.1[1] {
                (upper_triangle.0[2], upper_triangle.0[1], upper_triangle.1[2],
                    lower_triangle.0[1], lower_triangle.0[1], lower_triangle.1[1])
            } else if upper_triangle.0[2] == upper_triangle.1[2] {
                (upper_triangle.0[1], upper_triangle.0[2], upper_triangle.1[1],
                    lower_triangle.0[2], lower_triangle.0[1], lower_triangle.1[2])
            } else if upper_triangle.0[1] == upper_triangle.1[2] {
                (upper_triangle.0[2], upper_triangle.0[1], upper_triangle.1[1],
                    lower_triangle.0[1], lower_triangle.0[2], lower_triangle.1[0])
            } else {
                (upper_triangle.0[1], upper_triangle.0[2], upper_triangle.1[2],
                    lower_triangle.0[2], lower_triangle.0[1], lower_triangle.1[1])
            };

            let upper_quad = Quad::from_coordinates(RealNode::A, u_a, u_b, u_c, u_d);
            let lower_quad = Quad::from_coordinates(RealNode::C, l_a ,l_b, l_c,l_d);
            (upper_quad, lower_quad)
        };
        let upper = Matrix4x3::from_columns(&[upper_quad.a, upper_quad.b, upper_quad.c]);
        let lower = Matrix4x3::from_columns(&[lower_quad.a, lower_quad.b, lower_quad.c]);
        let b = -SMatrix::<f64, 8, 1>::from_row_iterator(upper_quad.d.into_iter().chain(lower_quad.d.into_iter()).cloned());
        println!("{upper}");
        println!("{lower}");
        
        let matrix =  SMatrix::<f64, 8, 3>::from_row_iterator(upper.transpose().into_iter().chain(lower.transpose().into_iter()).cloned());
        println!("{matrix}");
        let svd = matrix.svd(true, true);
        let coefficient = svd .solve(&b, 10e-8).unwrap();
        let b = matrix * coefficient;
        println!("{coefficient}, error: {b}");
        coefficient.push(1f64)
    }

    pub fn calculate_c_conditions(&self) -> List<Vector4<f64>, Edge> {
        let mut coefficients = List::<Vector4<f64>, Edge>::with_defaults(self.edges.len());
        self.edges
            .iter()
            .enumerate()
            .map(|(idx, edge)| (idx.into(), edge.as_ref().unwrap().opposite))
            .for_each(|(idx, opposite)| {
                if coefficients[idx].is_none() {
                    let coefficient = self.calculate_c_conditions_of_edge(idx);
                    coefficients[idx] = Some(coefficient);
                    coefficients[opposite] =  Some(coefficient);
                }
            });
        coefficients
    }

    pub fn calculate_projective_structure(self, target_error: f64, max_iterations: u32) -> ClosedTriangleMesh</* TODO: correct type */()> { 
        let mut equations = CEquations::new(&self);
        /*TODO: possibly transform c_conditions in correct type by e.g. combining with weights */
        let mut solution = equations.calculate();
        let mut iteration = 0;
        while iteration < max_iterations {
            equations = equations.improve_values();
            let next_solution = equations.calculate();

            println!("iteration: {iteration}, diff: {:?}, sol: {:?}", (&solution - &next_solution).norm(), &next_solution.norm());
            if (solution - &next_solution).norm() < target_error {
                break;
            }
            iteration += 1;
            solution = next_solution;
        }
        ClosedTriangleMesh::</* TODO: correct type */()> { triangle_data: List::with_defaults(self.triangles.len()), ..self }
    }
}

struct CEquationParameters {
    points: Vec<SVector<f64, 6>>,
    weights: Vec<SVector<f64, 2>>,
    coefficients: Vec<SVector<f64, 3>>
}

impl CEquationParameters {
    pub fn new(points: Vec<SVector<f64, 6>>, weights: Vec<SVector<f64, 2>>, coefficients: Vec<SVector<f64, 3>>) -> Self {
        assert_eq!(points.len(), weights.len());
        assert_eq!(points.len(), coefficients.len());
        Self { points, weights, coefficients }
    }

    pub fn to_vec(&self) -> DVector<f64> {
        let iter = self.points.iter().flat_map(|it| it.data.0[0])
            .chain(self.weights.iter().flat_map(|it| it.data.0[0]))
            .chain(self.coefficients.iter().flat_map(|it| it.data.0[0]));
        DVector::from_iterator(11 * self.points.len(), iter)
    }

    pub fn from_vec(vector: DVector<f64>) -> Self {
        assert!(vector.len() % 11 == 0);
        let N = vector.len() / 11;
        let (points, (weights, coefficients)) = vector.data.as_slice().split_at(6 * N).apply(|(points, it)| (points, it.split_at(2 * N)));
        Self {
            points: points.chunks(6).map(|it| SVector::<f64,6>::from_row_slice(it)).collect_vec(),
            weights: weights.chunks(2).map(|it| SVector::<f64,2>::from_row_slice(it)).collect_vec(),
            coefficients: coefficients.chunks(3).map(|it| SVector::<f64,3>::from_row_slice(it)).collect_vec()
        }
    }
}

struct CEquations<'M, TM: TriangleMesh + 'M> {
    parameters: CEquationParameters,
    mapping: Vec<(Index<Edge>, Index<Edge>)>,
    mesh: &'M TM,
}

impl<'M, TM: TriangleMesh + 'static> CEquations<'M, TM> {
    fn new(mesh: &'M TM) -> Self {
        let N = mesh.current_edges().len();
        let mut mapping = Vec::<(Index<Edge>, Index<Edge>)>::with_capacity(N);
        let mut points = Vec::<Vector6<f64>>::with_capacity(N);
        let mut weights = Vec::<Vector2<f64>>::with_capacity(N);
        let mut coefficients = Vec::<Vector3<f64>>::with_capacity(N);

        for (index, edge) in mesh.current_edges() {
            if mapping.contains(&(index, edge.opposite)) || mapping.contains(&(edge.opposite, index)) {
                continue;
            }
            mapping.push((index, edge.opposite));
            let a: Vector3<f64> = mesh[edge.source].as_ref().unwrap().coordinates.into();
            let b: Vector3<f64> = mesh[edge.target].as_ref().unwrap().coordinates.into();
            let (ep_a, ep_b) = (a + (b - a)/3f64, b + (a - b)/3f64);
            let vec = Vector6::from_row_iterator( ep_a.into_iter().chain(ep_b.into_iter()).cloned());
            points.push(vec);
            weights.push(Vector2::from_element(1f64));
        }

        let find_mapping = |edge| mapping.iter().enumerate().find(|(_, j)| j.0 == edge || j.1 == edge).unwrap().0;

        for (index, edge) in mesh.current_edges() {
            let i = find_mapping(index);
            if coefficients.get(i).is_some() {
                continue;
            }
            let opposite = mesh[edge.opposite].as_ref().unwrap();

            let (l_a, u_c): (Vector3<f64>, Vector3<f64>) = (mesh[edge.source].as_ref().unwrap().coordinates.into(), mesh[edge.target].as_ref().unwrap().coordinates.into());
            let (ep_a, ep_b) = points[i].apply(|it| (it.fixed_rows::<3>(0).clone_owned(), it.fixed_rows::<3>(3).clone_owned()));
            let (l_c, u_a) = if (ep_a - l_a).norm_squared() < (ep_b - l_a).norm_squared() { (ep_a, ep_b) } else { (ep_b, ep_a) };

            let find_ep = |index: Index<Edge>, node: Vector3<f64>| {
                let (first, second) = points[find_mapping(edge.previous)].apply(|it| (it.fixed_rows::<3>(0).clone_owned(), it.fixed_rows::<3>(3).clone_owned()));
                if (first - node).norm_squared() < ( second - node).norm_squared() { first } else { second }
            };

            let l_d = find_ep(edge.previous, l_a);
            let u_d = find_ep(edge.next, u_c);

            let l_b = find_ep(opposite.next, l_a);
            let u_b = find_ep(opposite.previous, u_c);

            let quads = (Quad::from(RealNode::C, u_a, u_b, u_c, u_d), Quad::from(RealNode::A, l_a, l_b, l_c, l_d));
            coefficients.push(quads.calculate_c_coefficients());
        }

        Self {
            parameters: CEquationParameters::new(points, weights, coefficients),
            mapping,
            mesh
        }
    }

    fn find_mapping(&self, edge: Index<Edge>) -> usize { Self::find_mapping_within(edge, &self.mapping) }

    fn find_mapping_within(edge: Index<Edge>, mapping: &Vec<(Index<Edge>, Index<Edge>)>) -> usize {
        mapping.iter().enumerate().find(|(_, j)| j.0 == edge || j.1 == edge).unwrap().0
    }

    fn get_quads_for_edge(&self, index: Index<Edge>, edge: &Edge) -> (Quad, Quad) {
        let i = self.find_mapping(index);
        let opposite = self.mesh[edge.opposite].as_ref().unwrap();

        let (l_a, u_c): (Vector3<f64>, Vector3<f64>) = (self.mesh[edge.source].as_ref().unwrap().coordinates.into(), self.mesh[edge.target].as_ref().unwrap().coordinates.into());
        let (ep_a, ep_b) = self.parameters.points[i].apply(|it| (it.fixed_rows::<3>(0).clone_owned(), it.fixed_rows::<3>(3).clone_owned()));
        let (l_c, u_a) = if (ep_a - l_a).norm_squared() < (ep_b - l_a).norm_squared() { (ep_a, ep_b) } else { (ep_b, ep_a) };

        let find_ep = |index: Index<Edge>, node: Vector3<f64>| {
            let (first, second) = self.parameters.points[self.find_mapping(edge.previous)].apply(|it| (it.fixed_rows::<3>(0).clone_owned(), it.fixed_rows::<3>(3).clone_owned()));
            if (first - node).norm_squared() < ( second - node).norm_squared() { first } else { second }
        };

        let l_d = find_ep(edge.previous, l_a);
        let u_d = find_ep(edge.next, u_c);

        let l_b = find_ep(opposite.next, l_a);
        let u_b = find_ep(opposite.previous, u_c);

        (Quad::from(RealNode::C, u_a, u_b, u_c, u_d), Quad::from(RealNode::A, l_a, l_b, l_c, l_d))
    }

    pub fn calculate(&self) -> DVector<f64> {
        let mut results = Vec::<(Index<Edge>, SVector<f64, 8>)>::with_capacity(self.parameters.points.len());
        for (index, edge) in self.mesh.current_edges() {
            if results.iter().find(|(i, _)| *i == index || *i == edge.opposite).is_some() {
                continue
            }
            let (matrix, b) = self.point_weight_matrix_of_edge(index, &edge)
                .data.0
                .split_at(3)
                .apply(|(matrix, b)| (SMatrix::<f64, 8, 3>::from_column_slice(matrix.as_flattened()), SVector::<f64, 8>::from_column_slice(&b[0])));
            let i = self.find_mapping(index);
            let coefficients = self.parameters.coefficients[i];
            results.push((index, matrix * coefficients - b));
        }
        DVector::from_iterator(8 * self.parameters.coefficients.len(),  results.into_iter().flat_map(|(_, vector)| vector.data.0[0]))
    }

    pub fn point_weight_matrix_of_edge(&self, index: Index<Edge>, edge: &Edge) -> SMatrix<f64, 8, 4> {
        let (upper, lower) = self.get_quads_for_edge(index, edge);
        let cols = [(upper.a, lower.a), (upper.b, lower.b), (upper.c, lower.c), (upper.d, lower.d)]
            .into_iter()
            .flat_map(|(a,b)| 
                a.into_iter()
                .chain(b.into_iter())
                .cloned()
                .collect::<Vec<_>>()
            );
        SMatrix::from_iterator(cols)
    }

    pub fn derivative_of_edge_after_p(&self, x: &Vector3<f64>, index: Index<Edge>, edge: &Edge) -> SMatrix<f64, 8, 3> {
        let i = self.find_mapping(index);
        let (l, u) = self.get_quads_for_edge(index, edge);
        let coefficents = self.parameters.coefficients[i];

        let derive_quad_after_x = |quad: &Quad, x: &Vector3<f64>| {
            let normalized_quad = quad.normalized(); 
            let to_derive = |a: &Vector4<f64>, b: &Vector3<f64>| { a[1] == b[0] && a[2] == b[1] && a[3] == b[2] };
            let derivative: SMatrix<f64, 3, 3> = if to_derive(&normalized_quad.a, x) {
                SMatrix::from_diagonal_element(coefficents[0] * quad.a[0]) 
            } else if to_derive(&normalized_quad.b, x) {
                SMatrix::from_diagonal_element(coefficents[1] * quad.b[0]) 
            } else if to_derive(&normalized_quad.c, x) {
                SMatrix::from_diagonal_element(coefficents[2] * quad.c[0]) 
            } else if to_derive(&normalized_quad.d, x) {
                SMatrix::from_diagonal_element(coefficents[3] * quad.d[0]) 
            } else {
                SMatrix::zeros()
            };
            derivative.insert_row(0, 0f64)
        };

        SMatrix::from_row_iterator([l, u].iter().flat_map(|quad| derive_quad_after_x(&quad, x).transpose().data.0).flatten())
    }

    pub fn derivative_of_edge_after_w(&self, x: &Vector3<f64>, index: Index<Edge>, edge: &Edge) -> SVector<f64, 8> {
        let i = self.find_mapping(index);
        let (l, u) = self.get_quads_for_edge(index, edge);
        let coefficents = self.parameters.coefficients[i];

        let derive_quad_after_w = |quad: &Quad| {
            let normalized_quad = quad.normalized(); 
            let to_derive = |a: &Vector4<f64>, b: &Vector3<f64>| { a[1] == b[0] && a[2] == b[1] && a[3] == b[2] };
            let derivative: SVector<f64, 4> = if to_derive(&normalized_quad.a, x) {
                coefficents[0] * normalized_quad.a
            } else if to_derive(&normalized_quad.b, x) {
                coefficents[1] * normalized_quad.b
            } else if to_derive(&normalized_quad.c, x) {
                coefficents[2] * normalized_quad.c
            } else if to_derive(&normalized_quad.d, x) {
                coefficents[3] * normalized_quad.d
            } else {
                SVector::zeros()
            };
            derivative
        };

        SVector::<f64, 8>::from_column_slice([l, u].map(|quad| derive_quad_after_w(&quad).data.0[0]).as_flattened())
    }

    pub fn derivative_of_edge_after_coefficients(&self, index: Index<Edge>, edge: &Edge) -> SMatrix<f64, 8, 3> {
        let (l, u) = self.get_quads_for_edge(index, edge);
        SMatrix::from_column_slice([l.a, u.a, l.b, u.b, l.c, u.c].map(|v| v.data.0[0]).as_flattened())
    }

    pub fn derivative(&self) -> DMatrix<f64> {
        let N = self.parameters.points.len();
        let edges = self.mesh.current_edges_undirected();
        let der_point_iter = self.parameters.points
            .iter()
            .flat_map(|x| [x.fixed_rows::<3>(0).into_owned(), x.fixed_rows::<3>(3).into_owned()])
            .map(|x| {
                let iter = edges.iter()
                    .flat_map(|(index, edge)| self.derivative_of_edge_after_p(&x, *index, edge).transpose().data.0)
                    .flatten();
                DMatrix::<f64>::from_row_iterator(8 * N, 3, iter)
            })
            .flat_map(|x| Into::<Vec<f64>>::into(x.data));
        let der_weight_iter = self.parameters.points
            .iter()
            .flat_map(|x| [x.fixed_rows::<3>(0).into_owned(), x.fixed_rows::<3>(3).into_owned()])
            .map(|x| {
                let iter = edges.iter()
                    .map(|(index, edge)| self.derivative_of_edge_after_w(&x, *index, edge).data.0[0])
                    .flatten();
                DVector::<f64>::from_iterator(8 * N, iter)
            })
            .flat_map(|x| Into::<Vec<f64>>::into(x.data));
        let der_coefficient_iter = (0..N)
            .map(|i| {
                edges.iter()
                    .enumerate()
                    .flat_map(|(j, (index, edge))| {
                        let matrix = if i == j {
                            self.derivative_of_edge_after_coefficients(*index, edge)
                        } else {
                            SMatrix::zeros()
                        };
                        matrix.transpose().data.0
                    })
                    .flatten()
                    .apply(|it| DMatrix::<f64>::from_row_iterator(8 * N, 3, it))
                    .apply(|matrix| Into::<Vec<f64>>::into(matrix.data))
            })
            .flatten();
        let points = der_point_iter.clone().collect_vec();
        let weights = der_weight_iter.clone().collect_vec();
        let coefficients = der_coefficient_iter.clone().collect_vec();
        let values = der_point_iter.chain(der_weight_iter).chain(der_coefficient_iter).collect_vec();
        DMatrix::<f64>::from_iterator(8 * N, 11 * N, values.into_iter())
    }

    pub fn improve_values(&self) -> Self {
        let derivative = self.derivative();
        let x_0 = self.parameters.to_vec();
        let b = &derivative * x_0 - self.calculate();
        let x = derivative.svd(true, true).solve(&b, 10e-8).unwrap();
        Self {
            mapping: self.mapping.clone(),
            mesh: &self.mesh,
            parameters: CEquationParameters::from_vec(x)
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::mesh::{FromMeshBuilder, MeshBuilder, UnfinishedNode};

    use super::*;

    fn tetraeder() -> ClosedTriangleMesh {
        let mut builder = MeshBuilder::default();
        let nodes: Vec<_> = vec![
            [-1.0f64, 0f64, 0f64],
            [1.0f64, 0f64, 0f64],
            [0f64,1.0f64, 0f64],
            [0f64, 0f64, 1.0f64],
        ].into_iter().map(|node| {
            builder.add_node(UnfinishedNode::new_f64(node))
        }).collect();
        vec![
            [0, 1, 2], [2, 1, 3],
            [0, 3, 1], [0, 2, 3] 
        ].into_iter().for_each(|[a, b, c]| {
            builder.add_triangle_by_nodes(nodes[a], nodes[b], nodes[c]).unwrap();
        });
        ClosedTriangleMesh::build(builder).unwrap()
    }

    #[test]
    pub fn subdivide_tetraeder() {
        let mesh = tetraeder();
        let submesh = mesh.subdivide();
    }

    #[test]
    pub fn tetraeder_c_coefficient() {
        let mesh = tetraeder();
        let submesh = mesh.subdivide();
        submesh.calculate_c_conditions_of_edge(0.into());
    }
}