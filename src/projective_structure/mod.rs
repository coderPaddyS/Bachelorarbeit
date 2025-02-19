use std::{collections::HashMap, time::Instant};

use apply::{Also, Apply};
use bevy::{ecs::system::IntoSystem, math::Vec3};
use itertools::Itertools;
use levenberg_marquardt::{LeastSquaresProblem, LevenbergMarquardt};
use nalgebra::{zero, ArrayStorage, Const, DMatrix, DVector, Dyn, Matrix4, Matrix4x1, Matrix4x3, MatrixMN, RowVector, SMatrix, SVector, ToTypenum, UninitVector, Vector2, Vector3, Vector4, Vector6, VectorN};

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
        self.a *= a;
        self.b *= b;
        self.c *= c;
        self.d *= d;
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
        let b = SMatrix::<f64, 8, 1>::from_row_iterator(self.0.d.into_iter().chain(self.1.d.into_iter()).cloned());
        println!("{upper}");
        println!("{lower}");
        
        let matrix =  SMatrix::<f64, 8, 3>::from_row_iterator(upper.transpose().into_iter().chain(lower.transpose().into_iter()).cloned());
        println!("{matrix}");
        let svd = matrix.svd(true, true);
        let coefficient = svd .solve(&b, 10e-13).unwrap();
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

#[derive(Clone, Copy, Debug)]
pub struct SubdividedTriangleData {
    source: [(Index<Edge>, [f64; 3]); 3],
    target: [(Index<Edge>, [f64; 3]); 3]
}

impl SubdividedTriangleData {
    pub fn new(edges: [(Index<Edge>, [[f64; 3]; 2]); 3]) -> SubdividedTriangleData {
        let (source, target): (Vec<_>, Vec<_>) = edges.map(|(idx, [s,t])| ((idx, s), (idx, t))).into_iter().unzip();
        Self { source: source.try_into().unwrap(), target: target.try_into().unwrap() }
    }

    pub fn get_points(&self, edge: Index<Edge>) -> ([f64; 3], [f64; 3]) {
        (self.source.iter().find(|(idx, _)| *idx == edge).unwrap().1, self.target.iter().find(|(idx, _)| *idx == edge).unwrap().1)
    }
}

impl MeshData for SubdividedTriangleData {
    type CollapseInfoInput = TriangleMeshHalfEdgeCollapse;
    type CollapseInfoOutput = TriangleMeshHalfEdgeCollapse;
    fn build_mesh(mesh: &ClosedTriangleMesh<Self>) -> Option<crate::mesh::MeshTriangleInfo> {
        let (nodes, colors): (Vec<[f32; 3]>, Vec<[u8; 4]>) = mesh.triangles
            .enumerate_some()
            .map(|(index, tri)| {
                let [a,b,c] = tri.corners;
                let ab = mesh.find_edge(a, b).unwrap();
                let edge_ab = mesh[ab].as_ref().unwrap();
                let subtriangles = mesh.triangle_data()[index]
                    .as_ref().unwrap()
                    .apply(|it| [it.get_points(ab), it.get_points(edge_ab.next), it.get_points(edge_ab.previous)])
                    .map(|(a,b)| [a,b])
                    .as_flattened()
                    .iter().collect_array::<6>().unwrap()
                    .also(|it| it.rotate_right(1))
                    .into_iter()
                    .array_chunks::<2>()
                    .zip([a,b,c].map(|n| mesh[n].as_ref().unwrap().coordinates).into_iter())
                    .flat_map(|( [l,r], s)| [s,*r,*l])
                    .map(|it| it.map(|i| i as f32))
                    .collect_vec();
                let nodes: Vec<_> = subtriangles.into_iter().map(|it| (it, [rand::random(), rand::random(), rand::random(), 128])).collect::<Vec<_>>();
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
            let idx = a.outgoing.iter().find(|out| edges[**out].as_ref().unwrap().target == triangle.corners[1]).unwrap();
            let (next, prev) = edges[*idx].as_ref().unwrap().apply(|it| (it.next, it.previous));

            let divide = |idx: Index<Edge>, s: &Node, t: &Node| {
                let [s, t] = [Vector3::from(s.coordinates), Vector3::from(t.coordinates)];
                (idx, [s + (t - s)/3f64, t + (s - t)/3f64].map(|it| it.into()))
            };
            triangle_data[index.into()] = Some(SubdividedTriangleData::new([divide(*idx, a, b), divide(next, b, c), divide(prev, c, a)]));
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
    // pub fn calculate_c_conditions_of_edge(&self, index: Index<Edge>) -> Vector4<f64> {
    //     let edge = self[index].as_ref().unwrap();
    //     let opposite = self[edge.opposite].as_ref().unwrap();
    //     let (s, t) = (edge.source, edge.target);
    //     let (upper_quad, lower_quad) = {
    //         // Identify the correct triangle at each side of the halfedge
    //         // The subdivided triangles are in the same order as the corners of the big triangle
    //         let triangle_data_edge = {
    //             let idx_s = self[edge.triangle].as_ref().unwrap().corners.iter().enumerate().find(|i| *i.1 == s).unwrap().0;
    //             self.triangle_data[edge.triangle].as_ref().unwrap().apply(|it| (it[idx_s], it[(idx_s + 1) % it.len()]))
    //         };
    //         let triangle_data_opposite = {
    //             let idx_s = self[opposite.triangle].as_ref().unwrap().corners.iter().enumerate().find(|i| *i.1 == t).unwrap().0;
    //             self.triangle_data[opposite.triangle].as_ref().unwrap().apply(|it| (it[idx_s], it[(idx_s + 1) % it.len()]))
    //         };

    //         // Connect each triangle of edge with the attached triangle of the opposite side.

    //         let upper_triangle = (triangle_data_edge.0, triangle_data_opposite.1);
    //         let lower_triangle = (triangle_data_edge.1, triangle_data_opposite.0);

    //         assert_eq!(upper_triangle.0[0], upper_triangle.1[0]);
    //         assert_eq!(lower_triangle.0[0], lower_triangle.1[0]);
    //         let u_a = upper_triangle.0[0];
    //         let l_c = lower_triangle.0[0];
    //         // TODO: Continue fixing triangl_data_edge -> upper_triangle and other

    //         // The other points of the triangle are always build in the same direction such that two connected points have opposites direction in the entries [1] and [2]
    //         let (u_b, u_c, u_d, l_b, l_a, l_d) = if upper_triangle.0[1] == upper_triangle.1[1] {
    //             (upper_triangle.0[2], upper_triangle.0[1], upper_triangle.1[2],
    //                 lower_triangle.0[1], lower_triangle.0[1], lower_triangle.1[1])
    //         } else if upper_triangle.0[2] == upper_triangle.1[2] {
    //             (upper_triangle.0[1], upper_triangle.0[2], upper_triangle.1[1],
    //                 lower_triangle.0[2], lower_triangle.0[1], lower_triangle.1[2])
    //         } else if upper_triangle.0[1] == upper_triangle.1[2] {
    //             (upper_triangle.0[2], upper_triangle.0[1], upper_triangle.1[1],
    //                 lower_triangle.0[1], lower_triangle.0[2], lower_triangle.1[0])
    //         } else {
    //             (upper_triangle.0[1], upper_triangle.0[2], upper_triangle.1[2],
    //                 lower_triangle.0[2], lower_triangle.0[1], lower_triangle.1[1])
    //         };

    //         let upper_quad = Quad::from_coordinates(RealNode::A, u_a, u_b, u_c, u_d);
    //         let lower_quad = Quad::from_coordinates(RealNode::C, l_a ,l_b, l_c,l_d);
    //         (upper_quad, lower_quad)
    //     };
    //     let upper = Matrix4x3::from_columns(&[upper_quad.a, upper_quad.b, upper_quad.c]);
    //     let lower = Matrix4x3::from_columns(&[lower_quad.a, lower_quad.b, lower_quad.c]);
    //     let b = -SMatrix::<f64, 8, 1>::from_row_iterator(upper_quad.d.into_iter().chain(lower_quad.d.into_iter()).cloned());
    //     println!("{upper}");
    //     println!("{lower}");
        
    //     let matrix =  SMatrix::<f64, 8, 3>::from_row_iterator(upper.transpose().into_iter().chain(lower.transpose().into_iter()).cloned());
    //     println!("{matrix}");
    //     let svd = matrix.svd(true, true);
    //     let coefficient = svd .solve(&b, 10e-8).unwrap();
    //     let b = matrix * coefficient;
    //     println!("{coefficient}, error: {b}");
    //     coefficient.push(1f64)
    // }

    // pub fn calculate_c_conditions(&self) -> List<Vector4<f64>, Edge> {
    //     let mut coefficients = List::<Vector4<f64>, Edge>::with_defaults(self.edges.len());
    //     self.edges
    //         .iter()
    //         .enumerate()
    //         .map(|(idx, edge)| (idx.into(), edge.as_ref().unwrap().opposite))
    //         .for_each(|(idx, opposite)| {
    //             if coefficients[idx].is_none() {
    //                 let coefficient = self.calculate_c_conditions_of_edge(idx);
    //                 coefficients[idx] = Some(coefficient);
    //                 coefficients[opposite] =  Some(coefficient);
    //             }
    //         });
    //     coefficients
    // }

    pub fn calculate_projective_structure(self, target_error: f64, max_iterations: usize) -> ClosedTriangleMesh</* TODO: correct type */()> { 
        let mut equations = CEquations::new(&self);
        let initial = equations.parameters.to_vec();
        let initial_param = CEquationParameters::from_vec(initial.clone());
        println!("initial solution quality: {}", equations.calculate().norm());
        let N = equations.mapping.len();
        let mut iteration = 0;
        let mut matrix = DMatrix::from_diagonal_element(11 * N, 11 * N, 1.0f64).resize(19 * N, 19 * N, 0f64);
        // let mut b = DVector::from_element(19 * N, 0f64);
        let mut b = equations.parameters.to_vec().resize_vertically(19 * N, 0f64);
        println!("calculating ps: Edges: {}", N);
        let now = Instant::now();

        println!("iteration: {iteration}, points: {}\n", equations.parameters.points.iter().map(|it| it.iter().map(|c| format!("{:.4}, ", c)).join("")).join(", "));
        println!("iteration: {iteration}, weights: {}\n", equations.parameters.weights.iter().map(|it| it.iter().map(|c| format!("{:.4}, ", c)).join("")).join(", "));
        println!("iteration: {iteration}, coefficients: {}\n", equations.parameters.coefficients.iter().map(|it| it.iter().map(|c| format!("{:.4}, ", c)).join("")).join(", "));

        while iteration < max_iterations {
            let (linearization, q) = equations.linearization();

            // Replace the sections of the matrix by the new C-Equations
            matrix.view_mut((11 * N, 0), (8 * N, 11 * N)).apply(|mut matrix| {
                for (index, row) in linearization.row_iter().enumerate() {
                    matrix.set_row(index, &row);
                }
            });
            matrix.view_mut((0, 11 * N), (11 * N, 8 * N)).apply(|mut matrix| {
                for (index, column) in linearization.transpose().column_iter().enumerate() {
                    matrix.set_column(index, &column);
                }
            });
            // let b_slice = &mut b.data.as_mut_slice()[6*N..11*N];
            // b_slice.copy_from_slice(&DVector::zeros(5 * N).as_slice());
            let b_slice = &mut b.data.as_mut_slice()[11*N..19*N];
            b_slice.copy_from_slice(q.data.as_slice());

            log::debug!("solution quality: {}", equations.calculate().norm());
            log::debug!("norm q: {}", q.norm());
            // println!("norm of D^T * Dx = D^T * q: {}", (&linearization.transpose() * &linearization).qr().solve(&(linearization.transpose() * &q)).unwrap().norm());
            log::debug!("rank composite: {}", matrix.rank(1e-16));
            log::debug!("rank D: {}", linearization.rank(1e-16));
            let solution = CEquationParameters::from_vec(matrix.clone().qr().solve(&b).unwrap().rows(0, 11*N).into());
            // let mut solution = CEquationParameters::from_vec(0.5f64 * (&initial - matrix.clone().qr().solve(&b).unwrap().rows(0, 11*N).into_owned()));
            // solution.points = initial_param.points.iter().zip(solution.points.iter()).map(|(a, b)| 0.5f64 * (*a - *b)).collect_vec();
            // solution.weights = initial_param.weights.iter().zip(solution.weights.iter()).map(|(a, b)| 0.5f64 * (*a - *b)).collect_vec();
            // solution.coefficients = initial_param.coefficients.iter().zip(solution.coefficients.iter()).map(|(a, b)| 0.5f64 * (*a - *b)).collect_vec();

            log::debug!("error to q with x_i: {}", (linearization * solution.to_vec() - q).norm());
            log::info!("norm (x_i - x_0): {}", (solution.to_vec() - &initial).norm());
            // println!("error to q with x_i+1: {}", (linearization * solution.to_vec() - q).norm());
            // let delta 
            // if (&solution)

            equations.parameters = solution;
            // log::info!("iteration: {iteration}, weights: {}\n", equations.parameters.weights.iter().map(|it| format!("{:.4}, {:.4}", it[0], it[1])).join(", "));
            log::info!("iteration: {iteration}, norm of residual: {}\n", equations.calculate().norm());
            iteration += 1;
        }

        println!("initial:      points: {}\n", initial_param.points.iter().map(|it| it.iter().map(|c| format!("{:.4}, ", c)).join("")).join(", "));
        println!("initial:      weights: {}\n", initial_param.weights.iter().map(|it| it.iter().map(|c| format!("{:.4}, ", c)).join("")).join(", "));
        println!("initial:      coefficients: {}\n", initial_param.coefficients.iter().map(|it| it.iter().map(|c| format!("{:.4}, ", c)).join("")).join(", "));

        println!("iteration: {iteration}, points: {}\n", equations.parameters.points.iter().map(|it| it.iter().map(|c| format!("{:.4}, ", c)).join("")).join(", "));
        println!("iteration: {iteration}, weights: {}\n", equations.parameters.weights.iter().map(|it| it.iter().map(|c| format!("{:.4}, ", c)).join("")).join(", "));
        println!("iteration: {iteration}, coefficients: {}\n", equations.parameters.coefficients.iter().map(|it| it.iter().map(|c| format!("{:.4}, ", c)).join("")).join(", "));

        println!("norm difference: points: {}\n", (DMatrix::from_row_iterator(N, 6, equations.parameters.points.iter().flatten().cloned()) - DMatrix::from_row_iterator(N, 6, initial_param.points.iter().flatten().cloned())).norm());
        println!("norm difference: weights: {}\n", (DMatrix::from_row_iterator(N, 2, equations.parameters.weights.iter().flatten().cloned()) - DMatrix::from_row_iterator(N, 2, initial_param.weights.iter().flatten().cloned())).norm());
        println!("norm difference: coefficients: {}\n", (DMatrix::from_row_iterator(N, 4, equations.parameters.coefficients.iter().flatten().cloned()) - DMatrix::from_row_iterator(N, 4, initial_param.coefficients.iter().flatten().cloned())).norm());

        println!("Time elapsed: {}", now.elapsed().as_millis());
        println!("calculating ps: Finished");
        ClosedTriangleMesh::</* TODO: correct type */()> { triangle_data: List::with_defaults(self.triangles.len()), ..self }
    }
}

struct CEquationParameters {
    points: Vec<SVector<f64, 6>>,
    weights: Vec<SVector<f64, 2>>,
    coefficients: Vec<SVector<f64, 4>>
}

impl CEquationParameters {
    pub fn new(points: Vec<SVector<f64, 6>>, weights: Vec<SVector<f64, 2>>, coefficients: Vec<SVector<f64, 3>>) -> Self {
        assert_eq!(points.len(), weights.len());
        assert_eq!(points.len(), coefficients.len());
        let coefficients = coefficients.into_iter().map(|v| v.insert_row(3, -1f64)).collect();
        Self { points, weights, coefficients }
    }

    /// Produces the vector representing the parameters of length 11 * N where N is the number of edges
    pub fn to_vec(&self) -> DVector<f64> {
        let iter = self.points.iter().flat_map(|it| it.data.0[0])
            .chain(self.weights.iter().flat_map(|it| it.data.0[0]))
            .chain(self.coefficients.iter().flat_map(|it| it.data.0[0][0..=2].iter().cloned()));
        DVector::from_iterator(11 * self.points.len(), iter)
    }

    /// Consumes the vector representing the parameters of length 11 * N where N is the number of edges
    pub fn from_vec(vector: DVector<f64>) -> Self {
        assert!(vector.len() % 11 == 0);
        let N = vector.len() / 11;
        let (points, (weights, coefficients)) = vector.data.as_slice().split_at(6 * N).apply(|(points, it)| (points, it.split_at(2 * N)));
        Self {
            points: points.chunks(6).map(|it| SVector::<f64,6>::from_row_slice(it)).collect_vec(),
            weights: weights.chunks(2).map(|it| SVector::<f64,2>::from_row_slice(it)).collect_vec(),
            coefficients: coefficients.chunks(3).map(|it| SVector::<f64,3>::from_row_slice(it).insert_row(3, -1f64)).collect_vec()
        }
    }
}

struct CEquations<'M, TM: TriangleMesh + 'M> {
    parameters: CEquationParameters,
    mapping: Vec<(Index<Edge>, Index<Edge>)>,
    mesh: &'M TM,
}

impl<'M, TM: TriangleMesh<Data = SubdividedTriangleData>> CEquations<'M, TM> {
    fn new(mesh: &'M TM) -> Self {
        let edges = mesh.current_edges_undirected();
        let N = edges.len();
        let mut mapping = Vec::<(Index<Edge>, Index<Edge>)>::with_capacity(N);
        let mut points = Vec::<Vector6<f64>>::with_capacity(N);
        let mut weights = Vec::<Vector2<f64>>::with_capacity(N);
        let mut coefficients = Vec::<Vector3<f64>>::with_capacity(N);

        for (index, edge) in edges {
            if mapping.contains(&(index, edge.opposite)) || mapping.contains(&(edge.opposite, index)) {
                continue;
            }
            mapping.push((index, edge.opposite));
            let data = mesh.triangle_data()[edge.triangle].as_ref().unwrap();
            let (a,b) = data.get_points(index);
            let vec = Vector6::from_row_iterator(a.into_iter().chain(b.into_iter()));
            points.push(vec);
            weights.push(Vector2::from_element(1f64));
        }

        let find_mapping = |edge| mapping.iter().enumerate().find(|(_, j)| j.0 == edge || j.1 == edge).unwrap().0;

        for (index, edge) in mesh.current_edges() {
            let i = find_mapping(index);
            if coefficients.get(i).is_some() {
                continue;
            }
            let quads = Self::get_weighted_quads_for_edge_with((&points, &weights), &mapping, mesh, index, &edge);
            coefficients.push(quads.calculate_c_coefficients());
        }

        Self {
            parameters: CEquationParameters::new(points, weights, coefficients),
            mapping,
            mesh
        }
    }

    fn get_weighted_points(edge: Index<Edge>, mapping: &Vec<(Index<Edge>, Index<Edge>)>, points: &Vec<SVector<f64, 6>>, weights: &Vec<SVector<f64, 2>>) -> ((f64, Vector3<f64>), (f64, Vector3<f64>)) {
        let (i, reverse) = mapping
            .into_iter()
            .enumerate()
            .find_map(|(i, (forward, backward))| if edge == *forward { Some((i, false)) } else if edge == *backward { Some((i, true)) } else { None })
            .unwrap();
        // let i = Self::find_mapping_within(edge, mapping);
        let (a,b) = (points[i], weights[i]).apply(|(points, weights)| {
            let (a,b) = (points.fixed_rows::<3>(0).into_owned(), points.fixed_rows::<3>(3).into_owned());
            ((weights[0], a), (weights[1], b))
        });
        if reverse { (b, a) } else { (a, b) }
    }

    fn find_mapping(&self, edge: Index<Edge>) -> usize { Self::find_mapping_within(edge, &self.mapping) }

    fn find_mapping_within(edge: Index<Edge>, mapping: &Vec<(Index<Edge>, Index<Edge>)>) -> usize {
        mapping.iter().enumerate().find(|(_, j)| j.0 == edge || j.1 == edge).unwrap().0
    }

    fn get_weighted_quads_for_edge_with((points, weights): (&Vec<SVector<f64, 6>>, &Vec<SVector<f64, 2>>), mapping: &Vec<(Index<Edge>, Index<Edge>)>, mesh: &'M TM, index: Index<Edge>, edge: &Edge) -> (Quad, Quad) {
        let opposite = mesh[edge.opposite].as_ref().unwrap();

        let (l_a, u_c): (Vector3<f64>, Vector3<f64>) = (mesh[edge.source].as_ref().unwrap().coordinates.into(), mesh[edge.target].as_ref().unwrap().coordinates.into());
        // let ((w_lc, l_c), (w_ua, u_a)) = Self::get_weighted_points(index, mapping, points, weights);
        let ((w_d, l_c), (w_b, u_a)) = Self::get_weighted_points(index, mapping, points, weights);

        let find_ep = |index: Index<Edge>, node: &Vector3<f64>| {
        // let find_ep = |index: Index<Edge>| {^
            // Self::get_weighted_points(index, mapping, points, weight^s).0
            let map = Self::find_mapping_within(index, mapping);
            let (_first, _second) = points[map].apply(|it| (it.fixed_rows::<3>(0).clone_owned(), it.fixed_rows::<3>(3).clone_owned()));
            let (w_f, w_s) = weights[map].apply(|it| (it[0], it[1]));
            if (_first - node).norm_squared() < ( _second - node).norm_squared() { (w_f, _first) } else { (w_s, _second) }
        };

        // let (l_d, w_ld) = find_ep(edge.previous, &l_a);
        // let (u_d, w_ud) = find_ep(edge.next, &u_c);
        // let (l_b, w_lb) = find_ep(opposite.next, &l_a);
        // let (u_b, w_ub) = find_ep(opposite.previous, &u_c);

        let (w_ld, l_d) = find_ep(edge.previous, &l_a);
        let (w_ud, u_d) = find_ep(edge.next, &u_c);
        let (w_lb, l_b) = find_ep(opposite.next, &l_a);
        let (w_ub, u_b) = find_ep(opposite.previous, &u_c);

        let (mut u, mut l) = (Quad::from(RealNode::C, u_a, u_b, u_c, u_d), Quad::from(RealNode::A, l_a, l_b, l_c, l_d));
        u.set_weights(w_b, w_ub, 1.0f64, w_ud);
        l.set_weights(1.0f64, w_lb, w_d, w_ld);
        // u.set_weights(1f64, w_d, 1.0f64, w_d);
        // l.set_weights(1.0f64, w_d, 1f64, w_d);
        (u, l)
    }

    fn get_weighted_quads_for_edge(&self, index: Index<Edge>, edge: &Edge) -> (Quad, Quad) {
        CEquations::<TM>::get_weighted_quads_for_edge_with((&self.parameters.points, &self.parameters.weights), &self.mapping, &self.mesh, index, edge)
    }

    pub fn calculate(&self) -> DVector<f64> {
        let mut results = Vec::<(Index<Edge>, SVector<f64, 8>)>::with_capacity(self.parameters.points.len());
        for (index, edge) in self.mesh.current_edges() {
            if results.iter().find(|(i, _)| *i == index || *i == edge.opposite).is_some() {
                continue
            }
            let matrix = self.point_weight_matrix_of_edge(index, &edge);
            // let (matrix, b) = self.point_weight_matrix_of_edge(index, &edge)
            //     .data.0
            //     .split_at(3)
            //     .apply(|(matrix, b)| (SMatrix::<f64, 8, 3>::from_column_slice(matrix.as_flattened()), SVector::<f64, 8>::from_column_slice(&b[0])));
            let i = self.find_mapping(index);
            let coefficients = self.parameters.coefficients[i];
            results.push((index, matrix * coefficients));
            // results.push((index, matrix * coefficients));
        }
        DVector::from_iterator(8 * self.parameters.coefficients.len(),  results.into_iter().flat_map(|(_, vector)| vector.data.0[0]))
    }

    pub fn point_weight_matrix_of_edge(&self, index: Index<Edge>, edge: &Edge) -> SMatrix<f64, 8, 4> {
        let (upper, lower) = self.get_weighted_quads_for_edge(index, edge);
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
        let (l, u) = self.get_weighted_quads_for_edge(index, edge);
        let coefficents = self.parameters.coefficients[i];

        let derive_quad_after_x = |quad: &Quad, x: &Vector3<f64>| {
            let normalized_quad = quad.normalized(); 
            let to_derive = |a: &Vector4<f64>, b: &Vector3<f64>| { (a[1] - b[0]).abs() < 1e-14  && (a[2] - b[1]).abs() < 1e-14 && (a[3] - b[2]).abs() < 1e-14 };
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
        let (l, u) = self.get_weighted_quads_for_edge(index, edge);
        let coefficents = self.parameters.coefficients[i];

        let derive_quad_after_w = |quad: &Quad| {
            let normalized_quad = quad.normalized(); 
            let to_derive = |a: &Vector4<f64>, b: &Vector3<f64>| { (a[1] - b[0]).abs() < 1e-14  && (a[2] - b[1]).abs() < 1e-14 && (a[3] - b[2]).abs() < 1e-14 };
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
        let (l, u) = self.get_weighted_quads_for_edge(index, edge);
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

    /// Returns the linearization (the first taylor approximation) of the C-Equations as a (A, b)-Tuple
    /// where $Ax = b$.
    pub fn linearization(&self) -> (DMatrix<f64>, DVector<f64>) {
        let derivative = self.derivative();
        let x = self.parameters.to_vec();
        let b = &derivative * x - self.calculate();
        (derivative, b)
    }

    pub fn improve_values(&self) -> Self {
        let (derivative, b) = self.linearization();
        let x = derivative.svd(true, true).solve(&b, 10e-8).unwrap();
        Self {
            mapping: self.mapping.clone(),
            mesh: &self.mesh,
            parameters: CEquationParameters::from_vec(x)
        }
    }
}

impl<'M, TM: TriangleMesh<Data = SubdividedTriangleData>> LeastSquaresProblem<f64, Dyn, Dyn> for CEquations<'M, TM> {
    type ResidualStorage = nalgebra::storage::Owned<f64, Dyn>;
    type JacobianStorage = nalgebra::storage::Owned<f64, Dyn, Dyn>;
    type ParameterStorage = nalgebra::storage::Owned<f64, Dyn>;

    fn set_params(&mut self, x: &nalgebra::Vector<f64, Dyn, Self::ParameterStorage>) {
        self.parameters = CEquationParameters::from_vec(x.clone())
    }

    fn params(&self) -> nalgebra::Vector<f64, Dyn, Self::ParameterStorage> {
        self.parameters.to_vec()
    }

    fn jacobian(&self) -> Option<nalgebra::Matrix<f64, Dyn, Dyn, Self::JacobianStorage>> {
        Some(self.derivative())
    }

    fn residuals(&self) -> Option<nalgebra::Vector<f64, Dyn, Self::ResidualStorage>> {
        Some(self.calculate())
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

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
    pub fn subdivide_tetraeder() {
        let mesh = tetraeder();
        let submesh = mesh.subdivide();
    }

    #[test]
    pub fn jacobian_correct() {
        let mesh = tetraeder();
        let submesh = mesh.subdivide();

        println!("number of edges: {}", submesh.current_edges_undirected().len());
        let mut equations = CEquations::new(&submesh);
        let jacobian = equations.jacobian().unwrap();
        let num = levenberg_marquardt::differentiate_numerically(&mut equations).unwrap();
        for (i, elem) in jacobian.iter().enumerate() {
            assert!((*elem - num[i]).abs() < 1e-10, "error of {} at index {i} with: jac: {elem}, num: {}", (*elem - num[i]).abs(), num[i]);
        }
        println!("eps: {}", (&num - &jacobian).norm());
        approx::assert_relative_eq!(num, jacobian, epsilon = 1e-10);
    }

    #[test]
    pub fn tetraeder_iterative_jacobian_correct() {
        let mesh = tetraeder();
        let submesh = mesh.subdivide();
        let mut equations = CEquations::new(&submesh);
        let N = equations.mapping.len();
        let initial = equations.parameters.to_vec();
        let mut iteration = 0;
        let mut matrix = DMatrix::from_diagonal_element(11 * N, 11 * N, 1.0f64).resize(19 * N, 19 * N, 0f64);
        // let mut b = DVector::from_element(19 * N, 0f64);
        let mut b = equations.parameters.to_vec().resize_vertically(19 * N, 0f64);

        while iteration < 5 {
            let (linearization, q) = equations.linearization();
            let num = levenberg_marquardt::differentiate_numerically(&mut equations).unwrap();
            for (i, elem) in linearization.iter().enumerate() {
                assert!((*elem - num[i]).abs() < 1e-10, "error of {} at index {i} with: jac: {elem}, num: {}", (*elem - num[i]).abs(), num[i]);
            }
            approx::assert_relative_eq!(num, linearization, epsilon = 1e-10);

            // Replace the sections of the matrix by the new C-Equations
            matrix.view_mut((11 * N, 0), (8 * N, 11 * N)).apply(|mut matrix| {
                for (index, row) in linearization.row_iter().enumerate() {
                    matrix.set_row(index, &row);
                }
            });
            matrix.view_mut((0, 11 * N), (11 * N, 8 * N)).apply(|mut matrix| {
                for (index, column) in linearization.transpose().column_iter().enumerate() {
                    matrix.set_column(index, &column);
                }
            });
            let b_slice = &mut b.data.as_mut_slice()[11*N..19*N];
            b_slice.copy_from_slice(q.data.as_slice());

            let solution = CEquationParameters::from_vec(matrix.clone().qr().solve(&b).unwrap().rows(0, 11*N).into());
            // println!("error to q with x_i+1: {}", (linearization * solution.to_vec() - q).norm());
            // let delta 
            // if (&solution)

            equations.parameters = solution;
            println!("iteration: {iteration}, norm of residual: {}", equations.calculate().norm());
            iteration += 1;
        }
    }

    #[test]
    pub fn tetraeder_projective_structure_correct() {
        pretty_env_logger::init();
        let mesh = tetraeder();
        let submesh = mesh.subdivide();
        submesh.calculate_projective_structure(1e-9, 1000);
    }

    #[test]
    pub fn cube_projective_structure_correct() {
        pretty_env_logger::init();
        let mesh = cube();
        let submesh = mesh.subdivide();
        submesh.calculate_projective_structure(1e-9, 200);
    }
}