use std::{collections::HashMap, time::Instant};

use apply::{Also, Apply};
use bevy::{ecs::system::IntoSystem, math::Vec3};
use itertools::Itertools;
use levenberg_marquardt::{LeastSquaresProblem, LevenbergMarquardt};
use nalgebra::{zero, ArrayStorage, ComplexField, Const, DMatrix, DVector, Dyn, Matrix3, Matrix4, Matrix4x1, Matrix4x3, MatrixMN, RowVector, SMatrix, SVector, ToTypenum, UninitVector, Vector2, Vector3, Vector4, Vector6, VectorN};

use crate::{mesh::{Edge, Index, List, MeshData, MeshTriangleInfo, Node, Triangle, TriangleMesh, TriangleMeshHalfEdgeCollapse}, ClosedTriangleMesh};

pub mod visualisation;
pub mod structure;

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
        
        let matrix =  SMatrix::<f64, 8, 3>::from_row_iterator(upper.transpose().into_iter().chain(lower.transpose().into_iter()).cloned());
        println!("{matrix}");
        println!("{b}");
        let qr = matrix.qr();
        let coefficient = qr.r().solve_upper_triangular_unchecked(&(qr.q().transpose() * b));
        println!("residual: {}", (b - matrix * coefficient).norm());
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
    fn build_mesh<TM: TriangleMesh<Data = Self> + ?Sized>(mesh: &TM) -> Option<crate::mesh::MeshTriangleInfo> {
        let (nodes, colors): (Vec<[f32; 3]>, Vec<[u8; 4]>) = mesh.triangles()
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
        for (index, triangle) in triangles.enumerate_some() {
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

pub enum CalculationProjectiveStructureStepsize {
    Break(f64),
    Reset(f64)
}

impl ClosedTriangleMesh<SubdividedTriangleData> {

    pub fn calculate_projective_structure<'M>(&'M mut self, target_error: f64, max_iterations: usize, rescales: usize, strategy: CalculationProjectiveStructureStepsize) -> (CEquations<'M, Self>, Vec<CEquationParameters>){ 
        let mut history = Vec::new();
        let mut equations = CEquations::new(self);
        history.push(equations.parameters.clone());
        let (initial, maps) = equations.parameters.to_vec();
        let initial_param = CEquationParameters::from_vec(initial.clone(), maps.clone());
        println!("initial solution quality: {}", equations.calculate().norm());
        let N = equations.mapping.len();
        let mut iteration = 0;
        let mut rescale = 0;
        let mut matrix = DMatrix::from_diagonal_element(11 * N, 11 * N, 1.0f64).resize(19 * N, 19 * N, 0f64); // &equations.parameters.to_vec().map(|i| (i.abs()).recip())
        
        // Maybe only stay near initial cps and weights
        // let mut matrix = DMatrix::from_diagonal_element(8 * N, 8 * N, 1.0f64).resize(19 * N, 19 * N, 0f64); // &equations.parameters.to_vec().map(|i| (i.abs()).recip())
        // let mut matrix = (DMatrix::from_diagonal_element(11 * N, 11 * N, 200f64 - 0.7f64 * equations.calculate().norm().recip())).resize(19 * N, 19 * N, 0f64);
        let mut stepsize = 1f64;
        // let mut b = DVector::from_element(19 * N, 0f64);
        println!("calculating ps: Edges: {}", N);
        let now = Instant::now();
        let mut min = equations.parameters.clone();
        let mut index_min = 0;
        let mut previous_params = equations.parameters.clone();
        // return equations;

        while rescale < rescales {
            println!("new baseline");
            println!("iteration: {iteration}, points: {}\n", equations.parameters.points.iter().map(|it| it.iter().map(|c| format!("{:.4}, ", c.1)).join("")).join(", "));
            println!("iteration: {iteration}, weights: {}\n", equations.parameters.weights.iter().map(|it| it.iter().map(|c| format!("{:.4}, ", c.1)).join("")).join(", "));
            println!("iteration: {iteration}, coefficients: {}\n", equations.parameters.coefficients.iter().map(|it| it.iter().map(|c| format!("{:.4}, ", c)).join("")).join(", "));
            let mut b = initial_param.to_vec().0.resize_vertically(19 * N, 0f64);
            // let mut b = equations.parameters.to_vec().0.resize_vertically(19 * N, 0f64);
            
            // Maybe only stay near initial cps and weights
            // let iter = equations.parameters.points.iter().flatten().flat_map(|(_, c)| c.data.0[0]).chain((0..2*N).map(|i| 1f64));
            // let mut b = DVector::<f64>::from_iterator(8 * N, iter).resize_vertically(19 * N, 0f64);
            // let rescale_initial = equations.parameters.to_vec();
            let rescale_initial = initial_param.to_vec();
            iteration = 0;
            stepsize = 1f64;
            let mut improvements = false;
            
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
                let solution = CEquationParameters::from_vec(matrix.clone().qr().solve(&b).unwrap().rows(0, 11*N).into(), maps.clone());

                log::debug!("error to q with x_i: {}", (linearization * solution.to_vec().0 - q).norm());
                log::info!("norm (x_i - x_0): {}", (solution.to_vec().0 - &initial).norm());
                log::info!("Rescale: norm (x_i - x_0): {}\n", (solution.to_vec().0 - &rescale_initial.0).norm());

                log::info!("equations residual: {}", equations.calculate().norm());
                log::info!("solutions residual: {}", equations.evaluate_parameter(&solution).norm());
                if equations.evaluate_parameter(&solution).norm() < equations.evaluate_parameter(&min).norm() {
                    log::info!("New min: {}", equations.evaluate_parameter(&min).norm());
                    min = solution.clone();
                    index_min = history.len() + 1;
                    previous_params = equations.parameters.clone();
                    equations.parameters = solution;
                    stepsize = 1f64.min(2f64 * stepsize);
                    log::info!("Increasing stepsize: {}", stepsize);
                    improvements = true;
                } else if equations.calculate().norm() < equations.evaluate_parameter(&solution).norm() {
                    match strategy {
                        CalculationProjectiveStructureStepsize::Break(threshold) => if stepsize < threshold { break } else { stepsize *= 0.5f64 },
                        CalculationProjectiveStructureStepsize::Reset(threshold) => if stepsize < threshold { stepsize = 1f64; } else { stepsize *= 0.5f64 }
                    }
                    log::info!("Decreasing stepsize: {}", stepsize);
                    // let factor = (equations.calculate().norm() / equations.evaluate_parameter(&solution).norm()).sqrt();
                    // equations.parameters = (previous_params.clone() + solution.clone() - &equations.parameters - &equations.parameters) * factor * stepsize + equations.parameters;
                    equations.parameters = (solution.clone() - &equations.parameters) * stepsize + equations.parameters;
                } else {
                    previous_params = equations.parameters.clone();
                    equations.parameters = solution;
                    stepsize = 1f64.min(2f64 * stepsize);
                    log::info!("Increasing stepsize: {}", stepsize);
                    improvements = true;
                }
                history.push(equations.parameters.clone());
                // let mut solution = CEquationParameters::from_vec(0.5f64 * (&initial - matrix.clone().qr().solve(&b).unwrap().rows(0, 11*N).into_owned()));
                // solution.points = initial_param.points.iter().zip(solution.points.iter()).map(|(a, b)| 0.5f64 * (*a - *b)).collect_vec();
                // solution.weights = initial_param.weights.iter().zip(solution.weights.iter()).map(|(a, b)| 0.5f64 * (*a - *b)).collect_vec();
                // solution.coefficients = initial_param.coefficients.iter().zip(solution.coefficients.iter()).map(|(a, b)| 0.5f64 * (*a - *b)).collect_vec();
    
                // println!("error to q with x_i+1: {}", (linearization * solution.to_vec() - q).norm());
                // let delta 
                // if (&solution)
    
                // log::info!("iteration: {iteration}, weights: {}\n", equations.parameters.weights.iter().map(|it| format!("{:.4}, {:.4}", it[0], it[1])).join(", "));
                log::info!("iteration: {iteration}, norm of residual: {}\n", equations.calculate().norm());

                if equations.calculate().norm() < target_error {
                    break;
                }

                iteration += 1;
            }
            if !improvements {
                log::info!("No new improvements after rescale: {rescale}");
                // break;
            }
            rescale += 1;
        }

        equations.parameters = min;

        println!("initial:      points: {}\n", initial_param.points.iter().map(|it| it.iter().flat_map(|c| c.1.data.0[0]).map(|c| format!("{:.4}, ", c)).join("")).join(", "));
        println!("initial:      weights: {}\n", initial_param.weights.iter().map(|it| it.iter().map(|c| format!("{:.4}, ", c.1)).join("")).join(", "));
        println!("initial:      coefficients: {}\n", initial_param.coefficients.iter().map(|it| it.iter().map(|c| format!("{:.4}, ", c)).join("")).join(", "));

        println!("rescale: {rescale}, iteration: {iteration}, points: {}\n", equations.parameters.points.iter().map(|it| it.iter().flat_map(|c| c.1.data.0[0]).map(|c| format!("{:.4}, ", c)).join("")).join(", "));
        println!("rescale: {rescale}, iteration: {iteration}, weights: {}\n", equations.parameters.weights.iter().map(|it| it.iter().map(|c| format!("{:.4}, ", c.1)).join("")).join(", "));
        println!("rescale: {rescale}, iteration: {iteration}, coefficients: {}\n", equations.parameters.coefficients.iter().map(|it| it.iter().map(|c| format!("{:.4}, ", c)).join("")).join(", "));

        println!("norm difference: points: {}\n", (DMatrix::from_row_iterator(N, 6, equations.parameters.points.iter().flatten().flat_map(|(_, i)| i).cloned()) - DMatrix::from_row_iterator(N, 6, initial_param.points.iter().flatten().flat_map(|(_, i)| i).cloned())).norm());
        println!("norm difference: weights: {}\n", (DMatrix::from_row_iterator(N, 2, equations.parameters.weights.iter().flatten().map(|(_, i)| i).cloned()) - DMatrix::from_row_iterator(N, 2, initial_param.weights.iter().flatten().map(|(_, i)| i).cloned())).norm());
        println!("norm difference: coefficients: {}\n", (DMatrix::from_row_iterator(N, 4, equations.parameters.coefficients.iter().flatten().cloned()) - DMatrix::from_row_iterator(N, 4, initial_param.coefficients.iter().flatten().cloned())).norm());
        
        let residual = equations.calculate().norm();
        println!("iteration: {iteration}, norm of residual: {}\n", residual);

        println!("Time elapsed: {}", now.elapsed().as_millis());
        println!("calculating ps: Finished");
        // CEquations {
        //     mapping: equations.mapping,
        //     parameters: equations.parameters,
        //     mesh: equations.mesh.drop_data()
        // }
        (equations, history[..index_min].into_iter().cloned().collect_vec())
    }
}

pub trait CalculateProjectiveStructure: Sized + TriangleMesh {
    fn calculate_projective_structure<'M>(&'M self) -> Vec<((Index<Edge>, Edge), SVector<f64, 4>)>;
}

impl CalculateProjectiveStructure for ClosedTriangleMesh<()> {
    fn calculate_projective_structure<'M>(&'M self) -> Vec<((Index<Edge>, Edge), SVector<f64, 4>)> {
        self.edges()
            .enumerate_some()
            .map(|(index, edge)| {
                let b = self[edge.next].as_ref().unwrap().target;
                let (a, c) = (edge.source, edge.target);
                let d = self[self[edge.opposite].as_ref().unwrap().next].as_ref().unwrap().target;
                let [a,b,c,d] = [a,b,c,d].map(|it| self[it].as_ref().unwrap().coordinates);
                let matrix = Matrix3::<f64>::from_iterator(a.into_iter().chain(b.into_iter()).chain(c.into_iter()));
                let coeffiecients = matrix.qr().solve(&Vector3::from(d)).unwrap().insert_row(3, -1f64);

                ((index, edge.clone()), coeffiecients)
            }).collect()
    }
}

#[derive(Clone)]
pub struct CEquationParameters {
    points: Vec<[(Index<Node>, SVector<f64, 3>); 2]>,
    weights: Vec<[(Index<Node>, f64); 2]>,
    coefficients: Vec<SVector<f64, 4>>
}

impl std::ops::Add for CEquationParameters {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self {
            coefficients: self.coefficients.into_iter().zip(rhs.coefficients.into_iter()).map(|(a, b)| a + b).collect_vec(),
            points: self.points.into_iter().zip(rhs.points.into_iter()).map(|(a, b)| [(a[0].0, a[0].1 + b[0].1), (a[1].0, a[1].1 + b[1].1)]).collect_vec(),
            weights: self.weights.into_iter().zip(rhs.weights.into_iter()).map(|(a, b)| [(a[0].0, a[0].1 + b[0].1), (a[1].0, a[1].1 + b[1].1)]).collect_vec()
        }
    }
}

impl std::ops::Sub for CEquationParameters {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            coefficients: self.coefficients.into_iter().zip(rhs.coefficients.into_iter()).map(|(a, b)| a - b).collect_vec(),
            points: self.points.into_iter().zip(rhs.points.into_iter()).map(|(a, b)| [(a[0].0, a[0].1 - b[0].1), (a[1].0, a[1].1 - b[1].1)]).collect_vec(),
            weights: self.weights.into_iter().zip(rhs.weights.into_iter()).map(|(a, b)|  [(a[0].0, a[0].1 - b[0].1), (a[1].0, a[1].1 - b[1].1)]).collect_vec()
        }
    }
}

impl std::ops::Sub<&Self> for CEquationParameters {
    type Output = Self;
    fn sub(self, rhs: &Self) -> Self::Output {
        Self {
            coefficients: self.coefficients.into_iter().zip(rhs.coefficients.iter()).map(|(a, b)| a - b).collect_vec(),
            points: self.points.into_iter().zip(rhs.points.iter()).map(|(a, b)| [(a[0].0, a[0].1 - b[0].1), (a[1].0, a[1].1 - b[1].1)]).collect_vec(),
            weights: self.weights.into_iter().zip(rhs.weights.iter()).map(|(a, b)|  [(a[0].0, a[0].1 - b[0].1), (a[1].0, a[1].1 - b[1].1)]).collect_vec()
        }
    }
}

impl std::ops::Mul<f64> for CEquationParameters {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self::Output {
        Self {
            coefficients: self.coefficients.into_iter().map(|a| a * rhs).collect_vec(),
            points: self.points.into_iter().map(|a| [(a[0].0, a[0].1 * rhs), (a[1].0, a[1].1 * rhs)]).collect_vec(),
            weights: self.weights.into_iter().map(|a| [(a[0].0, a[0].1 * rhs), (a[1].0, a[1].1 * rhs)]).collect_vec()
        }
    }
}

impl CEquationParameters {
    pub fn new(points: Vec<[(Index<Node>, SVector<f64, 3>); 2]>, weights: Vec<[(Index<Node>, f64); 2]>, coefficients: Vec<SVector<f64, 3>>) -> Self {
        assert_eq!(points.len(), weights.len());
        assert_eq!(points.len(), coefficients.len());
        let coefficients = coefficients.into_iter().map(|v| v.insert_row(3, -1f64)).collect();
        Self { points, weights, coefficients }
    }

    /// Produces the vector representing the parameters of length 11 * N where N is the number of edges
    pub fn to_vec(&self) -> (DVector<f64>, Vec<Index<Node>>) {
        let (maps, weights): (Vec<_>, Vec<_>) = self.weights.iter().flatten().cloned().unzip();
        let iter = self.points.iter().flatten().flat_map(|it| it.1.data.0[0])
            .chain(weights.into_iter())
            .chain(self.coefficients.iter().flat_map(|it| it.data.0[0][0..=2].iter().cloned()));
        (DVector::from_iterator(11 * self.points.len(), iter), maps)
    }

    /// Consumes the vector representing the parameters of length 11 * N where N is the number of edges
    pub fn from_vec(vector: DVector<f64>, maps: Vec<Index<Node>>) -> Self {
        assert!(vector.len() % 11 == 0);
        let N = vector.len() / 11;
        let (points, (weights, coefficients)) = vector.data.as_slice().split_at(6 * N).apply(|(points, it)| (points, it.split_at(2 * N)));
        Self {
            points: points.chunks(3).map(|it| SVector::<f64,3>::from_row_slice(it)).into_iter().zip(maps.clone().into_iter()).map(|(a,b)| (b, a)).chunks(2).into_iter().map(|mut it| it.next_array::<2>().unwrap()).collect_vec(),
            weights: maps.into_iter().zip(weights.to_owned()).chunks(2).into_iter().map(|mut it| it.next_array::<2>().unwrap()).collect_vec(),
            coefficients: coefficients.chunks(3).map(|it| SVector::<f64,3>::from_row_slice(it).insert_row(3, -1f64)).collect_vec()
        }
    }
}

pub trait CEquationMapping {
    fn find_mapping(&self, edge: Index<Edge>) -> usize;
}

impl CEquationMapping for (Vec<(Index<Edge>, Index<Edge>)>, CEquationParameters) {
    fn find_mapping(&self, edge: Index<Edge>) -> usize { 
        self.0.iter().enumerate().find(|(_, j)| j.0 == edge || j.1 == edge).unwrap().0
    }
}

pub struct CEquations<'M, TM: TriangleMesh + 'M> {
    parameters: CEquationParameters,
    pub mapping: Vec<(Index<Edge>, Index<Edge>)>,
    mesh: &'M mut TM,
}

impl<'M, MD: MeshData, TM: TriangleMesh<Data = MD>> CEquations<'M, TM> {
    pub fn find_mapping(&self, edge: Index<Edge>) -> usize { Self::find_mapping_within(edge, &self.mapping) }

    fn find_mapping_within(edge: Index<Edge>, mapping: &Vec<(Index<Edge>, Index<Edge>)>) -> usize {
        mapping.iter().enumerate().find(|(_, j)| j.0 == edge || j.1 == edge).unwrap().0
    }
}

impl<'M, TM: TriangleMesh<Data = SubdividedTriangleData>> CEquations<'M, TM> {
    fn new(mesh: &'M mut TM) -> Self {
        let edges = mesh.current_edges_undirected();
        let N = edges.len();
        let mut mapping = Vec::<(Index<Edge>, Index<Edge>)>::with_capacity(N);
        let mut points = Vec::<[(Index<Node>, Vector3<f64>); 2]>::with_capacity(N);
        let mut weights = Vec::<[(Index<Node>, f64); 2]>::with_capacity(N);
        let mut coefficients = Vec::<Vector3<f64>>::with_capacity(N);

        for (index, edge) in edges {
            if mapping.contains(&(index, edge.opposite)) || mapping.contains(&(edge.opposite, index)) {
                continue;
            }
            mapping.push((index, edge.opposite));
            let data = mesh.triangle_data()[edge.triangle].as_ref().unwrap();
            let (a,b) = data.get_points(index);
            points.push([(edge.source, a.into()), (edge.target, b.into())]);
            weights.push([(edge.source, 1f64), (edge.target, 1f64)]);
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

    fn get_weighted_points(index: Index<Edge>, edge: &Edge, mapping: &Vec<(Index<Edge>, Index<Edge>)>, points: &Vec<[(Index<Node>, SVector<f64, 3>); 2]>, weights: &Vec<[(Index<Node>, f64); 2]>) -> ((f64, Vector3<f64>), (f64, Vector3<f64>)) {
        let (i, reverse) = mapping
            .into_iter()
            .enumerate()
            .find_map(|(i, (forward, backward))| if index == *forward { Some((i, false)) } else if index == *backward { Some((i, true)) } else { None })
            .unwrap();
        // let i = Self::find_mapping_within(edge, mapping);
        let (a,b) = (points[i], weights[i]).apply(|(points, weights)| {
            let (a,b) = (points[0].1, points[1].1);
            ((weights[0], a), (weights[1], b))
        });
        if edge.source == a.0.0 {
            ((a.0.1, a.1), (b.0.1, b.1))
        } else {
            ((b.0.1, b.1), (a.0.1, a.1))
        }
    }

    fn get_weighted_quads_for_edge_with((points, weights): (&Vec<[(Index<Node>, SVector<f64, 3>); 2]>, &Vec<[(Index<Node>, f64); 2]>), mapping: &Vec<(Index<Edge>, Index<Edge>)>, mesh: &TM, index: Index<Edge>, edge: &Edge) -> (Quad, Quad) {
        let opposite = mesh[edge.opposite].as_ref().unwrap();

        let (l_a, u_c): (Vector3<f64>, Vector3<f64>) = (mesh[edge.source].as_ref().unwrap().coordinates.into(), mesh[edge.target].as_ref().unwrap().coordinates.into());
        // let ((w_lc, l_c), (w_ua, u_a)) = Self::get_weighted_points(index, mapping, points, weights);
        let ((w_d, l_c), (w_b, u_a)) = Self::get_weighted_points(index, mesh[index].as_ref().unwrap(), mapping, points, weights);

        let find_ep = |index: Index<Edge>, edge: &Edge, node: &Vector3<f64>| {
        // let find_ep = |index: Index<Edge>| {^
            // Self::get_weighted_points(index, mapping, points, weight^s).0
            let map = Self::find_mapping_within(index, mapping);
            let (_first, _second) = points[map].apply(|it| (it[0], it[1]));
            let (w_f, w_s) = weights[map].apply(|it| (it[0], it[1]));
          
            if edge.source == _first.0 {
                (w_f, _first.1)
            } else {
                (w_s, _second.1) 
            }
            // if (_first - node).norm_squared() < ( _second - node).norm_squared() { (w_f, _first) } else { }
        };

        // let (l_d, w_ld) = find_ep(edge.previous, &l_a);
        // let (u_d, w_ud) = find_ep(edge.next, &u_c);
        // let (l_b, w_lb) = find_ep(opposite.next, &l_a);
        // let (u_b, w_ub) = find_ep(opposite.previous, &u_c);

        let (w_ld, l_d) = find_ep(edge.previous, edge, &l_a);
        let (w_ud, u_d) = find_ep(edge.next, edge, &u_c);
        let (w_lb, l_b) = find_ep(opposite.next, edge, &l_a);
        let (w_ub, u_b) = find_ep(opposite.previous, edge, &u_c);

        let (mut u, mut l) = (Quad::from(RealNode::C, u_a, u_b, u_c, u_d), Quad::from(RealNode::A, l_a, l_b, l_c, l_d));
        u.set_weights(w_b, w_ub.1, 1.0f64, w_ud.1);
        l.set_weights(1.0f64, w_lb.1, w_d, w_ld.1);
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

    fn evaluate_parameter(&self, parameters: &CEquationParameters) -> DVector<f64> {
        let mut results = Vec::<(Index<Edge>, SVector<f64, 8>)>::with_capacity(parameters.points.len());
        for (index, edge) in self.mesh.current_edges() {
            if results.iter().find(|(i, _)| *i == index || *i == edge.opposite).is_some() {
                continue
            }
            let matrix = Self::point_weight_matrix_of_edge_with_parameters(self, parameters, index, &edge);
            // let (matrix, b) = self.point_weight_matrix_of_edge(index, &edge)
            //     .data.0
            //     .split_at(3)
            //     .apply(|(matrix, b)| (SMatrix::<f64, 8, 3>::from_column_slice(matrix.as_flattened()), SVector::<f64, 8>::from_column_slice(&b[0])));
            let i = self.find_mapping(index);
            let coefficients = parameters.coefficients[i];
            results.push((index, matrix * coefficients));
            // results.push((index, matrix * coefficients));
        }
        DVector::from_iterator(8 * parameters.coefficients.len(),  results.into_iter().flat_map(|(_, vector)| vector.data.0[0]))
    }

    fn point_weight_matrix_of_edge_with_parameters(&self, parameters: &CEquationParameters, index: Index<Edge>, edge: &Edge) -> SMatrix<f64, 8, 4> {
        let (upper, lower) = Self::get_weighted_quads_for_edge_with((&parameters.points, &parameters.weights), &self.mapping, &self.mesh, index, edge);
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
            .flat_map(|x| [x[0].1, x[1].1])
            .map(|x| {
                let iter = edges.iter()
                    .flat_map(|(index, edge)| self.derivative_of_edge_after_p(&x, *index, edge).transpose().data.0)
                    .flatten();
                DMatrix::<f64>::from_row_iterator(8 * N, 3, iter)
            })
            .flat_map(|x| Into::<Vec<f64>>::into(x.data));
        let der_weight_iter = self.parameters.points
            .iter()
            .flat_map(|x| [x[0].1, x[1].1])
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
        let x = self.parameters.to_vec().0;
        let b = &derivative * x - self.calculate();
        (derivative, b)
    }

    pub fn improve_values(self) -> Self {
        let maps = self.parameters.to_vec().1;
        let (derivative, b) = self.linearization();
        let x = derivative.svd(true, true).solve(&b, 10e-8).unwrap();
        Self {
            mapping: self.mapping.clone(),
            mesh: self.mesh,
            parameters: CEquationParameters::from_vec(x, maps)
        }
    }

    pub fn expand(&self) -> List<SVector<f64, 4>, Edge> {
        let mut coefficients = List::with_defaults(self.mesh.edges().len());

        for (index, edge) in self.mesh.current_edges() {
            let (coefficient, reverse) = self.mapping
                .iter()
                .enumerate()
                .find_map(|(i, (e,o))| if *e == index { Some((i, false)) } else if *o == index { Some((i, true)) } else { None })
                .unwrap();
            let coefficient = self.parameters.coefficients[coefficient];
            coefficients[index] = Some(
                if !reverse { coefficient } else { 
                    let [a, b, c, d] = coefficient.data.0[0];
                    // SVector::<f64, 4>::new(a / (-b), d / b, c / (-b), -1f64)
                    SVector::<f64, 4>::new(c / (-b), d / b, a / (-b), -1f64)
                    
                
                } 
            )
        }

        coefficients
    }
}

impl<'M, TM: TriangleMesh<Data = SubdividedTriangleData>> LeastSquaresProblem<f64, Dyn, Dyn> for CEquations<'M, TM> {
    type ResidualStorage = nalgebra::storage::Owned<f64, Dyn>;
    type JacobianStorage = nalgebra::storage::Owned<f64, Dyn, Dyn>;
    type ParameterStorage = nalgebra::storage::Owned<f64, Dyn>;

    fn set_params(&mut self, x: &nalgebra::Vector<f64, Dyn, Self::ParameterStorage>) {
        self.parameters = CEquationParameters::from_vec(x.clone(), self.parameters.to_vec().1)
    }

    fn params(&self) -> nalgebra::Vector<f64, Dyn, Self::ParameterStorage> {
        self.parameters.to_vec().0
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

    fn icosahedron() -> ClosedTriangleMesh {
        let mut builder = MeshBuilder::default();
        let nodes: Vec<_> = vec![
            [6.0,2.0257311121191335,0.85065080835204],
            [6.0,2.0257311121191335,-0.85065080835204],
            [6.0,0.9742688878808664,0.85065080835204],
            [6.0,0.9742688878808664,-0.85065080835204],
            [6.525731112119134,2.35065080835204,0.0],
            [6.525731112119134,0.64934919164796,0.0],
            [5.474268887880866,2.35065080835204,0.0],
            [5.474268887880866,0.64934919164796,0.0],
            [6.85065080835204,1.5,0.5257311121191336],
            [6.85065080835204,1.5,-0.5257311121191336],
            [5.14934919164796,1.5,0.5257311121191336],
            [5.14934919164796,1.5,-0.5257311121191336],
        ].into_iter().map(|node| {
            builder.add_node(UnfinishedNode::new(node))
        }).collect();
        vec![
            [0, 2, 8],
            [0, 4, 6],
            [0, 6, 10],
            [0, 8, 4],
            [0, 10, 2],
            [1, 3, 11],
            [1, 4, 9],
            [1, 6, 4],
            [1, 9, 3],
            [1, 11, 6],
            [2, 5, 8],
            [2, 7, 5],
            [2, 10, 7],
            [3, 5, 7],
            [3, 7, 11],
            [3, 9, 5],
            [4, 8, 9],
            [5, 9, 8],
            [6, 11, 10],
            [7, 10, 11],
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
    pub fn good_init_values() {
        let mesh = icosahedron();
        let mut submesh = mesh.subdivide();
        let equations = submesh.calculate_projective_structure(1e-9, 100, 0, CalculationProjectiveStructureStepsize::Break(0.125f64)).0;
        assert!(equations.calculate().norm() < 1e-9)
    }

    #[test]
    pub fn jacobian_correct() {
        let mesh = tetraeder();
        let mut submesh = mesh.subdivide();

        println!("number of edges: {}", submesh.current_edges_undirected().len());
        let mut equations = CEquations::new(&mut submesh);
        let jacobian = equations.jacobian().unwrap();
        let num = levenberg_marquardt::differentiate_numerically(&mut equations).unwrap();
        for (i, elem) in jacobian.iter().enumerate() {
            assert!((*elem - num[i]).abs() < 1e-10, "error of {} at index {i} with: jac: {elem}, num: {}", (*elem - num[i]).abs(), num[i]);
        }
        println!("eps: {}", (&num - &jacobian).norm());
        approx::assert_relative_eq!(num, jacobian, epsilon = 1e-10);
    }

    #[test]
    pub fn jacobian_correct_cube() {
        let mesh = cube();
        let mut submesh = mesh.subdivide();

        println!("number of edges: {}", submesh.current_edges_undirected().len());
        let mut equations = CEquations::new(&mut submesh);
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
        let mut submesh = mesh.subdivide();
        let mut equations = CEquations::new(&mut submesh);
        let N = equations.mapping.len();
        let initial = equations.parameters.to_vec();
        let mut iteration = 0;
        let mut matrix = DMatrix::from_diagonal_element(11 * N, 11 * N, 1.0f64).resize(19 * N, 19 * N, 0f64);
        // let mut b = DVector::from_element(19 * N, 0f64);
        let (mut b, maps) = equations.parameters.to_vec();
        let mut b = b.resize_vertically(19 * N, 0f64);

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

            let solution = CEquationParameters::from_vec(matrix.clone().qr().solve(&b).unwrap().rows(0, 11*N).into(), maps.clone());
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
        let mut submesh = mesh.subdivide();
        let structure = submesh.calculate_projective_structure(1e-12, 100, 1, CalculationProjectiveStructureStepsize::Break(0.125f64)).0;
        for (index, edge) in structure.mesh.current_edges_undirected() {
            let matrix = structure.point_weight_matrix_of_edge(index, &edge);
            let coefficients = structure.parameters.coefficients[structure.find_mapping(index)];
            assert_relative_eq!(SVector::zeros(), matrix * coefficients, epsilon = 1e-12);
            let b = structure.mesh[edge.previous].as_ref().unwrap().source;
            let d = structure.mesh[edge.opposite].as_ref().unwrap().apply(|it| structure.mesh[it.next].as_ref().unwrap().target);
            let points = [edge.source, b, edge.target].map(|it| structure.mesh[it].as_ref().unwrap().coordinates);
            assert_relative_eq!(SVector::<f64, 3>::from(structure.mesh[d].as_ref().unwrap().coordinates).insert_row(0, 1f64), SMatrix::<f64, 3, 3>::from_column_slice(points.as_flattened()).insert_row(0, 1f64) * coefficients.fixed_rows::<3>(0), epsilon = 1e-10);
        }
    }

    #[test]
    pub fn cube_projective_structure_correct() {
        pretty_env_logger::init();
        let mesh = cube();
        let mut submesh = mesh.subdivide();
        let structure = submesh.calculate_projective_structure(1e-14, 10000, 1, CalculationProjectiveStructureStepsize::Break(0.125f64)).0;
        for (index, edge) in structure.mesh.current_edges_undirected() {
            let matrix = structure.point_weight_matrix_of_edge(index, &edge);
            let coefficients = structure.parameters.coefficients[structure.find_mapping(index)];
            assert_relative_eq!(SVector::zeros(), matrix * coefficients, epsilon = 1e-12)
        }
    }

    #[test]
    pub fn icosahedron_projective_structure_correct() {
        pretty_env_logger::init();
        let mesh = icosahedron();
        let mut submesh = mesh.subdivide();
        let structure = submesh.calculate_projective_structure(1e-15, 10000, 1, CalculationProjectiveStructureStepsize::Reset(0.125f64)).0;
        for (index, edge) in structure.mesh.current_edges_undirected() {
            let matrix = structure.point_weight_matrix_of_edge(index, &edge);
            let coefficients = structure.parameters.coefficients[structure.find_mapping(index)];
            assert_relative_eq!(SVector::zeros(), matrix * coefficients, epsilon = 1e-12)
        }
    }
}