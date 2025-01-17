use std::collections::HashMap;

use apply::Apply;
use nalgebra::{Matrix4, Matrix4x1, MatrixMN, SMatrix, Vector3, Vector4};

use crate::{mesh::{Edge, Index, List, MeshData, MeshTriangleInfo, Node, Triangle}, ClosedTriangleMesh};

pub enum RealNode {
    A, C
}

pub struct Quad {
    pub real_node: RealNode,
    pub a: Vector4<f32>,
    pub b: Vector4<f32>,
    pub c: Vector4<f32>,
    pub d: Vector4<f32>,
}

impl Quad {
    fn from(real_node: RealNode, a: [f32; 3], b: [f32; 3], c: [f32; 3], d: [f32; 3]) -> Self {
        Self {
            real_node,
            a: Vector3::from(a).insert_row(0, 1f32),
            b: Vector3::from(b).insert_row(0, 1f32),
            c: Vector3::from(c).insert_row(0, 1f32),
            d: Vector3::from(d).insert_row(0, 1f32),
        }
    }
}

pub struct SubdividedTriangle {
    pub a: Vector4<f32>,
    pub b: Vector4<f32>,
    pub c: Vector4<f32>,
    pub d: Vector4<f32>,
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
        let lower_triangle = [(lower, 1f32), (lower + left / 3f32, 1.1f32), (lower + upper / 3f32, 1.1f32), (lower + right / 3f32, 1.1f32)].map(|(vec, weight)| vec.insert_row(0, weight));
        let upper_triangle = [(upper, 1f32), (upper + right / 3f32, 1.1f32), (upper + lower / 3f32, 1.1f32), (upper + left / 3f32, 1.1f32)].map(|(vec, weight)| vec.insert_row(0, weight));

        [lower_triangle, upper_triangle].map(|[a,b,c,d]| SubdividedTriangle { a, b, c, d })
    }
}

type SubdividedTriangleData = [[[f32; 3]; 3]; 3];

impl MeshData for SubdividedTriangleData {
    fn build_mesh(mesh: &ClosedTriangleMesh<Self>) -> Option<crate::mesh::MeshTriangleInfo> {
        let (nodes, colors): (Vec<[f32; 3]>, Vec<[u8; 4]>) = mesh.triangle_data.clone().take()
            .into_iter()
            .filter(|it| it.is_some())
            .flatten()
            .flatten()
            .map(|tri| {
                let nodes: Vec<_> = tri.into_iter().map(|it| (it, [rand::random(), rand::random(), rand::random(), 128])).collect::<Vec<_>>();
                // let normal = (Vector3::from(nodes[1]) - Vector3::from(nodes[0])).cross(&(Vector3::from(nodes[2]) - Vector3::from(nodes[0])));

                nodes
            })
            .flatten()
            .unzip();

        let triangles = (0..nodes.len()).into_iter().zip(colors).collect();
        Some(MeshTriangleInfo { nodes, triangles })
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
                let points: [[f32; 3]; 3]  = [a, a + (b - a)/3f32, a + (c - a)/3f32].map(|it| [it.x, it.y, it.z]);
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
    pub fn calculate_c_conditions_of_edge(&self, index: Index<Edge>) -> Vector4<f32> {
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

            let upper_quad = Quad::from(RealNode::A, u_a, u_b, u_c, u_d);
            let lower_quad = Quad::from(RealNode::C, l_a ,l_b, l_c,l_d);
            (upper_quad, lower_quad)
        };
        let upper = Matrix4::from_columns(&[upper_quad.a, upper_quad.b, upper_quad.c, upper_quad.d]);
        let lower = Matrix4::from_columns(&[lower_quad.a, lower_quad.b, lower_quad.c, lower_quad.d]);
        let b = SMatrix::<f32, 8, 1>::from_iterator([4.0f32, 0f32, 0f32, 0f32, 4f32, 0f32, 0f32, 0f32].into_iter());
        println!("{upper}");
        println!("{lower}");
        
        let matrix =  SMatrix::<f32, 8, 4>::from_row_iterator(upper.transpose().into_iter().chain(lower.transpose().into_iter()).cloned());
        println!("{matrix}");
        let svd = matrix.svd(true, true);
        let coefficient = svd .solve(&b, 10e-8).unwrap();
        let b = matrix * coefficient;
        println!("{coefficient}, error: {b}");
        coefficient
    }

    pub fn calculate_c_conditions(&self) -> List<Vector4<f32>, Edge> {
        let mut coefficients = List::<Vector4<f32>, Edge>::with_defaults(self.edges.len());
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
}

#[cfg(test)]
mod tests {
    use crate::mesh::{FromMeshBuilder, MeshBuilder, UnfinishedNode};

    use super::*;

    fn tetraeder() -> ClosedTriangleMesh {
        let mut builder = MeshBuilder::default();
        let nodes: Vec<_> = vec![
            [-1.0f32, 0f32, 0f32],
            [1.0f32, 0f32, 0f32],
            [0f32,1.0f32, 0f32],
            [0f32, 0f32, 1.0f32],
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