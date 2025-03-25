use apply::Apply;
use bevy::prelude::IntoSystem;
use nalgebra::Vector3;

use super::{ClosedTriangleMesh, Edge, Index, MeshData, Node, Triangle, TriangleMesh, TriangleMeshHalfEdgeCollapse};

pub trait TriangleRoundness {
    fn roundness(&self) -> f32;
}

impl TriangleRoundness for (Vector3<f32>, Vector3<f32>, Vector3<f32>) {
    fn roundness(&self) -> f32 {
        let (a, b, c) = self;
        let (ab, bc, ca) = ((b - a), (c - b), (a - c));

        // This can be improved performance vise by estimating the inequality, e.g. removing the implicit roots as thei're monotonous.
        let (l_ab, l_bc, l_ca) = (ab.norm(), bc.norm(), ca.norm());
        let semi_perimeter = 0.5f32 * (l_ab + l_bc + l_ca);
        let area = (semi_perimeter * (semi_perimeter - l_ab) * (semi_perimeter - l_bc) * (semi_perimeter - l_ca)).sqrt();
        let radius = area / semi_perimeter;
        ab.angle(&bc).max(bc.angle(&ca)).max(ca.angle(&ab)) / radius
    }
}

impl TriangleRoundness for (Vector3<f64>, Vector3<f64>, Vector3<f64>) {
    fn roundness(&self) -> f32 {
        let (a, b, c) = self;
        let (ab, bc, ca) = ((b - a), (c - b), (a - c));

        // This can be improved performance vise by estimating the inequality, e.g. removing the implicit roots as thei're monotonous.
        let (l_ab, l_bc, l_ca) = (ab.norm(), bc.norm(), ca.norm());
        let semi_perimeter = 0.5f64 * (l_ab + l_bc + l_ca);
        let area = (semi_perimeter * (semi_perimeter - l_ab) * (semi_perimeter - l_bc) * (semi_perimeter - l_ca)).sqrt();
        let radius = area / semi_perimeter;
        (ab.angle(&bc).max(bc.angle(&ca)).max(ca.angle(&ab)) / radius) as f32
    }
}

impl TriangleRoundness for [[f32; 3]; 3] {
    fn roundness(&self) -> f32 {
        (Vector3::from(self[0]), Vector3::from(self[1]), Vector3::from(self[2])).roundness()
    }
}

impl TriangleRoundness for [[f64; 3]; 3] {
    fn roundness(&self) -> f32 {
        (Vector3::from(self[0]), Vector3::from(self[1]), Vector3::from(self[2])).roundness()
    }
}

impl TriangleRoundness for ([f32; 3], [f32; 3], [f32; 3]) {
    fn roundness(&self) -> f32 {
        [self.0, self.1, self.2].roundness()
    }
}

impl TriangleRoundness for ([f64; 3], [f64; 3], [f64; 3]) {
    fn roundness(&self) -> f32 {
        [self.0, self.1, self.2].roundness()
    }
}

impl<TM: TriangleMesh> TriangleRoundness for (&TM, &Triangle) {
    fn roundness(&self) -> f32 {
        let (mesh, triangle) = self;
        triangle.corners.map(|node| mesh[node].as_ref().unwrap().coordinates).roundness()
    }
}

impl<TM: TriangleMesh> TriangleRoundness for (&TM, Index<Triangle>) {
    fn roundness(&self) -> f32 {
        (self.0, self.0[self.1].as_ref().unwrap()).roundness()
    }
}

pub trait ContractionOrder {
    fn calculate_contraction_order(&self) -> Option<usize>;
}

impl<TD: MeshData> ContractionOrder for (&ClosedTriangleMesh<TD>, Index<Edge>) {
    fn calculate_contraction_order(&self) -> Option<usize> {
        let (mesh, edge) = self;
        let (s,t) = mesh[*edge].as_ref().unwrap().apply(|edge| (edge.source, edge.target));
        let contracted_mesh = mesh.simulate_contract_edge(*edge);

        // Check the resulting dihedral angle between any two facettes where s was a part of
        // If the resulting dihedral angle is too large, the contraction is deemed to be invalid to reduce artifacts resulting due geometrics defecies, e.g. facettes collapsing on each other.
        let uncontractable = mesh[s].as_ref().unwrap().outgoing.iter()
            .filter_map(|edge| contracted_mesh[*edge].as_ref())
            .map(|edge| { println!("dihedral_angle: {}", (&contracted_mesh, edge).dihedral_angle().to_degrees()); edge })
            .map(|edge| (&contracted_mesh, edge).dihedral_angle().to_degrees() < 45f64)
            .any(|contractable| contractable);
        // if uncontractable {
        //     return None;
        // }
        let order = mesh[s].as_ref().unwrap().outgoing.iter()
            .filter_map(|edge| {
                contracted_mesh[*edge]
                    .as_ref()
                    .map(|edge| edge.triangle)
            })
            .filter_map(|triangle| {
                if let Some(triangle) = contracted_mesh[triangle].as_ref() {
                    Some((&contracted_mesh, triangle).roundness())
                } else {
                    None
                }
            })
            .sum::<f32>() as usize;
        Some(order)
    }
}

pub trait DihedralAngle {
    fn dihedral_angle(&self) -> f64;
}

impl DihedralAngle for (Vector3<f64>, Vector3<f64>) {
    fn dihedral_angle(&self) -> f64 {
        if self.0 == self.1 {
            return 0f64
        }
        let angle = self.0.angle(&self.1);
        return if angle == 0f64 { 180f64 } else { angle }
    }
}

impl DihedralAngle for ([Vector3<f64>; 3], [Vector3<f64>; 3]) {
    fn dihedral_angle(&self) -> f64 {
        ((self.0[1] - self.0[0]).cross(&(self.0[2] - self.0[0])), (self.1[1] - self.1[0]).cross(&(self.1[2] - self.1[0]))).dihedral_angle()
    }
}

impl<TD: MeshData> DihedralAngle for (&ClosedTriangleMesh<TD>, Index<Edge>) {
    fn dihedral_angle(&self) -> f64 {
        let edge = self.0[self.1].as_ref().unwrap();
        (self.0, edge).dihedral_angle()
    }
}

impl<TM: TriangleMesh> DihedralAngle for (&TM, &Edge) {
    fn dihedral_angle(&self) -> f64 {
        let edge = self.1;
        let opposite = self.0[edge.opposite].as_ref().unwrap();
        let (left, right) = (self.0[edge.triangle].as_ref().unwrap().corners, self.0[opposite.triangle].as_ref().unwrap().corners);
        let triangles: ([Vector3<f64>; 3], [Vector3<f64>; 3]) = (left.map(|n| Vector3::from(self.0[n].as_ref().unwrap().coordinates)), right.map(|n| Vector3::from(self.0[n].as_ref().unwrap().coordinates)));
        triangles.dihedral_angle()
    }
}