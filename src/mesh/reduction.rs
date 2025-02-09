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
    fn calculate_contraction_order(&self) -> usize;
}

impl<TD: MeshData> ContractionOrder for (&ClosedTriangleMesh<TD>, Index<Edge>) {
    fn calculate_contraction_order(&self) -> usize {
        let (mesh, edge) = self;
        let (s,t) = mesh[*edge].as_ref().unwrap().apply(|edge| (edge.source, edge.target));
        let contracted_mesh = mesh.simulate_contract_edge(*edge);
        mesh[s].as_ref().unwrap().outgoing.iter()
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
            .sum::<f32>() as usize
    }
}