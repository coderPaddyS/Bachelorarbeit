use std::collections::VecDeque;

use apply::{Also, Apply};
use nalgebra::{Matrix3, RowVector3, SMatrix, SVector, Vector3};

use crate::{mesh::{Edge, Index, List, MeshData, MeshTriangleInfo, Node, Triangle, TriangleMesh, UncontractableMesh}, ClosedTriangleMesh};

use super::{structure::ProjectiveStructure, CEquationParameters, CEquations};

pub struct ProjectiveStructureVisualisation<'p, TM: UncontractableMesh> {
    structure: &'p ProjectiveStructure<TM>,
    base: [(Index<Node>, [f64; 3]); 3]
}

impl<'p, TM: UncontractableMesh> ProjectiveStructureVisualisation<'p, TM> {

    pub fn new(structure: &'p ProjectiveStructure<TM>) -> Self {
        let corners = structure.current_triangles()[0].1.corners;
        Self { 
            base: corners.map(|i| (i, structure[i].as_ref().unwrap().coordinates)),
            structure,
        }
    }

    fn calculate_opposite_with_coefficient(coefficients: SVector<f64, 3>, [a,b,c]: [[f64; 3]; 3]) -> SVector<f64, 3> {
        SMatrix::from_column_slice([a,b,c].as_flattened()) * coefficients
    }

    fn subdivide(&self, [a,b,c]: &[(Index<Node>, Vector3<f64>); 3]) -> [(Index<Node>, Index<Node>, Vector3<f64>); 6] {
        [
            (a.0, b.0,  a.1 + (b.1 - a.1)/3f64),
            (a.0, c.0,  a.1 + (c.1 - a.1)/3f64),
            (b.0, a.0,  b.1 + (a.1 - b.1)/3f64),
            (b.0, c.0,  b.1 + (c.1 - b.1)/3f64),
            (c.0, a.0,  c.1 + (a.1 - c.1)/3f64),
            (c.0, b.0,  c.1 + (b.1 - c.1)/3f64),
        ]
    }
    
    fn calculate_intersection(p0: Vector3<f64>, dir0: Vector3<f64>, p1: Vector3<f64>, dir1: Vector3<f64>) -> Option<Vector3<f64>> {
        let d = p1 - p0;
        let dir = dir1 - dir0;
        let t = d[0] / dir[0];
        return if (d[1] - t * dir[1]).abs() > 1e-9 || (d[2] - t * dir[2]).abs() > 1e-9 { 
            None
        } else {
            Some(p0 + dir0 * t)
        }
    }

    fn calculate_opposite(&self, index: Index<Edge>, triangles: &List<[(Index<Node>, Vector3<f64>); 3], Triangle>) -> (Index<Triangle>, [(Index<Node>, Vector3<f64>); 3]) {
        let edge = self.structure[index].as_ref().unwrap();
        let triangle = triangles[edge.triangle].as_ref().unwrap();
        let cps = self.subdivide(triangle);
        // let [a,b,c] = if triangle[0].0 == edge.source && triangle[1].0 == edge.target {
        //     [triangle[0], triangle[2], triangle[1]]
        // } else if triangle[0].0 == edge.source && triangle[2].0 == edge.target {
        //     [triangle[0], triangle[1], triangle[2]]
        // } else if triangle[1].0 == edge.source && triangle[2].0 == edge.target {
        //     [triangle[1], triangle[0], triangle[2]]
        // } else if triangle[1].0 == edge.source && triangle[0].0 == edge.target {
        //     [triangle[1], triangle[2], triangle[0]]
        // } else if triangle[2].0 == edge.source && triangle[0].0 == edge.target {
        //     [triangle[2], triangle[1], triangle[0]]
        // } else {
        //     [triangle[2], triangle[0], triangle[1]]
        // };
        let [a, b, c] = [triangle.iter().find(|(i,_)| *i == edge.source).unwrap(), triangle.iter().find(|(i,_)| *i != edge.source && *i != edge.target).unwrap(), triangle.iter().find(|(i,_)| *i == edge.target).unwrap()];
        let coefficients = self.structure.get_coefficients(index);
        let o = Self::calculate_opposite_with_coefficient(coefficients.fixed_rows::<3>(0).clone_owned(), [a.1.into(),b.1.into(),c.1.into()]);
        let o1 = Self::calculate_opposite_with_coefficient(coefficients.fixed_rows::<3>(0).clone_owned(), [a.1.clone().data.0[0], cps.iter().find(|(s,t,p)| *s == a.0 && *t == b.0).unwrap().2.data.0[0], cps.iter().find(|(s,t,p)| *s == a.0 && *t == c.0).unwrap().2.data.0[0]]);
        let o2 = Self::calculate_opposite_with_coefficient(coefficients.fixed_rows::<3>(0).clone_owned(), [cps.iter().find(|(s,t,p)| *s == c.0 && *t == a.0).unwrap().2.data.0[0], cps.iter().find(|(s,t,p)| *s == c.0 && *t == b.0).unwrap().2.data.0[0], c.1.clone().data.0[0]]);

        let opp = self.structure[edge.opposite].as_ref().unwrap();
        let idx_opp = self.structure[opp.next].as_ref().unwrap().target;
        // let intersection = Self::calculate_intersection(a.1, a.1 - o1, c.1, c.1 - o2).expect("No intersection of the conditions");
        (
            opp.triangle, 
            [a.clone(), c.clone(), (idx_opp, o)]
            // [a.clone(), c.clone(), (idx_opp, intersection)]
        )
    }

    fn calculate_neighbours(&self, index: Index<Node>, edge: Index<Edge>, triangle: [(Index<Node>, Vector3<f64>); 3]) -> List<[(Index<Node>, Vector3<f64>); 3], Triangle> {
        assert!(triangle.iter().find(|(i, _)| *i == index).is_some());
        assert!(self.structure[index].as_ref().unwrap().outgoing.iter().find(|e| **e == edge).is_some());
        
        let mut idx_edge = edge;
        let edge = self.structure[edge].as_ref().unwrap();
        assert!(edge.source == index);
        
        let mut triangles = List::with_defaults(self.structure.current_edges().len());
        triangles[edge.triangle] = Some(triangle);


        loop {
            let (index, triangle) = self.calculate_opposite(idx_edge, &triangles);
            if triangles[index].is_some() {
                break;
            }
            triangles[index] = Some(triangle);
            idx_edge = self.structure[idx_edge].as_ref().unwrap().opposite.apply(|it| self.structure[it].as_ref().unwrap().next);
        }

        triangles
    }
}

impl<'p, TM: UncontractableMesh> VisualiseProjectiveStructure for ProjectiveStructureVisualisation<'p, TM> {
    fn visualise(&self) -> MeshTriangleInfo {
        let mut queue = VecDeque::new();
        let [(a,_), (b,_), (c, _)] = self.base;
        let ab = self.structure.find_edge(a, b).unwrap();
        let triangle = self.base.map(|(n, c)| (n, c.into()));
        queue.push_back((a, ab, triangle));
        queue.push_back((b, self.structure[ab].as_ref().unwrap().next, triangle));
        queue.push_back((c, self.structure[ab].as_ref().unwrap().previous, triangle));

        let mut triangles: Vec<[(Index<Node>, Vector3<f64>); 3]> = Vec::new();

        while let Some((node, edge, triangle)) = queue.pop_front() {
            self.calculate_neighbours(node, edge, triangle).take()
                .into_iter()
                .filter_map(|i| i)
                .for_each(|i| triangles.push(i));
        }

        let (triangles, nodes): (Vec<usize>, Vec<[f32; 3]>) = triangles
            .into_iter()
            .flatten()
            .enumerate()
            // .map(|(i, a)| (i, a.1.normalize().data.0[0].map(|it| it as f32)))
            .map(|(i, a)| (i, a.1.data.0[0].map(|it| it as f32)))
            .unzip();
        let triangles = triangles
            .into_iter()
            .map(|it| (it, [rand::random(), rand::random(), rand::random(), 128])).collect();
        MeshTriangleInfo {
            triangles,
            nodes,
        }
    }
}

pub trait VisualiseProjectiveStructure {
    fn visualise(&self) -> MeshTriangleInfo;
}

impl<'M, TD: MeshData, TM: TriangleMesh<Data = TD>> VisualiseProjectiveStructure for CEquations<'M, TM> {
    fn visualise(&self) -> MeshTriangleInfo {
        let mut triangles = List::<[(Index<Node>, Vector3<f64>); 3], Triangle>::with_defaults(self.mesh.triangles().len());

        // let triangle = self.mesh.current_triangles()[0].clone();
        // triangles[triangle.0] = Some(triangle.1.corners.map(|it| (it, self.mesh[it].as_ref().unwrap().coordinates.into())));
        // // let triangles = generate_triangles(self, triangle.0, triangles);

        // let init = self.mesh.current_triangles()[0].clone();
        // let mut init_edge = self.mesh.find_edge(init.1.corners[0], init.1.corners[1]).unwrap().apply(|it| (it, self.mesh[it].as_ref().unwrap()));
        // // let triangles: List<[(Index<Node>, Vector3<f64>); 3], Triangle> = List::with_defaults(self.mesh.triangles().len()).also(|it| it[triangle.0] = Some(triangle.1.corners.map(|it| (it, self.mesh[it].as_ref().unwrap().coordinates.into()))));
        // let mut triangles: Vec<[(Index<Node>, Vector3<f64>); 3]> = vec![triangle.1.corners.map(|it| (it, self.mesh[it].as_ref().unwrap().coordinates.into()))];

        
        // let mut queue: VecDeque<(usize, Index<Edge>)> = VecDeque::new();
        // queue.push_back((0, init_edge.0));
        
        // while let Some((init_triangle, idx_edge)) = queue.pop_front() {
        //     let mut prev_triangle = &triangles[init_triangle];
        //     let mut prev_triangle_index = init_triangle;
        //     let mut curr_edge = idx_edge;
        //     loop {
        //         let edge = self.mesh[curr_edge].as_ref().unwrap();

        //         let s = prev_triangle.iter().find(|it| it.0 == edge.source).unwrap().1;
        //         let t = prev_triangle.iter().find(|it| it.0 == edge.target).unwrap().1;
        //         let opp = prev_triangle.iter().find(|it| it.0 != edge.source && it.0 != edge.target).unwrap().1;
        //         let [a,b,c] = &self.parameters.coefficients[self.find_mapping(curr_edge)].data.0[0][0..3] else { panic!() };
        //         let d = *a * s + *b * opp + *c * t;
                
        //         if triangles.len() < self.mesh.current_triangles().len() {
        //             queue.push_back((prev_triangle_index, edge.next));
        //         }

        //         let next_edge = self.mesh[edge.opposite].as_ref().unwrap().next;
        //         triangles.push([(edge.source, s), (edge.target, t), (self.mesh[next_edge].as_ref().unwrap().target, d)]);

        //         if next_edge == idx_edge {
        //             break;
        //         }
        //         prev_triangle = triangles.last().unwrap();
        //         prev_triangle_index = triangles.len() - 1;
        //         curr_edge = next_edge;

        //     }
        // }
        

        // fn generate_triangles<'M, TD: MeshData, TM: TriangleMesh<Data = TD>>(equations: &'M CEquations<'M, TM>, triangle: Index<Triangle>, mut triangles: List<[(Index<Node>, Vector3<f64>); 3], Triangle>) -> List<[(Index<Node>, Vector3<f64>); 3], Triangle> {
        //     let [a, b, c] = triangles[triangle].as_ref().unwrap().map(|(i, n)| (i, n));
            
        //     let (index, edge) = equations.mesh.find_edge(a.0, b.0).unwrap().apply(|it| (it, equations.mesh[it].as_ref().unwrap()));
        //     let mut next_triangles = Vec::with_capacity(3);
        //     for (index, (idx_s, s), (idx_t, t), (idx_o, o)) in [(index, a, b, c), (edge.next, b, c, a), (edge.previous, c, a, b)] {
        //         let edge = equations.mesh[index].as_ref().unwrap();
        //         let idx_d = equations.mesh[edge.opposite].as_ref().unwrap().apply(|opp| equations.mesh[opp.next].as_ref().unwrap().target);
        //         let opp = equations.mesh[edge.opposite].as_ref().unwrap().triangle; 
        //         if triangles[opp].is_some() {
        //             continue;
        //         }
        //         let mapping = equations.find_mapping(index);
        //         let coefficients = equations.parameters.coefficients[mapping].fixed_rows::<3>(0);
        //         // let matrix = SMatrix::<f64, 3, 3>::zeros().also(|it| {
        //         //     // it.view_mut((0,0), (1,3)).set_row(0, &RowVector3::from_element(1f64));
        //         //     it.view_mut((0,0), (3,1)).set_column(0, &Vector3::from_column_slice((&s).into()));
        //         //     it.view_mut((0,1), (3,1)).set_column(0, &Vector3::from_column_slice((&o).into()));
        //         //     it.view_mut((0,2), (3,1)).set_column(0, &Vector3::from_column_slice((&t).into()));
        //         // });
        //         let matrix = Matrix3::from_iterator(s.into_iter().chain(o.into_iter()).chain(t.into_iter()).cloned());
        //         let d = matrix * coefficients;
        //         triangles[opp] = Some([(idx_s, s), (idx_d, d), (idx_t, t)]);
        //         next_triangles.push(opp);
        //     }

        //     for triangle in next_triangles {
        //         triangles = generate_triangles(equations, triangle, triangles);
        //     }
        //     triangles
        // }


        fn generate_triangles<'M, TD: MeshData, TM: TriangleMesh<Data = TD>>(equations: &'M CEquations<'M, TM>, triangle: Index<Triangle>, mut triangles: List<[(Index<Node>, Vector3<f64>); 3], Triangle>) -> List<[(Index<Node>, Vector3<f64>); 3], Triangle> {
            let [a, b, c] = triangles[triangle].as_ref().unwrap().map(|(i, n)| (i, n));
            
            let (index, edge) = equations.mesh.find_edge(a.0, b.0).unwrap().apply(|it| (it, equations.mesh[it].as_ref().unwrap()));
            let mut next_triangles = Vec::with_capacity(3);
            for (index, (idx_s, s), (idx_t, t), (idx_o, o)) in [(index, a, b, c), (edge.next, b, c, a), (edge.previous, c, a, b)] {
                let edge = equations.mesh[index].as_ref().unwrap();
                let idx_d = equations.mesh[edge.opposite].as_ref().unwrap().apply(|opp| equations.mesh[opp.next].as_ref().unwrap().target);
                let opp = equations.mesh[edge.opposite].as_ref().unwrap().triangle; 
                if triangles[opp].is_some() {
                    continue;
                }
                let mapping = equations.find_mapping(index);
                let coefficients = equations.parameters.coefficients[mapping].fixed_rows::<3>(0);
                // let matrix = SMatrix::<f64, 3, 3>::zeros().also(|it| {
                //     // it.view_mut((0,0), (1,3)).set_row(0, &RowVector3::from_element(1f64));
                //     it.view_mut((0,0), (3,1)).set_column(0, &Vector3::from_column_slice((&s).into()));
                //     it.view_mut((0,1), (3,1)).set_column(0, &Vector3::from_column_slice((&o).into()));
                //     it.view_mut((0,2), (3,1)).set_column(0, &Vector3::from_column_slice((&t).into()));
                // });
                let matrix = Matrix3::from_iterator(s.into_iter().chain(o.into_iter()).chain(t.into_iter()).cloned());
                let d = matrix * coefficients;
                triangles[opp] = Some([(idx_s, s), (idx_d, d), (idx_t, t)]);
                next_triangles.push(opp);
            }

            for triangle in next_triangles {
                triangles = generate_triangles(equations, triangle, triangles);
            }
            triangles
        }
        let triangle = self.mesh.current_triangles()[0].clone();
        triangles[triangle.0] = Some(triangle.1.corners.map(|it| (it, self.mesh[it].as_ref().unwrap().coordinates.into())));
        let triangles = generate_triangles(self, triangle.0, triangles);

        let (triangles, nodes): (Vec<usize>, Vec<[f32; 3]>) = triangles
            .take()
            .into_iter()
            .filter_map(|it| it)
            .flatten()
            .enumerate()
            // .map(|(i, a)| (i, a.1.normalize().data.0[0].map(|it| it as f32)))
            .map(|(i, a)| (i, a.1.data.0[0].map(|it| it as f32)))
            .unzip();
        let triangles = triangles
            .into_iter()
            .map(|it| (it, [rand::random(), rand::random(), rand::random(), 128])).collect();
        MeshTriangleInfo {
            triangles,
            nodes,
        }
    }
}

impl VisualiseProjectiveStructure for (&ClosedTriangleMesh<()>, Vec<((Index<Edge>, Edge), SVector<f64, 4>)>) {
    fn visualise(&self) -> MeshTriangleInfo {
        let (mesh, coefficients) = self;
        let mut triangles = List::<[(Index<Node>, Vector3<f64>); 3], Triangle>::with_defaults(mesh.triangles().len());

        fn generate_triangles(mesh: &ClosedTriangleMesh, coefficients: &Vec<((Index<Edge>, Edge), SVector<f64, 4>)>, triangle: Index<Triangle>, mut triangles: List<[(Index<Node>, Vector3<f64>); 3], Triangle>) -> List<[(Index<Node>, Vector3<f64>); 3], Triangle> {
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
                // let cs = coefficients.iter().find(|it| it.0.0 == index || it.0.1.opposite == index).unwrap().1.fixed_rows::<3>(0);
                let cs = coefficients.iter().find(|it| it.0.0 == index).unwrap().1.fixed_rows::<3>(0);
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
        let triangles = generate_triangles(mesh, coefficients, triangle.0, triangles);

        let (triangles, nodes): (Vec<usize>, Vec<[f32; 3]>) = triangles
            .take()
            .into_iter()
            .filter_map(|it| it)
            .flatten()
            .enumerate()
            // .map(|(i, a)| (i, a.1.normalize().data.0[0].map(|it| it as f32)))
            .map(|(i, a)| (i, a.1.data.0[0].map(|it| it as f32)))
            .unzip();
        let triangles = triangles
            .into_iter()
            .map(|it| (it, [rand::random(), rand::random(), rand::random(), 128])).collect();
        MeshTriangleInfo {
            triangles,
            nodes,
        }
    }
}

impl<TM: TriangleMesh> VisualiseProjectiveStructure for (&TM, List<SVector<f64, 4>, Edge>) {
    fn visualise(&self) -> MeshTriangleInfo {
        let (mesh, coefficients) = self;
        let mut triangles = List::<[(Index<Node>, Vector3<f64>); 3], Triangle>::with_defaults(mesh.triangles().len());

        // let triangle = mesh.current_triangles()[0].clone();
        // triangles[triangle.0] = Some(triangle.1.corners.map(|it| (it, mesh[it].as_ref().unwrap().coordinates.into())));
        // // let triangles = generate_triangles(self, triangle.0, triangles);

        // let init = mesh.current_triangles()[0].clone();
        // let mut init_edge = mesh.find_edge(init.1.corners[0], init.1.corners[1]).unwrap().apply(|it| (it, mesh[it].as_ref().unwrap()));
        // // let triangles: List<[(Index<Node>, Vector3<f64>); 3], Triangle> = List::with_defaults(self.mesh.triangles().len()).also(|it| it[triangle.0] = Some(triangle.1.corners.map(|it| (it, self.mesh[it].as_ref().unwrap().coordinates.into()))));
        // let mut triangles: Vec<[(Index<Node>, Vector3<f64>); 3]> = vec![triangle.1.corners.map(|it| (it, mesh[it].as_ref().unwrap().coordinates.into()))];

        
        // let mut queue: VecDeque<(usize, Index<Edge>)> = VecDeque::new();
        // queue.push_back((0, init_edge.0));
        
        // while let Some((init_triangle, idx_edge)) = queue.pop_front() {
        //     let mut prev_triangle = &triangles[init_triangle];
        //     let mut prev_triangle_index = init_triangle;
        //     let mut curr_edge = idx_edge;
        //     loop {
        //         let edge = mesh[curr_edge].as_ref().unwrap();

        //         let s = prev_triangle.iter().find(|it| it.0 == edge.source).unwrap().1;
        //         let t = prev_triangle.iter().find(|it| it.0 == edge.target).unwrap().1;
        //         let opp = prev_triangle.iter().find(|it| it.0 != edge.source && it.0 != edge.target).unwrap().1;
        //         let [a,b,c] = &coefficients[curr_edge].as_ref().unwrap().data.0[0][0..3] else { panic!() };
        //         let d = *a * s + *b * opp + *c * t;
                
        //         if triangles.len() < mesh.current_triangles().len() {
        //             queue.push_back((prev_triangle_index, edge.next));
        //         }

        //         let next_edge = mesh[edge.opposite].as_ref().unwrap().next;
        //         triangles.push([(edge.source, s), (edge.target, t), (mesh[next_edge].as_ref().unwrap().target, d)]);

        //         if next_edge == idx_edge {
        //             break;
        //         }
        //         prev_triangle = triangles.last().unwrap();
        //         prev_triangle_index = triangles.len() - 1;
        //         curr_edge = next_edge;

        //     }
        // }

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
        let triangles = generate_triangles(*mesh, coefficients, triangle.0, triangles);

        let (triangles, nodes): (Vec<usize>, Vec<[f32; 3]>) = triangles
            .take()
            .into_iter()
            .filter_map(|it| it)
            .flatten()
            .enumerate()
            // .map(|(i, a)| (i, a.1.normalize().data.0[0].map(|it| it as f32)))
            .map(|(i, a)| (i, a.1.data.0[0].map(|it| it as f32)))
            .unzip();
        let triangles = triangles
            .into_iter()
            .map(|it| (it, [rand::random(), rand::random(), rand::random(), 128])).collect();
        MeshTriangleInfo {
            triangles,
            nodes,
        }
    }
}


#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use crate::{mesh::{FromMeshBuilder, MeshBuilder, UnfinishedNode}, projective_structure::{CalculateProjectiveStructure, CalculationProjectiveStructureStepsize, SubdiviveMesh}, ClosedTriangleMesh};

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
    pub fn cube_visualise() {
        pretty_env_logger::init();
        let mesh = cube();
        let mut submesh = mesh.subdivide();
        // let info = submesh.calculate_projective_structure(1e-9, 100, 1).visualise();
        let info = submesh.calculate_projective_structure(1e-9, 10, 1, CalculationProjectiveStructureStepsize::Break(0.125f64)).0.visualise();
        assert_eq!(info.nodes.len(), 3 * submesh.nodes.len())
    }

    #[test]
    pub fn tetraeder_visualise() {
        pretty_env_logger::init();
        let mesh = tetraeder();
        let mut submesh = mesh.subdivide();
        let info = submesh.calculate_projective_structure(1e-9, 100, 1, CalculationProjectiveStructureStepsize::Break(0.125f64)).0.visualise();
        assert_eq!(info.nodes.len(), 3 * submesh.nodes.len())
    }

    #[test]
    pub fn cube_default_visualise() {
        let mesh = cube();
        let info = (&mesh, mesh.calculate_projective_structure()).visualise();
        // assert_eq!(info.nodes.len(), 3 * submesh.nodes.len())
    }
}