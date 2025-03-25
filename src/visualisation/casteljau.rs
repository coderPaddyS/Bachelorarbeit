use nalgebra::{SMatrix, SVector, Vector3};

use super::{bernstein::EvaluateBersteinPolynomial, genum::{Genum, GenumNorm}};

pub struct CasteljauInformation<'a> {
    degree: u8,
    nodes: &'a [Vector3<f64>; 3],
    control_points: &'a [Vector3<f64>],
    genum: Genum,
    bar: Vector3<f64>,
}

impl<'a> CasteljauInformation<'a> {
    fn get_pt(&self, v: Vector3<f64>, u: &SMatrix<f64, 3, 3>) -> Vector3<f64> {
        let pt = u * v;
        <Vector3<f64> as GenumNorm>::normalize(&pt, self.genum)
    }
    fn get_bar(&self, v: Vector3<f64>, u: &SMatrix<f64, 3, 3>) -> Vector3<f64> {
        if u.column_iter().find(|u| <Vector3<f64> as GenumNorm>::norm(&u.clone_owned(), self.genum) > 1e-3).is_some() {
            panic!("the points represented by u are not on a parametric plane of genum {:?}: {}", self.genum, u);
        }
        u.qr().solve(&v).unwrap()
    }

    pub fn casteljau(self) -> Vector3<f64> {
        let nodes = SMatrix::from_columns(self.nodes);
        let u = self.get_pt(self.bar, &nodes);
        let bar = self.get_bar(u, &nodes);
        let pt = SVector::zeros();

        for i in 0..self.degree {
            for j in 0..(self.degree - i) {
                let bernstein_value = bar.eval_bernstein_polynomial(i, j, self.degree - i - j);
            }
        }

        pt
    }
}