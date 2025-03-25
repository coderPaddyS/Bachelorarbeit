use nalgebra::{ComplexField, Scalar, Vector3};
use num::Float;

#[derive(Debug, Clone, Copy)]
pub enum Genum {
    Torus,
    Sphere,
    Hyperbolic
}

pub trait GenumNorm {
    fn normalize(&self, genum: Genum) -> Self;
    fn norm(&self, genum: Genum) -> f64;
}

impl GenumNorm for Vector3<f64> {
    fn normalize(&self, genum: Genum) -> Self {
        self / <Vector3<f64> as GenumNorm>::norm(self, genum)
    }
    fn norm(&self, genum: Genum) -> f64 {
        match genum {
            Genum::Hyperbolic => num::Float::sqrt(self[2] * self[2] - self[0] * self[0] - self[1] * self[1]),
            Genum::Sphere => self.norm(),
            Genum::Torus => 1f64
        }
    }
}
