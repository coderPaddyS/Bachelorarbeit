use nalgebra::Vector3;
use num::pow::Pow;

trait Factorial {
    fn factorial(&self) -> u64;
}

impl Factorial for u8 {
    fn factorial(&self) -> u64 {
        (1..=(*self as u64)).product()
    }
}

pub trait EvaluateBersteinPolynomial {
    fn eval_bernstein_polynomial(&self, i: u8, j: u8, k: u8) -> f64;
}

impl EvaluateBersteinPolynomial for Vector3<f64> {
    fn eval_bernstein_polynomial(&self, i: u8, j: u8, k: u8) -> f64 {
        self[0].pow(i) * self[1].pow(j) * self[2].pow(k) * ((i + j + k).factorial() as f64 / ((i.factorial() * j.factorial() * k.factorial()) as f64))
    }
}