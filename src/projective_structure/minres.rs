use nalgebra::{DMatrix, DVector};
use num::pow::Pow;

pub trait MinResSolver  {
    fn solve(&self, b: &DVector<f64>, init: &DVector<f64>, maxit: usize, error: f64) -> DVector<f64>;
}

impl MinResSolver for DMatrix<f64> {
    fn solve(&self, b: &DVector<f64>, init: &DVector<f64>, maxit: usize, error: f64) -> DVector<f64> {
        let mut r = b - self * init;
        let mut x = init.clone();
        let mut p0 = r.clone();
        let mut s0 = self * p0.clone();
        let mut p1 = p0.clone();
        let mut s1 = s0.clone();

        for iter in 0..maxit {
            let p2 = p1.clone();
            p1 = p0.clone();
            let s2 = s1.clone();
            s1 = s0.clone();
            println!("norm of s1: {}", s1.norm());
            println!("norm of s2: {}", s2.norm());

            let alpha = r.dot(&s1) / s1.norm_squared();
            x = x + alpha * &p1;
            r = r - alpha * &s1;

            if r.norm_squared() < error * error {
                break;
            }

            p0 = s1.clone();
            s0 = self * &s1;
            let beta1: f64 = s0.dot(&s1) / s1.norm_squared();
            p0 = p0 - beta1 * &p1;
            s0 = s0 - beta1 * &s1;
            
            if iter > 1 {
                let beta2 = s0.dot(&s2) / s2.norm_squared();
                p0 = p0 - beta2 * &p2;
                s0 = s0 - beta2 * &s2;
            }

            println!("norm of r: {}", r.norm())
        }
        x
    }
}

pub trait GMRESSolver  {
    fn gmres(&self, b: &DVector<f64>, init: &DVector<f64>, maxit: usize, error: f64) -> DVector<f64>;
}

impl GMRESSolver for DMatrix<f64> {
    fn gmres(&self, b: &DVector<f64>, init: &DVector<f64>, maxit: usize, tol: f64) -> DVector<f64> {
        let n = b.len();
        assert_eq!(init.len(), n);
        let iter = maxit.min(n);

        let r0 = b - self * (if *init == DVector::zeros(n) { b } else { init });
        let mut x = init.clone();

        if r0.norm_squared() <= tol * tol {
            return x;
        }

        let mut H = DMatrix::<f64>::zeros(iter + 1, iter);
        let mut V = DMatrix::<f64>::zeros(n, iter + 1);
        V.set_column(0, &r0.normalize());

        let mut cn = DVector::<f64>::zeros(iter);
        let mut sn = DVector::<f64>::zeros(iter);

        let mut gamma = DVector::<f64>::zeros(iter + 1);
        gamma[0] = r0.norm();

        let mut error = gamma[0]/b.norm();

        let mut last = 0;

        for k in 1..iter {
            let mut qk: DVector<f64> = self * V.column(k-1);
            for i in 1..k {
                H[(i-1,k-1)] = V.column(i-1).dot(&qk);
                qk -= H[(i-1,k-1)] * V.column(i-1);
            }
            H[(k,k-1)] = qk.norm();
            V.set_column(k, &(qk / H[(k,k-1)]));

            for i in 1..(k-1) {
                let temp = cn[i] * H[(i-1,k-1)] + sn[i]*H[(i,k-1)];
                H[(i,k-1)] = -sn[i]*H[(i-1,k-1)] + cn[i]*H[(i,k-1)];
                H[(i-1,k-1)] = temp;
            }

            let t = (H[(k-1,k-1)].powi(2) + H[(k,k-1)].powi(2)).sqrt();
            cn[k-1] = H[(k-1,k-1)] / t;
            sn[k-1] = H[(k,k-1)] / t;
            H[(k-1,k-1)] = t;
            println!("cn[k]: {}", cn[k-1]);
            println!("sn[k]: {}", sn[k-1]);

            H[(k-1,k-1)] = cn[k-1] * H[(k-1,k-1)] + sn[k-1] * H[(k,k-1)];
            H[(k,k-1)] = 0f64;

            gamma[k] = -sn[k]*gamma[k-1];
            gamma[k] *= cn[k];

            let err = gamma[k].abs() / b.norm();
            if err <= tol {
                last = k-1;
                break
            }
        }

        let y = H.view((0,0), (last, last)).solve_lower_triangular(&DVector::from_row_slice(&gamma.as_slice()[0..last])).unwrap();
        println!("y: {}", y);
        x = init + V.columns(0, last) * y;
        x
    }
}
