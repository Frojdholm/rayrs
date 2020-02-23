use std::ops::{Add, Mul, Sub};

#[derive(PartialEq, Debug, Clone)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub fn new(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3 { x, y, z }
    }

    pub fn dot(&self, other: &Vec3) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn cross(&self, other: &Vec3) -> Vec3 {
        Vec3 {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    pub fn mag2(&self) -> f64 {
        self.dot(self)
    }

    pub fn mag(&self) -> f64 {
        self.mag2().sqrt()
    }

    pub fn unit(&self) -> Vec3 {
        // TODO: implement div on Vec3?
        (1. / self.mag()) * self
    }

    pub fn orthonormal_basis(normal: &Vec3) -> (Vec3, Vec3) {
        let temp = if normal.x.abs() > 0.9 {
            Vec3::new(0., 1., 0.)
        } else {
            Vec3::new(1., 0., 0.)
        };
        let e1 = temp.cross(normal).unit();
        let e2 = normal.cross(&e1).unit();
        (e1, e2)
    }

    pub fn pow(&self, n: f64) -> Vec3 {
        Vec3 {
            x: self.x.powf(n),
            y: self.y.powf(n),
            z: self.z.powf(n),
        }
    }

    pub fn clip(&self, min: f64, max: f64) -> Vec3 {
        Vec3 {
            x: self.x.min(max).max(min),
            y: self.y.min(max).max(min),
            z: self.z.min(max).max(min),
        }
    }

    pub fn rotate(&self, rot: &Vec3) -> Vec3 {
        Vec3 {
            x: self.x * rot.y.cos() * rot.z.cos()
                + self.y * (rot.z.cos() * rot.x.sin() * rot.y.sin() - rot.x.cos() * rot.z.sin())
                + self.z * (rot.x.cos() * rot.z.cos() * rot.y.sin() + rot.x.sin() * rot.z.sin()),
            y: self.x * rot.y.cos() * rot.z.sin()
                + self.y * (rot.x.cos() * rot.z.cos() + rot.x.sin() * rot.y.sin() * rot.z.sin())
                + self.z * (rot.x.cos() * rot.y.sin() * rot.z.sin() - rot.z.cos() * rot.x.sin()),
            z: -self.x * rot.y.sin()
                + self.y * rot.y.cos() * rot.x.sin()
                + self.z * rot.x.cos() * rot.y.cos(),
        }
    }
}

impl Add for &Vec3 {
    type Output = Vec3;

    fn add(self, other: &Vec3) -> Vec3 {
        Vec3 {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl Sub for &Vec3 {
    type Output = Vec3;

    fn sub(self, other: &Vec3) -> Vec3 {
        Vec3 {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl Mul for &Vec3 {
    type Output = Vec3;

    fn mul(self, other: &Vec3) -> Vec3 {
        Vec3 {
            x: self.x * other.x,
            y: self.y * other.y,
            z: self.z * other.z,
        }
    }
}

impl Mul<f64> for &Vec3 {
    type Output = Vec3;

    fn mul(self, factor: f64) -> Vec3 {
        Vec3 {
            x: self.x * factor,
            y: self.y * factor,
            z: self.z * factor,
        }
    }
}

impl Mul<&Vec3> for f64 {
    type Output = Vec3;

    fn mul(self, other: &Vec3) -> Vec3 {
        Vec3 {
            x: other.x * self,
            y: other.y * self,
            z: other.z * self,
        }
    }
}

/**
 * Testing the vector math functions. Since we're testing floating-point
 * arithmetic with the `==` operator we make sure to choose values that
 * are small integers. This way we're guaranteed to get the correct
 * results.
 */
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_add() {
        let v1 = Vec3::new(1., 2., 3.);
        let v2 = Vec3::new(2., 4., 6.);
        let v3 = &v1 + &v2;
        assert_eq!(Vec3::new(3., 6., 9.), v3);
    }

    #[test]
    fn test_sub() {
        let v1 = Vec3::new(4., 3., 2.);
        let v2 = Vec3::new(1., 1., 1.);
        let v3 = &v1 - &v2;
        assert_eq!(Vec3::new(3., 2., 1.), v3);
    }

    #[test]
    fn test_mul() {
        let v1 = Vec3::new(1., 4., 8.);
        let v2 = Vec3::new(2., 2., 2.);
        let v3 = &v1 * &v2;
        assert_eq!(Vec3::new(2., 8., 16.), v3);
    }

    #[test]
    fn test_scalar_mul() {
        let v1 = Vec3::new(1., 2., 3.);
        let v2 = 3. * &v1;
        assert_eq!(Vec3::new(3., 6., 9.), v2);
    }

    #[test]
    fn test_mul_scalar() {
        let v1 = Vec3::new(1., 2., 3.);
        let v2 = &v1 * 3.;
        assert_eq!(Vec3::new(3., 6., 9.), v2);
    }

    #[test]
    fn test_dot() {
        let v1 = Vec3::new(1., 2., 3.);
        assert_eq!(14., v1.dot(&v1));
    }

    #[test]
    fn test_cross1() {
        let v1 = Vec3::new(1., 0., 0.);
        let v2 = Vec3::new(0., 1., 0.);
        let v3 = v1.cross(&v2);
        assert_eq!(Vec3::new(0., 0., 1.), v3);
    }

    #[test]
    fn test_cross2() {
        let v1 = Vec3::new(1., 0., 0.);
        let v2 = Vec3::new(0., 0., 1.);
        let v3 = v1.cross(&v2);
        assert_eq!(Vec3::new(0., -1., 0.), v3);
    }

    #[test]
    fn test_mag2() {
        let m = Vec3::new(1., 2., 3.).mag2();
        assert_eq!(14., m);
    }

    #[test]
    fn test_pow() {
        let v = Vec3::new(1., 2., 3.).pow(2.);
        assert_eq!(Vec3::new(1., 4., 9.), v);
    }

    #[test]
    fn test_clip() {
        let v = Vec3::new(-1., 2., 0.5).clip(0., 1.);
        assert_eq!(Vec3::new(0., 1., 0.5), v);
    }
}
