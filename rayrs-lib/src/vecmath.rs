use std::f64;
use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub, SubAssign};

pub trait Dot<Rhs = Self> {
    type Output;

    fn dot(self, other: Rhs) -> Self::Output;
}

pub trait Cross<Rhs = Self> {
    type Output;

    fn cross(self, other: Rhs) -> Self::Output;
}

pub trait VecElements<T> {
    fn x(self) -> T;
    fn y(self) -> T;
    fn z(self) -> T;
}

pub trait VecUnit {
    type Output;
    type VecOutput;

    fn mag(self) -> Self::Output;
    fn mag2(self) -> Self::Output;
    fn unit(self) -> Unit<Self::VecOutput>;
}

#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Vector<T> {
    data: [T; 3],
}

/// A wrapper type for unit vectors.
#[derive(PartialEq, Debug, Clone, Copy)]
pub struct Unit<T>(T);

impl Unit<Vector<f64>> {
    /// Construct a new unit vector.
    ///
    /// # Panics
    /// If the passed vector is not a unit vector.
    pub fn new(vec: Vector<f64>) -> Unit<Vector<f64>> {
        assert!((vec.mag2() - 1.).abs() < 4. * f64::EPSILON);

        Unit(vec)
    }

    /// Construct a new unit vector.
    ///
    /// # Note
    /// The programmer assures that the vector passed is in fact a unit vector.
    pub fn new_unchecked(vec: Vector<f64>) -> Unit<Vector<f64>> {
        debug_assert!((vec.mag2() - 1.).abs() < 4. * f64::EPSILON);

        Unit(vec)
    }

    /// Use this vector as a normal vector.
    pub fn as_vec(self) -> Vector<f64> {
        self.0
    }

    pub fn powf(self, n: f64) -> Vector<f64> {
        self.0.powf(n)
    }

    pub fn clip(self, min: f64, max: f64) -> Vector<f64> {
        self.0.clip(min, max)
    }

    pub fn rotate(self, rot: Vector<f64>) -> Unit<Vector<f64>> {
        Unit(self.0.rotate(rot))
    }

    pub fn xyz_in_range_inclusive(&self, min: f64, max: f64) -> bool {
        self.0.xyz_in_range_inclusive(min, max)
    }
}

impl<T: Copy> VecElements<T> for Unit<Vector<T>> {
    fn x(self) -> T {
        self.0.data[0]
    }

    fn y(self) -> T {
        self.0.data[1]
    }

    fn z(self) -> T {
        self.0.data[2]
    }
}

impl<T: Copy> VecElements<T> for Vector<T> {
    fn x(self) -> T {
        self.data[0]
    }

    fn y(self) -> T {
        self.data[1]
    }

    fn z(self) -> T {
        self.data[2]
    }
}

impl VecUnit for Unit<Vector<f64>> {
    type Output = f64;
    type VecOutput = Vector<f64>;

    fn mag(self) -> Self::Output {
        1.
    }

    fn mag2(self) -> Self::Output {
        1.
    }

    fn unit(self) -> Unit<Self::VecOutput> {
        self
    }
}

impl VecUnit for Vector<f64> {
    type Output = f64;
    type VecOutput = Self;

    fn mag(self) -> Self::Output {
        self.mag2().sqrt()
    }

    fn mag2(self) -> Self::Output {
        self.dot(self)
    }

    fn unit(self) -> Unit<Self::VecOutput> {
        Unit(self / self.mag())
    }
}

impl Dot for Vector<f64> {
    type Output = f64;

    fn dot(self, other: Vector<f64>) -> Self::Output {
        self.data[0] * other.data[0] + self.data[1] * other.data[1] + self.data[2] * other.data[2]
    }
}

impl Dot for Unit<Vector<f64>> {
    type Output = f64;

    fn dot(self, other: Unit<Vector<f64>>) -> Self::Output {
        self.0.dot(other.0)
    }
}

impl Dot<Vector<f64>> for Unit<Vector<f64>> {
    type Output = f64;

    fn dot(self, other: Vector<f64>) -> Self::Output {
        self.0.dot(other)
    }
}

impl Dot<Unit<Vector<f64>>> for Vector<f64> {
    type Output = f64;

    fn dot(self, other: Unit<Vector<f64>>) -> Self::Output {
        self.dot(other.0)
    }
}

impl Cross for Vector<f64> {
    type Output = Self;

    fn cross(self, other: Vector<f64>) -> Self::Output {
        Vector {
            data: [
                self.data[1] * other.data[2] - self.data[2] * other.data[1],
                self.data[2] * other.data[0] - self.data[0] * other.data[2],
                self.data[0] * other.data[1] - self.data[1] * other.data[0],
            ],
        }
    }
}

impl Cross for Unit<Vector<f64>> {
    type Output = Self;

    fn cross(self, other: Unit<Vector<f64>>) -> Self::Output {
        Unit(self.0.cross(other.0))
    }
}

impl Cross<Vector<f64>> for Unit<Vector<f64>> {
    type Output = Vector<f64>;

    fn cross(self, other: Vector<f64>) -> Self::Output {
        self.0.cross(other)
    }
}

impl Cross<Unit<Vector<f64>>> for Vector<f64> {
    type Output = Self;

    fn cross(self, other: Unit<Vector<f64>>) -> Self::Output {
        self.cross(other.0)
    }
}

pub type Vec3 = Vector<f64>;

impl<T: Copy> Vector<T> {
    pub fn new(x: T, y: T, z: T) -> Vector<T> {
        Vector { data: [x, y, z] }
    }

    pub fn iter(&self) -> std::slice::Iter<T> {
        self.data.iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<T> {
        self.data.iter_mut()
    }
}

impl Vector<f64> {
    pub fn ones() -> Vector<f64> {
        Vector { data: [1., 1., 1.] }
    }

    pub fn zeros() -> Vector<f64> {
        Vector { data: [0., 0., 0.] }
    }

    pub fn unit_x() -> Unit<Vector<f64>> {
        Unit(Vector { data: [1., 0., 0.] })
    }

    pub fn unit_y() -> Unit<Vector<f64>> {
        Unit(Vector { data: [0., 1., 0.] })
    }

    pub fn unit_z() -> Unit<Vector<f64>> {
        Unit(Vector { data: [0., 0., 1.] })
    }

    pub fn is_zeros(&self) -> bool {
        self.data == [0., 0., 0.]
    }

    pub fn is_ones(&self) -> bool {
        self.data == [1., 1., 1.]
    }

    pub fn orthonormal_basis(normal: Unit<Vector<f64>>) -> (Unit<Vector<f64>>, Unit<Vector<f64>>) {
        let normal = normal.as_vec();
        let temp = if normal.data[0].abs() > 0.9 {
            Vector::new(0., 1., 0.)
        } else {
            Vector::new(1., 0., 0.)
        };
        let e1 = temp.cross(normal).unit();
        let e2 = normal.cross(e1.as_vec()).unit();
        (e1, e2)
    }

    pub fn powf(self, n: f64) -> Vector<f64> {
        Vector {
            data: [
                self.data[0].powf(n),
                self.data[1].powf(n),
                self.data[2].powf(n),
            ],
        }
    }

    pub fn clip(self, min: f64, max: f64) -> Vector<f64> {
        Vector {
            data: [
                self.data[0].min(max).max(min),
                self.data[1].min(max).max(min),
                self.data[2].min(max).max(min),
            ],
        }
    }

    pub fn rotate(self, rot: Vector<f64>) -> Vector<f64> {
        Vector {
            data: [
                self.data[0] * rot.data[1].cos() * rot.data[2].cos()
                    + self.data[1]
                        * (rot.data[2].cos() * rot.data[0].sin() * rot.data[1].sin()
                            - rot.data[0].cos() * rot.data[2].sin())
                    + self.data[2]
                        * (rot.data[0].cos() * rot.data[2].cos() * rot.data[1].sin()
                            + rot.data[0].sin() * rot.data[2].sin()),
                self.data[0] * rot.data[1].cos() * rot.data[2].sin()
                    + self.data[1]
                        * (rot.data[0].cos() * rot.data[2].cos()
                            + rot.data[0].sin() * rot.data[1].sin() * rot.data[2].sin())
                    + self.data[2]
                        * (rot.data[0].cos() * rot.data[1].sin() * rot.data[2].sin()
                            - rot.data[2].cos() * rot.data[0].sin()),
                -self.data[0] * rot.data[1].sin()
                    + self.data[1] * rot.data[1].cos() * rot.data[0].sin()
                    + self.data[2] * rot.data[0].cos() * rot.data[1].cos(),
            ],
        }
    }

    pub fn xyz_in_range_inclusive(&self, min: f64, max: f64) -> bool {
        self.data[0] >= min
            && self.data[0] <= max
            && self.data[1] >= min
            && self.data[1] <= max
            && self.data[2] >= min
            && self.data[2] <= max
    }
}

impl Mul for Vector<f64> {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Vector {
            data: [
                self.data[0] * rhs.data[0],
                self.data[1] * rhs.data[1],
                self.data[2] * rhs.data[2],
            ],
        }
    }
}

impl Mul<f64> for Vector<f64> {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Vector {
            data: [self.data[0] * rhs, self.data[1] * rhs, self.data[2] * rhs],
        }
    }
}

impl Mul<Vector<f64>> for f64 {
    type Output = Vector<f64>;

    fn mul(self, rhs: Vector<f64>) -> Self::Output {
        Vector {
            data: [self * rhs.data[0], self * rhs.data[1], self * rhs.data[2]],
        }
    }
}

impl Mul for Unit<Vector<f64>> {
    type Output = Vector<f64>;

    fn mul(self, rhs: Self) -> Self::Output {
        self.0 * rhs.0
    }
}

impl Mul<Vector<f64>> for Unit<Vector<f64>> {
    type Output = Vector<f64>;

    fn mul(self, rhs: Vector<f64>) -> Self::Output {
        self.0 * rhs
    }
}

impl Mul<Unit<Vector<f64>>> for Vector<f64> {
    type Output = Self;

    fn mul(self, rhs: Unit<Vector<f64>>) -> Self::Output {
        self * rhs.0
    }
}

impl Mul<f64> for Unit<Vector<f64>> {
    type Output = Vector<f64>;

    fn mul(self, rhs: f64) -> Self::Output {
        self.0 * rhs
    }
}

impl Mul<Unit<Vector<f64>>> for f64 {
    type Output = Vector<f64>;

    fn mul(self, rhs: Unit<Vector<f64>>) -> Self::Output {
        self * rhs.0
    }
}

impl MulAssign for Vector<f64> {
    fn mul_assign(&mut self, rhs: Self) {
        self.data[0] *= rhs.data[0];
        self.data[1] *= rhs.data[1];
        self.data[2] *= rhs.data[2];
    }
}

impl MulAssign<f64> for Vector<f64> {
    fn mul_assign(&mut self, rhs: f64) {
        self.data[0] *= rhs;
        self.data[1] *= rhs;
        self.data[2] *= rhs;
    }
}

impl Div<f64> for Vector<f64> {
    type Output = Self;

    fn div(self, rhs: f64) -> Self::Output {
        Vector {
            data: [self.data[0] / rhs, self.data[1] / rhs, self.data[2] / rhs],
        }
    }
}

impl Div<f64> for Unit<Vector<f64>> {
    type Output = Vector<f64>;

    fn div(self, rhs: f64) -> Self::Output {
        self.0 / rhs
    }
}

impl DivAssign<f64> for Vector<f64> {
    fn div_assign(&mut self, rhs: f64) {
        self.data[0] /= rhs;
        self.data[1] /= rhs;
        self.data[2] /= rhs;
    }
}

impl Add for Vector<f64> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Vector {
            data: [
                self.data[0] + rhs.data[0],
                self.data[1] + rhs.data[1],
                self.data[2] + rhs.data[2],
            ],
        }
    }
}

impl Add for Unit<Vector<f64>> {
    type Output = Vector<f64>;

    fn add(self, rhs: Self) -> Self::Output {
        self.0 + rhs.0
    }
}

impl Add<Vector<f64>> for Unit<Vector<f64>> {
    type Output = Vector<f64>;

    fn add(self, rhs: Vector<f64>) -> Self::Output {
        self.0 + rhs
    }
}

impl Add<Unit<Vector<f64>>> for Vector<f64> {
    type Output = Self;

    fn add(self, rhs: Unit<Vector<f64>>) -> Self::Output {
        self + rhs.0
    }
}

impl AddAssign for Vector<f64> {
    fn add_assign(&mut self, rhs: Self) {
        self.data[0] += rhs.data[0];
        self.data[1] += rhs.data[1];
        self.data[2] += rhs.data[2];
    }
}

impl Sub for Vector<f64> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Vector {
            data: [
                self.data[0] - rhs.data[0],
                self.data[1] - rhs.data[1],
                self.data[2] - rhs.data[2],
            ],
        }
    }
}

impl Sub for Unit<Vector<f64>> {
    type Output = Vector<f64>;

    fn sub(self, rhs: Self) -> Self::Output {
        self.0 - rhs.0
    }
}

impl Sub<Vector<f64>> for Unit<Vector<f64>> {
    type Output = Vector<f64>;

    fn sub(self, rhs: Vector<f64>) -> Self::Output {
        self.0 - rhs
    }
}

impl Sub<Unit<Vector<f64>>> for Vector<f64> {
    type Output = Self;

    fn sub(self, rhs: Unit<Vector<f64>>) -> Self::Output {
        self - rhs.0
    }
}

impl SubAssign for Vector<f64> {
    fn sub_assign(&mut self, rhs: Self) {
        self.data[0] -= rhs.data[0];
        self.data[1] -= rhs.data[1];
        self.data[2] -= rhs.data[2];
    }
}

/// Testing the vector math functions. Since we're testing floating-point
/// arithmetic with the `==` operator we make sure to choose values that
/// are small integers. This way we're guaranteed to get the correct
/// results.
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_add() {
        let v1 = Vec3::new(1., 2., 3.);
        let v2 = Vec3::new(2., 4., 6.);
        let v3 = v1 + v2;
        assert_eq!(Vec3::new(3., 6., 9.), v3);
    }

    #[test]
    fn test_sub() {
        let v1 = Vec3::new(4., 3., 2.);
        let v2 = Vec3::new(1., 1., 1.);
        let v3 = v1 - v2;
        assert_eq!(Vec3::new(3., 2., 1.), v3);
    }

    #[test]
    fn test_mul() {
        let v1 = Vec3::new(1., 4., 8.);
        let v2 = Vec3::new(2., 2., 2.);
        let v3 = v1 * v2;
        assert_eq!(Vec3::new(2., 8., 16.), v3);
    }

    #[test]
    fn test_scalar_mul() {
        let v1 = Vec3::new(1., 2., 3.);
        let v2 = 3. * v1;
        assert_eq!(Vec3::new(3., 6., 9.), v2);
    }

    #[test]
    fn test_mul_scalar() {
        let v1 = Vec3::new(1., 2., 3.);
        let v2 = v1 * 3.;
        assert_eq!(Vec3::new(3., 6., 9.), v2);
    }

    #[test]
    fn test_dot() {
        let v1 = Vec3::new(1., 2., 3.);
        assert_eq!(14., v1.dot(v1));
    }

    #[test]
    fn test_cross1() {
        let v1 = Vec3::new(1., 0., 0.);
        let v2 = Vec3::new(0., 1., 0.);
        let v3 = v1.cross(v2);
        assert_eq!(Vec3::new(0., 0., 1.), v3);
    }

    #[test]
    fn test_cross2() {
        let v1 = Vec3::new(1., 0., 0.);
        let v2 = Vec3::new(0., 0., 1.);
        let v3 = v1.cross(v2);
        assert_eq!(Vec3::new(0., -1., 0.), v3);
    }

    #[test]
    fn test_mag2() {
        let m = Vec3::new(1., 2., 3.).mag2();
        assert_eq!(14., m);
    }

    #[test]
    fn test_pow() {
        let v = Vec3::new(1., 2., 3.).powf(2.);
        assert_eq!(Vec3::new(1., 4., 9.), v);
    }

    #[test]
    fn test_clip() {
        let v = Vec3::new(-1., 2., 0.5).clip(0., 1.);
        assert_eq!(Vec3::new(0., 1., 0.5), v);
    }
}
