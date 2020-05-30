//! A simple vector math library
//!
//! Contains the most essential vector operations needed by rayrs.
use std::f64;
use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub, SubAssign};

/// A 3-dimensional vector.
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Vector<T> {
    data: [T; 3],
}

/// Convenience type alias for the most commonly used vector.
///
/// This type alias is here for compatiblity since the vector type was
/// previously called `Vec3`.
pub type Vec3 = Vector<f64>;

/// A wrapper type for unit vectors.
#[derive(PartialEq, Debug, Clone, Copy)]
pub struct Unit<T>(T);

/// Vector dot product.
pub trait Dot<Rhs = Self> {
    type Output;

    fn dot(self, other: Rhs) -> Self::Output;
}

/// Vector cross product.
pub trait Cross<Rhs = Self> {
    type Output;

    fn cross(self, other: Rhs) -> Self::Output;
}

/// Trait for accessing the individual elements of the vector.
pub trait VecElements<T> {
    fn x(self) -> T;
    fn y(self) -> T;
    fn z(self) -> T;
}

/// Unit vector related operations.
pub trait VecUnit {
    type Output;
    type VecOutput;

    /// Get the magnitude of the vector.
    fn mag(self) -> Self::Output;
    /// Get the magnitude squared of the vector.
    fn mag2(self) -> Self::Output;
    /// Get the unit vector proportional to this vector.
    fn unit(self) -> Unit<Self::VecOutput>;
}

impl Unit<Vector<f64>> {
    /// Construct a new unit vector.
    ///
    /// # Panics
    /// If the passed vector is not a unit vector.
    ///
    /// # Examples
    /// A vector created through some other means can be made into a unit vector
    /// if the length is 1.
    /// ```
    /// use rayrs_lib::vecmath::{Unit, Vec3};
    ///
    /// let v = Vec3::new(1., 0., 0.);
    /// let unit_v = Unit::new(v);
    ///
    /// assert_eq!(Vec3::unit_x(), unit_v);
    /// ```
    ///
    /// Trying to create a unit vector from a vector that does not have a length
    /// of 1 will lead to a panic.
    /// ```should_panic
    /// use rayrs_lib::vecmath::{Unit, Vec3};
    ///
    /// let v = Vec3::new(2., 0., 0.);
    /// let unit_v = Unit::new(v); // Panic!
    /// ```
    pub fn new(vec: Vector<f64>) -> Unit<Vector<f64>> {
        assert!((vec.mag2() - 1.).abs() < 4. * f64::EPSILON);

        Unit(vec)
    }

    /// Construct a new unit vector.
    ///
    /// # Note
    /// The use must assure that the vector passed is in fact a unit vector.
    ///
    /// # Examples
    /// Creating unit vectors work the same as for the checked function.
    /// ```
    /// use rayrs_lib::vecmath::{Unit, Vec3};
    ///
    /// let v = Vec3::new(1., 0., 0.);
    /// let unit_v = Unit::new_unchecked(v);
    ///
    /// assert_eq!(Vec3::unit_x(), unit_v);
    /// ```
    ///
    /// However, vectors can be made into unit vectors when they are really not,
    /// which will cause erroneous results when using methods from the
    /// [`VecUnit`] trait.
    /// ```should_panic
    /// use rayrs_lib::vecmath::{Unit, Vec3, VecUnit};
    ///
    /// let v = Vec3::new(2., 0., 0.);
    /// let unit_v = Unit::new_unchecked(v);
    ///
    /// assert_eq!(v.mag2(), unit_v.mag2()); // Panic! 4.0 != 1.0
    /// ```
    ///
    /// [`VecUnit`]: ../vecmath/trait.VecUnit.html
    pub fn new_unchecked(vec: Vector<f64>) -> Unit<Vector<f64>> {
        debug_assert!((vec.mag2() - 1.).abs() < 4. * f64::EPSILON);

        Unit(vec)
    }

    /// Get the underlying Vector<f64>.
    ///
    /// # Examples
    ///
    /// ```
    /// use rayrs_lib::vecmath::{Unit, Vec3};
    ///
    /// let v = Vec3::new(1., 0., 0.);
    /// let unit_v = Unit::new(v);
    ///
    /// assert_eq!(v, unit_v.as_vec());
    /// ```
    pub fn as_vec(self) -> Vector<f64> {
        self.0
    }

    /// Calculate `powf` for all components of the vector.
    ///
    /// For examples see [`Vector::powf`].
    ///
    /// [`Vec3::powf`]: ../vecmath/struct.Vector.html#method.powf
    pub fn powf(self, n: f64) -> Vector<f64> {
        self.0.powf(n)
    }

    /// Clip the components of the vector.
    ///
    /// For examples see [`Vector::clip`].
    ///
    /// [`Vector::clip`]: ../vecmath/struct.Vector.html#method.clip
    pub fn clip(self, min: f64, max: f64) -> Vector<f64> {
        self.0.clip(min, max)
    }

    /// Rotate the vector around the xyz axis.
    ///
    /// The number of radians for each axis is determined by the components
    /// of the `rot` vector.
    ///
    /// For examples see [`Vector::rotate`].
    ///
    /// [`Vector::rotate`]: ../vecmath/struct.Vector.html#method.rotate
    pub fn rotate(self, rot: Vector<f64>) -> Unit<Vector<f64>> {
        Unit(self.0.rotate(rot))
    }

    /// Check if the components of the vector are in a range, inclusive bounds.
    ///
    /// For examples see [`Vector::xyz_in_range_inclusive`].
    ///
    /// [`Vector::xyz_in_range_inclusive`]:
    /// ../vecmath/struct.Vector.html#method.xyz_in_range_inclusive
    pub fn xyz_in_range_inclusive(&self, min: f64, max: f64) -> bool {
        self.0.xyz_in_range_inclusive(min, max)
    }
}

impl<T: Copy> Vector<T> {
    /// Construct a new vector.
    ///
    /// # Examples
    /// A vector can be constructed with any type that implements copy.
    /// ```
    /// use rayrs_lib::vecmath::{VecElements, Vector};
    ///
    /// let v = Vector::new(1, 2, 3);
    ///
    /// assert_eq!(v.x(), 1);
    /// assert_eq!(v.y(), 2);
    /// assert_eq!(v.z(), 3);
    /// ```
    ///
    /// Most methods are only implemented for `f64` and there is a convenience
    /// type alias for `Vector<f64>`.
    /// ```
    /// use rayrs_lib::vecmath::{Vec3, VecUnit};
    ///
    /// let v = Vec3::new(1., 2., 3.);
    ///
    /// assert_eq!(v.mag2(), 14.);
    /// ```
    pub fn new(x: T, y: T, z: T) -> Vector<T> {
        Vector { data: [x, y, z] }
    }

    /// Generate an iterator over the components of the vector.
    pub fn iter(&self) -> std::slice::Iter<T> {
        self.data.iter()
    }

    /// Generate an iterator over the components of the vector.
    ///
    /// This method gives back an iterator that produces mutable references
    /// to the underlying data.
    pub fn iter_mut(&mut self) -> std::slice::IterMut<T> {
        self.data.iter_mut()
    }
}

impl Vector<f64> {
    /// Construct a new vector filled with ones.
    ///
    /// # Examples
    /// ```
    /// use rayrs_lib::vecmath::Vec3;
    ///
    /// let v = Vec3::ones();
    ///
    /// assert!(v.is_ones());
    /// ```
    pub fn ones() -> Vector<f64> {
        Vector { data: [1., 1., 1.] }
    }

    /// Construct a new vector filled with zeros.
    ///
    /// # Examples
    /// ```
    /// use rayrs_lib::vecmath::Vec3;
    ///
    /// let v = Vec3::zeros();
    ///
    /// assert!(v.is_zeros());
    /// ```
    pub fn zeros() -> Vector<f64> {
        Vector { data: [0., 0., 0.] }
    }

    /// Construct a new unit vector aligned with the x-axis.
    ///
    /// # Examples
    /// ```
    /// use rayrs_lib::vecmath::{Vec3, VecElements};
    ///
    /// let v = Vec3::unit_x();
    ///
    /// assert!(v.x() == 1. && v.y() == 0. && v.z() == 0.);
    /// ```
    pub fn unit_x() -> Unit<Vector<f64>> {
        Unit(Vector { data: [1., 0., 0.] })
    }

    /// Construct a new unit vector aligned with the y-axis.
    ///
    /// # Examples
    /// ```
    /// use rayrs_lib::vecmath::{Vec3, VecElements};
    ///
    /// let v = Vec3::unit_y();
    ///
    /// assert!(v.x() == 0. && v.y() == 1. && v.z() == 0.);
    /// ```
    pub fn unit_y() -> Unit<Vector<f64>> {
        Unit(Vector { data: [0., 1., 0.] })
    }

    /// Construct a new unit vector aligned with the z-axis.
    ///
    /// # Examples
    /// ```
    /// use rayrs_lib::vecmath::{Vec3, VecElements};
    ///
    /// let v = Vec3::unit_z();
    ///
    /// assert!(v.x() == 0. && v.y() == 0. && v.z() == 1.);
    /// ```
    pub fn unit_z() -> Unit<Vector<f64>> {
        Unit(Vector { data: [0., 0., 1.] })
    }

    /// Check if the vector is all zeros.
    ///
    /// # Examples
    /// ```
    /// use rayrs_lib::vecmath::Vec3;
    ///
    /// let v = Vec3::zeros();
    ///
    /// assert!(v.is_zeros());
    /// ```
    pub fn is_zeros(&self) -> bool {
        self.data == [0., 0., 0.]
    }

    /// Check if the vector is all ones.
    ///
    /// # Examples
    /// ```
    /// use rayrs_lib::vecmath::Vec3;
    ///
    /// let v = Vec3::ones();
    ///
    /// assert!(v.is_ones());
    /// ```
    pub fn is_ones(&self) -> bool {
        self.data == [1., 1., 1.]
    }

    /// Construct an orthonormal basis from a single vector.
    ///
    /// The basis will be right-handed when taking the passed in vector as the
    /// z-axis and returned vectors as the x- and y-axis respectively.
    ///
    /// For a given normal axis this function should always return the same
    /// basis, but it is not guaranteed the vectors that are close will produce
    /// basis that are close.
    ///
    /// # Examples
    ///
    /// ```
    /// use rayrs_lib::vecmath::{Cross, Vec3};
    ///
    /// let z = Vec3::unit_z();
    /// let (x, y) = Vec3::orthonormal_basis(z);
    ///
    /// assert_eq!(x.cross(y), z);
    /// ```
    pub fn orthonormal_basis(normal: Unit<Vector<f64>>) -> (Unit<Vector<f64>>, Unit<Vector<f64>>) {
        // Try to choose a vector that is not close to the normal.
        let temp = if normal.x().abs() > 0.9 {
            Vector::unit_y()
        } else {
            Vector::unit_x()
        };

        // Construct the basis to be right-handed.
        let e1 = temp.cross(normal).unit();
        let e2 = normal.cross(e1).unit();
        (e1, e2)
    }

    /// Calculate `powf` for each component of the vector.
    ///
    /// # Examples
    /// ```
    /// use rayrs_lib::vecmath::Vec3;
    ///
    /// let rgb_linear = Vec3::ones() * 0.5;
    /// let rgb_gamma_corr = rgb_linear.powf(1. / 2.2);
    ///
    /// assert_eq!(
    ///     Vec3::new(0.7297400528407231, 0.7297400528407231, 0.7297400528407231),
    ///     rgb_gamma_corr
    /// );
    /// ```
    pub fn powf(self, n: f64) -> Vector<f64> {
        Vector {
            data: [
                self.data[0].powf(n),
                self.data[1].powf(n),
                self.data[2].powf(n),
            ],
        }
    }

    /// Clip the components of the vector.
    ///
    /// # Examples
    /// ```
    /// use rayrs_lib::vecmath::Vec3;
    ///
    /// let v = Vec3::new(0., 1., 2.);
    /// let v = v.clip(1., 1.);
    ///
    /// assert_eq!(Vec3::ones(), v);
    /// ```
    pub fn clip(self, min: f64, max: f64) -> Vector<f64> {
        Vector {
            data: [
                self.data[0].min(max).max(min),
                self.data[1].min(max).max(min),
                self.data[2].min(max).max(min),
            ],
        }
    }

    /// Rotate the vector around the x, y and z-axis.
    ///
    /// The components of `rot` determines the number of radians to rotate in
    /// each of the coordinate axis directions.
    ///
    /// # Examples
    /// ```
    /// use rayrs_lib::vecmath::{Unit, Vec3, VecUnit};
    /// use std::f64;
    ///
    /// let v = Vec3::new(0., 0., 1.);
    /// let rot = Vec3::new(0., f64::consts::FRAC_PI_2, 0.);
    ///
    /// assert!((Vec3::unit_x() - v.rotate(rot)).mag2() < f64::EPSILON);
    /// ```
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

    /// Check if all the components of the vector are in an inclusive range.
    ///
    /// # Examples
    /// ```
    /// use rayrs_lib::vecmath::Vec3;
    ///
    /// let rgb = Vec3::new(1., 0.5, 0.);
    ///
    /// assert!(rgb.xyz_in_range_inclusive(0., 1.));
    /// ```
    ///
    /// ```
    /// use rayrs_lib::vecmath::Vec3;
    ///
    /// let rgb = Vec3::new(2., 0.5, 0.);
    ///
    /// assert!(!rgb.xyz_in_range_inclusive(0., 1.));
    /// ```
    pub fn xyz_in_range_inclusive(&self, min: f64, max: f64) -> bool {
        self.data[0] >= min
            && self.data[0] <= max
            && self.data[1] >= min
            && self.data[1] <= max
            && self.data[2] >= min
            && self.data[2] <= max
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
