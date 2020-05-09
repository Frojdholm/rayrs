use super::vecmath::Vec3;
use super::Object;
use super::Ray;

use std::f64::consts::PI;
use std::ops::Range;

pub trait Hittable {
    fn intersect(&self, ray: Ray) -> Option<f64>;
    fn normal(&self, p: Vec3) -> Vec3;
    fn area(&self) -> f64;
    fn sample(&self) -> Vec3;
    fn bbox(&self) -> AxisAlignedBoundingBox;
}

#[derive(Debug)]
pub struct Flipped<T: Hittable>(T);

impl<T: Hittable> Flipped<T> {
    pub fn new(geom: T) -> Flipped<T> {
        Flipped(geom)
    }
}

impl<T: Hittable> Hittable for Flipped<T> {
    fn intersect(&self, ray: Ray) -> Option<f64> {
        self.0.intersect(ray)
    }

    fn normal(&self, p: Vec3) -> Vec3 {
        -1. * self.0.normal(p)
    }

    fn area(&self) -> f64 {
        self.0.area()
    }

    fn sample(&self) -> Vec3 {
        self.0.sample()
    }

    fn bbox(&self) -> AxisAlignedBoundingBox {
        self.0.bbox()
    }
}

#[derive(Debug)]
pub struct Sphere {
    radius2: f64,
    origin: Vec3,
}

impl Sphere {
    pub fn new(radius: f64, origin: Vec3) -> Sphere {
        assert!(radius > 0., "Radius has to be positive, r={}", radius);
        Sphere {
            radius2: radius * radius,
            origin,
        }
    }
}

impl Hittable for Sphere {
    fn intersect(&self, ray: Ray) -> Option<f64> {
        let odiff = ray.origin - self.origin;

        let a = ray.direction.mag_2();
        let b = 2. * ray.direction.dot(odiff);
        let c = odiff.mag_2() - self.radius2;

        let desc = b * b - 4. * a * c;

        if desc > 0. {
            let t1 = (-b - desc.sqrt()) / (2. * a);
            let t2 = (-b + desc.sqrt()) / (2. * a);
            if t1 < 0. {
                if t2 < 0. {
                    None
                } else {
                    Some(t2)
                }
            } else {
                Some(t1)
            }
        } else {
            None
        }
    }

    fn normal(&self, p: Vec3) -> Vec3 {
        (p - self.origin).unit()
    }

    fn area(&self) -> f64 {
        4. * PI * self.radius2
    }

    fn sample(&self) -> Vec3 {
        let u: f64 = rand::random();
        let phi: f64 = 2. * PI * rand::random::<f64>();

        let x = phi.cos() * 2. * (u * (1. - u)).sqrt();
        let y = phi.sin() * 2. * (u * (1. - u)).sqrt();
        let z = 1. - 2. * u;

        Vec3::new(x, y, z) * self.radius2.sqrt() + self.origin
    }

    fn bbox(&self) -> AxisAlignedBoundingBox {
        AxisAlignedBoundingBox::from(self)
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Axis {
    X,
    XRev,
    Y,
    YRev,
    Z,
    ZRev,
}

#[derive(Debug)]
pub struct Plane {
    u_range: Range<f64>,
    v_range: Range<f64>,
    pos: f64,
    axis: Axis,
}

impl Plane {
    pub fn new(axis: Axis, umin: f64, umax: f64, vmin: f64, vmax: f64, pos: f64) -> Plane {
        assert!(
            umin < umax && vmin < vmax,
            "Plane cannot be constructed with umin={}, umax={}, vmin={}, vmax={}",
            umin,
            umax,
            vmin,
            vmax
        );
        Plane {
            axis,
            u_range: Range {
                start: umin,
                end: umax,
            },
            v_range: Range {
                start: vmin,
                end: vmax,
            },
            pos,
        }
    }
}

impl Hittable for Plane {
    fn intersect(&self, ray: Ray) -> Option<f64> {
        match &self.axis {
            Axis::X | Axis::XRev => {
                if ray.direction.x() != 0. {
                    let t = (self.pos - ray.origin.x()) / ray.direction.x();
                    let p = ray.point(t);
                    if self.u_range.contains(&p.y()) && self.v_range.contains(&p.z()) {
                        Some(t)
                    } else {
                        None
                    }
                } else {
                    None
                }
            }
            Axis::Y | Axis::YRev => {
                if ray.direction.y() != 0. {
                    let t = (self.pos - ray.origin.y()) / ray.direction.y();
                    let p = ray.point(t);
                    if self.u_range.contains(&p.x()) && self.v_range.contains(&p.z()) {
                        Some(t)
                    } else {
                        None
                    }
                } else {
                    None
                }
            }
            Axis::Z | Axis::ZRev => {
                if ray.direction.z() != 0. {
                    let t = (self.pos - ray.origin.z()) / ray.direction.z();
                    let p = ray.point(t);
                    if self.u_range.contains(&p.x()) && self.v_range.contains(&p.y()) {
                        Some(t)
                    } else {
                        None
                    }
                } else {
                    None
                }
            }
        }
    }

    fn normal(&self, _p: Vec3) -> Vec3 {
        match &self.axis {
            Axis::X => Vec3::new(1., 0., 0.),
            Axis::XRev => Vec3::new(-1., 0., 0.),
            Axis::Y => Vec3::new(0., 1., 0.),
            Axis::YRev => Vec3::new(0., -1., 0.),
            Axis::Z => Vec3::new(0., 0., 1.),
            Axis::ZRev => Vec3::new(0., 0., -1.),
        }
    }

    fn area(&self) -> f64 {
        (self.u_range.end - self.u_range.start) * (self.v_range.end - self.v_range.start)
    }

    fn sample(&self) -> Vec3 {
        let u =
            rand::random::<f64>() * (self.u_range.end - self.u_range.start) + self.u_range.start;
        let v =
            rand::random::<f64>() * (self.v_range.end - self.v_range.start) + self.v_range.start;

        match &self.axis {
            Axis::X | Axis::XRev => Vec3::new(self.pos, u, v),
            Axis::Y | Axis::YRev => Vec3::new(u, self.pos, v),
            Axis::Z | Axis::ZRev => Vec3::new(u, v, self.pos),
        }
    }

    fn bbox(&self) -> AxisAlignedBoundingBox {
        AxisAlignedBoundingBox::from(self)
    }
}

#[derive(Debug)]
pub struct Triangle {
    p1: Vec3,
    p2: Vec3,
    p3: Vec3,
    e1: Vec3,
    e2: Vec3,
    normal: Vec3,
    area: f64,
}

impl Triangle {
    pub fn new(p1: Vec3, p2: Vec3, p3: Vec3) -> Triangle {
        let e1 = p2 - p1;
        let e2 = p3 - p1;
        let normal = e1.cross(e2);
        Triangle {
            p1,
            p2,
            p3,
            e1,
            e2,
            normal: normal.unit(),
            area: normal.mag() / 2.,
        }
    }
}

impl Hittable for Triangle {
    /// Möller-Trumbore algorithm for triangle intersection.
    fn intersect(&self, ray: Ray) -> Option<f64> {
        let t = ray.origin - self.p1;
        let p = ray.direction.cross(self.e2);
        let q = t.cross(self.e1);

        let den = p.dot(self.e1);

        let d = q.dot(self.e2) / den;
        let u = p.dot(t) / den;
        let v = q.dot(ray.direction) / den;

        if d < 0. || u < 0. || v < 0. || u + v > 1. {
            None
        } else {
            Some(d)
        }
    }

    fn normal(&self, _p: Vec3) -> Vec3 {
        self.normal
    }

    fn area(&self) -> f64 {
        self.area
    }

    fn sample(&self) -> Vec3 {
        Vec3::new(0.0, 0.0, 0.0)
    }

    fn bbox(&self) -> AxisAlignedBoundingBox {
        AxisAlignedBoundingBox::from(self)
    }
}

#[derive(Debug)]
pub struct AxisAlignedBoundingBox {
    x_range: Range<f64>,
    y_range: Range<f64>,
    z_range: Range<f64>,
}

impl AxisAlignedBoundingBox {
    pub fn new(xmin: f64, xmax: f64, ymin: f64, ymax: f64, zmin: f64, zmax: f64) -> Self {
        assert!(xmin <= xmax);
        assert!(ymin <= ymax);
        assert!(zmin <= zmax);

        AxisAlignedBoundingBox {
            x_range: xmin..xmax,
            y_range: ymin..ymax,
            z_range: zmin..zmax,
        }
    }

    pub fn xmin(&self) -> f64 {
        self.x_range.start
    }

    pub fn xmax(&self) -> f64 {
        self.x_range.end
    }

    pub fn ymin(&self) -> f64 {
        self.y_range.start
    }

    pub fn ymax(&self) -> f64 {
        self.y_range.end
    }

    pub fn zmin(&self) -> f64 {
        self.z_range.start
    }

    pub fn zmax(&self) -> f64 {
        self.z_range.end
    }

    pub fn intersect(&self, ray: Ray, tmin: f64, tmax: f64) -> bool {
        // TODO: Make this faster and take care of NaNs

        let xmax = self.x_range.end - ray.origin.x();
        let xmin = self.x_range.start - ray.origin.x();
        let inv_x = 1. / ray.direction.x();

        let (txmin, txmax) = if inv_x < 0. {
            (xmax * inv_x, xmin * inv_x)
        } else {
            (xmin * inv_x, xmax * inv_x)
        };

        let tmin = tmin.max(txmin);
        let tmax = tmax.min(txmax);

        if tmax <= tmin {
            return false;
        }

        let ymax = self.y_range.end - ray.origin.y();
        let ymin = self.y_range.start - ray.origin.y();
        let inv_y = 1. / ray.direction.y();

        let (tymin, tymax) = if inv_y < 0. {
            (ymax * inv_y, ymin * inv_y)
        } else {
            (ymin * inv_y, ymax * inv_y)
        };

        let tmin = tmin.max(tymin);
        let tmax = tmax.min(tymax);

        if tmax <= tmin {
            return false;
        }

        let zmax = self.z_range.end - ray.origin.z();
        let zmin = self.z_range.start - ray.origin.z();
        let inv_z = 1. / ray.direction.z();

        let (tzmin, tzmax) = if inv_z < 0. {
            (zmax * inv_z, zmin * inv_z)
        } else {
            (zmin * inv_z, zmax * inv_z)
        };

        let tmin = tmin.max(tzmin);
        let tmax = tmax.min(tzmax);

        if tmax <= tmin {
            return false;
        }

        true
    }

    pub fn from_object_list(objects: &[Object]) -> Self {
        assert!(!objects.is_empty());
        objects
            .iter()
            .skip(1)
            .fold(objects[0].bbox(), |bbox, el| bbox.expand(el.bbox()))
    }

    pub fn center(&self) -> Vec3 {
        let x = (self.x_range.end - self.x_range.start) / 2. + self.x_range.start;
        let y = (self.y_range.end - self.y_range.start) / 2. + self.y_range.start;
        let z = (self.z_range.end - self.z_range.start) / 2. + self.z_range.start;
        Vec3::new(x, y, z)
    }

    pub fn volume(&self) -> f64 {
        (self.x_range.end - self.x_range.start)
            * (self.y_range.end - self.y_range.start)
            * (self.z_range.end - self.z_range.start)
    }

    pub fn surface_area(&self) -> f64 {
        let x = self.x_range.end - self.x_range.start;
        let y = self.y_range.end - self.y_range.start;
        let z = self.z_range.end - self.z_range.start;
        2. * x * y + 2. * y * z + 2. * x * z
    }

    pub fn expand(self, other: Self) -> Self {
        let xmin = self.x_range.start.min(other.x_range.start);
        let xmax = self.x_range.end.max(other.x_range.end);
        let ymin = self.y_range.start.min(other.y_range.start);
        let ymax = self.y_range.end.max(other.y_range.end);
        let zmin = self.z_range.start.min(other.z_range.start);
        let zmax = self.z_range.end.max(other.z_range.end);

        Self::new(xmin, xmax, ymin, ymax, zmin, zmax)
    }
}

impl From<&Sphere> for AxisAlignedBoundingBox {
    fn from(sphere: &Sphere) -> Self {
        let radius = sphere.radius2.sqrt();
        let xmin = sphere.origin.x() - radius;
        let xmax = sphere.origin.x() + radius;
        let ymin = sphere.origin.y() - radius;
        let ymax = sphere.origin.y() + radius;
        let zmin = sphere.origin.z() - radius;
        let zmax = sphere.origin.z() + radius;
        AxisAlignedBoundingBox::new(xmin, xmax, ymin, ymax, zmin, zmax)
    }
}

impl From<&Plane> for AxisAlignedBoundingBox {
    fn from(plane: &Plane) -> Self {
        match plane.axis {
            Axis::X | Axis::XRev => {
                let (ymin, ymax) = (plane.u_range.start, plane.u_range.end);
                let (zmin, zmax) = (plane.v_range.start, plane.v_range.end);
                AxisAlignedBoundingBox::new(plane.pos, plane.pos, ymin, ymax, zmin, zmax)
            }
            Axis::Y | Axis::YRev => {
                let (xmin, xmax) = (plane.u_range.start, plane.u_range.end);
                let (zmin, zmax) = (plane.v_range.start, plane.v_range.end);
                AxisAlignedBoundingBox::new(xmin, xmax, plane.pos, plane.pos, zmin, zmax)
            }
            Axis::Z | Axis::ZRev => {
                let (xmin, xmax) = (plane.u_range.start, plane.u_range.end);
                let (ymin, ymax) = (plane.v_range.start, plane.v_range.end);
                AxisAlignedBoundingBox::new(xmin, xmax, ymin, ymax, plane.pos, plane.pos)
            }
        }
    }
}

impl From<&Triangle> for AxisAlignedBoundingBox {
    fn from(t: &Triangle) -> Self {
        let xmin = t.p1.x().min(t.p2.x().min(t.p3.x()));
        let ymin = t.p1.y().min(t.p2.y().min(t.p3.y()));
        let zmin = t.p1.z().min(t.p2.z().min(t.p3.z()));

        let xmax = t.p1.x().max(t.p2.x().max(t.p3.x()));
        let ymax = t.p1.y().max(t.p2.y().max(t.p3.y()));
        let zmax = t.p1.z().max(t.p2.z().max(t.p3.z()));

        AxisAlignedBoundingBox::new(xmin, xmax, ymin, ymax, zmin, zmax)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[should_panic]
    fn test_sphere_new_negative_radius() {
        let _sphere = Sphere::new(-1., Vec3::new(0., 0., 0.));
    }
    #[test]
    fn test_intersect_sphere_outside() {
        let sphere = Sphere::new(1., Vec3::new(0., 0., 0.));
        let ray = Ray::new(Vec3::new(0., 0., 5.), Vec3::new(0., 0., -1.));
        let t = sphere.intersect(ray).unwrap();
        assert!(t > 0.);
    }

    #[test]
    fn test_intersect_sphere_inside() {
        let sphere = Sphere::new(1., Vec3::new(0., 0., 0.));
        let ray = Ray::new(Vec3::new(0., 0., 0.), Vec3::new(0., 1., 0.));
        let t = sphere.intersect(ray).unwrap();
        assert!(t > 0.);
    }

    #[test]
    fn test_intersect_sphere_miss() {
        let sphere = Sphere::new(1., Vec3::new(0., 0., 0.));
        let ray = Ray::new(Vec3::new(0., 5., 0.), Vec3::new(0., 1., 0.));
        let t = sphere.intersect(ray);
        assert!(t.is_none());
    }

    #[test]
    #[should_panic]
    fn test_plane_new_min_max_order() {
        let _plane = Plane::new(Axis::Z, 1., -1., 1., -1., 0.);
    }

    #[test]
    fn test_intesect_x_plane_front() {
        let plane = Plane::new(Axis::X, -1., 1., -1., 1., 0.);
        let ray = Ray::new(Vec3::new(5., 0., 0.), Vec3::new(-1., 0., 0.));
        let t = plane.intersect(ray).unwrap();
        assert!(t > 0.);
    }

    #[test]
    fn test_intesect_x_plane_back() {
        let plane = Plane::new(Axis::X, -1., 1., -1., 1., 0.);
        let ray = Ray::new(Vec3::new(-5., 0., 0.), Vec3::new(1., 0., 0.));
        let t = plane.intersect(ray).unwrap();
        assert!(t > 0.);
    }

    #[test]
    fn test_intesect_y_plane_front() {
        let plane = Plane::new(Axis::Y, -1., 1., -1., 1., 0.);
        let ray = Ray::new(Vec3::new(0., 5., 0.), Vec3::new(0., -1., 0.));
        let t = plane.intersect(ray).unwrap();
        assert!(t > 0.);
    }

    #[test]
    fn test_intesect_y_plane_back() {
        let plane = Plane::new(Axis::Y, -1., 1., -1., 1., 0.);
        let ray = Ray::new(Vec3::new(0., -5., 0.), Vec3::new(0., 1., 0.));
        let t = plane.intersect(ray).unwrap();
        assert!(t > 0.);
    }

    #[test]
    fn test_intesect_z_plane_front() {
        let plane = Plane::new(Axis::Z, -1., 1., -1., 1., 0.);
        let ray = Ray::new(Vec3::new(0., 0., 5.), Vec3::new(0., 0., -1.));
        let t = plane.intersect(ray).unwrap();
        assert!(t > 0.);
    }

    #[test]
    fn test_intesect_z_plane_back() {
        let plane = Plane::new(Axis::Z, -1., 1., -1., 1., 0.);
        let ray = Ray::new(Vec3::new(0., 0., -5.), Vec3::new(0., 0., 1.));
        let t = plane.intersect(ray).unwrap();
        assert!(t > 0.);
    }

    #[test]
    fn test_plane_area() {
        let plane = Plane::new(Axis::Z, -1., 1., -1., 1., 0.);
        assert_eq!(4., plane.area())
    }

    #[test]
    fn test_plane_sample() {
        let plane = Plane::new(Axis::Z, -1., 1., -1., 1., 0.);
        let sample = plane.sample();
        assert!(
            plane.u_range.contains(&sample.x())
                && plane.v_range.contains(&sample.y())
                && sample.z() == 0.
        );
    }

    #[test]
    fn test_aabb_intersection_outside_x() {
        let bbox = AxisAlignedBoundingBox::new(-1., 1., -1., 1., -1., 1.);
        let ray = Ray::new(Vec3::new(-5., 0., 0.), Vec3::new(1., 0., 0.));
        assert!(bbox.intersect(ray, 0.001, 1000.));
    }

    #[test]
    fn test_aabb_intersection_outside_y() {
        let bbox = AxisAlignedBoundingBox::new(-1., 1., -1., 1., -1., 1.);
        let ray = Ray::new(Vec3::new(0., -5., 0.), Vec3::new(0., 1., 0.));
        assert!(bbox.intersect(ray, 0.0001, 1000.));
    }

    #[test]
    fn test_aabb_intersection_outside_z() {
        let bbox = AxisAlignedBoundingBox::new(-1., 1., -1., 1., -1., 1.);
        let ray = Ray::new(Vec3::new(0., 0., -5.), Vec3::new(0., 0., 1.));
        assert!(bbox.intersect(ray, 0.001, 1000.));
    }

    #[test]
    fn test_aabb_intersection_inside() {
        let bbox = AxisAlignedBoundingBox::new(-1., 1., -1., 1., -1., 1.);
        let ray = Ray::new(Vec3::new(0., 0., 0.), Vec3::new(0., 0., 1.));
        assert!(bbox.intersect(ray, 0.001, 1000.));
    }

    #[test]
    fn test_aabb_intersection_miss() {
        let bbox = AxisAlignedBoundingBox::new(-1., 1., -1., 1., -1., 1.);
        let ray = Ray::new(Vec3::new(1.1, 0., 0.), Vec3::new(0., 1., 1.));
        assert!(!bbox.intersect(ray, 0.001, 1000.));
    }

    #[test]
    fn test_aabb_intersection_miss2() {
        let bbox = AxisAlignedBoundingBox::new(-1., 1., -1., 1., -1., 1.);
        let ray = Ray::new(Vec3::new(2., 0., 0.), Vec3::new(-1., -2., 0.));
        assert!(!bbox.intersect(ray, 0.001, 1000.));
    }
}
