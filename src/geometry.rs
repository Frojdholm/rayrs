use super::vecmath::Vec3;
use super::Ray;

use std::f64::consts::PI;
use std::ops::{Mul, Range};

pub trait Hittable {
    fn intersect(&self, r: &Ray) -> Option<f64>;
    fn normal(&self, p: &Vec3) -> Vec3;
    fn area(&self) -> f64;
    fn sample(&self) -> Vec3;
}

#[derive(Debug, Clone)]
pub enum Geometry {
    Sphere(Sphere),
    Plane(Plane),
    Flipped(Box<Geometry>),
    Rotated(Vec3, Box<Geometry>),
    Translated(Vec3, Box<Geometry>),
}

impl Hittable for Geometry {
    fn intersect(&self, r: &Ray) -> Option<f64> {
        match self {
            Geometry::Sphere(s) => s.intersect(r),
            Geometry::Plane(p) => p.intersect(r),
            Geometry::Flipped(f) => f.intersect(r),
            Geometry::Rotated(rot, geom) => geom.intersect(&r.rotate(&rot)),
            Geometry::Translated(v, geom) => geom.intersect(&r.translate(&v.mul(-1.))),
        }
    }

    fn normal(&self, p: &Vec3) -> Vec3 {
        match self {
            Geometry::Sphere(s) => s.normal(p),
            Geometry::Plane(plane) => plane.normal(p),
            Geometry::Flipped(f) => -1. * &f.normal(p),
            Geometry::Rotated(rot, geom) => geom.normal(&p.rotate(&rot)).rotate(&rot.mul(-1.)),
            Geometry::Translated(v, geom) => geom.normal(&(p - &v)),
        }
    }

    fn area(&self) -> f64 {
        match self {
            Geometry::Sphere(s) => s.area(),
            Geometry::Plane(plane) => plane.area(),
            Geometry::Flipped(f) => f.area(),
            Geometry::Rotated(_, geom) => geom.area(),
            Geometry::Translated(_, geom) => geom.area(),
        }
    }

    fn sample(&self) -> Vec3 {
        match self {
            Geometry::Sphere(s) => s.sample(),
            Geometry::Plane(plane) => plane.sample(),
            Geometry::Flipped(f) => f.sample(),
            Geometry::Rotated(rot, geom) => geom.sample().rotate(&rot.mul(-1.)),
            Geometry::Translated(v, geom) => &geom.sample() - &v,
        }
    }
}

#[derive(Debug, Clone)]
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
    fn intersect(&self, r: &Ray) -> Option<f64> {
        let v = &r.origin - &self.origin;

        let a = r.direction.mag2();
        let b = 2. * r.direction.dot(&v);
        let c = v.mag2() - self.radius2;

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

    fn normal(&self, p: &Vec3) -> Vec3 {
        (p - &self.origin).unit()
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

        &Vec3::new(x, y, z).mul(self.radius2.sqrt()) + &self.origin
    }
}

#[derive(Debug, Clone)]
pub enum Axis {
    X,
    XRev,
    Y,
    YRev,
    Z,
    ZRev,
}

#[derive(Debug, Clone)]
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
    fn intersect(&self, r: &Ray) -> Option<f64> {
        match &self.axis {
            Axis::X | Axis::XRev => {
                if r.direction.x != 0. {
                    let t = (self.pos - r.origin.x) / r.direction.x;
                    let p = r.point(t);
                    if self.u_range.contains(&p.y) && self.v_range.contains(&p.z) {
                        Some(t)
                    } else {
                        None
                    }
                } else {
                    None
                }
            }
            Axis::Y | Axis::YRev => {
                if r.direction.y != 0. {
                    let t = (self.pos - r.origin.y) / r.direction.y;
                    let p = r.point(t);
                    if self.u_range.contains(&p.x) && self.v_range.contains(&p.z) {
                        Some(t)
                    } else {
                        None
                    }
                } else {
                    None
                }
            }
            Axis::Z | Axis::ZRev => {
                if r.direction.z != 0. {
                    let t = (self.pos - r.origin.z) / r.direction.z;
                    let p = r.point(t);
                    if self.u_range.contains(&p.x) && self.v_range.contains(&p.y) {
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

    fn normal(&self, _p: &Vec3) -> Vec3 {
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
        let t = sphere.intersect(&ray).unwrap();
        assert!(t > 0.);
    }

    #[test]
    fn test_intersect_sphere_inside() {
        let sphere = Sphere::new(1., Vec3::new(0., 0., 0.));
        let ray = Ray::new(Vec3::new(0., 0., 0.), Vec3::new(0., 1., 0.));
        let t = sphere.intersect(&ray).unwrap();
        assert!(t > 0.);
    }

    #[test]
    fn test_intersect_sphere_miss() {
        let sphere = Sphere::new(1., Vec3::new(0., 0., 0.));
        let ray = Ray::new(Vec3::new(0., 5., 0.), Vec3::new(0., 1., 0.));
        let t = sphere.intersect(&ray);
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
        let t = plane.intersect(&ray).unwrap();
        assert!(t > 0.);
    }

    #[test]
    fn test_intesect_x_plane_back() {
        let plane = Plane::new(Axis::X, -1., 1., -1., 1., 0.);
        let ray = Ray::new(Vec3::new(-5., 0., 0.), Vec3::new(1., 0., 0.));
        let t = plane.intersect(&ray).unwrap();
        assert!(t > 0.);
    }

    #[test]
    fn test_intesect_y_plane_front() {
        let plane = Plane::new(Axis::Y, -1., 1., -1., 1., 0.);
        let ray = Ray::new(Vec3::new(0., 5., 0.), Vec3::new(0., -1., 0.));
        let t = plane.intersect(&ray).unwrap();
        assert!(t > 0.);
    }

    #[test]
    fn test_intesect_y_plane_back() {
        let plane = Plane::new(Axis::Y, -1., 1., -1., 1., 0.);
        let ray = Ray::new(Vec3::new(0., -5., 0.), Vec3::new(0., 1., 0.));
        let t = plane.intersect(&ray).unwrap();
        assert!(t > 0.);
    }

    #[test]
    fn test_intesect_z_plane_front() {
        let plane = Plane::new(Axis::Z, -1., 1., -1., 1., 0.);
        let ray = Ray::new(Vec3::new(0., 0., 5.), Vec3::new(0., 0., -1.));
        let t = plane.intersect(&ray).unwrap();
        assert!(t > 0.);
    }

    #[test]
    fn test_intesect_z_plane_back() {
        let plane = Plane::new(Axis::Z, -1., 1., -1., 1., 0.);
        let ray = Ray::new(Vec3::new(0., 0., -5.), Vec3::new(0., 0., 1.));
        let t = plane.intersect(&ray).unwrap();
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
            plane.u_range.contains(&sample.x)
                && plane.v_range.contains(&sample.y)
                && sample.z == 0.
        );
    }
}
