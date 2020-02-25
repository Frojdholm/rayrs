use super::vecmath::Vec3;
use super::Object;
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

#[derive(Debug, Clone)]
pub struct AxisAlignedBoundingBox {
    x_range: Range<f64>,
    y_range: Range<f64>,
    z_range: Range<f64>,
}

impl AxisAlignedBoundingBox {
    fn new(xmin: f64, xmax: f64, ymin: f64, ymax: f64, zmin: f64, zmax: f64) -> Self {
        assert!(xmin <= xmax);
        assert!(ymin <= ymax);
        assert!(zmin <= zmax);

        AxisAlignedBoundingBox {
            x_range: xmin..xmax,
            y_range: ymin..ymax,
            z_range: zmin..zmax,
        }
    }

    fn intersect(&self, ray: &Ray, tmin: f64, tmax: f64) -> bool {
        // TODO: Make this faster and take care of NaNs

        let xmax = self.x_range.end - ray.origin.x;
        let xmin = self.x_range.start - ray.origin.x;
        let inv_x = 1. / ray.direction.x;

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

        let ymax = self.y_range.end - ray.origin.y;
        let ymin = self.y_range.start - ray.origin.y;
        let inv_y = 1. / ray.direction.y;

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

        let zmax = self.z_range.end - ray.origin.z;
        let zmin = self.z_range.start - ray.origin.z;
        let inv_z = 1. / ray.direction.z;

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

    fn from_object_list(objects: &Vec<Object>) -> Option<Self> {
        if let Some(bbox) = objects.get(0) {
            let mut bbox: AxisAlignedBoundingBox = bbox.into();
            for obj in objects.iter().skip(1) {
                bbox = bbox.expand(&obj.into());
            }
            Some(bbox)
        } else {
            None
        }
    }

    fn get_center(&self) -> Vec3 {
        let x = (self.x_range.end - self.x_range.start) / 2. + self.x_range.start;
        let y = (self.y_range.end - self.y_range.start) / 2. + self.y_range.start;
        let z = (self.z_range.end - self.z_range.start) / 2. + self.z_range.start;
        Vec3::new(x, y, z)
    }

    fn volume(&self) -> f64 {
        (self.x_range.end - self.x_range.start)
            * (self.y_range.end - self.y_range.start)
            * (self.z_range.end - self.z_range.start)
    }

    fn expand(&self, other: &Self) -> Self {
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
        let xmin = sphere.origin.x - radius;
        let xmax = sphere.origin.x + radius;
        let ymin = sphere.origin.y - radius;
        let ymax = sphere.origin.y + radius;
        let zmin = sphere.origin.z - radius;
        let zmax = sphere.origin.z + radius;
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

impl From<&Geometry> for AxisAlignedBoundingBox {
    fn from(geom: &Geometry) -> Self {
        match geom {
            Geometry::Sphere(s) => s.into(),
            Geometry::Plane(p) => p.into(),
            Geometry::Flipped(f) => (&**f).into(),
            // TODO: Instanced bounding boxes do not work
            Geometry::Rotated(_, _) => panic!("Not implemented"),
            Geometry::Translated(_, _) => panic!("Not implemented"),
        }
    }
}

impl From<&Object> for AxisAlignedBoundingBox {
    fn from(obj: &Object) -> Self {
        (&obj.geom).into()
    }
}

fn get_split_ind(axis: Axis, split: f64, data: &Vec<(Vec3, Object)>) -> usize {
    let mut ind = 0;
    match axis {
        Axis::X | Axis::XRev => {
            for (i, (v, _)) in data.iter().enumerate() {
                if v.x > split {
                    ind = i;
                    break;
                }
            }
        }
        Axis::Y | Axis::YRev => {
            for (i, (v, _)) in data.iter().enumerate() {
                if v.y > split {
                    ind = i;
                    break;
                }
            }
        }
        Axis::Z | Axis::ZRev => {
            for (i, (v, _)) in data.iter().enumerate() {
                if v.z > split {
                    ind = i;
                    break;
                }
            }
        }
    }
    ind
}

#[derive(Debug)]
pub struct Bvh {
    bvh: BvhTree,
}

impl Bvh {
    pub fn build(objects: Vec<Object>) -> Self {
        Bvh {
            bvh: BvhTree::build(objects),
        }
    }

    pub fn intersect(&self, ray: &Ray, tmin: f64, tmax: f64) -> Option<(f64, Object)> {
        self.bvh.intersect(ray, tmin, tmax)
    }

    pub fn volume(&self) -> f64 {
        self.bvh.volume()
    }
}

#[derive(Debug, Clone)]
enum BvhTree {
    Node(AxisAlignedBoundingBox, Vec<BvhTree>),
    LeafNode(Object),
}

impl BvhTree {
    fn build(objects: Vec<Object>) -> Self {
        // Having a BVH for 0 objects does not make sense
        assert!(objects.len() > 0);

        let bbox = AxisAlignedBoundingBox::from_object_list(&objects).unwrap();

        if objects.len() > 4 {
            // We only want 4 or less objects in the leaf nodes. So we split
            // this node along its longest axis and continue.

            // TODO: Make this more understandable..
            let x = bbox.x_range.end - bbox.x_range.start;
            let y = bbox.y_range.end - bbox.x_range.start;
            let z = bbox.z_range.end - bbox.z_range.start;

            let centers: Vec<Vec3> = objects
                .iter()
                .map(|geom| AxisAlignedBoundingBox::from(geom).get_center())
                .collect();

            let mean_center: Vec3 = centers
                .iter()
                .fold(Vec3::new(0., 0., 0.), |acc, el| &acc + &el)
                .mul(1. / centers.len() as f64);

            let mut data: Vec<(Vec3, Object)> = centers.into_iter().zip(objects).collect();

            // Find the index to split the sorted objects. If splitting by the
            // midpoint of the bounding box would lead to having all elements
            // in one side we split along object mean position instead.
            let ind = if x >= y && x >= z {
                data.sort_by(|a, b| a.0.x.partial_cmp(&b.0.x).unwrap());
                let ind = get_split_ind(Axis::X, bbox.get_center().x, &data);
                if ind != 0 {
                    ind
                } else {
                    get_split_ind(Axis::X, mean_center.x, &data)
                }
            } else if y >= z {
                data.sort_by(|a, b| a.0.y.partial_cmp(&b.0.y).unwrap());
                let ind = get_split_ind(Axis::Y, bbox.get_center().y, &data);
                if ind != 0 {
                    ind
                } else {
                    get_split_ind(Axis::Y, mean_center.y, &data)
                }
            } else {
                data.sort_by(|a, b| a.0.z.partial_cmp(&b.0.z).unwrap());
                let ind = get_split_ind(Axis::Z, bbox.get_center().z, &data);
                if ind != 0 {
                    ind
                } else {
                    get_split_ind(Axis::Z, mean_center.z, &data)
                }
            };

            // TODO: There must be a better way to sort geometry and centers together.
            let sorted_objects: Vec<Object> = data.into_iter().map(|item| item.1).collect();
            let (split1, split2) = sorted_objects.split_at(ind);
            BvhTree::Node(
                bbox,
                vec![
                    BvhTree::build(split1.to_vec()),
                    BvhTree::build(split2.to_vec()),
                ],
            )
        } else {
            // When we reach a small enough number of objects we can just add
            // them as leaf nodes.
            BvhTree::Node(
                bbox,
                objects
                    .into_iter()
                    .map(|obj| BvhTree::LeafNode(obj))
                    .collect(),
            )
        }
    }

    fn intersect(&self, ray: &Ray, tmin: f64, tmax: f64) -> Option<(f64, Object)> {
        match self {
            BvhTree::Node(bbox, children) => {
                if bbox.intersect(ray, tmin, tmax) {
                    let mut t = tmax;
                    let mut obj = None;
                    for child in children {
                        if let Some((t_hit, obj_hit)) = child.intersect(ray, tmin, tmax) {
                            if t_hit > tmin && t_hit < t {
                                t = t_hit;
                                obj = Some(obj_hit);
                            }
                        }
                    }

                    if obj.is_some() {
                        Some((t, obj.unwrap()))
                    } else {
                        None
                    }
                } else {
                    None
                }
            }
            BvhTree::LeafNode(obj) => match obj.geom.intersect(ray) {
                Some(t) => {
                    if t > tmin && t < tmax {
                        Some((t, (*obj).clone()))
                    } else {
                        None
                    }
                }
                None => None,
            },
        }
    }

    fn volume(&self) -> f64 {
        match self {
            BvhTree::Node(bbox, children) => {
                let volume = children.iter().fold(0., |acc, el| acc + el.volume());
                volume + bbox.volume()
            }
            BvhTree::LeafNode(obj) => {
                let bbox: AxisAlignedBoundingBox = obj.into();
                bbox.volume()
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::{Emission, Material};

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

    #[test]
    fn test_aabb_intersection_outside_x() {
        let bbox = AxisAlignedBoundingBox::new(-1., 1., -1., 1., -1., 1.);
        let ray = Ray::new(Vec3::new(-5., 0., 0.), Vec3::new(1., 0., 0.));
        assert!(bbox.intersect(&ray, 0.001, 1000.));
    }

    #[test]
    fn test_aabb_intersection_outside_y() {
        let bbox = AxisAlignedBoundingBox::new(-1., 1., -1., 1., -1., 1.);
        let ray = Ray::new(Vec3::new(0., -5., 0.), Vec3::new(0., 1., 0.));
        assert!(bbox.intersect(&ray, 0.0001, 1000.));
    }

    #[test]
    fn test_aabb_intersection_outside_z() {
        let bbox = AxisAlignedBoundingBox::new(-1., 1., -1., 1., -1., 1.);
        let ray = Ray::new(Vec3::new(0., 0., -5.), Vec3::new(0., 0., 1.));
        assert!(bbox.intersect(&ray, 0.001, 1000.));
    }

    #[test]
    fn test_aabb_intersection_inside() {
        let bbox = AxisAlignedBoundingBox::new(-1., 1., -1., 1., -1., 1.);
        let ray = Ray::new(Vec3::new(0., 0., 0.), Vec3::new(0., 0., 1.));
        assert!(bbox.intersect(&ray, 0.001, 1000.));
    }

    #[test]
    fn test_aabb_intersection_miss() {
        let bbox = AxisAlignedBoundingBox::new(-1., 1., -1., 1., -1., 1.);
        let ray = Ray::new(Vec3::new(1.1, 0., 0.), Vec3::new(0., 1., 1.));
        assert!(!bbox.intersect(&ray, 0.001, 1000.));
    }

    #[test]
    fn test_aabb_intersection_miss2() {
        let bbox = AxisAlignedBoundingBox::new(-1., 1., -1., 1., -1., 1.);
        let ray = Ray::new(Vec3::new(2., 0., 0.), Vec3::new(-1., -2., 0.));
        assert!(!bbox.intersect(&ray, 0.001, 1000.));
    }

    #[test]
    fn test_bvh_construction_x() {
        let objects: Vec<Object> = (0..8)
            .map(|i| {
                Object::sphere(
                    1.,
                    Vec3::new(-10.5 + 3. * (i as f64), 0., 0.),
                    Material::NoReflect,
                    Emission::Dark,
                )
            })
            .collect();

        let bvh = Bvh::build(objects);
        assert_eq!("Bvh { bvh: Node(AxisAlignedBoundingBox { x_range: -11.5..11.5, y_range: -1.0..1.0, z_range: -1.0..1.0 }, [Node(AxisAlignedBoundingBox { x_range: -11.5..-0.5, y_range: -1.0..1.0, z_range: -1.0..1.0 }, [LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: -10.5, y: 0.0, z: 0.0 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: -7.5, y: 0.0, z: 0.0 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: -4.5, y: 0.0, z: 0.0 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: -1.5, y: 0.0, z: 0.0 } }), mat: NoReflect, emission: Dark })]), Node(AxisAlignedBoundingBox { x_range: 0.5..11.5, y_range: -1.0..1.0, z_range: -1.0..1.0 }, [LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 1.5, y: 0.0, z: 0.0 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 4.5, y: 0.0, z: 0.0 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 7.5, y: 0.0, z: 0.0 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 10.5, y: 0.0, z: 0.0 } }), mat: NoReflect, emission: Dark })])]) }", format!("{:?}", bvh));
    }

    #[test]
    fn test_bvh_construction_y() {
        let objects: Vec<Object> = (0..8)
            .map(|i| {
                Object::sphere(
                    1.,
                    Vec3::new(0., -10.5 + 3. * (i as f64), 0.),
                    Material::NoReflect,
                    Emission::Dark,
                )
            })
            .collect();

        let bvh = Bvh::build(objects);
        assert_eq!("Bvh { bvh: Node(AxisAlignedBoundingBox { x_range: -1.0..1.0, y_range: -11.5..11.5, z_range: -1.0..1.0 }, [Node(AxisAlignedBoundingBox { x_range: -1.0..1.0, y_range: -11.5..-0.5, z_range: -1.0..1.0 }, [LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: -10.5, z: 0.0 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: -7.5, z: 0.0 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: -4.5, z: 0.0 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: -1.5, z: 0.0 } }), mat: NoReflect, emission: Dark })]), Node(AxisAlignedBoundingBox { x_range: -1.0..1.0, y_range: 0.5..11.5, z_range: -1.0..1.0 }, [LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: 1.5, z: 0.0 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: 4.5, z: 0.0 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: 7.5, z: 0.0 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: 10.5, z: 0.0 } }), mat: NoReflect, emission: Dark })])]) }", format!("{:?}", bvh));
    }

    #[test]
    fn test_bvh_construction_z() {
        let objects: Vec<Object> = (0..8)
            .map(|i| {
                Object::sphere(
                    1.,
                    Vec3::new(0., 0., -10.5 + 3. * (i as f64)),
                    Material::NoReflect,
                    Emission::Dark,
                )
            })
            .collect();

        let bvh = Bvh::build(objects);
        assert_eq!("Bvh { bvh: Node(AxisAlignedBoundingBox { x_range: -1.0..1.0, y_range: -1.0..1.0, z_range: -11.5..11.5 }, [Node(AxisAlignedBoundingBox { x_range: -1.0..1.0, y_range: -1.0..1.0, z_range: -11.5..-0.5 }, [LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: 0.0, z: -10.5 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: 0.0, z: -7.5 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: 0.0, z: -4.5 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: 0.0, z: -1.5 } }), mat: NoReflect, emission: Dark })]), Node(AxisAlignedBoundingBox { x_range: -1.0..1.0, y_range: -1.0..1.0, z_range: 0.5..11.5 }, [LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: 0.0, z: 1.5 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: 0.0, z: 4.5 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: 0.0, z: 7.5 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: 0.0, z: 10.5 } }), mat: NoReflect, emission: Dark })])]) }", format!("{:?}", bvh));
    }
}
