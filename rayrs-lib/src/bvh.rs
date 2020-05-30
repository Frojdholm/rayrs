use super::geometry::{Axis, AxisAlignedBoundingBox};
use super::vecmath::{Vec3, VecElements};
use super::{Object, Ray};

use std::f64;

fn split_ind(axis: Axis, split: f64, data: &[Vec3]) -> Option<usize> {
    match axis {
        Axis::X | Axis::XRev => data.iter().position(|v| v.x() > split),
        Axis::Y | Axis::YRev => data.iter().position(|v| v.y() > split),
        Axis::Z | Axis::ZRev => data.iter().position(|v| v.z() > split),
    }
}

fn calculate_sah(
    cost_traversal: f64,
    cost_intersect: f64,
    surface_area: f64,
    left: &[Object],
    right: &[Object],
) -> f64 {
    let p_left = if left.is_empty() {
        0.
    } else {
        let bbox = AxisAlignedBoundingBox::from_object_list(left);
        bbox.surface_area() / surface_area
    };

    let p_right = if right.is_empty() {
        0.
    } else {
        let bbox = AxisAlignedBoundingBox::from_object_list(right);
        bbox.surface_area() / surface_area
    };

    cost_traversal
        + cost_intersect * (p_left * (left.len() as f64) + p_right * (right.len() as f64))
}

pub enum RayIntersection<'a> {
    Hit { t: f64, obj: &'a Object },
    Miss,
}

impl<'a> RayIntersection<'a> {
    fn new(t: f64, obj: &'a Object) -> RayIntersection {
        RayIntersection::Hit { t, obj }
    }

    fn update(self, intersection: RayIntersection<'a>, tmin: f64) -> RayIntersection {
        match (self, intersection) {
            (Self::Miss, Self::Miss) => Self::Miss,
            (Self::Hit { t, obj }, Self::Miss) => Self::Hit { t, obj },
            (Self::Miss, Self::Hit { t, obj }) => Self::Hit { t, obj },
            (
                Self::Hit {
                    t: t_old,
                    obj: obj_old,
                },
                Self::Hit { t, obj },
            ) => {
                if t > tmin && t < t_old {
                    Self::Hit { t, obj }
                } else {
                    Self::Hit {
                        t: t_old,
                        obj: obj_old,
                    }
                }
            }
        }
    }
}

#[derive(Clone)]
enum BvhDataSorted {
    Sorted(Axis),
    Unsorted,
}

struct BvhData {
    objects: Vec<Object>,
    centers: Vec<Vec3>,
    sorted: BvhDataSorted,
}

impl BvhData {
    fn new(objects: Vec<Object>) -> BvhData {
        let centers = objects.iter().map(|obj| obj.bbox().center()).collect();
        BvhData {
            objects,
            centers,
            sorted: BvhDataSorted::Unsorted,
        }
    }

    fn sort(&mut self, axis: Axis) {
        match axis {
            Axis::X | Axis::XRev => {
                self.objects.sort_by(|a, b| {
                    a.bbox()
                        .center()
                        .x()
                        .partial_cmp(&b.bbox().center().x())
                        .unwrap()
                });
                self.centers
                    .sort_by(|a, b| a.x().partial_cmp(&b.x()).unwrap());
                self.sorted = BvhDataSorted::Sorted(Axis::X)
            }
            Axis::Y | Axis::YRev => {
                self.objects.sort_by(|a, b| {
                    a.bbox()
                        .center()
                        .y()
                        .partial_cmp(&b.bbox().center().y())
                        .unwrap()
                });
                self.centers
                    .sort_by(|a, b| a.y().partial_cmp(&b.y()).unwrap());
                self.sorted = BvhDataSorted::Sorted(Axis::Y);
            }
            Axis::Z | Axis::ZRev => {
                self.objects.sort_by(|a, b| {
                    a.bbox()
                        .center()
                        .z()
                        .partial_cmp(&b.bbox().center().z())
                        .unwrap()
                });
                self.centers
                    .sort_by(|a, b| a.z().partial_cmp(&b.z()).unwrap());
                self.sorted = BvhDataSorted::Sorted(Axis::Z);
            }
        }
    }

    fn split_index(&self, val: f64) -> Option<usize> {
        match &self.sorted {
            BvhDataSorted::Unsorted => None,
            BvhDataSorted::Sorted(axis) => split_ind(*axis, val, &self.centers),
        }
    }

    fn split(&self, ind: usize) -> (&[Object], &[Object]) {
        assert!(ind < self.objects.len());
        (&self.objects[..ind], &self.objects[ind..])
    }

    fn split_consume(mut self, ind: usize) -> (BvhData, BvhData) {
        assert!(ind < self.objects.len());
        let right_objects = self.objects.split_off(ind);
        let right_centers = self.centers.split_off(ind);
        let sorted = self.sorted.clone();
        (
            self,
            BvhData {
                objects: right_objects,
                centers: right_centers,
                sorted,
            },
        )
    }

    fn len(&self) -> usize {
        self.objects.len()
    }

    fn is_empty(&self) -> bool {
        self.objects.is_empty()
    }

    fn bbox(&self) -> AxisAlignedBoundingBox {
        AxisAlignedBoundingBox::from_object_list(&self.objects)
    }

    fn into_obj(mut self) -> Object {
        assert!(self.objects.len() == 1);
        self.objects.remove(0)
    }

    fn into_obj_vec(self) -> Vec<Object> {
        self.objects
    }
}

#[derive(Debug)]
pub enum BvhHeuristic {
    Midpoint,
    Sah { splits: u32 },
}

pub struct Bvh {
    bvh: BvhTree,
    //heuristic: BvhHeuristic,
}

impl Bvh {
    pub fn build(heuristic: BvhHeuristic, objects: Vec<Object>) -> Self {
        match heuristic {
            BvhHeuristic::Midpoint => Bvh {
                bvh: BvhTree::build_midpoint(BvhData::new(objects)),
                //heuristic,
            },
            BvhHeuristic::Sah { splits } => Bvh {
                bvh: BvhTree::build_sah(BvhData::new(objects), splits),
                //heuristic,
            },
        }
    }

    pub fn intersect(&self, ray: Ray, tmin: f64, tmax: f64) -> RayIntersection {
        self.bvh.intersect(ray, tmin, tmax)
    }

    // pub fn volume(&self) -> f64 {
    //     self.bvh.volume()
    // }
}

enum BvhTree {
    Node(AxisAlignedBoundingBox, Vec<BvhTree>),
    LeafNode(Object),
}

impl BvhTree {
    fn build_sah(mut data: BvhData, splits: u32) -> Self {
        // Having a BVH for 0 objects does not make sense
        assert!(!data.is_empty());

        let bbox = data.bbox();

        if data.len() > 4 {
            // We only want 4 or less objects in the leaf nodes. So we split
            // this node along its longest axis and continue.

            // TODO: Make this more understandable..
            let x = bbox.xmax() - bbox.xmin();
            let y = bbox.ymax() - bbox.ymin();
            let z = bbox.zmax() - bbox.zmin();
            let surface_area = bbox.surface_area();

            // Find the index to split the sorted objects. If splitting by the
            // SAH of the bounding box would lead to having all elements
            // in one side we split along object mean position instead.

            // We split by the longest axis first.
            let (min, len) = if x >= y && x >= z {
                data.sort(Axis::X);
                (bbox.xmin(), x)
            } else if y >= z {
                data.sort(Axis::Y);
                (bbox.ymin(), y)
            } else {
                data.sort(Axis::Z);
                (bbox.zmin(), z)
            };

            let split_dist = len / (splits - 1) as f64;
            let mut min_ind = None;
            let mut min_sah = f64::INFINITY;
            for i in 1..splits + 1 {
                if let Some(ind) = data.split_index(min + i as f64 * split_dist) {
                    let (left, right) = data.split(ind);
                    let sah = calculate_sah(0.3, 1., surface_area, left, right);
                    if sah < min_sah {
                        min_sah = sah;
                        min_ind = Some(ind);
                    }
                }
            }

            // If min_ind == None or ind == 0 or ind == data.len()-1 then all
            // objects are grouped in the split axis so we just split in the
            // middle.
            //
            // - min_ind == None happens when all objects are stacked at < split_dist
            // - ind == data.len-1 happens when all objects are stacked at bbox._max()
            let ind = if let Some(ind) = min_ind {
                if ind == 0 || ind == data.len() - 1 {
                    data.len() / 2
                } else {
                    ind
                }
            } else {
                data.len() / 2
            };

            let (left, right) = data.split_consume(ind);

            BvhTree::Node(
                bbox,
                vec![
                    if left.len() > 1 {
                        BvhTree::build_sah(left, splits)
                    } else {
                        BvhTree::LeafNode(left.into_obj())
                    },
                    if right.len() > 1 {
                        BvhTree::build_sah(right, splits)
                    } else {
                        BvhTree::LeafNode(right.into_obj())
                    },
                ],
            )
        } else {
            // When we reach a small enough number of objects we can just add
            // them as leaf nodes.
            BvhTree::Node(
                bbox,
                data.into_obj_vec()
                    .into_iter()
                    .map(BvhTree::LeafNode)
                    .collect(),
            )
        }
    }

    fn build_midpoint(mut data: BvhData) -> Self {
        // Having a BVH for 0 objects does not make sense
        assert!(!data.is_empty());

        let bbox = data.bbox();

        if data.len() > 4 {
            // We only want 4 or less objects in the leaf nodes. So we split
            // this node along its longest axis and continue.

            // TODO: Make this more understandable..
            let x = bbox.xmax() - bbox.xmin();
            let y = bbox.ymax() - bbox.ymin();
            let z = bbox.zmax() - bbox.zmin();

            // Find the index to split the sorted objects. If splitting by the
            // midpoint of the bounding box would lead to having all elements
            // in one side we split along object mean position instead.
            let split = if x >= y && x >= z {
                data.sort(Axis::X);
                bbox.center().x()
            } else if y >= z {
                data.sort(Axis::Y);
                bbox.center().y()
            } else {
                data.sort(Axis::Z);
                bbox.center().z()
            };

            let ind = data.split_index(split);

            // If ind is None or such that all objects would be in the same
            // container split the list in half instead.
            let ind = if let Some(ind) = ind {
                if ind == 0 || ind == data.len() - 1 {
                    data.len() / 2
                } else {
                    ind
                }
            } else {
                data.len() / 2
            };

            let (left, right) = data.split_consume(ind);
            BvhTree::Node(
                bbox,
                vec![
                    if left.len() > 1 {
                        BvhTree::build_midpoint(left)
                    } else {
                        BvhTree::LeafNode(left.into_obj())
                    },
                    if right.len() > 1 {
                        BvhTree::build_midpoint(right)
                    } else {
                        BvhTree::LeafNode(right.into_obj())
                    },
                ],
            )
        } else {
            // When we reach a small enough number of objects we can just add
            // them as leaf nodes.
            BvhTree::Node(
                bbox,
                data.into_obj_vec()
                    .into_iter()
                    .map(BvhTree::LeafNode)
                    .collect(),
            )
        }
    }

    fn intersect(&self, ray: Ray, tmin: f64, tmax: f64) -> RayIntersection {
        match self {
            BvhTree::Node(bbox, children) => {
                if bbox.intersect(ray, tmin, tmax) {
                    children
                        .iter()
                        .fold(RayIntersection::Miss, |intersection, child| {
                            intersection.update(child.intersect(ray, tmin, tmax), tmin)
                        })
                } else {
                    RayIntersection::Miss
                }
            }
            BvhTree::LeafNode(obj) => match obj.geom.intersect(ray) {
                Some(t) => {
                    if t > tmin && t < tmax {
                        RayIntersection::new(t, obj)
                    } else {
                        RayIntersection::Miss
                    }
                }
                None => RayIntersection::Miss,
            },
        }
    }

    // fn volume(&self) -> f64 {
    //     match self {
    //         BvhTree::Node(bbox, children) => {
    //             let volume = children.iter().fold(0., |acc, el| acc + el.volume());
    //             volume + bbox.volume()
    //         }
    //         BvhTree::LeafNode(obj) => {
    //             let bbox: AxisAlignedBoundingBox = obj.bbox();
    //             bbox.volume()
    //         }
    //     }
    // }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::{Emission, Material};

    // #[test]
    // fn test_bvh_midpoint_construction_x() {
    //     let objects: Vec<Object> = (0..8)
    //         .map(|i| {
    //             Object::sphere(
    //                 1.,
    //                 Vec3::new(-10.5 + 3. * (i as f64), 0., 0.),
    //                 Material::NoReflect,
    //                 Emission::Dark,
    //             )
    //         })
    //         .collect();

    //     let bvh = Bvh::build(BvhHeuristic::Midpoint, objects);
    //     //assert_eq!("Bvh { bvh: Node(AxisAlignedBoundingBox { x_range: -11.5..11.5, y_range: -1.0..1.0, z_range: -1.0..1.0 }, [Node(AxisAlignedBoundingBox { x_range: -11.5..-0.5, y_range: -1.0..1.0, z_range: -1.0..1.0 }, [LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: -10.5, y: 0.0, z: 0.0 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: -7.5, y: 0.0, z: 0.0 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: -4.5, y: 0.0, z: 0.0 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: -1.5, y: 0.0, z: 0.0 } }), mat: NoReflect, emission: Dark })]), Node(AxisAlignedBoundingBox { x_range: 0.5..11.5, y_range: -1.0..1.0, z_range: -1.0..1.0 }, [LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 1.5, y: 0.0, z: 0.0 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 4.5, y: 0.0, z: 0.0 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 7.5, y: 0.0, z: 0.0 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 10.5, y: 0.0, z: 0.0 } }), mat: NoReflect, emission: Dark })])]), heuristic: Midpoint }", format!("{:?}", bvh));
    // }

    // #[test]
    // fn test_bvh_midpoint_construction_y() {
    //     let objects: Vec<Object> = (0..8)
    //         .map(|i| {
    //             Object::sphere(
    //                 1.,
    //                 Vec3::new(0., -10.5 + 3. * (i as f64), 0.),
    //                 Material::NoReflect,
    //                 Emission::Dark,
    //             )
    //         })
    //         .collect();

    //     let bvh = Bvh::build(BvhHeuristic::Midpoint, objects);
    //     //assert_eq!("Bvh { bvh: Node(AxisAlignedBoundingBox { x_range: -1.0..1.0, y_range: -11.5..11.5, z_range: -1.0..1.0 }, [Node(AxisAlignedBoundingBox { x_range: -1.0..1.0, y_range: -11.5..-0.5, z_range: -1.0..1.0 }, [LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: -10.5, z: 0.0 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: -7.5, z: 0.0 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: -4.5, z: 0.0 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: -1.5, z: 0.0 } }), mat: NoReflect, emission: Dark })]), Node(AxisAlignedBoundingBox { x_range: -1.0..1.0, y_range: 0.5..11.5, z_range: -1.0..1.0 }, [LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: 1.5, z: 0.0 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: 4.5, z: 0.0 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: 7.5, z: 0.0 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: 10.5, z: 0.0 } }), mat: NoReflect, emission: Dark })])]), heuristic: Midpoint }", format!("{:?}", bvh));
    // }

    // #[test]
    // fn test_bvh_midpoint_construction_z() {
    //     let objects: Vec<Object> = (0..8)
    //         .map(|i| {
    //             Object::sphere(
    //                 1.,
    //                 Vec3::new(0., 0., -10.5 + 3. * (i as f64)),
    //                 Material::NoReflect,
    //                 Emission::Dark,
    //             )
    //         })
    //         .collect();

    //     let bvh = Bvh::build(BvhHeuristic::Midpoint, objects);
    //     //assert_eq!("Bvh { bvh: Node(AxisAlignedBoundingBox { x_range: -1.0..1.0, y_range: -1.0..1.0, z_range: -11.5..11.5 }, [Node(AxisAlignedBoundingBox { x_range: -1.0..1.0, y_range: -1.0..1.0, z_range: -11.5..-0.5 }, [LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: 0.0, z: -10.5 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: 0.0, z: -7.5 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: 0.0, z: -4.5 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: 0.0, z: -1.5 } }), mat: NoReflect, emission: Dark })]), Node(AxisAlignedBoundingBox { x_range: -1.0..1.0, y_range: -1.0..1.0, z_range: 0.5..11.5 }, [LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: 0.0, z: 1.5 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: 0.0, z: 4.5 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: 0.0, z: 7.5 } }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Sphere(Sphere { radius2: 1.0, origin: Vec3 { x: 0.0, y: 0.0, z: 10.5 } }), mat: NoReflect, emission: Dark })])]), heuristic: Midpoint }", format!("{:?}", bvh));
    // }

    // #[test]
    // fn test_bvh_midpoint_construction_sphere_in_center() {
    //     let top = Object::plane(
    //         Axis::YRev,
    //         -2.5,
    //         2.5,
    //         -2.5,
    //         2.5,
    //         2.5,
    //         Material::NoReflect,
    //         Emission::Dark,
    //     );
    //     let bottom = Object::plane(
    //         Axis::Y,
    //         -2.5,
    //         2.5,
    //         -2.5,
    //         2.5,
    //         -2.5,
    //         Material::NoReflect,
    //         Emission::Dark,
    //     );
    //     let back = Object::plane(
    //         Axis::Z,
    //         -2.5,
    //         2.5,
    //         -2.5,
    //         2.5,
    //         -2.5,
    //         Material::NoReflect,
    //         Emission::Dark,
    //     );
    //     let light = Object::plane(
    //         Axis::YRev,
    //         -0.8,
    //         0.8,
    //         -0.8,
    //         0.8,
    //         2.4999,
    //         Material::NoReflect,
    //         Emission::Emissive(5., Vec3::new(1., 1., 1.)),
    //     );
    //     let triangle1 = Object::triangle(
    //         Vec3::new(-1., -1., -0.5),
    //         Vec3::new(-0.5, -1., -1.),
    //         Vec3::new(1., 1., -1.5),
    //         Material::NoReflect,
    //         Emission::Dark,
    //     );

    //     let objects = vec![top, bottom, back, light, triangle1];

    //     let bvh = Bvh::build(BvhHeuristic::Midpoint, objects);
    //     //assert_eq!("Bvh { bvh: Node(AxisAlignedBoundingBox { x_range: -2.5..2.5, y_range: -2.5..2.5, z_range: -2.5..2.5 }, [Node(AxisAlignedBoundingBox { x_range: -2.5..2.5, y_range: -2.5..2.5, z_range: -2.5..2.5 }, [LeafNode(Object { geom: Plane(Plane { u_range: -2.5..2.5, v_range: -2.5..2.5, pos: 2.5, axis: YRev }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Plane(Plane { u_range: -2.5..2.5, v_range: -2.5..2.5, pos: -2.5, axis: Y }), mat: NoReflect, emission: Dark })]), Node(AxisAlignedBoundingBox { x_range: -2.5..2.5, y_range: -2.5..2.5, z_range: -2.5..0.8 }, [LeafNode(Object { geom: Plane(Plane { u_range: -2.5..2.5, v_range: -2.5..2.5, pos: -2.5, axis: Z }), mat: NoReflect, emission: Dark }), LeafNode(Object { geom: Plane(Plane { u_range: -0.8..0.8, v_range: -0.8..0.8, pos: 2.4999, axis: YRev }), mat: NoReflect, emission: Emissive(5.0, Vec3 { x: 1.0, y: 1.0, z: 1.0 }) }), LeafNode(Object { geom: Triangle(Triangle { p1: Vec3 { x: -1.0, y: -1.0, z: -0.5 }, p2: Vec3 { x: -0.5, y: -1.0, z: -1.0 }, p3: Vec3 { x: 1.0, y: 1.0, z: -1.5 }, e1: Vec3 { x: 0.5, y: 0.0, z: -0.5 }, e2: Vec3 { x: 2.0, y: 2.0, z: -1.0 }, normal: Vec3 { x: 0.6666666666666666, y: -0.3333333333333333, z: 0.6666666666666666 }, area: 0.75 }), mat: NoReflect, emission: Dark })])]), heuristic: Midpoint }", format!("{:?}", bvh));
    // }

    #[test]
    fn test_bvh_intersect_node_leafnode() {
        let sphere = Object::sphere(
            1.,
            Vec3::new(0., 0., 0.),
            Material::NoReflect,
            Emission::Dark,
        );
        let bvh = BvhTree::Node(sphere.bbox(), vec![BvhTree::LeafNode(sphere)]);
        let ray = Ray::new(Vec3::new(-5., 0., 0.), Vec3::new(1., 0., 0.));
        let t = if let RayIntersection::Hit { t, obj: _ } = bvh.intersect(ray, 0.001, 1000.) {
            t
        } else {
            panic!("Missed")
        };
        assert_eq!(4., t);
    }
}
