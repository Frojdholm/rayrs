mod bvh;
pub mod geometry;
pub mod image;
pub mod material;
pub mod vecmath;
pub mod wavefront_obj;

use vecmath::Vec3;

use geometry::{Axis, AxisAlignedBoundingBox, Hittable, Plane, Sphere, Triangle};
use material::{Brdf, DiracBrdf, Emission, Emitting, Material, MixKind};

use image::Image;

use bvh::{Bvh, BvhHeuristic, RayIntersection};

use rand::Rng;

use std::f64::consts::PI;
use std::ops::Range;

#[derive(Debug, Copy, Clone)]
pub struct Ray {
    pub origin: Vec3,
    pub direction: Vec3,
}

impl Ray {
    fn new(origin: Vec3, direction: Vec3) -> Ray {
        Ray { origin, direction }
    }

    fn point(&self, t: f64) -> Vec3 {
        self.origin + self.direction * t
    }

    // fn rotate(&self, rot: Vec3) -> Ray {
    //     Ray::new(self.origin.rotate(rot), self.direction.rotate(rot))
    // }

    // fn translate(&self, v: Vec3) -> Ray {
    //     Ray::new(self.origin + v, self.direction)
    // }
}

#[derive(Debug)]
pub struct Camera {
    origin: Vec3,
    up: Vec3,
    lookat: Vec3,
    fov: f64,
    width: f64,
    height: f64,
    ppc: u32,
    e_x: Vec3,
    e_y: Vec3,
    z: Vec3,
}

impl Camera {
    pub fn new(
        origin: Vec3,
        up: Vec3,
        lookat: Vec3,
        fov: f64,
        width: f64,
        height: f64,
        ppi: u32,
    ) -> Camera {
        assert!(fov > 0. && fov < 180.);
        assert!(width > 0.);
        assert!(height > 0.);
        assert_ne!(origin, lookat);

        let ppc = ((ppi as f64) * 2.54).round() as u32;

        let z = (lookat - origin).unit();
        let x = up.cross(z).unit();
        let y = z.cross(x).unit();

        Camera {
            origin,
            up,
            lookat,
            fov,
            width,
            height,
            ppc,
            e_x: x,
            e_y: y,
            z: (width.min(height) / (fov.to_radians() / 2.).tan()) * z,
        }
    }

    pub fn x_pixels(&self) -> usize {
        (self.width * (self.ppc as f64)).round() as usize
    }

    pub fn y_pixels(&self) -> usize {
        (self.height * (self.ppc as f64)).round() as usize
    }

    pub fn generate_primary_ray(&self, i: usize, j: usize) -> Ray {
        let i: f64 = i as f64;
        let j: f64 = j as f64;

        let x: f64 = (j + rand::random::<f64>()) / (self.ppc as f64) - self.width / 2.;
        let y: f64 = (i + rand::random::<f64>()) / (self.ppc as f64) - self.height / 2.;

        Ray::new(self.origin, self.z + x * self.e_x + y * self.e_y)
    }
}

// #[derive(Debug)]
// struct ListAccelerator {
//     objects: Vec<Object>,
// }

// impl ListAccelerator {
//     fn new(objects: Vec<Object>) -> Self {
//         ListAccelerator { objects }
//     }

//     fn intersect(&self, ray: Ray, tmin: f64, tmax: f64) -> Option<(f64, Object)> {
//         let mut t = tmax;
//         let mut obj = None;
//         for child in &self.objects {
//             if let Some(t_hit) = child.geom.intersect(ray) {
//                 if t_hit > tmin && t_hit < t {
//                     t = t_hit;
//                     obj = Some((*child).clone());
//                 }
//             }
//         }

//         if obj.is_some() {
//             Some((t, obj.unwrap()))
//         } else {
//             None
//         }
//     }
// }

pub struct Scene {
    bvh: Bvh,
    t_range: Range<f64>,
    hdri: Image,
}

impl Scene {
    pub fn new(
        objects: Vec<Object>,
        z_near: f64,
        z_far: f64,
        heuristic: BvhHeuristic,
        hdri: Image,
    ) -> Self {
        assert!(z_near >= 0.);
        assert!(z_far > z_near);

        Scene {
            bvh: Bvh::build(heuristic, objects),
            t_range: Range {
                start: z_near,
                end: z_far,
            },
            hdri,
        }
    }

    pub fn fresnel_test(z_near: f64, z_far: f64, hdri: Image) -> Self {
        let white = Material::Diffuse(1., Vec3::new(1., 1., 1.));
        let black = Material::Diffuse(1., Vec3::new(0., 0., 0.));
        let mix = Material::Mix(MixKind::Fresnel(1.5), Box::new(black), Box::new(white));

        let sphere = Object::sphere(0.1, Vec3::new(0., 0., 0.), mix, Emission::Dark);

        Scene::new(
            vec![sphere],
            z_near,
            z_far,
            BvhHeuristic::Sah { splits: 1000 },
            hdri,
        )
    }

    pub fn bvh_test(z_near: f64, z_far: f64, hdri: Image) -> Self {
        let max_spheres = 500;
        let mut rng = rand::thread_rng();

        let mut objects = Vec::with_capacity(max_spheres);

        for _ in 0..max_spheres {
            let sphere = Object::sphere(
                rng.gen_range(0.5, 1.),
                Vec3::new(
                    rng.gen_range(-10., 10.),
                    rng.gen_range(-1., 20.),
                    rng.gen_range(-20., 0.),
                ),
                Material::Diffuse(
                    1.,
                    Vec3::new(
                        rng.gen_range(0.1, 0.9),
                        rng.gen_range(0.1, 0.9),
                        rng.gen_range(0.1, 0.9),
                    ),
                ),
                Emission::Dark,
            );

            objects.push(sphere);
        }

        objects.push(Object::sphere(
            10000.,
            Vec3::new(0., -10001., 0.),
            Material::Diffuse(1., Vec3::new(1., 1., 1.)),
            Emission::Dark,
        ));

        let light = Object::plane(
            Axis::YRev,
            -0.8,
            0.8,
            -0.8,
            0.8,
            2.4999,
            Material::NoReflect,
            Emission::Emissive(5., Vec3::new(1., 1., 1.)),
        );
        objects.push(light);

        Scene::new(
            objects,
            z_near,
            z_far,
            BvhHeuristic::Sah { splits: 1000 },
            hdri,
        )
    }

    pub fn cornell_box(z_near: f64, z_far: f64, hdri: Image) -> Self {
        let green = Material::Diffuse(1., Vec3::new(0., 1., 0.));
        let red = Material::Diffuse(1., Vec3::new(1., 0., 0.));
        let white = Material::Diffuse(1., Vec3::new(1., 1., 1.));
        let mirror = Material::Mirror(1., Vec3::new(1., 1., 1.));
        let mix = Material::Mix(
            MixKind::Fresnel(1.5),
            Box::new(mirror),
            Box::new(white.clone()),
        );
        let cook_torrance = Material::CookTorrance {
            m: 0.5,
            color: Vec3::new(0.8, 0.8, 0.8), //Vec3::new(0.722, 0.451, 0.2),
        };

        let left = Object::plane(Axis::X, -2.5, 2.5, -2.5, 2.5, -2.5, red, Emission::Dark);
        let right = Object::plane(Axis::XRev, -2.5, 2.5, -2.5, 2.5, 2.5, green, Emission::Dark);
        let top = Object::plane(
            Axis::YRev,
            -2.5,
            2.5,
            -2.5,
            2.5,
            2.5,
            white.clone(),
            Emission::Dark,
        );
        let bottom = Object::plane(
            Axis::Y,
            -2.5,
            2.5,
            -2.5,
            2.5,
            -2.5,
            white.clone(),
            Emission::Dark,
        );
        let back = Object::plane(
            Axis::Z,
            -2.5,
            2.5,
            -2.5,
            2.5,
            -2.5,
            white.clone(),
            Emission::Dark,
        );
        let light = Object::plane(
            Axis::YRev,
            -0.8,
            0.8,
            -0.8,
            0.8,
            2.4999,
            Material::NoReflect,
            Emission::Emissive(5., Vec3::new(1., 1., 1.)),
        );

        let sphere1 = Object::sphere(0.5, Vec3::new(-1., -2., 0.5), mix, Emission::Dark);
        let sphere2 = Object::sphere(
            1.,
            Vec3::new(0.5, -1.5, -1.3),
            cook_torrance,
            Emission::Dark,
        );

        let objects = vec![left, right, top, bottom, back, light, sphere1, sphere2];
        Scene::new(objects, z_near, z_far, BvhHeuristic::Midpoint, hdri)
    }

    pub fn cook_torrance_test(z_near: f64, z_far: f64, hdri: Image) -> Self {
        let white = Material::Diffuse(1., Vec3::new(1., 1., 1.));
        let mirror = Material::Mirror(1., Vec3::new(1., 1., 1.));
        let mix = Material::Mix(
            MixKind::Fresnel(1.5),
            Box::new(mirror.clone()),
            Box::new(white.clone()),
        );
        let cook_torrance = Material::CookTorrance {
            m: 0.1,
            color: Vec3::new(0.8, 0.8, 0.8), //Vec3::new(0.722, 0.451, 0.2),
        };

        let bottom = Object::plane(
            Axis::Y,
            -3.5,
            3.5,
            -3.5,
            3.5,
            0.,
            white.clone(),
            Emission::Dark,
        );

        //let sphere1 = Object::sphere(0.5, Vec3::new(-1., -2., 0.5), mix.clone(), Emission::Dark);
        //let sphere2 = Object::sphere(1., Vec3::new(1., -1.5, -0.5), mix, Emission::Dark);
        let sphere = Object::sphere(1., Vec3::new(0., 1., 0.), cook_torrance, Emission::Dark);

        let objects = vec![bottom, sphere];
        Scene::new(objects, z_near, z_far, BvhHeuristic::Midpoint, hdri)
    }

    pub fn dragon(z_near: f64, z_far: f64, hdri: Image) -> Self {
        let green = Material::Diffuse(1., Vec3::new(0., 1., 0.));
        let red = Material::Diffuse(1., Vec3::new(1., 0., 0.));
        let white = Material::Diffuse(1., Vec3::new(1., 1., 1.));
        let mirror = Material::Mirror(1., Vec3::new(1., 1., 1.));
        let mix = Material::Mix(
            MixKind::Fresnel(1.5),
            Box::new(mirror),
            Box::new(white.clone()),
        );

        let left = Object::plane(Axis::X, -2.5, 2.5, -2.5, 2.5, -2.5, red, Emission::Dark);
        let right = Object::plane(Axis::XRev, -2.5, 2.5, -2.5, 2.5, 2.5, green, Emission::Dark);
        let top = Object::plane(
            Axis::YRev,
            -2.5,
            2.5,
            -2.5,
            2.5,
            2.5,
            white.clone(),
            Emission::Dark,
        );
        let bottom = Object::plane(
            Axis::Y,
            -2.5,
            2.5,
            -2.5,
            2.5,
            -2.5,
            white.clone(),
            Emission::Dark,
        );
        let back = Object::plane(Axis::Z, -2.5, 2.5, -2.5, 2.5, -2.5, white, Emission::Dark);
        let light = Object::plane(
            Axis::YRev,
            -0.8,
            0.8,
            -0.8,
            0.8,
            2.4999,
            Material::NoReflect,
            Emission::Emissive(5., Vec3::new(1., 1., 1.)),
        );

        let dragon = wavefront_obj::load_obj_file("dragon.obj").unwrap();
        let mut dragon = Object::from_triangles(dragon, mix, Emission::Dark);
        let mut objects = vec![left, right, top, bottom, back, light];
        objects.append(&mut dragon);
        Scene::new(
            objects,
            z_near,
            z_far,
            BvhHeuristic::Sah { splits: 1000 },
            hdri,
        )
    }

    pub fn background(&self, dir: Vec3) -> Vec3 {
        let dir = dir.unit();

        let phi = dir.z().atan2(dir.x()) + PI;
        let theta = dir.y().acos();

        let x = phi / (2. * PI) * self.hdri.width() as f64;
        let y = theta / PI * self.hdri.height() as f64;
        self.hdri.pixel(y.floor() as usize, x.floor() as usize)
    }
}

pub struct Object {
    geom: Box<dyn Hittable + Sync>,
    mat: Material,
    emission: Emission,
}

impl Object {
    pub fn sphere(radius: f64, origin: Vec3, mat: Material, emission: Emission) -> Object {
        Object {
            geom: Box::new(Sphere::new(radius, origin)),
            mat,
            emission,
        }
    }

    pub fn plane(
        axis: Axis,
        umin: f64,
        umax: f64,
        vmin: f64,
        vmax: f64,
        pos: f64,
        mat: Material,
        emission: Emission,
    ) -> Object {
        Object {
            geom: Box::new(Plane::new(axis, umin, umax, vmin, vmax, pos)),
            mat,
            emission,
        }
    }

    pub fn triangle(p1: Vec3, p2: Vec3, p3: Vec3, mat: Material, emission: Emission) -> Object {
        Object {
            geom: Box::new(Triangle::new(p1, p2, p3)),
            mat,
            emission,
        }
    }

    pub fn from_triangles(tris: Vec<Triangle>, mat: Material, emission: Emission) -> Vec<Object> {
        tris.into_iter()
            .map(|item| Object {
                geom: Box::new(item),
                mat: mat.clone(),
                emission: emission.clone(),
            })
            .collect()
    }

    pub fn box_geom(
        lower_left: Vec3,
        upper_right: Vec3,
        mat: Material,
        emission: Emission,
    ) -> Vec<Object> {
        vec![
            Object::plane(
                Axis::X,
                lower_left.y(),
                upper_right.y(),
                lower_left.z(),
                upper_right.z(),
                lower_left.x(),
                mat.clone(),
                emission.clone(),
            ),
            Object::plane(
                Axis::XRev,
                lower_left.y(),
                upper_right.y(),
                lower_left.z(),
                upper_right.z(),
                upper_right.x(),
                mat.clone(),
                emission.clone(),
            ),
            Object::plane(
                Axis::ZRev,
                lower_left.x(),
                upper_right.x(),
                lower_left.y(),
                upper_right.y(),
                lower_left.z(),
                mat.clone(),
                emission.clone(),
            ),
            Object::plane(
                Axis::Z,
                lower_left.x(),
                upper_right.x(),
                lower_left.y(),
                upper_right.y(),
                upper_right.z(),
                mat.clone(),
                emission.clone(),
            ),
            Object::plane(
                Axis::YRev,
                lower_left.x(),
                upper_right.x(),
                lower_left.z(),
                upper_right.z(),
                lower_left.y(),
                mat.clone(),
                emission.clone(),
            ),
            Object::plane(
                Axis::Y,
                lower_left.x(),
                upper_right.x(),
                lower_left.z(),
                upper_right.z(),
                lower_left.y(),
                mat,
                emission,
            ),
        ]
    }

    pub fn bbox(&self) -> AxisAlignedBoundingBox {
        self.geom.bbox()
    }
}

pub fn radiance(s: &Scene, mut r: Ray, max_bounces: u32) -> Vec3 {
    let mut throughput = Vec3::new(1., 1., 1.);

    for _ in 0..max_bounces {
        if let RayIntersection::Hit { t, obj } = s.bvh.intersect(r, s.t_range.start, s.t_range.end)
        {
            let position = r.point(t);
            let normal = obj.geom.normal(position);
            let incoming = -1. * r.direction.unit();

            let (color, new_ray) = if obj.mat.is_dirac() {
                // Materials that contain dirac deltas need to be handled
                // explicitly.
                let (color, new_ray) = obj.mat.evaluate(position, normal, incoming, None);
                if let Some(new_ray) = new_ray {
                    (color, new_ray)
                } else {
                    return color;
                }
            } else if let Some(pdf) = obj.mat.pdf() {
                // Otherwise if we can sample the material we will do it.
                // let pdf = Pdf::Mix(
                //     MixKind::Constant(0.8),
                //     Box::new(pdf),
                //     Box::new(Pdf::Hittable(s.lights[0].clone())),
                // );

                let outgoing = pdf.generate(position, normal, incoming);
                (
                    obj.mat.brdf(position, normal, incoming, outgoing)
                        / pdf.value(position, normal, incoming, outgoing),
                    Ray::new(position, outgoing),
                )
            } else {
                // If we hit a non-reflecting object we can just return.
                return throughput * obj.emission.emit();
            };
            throughput = throughput * color + obj.emission.emit();

            let p = throughput.x().max(throughput.y()).max(throughput.z());
            if rand::random::<f64>() > p {
                return throughput;
            }

            throughput /= p;
            r = new_ray;
        } else {
            return throughput * s.background(r.direction);
        }
    }

    Vec3::new(0., 0., 0.)
}
