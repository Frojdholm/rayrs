pub mod geometry;
pub mod material;
pub mod vecmath;
pub mod image;

use vecmath::Vec3;

use geometry::{Axis, Bvh, BvhHeuristic, Geometry, Hittable, Plane, Sphere};
use material::{Brdf, DiracBrdf, Emission, Emitting, Material, MixKind, Pdf};

use rand::Rng;

use std::ops::{Add, Mul, Range};

#[derive(Debug, Clone)]
pub struct Ray {
    pub origin: Vec3,
    pub direction: Vec3,
}

impl Ray {
    fn new(origin: Vec3, direction: Vec3) -> Ray {
        Ray { origin, direction }
    }

    fn point(&self, t: f64) -> Vec3 {
        &self.origin + &(&self.direction * t)
    }

    fn rotate(&self, rot: &Vec3) -> Ray {
        Ray::new(self.origin.rotate(rot), self.direction.rotate(rot))
    }

    fn translate(&self, v: &Vec3) -> Ray {
        Ray::new(&self.origin + &v, self.direction.clone())
    }
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

        let z = (&lookat - &origin).unit();
        let x = up.cross(&z).unit();
        let y = z.cross(&x).unit();

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
            z: (width.min(height) / (fov.to_radians() / 2.).tan()) * &z,
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

        Ray::new(
            self.origin.clone(),
            self.z.add(&self.e_x.mul(x)).add(&self.e_y.mul(y)),
        )
    }
}

#[derive(Debug)]
pub struct Scene {
    bvh: Bvh,
    lights: Vec<Geometry>,
    t_range: Range<f64>,
}

impl Scene {
    pub fn new(objects: Vec<Object>, z_near: f64, z_far: f64) -> Self {
        assert!(z_near >= 0.);
        assert!(z_far > z_near);

        let mut lights = Vec::new();
        for obj in objects.iter() {
            if let Emission::Emissive(_, _) = obj.emission {
                lights.push(obj.geom.clone());
            }
        }

        Scene {
            bvh: Bvh::build(BvhHeuristic::Sah, objects),
            lights,
            t_range: Range {
                start: z_near,
                end: z_far,
            },
        }
    }

    pub fn fresnel_test(z_near: f64, z_far: f64) -> Self {
        let white = Material::Diffuse(1., Vec3::new(1., 1., 1.));
        let black = Material::Diffuse(1., Vec3::new(0., 0., 0.));
        let mix = Material::Mix(
            MixKind::Fresnel(1.5),
            Box::new(black.clone()),
            Box::new(white.clone()),
        );

        let sphere = Object::sphere(1., Vec3::new(0., 0., 0.), mix.clone(), Emission::Dark);

        Scene::new(vec![sphere], z_near, z_far)
    }

    pub fn bvh_test(z_near: f64, z_far: f64) -> Self {
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

        Scene::new(objects, z_near, z_far)
    }

    pub fn cornell_box(z_near: f64, z_far: f64) -> Self {
        let green = Material::Diffuse(1., Vec3::new(0., 1., 0.));
        let red = Material::Diffuse(1., Vec3::new(1., 0., 0.));
        let white = Material::Diffuse(1., Vec3::new(1., 1., 1.));
        let mirror = Material::Mirror(1., Vec3::new(1., 1., 1.));
        let mix = Material::Mix(
            MixKind::Fresnel(1.5),
            Box::new(mirror.clone()),
            Box::new(white.clone()),
        );

        let left = Object::plane(
            Axis::X,
            -2.5,
            2.5,
            -2.5,
            2.5,
            -2.5,
            red.clone(),
            Emission::Dark,
        );
        let right = Object::plane(
            Axis::XRev,
            -2.5,
            2.5,
            -2.5,
            2.5,
            2.5,
            green.clone(),
            Emission::Dark,
        );
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

        let sphere1 = Object::sphere(0.5, Vec3::new(-1., -2., 0.5), mix.clone(), Emission::Dark);
        let sphere2 = Object::sphere(1., Vec3::new(0.5, -1.5, -1.3), mix.clone(), Emission::Dark);

        let objects = vec![left, right, top, bottom, back, light, sphere1, sphere2];
        Scene::new(objects, z_near, z_far)
    }

    pub fn background(&self, _dir: &Vec3) -> Vec3 {
        // let y = (dir.unit().y + 1.) / 2.;
        // &Vec3::new(0., 0., 1.).mul(y) + &Vec3::new(1., 1., 1.).mul(1. - y)
        Vec3::new(0., 0., 0.)
    }
}

#[derive(Debug, Clone)]
pub struct Object {
    geom: Geometry,
    mat: Material,
    emission: Emission,
}

impl Object {
    pub fn sphere(radius: f64, origin: Vec3, mat: Material, emission: Emission) -> Object {
        Object {
            geom: Geometry::Sphere(Sphere::new(radius, origin)),
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
            geom: Geometry::Plane(Plane::new(axis, umin, umax, vmin, vmax, pos)),
            mat,
            emission,
        }
    }

    pub fn rotated_translated_plane(
        axis: Axis,
        umin: f64,
        umax: f64,
        vmin: f64,
        vmax: f64,
        pos: f64,
        mat: Material,
        emission: Emission,
        rot: Vec3,
        v: Vec3,
    ) -> Object {
        Object {
            geom: Geometry::Rotated(
                rot,
                Box::new(Geometry::Translated(
                    v,
                    Box::new(Geometry::Plane(Plane::new(
                        axis, umin, umax, vmin, vmax, pos,
                    ))),
                )),
            ),
            mat,
            emission,
        }
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
                lower_left.y,
                upper_right.y,
                lower_left.z,
                upper_right.z,
                lower_left.x,
                mat.clone(),
                emission.clone(),
            ),
            Object::plane(
                Axis::XRev,
                lower_left.y,
                upper_right.y,
                lower_left.z,
                upper_right.z,
                upper_right.x,
                mat.clone(),
                emission.clone(),
            ),
            Object::plane(
                Axis::ZRev,
                lower_left.x,
                upper_right.x,
                lower_left.y,
                upper_right.y,
                lower_left.z,
                mat.clone(),
                emission.clone(),
            ),
            Object::plane(
                Axis::Z,
                lower_left.x,
                upper_right.x,
                lower_left.y,
                upper_right.y,
                upper_right.z,
                mat.clone(),
                emission.clone(),
            ),
            Object::plane(
                Axis::YRev,
                lower_left.x,
                upper_right.x,
                lower_left.z,
                upper_right.z,
                lower_left.y,
                mat.clone(),
                emission.clone(),
            ),
            Object::plane(
                Axis::Y,
                lower_left.x,
                upper_right.x,
                lower_left.z,
                upper_right.z,
                lower_left.y,
                mat.clone(),
                emission.clone(),
            ),
        ]
    }
}

pub fn radiance(s: &Scene, mut r: Ray, max_bounces: u32, rays: &mut f64) -> Vec3 {
    let mut throughput = Vec3::new(1., 1., 1.);

    for _ in 0..max_bounces {
        *rays += 1.;
        if let Some((t, obj)) = s.bvh.intersect(&r, s.t_range.start, s.t_range.end) {
            let position = r.point(t);
            let normal = obj.geom.normal(&position);
            let incoming = r.direction.unit().mul(-1.);

            let (color, new_ray) = if obj.mat.is_dirac() {
                // Materials that contain dirac deltas need to be handled
                // explicitly.
                let (color, new_ray) = obj.mat.evaluate(
                    &position,
                    &normal,
                    &incoming,
                    Some(Pdf::Hittable(s.lights[0].clone())),
                );
                if new_ray.is_none() {
                    return color;
                } else {
                    (color, new_ray.unwrap())
                }
            } else if let Some(pdf) = obj.mat.pdf() {
                // Otherwise if we can sample the material we will do it.
                let pdf = Pdf::Mix(
                    MixKind::Constant(0.8),
                    Box::new(pdf),
                    Box::new(Pdf::Hittable(s.lights[0].clone())),
                );

                let outgoing = pdf.generate(&position, &normal, &incoming);
                (
                    obj.mat
                        .brdf(&position, &normal, &incoming, &outgoing)
                        .mul(1. / pdf.value(&position, &normal, &incoming, &outgoing)),
                    Ray::new(position, outgoing),
                )
            } else {
                // If we hit a non-reflecting object we can just return.
                return &throughput * &obj.emission.emit();
            };
            throughput = throughput.mul(&color).add(&obj.emission.emit());

            let p = throughput.x.max(throughput.y).max(throughput.z);
            if rand::random::<f64>() > p {
                return throughput;
            }

            throughput = throughput.mul(1. / p);
            r = new_ray;
        } else {
            return &throughput * &s.background(&r.direction);
        }
    }

    Vec3::new(0., 0., 0.)
}
