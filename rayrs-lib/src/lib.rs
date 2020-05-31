//! rayrs_lib provides the core functionality of the ray-tracer.

pub mod bvh;
pub mod geometry;
pub mod image;
pub mod material;
pub mod test_scenes;
pub mod vecmath;
pub mod wavefront_obj;

use vecmath::{Cross, Unit, Vec3, VecElements, VecUnit};

use geometry::{Axis, AxisAlignedBoundingBox, Hittable, Plane, Sphere, Triangle};
use material::{Emission, Emitting, Material, ScatteringEvent};

use image::Image;

use bvh::{Bvh, BvhHeuristic, RayIntersection};

use std::f64::consts::PI;
use std::ops::Range;

/// Representation of a ray with an origin and direction.
#[derive(Debug, Copy, Clone)]
pub struct Ray {
    /// Ray origin
    pub origin: Vec3,
    /// Ray direction
    ///
    /// Note that the direction is not normalized.
    pub direction: Vec3,
}

impl Ray {
    /// Create a new ray.
    fn new(origin: Vec3, direction: Vec3) -> Ray {
        Ray { origin, direction }
    }

    /// Get the point along the parametric line defined by the ray.
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

/// Pinhole camera model.
#[derive(Debug)]
pub struct Camera {
    origin: Vec3,
    up: Vec3,
    lookat: Vec3,
    fov: f64,
    width: f64,
    height: f64,
    ppc: u32,
    e_x: Unit<Vec3>,
    e_y: Unit<Vec3>,
    z: Vec3,
}

impl Camera {
    /// Create a new Camera.
    ///
    /// The `fov` is the field of view in degrees in the x-direction. The unit
    /// of `width` and `height` is centimeters and internally the
    /// pixels-per-inch, ppi, is converted to pixels-per-centimeters, ppc.
    ///
    /// The up-direction is most commonly chosen to be the y- or z-axis.
    ///
    /// # Panics
    /// - If `fov` is not in the range (0, 180) degrees.
    /// - If `width` is not positive.
    /// - If `height` is not positive.
    /// - If `origin` and `lookat` are the same.
    ///
    /// # Examples
    /// ```
    /// use rayrs_lib::vecmath::Vec3;
    /// use rayrs_lib::Camera;
    ///
    /// let camera = Camera::new(
    ///     Vec3::ones(),
    ///     Vec3::unit_y().as_vec(),
    ///     Vec3::zeros(),
    ///     90.,
    ///     20.,
    ///     10.,
    ///     90,
    /// );
    /// ```
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

        // Construct the right-handed coordinate system. The x-axis will point
        // to the left since `z` points out from the camera origin.
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
            z: (width / (fov.to_radians() / 2.).tan()) * z,
        }
    }

    /// Get the number of pixels in the x-direction.
    ///
    /// # Examples
    /// ```
    /// # use rayrs_lib::Camera;
    /// # use rayrs_lib::vecmath::Vec3;
    /// let camera = Camera::new(
    ///     Vec3::ones(),
    ///     Vec3::unit_y().as_vec(),
    ///     Vec3::zeros(),
    ///     90.,
    ///     20.,
    ///     10.,
    ///     90,
    /// );
    /// let pixels = camera.x_pixels();
    /// assert_eq!(pixels, 4580);
    /// ```
    pub fn x_pixels(&self) -> usize {
        (self.width * (self.ppc as f64)).round() as usize
    }

    /// Get the number of pixels in the y-direction.
    ///
    /// # Examples
    /// ```
    /// # use rayrs_lib::Camera;
    /// # use rayrs_lib::vecmath::Vec3;
    /// let camera = Camera::new(
    ///     Vec3::ones(),
    ///     Vec3::unit_y().as_vec(),
    ///     Vec3::zeros(),
    ///     90.,
    ///     20.,
    ///     10.,
    ///     90,
    /// );
    /// let pixels = camera.y_pixels();
    /// assert_eq!(pixels, 2290);
    /// ```
    pub fn y_pixels(&self) -> usize {
        (self.height * (self.ppc as f64)).round() as usize
    }

    /// Generate a ray from the camera origin.
    ///
    /// # Note
    /// The pixel indices should be within the pixel bounds, otherwise the ray
    /// generate will be outside of the image.
    ///
    /// # Examples
    /// ```
    /// # use rayrs_lib::{Camera, Ray};
    /// # use rayrs_lib::vecmath::Vec3;
    /// let camera = Camera::new(
    ///     Vec3::ones(),
    ///     Vec3::unit_y().as_vec(),
    ///     Vec3::zeros(),
    ///     90.,
    ///     20.,
    ///     10.,
    ///     90,
    /// );
    ///
    /// // Generate a random ray through the first pixel of the image.
    /// let ray = camera.generate_primary_ray(0, 0);
    /// ```
    pub fn generate_primary_ray(&self, i: usize, j: usize) -> Ray {
        let i: f64 = i as f64;
        let j: f64 = j as f64;

        let x: f64 = (j + rand::random::<f64>()) / (self.ppc as f64) - self.width / 2.;
        let y: f64 = (i + rand::random::<f64>()) / (self.ppc as f64) - self.height / 2.;

        Ray::new(self.origin, self.z + x * self.e_x + y * self.e_y)
    }
}

/// The scene representation.
///
/// The `Scene` contains all the objects and other information about the scene.
pub struct Scene {
    bvh: Bvh,
    t_range: Range<f64>,
    hdri: Image,
}

impl Scene {
    /// Create a new Scene from a list of objects.
    ///
    /// # Panics
    /// If `z_near` is not positive or larger than `z_far`.
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

    /// Get the background color for this direction.
    ///
    /// This function maps the direction to spherical coordinates and uses those
    /// to fetch a pixel from the HDRI. The pixels are bilinearly interpolated.
    ///
    /// # Note
    /// The function assumes that the up direction is the y-coordinate.
    fn background(&self, dir: Vec3) -> Vec3 {
        let dir = dir.unit();

        let phi = dir.z().atan2(dir.x()) + PI;
        let theta = dir.y().acos();

        // Get a coordinate between [0, width-2] x [0, height-2].
        let x = phi / (2. * PI) * (self.hdri.width() - 1) as f64;
        let y = theta / PI * (self.hdri.height() - 1) as f64;

        // Get the neighboring pixel coordinates.
        let x_f = x.floor();
        let x_c = x.ceil();
        let y_f = y.floor();
        let y_c = y.ceil();

        let i = y_f as usize;
        let j = x_f as usize;

        let f = [
            self.hdri.pixel(i, j),
            self.hdri.pixel(i + 1, j),
            self.hdri.pixel(i, j + 1),
            self.hdri.pixel(i + 1, j + 1),
        ];

        // Bilinear interpolation
        f[0] * (x_c - x) * (y_c - y)
            + f[1] * (x_c - x) * (y - y_f)
            + f[2] * (x - x_f) * (y_c - y)
            + f[3] * (x - x_f) * (y - y_f)
    }

    /// Get the background color as a mix between white and blue.
    ///
    /// # Note
    /// The function assumes that the up direction is the y-coordinate.
    #[allow(dead_code)]
    fn sky(&self, dir: Vec3) -> Vec3 {
        let y = 0.5 * dir.unit().y() + 1.;
        return 2.0 * (y * Vec3::new(0.5, 0.5, 1.0) + (1. - y) * Vec3::ones());
    }
}

/// An object in the scene.
///
/// Represents an object with the necessary information to do intersection tests
/// and get surface properties at the intersection.
pub struct Object {
    geom: Box<dyn Hittable + Sync>,
    mat: Material,
    emission: Emission,
}

impl Object {
    /// Create a sphere object.
    ///
    /// # Examples
    /// ```
    /// use rayrs_lib::vecmath::Vec3;
    /// use rayrs_lib::Object;
    /// # use rayrs_lib::material::{Material, Emission, LambertianDiffuse};
    /// # let mat = Material::LambertianDiffuse(LambertianDiffuse::new(Vec3::ones()));
    /// # let emission = Emission::Dark;
    /// // `mat` and `emission` created previously.
    /// let sphere = Object::sphere(0.1, Vec3::zeros(), mat, emission);
    /// ```
    pub fn sphere(radius: f64, origin: Vec3, mat: Material, emission: Emission) -> Object {
        Object {
            geom: Box::new(Sphere::new(radius, origin)),
            mat,
            emission,
        }
    }

    /// Create a plane object.
    ///
    /// # Examples
    /// ```
    /// use rayrs_lib::geometry::Axis;
    /// use rayrs_lib::vecmath::Vec3;
    /// use rayrs_lib::Object;
    /// # use rayrs_lib::material::{Material, Emission, LambertianDiffuse};
    /// # let mat = Material::LambertianDiffuse(LambertianDiffuse::new(Vec3::ones()));
    /// # let emission = Emission::Dark;
    /// // `mat` and `emission` created previously.
    /// let plane = Object::plane(Axis::Y, -1., , 1., -1., 1., 0., mat, emission);
    /// ```
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

    /// Create a triangle object.
    ///
    /// # Note
    /// The winding order of the vertices is counter-clockwise.
    ///
    /// # Examples
    /// ```
    /// use rayrs_lib::vecmath::Vec3;
    /// use rayrs_lib::Object;
    /// # use rayrs_lib::material::{Material, Emission, LambertianDiffuse};
    /// # let mat = Material::LambertianDiffuse(LambertianDiffuse::new(Vec3::ones()));
    /// # let emission = Emission::Dark;
    /// // `mat` and `emission` created previously.
    /// let tri = Object::triangle(
    ///     Vec3::new(-1., 0., 0.),
    ///     Vec3::new(1., 0., 0.),
    ///     Vec3::new(0., 1., 0.),
    ///     mat,
    ///     emission,
    /// );
    /// ```
    pub fn triangle(p1: Vec3, p2: Vec3, p3: Vec3, mat: Material, emission: Emission) -> Object {
        Object {
            geom: Box::new(Triangle::new(p1, p2, p3)),
            mat,
            emission,
        }
    }

    /// Create a list of triangle objects.
    ///
    /// Used for making objects out of for example triangle meshes.
    ///
    /// # Note
    /// The winding order of the vertices is counter-clockwise.
    ///
    /// # Examples
    /// ```
    /// use rayrs_lib::vecmath::Vec3;
    /// use rayrs_lib::Object;
    /// # use rayrs_lib::material::{Material, Emission, LambertianDiffuse};
    /// # let mat = Material::LambertianDiffuse(LambertianDiffuse::new(Vec3::ones()));
    /// # let emission = Emission::Dark;
    /// # let tris = Vec::new();
    /// // `mat` and `emission` created previously.
    /// // load mesh into `tris`
    /// let tri_objects = Object::from_triangles(tris, mat, emission);
    /// ```
    pub fn from_triangles(tris: Vec<Triangle>, mat: Material, emission: Emission) -> Vec<Object> {
        tris.into_iter()
            .map(|item| Object {
                geom: Box::new(item),
                mat: mat.clone(),
                emission: emission.clone(),
            })
            .collect()
    }

    /// Create a list of sphere objects.
    ///
    /// Works the same as [`Object::from_triangles`], but for spheres instead.
    ///
    /// [`Object::from_triangles`]: ../struct.Object.html#method.from_triangles
    pub fn from_spheres(spheres: Vec<Sphere>, mat: Material, emission: Emission) -> Vec<Object> {
        spheres
            .into_iter()
            .map(|item| Object {
                geom: Box::new(item),
                mat: mat.clone(),
                emission: emission.clone(),
            })
            .collect()
    }

    /// Create a box.
    ///
    /// Creates a box that spans the lower left corner to the upper right
    /// corner. It works by creating a list of plane objects that define the
    /// box.
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

    /// Get the bounding box of the contained geometry.
    pub fn bbox(&self) -> AxisAlignedBoundingBox {
        self.geom.bbox()
    }
}

/// Calculate the radiance of the scene at the ray origin.
///
/// The function performs up to `max_bounces` ray bounces, calculating the total
/// contribution at the intial ray origin. The maximum number of bounces can be
/// set quite high since the rays are also terminated using Russian Roulette.
///
/// Importance sampling of the BSDFs in the scene is done to reduce variance.
pub fn radiance(s: &Scene, mut r: Ray, max_bounces: u32) -> Vec3 {
    let mut throughput = Vec3::ones();
    let mut light = Vec3::zeros();

    for _ in 0..max_bounces {
        if let RayIntersection::Hit { t, obj } = s.bvh.intersect(r, s.t_range.start, s.t_range.end)
        {
            let position = r.point(t);
            let normal = obj.geom.normal(position);
            let view = (-1. * r.direction).unit();

            match obj.mat.evaluate(position, normal, view, None) {
                ScatteringEvent::Scatter { color, ray } => {
                    light = light + throughput * obj.emission.emit();

                    throughput = throughput * color;

                    let p = throughput.x().max(throughput.y()).max(throughput.z());
                    if rand::random::<f64>() > p {
                        return light;
                    }

                    throughput /= p;

                    // Assign the new ray and go to the next iteration.
                    r = ray;
                }
                // If for some reason we shouldn't continue we just return the
                // total amount of light gathered so far.
                ScatteringEvent::NoScatter => return light,
            }
        } else {
            // We missed the scene so return the light with the background
            // color.
            return light + throughput * s.background(r.direction);
        }
    }

    light
}
