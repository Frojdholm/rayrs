//! Material models
//!
//! This module contains material models and their scattering behavior. It also
//! contains other utilities needed to evaluate materials such as different
//! functions for calculating Fresnel coefficients and PDFs.
//!
//! A functioning material can be created by wrapping one of the material models
//! inside of the [`Material`] enum. The [`evaulate`] function can then be
//! called to evaluate the material.
//!
//! # Examples
//! When a ray-object intersection has been found the position, normal and view
//! vectors can be retrieved. Using these and an optional pdf for importance
//! sampling the material can be evaluated. The return value is the
//! [`ScatteringEvent`] enum which can be either the [`Scatter`] or the
//! [`NoScatter`] variants depending on if the ray terminated or not.
//!
//! If the result is `ScatteringEvent::Scatter` then it contains the new
//! sampled ray and the throughput of the ray at this intersection.
//!
//! ```
//! use rayrs_lib::material::{LambertianDiffuse, Material, ScatteringEvent};
//! use rayrs_lib::vecmath::{Vec3, VecUnit};
//!
//! let mat = Material::LambertianDiffuse(LambertianDiffuse::new(Vec3::ones() * 0.8));
//!
//! // Hypothetical intersection
//! let position = Vec3::zeros();
//! let normal = Vec3::unit_z();
//! let view = Vec3::new(1., 0., 1.).unit();
//!
//! match mat.evaluate(position, normal, view, None) {
//!     ScatteringEvent::Scatter { color: _, ray: _ } => (), // Handle scatter
//!     ScatteringEvent::NoScatter => (),                    // Handle no scatter
//! }
//! ```
//!
//!
//! [`Material`]: ../material/enum.Material.html
//! [`evaluate`]: ../material/enum.Material.html#method.evaluate
//! [`ScatteringEvent`]: ../material/enum.ScatteringEvent.html
//! [`Scatter`]: ../material/enum.ScatteringEvent.html#variant.Scatter
//! [`NoScatter`]: ../material/enum.ScatteringEvent.html#variant.NoScatter

use super::geometry::Hittable;
use super::vecmath::{Dot, Unit, Vec3, VecUnit};
use super::Ray;

use std::f64;
use std::f64::consts::{FRAC_1_PI, FRAC_PI_2, PI};

/// Material wrapper enum.
///
/// The Material enum wraps all the material models so that they can be stored
/// in other structs. It also provides a single method for evaluating all
/// different types of materials.
#[derive(Debug, Clone, Copy)]
pub enum Material {
    LambertianDiffuse(LambertianDiffuse),
    Reflect(Reflect),
    Refract(Refract),
    Glass(Glass),
    CookTorrance(CookTorrance),
    CookTorranceRefract(CookTorranceRefract),
    CookTorranceGlass(CookTorranceGlass),
    Plastic(Plastic),
    NoReflect,
}

impl Material {
    /// Evaluate the material at the intersection.
    ///
    /// The pdf can be used to optionally importance sample some other
    /// direction than those specified by the material model.
    ///
    /// # Examples
    /// ```
    /// # use rayrs_lib::material::{LambertianDiffuse, Material, ScatteringEvent};
    /// # use rayrs_lib::vecmath::{Vec3, VecUnit};
    /// # let mat = Material::LambertianDiffuse(LambertianDiffuse::new(Vec3::ones() * 0.8));
    /// // Hypothetical intersection
    /// let position = Vec3::zeros();
    /// let normal = Vec3::unit_z();
    /// let view = Vec3::new(1., 0., 1.).unit();
    ///
    /// match mat.evaluate(position, normal, view, None) {
    ///     ScatteringEvent::Scatter { color: _, ray: _ } => (), // Handle scatter
    ///     ScatteringEvent::NoScatter => (),                    // Handle no scatter
    /// }
    /// ```
    pub fn evaluate(
        &self,
        position: Vec3,
        normal: Unit<Vec3>,
        view: Unit<Vec3>,
        pdf: Option<Pdf>,
    ) -> ScatteringEvent {
        match self {
            Self::LambertianDiffuse(d) => d.scatter(position, normal, view, pdf),
            Self::Reflect(r) => r.scatter(position, normal, view, pdf),
            Self::Refract(r) => r.scatter(position, normal, view, pdf),
            Self::Glass(g) => g.scatter(position, normal, view, pdf),
            Self::CookTorrance(ct) => ct.scatter(position, normal, view, pdf),
            Self::CookTorranceRefract(ctr) => ctr.scatter(position, normal, view, pdf),
            Self::CookTorranceGlass(ctg) => ctg.scatter(position, normal, view, pdf),
            Self::Plastic(p) => p.scatter(position, normal, view, pdf),
            Self::NoReflect => ScatteringEvent::NoScatter,
        }
    }
}

/// Represents a possible scattering event at a material interface
#[derive(Debug, Clone, Copy)]
pub enum ScatteringEvent {
    /// The result if the ray did not terminate.
    ///
    /// `color` is the throughput of the ray at this material interface and
    /// `ray` is the newly sampled ray.
    Scatter {
        color: Vec3,
        ray: Ray,
    },
    NoScatter,
}

/// A Fresnel coefficient.
#[derive(Debug, Clone, Copy)]
pub enum Fresnel {
    SchlickDielectric(f64),
    SchlickMetallic(Vec3),
}

/// Bidirectional scattering distribution function.
///
/// The BSDF captures both BRDF and BTDF material interactions.
pub trait Bsdf {
    /// Returns the result of scattering the light at the incident point.
    fn scatter(
        &self,
        position: Vec3,
        normal: Unit<Vec3>,
        view: Unit<Vec3>,
        pdf: Option<Pdf>,
    ) -> ScatteringEvent;
}

/// A simple BRDF modelling perfectly random scattering at the surface.
#[derive(Debug, Clone, Copy)]
pub struct LambertianDiffuse {
    color: Vec3,
}

/// A BRDF model for perfect specular reflection.
#[derive(Debug, Clone, Copy)]
pub struct Reflect {
    color: Vec3,
}

/// A BTDF model for perfect specular refraction.
#[derive(Debug, Clone, Copy)]
pub struct Refract {
    /// Index of refraction.
    ior: f64,
    color: Vec3,
}

/// A BSDF model describing glass.
///
/// This BSDF is just a mix material that mixes a specular reflection BRDF
/// and a specular refraction BTDF. It does so more efficiently than a mix
/// material since it can importance sample the Fresnel coefficient.
#[derive(Debug, Clone, Copy)]
pub struct Glass {
    ior: f64,
    refract: Refract,
    reflect: Reflect,
}

/// The Cook-Torrance material model.
///
/// The Cook-Torrance model is a microfacet model that describes specular
/// reflection. It has the formula
/// ```text
///     f(w_i, w_o) = DFG / (4pi * cos(theta_i) * cos(theta_o)),
/// ```
/// where `w_i` and `w_o` are the incoming and outgoing directions, `D` the
/// microfacet distribution, `G` the geometric factor and `F` the fresnel term.
/// In this case the microfacet distribution that has been used is the Beckmann
/// distribution and the geometric factor
/// ```text
///     G = min(2.0 * cos(theta_h) * cos(theta_o) / dot(H, w_o), 2.0 * cos(theta_h) * cos(theta_i) / dot(H, w_o), 1).
/// ```
/// The Fresnel term is calculated using the Schlick approximation.
#[derive(Debug, Clone, Copy)]
pub struct CookTorrance {
    /// Roughness
    alpha2: f64,
    fresnel: Fresnel,
    color: Vec3,
}

/// Adapted Cook-Torrance model for rough refraction.
///
/// The material model uses the microfacet normal to calculate specular
/// refraction rather than reflection. The procedure is mostly the same, but the
/// normalizing factors differ.
///
/// The struct contains an inner CookTorrance model, but forces a dieletric
/// Fresnel factor since the refraction code only works for dieletrics.
#[derive(Debug, Clone, Copy)]
pub struct CookTorranceRefract {
    ior: f64,
    cook_torrance: CookTorrance,
}

/// Adapted Cook-Torrance model for rough glass.
///
/// The material is just a mix material between a Cook-Torrance dieletric
/// reflective and refractive material. It is more efficient that a mix material
/// since it does importance sampling on the Fresnel coefficient. It is also
/// safer since it ensures the mixing IOR and material IOR are the same.
#[derive(Debug, Clone, Copy)]
pub struct CookTorranceGlass {
    ior: f64,
    cook_torrance: CookTorrance,
}

/// A material model that should mimic plastic.
///
/// The model is just a mix material between a diffuse and Cook-Torrance BRDF.
/// The material does importance sampling of the Fresnel coefficient to be more
/// efficient.
#[derive(Debug, Clone, Copy)]
pub struct Plastic {
    ior: f64,
    cook_torrance: CookTorrance,
    diffuse: LambertianDiffuse,
}

/// Reflection type of the material.
#[derive(Debug, Clone, Copy)]
pub enum ReflectType {
    Reflect,
    /// The Refract variant stores the index of refraction.
    Refract(f64),
}

/// A probability density function over the hemi-sphere.
pub enum Pdf<'a> {
    Cosine,
    Uniform,
    /// The Beckmann distribution is a distribution over halfway vectors.
    Beckmann(f64, ReflectType),
    Dirac(ReflectType),
    Hittable(&'a dyn Hittable),
    Mix(MixKind, Box<Pdf<'a>>, Box<Pdf<'a>>),
}

impl Bsdf for LambertianDiffuse {
    fn scatter(
        &self,
        position: Vec3,
        normal: Unit<Vec3>,
        view: Unit<Vec3>,
        pdf: Option<Pdf>,
    ) -> ScatteringEvent {
        let pdf = if let Some(pdf) = pdf {
            Pdf::Mix(MixKind::Constant(0.5), Box::new(pdf), Box::new(self.pdf()))
        } else {
            self.pdf()
        };
        let light = pdf.generate(position, normal, view);
        let color = self.brdf(position, normal, light, view) * normal.dot(light)
            / pdf.value(position, normal, light, view);

        ScatteringEvent::Scatter {
            color,
            ray: Ray::new(position, light.as_vec()),
        }
    }
}

impl Bsdf for Reflect {
    fn scatter(
        &self,
        position: Vec3,
        normal: Unit<Vec3>,
        view: Unit<Vec3>,
        _pdf: Option<Pdf>,
    ) -> ScatteringEvent {
        let pdf = self.pdf();
        let light = reflect(normal, view);
        let color = self.brdf(position, normal, light, view) * normal.dot(light)
            / pdf.value(position, normal, light, view);

        ScatteringEvent::Scatter {
            color,
            ray: Ray::new(position, light.as_vec()),
        }
    }
}

impl Bsdf for Refract {
    fn scatter(
        &self,
        position: Vec3,
        normal: Unit<Vec3>,
        view: Unit<Vec3>,
        _pdf: Option<Pdf>,
    ) -> ScatteringEvent {
        let pdf = self.pdf();

        let cos_theta = normal.dot(view);

        let direction = if cos_theta > 0. {
            ScatteringDirection::Entering
        } else {
            ScatteringDirection::Exiting
        };

        let normal = direction.normal(normal);

        let light = match refract(normal, view, direction.ior_ratio(self.ior)) {
            Some(l) => l,
            None => return ScatteringEvent::NoScatter,
        };

        let color = self.btdf(position, normal, light, view, direction, 1.)
            * normal.dot(light).abs()
            / pdf.value(position, normal, light, view);

        ScatteringEvent::Scatter {
            color,
            ray: Ray::new(position, light.as_vec()),
        }
    }
}

impl Bsdf for Glass {
    fn scatter(
        &self,
        position: Vec3,
        normal: Unit<Vec3>,
        view: Unit<Vec3>,
        _pdf: Option<Pdf>,
    ) -> ScatteringEvent {
        // The glass material will mix between the refract and reflection
        // based on the fresnel coefficient.
        let cos_theta = normal.dot(view);

        // Finding the direction lets us use the correct order of the IORs
        let direction = if cos_theta > 0. {
            ScatteringDirection::Entering
        } else {
            ScatteringDirection::Exiting
        };

        let normal = direction.normal(normal);

        let sin2theta = 1. - cos_theta * cos_theta;
        let ior_ratio = direction.ior_ratio(self.ior);

        if ior_ratio * ior_ratio * sin2theta >= 1. {
            // Total internal reflection
            let light = reflect(normal, view);
            let color = self.reflect.brdf(position, normal, light, view) * normal.dot(light);

            ScatteringEvent::Scatter {
                color,
                ray: Ray::new(position, light.as_vec()),
            }
        } else {
            let (ior_curr, ior_new) = direction.iors(self.ior);
            let fresnel = schlick_scalar(ior_curr, ior_new, normal, view);

            // Importance sample the Fresnel coefficient to choose between
            // reflection or refraction. The Fresnel coefficient cancels due to
            // the importance sampling.
            if rand::random::<f64>() < fresnel {
                let light = reflect(normal, view);
                let color = self.reflect.brdf(position, normal, light, view) * normal.dot(light);

                ScatteringEvent::Scatter {
                    color, // * fresnel / fresnel
                    ray: Ray::new(position, light.as_vec()),
                }
            } else {
                let light = refract(normal, view, ior_ratio).unwrap();
                let color = self
                    .refract
                    .btdf(position, normal, light, view, direction, 1.)
                    * normal.dot(light).abs();

                ScatteringEvent::Scatter {
                    color, // * (1 - fresnel) / (1 - fresnel)
                    ray: Ray::new(position, light.as_vec()),
                }
            }
        }
    }
}

impl Bsdf for CookTorrance {
    fn scatter(
        &self,
        position: Vec3,
        normal: Unit<Vec3>,
        view: Unit<Vec3>,
        _pdf: Option<Pdf>,
    ) -> ScatteringEvent {
        let pdf = Brdf::pdf(self);
        let halfway_vec = pdf.generate(position, normal, view);
        let light = reflect(halfway_vec, view);

        self.evaluate_reflection(
            position,
            normal,
            halfway_vec,
            view,
            light,
            pdf.value(position, normal, light, view),
        )
    }
}

impl Bsdf for CookTorranceRefract {
    fn scatter(
        &self,
        position: Vec3,
        normal: Unit<Vec3>,
        view: Unit<Vec3>,
        _pdf: Option<Pdf>,
    ) -> ScatteringEvent {
        // Finding the direction lets us choose the correct order of the IORs,
        // flip normals in the correct direction, etc..
        let direction = if normal.dot(view) > 0. {
            ScatteringDirection::Entering
        } else {
            ScatteringDirection::Exiting
        };
        let normal = direction.normal(normal);

        let ior_ratio = direction.ior_ratio(self.ior);

        let pdf = MicrofacetDistribution::Beckmann(self.cook_torrance.alpha2);
        let pdf_res = pdf.generate(normal);

        let halfway_vec = pdf_res.vector;
        let halfway_vec = direction.normal(halfway_vec);

        let light = match refract(halfway_vec, view, ior_ratio) {
            Some(l) => l,
            None => return ScatteringEvent::NoScatter,
        };

        self.cook_torrance.evaluate_refraction(
            position,
            normal,
            halfway_vec,
            view,
            light,
            pdf_res.value,
            direction,
            ior_ratio,
        )
    }
}

impl Bsdf for CookTorranceGlass {
    fn scatter(
        &self,
        position: Vec3,
        normal: Unit<Vec3>,
        view: Unit<Vec3>,
        _pdf: Option<Pdf>,
    ) -> ScatteringEvent {
        let pdf = MicrofacetDistribution::Beckmann(self.cook_torrance.alpha2);
        let pdf_res = pdf.generate(normal);

        let halfway_vec = pdf_res.vector;

        // Finding the direction lets us choose the correct order of the IORs,
        // flip normals in the correct direction, etc..
        let direction = if normal.dot(view) > 0. {
            ScatteringDirection::Entering
        } else {
            ScatteringDirection::Exiting
        };

        // Flip the halfway vector and normal to be in the same hemisphere as
        // the view vector.
        let halfway_vec = direction.normal(halfway_vec);

        let normal = direction.normal(normal);

        let cos_theta = halfway_vec.dot(view);

        let ior_ratio = direction.ior_ratio(self.ior);

        let sin2_theta = 1. - cos_theta * cos_theta;

        if ior_ratio * ior_ratio * sin2_theta >= 1. {
            // Total internal reflection
            let light = reflect(halfway_vec, view);

            self.cook_torrance.evaluate_reflection(
                position,
                normal,
                halfway_vec,
                view,
                light,
                pdf_res.value,
            )
        } else {
            let (ior_curr, ior_new) = direction.iors(self.ior);

            // The CookTorranceGlass model creates a CookTorrance with
            // dielectric Fresnel model so schlick_scalar will do the right
            // thing.
            let fresnel = schlick_scalar(ior_curr, ior_new, halfway_vec, view);

            // Importance sample the fresnel coefficient. Since the
            // Cook-Torrance model already contains the Fresnel coefficients
            // we need to divide due to the importance sampling.
            if rand::random::<f64>() < fresnel {
                let light = reflect(halfway_vec, view);

                match self.cook_torrance.evaluate_reflection(
                    position,
                    normal,
                    halfway_vec,
                    view,
                    light,
                    pdf_res.value,
                ) {
                    ScatteringEvent::Scatter { color, ray } => ScatteringEvent::Scatter {
                        color: color / fresnel,
                        ray,
                    },
                    ScatteringEvent::NoScatter => ScatteringEvent::NoScatter,
                }
            } else {
                let light = refract(halfway_vec, view, ior_ratio)
                    .expect(&format!("{}", sin2_theta * ior_ratio * ior_ratio));

                match self.cook_torrance.evaluate_refraction(
                    position,
                    normal,
                    halfway_vec,
                    view,
                    light,
                    pdf_res.value,
                    direction,
                    ior_ratio,
                ) {
                    ScatteringEvent::Scatter { color, ray } => ScatteringEvent::Scatter {
                        color: color / (1. - fresnel),
                        ray,
                    },
                    ScatteringEvent::NoScatter => ScatteringEvent::NoScatter,
                }
            }
        }
    }
}

impl Bsdf for Plastic {
    fn scatter(
        &self,
        position: Vec3,
        normal: Unit<Vec3>,
        view: Unit<Vec3>,
        pdf: Option<Pdf>,
    ) -> ScatteringEvent {
        let fresnel = schlick_scalar(1., self.ior, normal, view);

        // Importance sample the Fresnel coefficient to choose between
        // reflection or diffuse.
        if rand::random::<f64>() < fresnel {
            match self.cook_torrance.scatter(position, normal, view, pdf) {
                ScatteringEvent::Scatter { color, ray } => ScatteringEvent::Scatter {
                    color: color / fresnel,
                    ray,
                },
                ScatteringEvent::NoScatter => ScatteringEvent::NoScatter,
            }
        } else {
            self.diffuse.scatter(position, normal, view, pdf) //
                                                              // * (1 - fresnel)
                                                              //   / (1 - fresnel)
        }
    }
}

impl LambertianDiffuse {
    /// Constructs a new LambertianDiffuse
    ///
    /// # Panics
    /// If the color is outside of the range 0 to 1.
    ///
    /// # Examples
    /// ```
    /// use rayrs_lib::material::LambertianDiffuse;
    /// use rayrs_lib::vecmath::Vec3;
    ///
    /// let diffuse = LambertianDiffuse::new(Vec3::ones());
    /// ```
    pub fn new(color: Vec3) -> LambertianDiffuse {
        assert!(color.xyz_in_range_inclusive(0., 1.));

        LambertianDiffuse { color }
    }
}

impl Reflect {
    /// Constructs a new Reflect
    ///
    /// # Panics
    /// If the color is outside of the range 0 to 1.
    ///
    /// # Examples
    ///
    /// ```
    /// use rayrs_lib::material::Reflect;
    /// use rayrs_lib::vecmath::Vec3;
    ///
    /// let reflect = Reflect::new(Vec3::ones());
    /// ```
    pub fn new(color: Vec3) -> Reflect {
        assert!(color.xyz_in_range_inclusive(0., 1.));

        Reflect { color }
    }
}

impl Refract {
    /// Constructs a new Refract
    ///
    /// # Panics
    /// If the color is outside of the range 0 to 1 or if `ior <= 0.` or is not
    /// finite
    ///
    /// # Examples
    /// ```
    /// use rayrs_lib::material::Refract;
    /// use rayrs_lib::vecmath::Vec3;
    ///
    /// let refract = Refract::new(Vec3::ones(), 1.45);
    /// ```
    pub fn new(color: Vec3, ior: f64) -> Refract {
        assert!(color.xyz_in_range_inclusive(0., 1.));
        assert!(ior > 0.);
        assert!(ior.is_finite());

        Refract { color, ior }
    }
}

impl Glass {
    /// Constructs a new Glass
    ///
    /// # Panics
    /// If the color is outside of the range 0 to 1 or if `ior <= 0.` or is
    /// not finite.
    ///
    /// # Examples
    /// ```
    /// use rayrs_lib::material::Glass;
    /// use rayrs_lib::vecmath::Vec3;
    ///
    /// let glass = Glass::new(Vec3::ones(), 1.45);
    /// ```
    pub fn new(color: Vec3, ior: f64) -> Glass {
        assert!(color.xyz_in_range_inclusive(0., 1.));
        assert!(ior > 0.);
        assert!(ior.is_finite());

        Glass {
            ior,
            reflect: Reflect::new(color),
            refract: Refract::new(color, ior),
        }
    }
}

impl CookTorrance {
    /// Constructs a new CookTorrance.
    ///
    /// The `color` should in general be white. The color of metals is
    /// controlled by the Fresnel term.
    ///
    /// # Panics
    /// If color is outside of the range 0 to 1 or if `alpha <= 0.` or is not
    /// finite
    ///
    /// # Examples
    /// ```
    /// use rayrs_lib::material::CookTorrance;
    /// use rayrs_lib::material::Fresnel;
    /// use rayrs_lib::vecmath::Vec3;
    ///
    /// let cook_torrance =
    ///     CookTorrance::new(Vec3::ones(), 0.05, Fresnel::SchlickMetallic(Vec3::ones()));
    /// ```
    pub fn new(color: Vec3, alpha: f64, fresnel: Fresnel) -> CookTorrance {
        assert!(color.xyz_in_range_inclusive(0., 1.));
        assert!(alpha > 0.);
        assert!(alpha.is_finite());

        CookTorrance {
            alpha2: alpha * alpha,
            color,
            fresnel,
        }
    }

    /// Evaluate the BRDF Cook-Torrance model.
    ///
    /// `pdf` should be the value for the generated the halfway vector. Used
    /// for weighting the BRDF contribution properly.
    fn evaluate_reflection(
        &self,
        position: Vec3,
        normal: Unit<Vec3>,
        halfway_vec: Unit<Vec3>,
        view: Unit<Vec3>,
        light: Unit<Vec3>,
        pdf: f64,
    ) -> ScatteringEvent {
        if halfway_vec.dot(view) < 0. {
            return ScatteringEvent::NoScatter;
        }

        let nl = normal.dot(light);

        // If the light vector points inside the object we terminate
        if nl < 0.0 {
            return ScatteringEvent::NoScatter;
        }

        // Conversion factor for going from a distribution over halfway vectors
        // to a distribution over light vectors.
        let frac_dwh_dwi = 4.0 * halfway_vec.dot(light);

        let color = self.brdf(position, normal, light, view) * nl;
        let color = color / pdf * frac_dwh_dwi;

        // BRDF returns zero if we encountered any bad cases. Such as glancing
        // angles.
        if color.is_zeros() {
            ScatteringEvent::NoScatter
        } else {
            ScatteringEvent::Scatter {
                color,
                ray: Ray::new(position, light.as_vec()),
            }
        }
    }

    /// Evaluate the BTDF Cook-Torrance model.
    ///
    /// `pdf` should be the value for the generated the halfway vector. Used
    /// for weighting the BTDF contribution properly.
    fn evaluate_refraction(
        &self,
        position: Vec3,
        normal: Unit<Vec3>,
        halfway_vec: Unit<Vec3>,
        view: Unit<Vec3>,
        light: Unit<Vec3>,
        pdf: f64,
        direction: ScatteringDirection,
        ior_ratio: f64,
    ) -> ScatteringEvent {
        if halfway_vec.dot(view) < 0. {
            return ScatteringEvent::NoScatter;
        }

        let nl = normal.dot(light);

        // If the light vector points inside the object we terminate
        if nl > 0.0 {
            return ScatteringEvent::NoScatter;
        }

        let hl = halfway_vec.dot(light).abs();
        let hv = halfway_vec.dot(view).abs();

        // Conversion factor for going from a distribution over halfway vectors
        // to a distribution over light vectors.
        let denom = ior_ratio * hv + hl;
        let denom = denom * denom;
        let dwh_dwi = hl / denom;

        let color = self.btdf(position, normal, light, view, direction, 1.) * nl.abs()
        // TODO: Figure out if ior_ratio^2 should really be here..
        // The reasoning is that we are transfering importance so we need the
        // eta^2 normalization factor to get the adjoint BTDF.
            / (ior_ratio * ior_ratio);
        let color = color / (pdf * dwh_dwi);

        // BTDF returns zero if we encountered any bad cases. Such as glancing
        // angles.
        if color.is_zeros() {
            ScatteringEvent::NoScatter
        } else {
            ScatteringEvent::Scatter {
                color,
                ray: Ray::new(position, light.as_vec()),
            }
        }
    }
}

impl CookTorranceRefract {
    /// Constructs a new CookTorranceRefract.
    ///
    /// The index of refraction for the underlying Cook-Torrance model is
    /// automatically chosen to be dielectric.
    ///
    /// # Panics
    /// If color is outside of the range 0 to 1, if `alpha <= 0.` or is not
    /// finite or if `ior <= 0.` or is not finite.
    ///
    /// # Examples
    /// ```
    /// use rayrs_lib::material::CookTorranceRefract;
    /// use rayrs_lib::vecmath::Vec3;
    ///
    /// let ct_refract = CookTorranceRefract::new(Vec3::ones(), 0.05, 1.45);
    /// ```
    pub fn new(color: Vec3, alpha: f64, ior: f64) -> CookTorranceRefract {
        assert!(color.xyz_in_range_inclusive(0., 1.));
        assert!(alpha > 0.);
        assert!(alpha.is_finite());
        assert!(ior > 0.);
        assert!(ior.is_finite());

        CookTorranceRefract {
            ior,
            cook_torrance: CookTorrance::new(color, alpha, Fresnel::SchlickDielectric(ior)),
        }
    }
}

impl CookTorranceGlass {
    /// Constructs a new CookTorranceGlass.
    ///
    /// The index of refraction for the underlying Cook-Torrance model is
    /// automatically chosen to be dielectric.
    ///
    /// # Panics
    /// If color is outside of the range 0 to 1, if `alpha <= 0.` or is not
    /// finite or if `ior <= 0.` or is not finite.
    ///
    /// # Examples
    /// ```
    /// use rayrs_lib::material::CookTorranceGlass;
    /// use rayrs_lib::vecmath::Vec3;
    ///
    /// let ct_glass = CookTorranceGlass::new(Vec3::ones(), 0.05, 1.45);
    /// ```
    pub fn new(color: Vec3, alpha: f64, ior: f64) -> CookTorranceGlass {
        assert!(color.xyz_in_range_inclusive(0., 1.));
        assert!(alpha > 0.);
        assert!(alpha.is_finite());
        assert!(ior > 0.);
        assert!(ior.is_finite());

        CookTorranceGlass {
            ior,
            cook_torrance: CookTorrance::new(color, alpha, Fresnel::SchlickDielectric(ior)),
        }
    }
}

impl Plastic {
    /// Constructs a new Plastic.
    ///
    /// # Examples
    /// ```
    /// use rayrs_lib::material::Plastic;
    /// use rayrs_lib::vecmath::Vec3;
    ///
    /// let plastic = Plastic::new(Vec3::ones() * 0.8, Vec3::ones(), 0.05, 1.45);
    /// ```
    pub fn new(color: Vec3, spec_color: Vec3, alpha: f64, ior: f64) -> Plastic {
        assert!(color.xyz_in_range_inclusive(0., 1.));
        assert!(spec_color.xyz_in_range_inclusive(0., 1.));
        assert!(alpha > 0.);
        assert!(alpha.is_finite());
        assert!(ior > 0.);
        assert!(ior.is_finite());

        Plastic {
            ior,
            cook_torrance: CookTorrance::new(spec_color, alpha, Fresnel::SchlickDielectric(ior)),
            diffuse: LambertianDiffuse::new(color),
        }
    }
}

impl<'a> Pdf<'a> {
    /// Returns the value of the distribution for the light, view vector pair.
    pub fn value(
        &self,
        position: Vec3,
        normal: Unit<Vec3>,
        light: Unit<Vec3>,
        view: Unit<Vec3>,
    ) -> f64 {
        match self {
            Pdf::Cosine => normal.dot(light) * FRAC_1_PI,
            Pdf::Uniform => FRAC_PI_2,
            Pdf::Beckmann(alpha2, typ) => {
                let halfway_vec = match typ {
                    ReflectType::Reflect => light + view,
                    ReflectType::Refract(ior_ratio) => light + *ior_ratio * view,
                };

                if halfway_vec.is_zeros() {
                    // This is a singularity for the pdf. In this case the
                    // BRDF should return black as the color.
                    return 1.;
                }

                let halfway_vec = halfway_vec.unit();
                let nh = normal.dot(halfway_vec).abs();
                // Assume the halfway vector and normal are always in the
                // same hemisphere and do the absolute value.
                let theta_h = nh.acos();

                let tan_theta_h = theta_h.tan();

                if tan_theta_h.is_infinite() {
                    // This is a singularity for the pdf. In this case the
                    // BRDF should return black as the color.
                    return 1.;
                }
                (-tan_theta_h * tan_theta_h / alpha2).exp() / (PI * alpha2 * nh.powi(4))
            }
            Pdf::Dirac(_) => 1.,
            Pdf::Hittable(geom) => {
                let ray = Ray::new(position, light.as_vec());
                if let Some(t) = geom.intersect(ray) {
                    (position - ray.point(t)).mag2() / (normal.dot(light) * geom.area())
                } else {
                    0.
                }
            }
            Pdf::Mix(kind, pdf1, pdf2) => {
                if normal.dot(light) < 0. {
                    return f64::INFINITY;
                }

                let factor = kind.value(position, normal, light);
                factor * pdf1.value(position, normal, light, view)
                    + (1. - factor) * pdf2.value(position, normal, light, view)
            }
        }
    }

    /// Generate a vector from this distribution.
    ///
    /// The returned vector is generated from the position, normal and possibly
    /// the view vector. The interpretation of the vector depends on the PDF.
    /// For example the Dirac distribution can return a transmitted or reflected
    /// vector and the Beckmann distribution returns a halfway vector.
    ///
    /// # Examples
    ///
    /// ```
    /// use rayrs_lib::material::{Pdf, ReflectType};
    /// use rayrs_lib::vecmath::{Vec3, VecUnit};
    ///
    /// let pdf = Pdf::Beckmann(0.05, ReflectType::Reflect);
    ///
    /// let halfway_vec = pdf.generate(Vec3::zeros(), Vec3::unit_z(), Vec3::unit_z());
    /// ```
    pub fn generate(&self, position: Vec3, normal: Unit<Vec3>, view: Unit<Vec3>) -> Unit<Vec3> {
        match self {
            Pdf::Cosine => {
                let (e1, e2) = Vec3::orthonormal_basis(normal);

                let u: f64 = rand::random();
                let phi: f64 = 2. * PI * rand::random::<f64>();

                let x = phi.cos() * u.sqrt();
                let y = phi.sin() * u.sqrt();
                let z = (1. - u).sqrt();

                Unit::new_unchecked(x * e1 + y * e2 + z * normal)
            }
            Pdf::Uniform => {
                let (e1, e2) = Vec3::orthonormal_basis(normal);

                let u: f64 = rand::random();
                let phi: f64 = 2. * PI * rand::random::<f64>();

                let x = phi.cos() * (u * (1. - u)).sqrt();
                let y = phi.sin() * (u * (1. - u)).sqrt();
                let z = 1. - u;

                Unit::new_unchecked(x * e1 + y * e2 + z * normal)
            }
            Pdf::Beckmann(alpha2, _) => {
                let (e1, e2) = Vec3::orthonormal_basis(normal);

                let phi = 2. * PI * rand::random::<f64>();

                let tan2theta = -alpha2 * (1. - rand::random::<f64>()).ln();
                let costheta = 1.0 / (1.0 + tan2theta).sqrt();
                let sintheta = (1.0 - costheta * costheta).sqrt();

                let x = phi.cos() * sintheta;
                let y = phi.sin() * sintheta;

                let halfway_vec = x * e1 + y * e2 + costheta * normal;
                Unit::new_unchecked(halfway_vec)
            }
            Pdf::Dirac(typ) => match typ {
                ReflectType::Reflect => reflect(normal, view),
                ReflectType::Refract(ior) => {
                    refract(normal, view, *ior).unwrap_or_else(|| reflect(normal, view))
                }
            },
            Pdf::Hittable(geom) => (geom.sample() - position).unit(),
            Pdf::Mix(kind, pdf1, pdf2) => {
                if rand::random::<f64>() < kind.value(position, normal, view) {
                    pdf1.generate(position, normal, view)
                } else {
                    pdf2.generate(position, normal, view)
                }
            }
        }
    }

    /// Returns true if the distrbution contains a Dirac delta function.
    pub fn is_dirac(&self) -> bool {
        match self {
            Pdf::Dirac(_) => true,
            Pdf::Mix(_, mat1, mat2) => mat1.is_dirac() || mat2.is_dirac(),
            _ => false,
        }
    }
}

/// Lets an object emit light in the scene.
pub trait Emitting {
    fn emit(&self) -> Vec3;
}

/// Emission types.
///
/// The object can either emit light of a certain color or be dark.
#[derive(Debug, Clone)]
pub enum Emission {
    Emissive(f64, Vec3),
    Dark,
}

impl Emission {
    /// Create a new `Emission::Emissive`.
    ///
    /// # Panics
    /// If color is not in the range 0 to 1 or if `strength < 0.`.
    pub fn new(strength: f64, color: Vec3) -> Emission {
        assert!(strength >= 0.);
        assert!(
            color.xyz_in_range_inclusive(0., 1.),
            "RGB values need to be between 0 and 1"
        );
        Emission::Emissive(strength, color)
    }
}

impl Emitting for Emission {
    fn emit(&self) -> Vec3 {
        match self {
            Emission::Emissive(strength, color) => *strength * *color,
            Emission::Dark => Vec3::new(0., 0., 0.),
        }
    }
}

/// A bidirectional reflectance distribution function.
///
/// The BRDF is the simplest material model. Describing reflection of the light
/// in a single point.
trait Brdf {
    /// Evaluate the BRDF in the point for the given directions.
    fn brdf(&self, position: Vec3, normal: Unit<Vec3>, light: Unit<Vec3>, view: Unit<Vec3>)
        -> Vec3;
    /// Get the associated PDF for importance sampling.
    fn pdf(&self) -> Pdf; // TODO: Remove pdf from BRDF interface.
    /// Returns true if the BRDF contains a Dirac delta function.
    fn is_dirac(&self) -> bool; // TODO: Remove is_dirac from BRDF interface.
}

/// A bidirectional transmittance distribution function.
///
/// The BTDF describes the transmission of light at the incident point.
trait Btdf {
    /// Evaluate the BTDF in the point for the given view and light vector.
    ///
    /// The `directon` parameter is used to determine the order of IORs and the
    /// `ior` parameter is the IOR of the other participating medium.
    fn btdf(
        &self,
        position: Vec3,
        normal: Unit<Vec3>,
        light: Unit<Vec3>,
        view: Unit<Vec3>,
        direction: ScatteringDirection,
        ior: f64,
    ) -> Vec3;
    /// Get the associated PDF for importance sampling.
    fn pdf(&self) -> Pdf; // TODO: Remove pdf from BTDF interface
    /// Returns true if the BTDF contains a Dirac delta function.
    fn is_dirac(&self) -> bool; // TODO: Remove is_dirac from BTDF interface.
}

/// A generated PDF vector and value.
#[derive(Debug)]
struct PdfValue<T> {
    vector: T,
    value: f64,
}

/// A microfacet distribution.
#[derive(Debug)]
enum MicrofacetDistribution {
    /// The Beckmann distribution stores the alpha2 value.
    Beckmann(f64),
}

impl MicrofacetDistribution {
    /// Generate a vector and value from the distribution.
    fn generate(self, normal: Unit<Vec3>) -> PdfValue<Unit<Vec3>> {
        match self {
            Self::Beckmann(alpha2) => {
                let (e1, e2) = Vec3::orthonormal_basis(normal);

                let phi = 2. * PI * rand::random::<f64>();

                let tan2theta = -alpha2 * (1. - rand::random::<f64>()).ln();
                let costheta = 1.0 / (1.0 + tan2theta).sqrt();
                let sintheta = (1.0 - costheta * costheta).sqrt();

                let x = phi.cos() * sintheta;
                let y = phi.sin() * sintheta;

                let halfway_vec = x * e1 + y * e2 + costheta * normal;
                let nh = normal.dot(halfway_vec);
                PdfValue {
                    vector: Unit::new_unchecked(halfway_vec),
                    value: (-tan2theta / alpha2).exp() / (PI * alpha2 * nh.powi(4)),
                }
            }
        }
    }

    /// Get the value of the distribution.
    ///
    /// # Note
    /// This function can return 0.
    #[allow(dead_code)]
    fn value(&self, normal: Unit<Vec3>, halfway_vec: Unit<Vec3>) -> f64 {
        match self {
            Self::Beckmann(alpha2) => {
                let nh = normal.dot(halfway_vec);
                assert!(nh > 0.);

                let theta_h = nh.acos();

                let tan_theta_h = theta_h.tan();

                if tan_theta_h.is_infinite() {
                    // This is a singularity for the pdf. In this case the
                    // BRDF should return black as the color.
                    return 0.;
                }
                (-tan_theta_h * tan_theta_h / alpha2).exp() / (PI * alpha2 * nh.powi(4))
            }
        }
    }
}

/// Scattering direction for a transmission event.
#[derive(Debug, Clone, Copy)]
enum ScatteringDirection {
    /// Ray entering the object.
    Entering,
    /// Ray exiting the object.
    Exiting,
}

impl ScatteringDirection {
    /// Calculate the ior ratio of the object ior and the surrounding air.
    fn ior_ratio(&self, ior: f64) -> f64 {
        match self {
            Self::Entering => 1. / ior,
            Self::Exiting => ior,
        }
    }

    /// Flip the normal in the correct direction.
    ///
    /// Will flip the normal if it is not in the same direction as the view
    /// vector.
    ///
    /// # Note
    /// Calling this function more than once with the same normal will still
    /// flip it again.
    fn normal(&self, normal: Unit<Vec3>) -> Unit<Vec3> {
        match self {
            Self::Entering => normal,
            Self::Exiting => Unit::new_unchecked(-1. * normal),
        }
    }

    /// Return the IORs in the order IOR current, IOR new.
    ///
    /// The current default is to choose between the IOR of the object or air.
    fn iors(&self, ior: f64) -> (f64, f64) {
        match self {
            Self::Entering => (1., ior),
            Self::Exiting => (ior, 1.),
        }
    }
}

impl Brdf for LambertianDiffuse {
    fn brdf(
        &self,
        _position: Vec3,
        _normal: Unit<Vec3>,
        _light: Unit<Vec3>,
        _view: Unit<Vec3>,
    ) -> Vec3 {
        // The 1-over-PI factor is a normalizing factor for energy conservation.
        self.color * FRAC_1_PI
    }

    fn pdf(&self) -> Pdf {
        Pdf::Cosine
    }

    fn is_dirac(&self) -> bool {
        false
    }
}

impl Brdf for Reflect {
    fn brdf(
        &self,
        _position: Vec3,
        normal: Unit<Vec3>,
        light: Unit<Vec3>,
        _view: Unit<Vec3>,
    ) -> Vec3 {
        // Divide by the cos(theta) term to cancel with the projection term in
        // the integral.
        self.color / normal.dot(light).abs()
    }

    fn pdf(&self) -> Pdf {
        Pdf::Dirac(ReflectType::Reflect)
    }

    fn is_dirac(&self) -> bool {
        true
    }
}

impl Brdf for CookTorrance {
    fn brdf(
        &self,
        _position: Vec3,
        normal: Unit<Vec3>,
        light: Unit<Vec3>,
        view: Unit<Vec3>,
    ) -> Vec3 {
        let nv = normal.dot(view).abs();
        let nl = normal.dot(light).abs();

        let halfway_vec = view + light;

        // At glancing angles we have a singularity so we return the analytic
        // limit value which is 0.
        if nv == 0.0 || nl == 0.0 {
            return Vec3::zeros();
        }
        if halfway_vec.is_zeros() {
            return Vec3::zeros();
        }

        let halfway_vec = halfway_vec.unit();
        let nh = normal.dot(halfway_vec);

        let theta_h = nh.acos();

        let tan_theta_h = theta_h.tan();

        // Another singularity.
        if tan_theta_h.is_infinite() {
            return Vec3::zeros();
        }

        let beckmann =
            (-tan_theta_h * tan_theta_h / (self.alpha2)).exp() / (PI * self.alpha2 * nh.powi(4));
        let hv = halfway_vec.dot(view);
        let geometric_factor = (2.0 * nh * nv / hv).min((2.0 * nh * nl / hv).min(1.));

        self.color
            * self
                .fresnel
                .value(halfway_vec, view, ScatteringDirection::Entering)
            * beckmann
            * geometric_factor
            / (4.0 * nv * nl)
    }

    fn pdf(&self) -> Pdf {
        Pdf::Beckmann(self.alpha2, ReflectType::Reflect)
    }

    fn is_dirac(&self) -> bool {
        false
    }
}

impl Btdf for Refract {
    fn btdf(
        &self,
        _position: Vec3,
        normal: Unit<Vec3>,
        light: Unit<Vec3>,
        view: Unit<Vec3>,
        _direction: ScatteringDirection,
        _ior: f64,
    ) -> Vec3 {
        if light.dot(view) > 0. {
            // In case of total internal reflection we return black.
            Vec3::zeros()
        } else {
            // Divide out the cosine term that will be multiplied in the
            // integral.
            self.color / normal.dot(light).abs()
        }
    }

    fn pdf(&self) -> Pdf {
        Pdf::Dirac(ReflectType::Refract(self.ior))
    }

    fn is_dirac(&self) -> bool {
        true
    }
}

impl Btdf for CookTorrance {
    fn btdf(
        &self,
        _position: Vec3,
        normal: Unit<Vec3>,
        light: Unit<Vec3>,
        view: Unit<Vec3>,
        direction: ScatteringDirection,
        _ior: f64,
    ) -> Vec3 {
        let nv = normal.dot(view).abs();
        let nl = normal.dot(light).abs();

        let ior_self = match self.fresnel {
            Fresnel::SchlickDielectric(ior) => ior,
            Fresnel::SchlickMetallic(_) => panic!("A metallic CookTorrance cannot transmit"),
        };

        let ior_ratio = direction.ior_ratio(ior_self);

        // The direction of the halfway vector depends on the magnitude of
        // ior_ratio. If it is larger than 1 it will point in the same direction
        // as the view vector, otherwise it will point in the opposite direction
        // so we flip it.
        let halfway_vec = if ior_ratio > 1. {
            light + ior_ratio * view
        } else {
            (-ior_ratio) * view - light
        };

        if nv == 0.0 || nl == 0.0 {
            return Vec3::zeros();
        }

        if halfway_vec.is_zeros() {
            return Vec3::zeros();
        }

        let halfway_vec = halfway_vec.unit();

        // Use absolute value to force halfway_vec and normal to be in the same
        // hemisphere.
        let nh = normal.dot(halfway_vec);

        let theta_h = nh.acos();

        let tan_theta_h = theta_h.tan();

        if tan_theta_h.is_infinite() {
            return Vec3::zeros();
        }

        let beckmann =
            (-tan_theta_h * tan_theta_h / self.alpha2).exp() / (PI * self.alpha2 * nh.powi(4));

        let hl = halfway_vec.dot(light).abs();
        let hv = halfway_vec.dot(view).abs();

        // TODO: Figure out if hv should be in denominator of both.
        let geometric_factor = (2.0 * nh * nv / hv).min((2.0 * nh * nl / hv).min(1.));

        let denom = ior_ratio * hv + hl;
        let denom = denom * denom;

        let norm_fac = hv * hl / (nv * nl);

        let fresnel = self.fresnel.value(halfway_vec, view, direction);

        debug_assert!(beckmann > 0.);
        debug_assert!(geometric_factor > 0.);
        debug_assert!(fresnel.xyz_in_range_inclusive(0., 1.), dbg!(fresnel));

        self.color
            * (Vec3::ones() - fresnel)
            * beckmann
            * geometric_factor
            * norm_fac
            * ior_ratio
            * ior_ratio
            / denom
    }

    fn pdf(&self) -> Pdf {
        let ior = match self.fresnel {
            Fresnel::SchlickDielectric(ior) => ior,
            Fresnel::SchlickMetallic(_) => panic!("A metallic CookTorrance cannot transmit"),
        };
        Pdf::Beckmann(self.alpha2, ReflectType::Refract(ior))
    }

    fn is_dirac(&self) -> bool {
        false
    }
}

impl Fresnel {
    /// Get the value of the Fresnel coefficient.
    fn value(&self, normal: Unit<Vec3>, view: Unit<Vec3>, direction: ScatteringDirection) -> Vec3 {
        match self {
            Fresnel::SchlickDielectric(ior_self) => {
                let (ior_curr, ior_new) = direction.iors(*ior_self);
                let fresnel = schlick_scalar(ior_curr, ior_new, normal, view);
                Vec3::new(fresnel, fresnel, fresnel)
            }
            Fresnel::SchlickMetallic(r0) => schlick_vec(*r0, normal, view),
        }
    }
}

/// Schlick approximation for scalar r0.
fn schlick_scalar(ior_curr: f64, ior_new: f64, normal: Unit<Vec3>, view: Unit<Vec3>) -> f64 {
    let normal = normal;
    let view = view;

    let r0 = (ior_curr - ior_new) / (ior_curr + ior_new);
    let r0 = r0 * r0;
    r0 + (1. - r0) * (1. - normal.dot(view)).powi(5)
}

/// Schlick approximation for vector r0.
///
/// Can be used to approximate a metallic index of refraction.
fn schlick_vec(r0: Vec3, normal: Unit<Vec3>, view: Unit<Vec3>) -> Vec3 {
    let normal = normal;
    let view = view;

    r0 + (Vec3::new(1., 1., 1.) - r0) * (1. - normal.dot(view)).powi(5)
}

/// Returns the view vector reflected in the normal.
fn reflect(normal: Unit<Vec3>, view: Unit<Vec3>) -> Unit<Vec3> {
    let view = view;
    let normal = normal;
    Unit::new_unchecked(2. * view.dot(normal) * normal - view)
}

/// Returns the view vector refracted by the material in the normal.
///
/// # Note
/// In case of total interal reflection this
fn refract(normal: Unit<Vec3>, view: Unit<Vec3>, ior_ratio: f64) -> Option<Unit<Vec3>> {
    let view = view;
    let normal = normal;

    let cos_theta = view.dot(normal);

    let sin_theta = (1. - cos_theta * cos_theta).sqrt();

    // Total internal reflection
    if ior_ratio * sin_theta > 1. {
        return None;
    }

    let dir_parallel = ior_ratio * (cos_theta * normal - view);
    let dir_perp = -(1. - dir_parallel.mag2()).sqrt() * normal;
    Some(Unit::new_unchecked(dir_perp + dir_parallel))
}

/// A mix-factor for the mix pdf and mix material.
#[derive(Debug, Copy, Clone)]
pub enum MixKind {
    Constant(f64),
    Fresnel(f64),
}

impl MixKind {
    /// Get the value of the mix factor.
    fn value(&self, _position: Vec3, normal: Unit<Vec3>, light: Unit<Vec3>) -> f64 {
        match self {
            MixKind::Constant(factor) => *factor,
            MixKind::Fresnel(ior) => schlick_scalar(1., *ior, normal, light),
        }
    }
}
