//! Material models
//!
//! This module contains material models and their scattering behavior. It also
//! contains other utilities needed to evaluate materials such as different
//! functions for calculating Fresnel coefficients and PDFs.

use super::geometry::Hittable;
use super::vecmath::{Dot, Unit, Vec3, VecElements, VecUnit};
use super::Ray;

use std::f64;
use std::f64::consts::{FRAC_1_PI, FRAC_PI_2, PI};

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
    Scatter { color: Vec3, ray: Ray },
    NoScatter,
}

/// A Fresnel coefficient.
#[derive(Debug, Clone, Copy)]
pub enum Fresnel {
    SchlickDielectric(f64),
    SchlickMetallic(Vec3),
}

/// A bidirectional scattering distribution function.
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
/// where `w_i` and `w_o` are the incoming and outgoing directions, `D` the microfacet
/// distribution, `G` the geometric factor and `F` the fresnel term. In this case
/// the microfacet distribution that has been used is the Beckmann distribution
/// and the geometric factor
/// ```text
///     G = min(2.0 * cos(theta_h) * cos(theta_o) / dot(H, w_o), 2.0 * cos(theta_h) * cos(theta_i) / dot(H, w_i), 1).
/// ```
/// The Fresnel term is calculated using the Schlick approximation.
#[derive(Debug, Clone, Copy)]
pub struct CookTorrance {
    /// Roughness
    alpha2: f64,
    fresnel: Fresnel,
    color: Vec3,
}

#[derive(Debug, Clone, Copy)]
pub struct CookTorranceRefract {
    ior: f64,
    cook_torrance: CookTorrance,
}

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

// #[derive(Debug, Clone, Copy)]
// pub struct CookTorranceRefract {
//     alpha2: f64,
//     ior: f64,
//     color: Vec3,
// }

// #[derive(Debug, Clone, Copy)]
// pub struct CookTorranceGlass {
//     alpha2: f64,
//     ior: f64,
//     refract: CookTorranceRefract,
//     reflect: CookTorranceDielectric,
// }

#[derive(Debug, Clone, Copy)]
pub enum ReflectType {
    Reflect,
    Refract(f64),
}

/// A probability density function over the hemi-sphere.
pub enum Pdf<'a> {
    Cosine,
    Uniform,
    /// The Beckmann distribution is a distribution of halfway vectors.
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
        pdf: Option<Pdf>,
    ) -> ScatteringEvent {
        // The glass material will mix between the refract and reflection
        // based on the fresnel coefficient.
        let cos_theta = normal.dot(view);

        let direction = if cos_theta > 0. {
            ScatteringDirection::Entering
        } else {
            ScatteringDirection::Exiting
        };

        let cos_theta = cos_theta.abs();

        let sin_theta = (1. - cos_theta * cos_theta).sqrt();
        let ior_ratio = direction.ior_ratio(self.ior);

        if ior_ratio * sin_theta > 1. {
            // Total internal reflection
            self.reflect.scatter(position, normal, view, pdf)
        } else {
            let (ior_curr, ior_new) = direction.iors(self.ior);
            let fresnel = schlick_scalar(ior_curr, ior_new, normal, view);

            // Importance sample the Fresnel coefficient to choose between
            // reflection or refraction.
            if rand::random::<f64>() < fresnel {
                self.reflect.scatter(position, normal, view, pdf) // * fresnel / fresnel
            } else {
                self.refract.scatter(position, normal, view, pdf) // * (1 - fresnel) / (1 - fresnel)
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

            let fresnel = schlick_scalar(ior_curr, ior_new, halfway_vec, view);

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
            self.diffuse.scatter(position, normal, view, pdf) // * (1 - fresnel) / (1 - fresnel)
        }
    }
}

impl LambertianDiffuse {
    /// Constructs a new LambertianDiffuse
    ///
    /// # Examples
    ///
    /// ```
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
    /// # Examples
    ///
    /// ```
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
    /// # Examples
    ///
    /// ```
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
    /// # Examples
    ///
    /// ```
    /// let glass = Glass:new(Vec3::ones(), 1.45);
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
    /// # Examples
    ///
    /// ```
    /// let cook_torrance = CookTorrance::new(
    ///     Vec3::ones(),
    ///     0.05,
    ///     Fresnel::SchlickMetallic(Vec3::ones())
    /// );
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

    /// Evaluate the BRDF Cook-Torrance model
    ///
    /// The `pdf` should be the PDF used to generate the halfway vector. Used
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

        if color.is_zeros() {
            ScatteringEvent::NoScatter
        } else {
            ScatteringEvent::Scatter {
                color,
                ray: Ray::new(position, light.as_vec()),
            }
        }
    }

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
    /// Constructs a new Plastic
    ///
    /// # Examples
    ///
    /// ```
    /// let plastic = Plastic::new(Vec3::ones(), 0.05, 1.45)
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
    pub fn new(strength: f64, color: Vec3) -> Emission {
        assert!(strength >= 0.);
        assert!(
            (0.0..1.0).contains(&color.x())
                && (0.0..1.0).contains(&color.y())
                && (0.0..1.0).contains(&color.z()),
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
/// The BRDF is the simplest material model. Reflecting the light in the
/// incident point.
trait Brdf {
    /// Evaluate the BRDF in the point for the given directions.
    fn brdf(&self, position: Vec3, normal: Unit<Vec3>, light: Unit<Vec3>, view: Unit<Vec3>)
        -> Vec3;
    /// Get the associated PDF for importance sampling.
    fn pdf(&self) -> Pdf;
    /// Returns true if the BRDF contains a Dirac delta function.
    fn is_dirac(&self) -> bool;
}

/// A bidirectional transmittance distribution function.
///
/// The BTDF describes the transmission of light at the incident point.
trait Btdf {
    fn btdf(
        &self,
        position: Vec3,
        normal: Unit<Vec3>,
        light: Unit<Vec3>,
        view: Unit<Vec3>,
        direction: ScatteringDirection,
        ior: f64,
    ) -> Vec3;
    fn pdf(&self) -> Pdf;
    fn is_dirac(&self) -> bool;
}

struct PdfValue<T> {
    vector: T,
    value: f64,
}

enum MicrofacetDistribution {
    Beckmann(f64),
}

impl MicrofacetDistribution {
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
    fn normal(&self, normal: Unit<Vec3>) -> Unit<Vec3> {
        match self {
            Self::Entering => normal,
            Self::Exiting => Unit::new_unchecked(-1. * normal),
        }
    }

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
            // In case of total internal reflection we don't return anything.
            Vec3::zeros()
        } else {
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

        let halfway_vec = light + ior_ratio * view;

        // The above procedure yields a vector proportional to the halfway
        // vector, but we might have to flip it in the correct direction.
        let halfway_vec = if halfway_vec.dot(view) > 0. {
            halfway_vec
        } else {
            -1. * halfway_vec
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

        assert!(beckmann > 0.);
        assert!(geometric_factor > 0.);
        assert!(fresnel.xyz_in_range_inclusive(0., 1.), dbg!(fresnel));

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
