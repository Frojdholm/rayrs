use super::geometry::Hittable;
use super::vecmath::Vec3;
use super::Ray;

use std::f64;
use std::f64::consts::{FRAC_1_PI, PI};

pub enum ReflectType {
    Reflect,
    Refract(f64),
}

pub enum Pdf<'a> {
    Cosine,
    Uniform,
    Beckmann(f64, ReflectType),
    Dirac(ReflectType),
    Hittable(&'a dyn Hittable),
    Mix(MixKind, Box<Pdf<'a>>, Box<Pdf<'a>>),
}

impl<'a> Pdf<'a> {
    pub fn value(&self, position: Vec3, normal: Vec3, light: Vec3, view: Vec3) -> f64 {
        match self {
            Pdf::Cosine => normal.dot(light.unit()) / PI,
            Pdf::Uniform => 0.5 * PI,
            Pdf::Beckmann(alpha2, typ) => {
                let halfway_vec = match typ {
                    ReflectType::Reflect => light + view,
                    ReflectType::Refract(ior) => {
                        let ior_ratio = if normal.dot(view) > 0. {
                            1.0 / ior
                        } else {
                            *ior
                        };
                        light + ior_ratio * view
                    }
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
                let ray = Ray::new(position, light);
                if let Some(t) = geom.intersect(ray) {
                    (position - ray.point(t)).mag_2() / (normal.dot(light) * geom.area())
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

    pub fn generate(&self, position: Vec3, normal: Vec3, view: Vec3) -> Vec3 {
        match self {
            Pdf::Cosine => {
                let (e1, e2) = Vec3::orthonormal_basis(normal);

                let u: f64 = rand::random();
                let phi: f64 = 2. * PI * rand::random::<f64>();

                let x = phi.cos() * u.sqrt();
                let y = phi.sin() * u.sqrt();
                let z = (1. - u).sqrt();

                x * e1 + y * e2 + z * normal
            }
            Pdf::Uniform => {
                let (e1, e2) = Vec3::orthonormal_basis(normal);

                let u: f64 = rand::random();
                let phi: f64 = 2. * PI * rand::random::<f64>();

                let x = phi.cos() * (u * (1. - u)).sqrt();
                let y = phi.sin() * (u * (1. - u)).sqrt();
                let z = 1. - u;

                x * e1 + y * e2 + z * normal
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
                halfway_vec
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

    pub fn is_dirac(&self) -> bool {
        match self {
            Pdf::Dirac(_) => true,
            Pdf::Mix(_, mat1, mat2) => mat1.is_dirac() || mat2.is_dirac(),
            _ => false,
        }
    }
}

pub trait Emitting {
    fn emit(&self) -> Vec3;
}

#[derive(Debug, Clone)]
pub enum Emission {
    Emissive(f64, Vec3),
    Dark,
}

impl Emission {
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

trait Brdf {
    fn brdf(&self, position: Vec3, normal: Vec3, light: Vec3, view: Vec3) -> Vec3;
    fn pdf(&self) -> Pdf;
    fn is_dirac(&self) -> bool;
}

#[derive(Debug, Clone, Copy)]
pub struct LambertianDiffuse {
    color: Vec3,
}

impl Brdf for LambertianDiffuse {
    fn brdf(&self, _position: Vec3, _normal: Vec3, _light: Vec3, _view: Vec3) -> Vec3 {
        self.color * FRAC_1_PI
    }

    fn pdf(&self) -> Pdf {
        Pdf::Cosine
    }

    fn is_dirac(&self) -> bool {
        false
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Mirror {
    color: Vec3,
}

impl Brdf for Mirror {
    fn brdf(&self, _position: Vec3, normal: Vec3, light: Vec3, _view: Vec3) -> Vec3 {
        self.color / normal.dot(light).abs()
    }

    fn pdf(&self) -> Pdf {
        Pdf::Dirac(ReflectType::Reflect)
    }

    fn is_dirac(&self) -> bool {
        true
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Refract {
    ior: f64,
    color: Vec3,
}

impl Brdf for Refract {
    fn brdf(&self, _position: Vec3, normal: Vec3, light: Vec3, view: Vec3) -> Vec3 {
        if light.dot(view) > 0. {
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

#[derive(Debug, Clone)]
pub struct Glass {
    ior: f64,
    refract: Box<Material>,
    reflect: Box<Material>,
}

#[derive(Debug, Clone, Copy)]
pub struct CookTorranceMetallic {
    alpha2: f64,
    color: Vec3,
}

impl Brdf for CookTorranceMetallic {
    fn brdf(&self, _position: Vec3, normal: Vec3, light: Vec3, view: Vec3) -> Vec3 {
        let nv = normal.dot(view).abs();
        let nl = normal.dot(light).abs();

        let halfway_vec = view + light;

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

        if tan_theta_h.is_infinite() {
            return Vec3::zeros();
        }
        let beckmann =
            (-tan_theta_h * tan_theta_h / (self.alpha2)).exp() / (PI * self.alpha2 * nh.powi(4));
        let hv = halfway_vec.dot(view);
        let geometric_factor = (2.0 * nh * nv / hv).min((2.0 * nh * nl / hv).min(1.));

        let fresnel = Fresnel::SchlickMetallic(self.color).value(halfway_vec, view);

        fresnel * beckmann * geometric_factor / (4.0 * nv * nl)
    }

    fn pdf(&self) -> Pdf {
        Pdf::Beckmann(self.alpha2, ReflectType::Reflect)
    }

    fn is_dirac(&self) -> bool {
        false
    }
}

#[derive(Debug, Clone, Copy)]
pub struct CookTorranceDielectric {
    alpha2: f64,
    ior: f64,
    color: Vec3,
}

impl CookTorranceDielectric {
    fn evaluate(
        &self,
        position: Vec3,
        normal: Vec3,
        halfway_vec: Vec3,
        view: Vec3,
        pdf: Pdf,
    ) -> (Vec3, Option<Ray>) {
        let light = reflect(halfway_vec, view);

        let nl = normal.dot(light);
        // If the light vector points inside the object we terminate
        if nl < 0.0 {
            return (Vec3::zeros(), None);
        }

        let frac_dwh_dwi = 4.0 * halfway_vec.dot(light);

        let color = self.brdf(position, normal, light, view) * nl;
        let pdf = pdf.value(position, normal, light, view);
        let color = color / pdf * frac_dwh_dwi;

        assert!(pdf != 0.);

        if color.is_zeros() {
            (color, None)
        } else {
            (color, Some(Ray::new(position, light)))
        }
    }
}

impl Brdf for CookTorranceDielectric {
    fn brdf(&self, _position: Vec3, normal: Vec3, light: Vec3, view: Vec3) -> Vec3 {
        let nv = normal.dot(view).abs();
        let nl = normal.dot(light).abs();

        let halfway_vec = view + light;

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

        if tan_theta_h.is_infinite() {
            return Vec3::zeros();
        }
        let beckmann =
            (-tan_theta_h * tan_theta_h / (self.alpha2)).exp() / (PI * self.alpha2 * nh.powi(4));
        let hv = halfway_vec.dot(view);
        let geometric_factor = (2.0 * nh * nv / hv).min((2.0 * nh * nl / hv).min(1.));

        let fresnel = Fresnel::SchlickDielectric {
            ior_in: self.ior,
            ior_out: 1.,
        }
        .value(halfway_vec, view);

        self.color * fresnel * beckmann * geometric_factor / (4.0 * nv * nl)
    }

    fn pdf(&self) -> Pdf {
        Pdf::Beckmann(self.alpha2, ReflectType::Reflect)
    }

    fn is_dirac(&self) -> bool {
        false
    }
}

#[derive(Debug, Clone, Copy)]
pub struct CookTorranceRefract {
    alpha2: f64,
    ior: f64,
    color: Vec3,
}

impl CookTorranceRefract {
    fn evaluate(
        &self,
        position: Vec3,
        normal: Vec3,
        halfway_vec: Vec3,
        view: Vec3,
        pdf: Pdf,
    ) -> (Vec3, Option<Ray>) {
        let light = if let Some(light) = refract(halfway_vec, view, self.ior) {
            light
        } else {
            return (Vec3::zeros(), None);
        };

        let (ior_ratio, new_halfway_vec, new_normal) = if normal.dot(view) > 0.0 {
            (1.0 / self.ior, halfway_vec, normal)
        } else {
            (self.ior, -1. * halfway_vec, -1. * normal)
        };

        let nl = new_normal.dot(light);
        // If the light vector points outside the object we terminate
        if nl >= 0.0 {
            return (Vec3::zeros(), None);
        }

        let hl = new_halfway_vec.dot(light).abs();
        let denom = new_halfway_vec.dot(view).abs() + 1. / ior_ratio * hl;
        let denom = denom * denom;
        let dwh_dwi = 1. / (ior_ratio * ior_ratio) * hl / denom;

        let color = self.brdf(position, normal, light, view) * nl.abs()
            / (pdf.value(position, normal, light, view) * dwh_dwi);
        if color.is_zeros() {
            (color, None)
        } else {
            (color, Some(Ray::new(position, light)))
        }
    }
}

impl Brdf for CookTorranceRefract {
    fn brdf(&self, _position: Vec3, normal: Vec3, light: Vec3, view: Vec3) -> Vec3 {
        let nv = normal.dot(view);
        let nl = normal.dot(light).abs();

        let (ior_ratio, normal) = if nv > 0. {
            (1.0 / self.ior, normal)
        } else {
            (self.ior, -1. * normal)
        };

        let nv = nv.abs();

        let halfway_vec = light + ior_ratio * view;

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

        if tan_theta_h.is_infinite() {
            return Vec3::zeros();
        }

        let beckmann =
            (-tan_theta_h * tan_theta_h / (self.alpha2)).exp() / (PI * self.alpha2 * nh.powi(4));

        let hv = halfway_vec.dot(view).abs();
        let hl = halfway_vec.dot(light).abs();

        let geometric_factor = (2.0 * nh * nv / hv).min((2.0 * nh * nl / hl).min(1.));

        let fresnel = Fresnel::SchlickDielectric {
            ior_in: self.ior,
            ior_out: 1.,
        }
        .value(halfway_vec, view)
        .x();

        let denom = hv + 1. / ior_ratio * hl;
        let denom = denom * denom * nv * nl;

        self.color * (1.0 - fresnel) * beckmann * geometric_factor / denom * hv * hl
    }

    fn pdf(&self) -> Pdf {
        Pdf::Beckmann(self.alpha2, ReflectType::Refract(self.ior))
    }

    fn is_dirac(&self) -> bool {
        false
    }
}

#[derive(Debug, Clone)]
pub struct CookTorranceGlass {
    alpha2: f64,
    ior: f64,
    refract: CookTorranceRefract,
    reflect: CookTorranceDielectric,
}

#[derive(Debug, Clone)]
pub enum Material {
    Diffuse(LambertianDiffuse),
    Mirror(Mirror),
    Refract(Refract),
    Glass(Glass),
    CookTorranceMetallic(CookTorranceMetallic),
    CookTorranceDielectric(CookTorranceDielectric),
    CookTorranceRefract(CookTorranceRefract),
    CookTorranceGlass(CookTorranceGlass),
    Mix(MixKind, Box<Material>, Box<Material>),
    NoReflect,
}

impl Material {
    pub fn diffuse(color: Vec3) -> Material {
        Material::Diffuse(LambertianDiffuse { color })
    }

    pub fn mirror(color: Vec3) -> Material {
        Material::Mirror(Mirror { color })
    }

    pub fn refract(ior: f64, color: Vec3) -> Material {
        Material::Refract(Refract { ior, color })
    }

    pub fn glass(ior: f64, color: Vec3) -> Material {
        Material::Glass(Glass {
            ior,
            refract: Box::new(Material::refract(ior, color)),
            reflect: Box::new(Material::mirror(color)),
        })
    }

    pub fn cook_torrance_metallic(alpha: f64, color: Vec3) -> Material {
        Material::CookTorranceMetallic(CookTorranceMetallic {
            alpha2: alpha * alpha,
            color,
        })
    }

    pub fn cook_torrance_dielectric(alpha: f64, ior: f64, color: Vec3) -> Material {
        Material::CookTorranceDielectric(CookTorranceDielectric {
            alpha2: alpha * alpha,
            ior,
            color,
        })
    }

    pub fn cook_torrance_refract(alpha: f64, ior: f64, color: Vec3) -> Material {
        Material::CookTorranceRefract(CookTorranceRefract {
            alpha2: alpha * alpha,
            ior,
            color,
        })
    }

    pub fn cook_torrance_glass(alpha: f64, ior: f64, color: Vec3) -> Material {
        let alpha2 = alpha * alpha;
        Material::CookTorranceGlass(CookTorranceGlass {
            ior,
            alpha2,
            refract: CookTorranceRefract { alpha2, ior, color },
            reflect: CookTorranceDielectric { alpha2, ior, color },
        })
    }

    pub fn evaluate(
        &self,
        position: Vec3,
        normal: Vec3,
        view: Vec3,
        pdf: Option<Pdf>,
    ) -> (Vec3, Option<Ray>) {
        match self {
            Self::Diffuse(d) => {
                let pdf = if let Some(pdf) = pdf {
                    Pdf::Mix(MixKind::Constant(0.5), Box::new(pdf), Box::new(d.pdf()))
                } else {
                    d.pdf()
                };
                let light = pdf.generate(position, normal, view);
                let color = d.brdf(position, normal, light, view) * normal.dot(light)
                    / pdf.value(position, normal, light, view);
                (color, Some(Ray::new(position, light)))
            }
            Self::Mirror(m) => {
                let pdf = m.pdf();
                let light = pdf.generate(position, normal, view);
                let color = m.brdf(position, normal, light, view) * normal.dot(light)
                    / pdf.value(position, normal, light, view);
                (color, Some(Ray::new(position, light)))
            }
            Self::Refract(r) => {
                let pdf = r.pdf();
                let light = pdf.generate(position, normal, view);
                let color = r.brdf(position, normal, light, view) * normal.dot(light).abs()
                    / pdf.value(position, normal, light, view);
                (color, Some(Ray::new(position, light)))
            }
            Self::Glass(g) => {
                let cos_theta = normal.dot(view);

                let (ior_in, ior_out, cos_theta, normal) = if cos_theta > 0. {
                    (1., g.ior, cos_theta, normal)
                } else {
                    (g.ior, 1., -cos_theta, -1. * normal)
                };

                let sin_theta = (1. - cos_theta * cos_theta).sqrt();
                let ior_ratio = ior_in / ior_out;

                if ior_ratio * sin_theta > 1. {
                    // Total internal reflection
                    g.reflect.evaluate(position, normal, view, pdf)
                } else {
                    let fresnel =
                        Fresnel::SchlickDielectric { ior_in, ior_out }.value(normal, view);
                    if rand::random::<f64>() < fresnel.x() {
                        g.reflect.evaluate(position, normal, view, pdf) // * fresnel.x() / fresnel.x()
                    } else {
                        g.refract.evaluate(position, normal, view, pdf) // * (1 - fresnel.x()) / (1 - fresnel.x())
                    }
                }
            }
            Self::CookTorranceMetallic(ctm) => {
                let pdf = ctm.pdf();

                let halfway_vec = pdf.generate(position, normal, view);
                let light = 2. * halfway_vec.dot(view) * halfway_vec - view;

                let nl = normal.dot(light);
                if nl < 0.0 {
                    return (Vec3::zeros(), None);
                }

                let frac_dwh_dwi = 4.0 * halfway_vec.dot(light);

                let color = ctm.brdf(position, normal, light, view) * nl
                    / pdf.value(position, normal, light, view)
                    * frac_dwh_dwi;

                if color.is_zeros() {
                    (color, None)
                } else {
                    (color, Some(Ray::new(position, light)))
                }
            }
            Self::CookTorranceDielectric(ctd) => {
                let pdf = ctd.pdf();

                let halfway_vec = pdf.generate(position, normal, view);
                ctd.evaluate(position, normal, halfway_vec, view, pdf)
            }
            Self::CookTorranceRefract(ctr) => {
                let pdf = ctr.pdf();

                let halfway_vec = pdf.generate(position, normal, view);

                ctr.evaluate(position, normal, halfway_vec, view, pdf)
            }
            Self::CookTorranceGlass(ctg) => {
                let pdf = Pdf::Beckmann(ctg.alpha2, ReflectType::Reflect);
                let halfway_vec = pdf.generate(position, normal, view);

                let cos_theta = view.dot(halfway_vec);

                let (ior_in, ior_out, cos_theta, new_halfway_vec, normal) = if cos_theta > 0. {
                    (1., ctg.ior, cos_theta, halfway_vec, normal)
                } else {
                    (ctg.ior, 1., -cos_theta, -1. * halfway_vec, -1. * normal)
                };

                let sin2_theta = 1. - cos_theta * cos_theta;
                let ior_ratio = ior_in / ior_out;

                let light = if ior_ratio * ior_ratio * sin2_theta >= 1. {
                    // Total internal reflection
                    reflect(new_halfway_vec, view)
                } else {
                    refract(halfway_vec, view, ctg.ior)
                        .expect(&format!("{}", sin2_theta * ior_ratio * ior_ratio))
                };

                let nl = normal.dot(light).abs();
                let nv = normal.dot(view).abs();

                if nl == 0.0 || nv == 0.0 {
                    return (Vec3::zeros(), None);
                }
                let nh = normal.dot(new_halfway_vec);
                let theta_h = nh.acos();

                let tan_theta_h = theta_h.tan();

                if tan_theta_h.is_infinite() {
                    return (Vec3::zeros(), None);
                }
                let beckmann = (-tan_theta_h * tan_theta_h / ctg.alpha2).exp()
                    / (PI * ctg.alpha2 * nh.powi(4));

                let hl = new_halfway_vec.dot(light).abs();
                let hv = new_halfway_vec.dot(view).abs();

                let geometric_factor = (2.0 * nh * nl / nl).min((2.0 * nh * nv / hv).min(1.));

                if normal.dot(light) < 0. {
                    let inv_ior_ratio = 1. / ior_ratio;
                    let denom = hv + inv_ior_ratio * hl;
                    let denom = denom * denom;
                    let pdf = (-tan_theta_h * tan_theta_h / ctg.alpha2).exp()
                        / (PI * ctg.alpha2 * normal.dot(new_halfway_vec).powi(4))
                        * inv_ior_ratio
                        * inv_ior_ratio
                        * nl
                        / denom;

                    let col = ctg.refract.color /* * (1. - fresnel) */ * beckmann * geometric_factor
                        / (denom * nl * nv)
                        * nl
                        * hl
                        * hv;
                    (col / pdf, Some(Ray::new(position, light)))
                } else {
                    let pdf = (-tan_theta_h * tan_theta_h / ctg.alpha2).exp()
                        / (PI * ctg.alpha2 * normal.dot(new_halfway_vec).powi(4) * hl);
                    let col = ctg.reflect.color /* * fresnel */ * beckmann * geometric_factor
                        / (4.0 * nl * nv)
                        * nl;
                    (col / pdf, Some(Ray::new(position, light)))
                }
            }
            Self::Mix(factor, mat1, mat2) => {
                let fac = factor.value(position, normal, view);
                if rand::random::<f64>() < fac {
                    let (color, ray) = mat1.evaluate(position, normal, view, pdf);
                    (color / fac, ray)
                } else {
                    let (color, ray) = mat2.evaluate(position, normal, view, pdf);
                    (color / (1.0 - fac), ray)
                }
            }
            Self::NoReflect => (Vec3::zeros(), None),
        }
    }
}

fn schlick_dielectric(r0: f64, normal: Vec3, view: Vec3) -> f64 {
    r0 + (1. - r0) * (1. - normal.dot(view)).powi(5)
}

fn schlick_metallic(r0: Vec3, normal: Vec3, view: Vec3) -> Vec3 {
    r0 + (Vec3::new(1., 1., 1.) - r0) * (1. - normal.dot(view)).powi(5)
}

fn reflect(normal: Vec3, view: Vec3) -> Vec3 {
    2. * view.dot(normal) * normal - view
}

fn refract(normal: Vec3, view: Vec3, ior: f64) -> Option<Vec3> {
    let cos_theta = view.dot(normal);

    let (ior_ratio, cos_theta, normal) = if cos_theta > 0. {
        (1. / ior, cos_theta, normal)
    } else {
        (ior, -cos_theta, -1. * normal)
    };

    let sin_theta = (1. - cos_theta * cos_theta).sqrt();

    // Total internal reflection
    if ior_ratio * sin_theta > 1. {
        return None;
    }

    let dir_parallel = ior_ratio * (cos_theta * normal - view);
    let dir_perp = -(1. - dir_parallel.mag_2()).sqrt() * normal;
    Some(dir_perp + dir_parallel)
}

enum Fresnel {
    SchlickDielectric { ior_in: f64, ior_out: f64 },
    SchlickMetallic(Vec3),
}

impl Fresnel {
    fn value(&self, normal: Vec3, view: Vec3) -> Vec3 {
        match self {
            Fresnel::SchlickDielectric { ior_in, ior_out } => {
                let r0 = (ior_in - ior_out) / (ior_in + ior_out);
                let r0 = r0 * r0;
                let fresnel = schlick_dielectric(r0, normal, view);
                Vec3::new(fresnel, fresnel, fresnel)
            }
            Fresnel::SchlickMetallic(r0) => schlick_metallic(*r0, normal, view),
        }
    }
}
#[derive(Debug, Copy, Clone)]
pub enum MixKind {
    Constant(f64),
    Fresnel(f64),
}

impl MixKind {
    fn value(&self, _position: Vec3, normal: Vec3, light: Vec3) -> f64 {
        match self {
            MixKind::Constant(factor) => *factor,
            MixKind::Fresnel(ior) => {
                let r0 = (1. - ior) / (1. + ior);
                let r0 = r0 * r0;

                schlick_dielectric(r0, normal, light)
            }
        }
    }
}
