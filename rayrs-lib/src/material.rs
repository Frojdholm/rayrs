use super::geometry::Hittable;
use super::vecmath::Vec3;
use super::Ray;

use std::f64;
use std::f64::consts::PI;

pub enum DiracType {
    Reflect,
    Refract(f64),
}

pub enum Pdf<'a> {
    Cosine,
    Uniform,
    Beckmann(f64),
    Dirac(DiracType),
    Hittable(&'a dyn Hittable),
    Mix(MixKind, Box<Pdf<'a>>, Box<Pdf<'a>>),
}

impl<'a> Pdf<'a> {
    pub fn value(&self, position: Vec3, normal: Vec3, outgoing: Vec3, incoming: Vec3) -> f64 {
        match self {
            Pdf::Cosine => normal.dot(incoming.unit()) / PI,
            Pdf::Uniform => 0.5 * PI,
            Pdf::Beckmann(m) => {
                let halfway_vec = outgoing + incoming;

                if halfway_vec.x() == 0.0 && halfway_vec.y() == 0.0 && halfway_vec.z() == 0.0 {
                    return 0.;
                }

                let halfway_vec = halfway_vec.unit();
                let theta_h = normal.dot(halfway_vec).acos();

                let tan_theta_h = theta_h.tan();

                if tan_theta_h.is_infinite() {
                    return 0.;
                }
                (-tan_theta_h * tan_theta_h / (m * m)).exp()
                    / (4.
                        * PI
                        * m
                        * m
                        * normal.dot(halfway_vec).powi(4)
                        * outgoing.dot(halfway_vec))
            }
            Pdf::Dirac(_) => 0.,
            Pdf::Hittable(geom) => {
                let ray = Ray::new(position, incoming);
                if let Some(t) = geom.intersect(ray) {
                    (position - ray.point(t)).mag_2() / (normal.dot(incoming) * geom.area())
                } else {
                    0.
                }
            }
            Pdf::Mix(kind, pdf1, pdf2) => {
                if normal.dot(incoming) < 0. {
                    return f64::INFINITY;
                }

                let factor = kind.value(position, normal, incoming);
                factor * pdf1.value(position, normal, outgoing, incoming)
                    + (1. - factor) * pdf2.value(position, normal, outgoing, incoming)
            }
        }
    }

    pub fn generate(&self, position: Vec3, normal: Vec3, incoming: Vec3) -> Vec3 {
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
            Pdf::Beckmann(m) => {
                let (e1, e2) = Vec3::orthonormal_basis(normal);

                let phi = 2. * PI * rand::random::<f64>();

                let tan2theta = -m * m * (1. - rand::random::<f64>()).ln();
                let costheta = 1.0 / (1.0 + tan2theta).sqrt();
                let sintheta = (1.0 - costheta * costheta).sqrt();

                let x = phi.cos() * sintheta;
                let y = phi.sin() * sintheta;

                let halfway_vec = x * e1 + y * e2 + costheta * normal;
                2. * incoming.dot(halfway_vec) * halfway_vec - incoming
            }
            Pdf::Dirac(typ) => match typ {
                DiracType::Reflect => 2. * incoming.dot(normal) * normal - incoming,
                DiracType::Refract(ior) => {
                    let cos_theta = incoming.dot(normal);

                    let (ior_ratio, cos_theta, normal) = if cos_theta > 0. {
                        (1. / ior, cos_theta, normal)
                    } else {
                        (*ior, -cos_theta, -1. * normal)
                    };

                    let sin_theta = (1. - cos_theta * cos_theta).sqrt();

                    // Total internal reflection
                    if ior_ratio * sin_theta > 1. {
                        return 2. * cos_theta * normal - incoming;
                    }

                    let dir_parallel = ior_ratio * (cos_theta * normal - incoming);
                    let dir_perp = -(1. - dir_parallel.mag_2()).sqrt() * normal;
                    dir_perp + dir_parallel
                }
            },
            Pdf::Hittable(geom) => (geom.sample() - position).unit(),
            Pdf::Mix(kind, pdf1, pdf2) => {
                if rand::random::<f64>() < kind.value(position, normal, incoming) {
                    pdf1.generate(position, normal, incoming)
                } else {
                    pdf2.generate(position, normal, incoming)
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

pub trait Brdf {
    fn brdf(&self, position: Vec3, normal: Vec3, omega_i: Vec3, omega_o: Vec3) -> Vec3;
    fn pdf(&self) -> Option<Pdf>;
    fn is_dirac(&self) -> bool;
}

pub trait DiracBrdf {
    // TODO: Allow passing in a pdf and factor to allow for importance sampling
    // when doing non-dirac materials.
    fn evaluate(
        &self,
        position: Vec3,
        normal: Vec3,
        omega_i: Vec3,
        pdf: Option<Pdf>,
    ) -> (Vec3, Option<Ray>);
}

#[derive(Debug, Clone)]
pub struct Glass {
    ior: f64,
    attenuation: f64,
    color: Vec3,
}

impl Glass {
    pub fn new(ior: f64, attenuation: f64, color: Vec3) -> Glass {
        Glass {
            ior,
            attenuation,
            color,
        }
    }
}

#[derive(Debug, Clone)]
pub enum Material {
    Diffuse(f64, Vec3),
    Mirror(f64, Vec3),
    Glass(Glass),
    Refract {
        attenuation: f64,
        ior: f64,
        color: Vec3,
    },
    CookTorrance {
        m: f64,
        color: Vec3,
    },
    CookTorranceDielectric {
        ior: f64,
        m: f64,
        color: Vec3,
    },
    CookTorranceGlass {
        ior: f64,
        m: f64,
        color: Vec3,
    },
    Mix(MixKind, Box<Material>, Box<Material>),
    NoReflect,
}

impl Brdf for Material {
    fn brdf(&self, position: Vec3, normal: Vec3, outgoing: Vec3, incoming: Vec3) -> Vec3 {
        match self {
            Material::Diffuse(albedo, color) => *albedo / PI * normal.dot(incoming).abs() * *color,
            Material::Mirror(_, _) => Vec3::zeros(),
            Material::Refract {
                attenuation: _,
                ior: _,
                color: _,
            } => Vec3::zeros(),
            Material::Glass(_) => Vec3::zeros(),
            Material::CookTorranceGlass {
                ior: _,
                m: _,
                color: _,
            } => Vec3::zeros(),
            Material::CookTorrance { m, color } => {
                let n_wi = normal.dot(outgoing);
                let n_wo = normal.dot(incoming);

                let halfway_vec = outgoing + incoming;

                if n_wi <= 0.0 || n_wo <= 0.0 {
                    return Vec3::zeros();
                }

                if halfway_vec.x() == 0.0 && halfway_vec.y() == 0.0 && halfway_vec.z() == 0.0 {
                    return Vec3::zeros();
                }

                let halfway_vec = halfway_vec.unit();
                let theta_h = normal.dot(halfway_vec).acos();

                let tan_theta_h = theta_h.tan();

                if tan_theta_h.is_infinite() {
                    return Vec3::zeros();
                }
                let beckmann = (-tan_theta_h * tan_theta_h / (m * m)).exp()
                    / (PI * m * m * normal.dot(halfway_vec).powi(4));

                let nh = normal.dot(halfway_vec);
                let geometric_factor =
                    (2.0 * nh * n_wi / theta_h).min((2.0 * nh * n_wo / theta_h).min(1.));

                let fresnel = Fresnel::SchlickMetallic(*color).value(halfway_vec, outgoing);

                fresnel * beckmann * geometric_factor / (4.0 * n_wi * n_wo) * n_wi
            }
            Material::CookTorranceDielectric { ior, m, color } => {
                let n_wi = normal.dot(outgoing);
                let n_wo = normal.dot(incoming);

                let halfway_vec = outgoing + incoming;

                if n_wi <= 0.0 || n_wo <= 0.0 {
                    return Vec3::zeros();
                }

                if halfway_vec.x() == 0.0 && halfway_vec.y() == 0.0 && halfway_vec.z() == 0.0 {
                    return Vec3::zeros();
                }

                let halfway_vec = halfway_vec.unit();
                let theta_h = normal.dot(halfway_vec).acos();

                let tan_theta_h = theta_h.tan();

                if tan_theta_h.is_infinite() {
                    return Vec3::zeros();
                }
                let beckmann = (-tan_theta_h * tan_theta_h / (m * m)).exp()
                    / (PI * m * m * normal.dot(halfway_vec).powi(4));

                let nh = normal.dot(halfway_vec);
                let h_wi = halfway_vec.dot(incoming);
                let geometric_factor =
                    (2.0 * nh * n_wi / h_wi).min((2.0 * nh * n_wo / h_wi).min(1.));

                // let fresnel = Fresnel::SchlickDielectric {
                //     ior_in: 1.0,
                //     ior_out: *ior,
                // }
                // .value(halfway_vec, outgoing)
                // .x();
                *color /* * fresnel */ * beckmann * geometric_factor / (4.0 * n_wi * n_wo) * n_wi
            }
            Material::Mix(kind, mat1, mat2) => {
                if normal.dot(outgoing) < 0. {
                    return Vec3::zeros();
                }

                let factor = kind.value(position, normal, outgoing);

                factor * mat1.brdf(position, normal, outgoing, incoming)
                    + (1. - factor) * mat2.brdf(position, normal, outgoing, incoming)
            }
            Material::NoReflect => Vec3::zeros(),
        }
    }

    fn pdf(&self) -> Option<Pdf> {
        match self {
            Material::Diffuse(_, _) => Some(Pdf::Cosine),
            Material::Mirror(_, _) => Some(Pdf::Dirac(DiracType::Reflect)),
            Material::Refract {
                attenuation: _,
                ior,
                color: _,
            } => Some(Pdf::Dirac(DiracType::Refract(*ior))),
            Material::Glass(_) => None,
            Material::CookTorranceGlass {
                ior: _,
                m: _,
                color: _,
            } => None,
            Material::CookTorrance { m, color: _ } => Some(Pdf::Beckmann(*m)),
            Material::CookTorranceDielectric {
                ior: _,
                m,
                color: _,
            } => Some(Pdf::Beckmann(*m)),
            Material::Mix(kind, mat1, mat2) => {
                // TODO: Check the math here, maybe the pdf still needs to be
                // multiplied with factor...
                let pdf1 = mat1.pdf();
                let pdf2 = mat2.pdf();

                match (pdf1, pdf2) {
                    (Some(p1), Some(p2)) => Some(Pdf::Mix(*kind, Box::new(p1), Box::new(p2))),
                    (Some(p1), None) => Some(p1),
                    (None, Some(p2)) => Some(p2),
                    (None, None) => None,
                }
            }
            Material::NoReflect => None,
        }
    }

    fn is_dirac(&self) -> bool {
        match self {
            Material::Mirror(_, _) => true,
            Material::Refract {
                attenuation: _,
                ior: _,
                color: _,
            } => true,
            Material::Glass(_) => true,
            Material::CookTorranceGlass {
                ior: _,
                m: _,
                color: _,
            } => true,
            Material::Mix(_, mat1, mat2) => mat1.is_dirac() || mat2.is_dirac(),
            _ => false,
        }
    }
}

impl DiracBrdf for Material {
    fn evaluate(
        &self,
        position: Vec3,
        normal: Vec3,
        outgoing: Vec3,
        pdf: Option<Pdf>,
    ) -> (Vec3, Option<Ray>) {
        match self {
            Material::Diffuse(_, _) => {
                // If we try to evaluate the Diffuse material as a Dirac material
                // we basically just do the normal scatter.
                let pdf = match pdf {
                    Some(pdf) => Pdf::Mix(
                        MixKind::Constant(0.8),
                        Box::new(self.pdf().unwrap()),
                        Box::new(pdf),
                    ),
                    None => self.pdf().unwrap(),
                };

                let incoming = pdf.generate(position, normal, outgoing);
                (
                    self.brdf(position, normal, outgoing, incoming),
                    Some(Ray::new(position, incoming)),
                )
            }
            Material::CookTorrance { m: _, color: _ } => {
                let pdf = match pdf {
                    Some(pdf) => Pdf::Mix(
                        MixKind::Constant(0.8),
                        Box::new(self.pdf().unwrap()),
                        Box::new(pdf),
                    ),
                    None => self.pdf().unwrap(),
                };

                let incoming = pdf.generate(position, normal, outgoing);

                (
                    self.brdf(position, normal, outgoing, incoming),
                    Some(Ray::new(position, incoming)),
                )
            }
            Material::CookTorranceDielectric {
                ior: _,
                m: _,
                color: _,
            } => {
                let pdf = match pdf {
                    Some(pdf) => Pdf::Mix(
                        MixKind::Constant(0.8),
                        Box::new(self.pdf().unwrap()),
                        Box::new(pdf),
                    ),
                    None => self.pdf().unwrap(),
                };

                let incoming = pdf.generate(position, normal, outgoing);

                (
                    self.brdf(position, normal, outgoing, incoming),
                    Some(Ray::new(position, incoming)),
                )
            }
            Material::Mirror(attenuation, color) => {
                // Evaluating a mirror directly will yield a single direction
                // for every input direction. The BRDF contains a dirac delta
                // so we need to special case it.
                let pdf = self.pdf().unwrap();
                let incoming = pdf.generate(position, normal, outgoing);
                (*attenuation * *color, Some(Ray::new(position, incoming)))
            }
            Material::Refract {
                attenuation,
                ior: _,
                color,
            } => {
                let pdf = self.pdf().unwrap();
                let incoming = pdf.generate(position, normal, outgoing);
                (*attenuation * *color, Some(Ray::new(position, incoming)))
            }
            Material::Glass(glass) => {
                let cos_theta = outgoing.dot(normal);

                let (ior_in, ior_out, cos_theta, normal) = if cos_theta > 0. {
                    (1., glass.ior, cos_theta, normal)
                } else {
                    (glass.ior, 1., -cos_theta, -1. * normal)
                };

                let sin_theta = (1. - cos_theta * cos_theta).sqrt();
                let ior_ratio = ior_in / ior_out;

                let incoming = if ior_ratio * sin_theta > 1. {
                    // Total internal reflection
                    2. * cos_theta * normal - outgoing
                } else {
                    let fresnel =
                        Fresnel::SchlickDielectric { ior_in, ior_out }.value(normal, outgoing);
                    if rand::random::<f64>() < fresnel.x() {
                        2. * cos_theta * normal - outgoing
                    } else {
                        let dir_parallel = ior_ratio * (cos_theta * normal - outgoing);
                        let dir_perp = -(1. - dir_parallel.mag_2()).sqrt() * normal;
                        dir_perp + dir_parallel
                    }
                };

                (
                    glass.attenuation * glass.color,
                    Some(Ray::new(position, incoming)),
                )
            }
            Material::CookTorranceGlass { ior, m, color } => {
                let (e1, e2) = Vec3::orthonormal_basis(normal);

                let phi = 2. * PI * rand::random::<f64>();

                let tan2theta = -m * m * (1. - rand::random::<f64>()).ln();
                let costheta = 1.0 / (1.0 + tan2theta).sqrt();
                let sintheta = (1.0 - costheta * costheta).sqrt();

                let x = phi.cos() * sintheta;
                let y = phi.sin() * sintheta;

                let halfway_vec = x * e1 + y * e2 + costheta * normal;

                let cos_theta = outgoing.dot(halfway_vec);

                let (ior_in, ior_out, cos_theta, halfway_vec, normal) = if cos_theta > 0. {
                    (1., *ior, cos_theta, halfway_vec, normal)
                } else {
                    (*ior, 1., -cos_theta, -1. * halfway_vec, -1. * normal)
                };

                let sin_theta = (1. - cos_theta * cos_theta).sqrt();
                let ior_ratio = ior_in / ior_out;

                let incoming = if ior_ratio * sin_theta > 1. {
                    // Total internal reflection
                    2. * cos_theta * halfway_vec - outgoing
                } else {
                    let fresnel =
                        Fresnel::SchlickDielectric { ior_in, ior_out }.value(halfway_vec, outgoing);
                    if rand::random::<f64>() < fresnel.x() {
                        2. * cos_theta * halfway_vec - outgoing
                    } else {
                        let dir_parallel = ior_ratio * (cos_theta * halfway_vec - outgoing);
                        let dir_perp = -(1. - dir_parallel.mag_2()).sqrt() * halfway_vec;
                        dir_perp + dir_parallel
                    }
                };

                let n_wi = normal.dot(outgoing).abs();
                let n_wo = normal.dot(incoming).abs();

                if n_wi == 0.0 || n_wo == 0.0 {
                    return (Vec3::zeros(), None);
                }
                let nh = normal.dot(halfway_vec);
                let theta_h = nh.acos();

                let tan_theta_h = theta_h.tan();

                if tan_theta_h.is_infinite() {
                    return (Vec3::zeros(), None);
                }
                let beckmann =
                    (-tan_theta_h * tan_theta_h / (m * m)).exp() / (PI * m * m * nh.powi(4));

                let h_wi = halfway_vec.dot(outgoing).abs();
                let h_wo = halfway_vec.dot(incoming).abs();

                let geometric_factor =
                    (2.0 * nh * n_wi / h_wi).min((2.0 * nh * n_wo / h_wo).min(1.));

                if normal.dot(incoming) < 0. {
                    let denom = h_wo + ior_ratio * h_wi;
                    let denom = denom * denom;
                    let pdf = (-tan_theta_h * tan_theta_h / (m * m)).exp()
                        / (4. * PI * m * m * normal.dot(halfway_vec).powi(4))
                        * ior_ratio
                        * ior_ratio
                        * h_wi
                        / denom;

                    let col = *color /* * (1. - fresnel) */ * beckmann * geometric_factor
                        / (denom * n_wi * n_wo)
                        * n_wi
                        * h_wi
                        * h_wo;
                    (col / pdf, Some(Ray::new(position, incoming)))
                } else {
                    let pdf = (-tan_theta_h * tan_theta_h / (m * m)).exp()
                        / (4.
                            * PI
                            * m
                            * m
                            * normal.dot(halfway_vec).powi(4)
                            * outgoing.dot(halfway_vec));
                    let col = *color /* * fresnel */ * beckmann * geometric_factor
                        / (4.0 * n_wi * n_wo)
                        * n_wi;
                    (col / pdf, Some(Ray::new(position, incoming)))
                }
            }
            Material::Mix(kind, mat1, mat2) => {
                if rand::random::<f64>() < kind.value(position, normal, outgoing) {
                    mat1.evaluate(position, normal, outgoing, pdf)
                } else {
                    mat2.evaluate(position, normal, outgoing, pdf)
                }
            }
            Material::NoReflect => (Vec3::zeros(), None),
        }
    }
}

fn schlick_dielectric(r0: f64, normal: Vec3, outgoing: Vec3) -> f64 {
    r0 + (1. - r0) * (1. - normal.dot(outgoing)).powi(5)
}

fn schlick_metallic(r0: Vec3, normal: Vec3, outgoing: Vec3) -> Vec3 {
    r0 + (Vec3::new(1., 1., 1.) - r0) * (1. - normal.dot(outgoing)).powi(5)
}

enum Fresnel {
    SchlickDielectric { ior_in: f64, ior_out: f64 },
    SchlickMetallic(Vec3),
}

impl Fresnel {
    fn value(&self, normal: Vec3, outgoing: Vec3) -> Vec3 {
        match self {
            Fresnel::SchlickDielectric { ior_in, ior_out } => {
                let r0 = (ior_in - ior_out) / (ior_in + ior_out);
                let r0 = r0 * r0;
                let fresnel = schlick_dielectric(r0, normal, outgoing);
                Vec3::new(fresnel, fresnel, fresnel)
            }
            Fresnel::SchlickMetallic(r0) => schlick_metallic(*r0, normal, outgoing),
        }
    }
}
#[derive(Debug, Copy, Clone)]
pub enum MixKind {
    Constant(f64),
    Fresnel(f64),
}

impl MixKind {
    fn value(&self, _position: Vec3, normal: Vec3, outgoing: Vec3) -> f64 {
        match self {
            MixKind::Constant(factor) => *factor,
            MixKind::Fresnel(ior) => {
                let r0 = (1. - ior) / (1. + ior);
                let r0 = r0 * r0;

                schlick_dielectric(r0, normal, outgoing)
            }
        }
    }
}
