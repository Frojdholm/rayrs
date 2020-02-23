use super::geometry::Geometry;
use super::geometry::Hittable;
use super::vecmath::Vec3;
use super::Ray;

use std::f64::consts::PI;
use std::ops::{Add, Mul};

pub enum Pdf {
    Cosine,
    Uniform,
    Dirac,
    Hittable(Geometry),
    Mix(MixKind, Box<Pdf>, Box<Pdf>),
}

impl Pdf {
    pub fn value(&self, position: &Vec3, normal: &Vec3, outgoing: &Vec3, incoming: &Vec3) -> f64 {
        match self {
            Pdf::Cosine => normal.dot(&incoming.unit()) / PI,
            Pdf::Uniform => 0.5 * PI,
            Pdf::Dirac => 0.,
            Pdf::Hittable(geom) => {
                let ray = Ray::new((*position).clone(), (*incoming).clone());
                if let Some(t) = geom.intersect(&ray) {
                    (position - &ray.point(t)).mag2() / (normal.dot(incoming) * geom.area())
                } else {
                    0.
                }
            }
            Pdf::Mix(kind, pdf1, pdf2) => {
                let factor = kind.value(position, normal, outgoing);
                factor * pdf1.value(position, normal, outgoing, incoming)
                    + (1. - factor) * pdf2.value(position, normal, outgoing, incoming)
            }
        }
    }

    pub fn generate(&self, position: &Vec3, normal: &Vec3, outgoing: &Vec3) -> Vec3 {
        match self {
            Pdf::Cosine => {
                let (e1, e2) = Vec3::orthonormal_basis(normal);

                let u: f64 = rand::random();
                let phi: f64 = 2. * PI * rand::random::<f64>();

                let x = phi.cos() * u.sqrt();
                let y = phi.sin() * u.sqrt();
                let z = (1. - u).sqrt();

                e1.mul(x).add(&e2.mul(y)).add(&normal.mul(z))
            }
            Pdf::Uniform => {
                let (e1, e2) = Vec3::orthonormal_basis(normal);

                let u: f64 = rand::random();
                let phi: f64 = 2. * PI * rand::random::<f64>();

                let x = phi.cos() * (u * (1. - u)).sqrt();
                let y = phi.sin() * (u * (1. - u)).sqrt();
                let z = 1. - u;

                e1.mul(x).add(&e2.mul(y)).add(&normal.mul(z))
            }
            Pdf::Dirac => &normal.mul(2. * outgoing.dot(normal)) - outgoing,
            Pdf::Hittable(geom) => (&geom.sample() - position).unit(),
            Pdf::Mix(kind, pdf1, pdf2) => {
                if rand::random::<f64>() < kind.value(position, normal, outgoing) {
                    pdf1.generate(position, normal, outgoing)
                } else {
                    pdf2.generate(position, normal, outgoing)
                }
            }
        }
    }

    pub fn is_dirac(&self) -> bool {
        match self {
            Pdf::Dirac => true,
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
            color.x >= 0.
                && color.x <= 1.
                && color.y >= 0.
                && color.y <= 1.
                && color.z >= 0.
                && color.z <= 1.,
            "RGB values need to be between 0 and 1"
        );
        Emission::Emissive(strength, color)
    }
}

impl Emitting for Emission {
    fn emit(&self) -> Vec3 {
        match self {
            Emission::Emissive(strength, color) => *strength * color,
            Emission::Dark => Vec3::new(0., 0., 0.),
        }
    }
}

pub trait Brdf {
    fn brdf(&self, position: &Vec3, normal: &Vec3, omega_i: &Vec3, omega_o: &Vec3) -> Vec3;
    fn pdf(&self) -> Option<Pdf>;
    fn is_dirac(&self) -> bool;
}

pub trait DiracBrdf {
    // TODO: Allow passing in a pdf and factor to allow for importance sampling
    // when doing non-dirac materials.
    fn evaluate(&self, position: &Vec3, normal: &Vec3, omega_i: &Vec3) -> (Vec3, Option<Ray>);
}

#[derive(Debug, Clone)]
pub enum Material {
    Diffuse(f64, Vec3),
    Mirror(f64, Vec3),
    //GlossyDiffuse(f64, Vec3, Vec3),
    Mix(MixKind, Box<Material>, Box<Material>),
    NoReflect,
}

impl Brdf for Material {
    fn brdf(&self, position: &Vec3, normal: &Vec3, outgoing: &Vec3, incoming: &Vec3) -> Vec3 {
        match self {
            Material::Diffuse(albedo, color) => *albedo / PI * normal.dot(incoming) * color,
            Material::Mirror(_, _) => Vec3::new(0., 0., 0.),
            // Material::GlossyDiffuse(n, col_spec, col_diff) => {
            //     // TODO: Finish implementation
            //     let n_wi = normal.dot(outgoing);
            //     let n_wo = normal.dot(incoming);

            //     let omega_h = incoming + outgoing;

            //     let r0 = (1. - *n) / (1. + *n);
            //     let r0 = r0 * r0;
            //     let fresnel = r0 + (1. - r0) * n_wi.powf(5.);
            //     let brdf_diff = ((28. / (23. * PI)) * col_diff)
            //         .mul(&(&Vec3::new(1., 1., 1.) - col_spec))
            //         .mul(1. - (1. - n_wi / 2.).powf(5.))
            //         .mul(1. - (1. - n_wo / 2.).powf(5.));

            //     Vec3::new(0., 0., 0.)
            // }
            Material::Mix(kind, mat1, mat2) => {
                let factor = kind.value(position, normal, outgoing);
                &mat1.brdf(position, normal, outgoing, incoming).mul(factor)
                    + &mat2
                        .brdf(position, normal, outgoing, incoming)
                        .mul(1. - factor)
            }
            Material::NoReflect => Vec3::new(0., 0., 0.),
        }
    }

    fn pdf(&self) -> Option<Pdf> {
        match self {
            Material::Diffuse(_, _) => Some(Pdf::Cosine),
            Material::Mirror(_, _) => Some(Pdf::Dirac),
            Material::Mix(kind, mat1, mat2) => {
                // TODO: Check the math here, maybe the pdf still needs to be
                // multiplied with factor...
                let pdf1 = mat1.pdf();
                let pdf2 = mat2.pdf();

                if pdf1.is_some() && pdf2.is_some() {
                    Some(Pdf::Mix(
                        (*kind).clone(),
                        Box::new(pdf1.unwrap()),
                        Box::new(pdf2.unwrap()),
                    ))
                } else if pdf1.is_some() && pdf2.is_none() {
                    Some(pdf1.unwrap())
                } else if pdf1.is_none() && pdf2.is_some() {
                    Some(pdf2.unwrap())
                } else {
                    None
                }
            }
            Material::NoReflect => None,
        }
    }

    fn is_dirac(&self) -> bool {
        match self {
            Material::Mirror(_, _) => true,
            Material::Mix(_, mat1, mat2) => mat1.is_dirac() || mat2.is_dirac(),
            _ => false,
        }
    }
}

impl DiracBrdf for Material {
    fn evaluate(&self, position: &Vec3, normal: &Vec3, outgoing: &Vec3) -> (Vec3, Option<Ray>) {
        match self {
            Material::Diffuse(_, _) => {
                // If we try to evaluate the Diffuse material as a Dirac material
                // we basically just do the normal scatter.
                let pdf = self.pdf().unwrap();
                let incoming = pdf.generate(position, normal, outgoing);
                (
                    self.brdf(position, normal, outgoing, &incoming),
                    Some(Ray::new((*position).clone(), incoming)),
                )
            }
            Material::Mirror(attenuation, color) => {
                // Evaluating a mirror directly will yield a single direction
                // for every input direction. The BRDF contains a dirac delta
                // so we need to special case it.
                let pdf = self.pdf().unwrap();
                let incoming = pdf.generate(position, normal, outgoing);
                (
                    *attenuation * color,
                    Some(Ray::new((*position).clone(), incoming)),
                )
            }
            Material::Mix(kind, mat1, mat2) => {
                if rand::random::<f64>() < kind.value(position, normal, outgoing) {
                    mat1.evaluate(position, normal, outgoing)
                } else {
                    mat2.evaluate(position, normal, outgoing)
                }
            }
            Material::NoReflect => (Vec3::new(0., 0., 0.), None),
        }
    }
}

#[derive(Debug, Clone)]
pub enum MixKind {
    Constant(f64),
    Fresnel(f64),
}

impl MixKind {
    fn value(&self, _position: &Vec3, normal: &Vec3, outgoing: &Vec3) -> f64 {
        match self {
            MixKind::Constant(factor) => *factor,
            MixKind::Fresnel(ior) => {
                let r0 = (1. - ior) / (1. + ior);
                let r0 = r0 * r0;

                r0 + (1. - r0) * (1. - normal.dot(outgoing)).powf(5.)
            }
        }
    }
}
