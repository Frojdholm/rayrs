//! Very basic loading of .obj files.
//!
//! The file be loaded into a vector of basic shapes defined in [`geometry`].
//! These can the be made into [`Object`]s and added to a scene.
//!
//! [`geometry`]: ../geometry/index.html
//! [`Object`]: ../struct.Object.html

use super::geometry::{Sphere, Triangle};
use super::vecmath::Vec3;

use std::fs::File;
use std::io::{self, BufRead};

pub fn load_obj_file(filename: &str) -> io::Result<Vec<Triangle>> {
    let file = File::open(filename)?;
    let reader = io::BufReader::new(file);

    let mut vertices = Vec::new();
    let mut triangles = Vec::new();

    for line in reader.lines() {
        let text = line?;
        let v: Vec<_> = text.split(' ').collect();
        if v[0] == "v" {
            let x: f64 = v[1].parse().unwrap();
            let y: f64 = v[2].parse().unwrap();
            let z: f64 = v[3].parse().unwrap();
            vertices.push(Vec3::new(x, y, z));
        } else if v[0] == "f" {
            let i: usize = v[1].parse().unwrap();
            let j: usize = v[2].parse().unwrap();
            let k: usize = v[3].parse().unwrap();

            triangles.push(Triangle::new(
                vertices[i - 1],
                vertices[j - 1],
                vertices[k - 1],
            ));
        }
    }

    Ok(triangles)
}

pub fn load_obj_file_spheres(filename: &str, radius: f64) -> io::Result<Vec<Sphere>> {
    let file = File::open(filename)?;
    let reader = io::BufReader::new(file);

    let mut spheres = Vec::new();

    for line in reader.lines() {
        let text = line?;
        let v: Vec<_> = text.split(' ').collect();
        if v[0] == "v" {
            let x: f64 = v[1].parse().unwrap();
            let y: f64 = v[2].parse().unwrap();
            let z: f64 = v[3].parse().unwrap();
            spheres.push(Sphere::new(radius, Vec3::new(x, y, z)));
        }
    }

    Ok(spheres)
}
