use rayrs::vecmath::Vec3;
use rayrs::{Camera, Scene};

use std::fs::File;
use std::io::Result;
use std::io::Write;
use std::time::Instant;

const SPP: u32 = 200;

fn save_image(name: &str, image: Vec<Vec<Vec3>>) -> Result<()> {
    let mut f = File::create(name)?;
    write!(f, "P3\n{} {}\n255\n", image[0].len(), image.len())?;
    for v in image.iter().rev() {
        for e in v.iter().rev() {
            let pixel = e.clip(0., 1.).pow(1. / 2.2);
            write!(
                f,
                "{} {} {} ",
                (255.99 * pixel.x) as u8,
                (255.99 * pixel.y) as u8,
                (255.99 * pixel.z) as u8
            )?;
        }
        writeln!(f, "")?;
    }
    Ok(())
}

fn main() -> Result<()> {
    let c = Camera::new(
        Vec3::new(0., 0., 7.9),
        Vec3::new(0., 1., 0.),
        Vec3::new(0., 0., 0.),
        90.,
        2.,
        2.,
        90,
    );

    let s = Scene::cornell_box(0.000001, 1000000.);

    let mut image = vec![vec![Vec3::new(0., 0., 0.); c.x_pixels()]; c.y_pixels()];
    let now = Instant::now();
    for (j, row) in image.iter_mut().enumerate() {
        let start = Instant::now();
        let mut rays = 0.;
        for (i, e) in row.iter_mut().enumerate() {
            let mut pixel = Vec3::new(0., 0., 0.);
            for _ in 0..SPP {
                pixel = &pixel + &rayrs::radiance(&s, c.generate_primary_ray(i, j), 50, &mut rays);
            }
            *e = (1. / SPP as f64) * &pixel;
        }
        print!(
            "\r{:.3} Mrays/s         ",
            rays / start.elapsed().as_secs_f64() * 1.0e-6
        )
    }
    println!();
    println!("Time take: {:.3} s", now.elapsed().as_secs_f64());

    save_image("test.ppm", image)
}
