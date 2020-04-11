use rayrs::image::{Image, ImageFormat};
use rayrs::vecmath::Vec3;
use rayrs::{Camera, Scene};

use std::io::Result;

use std::time::Instant;

const SPP: u32 = 200;

fn main() -> Result<()> {
    let c = Camera::new(
        Vec3::new(0., 0., 7.9),
        Vec3::new(0., 1., 0.),
        Vec3::new(0., 0., 0.),
        90.,
        2.,
        2.,
        50,
    );

    let s = Scene::dragon(0.000001, 1000000.);

    let mut image = Image::new(c.x_pixels(), c.y_pixels());
    let now = Instant::now();
    for j in 0..image.get_width() {
        let start = Instant::now();
        let mut rays = 0.;
        for i in 0..image.get_height() {
            let mut pixel = Vec3::new(0., 0., 0.);
            for _ in 0..SPP {
                pixel += rayrs::radiance(
                    &s,
                    c.generate_primary_ray(image.get_height() - i, image.get_width() - j),
                    50,
                    &mut rays,
                );
            }
            image.set_pixel(i, j, (1. / SPP as f64) * pixel);
        }
        print!(
            "\r{:.3} Mrays/s         ",
            rays / start.elapsed().as_secs_f64() * 1.0e-6
        )
    }
    println!();
    println!("Time take: {:.3} s", now.elapsed().as_secs_f64());

    image.save("test.ppm", ImageFormat::PpmBinary)
}
