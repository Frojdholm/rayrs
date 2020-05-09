use rayrs_lib::image::{Image, ImageBlock, ImageFormat};
use rayrs_lib::vecmath::Vec3;
use rayrs_lib::{self, Camera, Scene};

use std::io::Result;

use std::time::Instant;

use rayon::prelude::*;

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

    let s = Scene::dragon(0.000_001, 1_000_000.);
    let width = c.x_pixels();
    let height = c.y_pixels();
    let (x_size, y_size, blocks) = Image::blocks(width, height, 64);
    let now = Instant::now();
    let blocks: Vec<ImageBlock> = blocks
        .into_par_iter()
        .map(|mut image| {
            for j in 0..image.width() {
                for i in 0..image.height() {
                    let mut pixel = Vec3::new(0., 0., 0.);
                    for _ in 0..SPP {
                        pixel += rayrs_lib::radiance(
                            &s,
                            c.generate_primary_ray(
                                height - i - image.offset_y(),
                                width - j - image.offset_x(),
                            ),
                            50,
                        );
                    }
                    image.set_pixel(i, j, (1. / SPP as f64) * pixel);
                }
            }
            image
        })
        .collect();

    let image = Image::from_blocks(&blocks, x_size, y_size, width, height);
    println!("Time taken: {:.3} s", now.elapsed().as_secs_f64());

    image.save("test.ppm", ImageFormat::PpmBinary)
}
