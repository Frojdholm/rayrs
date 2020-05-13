use rayrs_lib::image::{Image, ImageBlock};
use rayrs_lib::vecmath::Vec3;
use rayrs_lib::{self, Camera, Scene};

use std::fs::File;
use std::io::BufReader;
use std::time::Instant;

use image::hdr::HdrDecoder;
use image::{self, ColorType, ImageError};

use rayon::prelude::*;

const SPP: u32 = 1000;

fn main() -> Result<(), ImageError> {
    let hdri = File::open("small_cathedral_02_2k.hdr")?;
    let hdri = HdrDecoder::new(BufReader::new(hdri))?;
    let metadata = hdri.metadata();
    let hdri_pixels: Vec<Vec3> = hdri
        .read_image_hdr()?
        .iter()
        .map(|pixel| Vec3::new(pixel.0[0] as f64, pixel.0[1] as f64, pixel.0[2] as f64))
        .collect();

    let hdri = Image::from_pixels(
        metadata.width as usize,
        metadata.height as usize,
        hdri_pixels,
    );

    let c = Camera::new(
        Vec3::new(10., 10., 10.),
        Vec3::unit_y(),
        Vec3::unit_y(),
        20.,
        1920. / 1000.,
        1080. / 1000.,
        150,
    );

    let s = Scene::cook_torrance_test(0.000_001, 1_000_000., hdri);
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
                    image.set_pixel(i, j, pixel / SPP as f64);
                }
            }
            image
        })
        .collect();

    let image = Image::from_blocks(&blocks, x_size, y_size, width, height);
    println!("Time taken: {:.3} s", now.elapsed().as_secs_f64());

    image::save_buffer(
        "test.png",
        &image.to_raw_bytes(1. / 2.2),
        image.width() as u32,
        image.height() as u32,
        ColorType::Rgb8,
    )
}
