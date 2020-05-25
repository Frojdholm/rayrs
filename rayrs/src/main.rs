use rayrs_lib::image::{Image, ImageBlock};
use rayrs_lib::test_scenes;
use rayrs_lib::vecmath::Vec3;
use rayrs_lib::{self, Camera, Scene};

use std::env;
use std::fs::File;
use std::io::BufReader;
use std::path::Path;
use std::process;
use std::time::Instant;

use image::hdr::{HDREncoder, HdrDecoder};
use image::{self, ColorType, ImageError, Rgb};

use rayon::prelude::*;

const SPP: u32 = 2000;

pub fn generate_image<P, F>(hdri_path: P, outbase: &str, test_scene: F) -> Result<(), ImageError>
where
    P: AsRef<Path>,
    F: FnOnce(f64, f64, Image) -> (Camera, Scene),
{
    let hdri = File::open(hdri_path)?;
    let hdri = HdrDecoder::new(BufReader::new(hdri))?;
    let metadata = hdri.metadata();
    let hdri_pixels: Vec<Vec3> = hdri
        .read_image_hdr()?
        .iter()
        .map(|pixel| Vec3::new(pixel.0[0] as f64, pixel.0[1] as f64, pixel.0[2] as f64))
        .map(|pixel| pixel.clip(0., 3.))
        .collect();

    let hdri = Image::from_pixels(
        metadata.width as usize,
        metadata.height as usize,
        hdri_pixels,
    );

    let (c, s) = test_scene(0.000_001, 1_000_000., hdri);

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

    println!(
        "Time taken {}: {:.3} s",
        outbase,
        now.elapsed().as_secs_f64()
    );
    let image = Image::from_blocks(&blocks, x_size, y_size, width, height);

    image::save_buffer(
        format!("{}.png", outbase),
        &image.to_raw_bytes(1. / 2.2),
        image.width() as u32,
        image.height() as u32,
        ColorType::Rgb8,
    )?;

    let outfile = File::create(format!("{}.hdr", outbase))?;
    let encoder = HDREncoder::new(outfile);
    let (width, height) = (image.width(), image.height());
    let pixels: Vec<_> = image
        .pixels_f32()
        .into_iter()
        .map(|pixel| Rgb(pixel))
        .collect();
    encoder.encode(&pixels, width, height)
}

fn main() -> Result<(), ImageError> {
    let mut args = env::args();
    if args.len() != 2 {
        eprintln!("Usage: rayrs hdri_path");
        process::exit(1);
    }
    // Skip program name
    args.next();
    let hdri = args.next().unwrap();
    // generate_image(hdri, "copper_sphere", test_scenes::copper_single_sphere)?;
    // generate_image(hdri, "glass_sphere", test_scenes::glass_single_sphere)?;
    // generate_image(
    //     hdri,
    //     "spheres_metallic",
    //     test_scenes::cook_torrance_spheres_metallic,
    // )?;
    // generate_image(
    //     hdri,
    //     "spheres_plastic",
    //     test_scenes::cook_torrance_spheres_plastic,
    // )?;
    // generate_image(hdri, "copper_suzanne", test_scenes::copper_suzanne)?;
    // generate_image(hdri, "glass_suzanne", test_scenes::glass_suzanne)?;
    // generate_image(
    //     hdri,
    //     "cook_torrance_glass_sphere",
    //     test_scenes::cook_torrance_glass_single_sphere,
    // )?;
    // generate_image(
    //     hdri,
    //     "cook_torrance_spheres_frosted_glass",
    //     test_scenes::cook_torrance_spheres_frosted_glass,
    // )?;
    generate_image(hdri, "material_test", test_scenes::material_test)
}
