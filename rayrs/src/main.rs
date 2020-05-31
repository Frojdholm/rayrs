//! A path tracer that generates images based on simulating light as paths.
use rayrs_lib::image::{Image, ImageBlock};
use rayrs_lib::test_scenes;
use rayrs_lib::vecmath::{Vec3, VecElements};
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

/// Default samples per pixel.
const SPP: u32 = 2000;

/// Generate an image from a scene.
///
/// The function generates an image using the hdri and test scene. The result
/// is saved as `outbase.png` and `outbase.hdr`.
pub fn generate_image<P, F>(
    hdri_path: P,
    outbase: &str,
    spp: u32,
    test_scene: F,
) -> Result<(), ImageError>
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
    // Use 16x16 blocks.
    let (x_size, y_size, blocks) = Image::blocks(width, height, 16);

    let now = Instant::now();
    // Parallelized using rayon parallel iterators.
    let blocks: Vec<ImageBlock> = blocks
        .into_par_iter()
        .map(|mut image| {
            // Work on a single image block.
            for j in 0..image.width() {
                for i in 0..image.height() {
                    let mut pixel = Vec3::zeros();
                    for _ in 0..spp {
                        pixel += rayrs_lib::radiance(
                            &s,
                            c.generate_primary_ray(
                                // Image origin is upper left while camera
                                // origin is lower right.
                                height - i - image.offset_y(),
                                width - j - image.offset_x(),
                            ),
                            50,
                        );
                    }

                    if pixel.x().is_nan() || pixel.y().is_nan() || pixel.z().is_nan() {
                        eprintln!("NaN pixel detected");
                    }

                    if pixel.x() < 0. || pixel.y() < 0. || pixel.z() < 0. {
                        eprintln!("Negative pixel detected");
                    }

                    image.set_pixel(i, j, pixel / spp as f64);
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

    // Save png
    image::save_buffer(
        format!("{}.png", outbase),
        &image.to_raw_bytes(1. / 2.2),
        image.width() as u32,
        image.height() as u32,
        ColorType::Rgb8,
    )?;

    // Save HDR
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

/// Parse command line arguments.
fn parse_commandline(mut args: env::Args) -> Result<(String, u32), &'static str> {
    if args.len() < 2 {
        return Err("Usage: rayrs hdri_path [spp]");
    }

    args.next();
    let hdri = args.next().unwrap();
    let spp = match args.next() {
        Some(spp) => spp.parse().unwrap_or_else(|_| SPP),
        None => SPP,
    };

    Ok((hdri, spp))
}

fn main() -> Result<(), ImageError> {
    let (hdri, spp) = parse_commandline(env::args()).unwrap_or_else(|v| {
        eprintln!("{}", v);
        process::exit(1)
    });
    // generate_image(
    //     &hdri,
    //     "copper_sphere",
    //     spp,
    //     test_scenes::copper_single_sphere,
    // )?;
    // generate_image(&hdri, "glass_sphere", spp,
    // test_scenes::glass_single_sphere)?; generate_image(
    //     &hdri,
    //     "spheres_metallic",
    //     spp,
    //     test_scenes::cook_torrance_spheres_metallic,
    // )?;
    // generate_image(
    //     &hdri,
    //     "spheres_plastic",
    //     spp,
    //     test_scenes::cook_torrance_spheres_plastic,
    // )?;
    // generate_image(&hdri, "copper_suzanne", spp, test_scenes::copper_suzanne)?;
    // generate_image(&hdri, "glass_suzanne", spp, test_scenes::glass_suzanne)?;
    // generate_image(
    //     &hdri,
    //     "cook_torrance_glass_sphere",
    //     spp,
    //     test_scenes::cook_torrance_glass_single_sphere,
    // )?;
    // generate_image(
    //     &hdri,
    //     "cook_torrance_spheres_frosted_glass",
    //     spp,
    //     test_scenes::cook_torrance_spheres_frosted_glass,
    // )?;
    // generate_image(
    //     &hdri,
    //     "cook_torrance_spheres_cook_torrance_refract",
    //     spp,
    //     test_scenes::cook_torrance_spheres_cook_torrance_refract,
    // )?;
    generate_image(hdri, "material_test", spp, test_scenes::material_test)
    // generate_image(&hdri, "dragon", spp, test_scenes::sphere_dragon)
}
