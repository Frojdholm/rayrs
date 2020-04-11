use super::vecmath::Vec3;

use std::fs::File;
use std::io::Result;
use std::io::Write;

pub enum ImageFormat {
    PpmAscii,
    PpmBinary,
}

pub struct Image {
    width: usize,
    height: usize,
    image: Vec<Vec3>,
}

impl Image {
    pub fn new(width: usize, height: usize) -> Image {
        Image {
            width,
            height,
            image: vec![Vec3::new(0.0, 0.0, 0.0); width * height],
        }
    }

    pub fn get_width(&self) -> usize {
        self.width
    }

    pub fn get_height(&self) -> usize {
        self.height
    }

    pub fn pixel(&self, i: usize, j: usize) -> Vec3 {
        assert!(i < self.width && j < self.height);
        self.image[i * self.height + j]
    }

    pub fn set_pixel(&mut self, i: usize, j: usize, val: Vec3) {
        assert!(i < self.width && j < self.height);
        self.image[i * self.height + j] = val;
    }

    fn to_raw_bytes(&self, gamma: f64) -> Vec<u8> {
        let mut v = Vec::with_capacity(self.image.len() * 3);
        for val in self.image.iter() {
            let pixel = val.clip(0., 1.0).pow(gamma);
            let r = (255.99 * pixel.x()) as u8;
            let g = (255.99 * pixel.y()) as u8;
            let b = (255.99 * pixel.z()) as u8;
            v.push(r);
            v.push(g);
            v.push(b);
        }
        v
    }

    pub fn save(&self, filename: &str, image_format: ImageFormat) -> Result<()> {
        let gamma = 1. / 2.2;
        let mut f = File::create(filename)?;
        let raw_pixels = self.to_raw_bytes(gamma);

        match image_format {
            ImageFormat::PpmAscii => {
                write!(f, "P3\n{} {}\n255\n", self.width, self.height)?;
                f.write_all(
                    raw_pixels
                        .into_iter()
                        .map(|v| v.to_string())
                        .collect::<Vec<String>>()
                        .join(" ")
                        .as_bytes(),
                )
            }
            ImageFormat::PpmBinary => {
                write!(f, "P6\n{} {}\n255\n", self.width, self.height)?;
                f.write_all(&raw_pixels)
            }
        }
    }
}
