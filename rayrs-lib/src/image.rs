use super::vecmath::Vec3;

use std::fs::File;
use std::io::Result;
use std::io::Write;

pub enum ImageFormat {
    PpmAscii,
    PpmBinary,
}

pub struct ImageBlock {
    offset_x: usize,
    offset_y: usize,
    image: Image,
}

impl ImageBlock {
    fn new(width: usize, height: usize, offset_x: usize, offset_y: usize) -> ImageBlock {
        ImageBlock {
            offset_x,
            offset_y,
            image: Image::new(width, height),
        }
    }

    pub fn offset_x(&self) -> usize {
        self.offset_x
    }

    pub fn offset_y(&self) -> usize {
        self.offset_y
    }

    pub fn width(&self) -> usize {
        self.image.width()
    }

    pub fn height(&self) -> usize {
        self.image.height()
    }

    pub fn pixel(&self, i: usize, j: usize) -> Vec3 {
        self.image.pixel(i, j)
    }

    pub fn set_pixel(&mut self, i: usize, j: usize, val: Vec3) {
        self.image.set_pixel(i, j, val)
    }
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

    pub fn blocks(
        width: usize,
        height: usize,
        block_size: usize,
    ) -> (usize, usize, Vec<ImageBlock>) {
        assert!(width > block_size && height > block_size);
        let mut images = Vec::new();

        let mut height_left = height as i32;
        let mut width_left = width as i32;

        while height_left > 0 {
            while width_left > 0 {
                let offset_x = width - width_left as usize;
                let offset_y = height - height_left as usize;

                if width_left < block_size as i32 {
                    if height_left < block_size as i32 {
                        images.push(ImageBlock::new(
                            width_left as usize,
                            height_left as usize,
                            offset_x,
                            offset_y,
                        ));
                    } else {
                        images.push(ImageBlock::new(
                            width_left as usize,
                            block_size,
                            offset_x,
                            offset_y,
                        ));
                    }
                } else {
                    if height_left < block_size as i32 {
                        images.push(ImageBlock::new(
                            block_size,
                            height_left as usize,
                            offset_x,
                            offset_y,
                        ));
                    } else {
                        images.push(ImageBlock::new(block_size, block_size, offset_x, offset_y));
                    }
                }
                width_left -= block_size as i32;
            }
            height_left -= block_size as i32;
            width_left = width as i32;
        }

        let x_size = if width % block_size == 0 {
            width / block_size
        } else {
            width / block_size + 1
        };

        let y_size = if height % block_size == 0 {
            height / block_size
        } else {
            height / block_size + 1
        };
        assert!(x_size * y_size == images.len());
        (x_size, y_size, images)
    }

    pub fn from_blocks(
        blocks: &[ImageBlock],
        width: usize,
        height: usize,
        pix_width: usize,
        pix_height: usize,
    ) -> Image {
        assert!(width * height == blocks.len());

        let mut pixels = vec![Vec3::new(0.0, 0.0, 0.0); pix_width * pix_height];

        for block in blocks {
            for i in 0..block.height() {
                for j in 0..block.width() {
                    pixels[(i + block.offset_y) * pix_height + j + block.offset_x] =
                        block.pixel(i, j);
                }
            }
        }

        Image {
            width: pix_width,
            height: pix_height,
            image: pixels,
        }
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    pub fn pixel(&self, i: usize, j: usize) -> Vec3 {
        assert!(i < self.height && j < self.width);
        self.image[i * self.width + j]
    }

    pub fn set_pixel(&mut self, i: usize, j: usize, val: Vec3) {
        assert!(i < self.height && j < self.width);
        self.image[i * self.width + j] = val;
    }

    fn to_raw_bytes(&self, gamma: f64) -> Vec<u8> {
        self.image
            .iter()
            .fold(Vec::with_capacity(self.image.len() * 3), |mut acc, val| {
                let pixel = val.clip(0., 1.).powf(gamma);
                let r = (255.99 * pixel.x()) as u8;
                let g = (255.99 * pixel.y()) as u8;
                let b = (255.99 * pixel.z()) as u8;
                acc.push(r);
                acc.push(g);
                acc.push(b);
                acc
            })
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
