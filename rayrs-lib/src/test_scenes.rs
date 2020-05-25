use super::bvh::BvhHeuristic;
use super::geometry::Axis;
use super::image::Image;
use super::material::{Emission, Material};
use super::vecmath::Vec3;
use super::wavefront_obj;
use super::{Camera, Object, Scene};

fn single_sphere(z_near: f64, z_far: f64, hdri: Image, mat: Material) -> (Camera, Scene) {
    let floor = Material::cook_torrance_metallic(0.5, Vec3::ones() * 0.8);
    let bottom = Object::plane(Axis::Y, -25., 25., -25., 25., 0., floor, Emission::Dark);

    let sphere = Object::sphere(1., Vec3::unit_y(), mat, Emission::Dark);
    let objects = vec![bottom, sphere];

    (
        Camera::new(
            Vec3::new(0., 5., 10.),
            Vec3::unit_y(),
            Vec3::unit_y(),
            50.,
            1920. / 1000.,
            1080. / 1000.,
            200,
        ),
        Scene::new(
            objects,
            z_near,
            z_far,
            BvhHeuristic::Sah { splits: 1000 },
            hdri,
        ),
    )
}

pub fn copper_single_sphere(z_near: f64, z_far: f64, hdri: Image) -> (Camera, Scene) {
    let mat = Material::cook_torrance_metallic(0.05, Vec3::new(0.722, 0.451, 0.2));
    single_sphere(z_near, z_far, hdri, mat)
}

pub fn glass_single_sphere(z_near: f64, z_far: f64, hdri: Image) -> (Camera, Scene) {
    let mat = Material::glass(1.45, Vec3::ones() * 0.8);
    single_sphere(z_near, z_far, hdri, mat)
}

pub fn cook_torrance_glass_single_sphere(z_near: f64, z_far: f64, hdri: Image) -> (Camera, Scene) {
    let mat = Material::cook_torrance_glass(0.05, 1.45, Vec3::ones() * 0.8);
    single_sphere(z_near, z_far, hdri, mat)
}

fn suzanne(z_near: f64, z_far: f64, hdri: Image, mat: Material) -> (Camera, Scene) {
    let floor = Material::cook_torrance_metallic(0.5, Vec3::ones() * 0.8);
    let bottom = Object::plane(Axis::Y, -25., 25., -25., 25., 0., floor, Emission::Dark);

    let suzanne = wavefront_obj::load_obj_file("suzanne.obj").unwrap();
    let suzanne = Object::from_triangles(suzanne, mat, Emission::Dark);

    let mut objects = vec![bottom];
    objects.extend(suzanne);

    (
        Camera::new(
            Vec3::new(0., 5., 10.),
            Vec3::unit_y(),
            Vec3::unit_y(),
            50.,
            1920. / 1000.,
            1080. / 1000.,
            200,
        ),
        Scene::new(
            objects,
            z_near,
            z_far,
            BvhHeuristic::Sah { splits: 1000 },
            hdri,
        ),
    )
}

pub fn copper_suzanne(z_near: f64, z_far: f64, hdri: Image) -> (Camera, Scene) {
    let mat = Material::cook_torrance_metallic(0.05, Vec3::new(0.722, 0.451, 0.2));
    suzanne(z_near, z_far, hdri, mat)
}

pub fn glass_suzanne(z_near: f64, z_far: f64, hdri: Image) -> (Camera, Scene) {
    let mat = Material::glass(1.45, Vec3::ones() * 0.8);
    suzanne(z_near, z_far, hdri, mat)
}

fn multiple_spheres(z_near: f64, z_far: f64, hdri: Image, mats: Vec<Material>) -> (Camera, Scene) {
    let floor = Material::cook_torrance_metallic(0.5, Vec3::ones() * 0.8);
    let bottom = Object::plane(Axis::Y, -25., 25., -25., 25., 0., floor, Emission::Dark);
    let len = mats.len() as isize;
    let spheres: Vec<_> = mats
        .into_iter()
        .enumerate()
        .map(|(i, mat)| {
            Object::sphere(
                1.,
                Vec3::unit_y() + Vec3::new(2.2 * (i as isize - len / 2) as f64, 0., 0.),
                mat,
                Emission::Dark,
            )
        })
        .collect();
    let mut objects = vec![bottom];
    objects.extend(spheres);

    (
        Camera::new(
            Vec3::new(0., 10., 20.),
            Vec3::unit_y(),
            Vec3::unit_y(),
            72.,
            1920. / 1000.,
            400. / 1000.,
            250,
        ),
        Scene::new(
            objects,
            z_near,
            z_far,
            BvhHeuristic::Sah { splits: 1000 },
            hdri,
        ),
    )
}

pub fn cook_torrance_spheres_metallic(z_near: f64, z_far: f64, hdri: Image) -> (Camera, Scene) {
    let mats = (0..7)
        .map(|i| Material::cook_torrance_metallic(0.01 * (4 * i + 1) as f64, Vec3::ones() * 0.8))
        .collect();
    multiple_spheres(z_near, z_far, hdri, mats)
}

pub fn cook_torrance_spheres_plastic(z_near: f64, z_far: f64, hdri: Image) -> (Camera, Scene) {
    let mats = (0..7)
        .map(|i| {
            Material::cook_torrance_dielectric(0.01 * (4 * i + 1) as f64, 1.45, Vec3::ones() * 0.8)
        })
        .collect();

    multiple_spheres(z_near, z_far, hdri, mats)
}

pub fn cook_torrance_spheres_frosted_glass(
    z_near: f64,
    z_far: f64,
    hdri: Image,
) -> (Camera, Scene) {
    let mats = (0..7)
        .map(|i| Material::cook_torrance_glass(0.01 * (4 * i + 1) as f64, 1.45, Vec3::ones() * 0.8))
        .collect();
    multiple_spheres(z_near, z_far, hdri, mats)
}

pub fn material_test(z_near: f64, z_far: f64, hdri: Image) -> (Camera, Scene) {
    let mats = vec![
        Material::diffuse(Vec3::ones() * 0.8),
        Material::cook_torrance_metallic(0.05, Vec3::ones() * 0.8),
        Material::cook_torrance_dielectric(0.05, 1.45, Vec3::ones() * 0.8),
        Material::cook_torrance_refract(0.05, 1.45, Vec3::ones() * 0.8),
        Material::cook_torrance_glass(0.05, 1.45, Vec3::ones()),
        Material::mirror(Vec3::ones() * 0.8),
        Material::refract(1.45, Vec3::ones() * 0.8),
        Material::glass(1.45, Vec3::ones() * 0.8),
    ];
    let floor = Material::cook_torrance_metallic(0.5, Vec3::ones() * 0.8);

    let bottom = Object::plane(
        Axis::Y,
        -25.,
        25.,
        -25.,
        25.,
        0.,
        floor.clone(),
        Emission::Dark,
    );
    let back = Object::plane(Axis::ZRev, -25., 25., 0., 5., -5., floor, Emission::Dark);
    let len = mats.len() as isize;
    let spheres: Vec<_> = mats
        .into_iter()
        .enumerate()
        .map(|(i, mat)| {
            Object::sphere(
                1.,
                Vec3::unit_y() + Vec3::new(2.2 * (i as isize - len / 2) as f64, 0., 0.),
                mat,
                Emission::Dark,
            )
        })
        .collect();
    let mut objects = vec![bottom, back];
    objects.extend(spheres);

    (
        Camera::new(
            Vec3::new(0., 3., 20.),
            Vec3::unit_y(),
            Vec3::unit_y(),
            90.,
            1920. / 1000.,
            250. / 1000.,
            250,
        ),
        Scene::new(
            objects,
            z_near,
            z_far,
            BvhHeuristic::Sah { splits: 1000 },
            hdri,
        ),
    )
}
