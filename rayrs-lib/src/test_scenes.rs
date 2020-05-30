use super::bvh::BvhHeuristic;
use super::geometry::Axis;
use super::image::Image;
use super::material::{
    CookTorrance, CookTorranceGlass, CookTorranceRefract, Emission, Fresnel, Glass,
    LambertianDiffuse, Material, Plastic, Reflect, Refract,
};
use super::vecmath::Vec3;
use super::wavefront_obj;
use super::{Camera, Object, Scene};

fn single_sphere(z_near: f64, z_far: f64, hdri: Image, mat: Material) -> (Camera, Scene) {
    let floor = Material::CookTorrance(CookTorrance::new(
        Vec3::ones(),
        0.5,
        Fresnel::SchlickMetallic(Vec3::ones() * 0.8),
    ));

    let bottom = Object::plane(Axis::Y, -25., 25., -25., 25., 0., floor, Emission::Dark);

    let sphere = Object::sphere(1., Vec3::unit_y().as_vec(), mat, Emission::Dark);
    let objects = vec![bottom, sphere];

    (
        Camera::new(
            Vec3::new(0., 5., 10.),
            Vec3::unit_y().as_vec(),
            Vec3::unit_y().as_vec(),
            50.,
            1920. / 500.,
            1080. / 500.,
            100,
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
    let mat = Material::CookTorrance(CookTorrance::new(
        Vec3::ones(),
        0.05,
        Fresnel::SchlickMetallic(Vec3::new(0.722, 0.451, 0.2)),
    ));
    single_sphere(z_near, z_far, hdri, mat)
}

pub fn glass_single_sphere(z_near: f64, z_far: f64, hdri: Image) -> (Camera, Scene) {
    let mat = Material::Glass(Glass::new(Vec3::ones() * 0.8, 1.45));
    single_sphere(z_near, z_far, hdri, mat)
}

pub fn cook_torrance_glass_single_sphere(z_near: f64, z_far: f64, hdri: Image) -> (Camera, Scene) {
    let mat = Material::CookTorranceGlass(CookTorranceGlass::new(Vec3::ones() * 0.8, 0.05, 1.45));
    single_sphere(z_near, z_far, hdri, mat)
}

fn obj_scene(
    z_near: f64,
    z_far: f64,
    hdri: Image,
    mat: Material,
    obj_path: &str,
) -> (Camera, Scene) {
    let floor = Material::CookTorrance(CookTorrance::new(
        Vec3::ones(),
        0.5,
        Fresnel::SchlickMetallic(Vec3::ones() * 0.8),
    ));

    let bottom = Object::plane(Axis::Y, -25., 25., -25., 25., 0., floor, Emission::Dark);

    let obj = wavefront_obj::load_obj_file(obj_path).unwrap();
    let obj = Object::from_triangles(obj, mat, Emission::Dark);

    let mut objects = vec![bottom];
    objects.extend(obj);

    (
        Camera::new(
            Vec3::new(0., 5., 10.),
            Vec3::unit_y().as_vec(),
            Vec3::unit_y().as_vec(),
            50.,
            1920. / 500.,
            1080. / 500.,
            100,
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

fn obj_scene_sphere(
    z_near: f64,
    z_far: f64,
    hdri: Image,
    mat: Material,
    obj_path: &str,
    radius: f64,
) -> (Camera, Scene) {
    let floor = Material::CookTorrance(CookTorrance::new(
        Vec3::ones(),
        0.5,
        Fresnel::SchlickMetallic(Vec3::ones() * 0.8),
    ));

    let bottom = Object::plane(Axis::Y, -25., 25., -25., 25., 0., floor, Emission::Dark);

    let obj = wavefront_obj::load_obj_file_spheres(obj_path, radius).unwrap();
    let obj = Object::from_spheres(obj, mat, Emission::Dark);

    let mut objects = vec![bottom];
    objects.extend(obj);

    (
        Camera::new(
            Vec3::new(0., 5., 10.),
            Vec3::unit_y().as_vec(),
            Vec3::unit_y().as_vec(),
            50.,
            1920. / 500.,
            1080. / 500.,
            100,
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
    let mat = Material::CookTorrance(CookTorrance::new(
        Vec3::ones(),
        0.05,
        Fresnel::SchlickMetallic(Vec3::new(0.722, 0.451, 0.2)),
    ));

    obj_scene(z_near, z_far, hdri, mat, "suzanne.obj")
}

pub fn glass_suzanne(z_near: f64, z_far: f64, hdri: Image) -> (Camera, Scene) {
    let mat = Material::Glass(Glass::new(Vec3::ones() * 0.8, 1.45));
    obj_scene(z_near, z_far, hdri, mat, "suzanne.obj")
}

pub fn sphere_dragon(z_near: f64, z_far: f64, hdri: Image) -> (Camera, Scene) {
    let mat = Material::CookTorrance(CookTorrance::new(
        Vec3::ones(),
        0.02,
        Fresnel::SchlickMetallic(Vec3::new(0.2, 0.4, 0.9)),
    ));
    obj_scene_sphere(z_near, z_far, hdri, mat, "dragon.obj", 0.025)
}

fn multiple_spheres(z_near: f64, z_far: f64, hdri: Image, mats: Vec<Material>) -> (Camera, Scene) {
    let floor = Material::CookTorrance(CookTorrance::new(
        Vec3::ones(),
        0.5,
        Fresnel::SchlickMetallic(Vec3::ones() * 0.8),
    ));

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
            Vec3::unit_y().as_vec(),
            Vec3::unit_y().as_vec(),
            72.,
            1920. / 500.,
            400. / 500.,
            125,
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
        .map(|i| {
            Material::CookTorrance(CookTorrance::new(
                Vec3::ones(),
                0.01 * (4 * i + 1) as f64,
                Fresnel::SchlickMetallic(Vec3::ones() * 0.8),
            ))
        })
        .collect();
    multiple_spheres(z_near, z_far, hdri, mats)
}

pub fn cook_torrance_spheres_plastic(z_near: f64, z_far: f64, hdri: Image) -> (Camera, Scene) {
    let mats = (0..7)
        .map(|i| {
            Material::Plastic(Plastic::new(
                Vec3::ones() * 0.8,
                Vec3::ones(),
                0.01 * (4 * i + 1) as f64,
                1.45,
            ))
        })
        .collect();

    multiple_spheres(z_near, z_far, hdri, mats)
}

pub fn cook_torrance_spheres_frosted_glass(
    z_near: f64,
    z_far: f64,
    hdri: Image,
) -> (Camera, Scene) {
    let mats: Vec<_> = (0..7)
        .map(|i| {
            Material::CookTorranceGlass(CookTorranceGlass::new(
                Vec3::ones(),
                0.01 * (4 * i + 1) as f64,
                1.45,
            ))
        })
        .collect();
    multiple_spheres(z_near, z_far, hdri, mats)
}

pub fn cook_torrance_spheres_cook_torrance_refract(
    z_near: f64,
    z_far: f64,
    hdri: Image,
) -> (Camera, Scene) {
    let mut mats: Vec<_> = (0..6)
        .map(|i| {
            Material::CookTorranceRefract(CookTorranceRefract::new(
                Vec3::ones(),
                0.01 * (4 * i + 1) as f64,
                1.45,
            ))
        })
        .collect();
    mats.insert(0, Material::Refract(Refract::new(Vec3::ones(), 1.45)));
    multiple_spheres(z_near, z_far, hdri, mats)
}

pub fn material_test(z_near: f64, z_far: f64, hdri: Image) -> (Camera, Scene) {
    let mats = vec![
        Material::LambertianDiffuse(LambertianDiffuse::new(Vec3::ones() * 0.8)),
        Material::Plastic(Plastic::new(Vec3::ones() * 0.8, Vec3::ones(), 0.05, 1.45)),
        Material::Reflect(Reflect::new(Vec3::ones() * 0.8)),
        Material::CookTorrance(CookTorrance::new(
            Vec3::ones(),
            0.05,
            Fresnel::SchlickMetallic(Vec3::ones() * 0.8),
        )),
        Material::Glass(Glass::new(Vec3::ones(), 1.45)),
        Material::CookTorranceGlass(CookTorranceGlass::new(Vec3::ones(), 0.05, 1.45)),
        Material::NoReflect,
    ];
    let floor = Material::CookTorrance(CookTorrance::new(
        Vec3::ones(),
        0.5,
        Fresnel::SchlickMetallic(Vec3::ones() * 0.8),
    ));

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
            Vec3::new(0., 3., 20.),
            Vec3::unit_y().as_vec(),
            Vec3::unit_y().as_vec(),
            90.,
            1920. / 500.,
            250. / 500.,
            125,
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
