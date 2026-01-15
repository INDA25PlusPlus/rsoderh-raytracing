use std::{
    fs,
    ops::{self, Index},
    path::{Path, PathBuf},
};

use anyhow::anyhow;
use cgmath::Deg;
use glam::{Mat3, Vec3, vec3};

use crate::{
    camera::Camera,
    mesh::{Mesh, PackedMeshes},
};

#[derive(Debug, Clone, encase::ShaderType)]
pub struct Material {
    // The albedo for diffuse materials, and metal reflectance for metals.
    pub color: Vec3,
    pub roughness: f32,
    pub metallic: f32,
    pub emission: Vec3,
}

#[derive(Debug, Clone, Copy)]
pub enum Axis {
    X,
    Y,
    Z,
}

impl From<u8> for Axis {
    fn from(value: u8) -> Self {
        match value % 3 {
            0 => Axis::X,
            1 => Axis::Y,
            2 => Axis::Z,
            _ => unreachable!(),
        }
    }
}

impl From<Axis> for u8 {
    fn from(value: Axis) -> Self {
        match value {
            Axis::X => 0,
            Axis::Y => 1,
            Axis::Z => 2,
        }
    }
}

impl Index<Axis> for Vec3 {
    type Output = f32;
    fn index(&self, index: Axis) -> &Self::Output {
        self.index(u8::from(index) as usize)
    }
}

#[derive(Debug, Clone, Copy, Default, encase::ShaderType)]
pub struct Bounds3 {
    pub min: Vec3,
    pub max: Vec3,
}

impl Bounds3 {
    pub const fn identity() -> Self {
        Self {
            min: Vec3::MAX,
            max: Vec3::MIN,
        }
    }

    /// Constructs bounding box containing the points.
    pub fn from_points(points: impl IntoIterator<Item = Vec3>) -> Self {
        let (min, max) = points
            .into_iter()
            .fold((Vec3::MAX, Vec3::MIN), |(min, max), point| {
                (min.min(point), max.max(point))
            });
        assert!(min != Vec3::MAX);
        assert!(max != Vec3::MIN);

        Bounds3 { min, max }
    }

    /// Constructs bounding box containing all given bounds.
    pub fn from_bounds(bounds: impl IntoIterator<Item = Bounds3>) -> Self {
        let result = bounds.into_iter().fold(
            Bounds3 {
                min: Vec3::MAX,
                max: Vec3::MIN,
            },
            |accumulator, bounds| &accumulator | &bounds,
        );
        assert!(result.min != Vec3::MAX);
        assert!(result.max != Vec3::MIN);

        result
    }

    pub fn center(&self) -> Vec3 {
        self.min * 0.5 + self.max * 0.5
    }

    /// Transforms point so that `self.min` maps to `vec3(0., 0., 0.)` and `self.max` maps to
    /// `vec3(1., 1., 1.)`.
    pub fn to_relative(&self, point: Vec3) -> Vec3 {
        point - self.min / (self.max - self.min)
    }

    /// Returns the axis along which self is the longest.
    pub fn max_axis(&self) -> Axis {
        let diagonal = self.max - self.min;
        if diagonal.z > diagonal.x && diagonal.z > diagonal.y {
            Axis::Z
        } else if diagonal.y > diagonal.x {
            Axis::Y
        } else {
            Axis::X
        }
    }

    /// Returns the surface area of the bound box's six faces.
    pub fn surface_area(&self) -> f32 {
        let diagonal = self.max - self.min;
        2. * (diagonal.x * diagonal.y + diagonal.x * diagonal.z + diagonal.y * diagonal.z)
    }
}

impl ops::BitOr<&Bounds3> for &Bounds3 {
    type Output = Bounds3;

    /// Returns the union of `self` and `rhs`.
    fn bitor(self, rhs: &Bounds3) -> Self::Output {
        Bounds3 {
            min: self.min.min(rhs.min),
            max: self.max.max(rhs.max),
        }
    }
}

pub trait Hittable: HittableClone {
    fn bounds(&self) -> Bounds3;
}

pub trait HittableClone {
    fn clone_box(&self) -> Box<dyn Hittable>;
}

impl<T> HittableClone for T
where
    T: 'static + Hittable + Clone,
{
    fn clone_box(&self) -> Box<dyn Hittable> {
        Box::new(self.clone())
    }
}

impl Clone for Box<dyn Hittable> {
    fn clone(&self) -> Self {
        self.clone_box()
    }
}

#[derive(Debug, Clone, encase::ShaderType)]
pub struct Sphere {
    pub pos: Vec3,
    pub radius: f32,
    pub material_id: u32,
}

impl Hittable for Sphere {
    fn bounds(&self) -> Bounds3 {
        Bounds3 {
            min: self.pos - vec3(self.radius, self.radius, self.radius),
            max: self.pos + vec3(self.radius, self.radius, self.radius),
        }
    }
}

#[derive(Debug, Clone)]
pub struct Plane {
    pub pos: Vec3,
    pub forward: Vec3,
    pub right: Vec3,
    pub material_id: u32,
}

impl Plane {
    pub fn to_uniform(&self) -> UniformPlane {
        let normal = self.forward.cross(self.right).normalize();

        UniformPlane {
            pos: self.pos,
            normal,
            base_change_matrix: Mat3::from_cols(self.right, normal, self.forward).inverse(),
            material_id: self.material_id,
        }
    }
}

impl Hittable for Plane {
    fn bounds(&self) -> Bounds3 {
        Bounds3::from_points([self.pos, self.pos + self.forward + self.right])
    }
}

/// The plane data sent to the GPU.
#[derive(Debug, Clone, encase::ShaderType)]
pub struct UniformPlane {
    pub pos: Vec3,
    pub normal: Vec3,
    /// Let
    /// - plane be the original plane
    /// - normal = plane.forward × plane.right
    /// - B be the base {plane.right, normal, plane.forward}
    ///
    /// The this is the change-of-basis matrix which converts between the standard base and B.
    pub base_change_matrix: Mat3,
    pub material_id: u32,
}

#[derive(Debug, Clone)]
pub struct Scene {
    pub materials: Vec<Material>,
    pub spheres: Vec<Sphere>,
    pub planes: Vec<Plane>,
    pub meshes: PackedMeshes,
    pub camera: Camera,
}

impl Scene {
    /// Load scene from TOML file containing scene description.
    pub fn load_toml(path: impl AsRef<Path>) -> anyhow::Result<Self> {
        let content = fs::read_to_string(&path).map_err(|err| {
            anyhow!(
                "Couldn't open scene {}:\n  {}",
                path.as_ref().to_string_lossy(),
                err
            )
        })?;
        let scene_descr: SceneDescriptor = toml::from_str(&content).map_err(|err| {
            anyhow!(
                "Couldn't parse scene {}:\n  {}",
                path.as_ref().to_string_lossy(),
                err
            )
        })?;

        scene_descr.build_scene(path.as_ref())
    }
}

#[derive(Hash)]
pub struct SceneState {
    pub camera: Camera,
    pub environment_index: u32,
    /// Runtime configurable number in 0..=9. Can be used for any kind of quick temporary
    /// configuration during development. Is changed via the number keys.
    pub dev_index: u32,
}

#[derive(Debug, serde::Deserialize)]
struct MaterialDescriptor {
    name: String,
    color: Vec3,
    roughness: f32,
    metallic: f32,
    emission: Vec3,
}

#[derive(Debug, serde::Deserialize)]
enum ObjectDescriptor {
    Sphere {
        /// Material name
        material: String,
        pos: Vec3,
        radius: f32,
    },
    Plane {
        /// Material name
        material: String,
        pos: Vec3,
        forward: Vec3,
        right: Vec3,
    },
    Mesh {
        /// Material name
        material: String,
        /// Path to OBJ file, relative to TOML file.
        path: PathBuf,
    },
}

#[derive(Debug, Clone, serde::Deserialize)]
struct CameraDescriptor {
    pub pos: Vec3,
    pub yaw: Deg<f32>,
    pub pitch: Deg<f32>,
    /// The cameras vertical fov. The horizontal fov is calculated to match in the shader.
    pub fov_y: Deg<f32>,
}

impl From<CameraDescriptor> for Camera {
    fn from(value: CameraDescriptor) -> Self {
        Self {
            pos: value.pos,
            yaw: value.yaw.into(),
            pitch: value.pitch.into(),
            fov_y: value.fov_y.into(),
        }
    }
}

#[derive(Debug, serde::Deserialize)]
struct SceneDescriptor {
    material: Vec<MaterialDescriptor>,
    object: Vec<ObjectDescriptor>,
    /// Initial camera position.
    camera: CameraDescriptor,
}

impl SceneDescriptor {
    fn build_scene(self, descriptor_path: &Path) -> anyhow::Result<Scene> {
        let get_material_index = |material_name: &str| -> Option<u32> {
            self.material
                .iter()
                .enumerate()
                .find(|(_, material)| material.name == material_name)
                .map(|(index, _)| index as u32)
        };

        let create_object_error = |index: usize, type_: &str, msg: &str| -> anyhow::Error {
            anyhow!(
                "Error in object {} ({}): {}\n  --> {}",
                index,
                type_,
                msg,
                descriptor_path.to_string_lossy(),
            )
        };

        let create_material_error =
            |index: usize, type_: &str, material_name: &str| -> anyhow::Error {
                create_object_error(
                    index,
                    type_,
                    &format!("Material '{}' does not exist.", material_name),
                )
            };

        let materials = self
            .material
            .iter()
            .map(|descr| Material {
                color: descr.color,
                roughness: descr.roughness,
                metallic: descr.metallic,
                emission: descr.emission,
            })
            .collect::<Vec<_>>();

        let mut spheres = Vec::new();
        let mut planes = Vec::new();
        let mut meshes = Vec::new();

        for (i, object) in self.object.into_iter().enumerate() {
            match object {
                ObjectDescriptor::Sphere {
                    material,
                    pos,
                    radius,
                } => {
                    let Some(material_id) = get_material_index(&material) else {
                        return Err(create_material_error(i, "Sphere", &material));
                    };

                    spheres.push(Sphere {
                        pos,
                        radius,
                        material_id,
                    });
                }
                ObjectDescriptor::Plane {
                    material,
                    pos,
                    forward,
                    right,
                } => {
                    let Some(material_id) = get_material_index(&material) else {
                        return Err(create_material_error(i, "Plane", &material));
                    };

                    planes.push(Plane {
                        pos,
                        forward,
                        right,
                        material_id,
                    });
                }
                ObjectDescriptor::Mesh { material, path } => {
                    let Some(material_id) = get_material_index(&material) else {
                        return Err(create_material_error(i, "Mesh", &material));
                    };

                    let content = fs::read_to_string(
                        descriptor_path
                            .parent()
                            .unwrap_or(&PathBuf::from("."))
                            .join(&path),
                    )
                    .map_err(|err| {
                        create_object_error(
                            i,
                            "Mesh",
                            &format!(
                                "Cannot open '{}': {}",
                                path.to_string_lossy(),
                                err.to_string(),
                            ),
                        )
                    })?;

                    meshes.push(
                        Mesh::load(content, material_id)
                            .map_err(|err| create_object_error(i, "Mesh", &err.to_string()))?,
                    );
                }
            }
        }

        Ok(Scene {
            materials: materials,
            spheres,
            planes,
            meshes: PackedMeshes::pack_meshes(&meshes),
            camera: self.camera.into(),
        })
    }
}
