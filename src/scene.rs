use glam::{Mat3, Vec3};

use crate::camera::Camera;

#[derive(Debug, Clone, encase::ShaderType)]
pub struct Material {
    pub color: Vec3,
    pub roughness: f32,
    pub emission_strength: f32,
}

#[derive(Debug, Clone, encase::ShaderType)]
pub struct Sphere {
    pub pos: Vec3,
    pub radius: f32,
    pub material_id: u32,
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
        let normal = self.forward.cross(self.right);

        UniformPlane {
            pos: self.pos,
            normal,
            base_change_matrix: Mat3::from_cols(self.right, normal, self.forward).inverse(),
            material_id: self.material_id,
        }
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

pub struct Scene {
    pub materials: Vec<Material>,
    pub spheres: Vec<Sphere>,
    pub planes: Vec<Plane>,
    pub camera: Camera,
}
