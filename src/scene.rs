use glam::Vec3;

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

pub struct Scene {
    pub materials: Vec<Material>,
    pub spheres: Vec<Sphere>,
    pub camera: Camera,
}
