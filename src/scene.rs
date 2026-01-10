use std::ops::{self, Index};

use glam::{Mat3, Vec3, vec3};

use crate::{camera::Camera, mesh::PackedMeshes};

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

pub struct Scene {
    pub materials: Vec<Material>,
    pub spheres: Vec<Sphere>,
    pub planes: Vec<Plane>,
    pub meshes: PackedMeshes,
    pub camera: Camera,
}

#[derive(Hash)]
pub struct SceneState {
    pub camera: Camera,
    pub environment_index: u32,
    /// Runtime configurable number in 0..=9. Can be used for any kind of quick temporary
    /// configuration during development. Is changed via the number keys.
    pub dev_index: u32,
}
