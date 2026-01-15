use glam::{Vec3, vec3};
use wavefront_obj::obj;

use crate::scene::{Bounds3, Hittable};

/// A triangle
pub struct Triangle {
    /// The triangles three corners, as indices into the associated `vertices` array.
    pub vertices: (usize, usize, usize),
    /// The triangles three normals, as indices into the associated `normals` array.
    pub normals: (usize, usize, usize),
    pub material_id: u32,
}

/// A collection of triangles with
pub struct Mesh {
    vertices: Vec<Vec3>,
    normals: Vec<Vec3>,
    /// The triangles as triplets of pairs of indices into `vertices` and `normals`, giving its
    /// three corner positions and normals.
    triangles: Vec<Triangle>,
}

fn vec3_from_vertex(vertex: &obj::Vertex) -> Vec3 {
    vec3(vertex.x as f32, vertex.y as f32, vertex.z as f32)
}

impl Mesh {
    pub fn load(src: impl AsRef<str>, material_id: u32) -> anyhow::Result<Self> {
        let obj = obj::parse(src)?;

        let mut vertices = Vec::new();
        let mut normals = Vec::new();
        let mut triangles = Vec::<Triangle>::new();

        for object in obj.objects {
            let vertices_offset = vertices.len();
            let normals_offset = normals.len();
            vertices.extend(
                object
                    .vertices
                    .iter()
                    .map(|vertex| vec3_from_vertex(vertex)),
            );
            normals.extend(object.normals.iter().map(|normal| vec3_from_vertex(normal)));

            for geometry in object.geometry {
                println!("Material: {:?}", geometry.material_name);
                triangles.extend(geometry.shapes.into_iter().filter_map(|shape| {
                    match shape.primitive {
                        obj::Primitive::Triangle(a, b, c) => {
                            Some(Triangle {
                                vertices: (
                                    vertices_offset + a.0,
                                    vertices_offset + b.0,
                                    vertices_offset + c.0,
                                ),
                                normals: (
                                    normals_offset
                                        + a.2.expect("Object must include baked normals"),
                                    normals_offset
                                        + b.2.expect("Object must include baked normals"),
                                    normals_offset
                                        + c.2.expect("Object must include baked normals"),
                                ),
                                // TODO: Support loading materials from file.
                                material_id,
                            })
                        }
                        _ => None,
                    }
                }));
            }
        }

        Ok(Self {
            vertices,
            normals,
            triangles,
        })
    }
}

#[derive(Debug, Clone)]
pub struct PackedMeshes {
    pub vertices: Vec<Vec3>,
    pub normals: Vec<Vec3>,
    pub triangles: Vec<TriangleUniform>,
}

impl PackedMeshes {
    pub fn pack_meshes(meshes: &[Mesh]) -> Self {
        let mut vertices = Vec::new();
        let mut normals = Vec::new();
        let mut triangles = Vec::new();

        for mesh in meshes {
            let vertex_offset = vertices.len();
            let normal_offset = normals.len();
            triangles.extend(mesh.triangles.iter().map(|triangle| {
                TriangleUniform::with_offset(triangle, vertex_offset, normal_offset)
            }));

            vertices.extend(mesh.vertices.iter().map(|x| *x));
            normals.extend(mesh.normals.iter().map(|x| *x));
        }

        Self {
            vertices,
            normals,
            triangles,
        }
    }

    pub fn vertices(&self) -> &[Vec3] {
        &self.vertices
    }

    pub fn normals(&self) -> &[Vec3] {
        &self.normals
    }

    pub fn triangles(&self) -> &[TriangleUniform] {
        &self.triangles
    }

    pub fn hittable_triangles(&self) -> impl IntoIterator<Item = HittableTriangle> {
        self.triangles.iter().map(|triangle| HittableTriangle {
            vertices: (
                self.vertices[triangle.vertex_0 as usize],
                self.vertices[triangle.vertex_1 as usize],
                self.vertices[triangle.vertex_2 as usize],
            ),
        })
    }
}

#[derive(Debug, Clone)]
pub struct HittableTriangle {
    pub vertices: (Vec3, Vec3, Vec3),
}

impl Hittable for HittableTriangle {
    fn bounds(&self) -> Bounds3 {
        Bounds3::from_points([self.vertices.0, self.vertices.1, self.vertices.2])
    }
}

/// A triangle.
#[derive(Debug, Clone, encase::ShaderType)]
pub struct TriangleUniform {
    /// The triangle's 1st corner, as index into the associated `vertices` array.
    pub vertex_0: u32,
    /// The triangle's 2st corner, as index into the associated `vertices` array.
    pub vertex_1: u32,
    /// The triangle's 3st corner, as index into the associated `vertices` array.
    pub vertex_2: u32,
    /// The triangle's 1st corner's normal, as index into the associated `normals` array.
    pub normal_0: u32,
    /// The triangle's 2st corner's normal, as index into the associated `normals` array.
    pub normal_1: u32,
    /// The triangle's 3st corner's normal, as index into the associated `normals` array.
    pub normal_2: u32,
    pub material_id: u32,
}

impl TriangleUniform {
    fn with_offset(triangle: &Triangle, vertex_offset: usize, normal_offset: usize) -> Self {
        Self {
            vertex_0: (triangle.vertices.0 + vertex_offset) as u32,
            vertex_1: (triangle.vertices.1 + vertex_offset) as u32,
            vertex_2: (triangle.vertices.2 + vertex_offset) as u32,
            normal_0: (triangle.normals.0 + normal_offset) as u32,
            normal_1: (triangle.normals.1 + normal_offset) as u32,
            normal_2: (triangle.normals.2 + normal_offset) as u32,
            material_id: triangle.material_id,
        }
    }
}
