struct Camera {
    pos: vec3<f32>,
    rot_transform: mat3x3<f32>,
    /// The vertical fov, in radians.
    fov_y: f32,
}

struct Ray {
    origin: vec3<f32>,
    /// Must be normalized.
    direction: vec3<f32>,
}

struct Bounds3 {
    min: vec3<f32>,
    max: vec3<f32>,
}

struct HitInfo {
    did_hit: bool,
    distance: f32,
    hit_point: vec3<f32>,
    normal: vec3<f32>,
    material_id: u32,
}

const NO_HIT = HitInfo(
    false,
    0.0,
    vec3<f32>(0.0),
    vec3<f32>(0.0),
    0u,
);

struct BsdfSample {
    /// The reflected camera view direction. If this is the zero vector, some
    /// error occurred while this sample was calculated.
    ray_direction: vec3<f32>,
    /// How much incoming radiance from `ray_direction` is scattered in the
    /// outgoing direction.
    scattering: vec3<f32>,
    /// I assume that this is the probability density that the `wi_world`
    /// would've been chosen.
    pdf: f32,
}

/// Represents a local coordinate space with the normal of a surface point
/// being the forward direction (z+).
struct Frame {
    tangent: vec3<f32>,
    bitangent: vec3<f32>,
    normal: vec3<f32>,
}

fn make_frame(normal: vec3<f32>) -> Frame {    
    // Pick a helper axis that is not parallel to normal.
    let helper = select(
        vec3<f32>(1,0,0),
        vec3<f32>(0,0,1),
        abs(normal.z) < 0.999
    );
    
    let tangent = normalize(cross(helper, normal));
    let bitangent = cross(normal, tangent);
    
    return Frame(tangent, bitangent, normal);
}

fn to_frame_local(frame: Frame, vec_world: vec3<f32>) -> vec3<f32> {
    // Return the components in the basis.
    return vec3<f32>(
        dot(vec_world, frame.tangent),
        dot(vec_world, frame.bitangent),
        dot(vec_world, frame.normal),
    );
}

fn to_frame_world(frame: Frame, vec_local: vec3<f32>) -> vec3<f32> {
    return normalize(
        frame.tangent * vec_local.x
        + frame.bitangent * vec_local.y
        + frame.normal * vec_local.z
    );
}

struct Material {
    color: vec3<f32>,
    roughness: f32,
    metallic: f32,
    emission: vec3<f32>,
}

struct Sphere {
    pos: vec3<f32>,
    radius: f32,
    material_id: u32,
}

struct Plane {
    pos: vec3<f32>,
    normal: vec3<f32>,
    /// Let
    /// - plane be the original plane
    /// - normal = plane.forward × plane.right
    /// - B be the base {plane.right, normal, plane.forward}
    ///
    /// The this is the change-of-basis matrix which converts between the standard base and B.
    base_change_matrix: mat3x3<f32>,
    material_id: u32,
}

struct Triangle {
    /// The triangle's 1st corner, as index into `vertices`.
    vertex_0: u32,
    /// The triangle's 2st corner, as index into `vertices`.
    vertex_1: u32,
    /// The triangle's 3st corner, as index into `vertices`.
    vertex_2: u32,
    /// The triangle's 1st corner's normal, as index into `normals`.
    normal_0: u32,
    /// The triangle's 2st corner's normal, as index into `normals`.
    normal_1: u32,
    /// The triangle's 3st corner's normal, as index into `normals`.
    normal_2: u32,
    material_id: u32,
}

struct PrimitiveInfo {
    /// Which object array this primitive is in. 0 for spheres and 1 for planes.
    primitive_type: u32,
    /// The index of this primitive in its associated array.
    index: u32,
}

struct BvhNode {
    bounds: Bounds3,
    /// If leaf node, stores the index its primitives start at in the primitives
    /// array.
    /// If interior node, stores the index of the second child node. The first
    /// child node's index is equal to this node's index + 1.
    primitives_or_second_child_index: u32,
    /// The amount of primitives contained in this node if it's a leaf node,
    /// otherwise 0.
    primitives_len: u32,
    split_axis: u32,
}

// Environment types

struct EnvironmentMetadata {
    width: u32,
    height: u32,
    /// Index in `environment_alias_maps` that this environment's alias table
    /// starts at. Its length is calculated as `self.width * self.height`.
    alias_table_start_index: u32,
}

struct AliasEntry {
    /// Probability threshold that `alias_index` is picked over this entry's
    /// index. Is in [0,1).
    probability: f32,
    /// Index which maps to a pixel on the environment texture, the "alias"
    /// which is activated if the threshold probability is reached.
    alias_index: u32,
    /// The entry's value of the Probability Mass Function. (Basically a PDF but
    /// for discreet functions.)
    pmf: f32,
    /// Keep struct 16-bytes aligned.
    _pad: u32,
}

@group(0) @binding(0)
var out_texture: texture_storage_2d<rgba16float, read_write>;

@group(0) @binding(1)
var cumulative_light_texture: texture_storage_2d<rgba32float, read_write>;

@group(0) @binding(2)
var environment_sampler: sampler;

@group(0) @binding(3)
var environment_textures: binding_array<texture_2d<f32>>;

@group(0) @binding(4)
var<storage> environments: array<EnvironmentMetadata>;

@group(0) @binding(5)
var<storage> environment_alias_tables: array<AliasEntry>;

@group(1) @binding(0)
var<uniform> camera: Camera;

@group(1) @binding(1)
var<uniform> resolution: vec2<u32>;

@group(1) @binding(2)
var<uniform> time_secs: f32;

@group(1) @binding(3)
var<uniform> sample_count: u32;

@group(1) @binding(4)
var<uniform> environment_map_index: u32;

@group(1) @binding(5)
var<uniform> dev_index: u32;

@group(2) @binding(0)
var<storage, read> materials: array<Material>;

@group(2) @binding(1)
var<storage, read> spheres: array<Sphere>;

@group(2) @binding(2)
var<storage, read> planes: array<Plane>;

@group(2) @binding(3)
var<storage, read> vertices: array<vec3<f32>>;

@group(2) @binding(4)
var<storage, read> normals: array<vec3<f32>>;

@group(2) @binding(5)
var<storage, read> triangles: array<Triangle>;

@group(2) @binding(6)
var<storage, read> primitives: array<PrimitiveInfo>;

@group(2) @binding(7)
var<storage, read> bvh_nodes: array<BvhNode>;

const MAX_BOUNCES: u32 = 10;

// Largest representable f32 (actual IEEE infinity can't be used).
const INFINITY = 1.70141183460469231732e+38f;
// Smallest representable f32 (actual IEEE negative infinity can't be used).
const NEG_INFINITY = -1.70141183460469231732e+38f;

const PI = 3.14159;
const INV_PI = 1. / PI;

fn lerp_vec3f(a: vec3<f32>, b: vec3<f32>, t: f32) -> vec3<f32> {
    return (1.0 - t) * a + t * b;
}

fn lerp_f32(a: f32, b: f32, t: f32) -> f32 {
    return (1.0 - t) * a + t * b;
}

/// Returns value equivalent to `length(v) * length(v)`.
fn length_squared(v: vec3<f32>) -> f32 {
    return dot(v, v);
}

/// Returns the largest component of `v`.
fn max_component(v: vec3<f32>) -> f32 {
    return max(v.x, max(v.y, v.z));
}

/// Checks if `ray` intersects the given `bounds`.
/// `inv_direction` should be equal to `1. / ray.direction`.
fn ray_intersects_bounds(
    ray: Ray,
    bounds: Bounds3,
    inv_direction: vec3<f32>
) -> bool {
    var t_0 = 0.;
    var t_1 = INFINITY;
    for (var axis = 0u; axis < 3; axis++) {
        // Intersect with the two faces along `axis`.
        
        var t_near = (bounds.min[axis] - ray.origin[axis]) * inv_direction[axis];
        var t_far = (bounds.max[axis] - ray.origin[axis]) * inv_direction[axis];
        
        if t_near > t_far {
            let t_near_old = t_near;
            t_near = t_far;
            t_far = t_near_old;
        }
        
        if t_near > t_0 {
            t_0 = t_near;
        }
        if t_far < t_1 {
            t_1 = t_far;
        }
        if t_0 > t_1 {
            return false;
        }
    }
    
    return true;
}

fn cast_ray_sphere(ray: Ray, sphere: Sphere) -> HitInfo {
    const EPSILON = 1.0e-4;
    
    let l = ray.origin - sphere.pos;
    let a = dot(ray.direction, ray.direction);
    let b = 2 * dot(ray.direction, l);
    let c = dot(l, l) - sphere.radius * sphere.radius;
    
    // Solve at^2 + bt + c = 0
    var t: f32;
    let discriminant = b * b - 4 * a * c;
    if discriminant < 0 {
        return NO_HIT;
    } else if discriminant == 0 {
        t = -0.5 * b / a;
    } else {
        let q = select(
            -0.5 * (b - sqrt(discriminant)),
            -0.5 * (b + sqrt(discriminant)),
            b > 0,
        );
        let t_0 = q / a;
        let t_1 = c / q;
        
        // We ignore collisions from rays that start very close to the surface
        // to address shadow acne.
        
        if t_0 < EPSILON {
            t = t_1;
        } else if t_1 < EPSILON {
            t = t_0;
        } else {
            t = min(t_0, t_1);
        }
    }
    
    if t < EPSILON {
        return NO_HIT;
    }

    let hit_point = ray.origin + ray.direction * t;
    
    var normal = normalize(hit_point - sphere.pos);
    
    // We consider origins that are slightly outside the sphere as within it to
    // correctly reflect rays bouncing in sphere interior.
    // TODO: This causes strange acne-like noise occurring when the camera is
    //   relatively far away from the surface, presumably because some of the
    //   reflected rays end up intersecting with the same surface, and are then
    //   considered as having originated inside the sphere, being reflected into
    //   it.
    if length_squared(sphere.pos - ray.origin) - sphere.radius * sphere.radius
        < 1.0e-6
    {
        // Ray originates within sphere, flip normal.
        normal *= -1.;
    }
    
    return HitInfo(
        true,
        t,
        hit_point,
        normal,
        sphere.material_id,
    );
}

fn cast_ray_plane(ray: Ray, plane: Plane) -> HitInfo {
    let denominator = dot(plane.normal, ray.direction);
    if (abs(denominator) < 0.0001) {
        // The ray is parallel to the plane
        return NO_HIT;
    }
    let t = dot(plane.normal, plane.pos - ray.origin) / denominator;
    // Ignore collisions that are very close to surface. This addresses shadow
    // acne.
    if (t < 0.001) {
        // Intersection point is behind ray
        return NO_HIT;
    }
    
    // Point on plane the ray would intersect with if the plane extended
    // infinitely
    let inter = ray.origin + ray.direction * t;
        
    // `inter`, but relative to the plane's position.
    let inter_local = inter - plane.pos;
    // The point in the "plane's base".
    let inter_plane_space = plane.base_change_matrix * inter_local;
    
    if inter_plane_space.x < 0.
        || 1. < inter_plane_space.x
        || inter_plane_space.z < 0.
        || 1. < inter_plane_space.z
    {
        return NO_HIT;
    }
    
    var normal = plane.normal;
    if dot(ray.origin, normal) < 0. {
        // Ray originates below plane, flip normal.
        normal *= -1;
    }
    
    return HitInfo(
        true,
        t,
        inter,
        normal,
        plane.material_id,
    );
}

/// Algorithm provided by ChatGPT. :)
fn cast_ray_triangle(ray: Ray, triangle: Triangle) -> HitInfo {
    let a = vertices[triangle.vertex_0];
    let b = vertices[triangle.vertex_1];
    let c = vertices[triangle.vertex_2];
    
    let pos = a;
    let edge_0 = b - a;
    let edge_1 = c - a;
    
    let perp_to_edge_0 = cross(ray.origin - pos, edge_0);
    let perp_to_edge_1 = cross(ray.direction, edge_1);
    let determinant = dot(edge_0, perp_to_edge_1);
    let inverse_determinant = 1. / determinant;
    
    if abs(determinant) < 1.0e-8 {
        // Ray is parallel to triangle.
        return NO_HIT;
    }
    
    // Barycentric coordinates
    let u = dot(ray.origin - pos, perp_to_edge_1) * inverse_determinant;
    let v = dot(ray.direction, perp_to_edge_0) * inverse_determinant;
    
    if u < 0. || 1. < u {
        return NO_HIT;
    }
    if v < 0. || 1. < (u + v) {
        return NO_HIT;
    }
    
    var t = dot(edge_1, perp_to_edge_0) * inverse_determinant;
    // Ignore collisions which originate very close to surface
    if t < 1.0e-5 {
        // Intersection is behind the ray origin.
        return NO_HIT;
    }
    
    // Interpolate vertex normals
    let normal_0 = normals[triangle.normal_0];
    let normal_1 = normals[triangle.normal_1];
    let normal_2 = normals[triangle.normal_2];
    
    var normal =
        normalize((1. - u - v) * normal_0 + u * normal_1 + v * normal_2);
    
    if dot(normal, ray.direction) > 0. {
        // Ray hit the triangle's back side, flip normal.
        normal *= -1.;
    }
    
    return HitInfo(
        true,
        t,
        ray.origin + ray.direction * t,
        normal,
        triangle.material_id,
    );
}


fn cast_ray_bvh(ray: Ray) -> HitInfo {
    let ray_inv_direction = 1. / ray.direction;
    
    var result = HitInfo(
        false,
        INFINITY,
        vec3<f32>(0.0),
        vec3<f32>(0.0),
        0u,
    );
    
    var hit = false;
    // Stack of indices of nodes to test intersection with.
    var nodes_to_visit: array<u32, 64>;
    var stack_length = 0u;
    
    var current_node_index = 0u;
    
    loop {
        let node = bvh_nodes[current_node_index];
        if ray_intersects_bounds(ray, node.bounds, ray_inv_direction) {
            if node.primitives_len > 0 {
                // `node` is a leaf node.
                
                for (var i = 0u; i < node.primitives_len; i++) {
                    let info = primitives[node.primitives_or_second_child_index + i];
                    switch info.primitive_type {
                        case 0 {
                            let sphere = spheres[info.index];
                            
                            let info = cast_ray_sphere(ray, sphere);
                            if info.did_hit && info.distance < result.distance {
                                result = info;
                            }
                        }
                        case 1 {
                            let plane = planes[info.index];
                            
                            let info = cast_ray_plane(ray, plane);
                            if info.did_hit && info.distance < result.distance {
                                result = info;
                            }
                        }
                        case 2 {
                            let triangle = triangles[info.index];
                            
                            let info = cast_ray_triangle(ray, triangle);
                            if info.did_hit && info.distance < result.distance {
                                result = info;
                            }
                        }
                        default {
                            // Unreachable (surely)
                        }
                    }
                }
                
                if stack_length == 0 {
                    // We have walked through all relevant nodes.
                    break;
                }
                stack_length--;
                current_node_index = nodes_to_visit[stack_length];
            } else {
                // `node` is an interior node.
                
                // No idea why this works/is necessary. :)
                if ray_inv_direction[node.split_axis] < 0. {
                    // Visit second child first.
                    nodes_to_visit[stack_length] = current_node_index + 1;
                    stack_length++;
                    current_node_index = node.primitives_or_second_child_index;
                } else {
                    // Visit first child to stack.
                    nodes_to_visit[stack_length] =
                        node.primitives_or_second_child_index;
                    stack_length++;
                    current_node_index = current_node_index + 1;
                }
            }
        } else {
            if stack_length == 0 {
                // We have walked through all relevant nodes.
                break;
            }
            stack_length--;
            current_node_index = nodes_to_visit[stack_length];
        }
    }
    
    if !result.did_hit {
        return NO_HIT;
    }
    
    return result;
}

// Cast ray through scene, returning the surface if one was hit.
fn cast_ray(ray: Ray) -> HitInfo {
    var result = HitInfo(
        false,
        INFINITY,
        vec3<f32>(0.0),
        vec3<f32>(0.0),
        0u,
    );
    
    {
        let result = cast_ray_bvh(ray);
        if result.did_hit {
            return result;
        }
    }
    
    for (var i: u32 = 0; i < arrayLength(&spheres); i++) {
        let sphere = spheres[i];
        
        let info = cast_ray_sphere(ray, sphere);
        if info.did_hit && info.distance < result.distance {
            result = info;
        }
    }
    for (var i: u32 = 0; i < arrayLength(&planes); i++) {
        let plane = planes[i];
        
        let info = cast_ray_plane(ray, plane);
        if info.did_hit && info.distance < result.distance {
            result = info;
        }
    }
    
    return result;
}

// Random number generator

fn salt_rng(rng_state: ptr<function, u32>, salt: u32) {
    *rng_state = *rng_state ^ salt;
    // Mix RNG state
    random_u32_uniform(rng_state);
}

fn random_u32_uniform(rng_state: ptr<function, u32>) -> u32 {
    *rng_state = *rng_state * 747796405 + 2891336453;
    var result = ((*rng_state >> ((*rng_state >> 28) + 4)) ^ *rng_state)
        * 277803737;
    result = (result >> 22) ^ result;
    return result;
}

// Returns a random float in the range [0,1), updating the RNG state in the
// process.
fn random_uniform(rng_state: ptr<function, u32>) -> f32 {
    return f32(random_u32_uniform(rng_state)) / 4294967295.0;
}

// Returns a uniformly distributed random point on the unit circle. The point's
// length is in the range [0,1).
fn random_in_circle_uniform(rng_state: ptr<function, u32>) -> vec2<f32> {
    let angle: f32 = random_uniform(rng_state) * 2 * 3.1415926;
    let pointOnCircle = vec2(cos(angle), sin(angle));
    return pointOnCircle * sqrt(random_uniform(rng_state));
}

// Returns a uniformly distributed random point on the unit sphere. The point's
// length is in the range [0,1).
// 
// Algorithm generated by ChatGPT. :)
// 
// Apparently the distribution is uniform if longitude and cos(latitude) are
// uniformly distributed.
fn random_in_sphere_uniform(rng_state: ptr<function, u32>) -> vec3<f32> {
    let u = random_uniform(rng_state);
    let v = random_uniform(rng_state);
    
    let longitude = 2. * PI * u;
    // This is equal to cos(latitude).
    let z = 2. * v - 1.;
    
    let r = sqrt(1. - z * z);
    
    let x = r * cos(longitude);
    let y = r * sin(longitude);
    
    return vec3(x, y, z);
}

// Returns a uniformly distributed random point on the unit sphere's hemisphere
// defined by the given normal. The point's length is in the range [0,1).
fn random_in_hemisphere_uniform(normal: vec3<f32>, rng_state: ptr<function, u32>) -> vec3<f32> {
    let point = random_in_sphere_uniform(rng_state);
    return point * sign(dot(normal, point));
}

// Environment sampling

/// Returns the alias entry with the given index from the given environments
/// alias table.
fn environment_alias_entry(index: u32, environment_index: u32) -> AliasEntry {
    let absolute_index =
        environments[environment_index].alias_table_start_index + index;
    return environment_alias_tables[absolute_index];
}

fn environment_direction_alias_entry(
    direction: vec3<f32>,
    environment_index: u32
) -> AliasEntry {
    let metadata = environments[environment_index];
    
    let uv = direction_to_equirectangular_uv(direction);
    let x = min(u32(uv.x * f32(metadata.width)), metadata.width - 1);
    let y = min(u32(uv.y * f32(metadata.height)), metadata.height - 1);
    let index = x + y * metadata.width;
    
    return environment_alias_entry(index, environment_index);
}

/// Sample random index according to the distribution defined by the selected
/// environment's alias table.
fn random_index_in_environment(
    environment_index: u32,
    rng_state: ptr<function, u32>,
) -> u32 {
    let metadata = environments[environment_index];
    // Length of the alias table.
    let length = metadata.width
        * metadata.height;
    
    let index = min(u32(random_uniform(rng_state) * f32(length)), length - 1);
    let entry = environment_alias_entry(index, environment_index);

    return select(
        entry.alias_index,
        index,
        random_uniform(rng_state) < entry.probability,
    );
}

/// Returns the UV-coordinates on an equirectangular texture which maps to the
/// given direction. `direction` must be normalized!
fn direction_to_equirectangular_uv(direction: vec3<f32>) -> vec2<f32> {
    let u = atan2(direction.z, direction.x) * INV_PI * 0.5 + 0.5;
    let v = 0.5 - asin(direction.y) * INV_PI;
    return vec2<f32>(u, v);
}

/// Returns the direction which maps to the given UV-coordinates of an
/// equirectangular texture.
fn equirectangular_uv_to_direction(uv: vec2<f32>) -> vec3<f32> {
    // Horizontal angle.
    let phi = (2. * uv.x - 1.) * PI;
    // Vertical angle.
    let theta = PI * uv.y;
    
    let sin_theta = sin(theta);
    let cos_theta = cos(theta);
    
    return vec3<f32>(
        sin_theta * cos(phi),
        cos_theta,
        sin_theta * sin(phi),
    );
}

// Calculates the approximate solid angle of a single lat-long pixel in the
// given environment's texture at v, i.e. the "vertical angle" where v is in
// [0,1].
// I think it can be thought of as "the mount of area covered in the field of
// view".
fn environment_pixel_solid_angle(
    v: f32,
    environment: EnvironmentMetadata
) -> f32 {
    let theta = PI * v;
    let sin_t = max(1.0e-6, sin(theta));
    // Change of the horizontal and vertical angle (in radians) per pixel.
    let d_phi   = 2. * PI / f32(environment.width);
    let d_theta = PI / f32(environment.height);
    return d_phi * d_theta * sin_t;
}

/// Calculates the value of the given environment's HDRI's PDF for the given
/// direction.
fn environment_direction_pdf(
    direction: vec3<f32>,
    environment_index: u32
) -> f32 {
    let metadata = environments[environment_index];
    
    let uv = direction_to_equirectangular_uv(direction);
    let x = min(u32(uv.x * f32(metadata.width)), metadata.width - 1);
    let y = min(u32(uv.y * f32(metadata.height)), metadata.height - 1);
    let index = x + y * metadata.width;
    
    let pmf = environment_alias_entry(index, environment_index).pmf;
    let delta_solid_angle = environment_pixel_solid_angle(uv.y, metadata);
    let pdf = pmf / delta_solid_angle;
    
    return pdf;
}

struct EnvironmentSample {
    // The sampled direction.
    direction: vec3<f32>,
    // The incoming light for this sample.
    radiance: vec3<f32>,
    // The PDF's value for `direction`.
    pdf: f32,
}

/// Sample the given environment according to the distribution defined by its
/// alias table.
fn sample_environment(
    environment_index: u32,
    rng_state: ptr<function, u32>
) -> EnvironmentSample {
    let metadata = environments[environment_index];
    
    // Pick index via alias table.
    let index = random_index_in_environment(environment_index, rng_state);
    
    // Calculate index's coresponding coordinates. The indices are laid out in
    // row-major order.
    let x = index % metadata.width;
    let y = index / metadata.width;
    
    // Add jitter within pixel to emulate continuous sampling.
    let jitter_x = random_uniform(rng_state);
    let jitter_y = random_uniform(rng_state);
    
    let uv = vec2<f32>(
        (f32(x) + jitter_x) / f32(metadata.width),
        (f32(y) + jitter_y) / f32(metadata.height),
    );
    
    let direction = equirectangular_uv_to_direction(uv);
    
    let radiance = textureSampleLevel(
        environment_textures[environment_map_index],
        environment_sampler,
        uv,
        0,
    ).xyz;
    
    // Calculate the PDF's value, measured per steradian.
    let pmf = environment_alias_entry(index, environment_index).pmf;
    let delta_solid_angle = environment_pixel_solid_angle(uv.y, metadata);
    let pdf = pmf / delta_solid_angle;
    
    return EnvironmentSample(direction, radiance, pdf);
}

fn sky_light(ray_direction: vec3<f32>) -> vec3<f32> {
    let uv = direction_to_equirectangular_uv(ray_direction);
    
    return textureSampleLevel(
        environment_textures[environment_map_index],
        environment_sampler,
        uv,
        0,
    ).xyz;
}

/// The parameters representing a surface material required for a BSDF shader.
struct BsdfMaterial {
    // The albedo for diffuse materials, and metal reflectance for metals.
    color: vec3<f32>,
    metallic: f32,
    /// Surface variance parameter. This somehow represents the surface
    /// microfacet normal distribution. The distribution describes the
    /// percentage of microfacets with a certain normal for a given surface
    /// point.
    alpha: f32,
    /// The fresnel reflectance at normal incidence, i.e. the fraction of light
    /// that is reflected when a ray of light hits the surface head-on.
    f0: vec3<f32>,
    /// The emitted light from the surface.
    emission: vec3<f32>,
}

fn make_bsdf_material(material: Material) -> BsdfMaterial {
    // TODO: Support perfectly smooth materials using separate shader.
    let alpha = max(0.001, material.roughness * material.roughness);
    return BsdfMaterial(
        material.color,
        material.metallic,
        alpha,
        surface_f0(material),
        material.emission,
    );
}

/// The fresnel reflectance at normal incidence of a dielectric surface (a
/// non-metal). Read non-conductive.
const DIELECTRIC_F0 = vec3<f32>(0.04, 0.04, 0.04);

/// Computes fresnel reflectance at normal incidence for the given material,
/// i.e. the fraction of light that is reflected when a ray of light hits the
/// surface head-on.
fn surface_f0(material: Material) -> vec3<f32> {
    // F0 is equal to the base color for metallic matterials and `DIELECTRIC_F0` for
    // diffuse materials.
    return lerp_vec3f(DIELECTRIC_F0, material.color, saturate(material.metallic));
}

/// Computes the diffuse reflectance coefficient for the given material, i.e.
/// the fraction of incoming light that enters the surface, scatters internally,
/// and then exits back out diffusely.
fn surface_kd(material: BsdfMaterial) -> vec3<f32> {
    let kd0 = material.color * (1 - saturate(material.metallic));
    return kd0 * (1 - max_component(material.f0));
}

/// Computes the "luminance" of a color.
fn luminance(color: vec3<f32>) -> f32 {
    return 0.2126 * color.r + 0.7152 * color.g + 0.0722 * color.b;
}

/// Returns the point of the hemisphere pointed towards +z, corresponding to the
/// given pair of coordinates. These should be in the range [0,1). If the
/// coordinates have been selected by a uniform distribution the point in the
/// hemisphere has a "cosine-weighted distribution".
fn sample_cosine_hemisphere(sample: vec2<f32>) -> vec3<f32> {
    let r = sqrt(sample.x);
    let phi = 2 * PI * sample.y;

    let x = r * cos(phi);
    let y = r * sin(phi);
    let z = sqrt(max(0., 1. - x * x - y * y));

    return vec3(x, y, z);
}

/// Returns the point on the unit disk corresponding to the given pair of
/// coordinates. These should be in the range [0,1). If the coordinates have
/// been selected by a uniform distribution the points in the disk will also
/// have a uniform distribution.
fn sample_uniform_disk(sample: vec2<f32>) -> vec2<f32> {
    let radius = sqrt(sample.x);
    let azimuth = 2 * PI * sample.y;
    return vec2<f32>(radius * cos(azimuth), radius * sin(azimuth));
}

/// Returns the PDF value of a direction in frame local space or something.
fn pdf_cosine_hemisphere(wi: vec3<f32>) -> f32 {
    if wi.z <= 0 {
        return 0.;
    }
    return wi.z / PI;
}

/// Calculates the microfacet orientation density of the given microfacet normal
/// h for a material with the given surface variance `alpha`. If n is the
/// surface normal, then the argument `normal_dot_h` should be equal to dot(n,h).
fn d_ggx(normal_dot_h: f32, alpha: f32) -> f32 {
    let alpha_2 = alpha * alpha;
    let denominator = (normal_dot_h * normal_dot_h) * (alpha_2 - 1.) + 1.;
    return alpha_2 / (PI * denominator * denominator);
}

/// Returns the value of `sample_ggx_visible_half_vector`'s PDF.
fn pdf_ggx_half_vector_visible(h: vec3<f32>, wo: vec3<f32>, alpha: f32) -> f32 {    
    // Remember, normal = vec3(0, 0, 1) in the view space (which `h` and `wo`
    // are in).
    let normal_dot_h = h.z;
    let normal_dot_wo = wo.z;
    
    if normal_dot_h <= 0. {
      return 0.;
    }
    
    return d_ggx(normal_dot_h, alpha)
        * g1_ggx(normal_dot_wo, alpha)
        * max(0., dot(wo, h))
        / normal_dot_wo;
}

/// Computes the GGX half-vector direction vector for the given pair of
/// coordinates. These should be in the range [0,1).
/// `alpha` is the surface variance parameter, i.e. the "roughness" of the
/// surface.
/// This makes sure to only return half-vectors that are visible from the given
/// out ray direction `wo`. I.e. it implements VNDF sampling.
/// Here is an example of a half-vector h that wouldn't be returned, since the
/// surface would cover it from wo.
///             /\ surface normal
///             |       __
///             |    _,-´| wo
///    h __     |_,-´
///     |`-_   /---------
///         `-/ <-- microfacet surface
///   _______/
fn sample_ggx_visible_half_vector(
    sample: vec2<f32>,
    wo: vec3<f32>,
    alpha: f32,
) -> vec3<f32> {
    // Stretch the outgoing direction. For isotropic GGX we stretch it by alpha.
    // TODO: Support anisotropy by stretching x and y differently.
    let view_stretched =
        normalize(wo * vec3(alpha, alpha, 1.));

    // The papers code
    // Build frame around the stretched view direction.
    let length_squared = dot(view_stretched.xy, view_stretched.xy);
    let tangent_x = select(
        vec3(1., 0., 0.),
        vec3(-view_stretched.y, view_stretched.x, 0.) * inverseSqrt(length_squared),
        length_squared > 0.,
    );
    let tangent_y = cross(view_stretched, tangent_x);
    
    var disk_point = sample_uniform_disk(sample);
    // Warp the disk sample so it matches the "visible normals" distribution.
    // Interpolate between "a circle and a line segment" depending on the
    // grazing angle.
    // This corresponds to constructing (t_1, t_2') in figure 6 from this
    // paper: http://jcgt.org/published/0007/04/01/.
    disk_point.y = lerp_f32(
        sqrt(max(0., 1. - disk_point.x * disk_point.x)),
        disk_point.y,
        view_stretched.z,
    );
    
    // Construct the sampled normal in the view frame's local space, i.e. the
    // space which `wo` is in (world space from the stretch frame's
    // perspective).
    let half_vector_stretched = disk_point.x * tangent_x
        + disk_point.y * tangent_y
        + sqrt(max(0., 1. - disk_point.x * disk_point.x - disk_point.y * disk_point.y)) * view_stretched;
    
    // Unstretch back into original local shading space.
    let half_vector = normalize(vec3<f32>(
        alpha * half_vector_stretched.x,
        alpha * half_vector_stretched.y,
        max(0., half_vector_stretched.z)
    ));
    
    return half_vector;
}

/// Computes the average self-occlusion along a direction v for a material with
/// the given surface variance `alpha`. If n is the surface normal, then the
/// argument `normal_dot_v` should be equal to dot(n,v).
fn lambda_ggx(normal_dot_v: f32, alpha: f32) -> f32 {
    let normal_dot_v_2 = normal_dot_v * normal_dot_v;
    
    return (sqrt(1 + alpha * alpha * (1 - normal_dot_v_2) / normal_dot_v_2)
        - 1)
        / 2;
}

/// I don't understand this, but according to ChatGPT, this calculates the
/// "single-direction visibility probability" from direction v, for a material
/// with the given surface variance `alpha`. If n is the surface normal, then
/// the argument `normal_dot_v` should be equal to dot(n,v).
fn g1_ggx(normal_dot_v: f32, alpha: f32) -> f32 {
    return 1. / (1 + lambda_ggx(normal_dot_v, alpha));
}

/// Calculates the probability that a microfacet is visible from both the viewer
/// direction o and light direction i, given a material with the surface
/// variance `alpha`.
///
/// If n is the surface normal, then the argument `normal_dot_o` and
/// `normal_dot_i` should be equal to dot(n,o) and dot(n,i) respectively.
fn g_smith_ggx(normal_dot_o: f32, normal_dot_i: f32, alpha: f32) -> f32 {
    return g1_ggx(normal_dot_o, alpha) * g1_ggx(normal_dot_i, alpha);
}

/// I also don't understand this. According to ChatGPT, this should calculate
/// the "angle-dependent surface reflectivity".
///
/// It answers the question "Given that light hits a microfacet at this angle,
/// how much of it is reflected instead of entering the material?"
fn f_schlick(f0: vec3<f32>, cos_theta: f32) -> vec3<f32> {
    let x = (1 - saturate(cos_theta));
    let x_2 = x * x;
    let x_5 = x_2 * x_2 * x;
    
    return f0 + (vec3(1.) - f0) * x_5;
}

fn bsdf_eval_local(
    wo: vec3<f32>,
    wi: vec3<f32>,
    material: BsdfMaterial
) -> vec3<f32> {
    if wo.z <= 0 || wi.z <= 0 {
        return vec3(0.,0.,0.);
    }
    
    /// The dot product of the surface normal and the respective light direction
    /// vectors.
    let normal_dot_wo = wo.z;
    let normal_dot_wi = wi.z;
    
    // The half-vector (whatever that is...)
    let h = normalize(wo + wi);
    let normal_dot_h = saturate(h.z);
    
    // Specular microfacet
    let orientation_density = d_ggx(normal_dot_h, material.alpha);
    let visibility_probability = g_smith_ggx(normal_dot_wo, normal_dot_wi, material.alpha);
    let surface_reflectivity = f_schlick(material.f0, dot(h, wo));
    
    // No clue what this is. Apparently described by "spectrum".
    let fs = (orientation_density * visibility_probability)
        / (4 * normal_dot_wo * normal_dot_wi)
        * surface_reflectivity;
    
    
    // Somehow described by "diffuse lambert":
    let kd = surface_kd(material);
    let fd = kd * (1 / PI);
    
    return fd + fs;
}

fn pdf_specular_wi_visible(wo: vec3<f32>, wi: vec3<f32>, alpha: f32) -> f32 {
    if wo.z <= 0. || wi.z <= 0. {
        return 0.;
    }
    
    let h = normalize(wo + wi);
    
    let wo_dot_h = abs(dot(wo, h));
    if wo_dot_h <= 0. {
        return 0.;
    }
    
    return pdf_ggx_half_vector_visible(h, wo, alpha) / (4 * wo_dot_h);
}

fn bsdf_pdf_local(wo: vec3<f32>, wi: vec3<f32>, material: BsdfMaterial) -> f32 {
    if wo.z <= 0. || wi.z <= 0. {
        return 0.;
    }
    
    let specular_probability = saturate(luminance(material.f0));
    let diffuse_probability = 1. - specular_probability;
    
    return diffuse_probability * pdf_cosine_hemisphere(wi)
        + specular_probability * pdf_specular_wi_visible(wo, wi, material.alpha);
}

fn bsdf_sample(
    ray: Ray,
    surface_normal: vec3<f32>,
    material: BsdfMaterial,
    rng_state: ptr<function, u32>,
) -> BsdfSample {
    // Points from the surface point to the camera (i.e. previous surface).
    let wo_world = -ray.direction;
    
    if dot(surface_normal, wo_world) <= 0 {
        return BsdfSample(
            vec3(0., 0., 0.),
            vec3(0., 0., 1.),
            0.
        );
    }
    
    let frame = make_frame(surface_normal);
    
    let wo = to_frame_local(frame, wo_world);
    
    if wo.z <= 0 {
        // wo points below the surface (can happen with shading normals), bail.
        return BsdfSample(
            vec3(0., 0., 0.),
            vec3(0., 1., 0.),
            0.
        );
    }
    
    // I think these names are correct.
    let specular_probability = saturate(luminance(material.f0));
    let diffuse_probability = 1. - specular_probability;
    
    // Choose lobe to sample.
    var wi: vec3<f32>;
    let sample = random_uniform(rng_state);
    if sample < diffuse_probability {
        // Sample diffuse.
        // For some reason we reuse the previous sample.
        wi = sample_cosine_hemisphere(vec2(
            sample / max(diffuse_probability, 1.e-6),
            random_uniform(rng_state),
        ));
    } else {
        // Sample specular.
        let h = sample_ggx_visible_half_vector(
            vec2(
                (sample - diffuse_probability)
                    / max(specular_probability, 1.e-6),
                random_uniform(rng_state),
            ),
            wo,
            material.alpha,
        );
        
        // Reflect wo about h to get wi (still in the frame's local space).
        // This convention assumes wo and wi are both above surface.
        // Note: it's critical that both `wo` and `h` are normalized.
        wi = reflect(-wo, h);
        if wi.z <= 0. {
            // `-wo` is blocked from "seeing" `h`'s surface, which the sampling
            // function should prevent (see the diagram in
            // sample_ggx_visible_half_vector).
            // This can still occur due to imprecision though.
            return BsdfSample(
                vec3(1., 0., 0.),
                vec3(1., 0., 0.),
                0.
            );
        }
    }
    
    let scattering = bsdf_eval_local(wo, wi, material);
    let pdf = bsdf_pdf_local(wo, wi, material);
    let wi_world = to_frame_world(frame, wi);
    
    if dot(surface_normal, wi_world) < 0 {
        return BsdfSample(
            vec3(0., 0., 0.),
            vec3(0., 1., 0.),
            0.
        );
    }
    
    return BsdfSample(wi_world, scattering, pdf);
}

/// Magic function which combines two sampled PDFs which gives a "sharpened"
/// weight. (I understand this very poorly.)
fn power_heuristic(pdf_a: f32, pdf_b: f32) -> f32 {
    let pdf_a_2 = pdf_a * pdf_a;
    let pdf_b_2 = pdf_b * pdf_b;
    return pdf_a_2 / (pdf_a_2 + pdf_b_2);
}

// Trace ray through scene, returning the collected light.
fn trace_ray(ray_arg: Ray, rng_state: ptr<function, u32>) -> vec3<f32> {
    var ray = ray_arg;
    
    var incoming_light = vec3<f32>(0.);
    var throughput = vec3<f32>(1.);
    var last_sample_pdf = 1.;
    
    for (var bounce_count = 0u; bounce_count < MAX_BOUNCES; bounce_count++) {
        var info = cast_ray(ray);
        if !info.did_hit {
            // Ray escaped into environment.
            let environment_light = sky_light(ray.direction);
            let pdf =
                environment_direction_pdf(ray.direction, environment_map_index);
            let weight = power_heuristic(last_sample_pdf, pdf);
            
            incoming_light += throughput * environment_light * weight;
            break;
        }
        
        let material = make_bsdf_material(materials[info.material_id]);
        
        // Add surface emission with pre-bounce throughput
        incoming_light += throughput * material.emission;
        
        // Do Next Event Estimation and MIS environment sample
        {
            let environment = sample_environment(environment_map_index, rng_state);
            let wo_world = -ray.direction;
            let wi_world = environment.direction;
            
            let cos_theta = max(0., dot(info.normal, wi_world));
            
            if (cos_theta > 0.0
                && environment.pdf > 0.0
                // Check that environment sample isn't occluded.
                && !cast_ray_bvh(Ray(info.hit_point, environment.direction))
                    .did_hit)
            {
                let frame = make_frame(info.normal);
                let wo = to_frame_local(frame, wo_world);
                let wi = to_frame_local(frame, wi_world);
                
                let scattering = bsdf_eval_local(wo, wi, material);
                let pdf_bsdf = bsdf_pdf_local(wo, wi, material);
                let weight = power_heuristic(environment.pdf, pdf_bsdf);
                incoming_light += throughput
                    * weight
                    * environment.radiance
                    * scattering
                    * cos_theta
                    / environment.pdf;
            }
        }
        
        // Sample BSDF and continue the ray path.
        {
            let sample = bsdf_sample(ray, info.normal, material, rng_state);
            if all(sample.ray_direction == vec3<f32>(0.)) {
                // Error occurred during sample calculation, show scattering as
                // debug information.
                incoming_light = sample.scattering;
                break;
            }
            if sample.pdf <= 0. {
                // Ray probability density is zero, terminate ray path.
                break;
            }
            
            // We update throughput using
            // throughput *= light * cos(theta) / pdf
            //   where cos(theta) = normal·wi in WORLD space,
            //                      using shading normal.
            let cos_theta = max(0., dot(info.normal, sample.ray_direction));
            throughput *= sample.scattering * (cos_theta / sample.pdf);
            
            if (length(throughput) < 0.001) {
                // Ray contribution is negligible, terminate.
                break;
            }
            
            last_sample_pdf = sample.pdf;
            ray = Ray(
                info.hit_point,
                sample.ray_direction,
            );
        }
    }
    
    return incoming_light;
}

@compute @workgroup_size(16, 16, 1)
fn main(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let pixel_coords = vec2<u32>(global_id.x, global_id.y);
    
    let pixel_index = pixel_coords.y * resolution.x + pixel_coords.x;
    var rng_state: u32 = 0;
    salt_rng(&rng_state, pixel_index);
    salt_rng(&rng_state, sample_count);
    
    if dev_index == 2 {
        // Draw pixels based on distribution.
        let count = 20u;
        for (var i = 0u; i < count; i++) {
            let metadata = environments[environment_map_index];
            let index =
                random_index_in_environment(environment_map_index, &rng_state);
            // Calculate index's coresponding coordinates. The indices are laid
            // out in row-major order.
            let x = index % metadata.width;
            let y = index / metadata.width;
            
            let color = textureLoad(out_texture, vec2(x, y)).xyz
                + vec3(0.1, 0.1, 0.1) / f32(count);
            
            textureStore(out_texture, vec2(x, y), vec4(color, 0.));
        }
        return;
    } else if dev_index == 3 {
        // Display HDRI.
        let color = textureLoad(environment_textures[environment_map_index], pixel_coords, 0).xyz;
        
        textureStore(out_texture, pixel_coords, vec4(saturate(color), 0.));
        return;
    }
    
    let jittered_pixel_coords =
        vec2<f32>(pixel_coords) + random_in_circle_uniform(&rng_state);
    
    // Each component is in the range -1.0 to 1.0.
    let screen_coords =
        ((jittered_pixel_coords / vec2<f32>(resolution)) * 2.0
            - vec2<f32>(1.0))
            * vec2<f32>(1.0, -1.0);
    
    // Let the b be the vertical component of ray_camera_space. For the top
    // row of pixels we want the triangle with a base of 1 and height of b
    // to have the angle opposite to b be equal to fov_y/2.
    // The definition of sin gives us this equation: sin(fov_y/2) = b / 1
    let max_y_component = sin(camera.fov_y / 2);
    
    let aspect_ratio = f32(resolution.x) / f32(resolution.y);
    
    let ray_camera_space = vec3<f32>(
        screen_coords.x * max_y_component * aspect_ratio,
        screen_coords.y * max_y_component,
        -1.0,
    );
    let ray = Ray(camera.pos, normalize(camera.rot_transform * ray_camera_space));
    
    let sample_light = trace_ray(ray, &rng_state);
    
    // The total collected light over cumulative frames.
    let total_light = textureLoad(cumulative_light_texture, pixel_coords).xyz
        + sample_light;
    let pixel_color = total_light / f32(sample_count + 1);
    
    textureStore(cumulative_light_texture, pixel_coords, vec4<f32>(total_light, 1.0));
    textureStore(out_texture, pixel_coords, vec4<f32>(pixel_color, 1.0));
}
