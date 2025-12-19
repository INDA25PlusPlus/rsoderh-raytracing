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

struct Material {
    color: vec3<f32>,
    roughness: f32,
    emission_strength: f32,
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

@group(0) @binding(0)
var out_texture: texture_storage_2d<rgba16float, write>;

@group(1) @binding(0)
var<uniform> camera: Camera;

@group(1) @binding(1)
var<uniform> resolution: vec2<u32>;

@group(1) @binding(2)
var<uniform> time_secs: f32;

@group(2) @binding(0)
var<storage, read> materials: array<Material>;

@group(2) @binding(1)
var<storage, read> spheres: array<Sphere>;

@group(2) @binding(2)
var<storage, read> planes: array<Plane>;

const NUM_SAMPLES: u32 = 100;

// Largest representable f32 (actual IEEE infinity can't be used).
const INFINITY = 1.70141183460469231732e+38f;
// Smallest representable f32 (actual IEEE negative infinity can't be used).
const NEG_INFINITY = -1.70141183460469231732e+38f;

// Random number generator
// Based on algorithm from:
// https://marktension.nl/blog/my_favorite_wgsl_random_func_so_far/

fn hash_u32(x: u32) -> u32 {
    var result = x;
    result += (result << 10);
    result ^= (result >> 6);
    result += (result << 3);
    result ^= (result >> 11);
    result += (result << 15);
    return result;
}

// Construct a float with half-open range [0,1) using low 23 bits. All zeroes
// yields 0.0, all ones yields the next smallest representable value below 1.0.
fn float_construct_from_u32(m_in: u32) -> f32 {
    const ieee_mantissa = 0x007FFFFFu; // binary32 mantissa bitmask
    const ieee_one = 0x3F800000u;      // 1.0 in IEEE binary32

    var m = m_in;
    m &= ieee_mantissa;              // Keep only mantissa bits (fractional part)
    m |= ieee_one;                   // Add fractional part to 1.0

    let f = bitcast<f32>(m);        // Range [1:2]
    return f - 1.0;                 // Range [0:1]
}

// Returns a random float in the range [0,1), updating the RNG state in the
// process.
fn random_uniform(rng_state: ptr<function, u32>) -> f32 {
    *rng_state += 1;
    return float_construct_from_u32(hash_u32(*rng_state));
}

// Returns a uniformly distributed random point on the unit circle. The point's
// length is in the range [0,1).
fn random_in_circle_uniform(rng_state: ptr<function, u32>) -> vec2<f32> {
    let angle: f32 = random_uniform(rng_state) * 2 * 3.1415926;
    let pointOnCircle = vec2(cos(angle), sin(angle));
    return pointOnCircle * sqrt(random_uniform(rng_state));
}

fn sky_color(ray_direction: vec3<f32>) -> vec3<f32> {
    return vec3(0.57, 1.0, 0.72);
}

fn cast_ray_sphere(ray: Ray, sphere: Sphere) -> HitInfo {
    let l = ray.origin - sphere.pos;
    let a = dot(ray.direction, ray.direction);
    let b = 2 * dot(ray.direction, l);
    let c = dot(l, l) - sphere.radius * sphere.radius;
    
    // Solve at^2 + bt + c = 0
    var t: f32;
    let discriminant = b * b - 4 * a * c;
    if discriminant < 0 {
        return HitInfo(
            false,
            0.0,
            vec3<f32>(0.0),
            vec3<f32>(0.0),
            0u,
        );
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
        if t_0 < 0 {
            t = t_1;
        } else if t_1 < 0 {
            t = t_0;
        } else {
            t = min(t_0, t_1);
        }
    }
    
    if t < 0 {
        return HitInfo(
            false,
            0.0,
            vec3<f32>(0.0),
            vec3<f32>(0.0),
            0u,
        );
    }
    
    let hit_point = ray.origin + ray.direction * t;
    
    return HitInfo(
        true,
        t,
        hit_point,
        normalize(hit_point - sphere.pos),
        sphere.material_id,
    );
}

fn cast_ray_plane(ray: Ray, plane: Plane) -> HitInfo {
    let denominator = dot(plane.normal, ray.direction);
    if (abs(denominator) < 0.0001f) {
        return NO_HIT;
    }
    let t = dot(plane.pos - ray.origin, plane.normal) / denominator;
    if (t < 0) {
        // Intersection point is behind ray
        return NO_HIT;
    }
    
    // Point on plane the ray would intersect with if the plane extended
    // infinitely, relative to the plane's position.
    let inter = ray.origin + ray.direction * t - plane.pos;
    
    // The point in the "plane's base".
    let inter_plane_space = plane.base_change_matrix * inter;
    
    if inter_plane_space.x < 0.
        || 1. < inter_plane_space.x
        || inter_plane_space.z < 0.
        || 1. < inter_plane_space.z
    {
        return NO_HIT;
    }
    
    return HitInfo(
        true,
        t,
        inter,
        plane.normal,
        plane.material_id,
    );
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

// Trace ray through scene, returning the collected light.
fn trace_ray(ray: Ray) -> vec3<f32> {
    let info = cast_ray(ray);
    
    return vec3(select(
        sky_color(ray.direction),
        materials[info.material_id].color,
        info.did_hit,
    ));
}

@compute @workgroup_size(16, 16, 1)
fn main(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let pixel_coords = vec2<u32>(global_id.x, global_id.y);
    
    let pixel_index = pixel_coords.y * resolution.x + pixel_coords.x;
    var rng_state: u32 = pixel_index;
    
    var pixel_color = vec3(0.0);
    for (var i = 0u; i < NUM_SAMPLES; i++) {
        let jittered_pixel_coords =
            vec2<f32>(pixel_coords) + random_in_circle_uniform(&rng_state);
        
        // Each component is in the range -1.0 to 1.0.
        let screen_coords =
            ((jittered_pixel_coords / vec2<f32>(resolution)) * 2.0
                - vec2<f32>(1.0))
                * vec2<f32>(1.0, -1.0);
        
        // Let the h be the vertical component of ray_camera_space. For the top
        // row of pixels we want the triangle with a base of 1 and height of h
        // to have the angle opposite to h be equal to fov_y/2.
        // The definition of sin gives us this equation: sin(fov_y/2) = h / 1
        let max_y_component = sin(camera.fov_y / 2);
        
        let aspect_ratio = f32(resolution.x) / f32(resolution.y);
        
        let ray_camera_space = vec3<f32>(
            screen_coords.x * max_y_component * aspect_ratio,
            screen_coords.y * max_y_component,
            -1.0,
        );
        let ray = Ray(camera.pos, normalize(camera.rot_transform * ray_camera_space));
        
        pixel_color += trace_ray(ray);
    }
    pixel_color /= f32(NUM_SAMPLES);
    
    
    textureStore(out_texture, pixel_coords, vec4<f32>(pixel_color, 1.0));
}
