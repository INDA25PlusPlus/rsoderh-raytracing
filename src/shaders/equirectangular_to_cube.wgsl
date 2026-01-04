struct Face {
    forward: vec3<f32>,
    up: vec3<f32>,
    right: vec3<f32>,
}

@group(0) @binding(0)
var src: texture_2d<f32>;

@group(0) @binding(1)
var dest: texture_storage_2d_array<rgba32float, write>;

@compute
@workgroup_size(16, 16, 1)
fn main(
    @builtin(global_invocation_id)
    gid: vec3<u32>,
) {
    // If the texture size is not divisible by 16 `gid.xy` may point outside
    // texture bounds.
    if any(gid.xy >= textureDimensions(dest)) {
        return;   
    }
    
    // All six faces of a cube, in the order defined by the "cube"
    // `TextureViewDimension` in the standard.
    const FACES = array(
        // +X
        Face(
            vec3(1.0, 0.0, 0.0),
            vec3(0.0, 1.0, 0.0),
            vec3(0.0, 0.0, -1.0),
        ),
        // -X
        Face (
            vec3(-1.0, 0.0, 0.0),
            vec3(0.0, 1.0, 0.0),
            vec3(0.0, 0.0, 1.0),
        ),
        // +Y
        Face (
            vec3(0.0, -1.0, 0.0),
            vec3(0.0, 0.0, 1.0),
            vec3(1.0, 0.0, 0.0),
        ),
        // -Y
        Face (
            vec3(0.0, 1.0, 0.0),
            vec3(0.0, 0.0, -1.0),
            vec3(1.0, 0.0, 0.0),
        ),
        // +Z
        Face (
            vec3(0.0, 0.0, 1.0),
            vec3(0.0, 1.0, 0.0),
            vec3(1.0, 0.0, 0.0),
        ),
        // -Z
        Face (
            vec3(0.0, 0.0, -1.0),
            vec3(0.0, 1.0, 0.0),
            vec3(-1.0, 0.0, 0.0),
        ),
    );
    
    let dest_dimensions = vec2<f32>(textureDimensions(dest));
    // Coordinates relative to the cubemap face.
    let cube_coords = vec2<f32>(gid.xy) / vec2<f32>(textureDimensions(dest)) * 2. - 1.;
    
    let face = FACES[gid.z];
    let spherical_coords = normalize(
        face.forward + face.right * cube_coords.x + face.up * cube_coords.y
    );
    
    let inv_atan = vec2<f32>(0.1591, 0.3183); // TODO: What
    // Coordinates on the equirectangular texture.
    let equirect_coords = vec2<f32>(
        atan2(spherical_coords.z, spherical_coords.x),
        asin(spherical_coords.y)
    ) * inv_atan
        + 0.5;
    let equirect_pixel_coords =
        vec2<u32>(equirect_coords * vec2<f32>(textureDimensions(src)));
    
    // We use `textureLoad` as `textureSample` isn't allowed in compute shaders.
    let sample = textureLoad(src, equirect_pixel_coords, 0);
    
    textureStore(dest, gid.xy, gid.z, sample);
}
