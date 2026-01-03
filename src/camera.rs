use std::{hash::Hash, sync::Arc};

use anyhow::anyhow;
use base64::Engine;
use cgmath::{Deg, Rad};
use glam::{Mat3, Vec2, Vec3, vec2, vec3};
use winit::{
    keyboard::{Key, NamedKey, SmolStr},
    window::{CursorGrabMode, Window},
};

#[derive(Debug, Clone)]
pub struct Camera {
    pub pos: Vec3,
    pub yaw: Rad<f32>,
    pub pitch: Rad<f32>,
    /// The cameras vertical fov. The horizontal fov is calculated to match in the shader.
    pub fov_y: Rad<f32>,
}

impl Camera {
    /// Returns a matrix which transforms a vector from camera view space to world space, i.e.
    /// applies the camera rotation. Note that the position isn't applied!
    pub fn rot_transform(&self) -> Mat3 {
        Mat3::from_axis_angle(Vec3::Y, self.yaw.0) * Mat3::from_axis_angle(Vec3::X, self.pitch.0)
    }

    pub fn serialize(&self) -> String {
        // let buffer = Vec::new()k

        let mut data = [0u8; size_of::<Vec3>() + size_of::<f32>() * 3];
        // let data: &[u8; size_of::<Vec3>()] = bytemuck::cast_ref(&self.pos);
        {
            let (pos, rest) = data.split_at_mut(size_of::<Vec3>());
            let (yaw, rest) = rest.split_at_mut(size_of::<f32>());
            let (pitch, rest) = rest.split_at_mut(size_of::<f32>());
            let (fov_y, _) = rest.split_at_mut(size_of::<f32>());

            pos.copy_from_slice(bytemuck::cast_slice(&[self.pos]));
            yaw.copy_from_slice(bytemuck::cast_slice(&[self.yaw.0]));
            pitch.copy_from_slice(bytemuck::cast_slice(&[self.pitch.0]));
            fov_y.copy_from_slice(bytemuck::cast_slice(&[self.fov_y.0]));
        }

        let engine = base64::engine::general_purpose::STANDARD;
        engine.encode(data)
    }

    pub fn deserialize(encoded: &str) -> anyhow::Result<Self> {
        let engine = base64::engine::general_purpose::STANDARD;
        let data = engine.decode(encoded)?;

        let len_error = || {
            anyhow!(
                "Couldn't deserialize camera: binary data ({} bytes) not 24 bytes",
                data.len()
            )
        };

        let (pos, rest) = data
            .split_at_checked(size_of::<Vec3>())
            .ok_or_else(len_error)?;
        let (yaw, rest) = rest
            .split_at_checked(size_of::<f32>())
            .ok_or_else(len_error)?;
        let (pitch, rest) = rest
            .split_at_checked(size_of::<f32>())
            .ok_or_else(len_error)?;
        let (fov_y, rest) = rest
            .split_at_checked(size_of::<f32>())
            .ok_or_else(len_error)?;
        if rest.len() != 0 {
            return Err(len_error());
        }

        let pos = bytemuck::cast_slice(pos)[0];
        let yaw = bytemuck::cast_slice(yaw)[0];
        let pitch = bytemuck::cast_slice(pitch)[0];
        let fov_y = bytemuck::cast_slice(fov_y)[0];

        Ok(Self {
            pos,
            yaw: Rad(yaw),
            pitch: Rad(pitch),
            fov_y: Rad(fov_y),
        })
    }
}

impl Hash for Camera {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        // I don't care about NaN inequality. >:)
        bytemuck::cast_ref::<_, [u32; 3]>(&self.pos).hash(state);
        bytemuck::cast_ref::<_, u32>(&self.yaw.0).hash(state);
        bytemuck::cast_ref::<_, u32>(&self.pitch.0).hash(state);
        bytemuck::cast_ref::<_, u32>(&self.fov_y.0).hash(state);
    }
}

/// The camera data sent to the GPU
#[derive(Debug, Copy, Clone, encase::ShaderType)]
pub struct CameraUniform {
    pub pos: Vec3,
    pub rot_transform: Mat3,
    /// The vertical fov, in radians.
    pub fov_y: f32,
}

impl CameraUniform {
    pub fn new(camera: &Camera) -> Self {
        Self {
            pos: camera.pos,
            rot_transform: camera.rot_transform(),
            fov_y: camera.fov_y.0,
        }
    }
}

/// Represents which keys are used for forwards, left, backwards, and right movement.
#[derive(Debug, Clone)]
pub struct KeyboardLayout {
    forward: Key,
    left: Key,
    back: Key,
    right: Key,
    /// Button which toggles mouse capturing, locking the mouse for camera movement.
    capture_mouse: Key,
    /// Button which prints camera state.
    print_camera_state: Key,
}

impl KeyboardLayout {
    pub fn parse_config(movement_config: &str, other_config: &str) -> anyhow::Result<Self> {
        let chars = movement_config
            .chars()
            .map(|chr| chr.to_ascii_lowercase())
            .collect::<Box<[char]>>();
        let Ok(&[forward, left, back, right]): Result<&[char; _], _> = chars.as_ref().try_into()
        else {
            return Err(anyhow!(
                "Invalid keyboard config '{}': expected 4 characters.",
                movement_config,
            ));
        };

        let chars = other_config
            .chars()
            .map(|chr| chr.to_ascii_lowercase())
            .collect::<Box<[char]>>();
        let Ok(&[capture_mouse, print_camera_state]): Result<&[char; _], _> =
            chars.as_ref().try_into()
        else {
            return Err(anyhow!(
                "Invalid mouse capture config '{}': expected 2 character.",
                other_config,
            ));
        };

        fn parse_key(chr: char) -> Key {
            return Key::Character(SmolStr::from(chr.to_string()));
        }

        Ok(Self {
            forward: parse_key(forward),
            left: parse_key(left),
            back: parse_key(back),
            right: parse_key(right),
            capture_mouse: parse_key(capture_mouse),
            print_camera_state: parse_key(print_camera_state),
        })
    }
}

#[derive(Debug, Clone)]
pub struct CameraController {
    layout: KeyboardLayout,
    cursor_captured: bool,
    cursor_captured_pressed: bool,
    print_camera_state_pressed: bool,
    shift_pressed: bool,
    forward_pressed: bool,
    back_pressed: bool,
    left_pressed: bool,
    right_pressed: bool,
    space_pressed: bool,
    velocity: Vec3,
    delta_pixels: Vec2,
}

impl CameraController {
    /// Maximum player speed in units / second
    const MAX_SPEED: f32 = 3.0;
    /// Acceleration in units / second^2
    const ACCELERATION: f32 = 10.0;
    /// Friction in units / second^2
    const FRICTION: f32 = 15.0;
    /// Camera turn factor in degrees / pixel.
    /// Decides the amount of degrees to turn per pixel the cursor was moved.
    const TURN_FACTOR: f32 = 0.25;

    pub fn new(layout: KeyboardLayout) -> Self {
        Self {
            layout,
            cursor_captured: false,
            cursor_captured_pressed: false,
            print_camera_state_pressed: false,
            shift_pressed: false,
            forward_pressed: false,
            back_pressed: false,
            left_pressed: false,
            right_pressed: false,
            space_pressed: false,
            velocity: Vec3::ZERO,
            delta_pixels: Vec2::ZERO,
        }
    }

    pub fn handle_key(
        &mut self,
        window: &Arc<Window>,
        key: &Key,
        is_pressed: bool,
        camera: &Camera,
    ) -> anyhow::Result<()> {
        if key == &self.layout.forward {
            self.forward_pressed = is_pressed;
        } else if key == &self.layout.back {
            self.back_pressed = is_pressed;
        } else if key == &self.layout.left {
            self.left_pressed = is_pressed;
        } else if key == &self.layout.right {
            self.right_pressed = is_pressed;
        } else if key == &self.layout.capture_mouse {
            if !self.cursor_captured_pressed && is_pressed {
                self.cursor_captured = !self.cursor_captured;

                if self.cursor_captured {
                    window.set_cursor_grab(CursorGrabMode::Locked)?;
                    window.set_cursor_visible(false);
                } else {
                    window.set_cursor_grab(CursorGrabMode::None)?;
                    window.set_cursor_visible(true);
                }
            }
            self.cursor_captured_pressed = is_pressed;
        } else if key == &self.layout.print_camera_state {
            if !self.print_camera_state_pressed && is_pressed {
                Self::print_camera_state(camera)
            }
            self.print_camera_state_pressed = is_pressed;
        } else if let Key::Named(key) = key {
            match key {
                NamedKey::Shift => {
                    self.shift_pressed = is_pressed;
                }
                NamedKey::Space => {
                    self.space_pressed = is_pressed;
                }
                _ => {}
            }
        }

        Ok(())
    }

    pub fn handle_mouse_motion(&mut self, delta: (f64, f64)) {
        if self.cursor_captured {
            self.delta_pixels += vec2(delta.0 as f32, delta.1 as f32);
        }
    }

    fn print_camera_state(camera: &Camera) {
        println!("{:#?}", camera);
        println!("state: (for use with --state)\n  {}", camera.serialize());
    }

    pub fn update(&mut self, camera: &mut Camera, delta_seconds: f32) {
        // Update position.

        // Unscaled velocity vector
        let velocity_direction = Mat3::from_axis_angle(Vec3::Y, camera.yaw.0)
            * vec3(
                (if self.right_pressed { 1. } else { 0. })
                    + (if self.left_pressed { -1. } else { 0. }),
                (if self.space_pressed { 1. } else { 0. })
                    + (if self.shift_pressed { -1. } else { 0. }),
                (if self.back_pressed { 1. } else { 0. })
                    + (if self.forward_pressed { -1. } else { 0. }),
            );
        let target_velocity = velocity_direction.normalize_or_zero() * Self::MAX_SPEED;

        let acceleration = if target_velocity == Vec3::ZERO {
            Self::FRICTION
        } else {
            Self::ACCELERATION
        };

        fn move_towards(current: Vec3, target: Vec3, max_delta: f32) -> Vec3 {
            let delta = target - current;
            let dist = delta.length();

            if dist <= max_delta || dist == 0.0 {
                return target;
            }

            return current + delta.normalize() * max_delta;
        }

        self.velocity = move_towards(self.velocity, target_velocity, acceleration * delta_seconds);

        /// Minimum speed before velocity is clamped to zero.
        const EPSILON: f32 = 0.001;

        if self.velocity.length() < EPSILON {
            self.velocity = Vec3::ZERO;
        }

        camera.pos += self.velocity * delta_seconds;

        // Update rotation.

        let delta_yaw = Deg(-self.delta_pixels.x * Self::TURN_FACTOR);
        let delta_pitch = Deg(-self.delta_pixels.y * Self::TURN_FACTOR);
        camera.yaw += delta_yaw.into();
        camera.pitch += delta_pitch.into();
        self.delta_pixels = Vec2::ZERO;
    }
}
