use std::sync::Arc;

use cgmath::{Deg, Rad};
use glam::{Vec3, vec3};
#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;
use winit::{
    application::ApplicationHandler, event::*, event_loop::ActiveEventLoop,
    platform::modifier_supplement::KeyEventExtModifierSupplement, window::Window,
};

use crate::{
    asset,
    camera::{Camera, KeyboardLayout},
    mesh::{Mesh, PackedMeshes},
    scene::{Material, Plane, Scene, Sphere},
    state::State,
};

pub struct App {
    #[cfg(target_arch = "wasm32")]
    proxy: Option<winit::event_loop::EventLoopProxy<State>>,
    keyboard_layout: KeyboardLayout,
    camera_state: Option<String>,
    state: Option<State>,
}

impl App {
    pub fn new(
        keyboard_layout: KeyboardLayout,
        camera_state: Option<String>,
        #[cfg(target_arch = "wasm32")] event_loop: &EventLoop<State>,
    ) -> Self {
        #[cfg(target_arch = "wasm32")]
        let proxy = Some(event_loop.create_proxy());
        Self {
            state: None,
            keyboard_layout,
            camera_state,
            #[cfg(target_arch = "wasm32")]
            proxy,
        }
    }
}

impl ApplicationHandler<State> for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        #[allow(unused_mut)]
        let mut window_attributes = Window::default_attributes();

        #[cfg(target_arch = "wasm32")]
        {
            use wasm_bindgen::JsCast;
            use winit::platform::web::WindowAttributesExtWebSys;

            const CANVAS_ID: &str = "canvas";

            let window = wgpu::web_sys::window().unwrap_throw();
            let document = window.document().unwrap_throw();
            let canvas = document.get_element_by_id(CANVAS_ID).unwrap_throw();
            let html_canvas_element = canvas.unchecked_into();
            window_attributes = window_attributes.with_canvas(Some(html_canvas_element));
        }

        let window = Arc::new(event_loop.create_window(window_attributes).unwrap());

        let camera = self
            .camera_state
            .as_ref()
            .map(|state| Camera::deserialize(&state).unwrap())
            .unwrap_or(Camera {
                // position the camera 1 unit up and 2 units back
                // +z is out of the screen
                pos: (0.0, 1.0, 3.0).into(),
                pitch: Rad(0.0),
                yaw: Rad(0.0),
                fov_y: Deg(100.0).into(),
            });
        let scene = Scene {
            camera,
            materials: vec![
                Material {
                    color: vec3(1.0, 0.63, 0.41),
                    roughness: 1.,
                    metallic: 0.,
                    emission: Vec3::ZERO,
                },
                Material {
                    color: vec3(0.56, 1.0, 0.52),
                    roughness: 1.,
                    metallic: 0.,
                    emission: Vec3::ZERO,
                },
                // Ground
                Material {
                    color: vec3(0.95, 0.95, 0.95),
                    roughness: 1.,
                    metallic: 0.,
                    emission: Vec3::ZERO,
                },
                // Marker
                Material {
                    color: vec3(1.0, 1.0, 1.0),
                    roughness: 1.,
                    metallic: 0.,
                    emission: vec3(1.0, 0.0, 0.0),
                },
                // Metallic Mirror
                Material {
                    color: vec3(0.8, 0.8, 0.8),
                    roughness: 0.0,
                    metallic: 1.,
                    emission: Vec3::ZERO,
                },
                // Dielectric Mirror
                Material {
                    color: vec3(0.8, 0.8, 0.8),
                    roughness: 0.0,
                    metallic: 0.,
                    emission: Vec3::ZERO,
                },
            ],
            spheres: vec![
                Sphere {
                    pos: vec3(0.0, 1.1, -2.0),
                    radius: 1.0,
                    material_id: 0,
                },
                Sphere {
                    pos: vec3(1.3, 1.1, -1.5),
                    radius: 1.3,
                    material_id: 1,
                },
                // Mirrors
                Sphere {
                    pos: vec3(1.2, 1.1, 1.0),
                    radius: 0.6,
                    material_id: 4,
                },
                Sphere {
                    pos: vec3(2.6, 1.1, 1.0),
                    radius: 0.6,
                    material_id: 5,
                },
                // Markers
                Sphere {
                    pos: vec3(0., 0., 0.),
                    radius: 0.05,
                    material_id: 3,
                },
                Sphere {
                    pos: vec3(-4., 0., -5.),
                    radius: 0.05,
                    material_id: 3,
                },
                Sphere {
                    pos: vec3(6., 0., 5.),
                    radius: 0.05,
                    material_id: 3,
                },
                Sphere {
                    pos: vec3(1., -0.05, 1.),
                    radius: 0.05,
                    material_id: 3,
                },
                Sphere {
                    pos: vec3(1.2, -0.03, 1.),
                    radius: 0.05,
                    material_id: 3,
                },
                Sphere {
                    pos: vec3(1.4, -0.01, 1.),
                    radius: 0.05,
                    material_id: 3,
                },
            ],
            planes: vec![Plane {
                pos: vec3(-4., 0., -5.),
                forward: vec3(0., 0., 10.),
                right: vec3(10., 0., 0.),
                material_id: 2,
            }],
            meshes: PackedMeshes::pack_meshes(&[
                // Suzanne is way to expensive...
                // Mesh::load(asset::include_str!("../assets/suzanne.obj")).expect("Uh oh..."),
                Mesh::load(asset::include_str!("../assets/cube.obj")).expect("Uh oh..."),
            ]),
        };

        #[cfg(not(target_arch = "wasm32"))]
        {
            // If we are not on web we can use pollster to
            // await the
            self.state = Some(
                pollster::block_on(State::new(window, self.keyboard_layout.clone(), scene))
                    .unwrap(),
            );
        }

        #[cfg(target_arch = "wasm32")]
        {
            // Run the future asynchronously and use the
            // proxy to send the results to the event loop
            if let Some(proxy) = self.proxy.take() {
                wasm_bindgen_futures::spawn_local(async move {
                    assert!(
                        proxy
                            .send_event(
                                State::new(window, self.keyboard_layout.clone(), scene)
                                    .await
                                    .expect("Unable to create canvas!!!")
                            )
                            .is_ok()
                    )
                });
            }
        }
    }

    #[allow(unused_mut)]
    fn user_event(&mut self, _event_loop: &ActiveEventLoop, mut event: State) {
        // This is where proxy.send_event() ends up
        #[cfg(target_arch = "wasm32")]
        {
            event.window.request_redraw();
            event.resize(
                event.window.inner_size().width,
                event.window.inner_size().height,
            );
        }
        self.state = Some(event);
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _window_id: winit::window::WindowId,
        event: WindowEvent,
    ) {
        let state = match &mut self.state {
            Some(state) => state,
            None => return,
        };

        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(size) => state.resize(size.width, size.height),
            WindowEvent::RedrawRequested => {
                state.update();
                match state.render() {
                    Ok(_) => {}
                    // Reconfigure the surface if it's lost or outdated
                    Err(wgpu::SurfaceError::Lost | wgpu::SurfaceError::Outdated) => {
                        let size = state.window.inner_size();
                        state.resize(size.width, size.height);
                    }
                    Err(e) => {
                        log::error!("Unable to render {}", e);
                    }
                }
            }
            WindowEvent::KeyboardInput {
                event: event @ KeyEvent { repeat: false, .. },
                ..
            } => state
                .handle_key(
                    event_loop,
                    &event.key_without_modifiers(),
                    event.state.is_pressed(),
                )
                .unwrap(),
            _ => {}
        }
    }

    fn device_event(
        &mut self,
        _event_loop: &ActiveEventLoop,
        _device_id: DeviceId,
        event: DeviceEvent,
    ) {
        let state = match &mut self.state {
            Some(state) => state,
            None => return,
        };

        match event {
            DeviceEvent::MouseMotion { delta } => state.handle_mouse_motion(delta),
            _ => {}
        }
    }
}
