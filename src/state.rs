use std::{
    sync::Arc,
    time::{self, Instant},
};

use wgpu::{BindGroupDescriptor, util::DeviceExt};
use winit::{
    event_loop::ActiveEventLoop,
    keyboard::{Key, NamedKey},
    window::Window,
};

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

use crate::{
    camera::{Camera, CameraController, CameraUniform, KeyboardLayout},
    hdr,
    scene::Scene,
};

// This will store the state of our game
pub struct State {
    pub start_time: time::Instant,
    pub last_tick_time: time::Instant,
    pub surface: wgpu::Surface<'static>,
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    pub config: wgpu::SurfaceConfiguration,
    pub is_surface_configured: bool,
    pub window: Arc<Window>,
    pub pipeline: wgpu::ComputePipeline,
    pub hdr: hdr::HdrPipeline,
    pub texture_bind_group_layout: wgpu::BindGroupLayout,
    pub texture_bind_group: wgpu::BindGroup,
    pub camera: Camera,
    pub camera_controller: CameraController,
    pub camera_buffer: wgpu::Buffer,
    pub resolution_buffer: wgpu::Buffer,
    pub time_secs_buffer: wgpu::Buffer,
    pub render_bind_group: wgpu::BindGroup,
    pub scene_bind_group: wgpu::BindGroup,
}

impl State {
    // We don't need this to be async right now,
    // but we will in the next tutorial
    pub async fn new(
        window: Arc<Window>,
        keyboard_layout: KeyboardLayout,
        scene: Scene,
    ) -> anyhow::Result<Self> {
        let start_time = time::Instant::now();

        let size = window.inner_size();

        // The instance is a handle to our GPU
        // BackendBit::PRIMARY => Vulkan + Metal + DX12 + Browser WebGPU
        let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor {
            #[cfg(not(target_arch = "wasm32"))]
            backends: wgpu::Backends::PRIMARY,
            #[cfg(target_arch = "wasm32")]
            backends: wgpu::Backends::GL,
            ..Default::default()
        });

        let surface = instance.create_surface(window.clone()).unwrap();

        let adapter = instance.request_adapter(&Default::default()).await.unwrap();
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                required_features: wgpu::Features::CLEAR_TEXTURE,
                ..Default::default()
            })
            .await
            .unwrap();

        let surface_caps = surface.get_capabilities(&adapter);
        // Shader code in this tutorial assumes an sRGB surface texture. Using a different
        // one will result in all the colors coming out darker. If you want to support non
        // sRGB surfaces, you'll need to account for that when drawing to the frame.
        let surface_format = surface_caps
            .formats
            .iter()
            .find(|f| f.is_srgb())
            .copied()
            .unwrap_or(surface_caps.formats[0]);
        let config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format: surface_format,
            width: size.width,
            height: size.height,
            present_mode: surface_caps.present_modes[0],
            alpha_mode: surface_caps.alpha_modes[0],
            view_formats: vec![],
            desired_maximum_frame_latency: 2,
        };

        let hdr = hdr::HdrPipeline::new(&device, &config);
        let texture_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("texture_bind_group_layout"),
                entries: &[
                    // Output texture
                    wgpu::BindGroupLayoutEntry {
                        binding: 0,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::StorageTexture {
                            access: wgpu::StorageTextureAccess::WriteOnly,
                            format: wgpu::TextureFormat::Rgba16Float,
                            view_dimension: wgpu::TextureViewDimension::D2,
                        },
                        count: None,
                    },
                ],
            });
        let texture_bind_group = device.create_bind_group(&BindGroupDescriptor {
            label: Some("texture_bind_group"),
            layout: &texture_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(hdr.view()),
            }],
        });

        let camera_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Camera"),
            contents: &encase::StorageBuffer::<()>::content_of::<_, Vec<u8>>(&CameraUniform::new(
                &scene.camera,
            ))
            .unwrap(),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let resolution_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Screen Resolution"),
            contents: bytemuck::cast_slice(&[config.width, config.height]),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let time_secs_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Time Secs"),
            contents: bytemuck::cast_slice(&[0.0f32]),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let render_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                entries: &[
                    // Camera view transform
                    wgpu::BindGroupLayoutEntry {
                        binding: 0,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Uniform,
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                    // Screen resolution
                    wgpu::BindGroupLayoutEntry {
                        binding: 1,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Uniform,
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                    // Time since state creating (seconds)
                    wgpu::BindGroupLayoutEntry {
                        binding: 2,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Uniform,
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                ],
                label: Some("render_bind_group_layout"),
            });

        let render_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &render_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: camera_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: resolution_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: time_secs_buffer.as_entire_binding(),
                },
            ],
            label: Some("render_bind_group"),
        });

        let materials_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Materials"),
            contents: &encase::StorageBuffer::<()>::content_of::<_, Vec<u8>>(&scene.materials)
                .unwrap(),
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        });

        let spheres_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Spheres"),
            contents: &encase::StorageBuffer::<()>::content_of::<_, Vec<u8>>(&scene.spheres)
                .unwrap(),
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        });

        let scene_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                entries: &[
                    // Materials
                    wgpu::BindGroupLayoutEntry {
                        binding: 0,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: true },
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                    // Spheres
                    wgpu::BindGroupLayoutEntry {
                        binding: 1,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: true },
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                ],
                label: Some("scene_bind_group_layout"),
            });

        let scene_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &scene_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: materials_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: spheres_buffer.as_entire_binding(),
                },
            ],
            label: Some("scene_bind_group"),
        });

        let shader = device.create_shader_module(wgpu::include_wgsl!("shaders/shader.wgsl"));

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Compute Pipeline Layout"),
            bind_group_layouts: &[
                &texture_bind_group_layout,
                &render_bind_group_layout,
                &scene_bind_group_layout,
            ],
            push_constant_ranges: &[],
        });

        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("Compute Pipeline"),
            layout: Some(&pipeline_layout),
            module: &shader,
            entry_point: None,
            compilation_options: Default::default(),
            cache: Default::default(),
        });

        Ok(Self {
            start_time,
            last_tick_time: start_time,
            surface,
            device,
            queue,
            config,
            is_surface_configured: false,
            window,
            pipeline,
            hdr,
            texture_bind_group_layout,
            texture_bind_group,
            camera: scene.camera,
            camera_controller: CameraController::new(keyboard_layout),
            camera_buffer,
            resolution_buffer,
            time_secs_buffer,
            render_bind_group,
            scene_bind_group,
        })
    }

    pub fn resize(&mut self, width: u32, height: u32) {
        if width > 0 && height > 0 {
            self.config.width = width;
            self.config.height = height;
            self.surface.configure(&self.device, &self.config);
            self.hdr.resize(&self.device, width, height);
            // Update compute shader to use the new HDR texture.
            self.texture_bind_group = self.device.create_bind_group(&BindGroupDescriptor {
                label: Some("texture_bind_group"),
                layout: &self.texture_bind_group_layout,
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(self.hdr.view()),
                }],
            });

            self.is_surface_configured = true;
        }
    }

    pub fn handle_key(
        &mut self,
        event_loop: &ActiveEventLoop,
        key: &Key,
        is_pressed: bool,
    ) -> anyhow::Result<()> {
        self.camera_controller
            .handle_key(&self.window, key, is_pressed)?;
        match (key, is_pressed) {
            (Key::Named(NamedKey::Escape), true) => event_loop.exit(),
            _ => {}
        }

        Ok(())
    }

    pub fn handle_mouse_motion(&mut self, delta: (f64, f64)) {
        self.camera_controller.handle_mouse_motion(delta);
    }

    pub fn update(&mut self) {
        let delta_seconds = self.last_tick_time.elapsed().as_secs_f32();
        self.last_tick_time = Instant::now();
        self.camera_controller
            .update(&mut self.camera, delta_seconds);

        self.queue.write_buffer(
            &self.camera_buffer,
            0,
            &encase::StorageBuffer::<()>::content_of::<_, Vec<u8>>(&CameraUniform::new(
                &self.camera,
            ))
            .unwrap(),
        );
        self.queue.write_buffer(
            &self.resolution_buffer,
            0,
            bytemuck::cast_slice(&[self.config.width, self.config.height]),
        );
        {
            let time_secs = self.start_time.elapsed().as_secs_f32();
            self.queue.write_buffer(
                &self.time_secs_buffer,
                0,
                bytemuck::cast_slice(&[time_secs]),
            );
        }
    }

    pub fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
        self.window.request_redraw();

        // We can't render unless the surface is configured
        if !self.is_surface_configured {
            return Ok(());
        }

        let output = self.surface.get_current_texture()?;

        let view = output
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Render Encoder"),
            });

        encoder.clear_texture(
            &self.hdr.texture.texture,
            &wgpu::ImageSubresourceRange::default(),
        );

        {
            let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("Compute Pass"),
                ..Default::default()
            });

            compute_pass.set_pipeline(&self.pipeline);
            compute_pass.set_bind_group(0, &self.texture_bind_group, &[]);
            compute_pass.set_bind_group(1, &self.render_bind_group, &[]);
            compute_pass.set_bind_group(2, &self.scene_bind_group, &[]);
            let (x, y) = (
                self.config.width.div_ceil(16),
                self.config.height.div_ceil(16),
            );

            compute_pass.dispatch_workgroups(x, y, 1);
        }

        self.hdr.process(&mut encoder, &view);

        // submit will accept anything that implements IntoIter
        self.queue.submit(std::iter::once(encoder.finish()));
        output.present();

        Ok(())
    }
}
