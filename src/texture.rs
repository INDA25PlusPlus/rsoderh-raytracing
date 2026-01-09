use std::io::Read;

use image::{ImageDecoder, codecs::hdr::HdrDecoder};

use crate::asset;

pub struct Texture {
    pub texture: wgpu::Texture,
    pub view: wgpu::TextureView,
    pub sampler: wgpu::Sampler,
    pub size: wgpu::Extent3d,
}

impl Texture {
    pub const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

    pub(crate) fn create_2d_texture(
        device: &wgpu::Device,
        width: u32,
        height: u32,
        format: wgpu::TextureFormat,
        usage: wgpu::TextureUsages,
        mag_filter: wgpu::FilterMode,
        label: Option<&str>,
    ) -> Self {
        let size = wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        };
        Self::create_texture(
            device,
            label,
            size,
            format,
            usage,
            wgpu::TextureDimension::D2,
            mag_filter,
        )
    }

    pub fn create_texture(
        device: &wgpu::Device,
        label: Option<&str>,
        size: wgpu::Extent3d,
        format: wgpu::TextureFormat,
        usage: wgpu::TextureUsages,
        dimension: wgpu::TextureDimension,
        mag_filter: wgpu::FilterMode,
    ) -> Self {
        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label,
            size,
            mip_level_count: 1,
            sample_count: 1,
            dimension,
            format,
            usage,
            view_formats: &[],
        });

        let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            mag_filter,
            min_filter: wgpu::FilterMode::Nearest,
            mipmap_filter: wgpu::FilterMode::Nearest,
            ..Default::default()
        });

        Self {
            texture,
            view,
            sampler,
            size,
        }
    }

    pub fn from_hdr(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        data: impl Read,
        label: &str,
    ) -> anyhow::Result<Self> {
        let hdr_decoder = HdrDecoder::new(data)?;
        let meta = hdr_decoder.metadata();

        // HDR images are always in this format.
        let format = wgpu::TextureFormat::Rgba32Float;
        let pixels = {
            let pixel_count = meta.width as usize * meta.height as usize;
            // The image's pixels bytes in Rgb32Float format and row major order.
            let mut pixels =
                vec![0; pixel_count * hdr_decoder.color_type().bytes_per_pixel() as usize];
            hdr_decoder.read_image(&mut pixels)?;

            let pixels_rgb: &[[f32; 3]] = bytemuck::cast_slice(&pixels);

            // Convert Rgb32Float to Rgba32Float.
            let mut pixels_rgba = vec![[0., 0., 0., 0.]; pixel_count];
            for (i, dest) in pixels_rgba.iter_mut().enumerate() {
                dest[..3].copy_from_slice(&pixels_rgb[i]);
            }

            pixels_rgba
        };

        // Create source texture and load HDR image into it.
        let result = Texture::create_2d_texture(
            device,
            meta.width,
            meta.height,
            format,
            wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            wgpu::FilterMode::Linear,
            Some(label),
        );
        queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture: &result.texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            bytemuck::cast_slice(&pixels),
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(
                    meta.width * image::ColorType::Rgba32F.bytes_per_pixel() as u32,
                ),
                rows_per_image: Some(meta.height),
            },
            result.size,
        );

        Ok(result)
    }
}

pub struct CubeTexture {
    texture: wgpu::Texture,
    view: wgpu::TextureView,
}

#[allow(unused)]
impl CubeTexture {
    /// Convert a HDR image containing an equirectangular environment map to a cube texture.
    /// `size` referers to the width and height of the result texture's faces in pixels.
    pub fn from_equirectangular_hdr(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        data: impl Read,
        size: u32,
        label: &str,
    ) -> anyhow::Result<CubeTexture> {
        let src = Texture::from_hdr(
            device,
            queue,
            data,
            "CubeTexture::from_equirectangular_hdr::src",
        )?;
        let format = src.texture.format();

        let dest_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some(label),
            size: wgpu::Extent3d {
                width: size,
                height: size,
                // A cube has six faces.
                depth_or_array_layers: 6,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format,
            usage: wgpu::TextureUsages::STORAGE_BINDING | wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        });
        let dest_storage_view = dest_texture.create_view(&wgpu::TextureViewDescriptor {
            label: Some(label),
            // Normally you'd use `TextureViewDimension::Cube`, but we can't use that with a
            // STORAGE_BINDING.
            dimension: Some(wgpu::TextureViewDimension::D2Array),
            ..Default::default()
        });

        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("CubeTexture::from_equirectangular_hdr::bind_group_layout"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: false },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::StorageTexture {
                        access: wgpu::StorageTextureAccess::WriteOnly,
                        format,
                        view_dimension: wgpu::TextureViewDimension::D2Array,
                    },
                    count: None,
                },
            ],
        });

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("CubeTexture::from_equirectangular_hdr::bind_group"),
            layout: &bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&src.view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::TextureView(&dest_storage_view),
                },
            ],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("CubeTexture::from_equirectangular_hdr::pipeline_layout"),
            bind_group_layouts: &[&bind_group_layout],
            push_constant_ranges: &[],
        });

        let shader = device
            .create_shader_module(asset::include_wgsl!("shaders/equirectangular_to_cube.wgsl"));

        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("CubeTexture::from_equirectangular_hdr::pipeline"),
            layout: Some(&pipeline_layout),
            module: &shader,
            entry_point: None,
            compilation_options: Default::default(),
            cache: None,
        });

        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Render Encoder"),
        });

        {
            let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("Compute Pass"),
                ..Default::default()
            });

            compute_pass.set_pipeline(&pipeline);
            compute_pass.set_bind_group(0, &bind_group, &[]);
            let num_workgroups = size.div_ceil(16);
            compute_pass.dispatch_workgroups(num_workgroups, num_workgroups, 6);
        }

        queue.submit([encoder.finish()]);

        // Create view stored in struct.
        let view = dest_texture.create_view(&wgpu::TextureViewDescriptor {
            label: Some(label),
            dimension: Some(wgpu::TextureViewDimension::Cube),
            array_layer_count: Some(6),
            ..Default::default()
        });

        Ok(Self {
            texture: dest_texture,
            view,
        })
    }

    #[allow(unused)]
    pub fn texture(&self) -> &wgpu::Texture {
        &self.texture
    }

    pub fn view(&self) -> &wgpu::TextureView {
        &self.view
    }
}
