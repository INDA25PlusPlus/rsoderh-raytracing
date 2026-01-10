use std::f32::consts::PI;

use wgpu::util::DeviceExt;

use crate::texture::Texture;

/// Stores an array of environment HDRIs and their acceleration structures.
#[derive(Debug, Clone)]
pub struct EnvironmentMaps {
    textures: Box<[Texture]>,
    metadata: Box<[EnvironmentMetadata]>,
    metadata_buffer: wgpu::Buffer,
    // The concatenated environment's alias map's entries. Allows O(1) biased sampling.
    alias_maps: Box<[AliasEntry]>,
    alias_maps_buffer: wgpu::Buffer,
}

impl EnvironmentMaps {
    pub fn new(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        hdris: &[image::Rgb32FImage],
    ) -> anyhow::Result<Self> {
        let textures = hdris
            .into_iter()
            .map(|hdri| Texture::from_rgb32float(device, queue, hdri, "Environment Texture"))
            .collect::<Result<Box<[_]>, _>>()?;

        let mut alias_maps = Vec::new();
        let metadata = hdris
            .into_iter()
            .map(|hdri| {
                let result = EnvironmentMetadata {
                    width: hdri.width(),
                    height: hdri.height(),
                    alias_table_start_index: alias_maps.len() as u32,
                };
                let table = AliasTable::build_by_luminance(hdri);
                alias_maps.extend_from_slice(&table.entries);

                result
            })
            .collect::<Box<[_]>>();

        let metadata_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Environment metadata"),
            contents: &encase::StorageBuffer::<()>::content_of::<_, Vec<u8>>(&metadata)?,
            usage: wgpu::BufferUsages::STORAGE,
        });

        let alias_maps_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Environment alias maps"),
            contents: &encase::StorageBuffer::<()>::content_of::<_, Vec<u8>>(&alias_maps)?,
            usage: wgpu::BufferUsages::STORAGE,
        });

        Ok(Self {
            textures,
            metadata,
            metadata_buffer,
            alias_maps: alias_maps.into_boxed_slice(),
            alias_maps_buffer,
        })
    }

    pub fn textures(&self) -> &[Texture] {
        &self.textures
    }

    pub fn metadata(&self) -> &[EnvironmentMetadata] {
        &self.metadata
    }

    pub fn alias_maps(&self) -> &[AliasEntry] {
        &self.alias_maps
    }

    pub fn alias_maps_buffer(&self) -> &wgpu::Buffer {
        &self.alias_maps_buffer
    }

    pub fn metadata_buffer(&self) -> &wgpu::Buffer {
        &self.metadata_buffer
    }
}

/// Computes the "luminance" of a color.
fn luminance(color: image::Rgb<f32>) -> f32 {
    return 0.2126 * color.0[0] + 0.7152 * color.0[1] + 0.0722 * color.0[2];
}

struct AliasTable {
    entries: Box<[AliasEntry]>,
}

impl AliasTable {
    pub fn build_by_luminance(hdri: &image::Rgb32FImage) -> Self {
        let probability_weights = hdri
            .enumerate_pixels()
            .map(|(_, y, color)| {
                let angle_y = PI * (y as f32 + 0.5) / hdri.height() as f32;

                luminance(*color) * angle_y.sin()
            })
            .collect::<Box<[_]>>();

        // Length of the alias table, i.e. the amount of pixels in the source image.
        let length = probability_weights.len();

        let weight_sum: f32 = probability_weights.iter().sum();
        // Normalized probability weights. The average probability is 1.
        let probabilities = {
            let mut probabilities = probability_weights;
            for weight in &mut probabilities {
                *weight = *weight * length as f32 / weight_sum;
            }
            probabilities
        };
        let mut alias_probabilities = probabilities.clone();

        // Build table

        let mut small_indices = probabilities
            .iter()
            .enumerate()
            .filter_map(|(i, probability)| (*probability < 1.).then_some(i))
            .collect::<Vec<_>>();
        let mut large_indices = probabilities
            .iter()
            .enumerate()
            .filter_map(|(i, probability)| (*probability >= 1.).then_some(i))
            .collect::<Vec<_>>();

        let mut entries: Box<[Option<AliasEntry>]> = vec![None; length].into_boxed_slice();
        loop {
            let Some(small_index) = small_indices.pop() else {
                break;
            };
            let Some(large_index) = large_indices.pop() else {
                break;
            };

            let entry = AliasEntry {
                probability: alias_probabilities[small_index],
                alias_index: large_index as u32,
                pmf: probabilities[small_index] / length as f32,
                _pad: 0,
            };
            entries[small_index] = Some(entry);

            alias_probabilities[large_index] -= 1. - alias_probabilities[small_index];

            if alias_probabilities[large_index] < 1. {
                // Large index's probability is small enough to be assigned an entry. Push to stack.
                small_indices.push(large_index);
            } else {
                large_indices.push(large_index);
            }
        }

        // Handle entries which weren't assigned.
        let mut leftover_count = 0u32;
        let entries = entries
            .into_iter()
            .enumerate()
            .map(|(i, entry)| {
                entry.unwrap_or_else(|| {
                    leftover_count += 1;
                    // Assign default entry.
                    AliasEntry {
                        probability: 1.,
                        alias_index: i as u32,
                        pmf: 1. / length as f32,
                        _pad: 0,
                    }
                })
            })
            .collect::<Box<[_]>>();
        log::info!(
            "AliasTable: {} left over pixels out of {}",
            leftover_count,
            entries.len()
        );

        Self { entries }
    }
}

// Shader types

#[derive(Debug, Clone, encase::ShaderType)]
pub struct EnvironmentMetadata {
    width: u32,
    height: u32,
    /// Index in `environment_alias_maps` that this environment's alias table
    /// starts at. Its length is calculated as `self.width * self.height`.
    alias_table_start_index: u32,
}

#[derive(Debug, Clone, encase::ShaderType)]
pub struct AliasEntry {
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
