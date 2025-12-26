//! Implementation of a bounding volume hierarchy.
//! Based on code from this guide:
//! https://www.pbr-book.org/3ed-2018/Primitives_and_Intersection_Acceleration/Bounding_Volume_Hierarchies
//!
//! I don't understand the algorithm well, so the code isn't pretty.

use std::ops::Range;

use glam::Vec3;
use ordered_float::NotNan;

use crate::scene::{Axis, Bounds3, Hittable, Scene};

pub fn build_bvh(scene: &Scene) -> (Vec<PrimitiveInfoUniform>, Vec<BvhNodeUniform>) {
    let mut primitives = PrimitiveInfo::build_array(scene);

    let bvh_nodes = BvhNodeUniform::build(&mut primitives);

    (
        primitives
            .into_iter()
            .map(|info| PrimitiveInfoUniform {
                index: info.index,
                primitive_type: info.primitive_type,
            })
            .collect(),
        bvh_nodes,
    )
}

#[derive(Clone, Copy)]
struct PrimitiveInfo<'a> {
    /// Which object array this primitive is in. 0 for circles and 1 for planes.
    primitive_type: u32,
    /// The index of this primitive in its associated array.
    index: u32,
    primitive: &'a dyn Hittable,
}

impl<'a> PrimitiveInfo<'a> {
    pub fn build_array(scene: &'a Scene) -> Vec<Self> {
        let mut result = Vec::new();
        result.extend(
            scene
                .spheres
                .iter()
                .enumerate()
                .map(|(index, sphere)| Self {
                    primitive_type: 0,
                    index: index as u32,
                    primitive: sphere,
                }),
        );
        result.extend(scene.planes.iter().enumerate().map(|(index, plane)| Self {
            primitive_type: 1,
            index: index as u32,
            primitive: plane,
        }));

        result
    }
}

impl<'a> Hittable for PrimitiveInfo<'a> {
    fn bounds(&self) -> Bounds3 {
        self.primitive.bounds()
    }
}

#[derive(Debug, Clone, encase::ShaderType)]
pub struct PrimitiveInfoUniform {
    /// Which object array this primitive is in. 0 for spheres and 1 for planes.
    primitive_type: u32,
    /// The index of this primitive in its associated array.
    index: u32,
}

#[derive(Debug, Clone, Default, encase::ShaderType)]
pub struct BvhNodeUniform {
    bounds: Bounds3,
    /// If leaf node, stores the index its primitives start at in the primitives info array.
    /// If interior node, stores the index of the second child node. The first child node's index
    /// is equal to this node's index + 1.
    primitives_or_second_child_index: u32,
    /// The amount of primitives contained in this node if it's a leaf node, otherwise 0.
    primitives_len: u32,
    split_axis: u32,
}

impl BvhNodeUniform {
    fn new_leaf(bounds: Bounds3, primitive_indices: Range<usize>) -> Self {
        Self {
            bounds,
            primitives_or_second_child_index: primitive_indices.start as u32,
            primitives_len: primitive_indices.len() as u32,
            split_axis: 0,
        }
    }

    fn new_interior(bounds: Bounds3, second_child_index: u32, split_axis: Axis) -> Self {
        Self {
            bounds,
            primitives_or_second_child_index: second_child_index,
            primitives_len: 0,
            split_axis: u8::from(split_axis) as u32,
        }
    }

    fn build(primitives: &mut [PrimitiveInfo<'_>]) -> Vec<BvhNodeUniform> {
        let mut ordered_primitives = Vec::new();
        let mut total_nodes = 0;
        let root = hlbvh_build(primitives, &mut total_nodes, &mut ordered_primitives);
        primitives.copy_from_slice(&mut ordered_primitives);

        let mut linear_nodes = Vec::new();
        Self::flatten_build_node(&root, &mut linear_nodes);
        linear_nodes
    }

    fn flatten_build_node(root: &BvhBuildNode, linear_nodes: &mut Vec<BvhNodeUniform>) -> u32 {
        match root {
            BvhBuildNode::Leaf { bounds, primitives } => {
                linear_nodes.push(Self::new_leaf(*bounds, primitives.clone()));
                (linear_nodes.len() - 1) as u32
            }
            BvhBuildNode::Node {
                bounds,
                children,
                split_axis,
            } => {
                // Make sure that this node is inserted before its children.
                linear_nodes.push(BvhNodeUniform::new_interior(*bounds, 0, *split_axis));
                let parent_index = linear_nodes.len() - 1;

                // Ignore first child's index, as it's stored implicitly.
                let _ = Self::flatten_build_node(&children[0], linear_nodes);
                let second_child_index = Self::flatten_build_node(&children[1], linear_nodes);

                linear_nodes[parent_index].primitives_or_second_child_index = second_child_index;
                parent_index as u32
            }
        }
    }
}

#[derive(Debug, Clone)]
enum BvhBuildNode {
    Leaf {
        bounds: Bounds3,
        // Range of indices of the contained primitives from the associated ordered primitives list.
        primitives: Range<usize>,
    },
    Node {
        bounds: Bounds3,
        children: Box<[BvhBuildNode; 2]>,
        split_axis: Axis,
    },
}

impl BvhBuildNode {
    fn new_leaf(primitives: Range<usize>, bounds: Bounds3) -> Self {
        Self::Leaf { bounds, primitives }
    }
    fn new_interior(split_axis: Axis, children: [BvhBuildNode; 2]) -> Self {
        Self::Node {
            bounds: Bounds3::from_bounds(children.iter().map(|node| node.bounds())),
            children: Box::new(children),
            split_axis,
        }
    }

    fn bounds(&self) -> Bounds3 {
        match self {
            BvhBuildNode::Leaf { bounds, .. } => *bounds,
            BvhBuildNode::Node { bounds, .. } => *bounds,
        }
    }
}

#[derive(Debug, Clone)]
struct MortonPrimitive {
    /// The index of the primitive in the associated array.
    index: usize,
    morton_code: u32,
}

impl MortonPrimitive {
    fn new(index: usize, primitives: &[PrimitiveInfo<'_>], bounds: &Bounds3) -> Self {
        const MORTON_BITS: u32 = 10;
        const MORTON_SCALE: u32 = 1 << MORTON_BITS;

        let center_offset = bounds.to_relative(primitives[index].bounds().center());
        let morton_code = Self::encode_morton(center_offset * MORTON_SCALE as f32);

        Self { index, morton_code }
    }
    /// I hate this name, but I don't understand the function well enough to make a better one.
    fn left_shift_3(mut x: u32) -> u32 {
        if x == (1 << 10) {
            x -= 1;
        }
        x = (x | (x << 16)) & 0b00000011000000000000000011111111;
        x = (x | (x << 8)) & 0b00000011000000001111000000001111;
        x = (x | (x << 4)) & 0b00000011000011000011000011000011;
        x = (x | (x << 2)) & 0b00001001001001001001001001001001;
        return x;
    }

    /// Takes in a vector of floats in the range 0..2^10.
    fn encode_morton(vector: Vec3) -> u32 {
        (Self::left_shift_3(vector.z as u32) << 2)
            | (Self::left_shift_3(vector.y as u32) << 1)
            | Self::left_shift_3(vector.x as u32)
    }
}

fn hlbvh_build<'a>(
    primitives: &[PrimitiveInfo<'a>],
    total_nodes: &mut usize,
    ordered_primitives: &mut Vec<PrimitiveInfo<'a>>,
) -> BvhBuildNode {
    let centers_bounds = Bounds3::from_points(
        primitives
            .iter()
            .map(|primitive| primitive.bounds().center()),
    );

    let mut morton_primitives = (0..primitives.len())
        .map(|index| MortonPrimitive::new(index, primitives, &centers_bounds))
        .collect::<Box<[_]>>();
    morton_primitives.sort_by_key(|morton| morton.morton_code);

    let mut treelets = Vec::new();
    let mut start = 0;
    for end in 1..=morton_primitives.len() {
        /// Funny magic mask
        const MASK: u32 = 0b00111111111111000000000000000000;

        if end == morton_primitives.len()
            || (morton_primitives[start].morton_code & MASK)
                != (morton_primitives[end].morton_code & MASK)
        {
            const FIRST_BIT_INDEX: i8 = 29 - 12;

            treelets.push(emit_lbvh(
                primitives,
                &morton_primitives[start..end],
                total_nodes,
                ordered_primitives,
                FIRST_BIT_INDEX,
            ));

            start = end;
        }
    }

    return build_upper_sah(treelets);
}

fn emit_lbvh<'a>(
    // build_nodes: &mut Vec<BvhBuildNode>,
    primitives: &[PrimitiveInfo<'a>],
    morton_primitives: &[MortonPrimitive],
    total_nodes: &mut usize,
    ordered_primitives: &mut Vec<PrimitiveInfo<'a>>,
    bit_index: i8,
) -> BvhBuildNode {
    const MAX_PRIMITIVES_PER_NODE: usize = 5;

    if bit_index == -1 || morton_primitives.len() < MAX_PRIMITIVES_PER_NODE {
        *total_nodes += 1;
        let bounds = Bounds3::from_bounds(
            morton_primitives
                .iter()
                .map(|morton_primitive| primitives[morton_primitive.index].bounds()),
        );
        let first_primitive_index = ordered_primitives.len();
        ordered_primitives.extend(
            morton_primitives
                .iter()
                .map(|morton_primitive| primitives[morton_primitive.index]),
        );
        return BvhBuildNode::new_leaf(first_primitive_index..ordered_primitives.len(), bounds);
    } else {
        let mask = 1 << bit_index;
        if (morton_primitives[0].morton_code & mask)
            == (morton_primitives
                .last()
                .expect("There is at least one primitive")
                .morton_code
                & mask)
        {
            return emit_lbvh(
                primitives,
                morton_primitives,
                total_nodes,
                ordered_primitives,
                bit_index - 1,
            );
        }

        // TODO: Rewrite using morton_primitives.binary_search_by(f)
        let mut search_start = 0;
        let mut search_end = morton_primitives.len() - 1;
        while search_start + 1 != search_end {
            let mid = (search_start + search_end) / 2;
            if (morton_primitives[search_start].morton_code & mask)
                == (morton_primitives[search_end].morton_code & mask)
            {
                search_start = mid;
            } else {
                search_end = mid;
            }
        }
        let split_index = search_end;

        let children = [
            emit_lbvh(
                primitives,
                &morton_primitives[..split_index],
                total_nodes,
                ordered_primitives,
                bit_index - 1,
            ),
            emit_lbvh(
                primitives,
                &morton_primitives[split_index..],
                total_nodes,
                ordered_primitives,
                bit_index - 1,
            ),
        ];
        // bit_index should not be negative at this point (I think...)
        let axis = Axis::from(bit_index as u8);
        BvhBuildNode::new_interior(axis, children)
    }
}

fn build_upper_sah(mut treelets: Vec<BvhBuildNode>) -> BvhBuildNode {
    // This should be true, right?
    assert!(treelets.len() != 0);

    if treelets.len() == 1 {
        return treelets.swap_remove(0);
    }

    let bounds = Bounds3::from_bounds(treelets.iter().map(|node| node.bounds()));

    let center_bounds = Bounds3::from_points(treelets.iter().map(|node| node.bounds().center()));

    let max_axis = center_bounds.max_axis();
    assert_ne!(center_bounds.min[max_axis], center_bounds.max[max_axis]);

    #[derive(Clone, Copy)]
    struct BucketInfo {
        count: usize,
        bounds: Bounds3,
    }
    const BUCKET_COUNT: usize = 12;
    let mut buckets = [BucketInfo {
        count: 0,
        bounds: Bounds3::identity(),
    }; BUCKET_COUNT];

    for treelet in &treelets {
        let center = (treelet.bounds().min[max_axis] + treelet.bounds().max[max_axis]) * 0.5;
        let mut bucket_index = (BUCKET_COUNT as f32
            * ((center - center_bounds.min[max_axis])
                / (center_bounds.max[max_axis] - center_bounds.min[max_axis])))
            as usize;
        if bucket_index == BUCKET_COUNT {
            bucket_index = BUCKET_COUNT - 1;
        }
        assert!(bucket_index < BUCKET_COUNT);

        buckets[bucket_index].count += 1;
        buckets[bucket_index].bounds = &buckets[bucket_index].bounds | &treelet.bounds();
    }

    let mut costs = [0.0; BUCKET_COUNT - 1];
    for index in 0..(BUCKET_COUNT - 1) {
        let (buckets_0, buckets_1) = buckets.split_at(index + 1);

        let bounds_0 = Bounds3::from_bounds(buckets_0.iter().map(|bucket| bucket.bounds));
        let count_0: usize = buckets_0.iter().map(|bucket| bucket.count).sum();

        let bounds_1 = Bounds3::from_bounds(buckets_1.iter().map(|bucket| bucket.bounds));
        let count_1: usize = buckets_1.iter().map(|bucket| bucket.count).sum();

        costs[index] = 0.125
            + (count_0 as f32 * bounds_0.surface_area() + count_1 as f32 * bounds_1.surface_area())
                / bounds.surface_area();
    }

    let min_cost_index = costs
        .iter()
        .map(|cost| NotNan::new(*cost).expect("bounds have non-zero surface area"))
        .enumerate()
        .min_by_key(|(_, cost)| *cost)
        .map(|(i, _)| i)
        .expect("costs has length > 0");

    let (treelets_0, treelets_1): (Vec<BvhBuildNode>, Vec<BvhBuildNode>) =
        treelets.into_iter().partition(|treelet| {
            let center = (treelet.bounds().min[max_axis] + treelet.bounds().max[max_axis]) * 0.5;

            let mut bucket_index = (BUCKET_COUNT as f32
                * ((center - center_bounds.min[max_axis])
                    / (center_bounds.max[max_axis] - center_bounds.min[max_axis])))
                as usize;
            if bucket_index == BUCKET_COUNT {
                bucket_index = BUCKET_COUNT - 1;
            }
            assert!(bucket_index < BUCKET_COUNT);

            bucket_index <= min_cost_index
        });

    if treelets_0.len() == 0 || treelets_1.len() == 0 {
        dbg!(treelets_0.len(), treelets_1.len());
        dbg!(&treelets_0, &treelets_1);
    }
    assert!(treelets_0.len() != 0);
    assert!(treelets_1.len() != 0);

    BvhBuildNode::new_interior(
        max_axis,
        [build_upper_sah(treelets_0), build_upper_sah(treelets_1)],
    )
}
