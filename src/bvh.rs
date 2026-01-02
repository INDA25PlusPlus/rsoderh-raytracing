//! Implementation of a bounding volume hierarchy.
//! Based on code from this guide:
//! https://www.pbr-book.org/3ed-2018/Primitives_and_Intersection_Acceleration/Bounding_Volume_Hierarchies
//!
//! I don't understand the algorithm well, so the code isn't pretty.

use std::ops::Range;

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

#[derive(Clone)]
struct PrimitiveInfo {
    /// Which object array this primitive is in. 0 for circles, 1 for planes, and 2 for triangles.
    primitive_type: u32,
    /// The index of this primitive in its associated array.
    index: u32,
    primitive: Box<dyn Hittable>,
}

impl PrimitiveInfo {
    pub fn build_array(scene: &Scene) -> Vec<Self> {
        let mut result = Vec::new();
        result.extend(
            scene
                .spheres
                .iter()
                .enumerate()
                .map(|(index, sphere)| Self {
                    primitive_type: 0,
                    index: index as u32,
                    primitive: Box::new(sphere.clone()),
                }),
        );
        result.extend(scene.planes.iter().enumerate().map(|(index, plane)| Self {
            primitive_type: 1,
            index: index as u32,
            primitive: Box::new(plane.clone()),
        }));
        result.extend(
            scene
                .meshes
                .hittable_triangles()
                .into_iter()
                .enumerate()
                .map(|(index, triangle)| Self {
                    primitive_type: 2,
                    index: index as u32,
                    primitive: Box::new(triangle),
                }),
        );

        result
    }
}

impl Hittable for PrimitiveInfo {
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

    fn build(primitives: &mut [PrimitiveInfo]) -> Vec<BvhNodeUniform> {
        let mut ordered_primitives = Vec::new();
        let root = build_sah(primitives, &mut ordered_primitives);

        fn tree_depth(node: &BvhBuildNode) -> u32 {
            match node {
                BvhBuildNode::Leaf { .. } => 0,
                BvhBuildNode::Node { children, .. } => {
                    let depth_a = tree_depth(&children[0]);
                    let depth_b = tree_depth(&children[1]);

                    return depth_a.max(depth_b) + 1;
                }
            }
        }
        fn tree_node_count(node: &BvhBuildNode) -> u32 {
            match node {
                BvhBuildNode::Leaf { .. } => 1,
                BvhBuildNode::Node { children, .. } => {
                    tree_node_count(&children[0]) + tree_node_count(&children[1]) + 1
                }
            }
        }
        let depth = tree_depth(&root);
        log::info!("tree depth: {}", depth);
        let node_count = tree_node_count(&root);
        log::info!("tree node count: {}", node_count);

        primitives.clone_from_slice(&mut ordered_primitives);

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

fn build_sah(
    primitives: &mut [PrimitiveInfo],
    ordered_primitives: &mut Vec<PrimitiveInfo>,
) -> BvhBuildNode {
    const MAX_PRIMITIVES_PER_LEAF: usize = 5; // tune (8 or 16 often good)
    const BUCKET_COUNT: usize = 12;

    assert!(!primitives.is_empty());

    // Bounds of all primitives
    let bounds = Bounds3::from_bounds(primitives.iter().map(|p| p.bounds()));

    if primitives.len() <= MAX_PRIMITIVES_PER_LEAF {
        let first_index = ordered_primitives.len();
        ordered_primitives.extend(primitives.iter().cloned());
        return BvhBuildNode::new_leaf(first_index..ordered_primitives.len(), bounds);
    }

    // Centroid bounds (used to pick axis + bucket mapping)
    let center_bounds = Bounds3::from_points(primitives.iter().map(|p| p.bounds().center()));
    let max_axis = center_bounds.max_axis();

    // If degenerate along axis, just make a leaf (or do a median split fallback)
    let min_c = center_bounds.min[max_axis];
    let max_c = center_bounds.max[max_axis];
    if min_c == max_c {
        let first = ordered_primitives.len();
        ordered_primitives.extend(primitives.iter().cloned());
        return BvhBuildNode::new_leaf(first..ordered_primitives.len(), bounds);
    }

    #[derive(Clone, Copy)]
    struct BucketInfo {
        count: usize,
        bounds: Bounds3,
    }

    let mut buckets = [BucketInfo {
        count: 0,
        bounds: Bounds3::identity(),
    }; BUCKET_COUNT];

    // Computes bucket index of a primitive by its centroid.
    let bucket_index_of = |primitive: &PrimitiveInfo| -> usize {
        let center = primitive.bounds().center()[max_axis];

        let mut bucket_index = (BUCKET_COUNT as f32
            * ((center - center_bounds.min[max_axis])
                / (center_bounds.max[max_axis] - center_bounds.min[max_axis])))
            as usize;
        if bucket_index == BUCKET_COUNT {
            bucket_index = BUCKET_COUNT - 1;
        }
        bucket_index
    };

    // Fill buckets
    for primitive in primitives.iter() {
        let index = bucket_index_of(primitive);
        buckets[index].count += 1;
        buckets[index].bounds = &buckets[index].bounds | &primitive.bounds();
    }

    // Evaluate SAH costs for all bucket splits.
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

    // Partition prims in place so that all primitives below `split_index` belong in the bucket
    // `min_cost_index` or lower.
    let mut split_index = 0;
    {
        let mut end_index = primitives.len();
        while split_index < end_index {
            if bucket_index_of(&primitives[split_index]) <= min_cost_index {
                split_index += 1;
            } else {
                end_index -= 1;
                primitives.swap(split_index, end_index);
            }
        }
    }

    if split_index == 0 || split_index == primitives.len() {
        // Partitioning failed and all primitives are on one side, fall back to median split.
        let mid = primitives.len() / 2;
        primitives.select_nth_unstable_by(mid, |a, b| {
            a.bounds().center()[max_axis]
                .partial_cmp(&b.bounds().center()[max_axis])
                .unwrap()
        });
        split_index = mid;
    }

    let (left, right) = primitives.split_at_mut(split_index);

    BvhBuildNode::new_interior(
        max_axis,
        [
            build_sah(left, ordered_primitives),
            build_sah(right, ordered_primitives),
        ],
    )
}
