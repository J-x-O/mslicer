use std::{collections::HashMap, range::Range, time::Instant};

use common::{geometry::convex_hull, units::Milimeters};
use nalgebra::{Vector2, Vector3};
use slicer::geometry::mesh::{Mesh, MeshBuilder};
use tracing::info;

use crate::supports::{Part, PartKind, SupportId, SupportSet};

pub type FaceMap = HashMap<SupportId, Range<u32>>;

#[derive(Clone, PartialEq)]
pub struct SupportPreset {
    pub radius: Milimeters,
    pub margin: Milimeters,

    pub tip_radius: Milimeters,
    pub tip_length: Milimeters,
}

#[derive(Clone, Copy)]
pub struct Transform {
    pub position: Vector3<f32>,
    pub scale: Vector3<f32>,
    pub _rotation: f32,
}

/// Builds a render/slice mesh for the given support set, applying `transform`
/// to bring parts from model-local space into world space. Returns the mesh and
/// the face range owned by each support, keyed by id.
pub fn build_supports(
    supports: &SupportSet,
    transform: Transform,
    resolution: u32,
) -> Option<(Mesh, FaceMap)> {
    let start = Instant::now();
    let mut builder = MeshBuilder::new();
    let mut map = HashMap::new();

    for support in &supports.supports {
        let face_start = builder.next_face_idx();
        for part in &support.parts {
            add_part(&mut builder, part, transform, resolution);
        }
        let face_end = builder.next_face_idx();
        map.insert(support.id, Range::from(face_start..face_end));
    }

    for brace in &supports.braces {
        add_part(&mut builder, brace, transform, resolution);
    }

    // The raft is a catalogue of parts in model-local space (only the base
    // disc for now); meshed exactly like any other part.
    if let Some(raft) = &supports.raft {
        for part in raft {
            add_part(&mut builder, part, transform, resolution);
        }
    }

    // Legacy single-hull raft: one convex hull around every pillar foot. Kept
    // until the per-island raft (M5) replaces it. Reconstructed from the
    // vertical pillars so it matches the pre-SupportSet output exactly.
    let mut feet = Vec::new();
    for support in &supports.supports {
        for part in &support.parts {
            if part.kind == PartKind::Pillar {
                // Mirror the old transform: scale the xy, drop the z (the foot
                // sits at plate level), and use the part's outer radius.
                let xy = part.b.xy().component_mul(&transform.scale.xy());
                let r = part.radius_b;
                for i in 0..(resolution * 2) {
                    let angle = i as f32 / resolution as f32 * std::f32::consts::PI;
                    let normal = Vector2::new(angle.cos(), angle.sin());
                    feet.push(xy + normal * r);
                }
            }
        }
    }
    if !feet.is_empty() {
        build_raft(1.0, 1.0, &feet, &mut builder);
    }

    (!builder.is_empty()).then(|| {
        let mesh = builder.build();
        let (faces, duration) = (mesh.face_count(), start.elapsed());
        info!("Generated support mesh with {faces} faces in {duration:?}");
        (mesh, map)
    })
}

/// Applies the world transform to a point and meshes a single part.
///
/// Note: this mirrors the pre-SupportSet transform exactly (scale then a
/// z-translation). It is known to be incomplete — see `//rotate!?` in the
/// history — but fixing the support/mesh space relationship is out of scope
/// for M1 and would change rendered output.
fn add_part(builder: &mut MeshBuilder, part: &Part, transform: Transform, resolution: u32) {
    let mut a = part.a.component_mul(&transform.scale);
    let mut b = part.b.component_mul(&transform.scale);
    a.z += transform.position.z;
    b.z += transform.position.z;

    match part.kind {
        PartKind::ContactSphere | PartKind::Joint => {
            builder.add_sphere(a, part.radius_a, resolution);
        }
        PartKind::TipCone
        | PartKind::Link
        | PartKind::Pillar
        | PartKind::PillarLower
        | PartKind::Slim
        | PartKind::Boss
        | PartKind::Base
        | PartKind::Tail
        | PartKind::Brace => {
            builder.add_cylinder((a, b), (part.radius_a, part.radius_b), resolution);
        }
    }
}

// The legacy raft builder, kept for the single-plate convex-hull path until the
// real per-island raft lands in M5. Meshes one hull around every pillar foot.
pub fn build_raft(
    raft_offset: f32,
    raft_height: f32,
    points: &[Vector2<f32>],
    builder: &mut MeshBuilder,
) {
    let hull = convex_hull(points);
    let idx = builder.next_idx();
    for i in 0..hull.len() {
        let point = hull[i];
        let next = hull[(i + 1) % hull.len()];
        let prev = hull[(i + hull.len() - 1) % hull.len()];

        let edge_1 = next - point;
        let edge_2 = point - prev;
        let offset = (Vector2::new(edge_1.y, -edge_1.x).normalize()
            + Vector2::new(edge_2.y, -edge_2.x).normalize())
        .normalize();

        builder.add_vertex(point.push(0.0));
        builder.add_vertex((point + offset * raft_offset).push(raft_height));
    }

    let verts = builder.next_idx() - idx;
    for i in (0..verts).step_by(2) {
        if i != 0 && i + 3 < verts {
            builder.add_face([idx, idx + i + 2, idx + i]);
            builder.add_face([idx + 1, idx + i + 1, idx + i + 3]);
        }

        builder.add_quad([
            idx + i % verts,
            idx + (i + 1) % verts,
            idx + (i + 2) % verts,
            idx + (i + 3) % verts,
        ]);
    }
}

impl Default for Transform {
    fn default() -> Self {
        Self {
            position: Default::default(),
            scale: Vector3::repeat(1.0),
            _rotation: 0.0,
        }
    }
}

impl SupportPreset {
    pub const LIGHT: Self = Self {
        radius: Milimeters::new(0.4),
        margin: Milimeters::new(0.5),
        tip_radius: Milimeters::new(0.2),
        tip_length: Milimeters::new(2.0),
    };

    pub const MDEIUM: Self = Self {
        radius: Milimeters::new(0.6),
        margin: Milimeters::new(0.5),
        tip_radius: Milimeters::new(0.2),
        tip_length: Milimeters::new(2.0),
    };

    pub const HEAVY: Self = Self {
        radius: Milimeters::new(0.8),
        margin: Milimeters::new(0.5),
        tip_radius: Milimeters::new(0.2),
        tip_length: Milimeters::new(2.0),
    };
}

impl Default for SupportPreset {
    fn default() -> Self {
        Self::MDEIUM
    }
}
