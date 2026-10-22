use std::{collections::HashMap, range::Range, time::Instant};

use common::units::{Milimeter, Milimeters};
use nalgebra::Vector3;
use slicer::geometry::mesh::{Mesh, MeshBuilder};
use tracing::info;

use crate::supports::{
    BaseConfig, BaseShape, Part, PartKind, SupportId, SupportSet,
    raft::{RaftConfig, RaftShape},
};

pub type FaceMap = HashMap<SupportId, Range<u32>>;

#[derive(Clone, PartialEq)]
pub struct SupportPreset {
    pub radius: Milimeters,
    pub margin: Milimeters,

    pub tip_radius: Milimeters,
    pub tip_length: Milimeters,
    /// How far the tip sinks into the model at the contact, in millimetres
    /// (§2.2 step 2, §3.1: 0.3 / 0.4 / 0.6 for Light / Middle / Heavy).
    pub contact_depth: Milimeters,
    /// How far a model-landing tail sinks into the lower surface along its
    /// axis (§2.4 step 5, §3.1 `supporttailtouchdepth`: 0.2 / 0.2 / 0.3).
    pub tail_depth: Milimeters,
    /// Radius of the slim (small) pillar (§3.1 `supportmiddlesmallpillardiameter`
    /// 0.4 / 0.5 / 0.5 Ø). A model-landing tail leans at most 2 × slim Ø
    /// sideways (§2.4 step 4).
    pub slim_radius: Milimeters,
    /// How far a slim pillar's lower end sinks into the model (§3.1.1 Q3,
    /// `supportmiddlesmallpillarlowercontactdepth`: 0.25 / 0.3 / 0.3).
    pub slim_depth: Milimeters,

    /// Base under each plate pillar (§3.1, §3.2 `supportbottomshape`).
    pub base_shape: BaseShape,
    /// Base radius, half of `supportbottomdiameter` (10 / 12 / 12 Ø).
    pub base_radius: Milimeters,
    /// `supportbottomthickness` (0.8 / 1.0 / 1.0).
    pub base_thickness: Milimeters,
    /// `heightofboss` (1.0, all presets).
    pub boss_height: Milimeters,
    /// Half of `diameterofbossbottom` (2.2 Ø, all presets).
    pub boss_radius: Milimeters,

    /// Raft under the plate pillars (§3.3, §8.1 `raftshape`: None / None /
    /// Skate). The rest are the shared §8.2 raft values.
    pub raft_shape: RaftShape,
    /// `raftarearatio`, as a factor (110 % = 1.1).
    pub raft_area_ratio: f32,
    pub raft_thickness: Milimeters,
    pub raft_height: Milimeters,
    /// `raftangle`, degrees.
    pub raft_angle: f32,
    pub raft_grid_cell: Milimeters,
    pub raft_grid_width: Milimeters,
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

    if let Some(raft) = &supports.raft {
        raft.mesh(&mut builder, |v| to_world(v, transform));
    }

    (!builder.is_empty()).then(|| {
        let mesh = builder.build();
        let (faces, duration) = (mesh.face_count(), start.elapsed());
        info!("Generated support mesh with {faces} faces in {duration:?}");
        (mesh, map)
    })
}

/// Applies the world transform to a model-local point.
fn to_world(point: Vector3<f32>, transform: Transform) -> Vector3<f32> {
    let mut point = point.component_mul(&transform.scale);
    point.z += transform.position.z;
    point
}

/// Applies the world transform to a point and meshes a single part.
///
/// Note: this mirrors the pre-SupportSet transform exactly (scale then a
/// z-translation). It is known to be incomplete — see `//rotate!?` in the
/// history — but fixing the support/mesh space relationship is out of scope
/// for M1 and would change rendered output.
fn add_part(builder: &mut MeshBuilder, part: &Part, transform: Transform, resolution: u32) {
    let (a, b) = (to_world(part.a, transform), to_world(part.b, transform));

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
        contact_depth: Milimeters::new(0.3),
        tail_depth: Milimeters::new(0.2),
        slim_radius: Milimeters::new(0.2),
        slim_depth: Milimeters::new(0.25),
        base_shape: BaseShape::Skate,
        base_radius: Milimeters::new(5.0),
        base_thickness: Milimeters::new(0.8),
        boss_height: Milimeters::new(1.0),
        boss_radius: Milimeters::new(1.1),
        raft_shape: RaftShape::None,
        raft_area_ratio: 1.1,
        raft_thickness: Milimeters::new(1.0),
        raft_height: Milimeters::new(1.8),
        raft_angle: 30.0,
        raft_grid_cell: Milimeters::new(2.0),
        raft_grid_width: Milimeters::new(2.0),
    };

    pub const MDEIUM: Self = Self {
        radius: Milimeters::new(0.6),
        margin: Milimeters::new(0.5),
        tip_radius: Milimeters::new(0.2),
        tip_length: Milimeters::new(2.0),
        contact_depth: Milimeters::new(0.4),
        tail_depth: Milimeters::new(0.2),
        slim_radius: Milimeters::new(0.25),
        slim_depth: Milimeters::new(0.3),
        base_shape: BaseShape::Skate,
        base_radius: Milimeters::new(6.0),
        base_thickness: Milimeters::new(1.0),
        boss_height: Milimeters::new(1.0),
        boss_radius: Milimeters::new(1.1),
        raft_shape: RaftShape::None,
        raft_area_ratio: 1.1,
        raft_thickness: Milimeters::new(1.0),
        raft_height: Milimeters::new(1.8),
        raft_angle: 30.0,
        raft_grid_cell: Milimeters::new(2.0),
        raft_grid_width: Milimeters::new(2.0),
    };

    pub const HEAVY: Self = Self {
        radius: Milimeters::new(0.8),
        margin: Milimeters::new(0.5),
        tip_radius: Milimeters::new(0.2),
        tip_length: Milimeters::new(2.0),
        contact_depth: Milimeters::new(0.6),
        tail_depth: Milimeters::new(0.3),
        slim_radius: Milimeters::new(0.25),
        slim_depth: Milimeters::new(0.3),
        base_shape: BaseShape::Skate,
        base_radius: Milimeters::new(6.0),
        base_thickness: Milimeters::new(1.0),
        boss_height: Milimeters::new(1.0),
        boss_radius: Milimeters::new(1.1),
        raft_shape: RaftShape::Skate,
        raft_area_ratio: 1.1,
        raft_thickness: Milimeters::new(1.0),
        raft_height: Milimeters::new(1.8),
        raft_angle: 30.0,
        raft_grid_cell: Milimeters::new(2.0),
        raft_grid_width: Milimeters::new(2.0),
    };
}

impl SupportPreset {
    /// The plate-pillar foot settings (§3.2).
    pub fn base_config(&self) -> BaseConfig {
        BaseConfig {
            shape: self.base_shape,
            radius: self.base_radius.get::<Milimeter>(),
            thickness: self.base_thickness.get::<Milimeter>(),
            boss_height: self.boss_height.get::<Milimeter>(),
            boss_radius: self.boss_radius.get::<Milimeter>(),
        }
    }

    /// The raft settings (§3.3). A raft takes each base's footprint.
    pub fn raft_config(&self) -> RaftConfig {
        RaftConfig {
            shape: self.raft_shape,
            foot_radius: self.base_radius.get::<Milimeter>(),
            area_ratio: self.raft_area_ratio,
            thickness: self.raft_thickness.get::<Milimeter>(),
            height: self.raft_height.get::<Milimeter>(),
            angle: self.raft_angle,
            grid_cell: self.raft_grid_cell.get::<Milimeter>(),
            grid_width: self.raft_grid_width.get::<Milimeter>(),
        }
    }
}

impl Default for SupportPreset {
    fn default() -> Self {
        Self::MDEIUM
    }
}
