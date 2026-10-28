use std::{collections::HashMap, range::Range, time::Instant};

use common::{
    serde::{Deserializer, Serializer},
    units::{Milimeter, Milimeters},
};
use nalgebra::Vector3;
use slicer::geometry::mesh::{Mesh, MeshBuilder};
use tracing::info;

use crate::supports::{
    BaseConfig, BaseShape, Part, PartKind, SupportId, SupportSet,
    raft::{RaftConfig, RaftShape},
};

pub type FaceMap = HashMap<SupportId, Range<u32>>;

/// Support sizes. Values in the docs are Light / Middle / Heavy.
#[derive(Clone, Debug, PartialEq)]
pub struct SupportPreset {
    /// Pillar radius (0.8 / 1.2 / 1.5 Ø).
    pub radius: Milimeters,
    /// Collision clearance added to the pillar radius (0.6).
    pub margin: Milimeters,

    /// Tip-cone radius at the contact (0.3 / 0.4 / 0.6 Ø).
    pub tip_radius: Milimeters,
    /// Tip-cone length (2 / 2 / 3).
    pub tip_length: Milimeters,
    /// How far the tip sinks into the model at the contact, in millimetres
    /// (0.3 / 0.4 / 0.6).
    pub contact_depth: Milimeters,
    /// How far a model-landing tail sinks into the lower surface along its
    /// axis (0.2 / 0.2 / 0.3).
    pub tail_depth: Milimeters,
    /// Radius of the slim (small) pillar (0.4 / 0.5 / 0.5 Ø). A model-landing
    /// tail leans at most 2 × slim Ø sideways.
    pub slim_radius: Milimeters,
    /// How far a slim pillar's lower end sinks into the model
    /// (0.25 / 0.3 / 0.3).
    pub slim_depth: Milimeters,

    /// Base under each plate pillar.
    pub base_shape: BaseShape,
    /// Base radius (10 / 12 / 12 Ø).
    pub base_radius: Milimeters,
    /// Base thickness (0.8 / 1.0 / 1.0).
    pub base_thickness: Milimeters,
    /// Boss height (1.0, all presets).
    pub boss_height: Milimeters,
    /// Boss bottom radius (2.2 Ø, all presets).
    pub boss_radius: Milimeters,

    /// Raft under the plate pillars (None / None / Skate). The rest are
    /// shared raft values.
    pub raft_shape: RaftShape,
    /// Raft area ratio, as a factor (110 % = 1.1).
    pub raft_area_ratio: f32,
    pub raft_thickness: Milimeters,
    pub raft_height: Milimeters,
    /// Skate wall angle, degrees.
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

/// Support parts (contacts, tips, joints, feet, raft outlines) are generated in
/// world space already — every routed coordinate above came from
/// `Mesh::transform`. So no further transform is applied here: re-applying the
/// model's scale or z-offset would double-count the buildplate offset and send
/// the support mesh bouncing away from the plate.
fn to_world(point: Vector3<f32>, _transform: Transform) -> Vector3<f32> {
    point
}

/// Meshes a single support part. Parts are world-space, so no transform is
/// applied beyond the (now-identity) `to_world`.
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
        margin: Milimeters::new(0.6),
        tip_radius: Milimeters::new(0.15),
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

    pub const MIDDLE: Self = Self {
        radius: Milimeters::new(0.6),
        margin: Milimeters::new(0.6),
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
        margin: Milimeters::new(0.6),
        tip_radius: Milimeters::new(0.3),
        tip_length: Milimeters::new(3.0),
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
    /// The built-in presets and their names.
    pub const PRESETS: [(&str, Self); 3] = [
        ("Light", Self::LIGHT),
        ("Middle", Self::MIDDLE),
        ("Heavy", Self::HEAVY),
    ];

    /// The name of the built-in preset these settings match, if any.
    pub fn name(&self) -> Option<&'static str> {
        (Self::PRESETS.iter()).find_map(|(name, preset)| (preset == self).then_some(*name))
    }

    /// The plate-pillar foot settings.
    pub fn base_config(&self) -> BaseConfig {
        BaseConfig {
            shape: self.base_shape,
            radius: self.base_radius.get::<Milimeter>(),
            thickness: self.base_thickness.get::<Milimeter>(),
            boss_height: self.boss_height.get::<Milimeter>(),
            boss_radius: self.boss_radius.get::<Milimeter>(),
        }
    }

    /// Takes the raft settings from `raft`; its foot radius comes from the
    /// base diameter, so it is left alone.
    pub fn set_raft(&mut self, raft: &RaftConfig) {
        self.raft_shape = raft.shape;
        self.raft_area_ratio = raft.area_ratio;
        self.raft_thickness = Milimeters::new(raft.thickness);
        self.raft_height = Milimeters::new(raft.height);
        self.raft_angle = raft.angle;
        self.raft_grid_cell = Milimeters::new(raft.grid_cell);
        self.raft_grid_width = Milimeters::new(raft.grid_width);
    }

    /// The raft settings. A raft takes each base's footprint.
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

/// Runs `$apply!(field)` for every length field of [`SupportPreset`], then
/// `$other!(field)` for every other field. The destructuring makes adding a
/// field without listing it here a compile error.
macro_rules! preset_fields {
    ($apply:ident, $other:ident) => {{
        #[allow(unused)]
        let SupportPreset {
            radius,
            margin,
            tip_radius,
            tip_length,
            contact_depth,
            tail_depth,
            slim_radius,
            slim_depth,
            base_shape,
            base_radius,
            base_thickness,
            boss_height,
            boss_radius,
            raft_shape,
            raft_area_ratio,
            raft_thickness,
            raft_height,
            raft_angle,
            raft_grid_cell,
            raft_grid_width,
        } = SupportPreset::MIDDLE;
        $apply!(radius);
        $apply!(margin);
        $apply!(tip_radius);
        $apply!(tip_length);
        $apply!(contact_depth);
        $apply!(tail_depth);
        $apply!(slim_radius);
        $apply!(slim_depth);
        $apply!(base_radius);
        $apply!(base_thickness);
        $apply!(boss_height);
        $apply!(boss_radius);
        $apply!(raft_thickness);
        $apply!(raft_height);
        $apply!(raft_grid_cell);
        $apply!(raft_grid_width);
        $other!(base_shape);
        $other!(raft_shape);
        $other!(raft_area_ratio);
        $other!(raft_angle);
    }};
}

impl SupportPreset {
    /// Copies onto `self` every setting that differs between `before` and
    /// `after`, leaving the rest as they are. Editing several supports at once
    /// changes just the edited setting on each.
    pub fn apply_changes(&mut self, before: &Self, after: &Self) {
        macro_rules! copy {
            ($field:ident) => {
                if before.$field != after.$field {
                    self.$field = after.$field;
                }
            };
        }
        preset_fields!(copy, copy);
    }

    pub fn serialize<T: Serializer>(&self, ser: &mut T) {
        macro_rules! length {
            ($field:ident) => {
                ser.write_f32_be(self.$field.get::<Milimeter>())
            };
        }
        macro_rules! skip {
            ($field:ident) => {};
        }
        preset_fields!(length, skip);
        ser.write_u8(self.base_shape.discriminant());
        ser.write_u8(self.raft_shape.discriminant());
        ser.write_f32_be(self.raft_area_ratio);
        ser.write_f32_be(self.raft_angle);
    }

    pub fn deserialize<T: Deserializer>(des: &mut T) -> Self {
        let mut preset = Self::MIDDLE;
        macro_rules! length {
            ($field:ident) => {
                preset.$field = Milimeters::new(des.read_f32_be())
            };
        }
        macro_rules! skip {
            ($field:ident) => {};
        }
        preset_fields!(length, skip);
        preset.base_shape = BaseShape::from_discriminant(des.read_u8());
        preset.raft_shape = RaftShape::from_discriminant(des.read_u8());
        preset.raft_area_ratio = des.read_f32_be();
        preset.raft_angle = des.read_f32_be();
        preset
    }
}

impl Default for SupportPreset {
    fn default() -> Self {
        Self::MIDDLE
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn apply_changes_copies_only_edited_settings() {
        let mut heavy = SupportPreset::HEAVY;
        let before = SupportPreset::LIGHT;
        let after = SupportPreset {
            tip_radius: Milimeters::new(0.33),
            base_shape: BaseShape::Cone,
            ..before.clone()
        };
        heavy.apply_changes(&before, &after);
        assert_eq!(
            heavy,
            SupportPreset {
                tip_radius: Milimeters::new(0.33),
                base_shape: BaseShape::Cone,
                ..SupportPreset::HEAVY
            }
        );
    }
}
