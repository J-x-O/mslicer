//! Support parts and the [`SupportSet`], the geometry a support becomes after
//! routing.
//!
//! A routed support is still just three points (contact, tip end, joint), but
//! once generation finishes it is expanded into a [`SupportSet`]: a list of
//! typed [`Part`]s stacked from the plate up, and a foot recording where the
//! support lands. Meshes are generated per part so editing, slicing and the
//! raft stay independent of how the route was found.

use nalgebra::{Vector2, Vector3};

use common::serde::{Deserializer, SerdeExt, Serializer};

use crate::supports::SupportId;

/// How a support point was chosen. Coarse for now: routing (Stage 2 of the
/// plan) still produces bare contact points, and point classification only
/// arrives properly in M2. Manual covers hand-placed supports.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PointKind {
    /// A classified contact point; `depth` is the tip sink depth below the
    /// surface.
    Contact,
    /// A hand-placed support.
    Manual,
}

/// A contact point on the model together with the direction the tip leaves it.
#[derive(Clone, Copy, Debug)]
pub struct SupportPoint {
    pub pos: Vector3<f32>,
    pub normal: Vector3<f32>,
    pub kind: PointKind,
    /// How far the tip sinks into the model, in millimetres.
    pub depth: f32,
}

/// Where a support's lowest pillar ends up.
#[derive(Clone, Copy, Debug)]
pub enum Foot {
    /// Lands on the build plate at this xy position.
    Plate { xy: Vector2<f32> },
    /// Lands on the model at this position, along this normal.
    Model {
        pos: Vector3<f32>,
        normal: Vector3<f32>,
    },
}

/// The shape of a part's cross-section.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Shape {
    /// Circular cross-section (cylinder, cone, sphere).
    Round,
}

/// A single printable support part. A part is a swept segment from `a` to `b`
/// with an optionally tapered cross-section, plus the joint sphere shape where
/// relevant.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PartKind {
    /// Sphere sunk into the model at the contact point.
    ContactSphere,
    /// Tapered tip from the contact point to the tip end.
    TipCone,
    /// Sphere at the tip end / link ends.
    Joint,
    /// Cylinder between two joints.
    Link,
    /// Main vertical pillar.
    Pillar,
    /// Vertical pillar from `a` down to the plate (`b.z == 0`).
    PillarLower,
    /// Thin rod for short model-to-model gaps.
    Slim,
    /// Cone flaring from pillar to base.
    Boss,
    /// Disc at the plate.
    Base,
    /// Downward cone landing on a lower model surface.
    Tail,
    /// 45° cylinder between pillars.
    Brace,
}

/// A part: a segment from `a` to `b` swept with radii `radius_a` (at `a`) and
/// `radius_b` (at `b`).
#[derive(Clone, Copy, Debug)]
pub struct Part {
    pub kind: PartKind,
    pub a: Vector3<f32>,
    pub b: Vector3<f32>,
    pub radius_a: f32,
    pub radius_b: f32,
    pub shape: Shape,
}

/// A single support: its contact point, the parts that make it up (plate-up
/// order), and its foot.
#[derive(Clone, Debug)]
pub struct Support {
    pub id: SupportId,
    pub point: SupportPoint,
    pub parts: Vec<Part>,
    pub foot: Foot,
    /// Shared-trunk group id; unused until topology (M4), but part of the
    /// serialised form now so the format stays stable.
    pub group: u32,
}

/// Everything a generated set of supports consists of.
#[derive(Clone, Debug)]
pub struct SupportSet {
    pub supports: Vec<Support>,
    pub braces: Vec<Part>,
    /// The raft mesh, if any, in model-local space.
    pub raft: Option<Vec<Part>>,
    /// The lift (in millimetres) applied to the model while supports exist.
    pub lift: f32,
}

impl SupportSet {
    /// Adds a support built from three routed points: contact, tip end and
    /// joint. The pillar then drops vertically from the joint to the plate.
    ///
    /// `tip_radius` and `radius` are millimetres.
    pub fn add_routed(
        &mut self,
        id: SupportId,
        tip_radius: f32,
        radius: f32,
        [contact, tip, joint]: [Vector3<f32>; 3],
    ) {
        let foot_xy = joint.xy();
        let foot = Foot::Plate { xy: foot_xy };

        self.supports.push(Support {
            id,
            point: SupportPoint {
                pos: contact,
                normal: (tip - contact).normalize(),
                kind: PointKind::Contact,
                depth: 0.0,
            },
            parts: vec![
                Part {
                    kind: PartKind::ContactSphere,
                    a: contact,
                    b: contact,
                    radius_a: tip_radius,
                    radius_b: tip_radius,
                    shape: Shape::Round,
                },
                Part {
                    kind: PartKind::TipCone,
                    a: contact,
                    b: tip,
                    radius_a: tip_radius,
                    radius_b: radius,
                    shape: Shape::Round,
                },
                Part {
                    kind: PartKind::Link,
                    a: tip,
                    b: joint,
                    radius_a: radius,
                    radius_b: radius,
                    shape: Shape::Round,
                },
                Part {
                    kind: PartKind::Pillar,
                    a: joint,
                    b: joint.xy().push(0.0),
                    radius_a: radius,
                    radius_b: radius,
                    shape: Shape::Round,
                },
            ],
            foot,
            group: 0,
        });
    }
}

impl Default for SupportSet {
    fn default() -> Self {
        Self {
            supports: Vec::new(),
            braces: Vec::new(),
            raft: None,
            lift: 5.0,
        }
    }
}

// --------------------------------------------------------------------------
// Serialization. `SupportSet` owns its binary format so project storage stays
// thin. The shape grows by appending; the project file version guards against
// mismatched readers, so there is no in-band version here.
// --------------------------------------------------------------------------

impl PartKind {
    const fn discriminant(self) -> u8 {
        match self {
            PartKind::ContactSphere => 0,
            PartKind::TipCone => 1,
            PartKind::Joint => 2,
            PartKind::Link => 3,
            PartKind::Pillar => 4,
            PartKind::PillarLower => 5,
            PartKind::Slim => 6,
            PartKind::Boss => 7,
            PartKind::Base => 8,
            PartKind::Tail => 9,
            PartKind::Brace => 10,
        }
    }

    fn from_discriminant(x: u8) -> Self {
        match x {
            0 => PartKind::ContactSphere,
            1 => PartKind::TipCone,
            2 => PartKind::Joint,
            3 => PartKind::Link,
            4 => PartKind::Pillar,
            5 => PartKind::PillarLower,
            6 => PartKind::Slim,
            7 => PartKind::Boss,
            8 => PartKind::Base,
            9 => PartKind::Tail,
            _ => PartKind::Brace,
        }
    }
}

impl PointKind {
    const fn discriminant(self) -> u8 {
        match self {
            PointKind::Contact => 0,
            PointKind::Manual => 1,
        }
    }

    fn from_discriminant(x: u8) -> Self {
        match x {
            0 => PointKind::Contact,
            _ => PointKind::Manual,
        }
    }
}

impl Part {
    pub fn serialize<T: Serializer>(&self, ser: &mut T) {
        ser.write_u8(self.kind.discriminant());
        self.a.serialize(ser);
        self.b.serialize(ser);
        ser.write_f32_be(self.radius_a);
        ser.write_f32_be(self.radius_b);
        ser.write_u8(self.shape as u8);
    }

    pub fn deserialize<T: Deserializer>(des: &mut T) -> Self {
        let kind = PartKind::from_discriminant(des.read_u8());
        let a = Vector3::deserialize(des);
        let b = Vector3::deserialize(des);
        let radius_a = des.read_f32_be();
        let radius_b = des.read_f32_be();
        let shape = des.read_u8();
        Self {
            kind,
            a,
            b,
            radius_a,
            radius_b,
            shape: match shape {
                0 => Shape::Round,
                _ => Shape::Round,
            },
        }
    }
}

impl SupportPoint {
    pub fn serialize<T: Serializer>(&self, ser: &mut T) {
        self.pos.serialize(ser);
        self.normal.serialize(ser);
        ser.write_u8(self.kind.discriminant());
        ser.write_f32_be(self.depth);
    }

    pub fn deserialize<T: Deserializer>(des: &mut T) -> Self {
        Self {
            pos: Vector3::deserialize(des),
            normal: Vector3::deserialize(des),
            kind: PointKind::from_discriminant(des.read_u8()),
            depth: des.read_f32_be(),
        }
    }
}

impl Foot {
    pub fn serialize<T: Serializer>(&self, ser: &mut T) {
        match self {
            Foot::Plate { xy } => {
                ser.write_u8(0);
                xy.serialize(ser);
            }
            Foot::Model { pos, normal } => {
                ser.write_u8(1);
                pos.serialize(ser);
                normal.serialize(ser);
            }
        }
    }

    pub fn deserialize<T: Deserializer>(des: &mut T) -> Self {
        match des.read_u8() {
            0 => Foot::Plate {
                xy: Vector2::deserialize(des),
            },
            _ => Foot::Model {
                pos: Vector3::deserialize(des),
                normal: Vector3::deserialize(des),
            },
        }
    }
}

impl Support {
    pub fn serialize<T: Serializer>(&self, ser: &mut T) {
        ser.write_u32_be(self.id.raw());
        self.point.serialize(ser);
        self.foot.serialize(ser);
        ser.write_u32_be(self.parts.len() as u32);
        for part in &self.parts {
            part.serialize(ser);
        }
        ser.write_u32_be(self.group);
    }

    pub fn deserialize<T: Deserializer>(des: &mut T) -> Self {
        let id = SupportId::from_raw(des.read_u32_be());
        let point = SupportPoint::deserialize(des);
        let foot = Foot::deserialize(des);
        let parts = (0..des.read_u32_be())
            .map(|_| Part::deserialize(des))
            .collect();
        let group = des.read_u32_be();
        Self {
            id,
            point,
            parts,
            foot,
            group,
        }
    }
}

impl SupportSet {
    pub fn serialize<T: Serializer>(&self, ser: &mut T) {
        ser.write_f32_be(self.lift);

        ser.write_u32_be(self.supports.len() as u32);
        for support in &self.supports {
            support.serialize(ser);
        }

        ser.write_u32_be(self.braces.len() as u32);
        for brace in &self.braces {
            brace.serialize(ser);
        }

        match &self.raft {
            Some(raft) => {
                ser.write_bool(true);
                ser.write_u32_be(raft.len() as u32);
                for part in raft {
                    part.serialize(ser);
                }
            }
            None => ser.write_bool(false),
        }
    }

    pub fn deserialize<T: Deserializer>(des: &mut T) -> Self {
        let lift = des.read_f32_be();

        let supports = (0..des.read_u32_be())
            .map(|_| Support::deserialize(des))
            .collect();

        let braces = (0..des.read_u32_be())
            .map(|_| Part::deserialize(des))
            .collect();

        let raft = des.read_bool().then(|| {
            (0..des.read_u32_be())
                .map(|_| Part::deserialize(des))
                .collect()
        });

        Self {
            supports,
            braces,
            raft,
            lift,
        }
    }
}

#[cfg(test)]
mod tests {
    use common::serde::{SizedSerializer, SliceDeserializer};

    use super::*;

    #[test]
    fn support_set_roundtrips() {
        let mut set = SupportSet::default();
        set.add_routed(
            SupportId::from_raw(7),
            0.2,
            0.6,
            [
                Vector3::new(1.0, 2.0, 3.0),
                Vector3::new(1.1, 2.2, 5.0),
                Vector3::new(3.0, 4.0, 1.0),
            ],
        );
        set.add_routed(
            SupportId::from_raw(9),
            0.2,
            0.8,
            [
                Vector3::new(-1.0, 0.0, 2.0),
                Vector3::new(-1.0, 0.0, 4.0),
                Vector3::new(-2.0, 1.0, 0.5),
            ],
        );
        set.braces.push(Part {
            kind: PartKind::Brace,
            a: Vector3::new(0.0, 0.0, 0.0),
            b: Vector3::new(1.0, 0.0, 1.0),
            radius_a: 0.6,
            radius_b: 0.6,
            shape: Shape::Round,
        });
        set.raft = Some(vec![Part {
            kind: PartKind::Base,
            a: Vector3::new(0.0, 0.0, 0.0),
            b: Vector3::new(0.0, 0.0, 1.0),
            radius_a: 5.0,
            radius_b: 5.0,
            shape: Shape::Round,
        }]);

        let mut buffer = vec![0; 1024];
        {
            let mut ser = SizedSerializer::new(&mut buffer);
            set.serialize(&mut ser);
        }

        let mut des = SliceDeserializer::new(&buffer);
        let out = SupportSet::deserialize(&mut des);

        assert_eq!(out.lift, set.lift);
        assert_eq!(out.supports.len(), 2);
        assert_eq!(out.braces.len(), 1);
        assert!(out.raft.is_some());

        assert_eq!(out.supports[0].id.raw(), 7);
        assert_eq!(out.supports[1].id.raw(), 9);

        for (a, b) in out.supports.iter().zip(set.supports.iter()) {
            assert_eq!(a.parts.len(), b.parts.len());
            assert_eq!(a.point.pos, b.point.pos);
            assert_eq!(a.point.normal, b.point.normal);
            for (pa, pb) in a.parts.iter().zip(b.parts.iter()) {
                assert_eq!(pa.kind, pb.kind);
                assert_eq!(pa.a, pb.a);
                assert_eq!(pa.b, pb.b);
                assert_eq!(pa.radius_a, pb.radius_a);
                assert_eq!(pa.radius_b, pb.radius_b);
            }
        }
    }
}
