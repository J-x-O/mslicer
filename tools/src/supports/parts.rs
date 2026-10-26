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

use crate::supports::{
    SupportId,
    auto::{Landing, LandingKind},
    raft::{Raft, RaftConfig},
};

/// How a support point was chosen: automatic placement or by hand.
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

/// Pillars whose length plus 0.6 × the tip-cone length is below this get a
/// short foot: a base but no boss.
pub const SHORT_PILLAR: f32 = 5.0;
/// Stacked parts overlap by this much at each joint so faces don't coincide.
pub const JOINT_SHRINK: f32 = 0.002;
/// Wall angle of a Skate base, from the plate; the same as the Skate raft's
/// wall.
const SKATE_ANGLE: f32 = 30.0;

/// The base under a plate pillar. Cube and Prism
/// are meshed round until parts carry other cross-sections.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BaseShape {
    /// No base or boss.
    None,
    /// A disc whose outer wall slopes in at [`SKATE_ANGLE`].
    Skate,
    /// A frustum from the base diameter down to the boss bottom diameter.
    Cone,
    /// A flat disc.
    Cylinder,
}

/// How plate pillars stand on the plate. Lengths are millimetres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BaseConfig {
    pub shape: BaseShape,
    /// Base radius (10 / 12 / 12 Ø for Light / Middle / Heavy).
    pub radius: f32,
    /// Base thickness (0.8 / 1.0 / 1.0).
    pub thickness: f32,
    /// Boss height (1.0).
    pub boss_height: f32,
    /// Boss bottom radius (2.2 Ø).
    pub boss_radius: f32,
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
    /// Shared-trunk group id; unused for now, but part of the serialised form
    /// so the format stays stable.
    pub group: u32,
}

/// Everything a generated set of supports consists of.
#[derive(Clone, Debug)]
pub struct SupportSet {
    pub supports: Vec<Support>,
    pub braces: Vec<Part>,
    /// The raft under the plate pillars, if any, in model-local space.
    pub raft: Option<Raft>,
    /// The lift (in millimetres) applied to the model while supports exist.
    pub lift: f32,
}

impl SupportSet {
    /// Adds a support built from three routed points: contact, tip end and
    /// joint. The pillar then drops vertically from the joint to the plate.
    ///
    /// `tip_radius` and `radius` are millimetres. `contact_depth` is how far the
    /// tip cone sinks into the model at the contact (the cone itself sinks in;
    /// no contact sphere is emitted).
    pub fn add_routed(
        &mut self,
        id: SupportId,
        tip_radius: f32,
        radius: f32,
        contact_depth: f32,
        [contact, tip, joint]: [Vector3<f32>; 3],
    ) {
        let foot_xy = joint.xy();
        let foot = Foot::Plate { xy: foot_xy };

        let normal = {
            let d = tip - contact;
            if d.magnitude_squared() > f32::EPSILON {
                d.normalize()
            } else {
                -Vector3::z()
            }
        };

        // The tip cone sinks into the model by `contact_depth` (there is no
        // contact sphere). Its near end is `contact − dir × depth`, into the
        // model; its far end is `tip`.
        let tip_a = contact - normal * contact_depth;

        let mut parts = vec![Part {
            kind: PartKind::TipCone,
            a: tip_a,
            b: tip,
            radius_a: tip_radius,
            radius_b: radius,
            shape: Shape::Round,
        }];

        // The link only exists when the pillar sits laterally from the tip end.
        // A straight drop has `tip == joint`, so there is no link; emitting a
        // zero-length cylinder would feed a degenerate normal into the mesher.
        // When the link is present there are two join vertices — the tip end
        // (tip cone meets link) and the joint (link meets pillar) — and each
        // gets its own ball, as on model landings.
        if (tip - joint).magnitude_squared() > f32::EPSILON {
            parts.push(Part {
                kind: PartKind::Link,
                a: tip,
                b: joint,
                radius_a: radius,
                radius_b: radius,
                shape: Shape::Round,
            });

            // Ball at the tip end: where the tip cone meets the 45° link.
            parts.push(Part {
                kind: PartKind::Joint,
                a: tip,
                b: tip,
                radius_a: radius * 1.04,
                radius_b: radius * 1.04,
                shape: Shape::Round,
            });
        }

        // The joint sphere sits on the vertex where the tip/link meets the
        // pillar. A single sphere, centred exactly on the join with no axial
        // offset, radius 0.52 · (2 · radius) = 1.04 · radius.
        // For a straight drop (`tip == joint`) this is also the sole ball.
        parts.push(Part {
            kind: PartKind::Joint,
            a: joint,
            b: joint,
            radius_a: radius * 1.04,
            radius_b: radius * 1.04,
            shape: Shape::Round,
        });

        parts.push(Part {
            kind: PartKind::Pillar,
            a: joint,
            b: joint.xy().push(0.0),
            radius_a: radius,
            radius_b: radius,
            shape: Shape::Round,
        });

        self.supports.push(Support {
            id,
            point: SupportPoint {
                pos: contact,
                normal,
                kind: PointKind::Contact,
                depth: contact_depth,
            },
            parts,
            foot,
            group: 0,
        });
    }

    /// Adds a support that lands on the model: contact sphere, tip
    /// cone, a joint at the tip end, a vertical pillar down to the foot, a
    /// joint at the foot, and one tail per landing tail. A tail is the tip cone
    /// mirrored: pillar-wide at the foot, tip-narrow where it sinks in.
    ///
    /// A slim landing swaps the pillar for a slim, sizes both
    /// joints from the slim radius, and has no tail: the slim's end ball
    /// already reaches the sunk point.
    ///
    /// `tip_radius` and `radius` are millimetres.
    pub fn add_landing(&mut self, id: SupportId, tip_radius: f32, radius: f32, landing: &Landing) {
        let round = |kind, a, b, radius_a, radius_b| Part {
            kind,
            a,
            b,
            radius_a,
            radius_b,
            shape: Shape::Round,
        };
        let Landing {
            contact,
            tip,
            foot,
            depth,
            ..
        } = *landing;

        // Both landing joints share one radius: the plate pillars'
        // 1.04 × radius.
        let (body, body_radius, foot_ball) = match landing.kind {
            LandingKind::Pillar => (PartKind::Pillar, radius, radius * 1.04),
            // The slim's end ball is exactly the slim radius.
            LandingKind::Slim { radius } => (PartKind::Slim, radius, radius),
        };
        let joint = body_radius * 1.04;
        let mut parts = vec![
            round(
                PartKind::ContactSphere,
                contact,
                contact,
                tip_radius,
                tip_radius,
            ),
            round(PartKind::TipCone, contact, tip, tip_radius, radius),
            round(PartKind::Joint, tip, tip, joint, joint),
            round(body, tip, foot, body_radius, body_radius),
            round(PartKind::Joint, foot, foot, foot_ball, foot_ball),
        ];
        if landing.kind == LandingKind::Pillar {
            for &(end, _) in &landing.tails {
                parts.push(round(PartKind::Tail, foot, end, radius, tip_radius));
            }
        }

        let normal = (tip - contact).try_normalize(f32::EPSILON);
        let (pos, foot_normal) = landing
            .tails
            .first()
            .copied()
            .unwrap_or((foot, Vector3::z()));
        self.supports.push(Support {
            id,
            point: SupportPoint {
                pos: contact,
                normal: normal.unwrap_or(-Vector3::z()),
                kind: PointKind::Contact,
                depth,
            },
            parts,
            foot: Foot::Model {
                pos,
                normal: foot_normal,
            },
            group: 0,
        });
    }

    /// Truncates the vertical pillar of support `index` so it ends at `bottom_z`
    /// (a fork point) instead of reaching the plate. Used by the brace network's
    /// inverted-V forks, where the middle pillar of a
    /// collinear triple stops short and is carried by its neighbours.
    pub fn truncate_pillar(&mut self, index: usize, bottom_z: f32) {
        if let Some(support) = self.supports.get_mut(index)
            && let Some(pillar) = support
                .parts
                .iter_mut()
                .find(|p| p.kind == PartKind::Pillar)
        {
            pillar.b.z = bottom_z;
        }
    }
}

impl SupportSet {
    /// Gives every plate pillar its foot: a raft under all of
    /// them when the raft shape is set, else a base each.
    pub fn add_feet(&mut self, base: &BaseConfig, raft: &RaftConfig) {
        let feet: Vec<_> = (self.supports.iter())
            .filter_map(|s| plate_pillar(s).map(|i| s.parts[i].a.xy()))
            .collect();
        match Raft::build(raft, &feet) {
            Some(raft) => {
                self.stand_on_raft(&raft);
                self.raft = Some(raft);
            }
            None => {
                self.raft = None;
                self.add_bases(base);
            }
        }
    }

    /// Stands every plate pillar whose foot lies inside the raft on its top,
    /// [`JOINT_SHRINK`] deep and with no base or boss.
    /// Feet outside the raft stay on the plate.
    pub fn stand_on_raft(&mut self, raft: &Raft) {
        let floor = raft.top() - JOINT_SHRINK;
        for support in &mut self.supports {
            let Some(pillar) = plate_pillar(support) else {
                continue;
            };
            let a = support.parts[pillar].a;
            if !raft.contains(a.xy()) {
                continue;
            }
            if a.z - floor > JOINT_SHRINK {
                support.parts[pillar].b.z = floor;
            } else {
                support.parts.remove(pillar);
            }
        }
    }

    /// Stands every plate pillar on the plate.
    ///
    /// Only pillars that reach z = 0 are touched, so this skips fork-truncated
    /// pillars and model landings, and running it twice changes
    /// nothing. Short pillars (length + 0.6 × tip length < 5 mm) get a base
    /// only; long ones get a base, a boss, then the pillar. Each part starts
    /// [`JOINT_SHRINK`] below the top of the one beneath it.
    pub fn add_bases(&mut self, cfg: &BaseConfig) {
        if cfg.shape == BaseShape::None {
            return;
        }
        for support in &mut self.supports {
            let Some(pillar) = plate_pillar(support) else {
                continue;
            };
            let tip = (support.parts.iter())
                .find(|p| p.kind == PartKind::TipCone)
                .map_or(0.0, |p| (p.b - p.a).magnitude());
            let Part { a, radius_b, .. } = support.parts[pillar];
            let short = a.z + 0.6 * tip < SHORT_PILLAR;
            let xy = a.xy();

            let top = match cfg.shape {
                BaseShape::Skate => {
                    let inset = cfg.thickness / SKATE_ANGLE.to_radians().tan();
                    (cfg.radius - inset).max(cfg.boss_radius)
                }
                BaseShape::Cone => cfg.boss_radius,
                BaseShape::Cylinder | BaseShape::None => cfg.radius,
            };
            let mut parts = vec![round(
                PartKind::Base,
                xy.push(0.0),
                xy.push(cfg.thickness),
                cfg.radius,
                top,
            )];
            let mut floor = cfg.thickness - JOINT_SHRINK;
            if !short {
                let boss_top = floor + cfg.boss_height;
                parts.push(round(
                    PartKind::Boss,
                    xy.push(floor),
                    xy.push(boss_top),
                    cfg.boss_radius,
                    radius_b,
                ));
                floor = boss_top - JOINT_SHRINK;
            }
            // A joint already inside the base needs no pillar at all.
            if a.z - floor > JOINT_SHRINK {
                support.parts[pillar].b.z = floor;
            } else {
                support.parts.remove(pillar);
            }
            support.parts.extend(parts);
        }
    }
}

/// The pillar of a support that still reaches the plate, if any.
fn plate_pillar(support: &Support) -> Option<usize> {
    (support.parts.iter()).position(|p| p.kind == PartKind::Pillar && p.b.z == 0.0)
}

fn round(kind: PartKind, a: Vector3<f32>, b: Vector3<f32>, radius_a: f32, radius_b: f32) -> Part {
    Part {
        kind,
        a,
        b,
        radius_a,
        radius_b,
        shape: Shape::Round,
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
                raft.serialize(ser);
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

        let raft = des.read_bool().then(|| Raft::deserialize(des));

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
    use crate::supports::raft::RaftShape;

    #[test]
    fn support_set_roundtrips() {
        let mut set = SupportSet::default();
        set.add_routed(
            SupportId::from_raw(7),
            0.2,
            0.6,
            0.4,
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
            0.4,
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
        set.raft = Raft::build(&RAFT, &[Vector2::new(1.0, 2.0)]);

        let mut buffer = vec![0; 4096];
        {
            let mut ser = SizedSerializer::new(&mut buffer);
            set.serialize(&mut ser);
        }

        let mut des = SliceDeserializer::new(&buffer);
        let out = SupportSet::deserialize(&mut des);

        assert_eq!(out.lift, set.lift);
        assert_eq!(out.supports.len(), 2);
        assert_eq!(out.braces.len(), 1);
        let (raft, original) = (out.raft.unwrap(), set.raft.unwrap());
        assert_eq!(raft.config, original.config);
        assert_eq!(raft.islands, original.islands);

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

    #[test]
    fn landing_support_ends_in_a_tail() {
        let mut set = SupportSet::default();
        let landing = Landing {
            kind: LandingKind::Pillar,
            contact: Vector3::new(1.0, 1.0, 20.0),
            tip: Vector3::new(1.0, 1.0, 18.0),
            foot: Vector3::new(1.0, 1.0, 12.0),
            tails: vec![(Vector3::new(1.0, 1.0, 9.8), Vector3::z())],
            depth: 0.4,
        };
        set.add_landing(SupportId::from_raw(3), 0.2, 0.6, &landing);

        let support = &set.supports[0];
        let kinds: Vec<_> = support.parts.iter().map(|p| p.kind).collect();
        assert_eq!(
            kinds,
            [
                PartKind::ContactSphere,
                PartKind::TipCone,
                PartKind::Joint,
                PartKind::Pillar,
                PartKind::Joint,
                PartKind::Tail,
            ]
        );

        let tail = support.parts.last().unwrap();
        assert_eq!(tail.a, landing.foot);
        assert_eq!(tail.b, landing.tails[0].0);
        assert_eq!((tail.radius_a, tail.radius_b), (0.6, 0.2));
        assert!(matches!(support.foot, Foot::Model { .. }));
        assert_eq!(support.point.depth, 0.4);
    }

    #[test]
    fn routed_support_has_joint() {
        let mut set = SupportSet::default();
        let radius = 0.6;
        set.add_routed(
            SupportId::from_raw(1),
            0.2,
            radius,
            0.4,
            [
                Vector3::new(1.0, 2.0, 3.0),
                Vector3::new(1.3, 2.4, 5.0),
                Vector3::new(3.0, 4.0, 1.0),
            ],
        );

        let support = &set.supports[0];
        let joints: Vec<_> = support
            .parts
            .iter()
            .filter(|p| p.kind == PartKind::Joint)
            .collect();
        assert_eq!(
            joints.len(),
            2,
            "routed support with a link should carry two balls"
        );

        // The link's lower end (where it meets the pillar) carries one ball,
        // and the tip end (where the tip cone meets the link) the other.
        let [pillar_joint, tip_joint] = [joints[1], joints[0]];
        assert_eq!(pillar_joint.a, Vector3::new(3.0, 4.0, 1.0));
        assert_eq!(tip_joint.a, Vector3::new(1.3, 2.4, 5.0));

        for joint in joints {
            assert_eq!(joint.b, joint.a);
            // 1.04 × radius, the 0.52·diameter joint rule.
            assert!((joint.radius_a - 1.04 * radius).abs() < 1e-6);
            assert_eq!(joint.radius_b, joint.radius_a);
            assert_eq!(joint.shape, Shape::Round);
        }
    }

    #[test]
    fn straight_drop_has_single_joint() {
        let mut set = SupportSet::default();
        let contact = Vector3::new(1.0, 2.0, 5.0);
        set.add_routed(
            SupportId::from_raw(2),
            0.2,
            0.6,
            0.4,
            [contact, contact, contact],
        );

        // A straight drop (tip == joint) keeps exactly one ball: TipCone,
        // Joint, Pillar.
        let support = &set.supports[0];
        let joints: Vec<_> = support
            .parts
            .iter()
            .filter(|p| p.kind == PartKind::Joint)
            .collect();
        assert_eq!(joints.len(), 1);
    }

    const BASE: BaseConfig = BaseConfig {
        shape: BaseShape::Skate,
        radius: 6.0,
        thickness: 1.0,
        boss_height: 1.0,
        boss_radius: 1.1,
    };

    const RAFT: RaftConfig = RaftConfig {
        shape: RaftShape::Skate,
        foot_radius: 6.0,
        area_ratio: 1.1,
        thickness: 1.0,
        height: 1.8,
        angle: 30.0,
        grid_cell: 2.0,
        grid_width: 2.0,
    };

    #[test]
    fn raft_replaces_bases() {
        let mut set = plate_support(10.0);
        set.add_feet(&BASE, &RAFT);
        assert!(set.raft.is_some());
        assert!(part(&set, PartKind::Base).is_none());
        assert!(part(&set, PartKind::Boss).is_none());
        assert!((part(&set, PartKind::Pillar).unwrap().b.z - 0.998).abs() < 1e-6);

        let mut set = plate_support(10.0);
        let none = RaftConfig {
            shape: RaftShape::None,
            ..RAFT
        };
        set.add_feet(&BASE, &none);
        assert!(set.raft.is_none());
        assert!(part(&set, PartKind::Base).is_some());
    }

    fn plate_support(joint_z: f32) -> SupportSet {
        let mut set = SupportSet::default();
        let joint = Vector3::new(1.0, 2.0, joint_z);
        let contact = joint + Vector3::new(0.0, 0.0, 2.0);
        set.add_routed(
            SupportId::from_raw(1),
            0.2,
            0.6,
            0.4,
            [contact, joint, joint],
        );
        set
    }

    fn part(set: &SupportSet, kind: PartKind) -> Option<Part> {
        set.supports[0]
            .parts
            .iter()
            .find(|p| p.kind == kind)
            .copied()
    }

    #[test]
    fn long_pillar_stands_on_boss_and_base() {
        let mut set = plate_support(10.0);
        set.add_bases(&BASE);

        let base = part(&set, PartKind::Base).unwrap();
        let boss = part(&set, PartKind::Boss).unwrap();
        let pillar = part(&set, PartKind::Pillar).unwrap();
        assert_eq!((base.a.z, base.b.z), (0.0, 1.0));
        assert_eq!(base.radius_a, 6.0);
        assert!(base.radius_b < base.radius_a && base.radius_b >= BASE.boss_radius);
        assert!((boss.a.z - 0.998).abs() < 1e-6 && (boss.b.z - 1.998).abs() < 1e-6);
        assert_eq!((boss.radius_a, boss.radius_b), (1.1, 0.6));
        assert!((pillar.b.z - 1.996).abs() < 1e-6);
        assert_eq!(pillar.b.xy(), base.a.xy());
    }

    #[test]
    fn short_pillar_gets_no_boss() {
        // 3 + 0.6 × 2 = 4.2 < 5 mm.
        let mut set = plate_support(3.0);
        set.add_bases(&BASE);
        assert!(part(&set, PartKind::Boss).is_none());
        let pillar = part(&set, PartKind::Pillar).unwrap();
        assert!((pillar.b.z - 0.998).abs() < 1e-6);
    }

    #[test]
    fn no_base_leaves_the_pillar_on_the_plate() {
        let mut set = plate_support(10.0);
        set.add_bases(&BaseConfig {
            shape: BaseShape::None,
            ..BASE
        });
        assert!(part(&set, PartKind::Base).is_none());
        assert_eq!(part(&set, PartKind::Pillar).unwrap().b.z, 0.0);
    }

    #[test]
    fn bases_are_added_once_and_skip_truncated_pillars() {
        let mut set = plate_support(10.0);
        set.add_routed(
            SupportId::from_raw(2),
            0.2,
            0.6,
            0.4,
            [
                Vector3::new(5.0, 0.0, 14.0),
                Vector3::new(5.0, 0.0, 12.0),
                Vector3::new(5.0, 0.0, 12.0),
            ],
        );
        set.truncate_pillar(1, 6.0);
        set.add_bases(&BASE);
        set.add_bases(&BASE);

        let count = |s: &Support| s.parts.iter().filter(|p| p.kind == PartKind::Base).count();
        assert_eq!(count(&set.supports[0]), 1);
        assert_eq!(count(&set.supports[1]), 0);
    }
}
