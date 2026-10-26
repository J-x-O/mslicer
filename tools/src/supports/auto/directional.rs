//! Directional support: a single straight part between two points the
//! user picks, bypassing routing.
//!
//! Where auto-support routes a contact down to the plate, a directional support
//! is one segment from a first click to a second click. It is how users bridge
//! from the model to an existing support, or set a tip at an angle the auto
//! path would not choose. Nothing routes it; the two clicks fix the geometry
//! outright.

use nalgebra::Vector3;
use slicer::geometry::{mesh::Mesh, partitioning::Bvh};

use common::units::Milimeter;

use crate::supports::{
    Part, PartKind, Shape,
    auto::{collision, tip},
    mesh::SupportPreset,
};

/// The mode of a directional support. The mode decides which single part
/// (or slim + joint) is built between the two clicked points.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DirectionMode {
    /// Start on the model: one tip cone from the contact to the end
    /// point, tip-narrow at the contact and pillar-wide at the end.
    Tip,
    /// A slim segment plus a joint sphere at the end.
    Slim,
    /// Started on an existing support: one constant-radius pillar segment.
    Pillar,
}

/// The parts a directional support expands into, plus whether it collides with
/// the model (the collision tests only pick a warning colour).
#[derive(Clone, Debug)]
pub struct Directional {
    pub parts: Vec<Part>,
    /// `true` when any collision test reports a part–model collision. The UI shows a
    /// warning colour rather than refusing the support.
    pub collides: bool,
}

/// Builds the parts for a directional support from `start` to `end` in `mode`,
/// using the preset's sizes, and runs the collision tests so the caller can
/// draw a warning colour instead of a refusal.
pub fn build_directional(
    mesh: &Mesh,
    bvh: &Bvh,
    preset: &SupportPreset,
    mode: DirectionMode,
    start: Vector3<f32>,
    end: Vector3<f32>,
) -> Directional {
    let (tip_radius, radius) = (
        preset.tip_radius.get::<Milimeter>(),
        preset.radius.get::<Milimeter>(),
    );
    let slim_radius = preset.slim_radius.get::<Milimeter>();

    let parts = match mode {
        DirectionMode::Tip => {
            // A tip cone: narrow at the contact, pillar-wide at the end. The two
            // clicks fix the direction outright (nothing routes it).
            vec![Part {
                kind: PartKind::TipCone,
                a: start,
                b: end,
                radius_a: tip_radius,
                radius_b: radius,
                shape: Shape::Round,
            }]
        }
        DirectionMode::Slim => {
            // A slim rod plus a joint sphere at the end.
            vec![
                Part {
                    kind: PartKind::Slim,
                    a: start,
                    b: end,
                    radius_a: slim_radius,
                    radius_b: slim_radius,
                    shape: Shape::Round,
                },
                Part {
                    kind: PartKind::Joint,
                    a: end,
                    b: end,
                    radius_a: slim_radius * 1.04,
                    radius_b: slim_radius * 1.04,
                    shape: Shape::Round,
                },
            ]
        }
        DirectionMode::Pillar => {
            // One pillar segment at the pillar radius.
            vec![Part {
                kind: PartKind::Pillar,
                a: start,
                b: end,
                radius_a: radius,
                radius_b: radius,
                shape: Shape::Round,
            }]
        }
    };

    // Collision tests, run only for the warning colour. A slim or pillar uses the
    // link-style full-radius shell; a tip uses the horn shell with its sink.
    let collides = parts.iter().any(|p| match p.kind {
        PartKind::TipCone => collision::horn_test(
            mesh,
            bvh,
            p.a,
            p.b,
            p.radius_b,
            preset.contact_depth.get::<Milimeter>(),
        ),
        PartKind::Slim | PartKind::Pillar | PartKind::Joint => {
            collision::link_test(mesh, bvh, p.a, p.b, p.radius_a.max(p.radius_b))
        }
        _ => false,
    });

    Directional { parts, collides }
}

/// Builds the parts for an upward tip from an existing joint up to the model.
/// The tip runs from the target contact point down to the joint, wide at the
/// joint and tip-narrow at the contact; when `contact_shape` is `true` a
/// contact sphere also sinks into the model at the target.
///
/// Returns `None` when the target is refused: it must not be flatter than the
/// tip tilt clamp (70° from straight up) or fail the horn test.
pub fn build_upward_tip(
    mesh: &Mesh,
    bvh: &Bvh,
    preset: &SupportPreset,
    joint: Vector3<f32>,
    target: Vector3<f32>,
    contact_shape: bool,
) -> Option<Vec<Part>> {
    // The direction the tip must leave the joint: toward the target, pointing
    // up. Refuse a target more than 70° off vertical.
    let d = (target - joint).try_normalize(f32::EPSILON)?;
    if d.z < tip::MAX_CONNECT_ANGLE_COS {
        return None;
    }

    let (tip_radius, radius) = (
        preset.tip_radius.get::<Milimeter>(),
        preset.radius.get::<Milimeter>(),
    );

    // Horn test on the tip, pulled back by the contact sink so the sunk
    // contact region isn't falsely reported.
    if collision::horn_test(
        mesh,
        bvh,
        target,
        joint,
        radius,
        preset.contact_depth.get::<Milimeter>(),
    ) {
        return None;
    }

    let mut parts = Vec::new();
    if contact_shape {
        parts.push(Part {
            kind: PartKind::ContactSphere,
            a: target,
            b: target,
            radius_a: tip_radius,
            radius_b: tip_radius,
            shape: Shape::Round,
        });
    }
    parts.push(Part {
        kind: PartKind::TipCone,
        a: target,
        b: joint,
        radius_a: tip_radius,
        radius_b: radius,
        shape: Shape::Round,
    });
    Some(parts)
}

#[cfg(test)]
mod tests {
    use common::progress::Progress;
    use nalgebra::Vector3;
    use slicer::geometry::{mesh::Mesh, partitioning::Bvh};

    use super::*;

    /// A flat 40 × 40 mm slab lifted 10 mm off the plate.
    fn slab() -> (Mesh, Bvh) {
        let mut verts = Vec::new();
        let mut faces = Vec::new();
        let v = Vector3::new;
        let base = 0u32;
        for i in 0..8 {
            let pick = |bit, lo: f32, hi: f32| if i & bit == 0 { lo } else { hi };
            verts.push(v(
                pick(1, -20.0, 20.0),
                pick(2, -20.0, 20.0),
                pick(4, 10.0, 12.0),
            ));
        }
        #[rustfmt::skip]
        let quads = [
            [0u32, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4],
            [2, 6, 7, 3], [0, 4, 6, 2], [1, 3, 7, 5],
        ];
        for [a, b, c, d] in quads {
            faces.push([base + a, base + b, base + c]);
            faces.push([base + a, base + c, base + d]);
        }
        let mesh = Mesh::new(verts, faces);
        let bvh = Bvh::build(mesh.inner(), Progress::new());
        (mesh, bvh)
    }

    #[test]
    fn tip_mode_builds_a_tip_cone() {
        let (mesh, bvh) = slab();
        let preset = SupportPreset::default();
        let d = build_directional(
            &mesh,
            &bvh,
            &preset,
            DirectionMode::Tip,
            Vector3::new(0.0, 0.0, 10.0),
            Vector3::new(0.0, 0.0, 4.0),
        );
        assert_eq!(d.parts.len(), 1);
        assert_eq!(d.parts[0].kind, PartKind::TipCone);
    }

    #[test]
    fn slim_mode_adds_a_joint() {
        let (mesh, bvh) = slab();
        let preset = SupportPreset::default();
        let d = build_directional(
            &mesh,
            &bvh,
            &preset,
            DirectionMode::Slim,
            Vector3::new(0.0, 0.0, 8.0),
            Vector3::new(2.0, 0.0, 4.0),
        );
        assert_eq!(d.parts.len(), 2);
        assert_eq!(d.parts[0].kind, PartKind::Slim);
        assert_eq!(d.parts[1].kind, PartKind::Joint);
    }

    #[test]
    fn directional_reports_collision() {
        let (mesh, bvh) = slab();
        let preset = SupportPreset::default();
        // A tip straight through the slab (from above down through it) must
        // collide.
        let d = build_directional(
            &mesh,
            &bvh,
            &preset,
            DirectionMode::Tip,
            Vector3::new(0.0, 0.0, 14.0),
            Vector3::new(0.0, 0.0, 0.0),
        );
        assert!(d.collides);
    }

    #[test]
    fn upward_tip_builds_a_tip_to_the_joint() {
        let (mesh, bvh) = slab();
        let preset = SupportPreset::default();
        // Joint below the slab underside; target on the slab underside. An
        // upward tip runs from the target down to the joint.
        let joint = Vector3::new(0.0, 0.0, 4.0);
        let target = Vector3::new(0.0, 0.0, 10.0);
        let parts = build_upward_tip(&mesh, &bvh, &preset, joint, target, true).unwrap();
        assert_eq!(parts.len(), 2);
        assert_eq!(parts[0].kind, PartKind::ContactSphere);
        assert_eq!(parts[1].kind, PartKind::TipCone);
        assert_eq!(parts[1].a, target);
        assert_eq!(parts[1].b, joint);
    }

    #[test]
    fn upward_tip_refuses_flat_targets() {
        let (mesh, bvh) = slab();
        let preset = SupportPreset::default();
        // A target nearly level with the joint (more than 70° off vertical) is
        // refused by the tilt clamp.
        let joint = Vector3::new(0.0, 0.0, 8.0);
        let target = Vector3::new(20.0, 0.0, 8.0);
        assert!(build_upward_tip(&mesh, &bvh, &preset, joint, target, false).is_none());
    }
}
