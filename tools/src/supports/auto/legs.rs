//! Stabilising legs.
//!
//! After the brace network is built, a tall plate pillar that is still sparsely
//! braced gets at most one diagonal leg. The leg starts on the pillar surface
//! 2 mm below its top, leaves at 45° down for a short cone, and is then routed
//! to the plate like any other support (straight drop, then the lateral
//! search). Finally it is braced back to the pillar it stabilises.
//!
//! The leg is a plain routed `[contact, tip, joint]` triplet whose contact sits
//! on the pillar instead of the model, so it is meshed exactly like a plate
//! support.
//!
//! Only plate pillars are considered. A forked pillar stops about 3 mm under
//! its top and can never reach the height trigger, and legs for model-landing
//! pillars (only directions with model below) aren't built.

use std::f32::consts::{FRAC_1_SQRT_2, TAU};

use nalgebra::{Vector2, Vector3};
use slicer::geometry::{mesh::Mesh, partitioning::Bvh};

use crate::supports::{
    Part, PartKind,
    auto::{RouteSizes, route_support, topology},
};

/// A pillar this tall gets a leg (H ≥ 30 mm)...
const TALL_PILLAR: f32 = 30.0;

/// ...or this tall when it hangs under a long link (H ≥ 20 mm).
const TALL_LINKED_PILLAR: f32 = 20.0;

/// A tip-to-pillar link with more horizontal run than this is "long" (5 mm).
const LONG_LINK_RUN: f32 = 5.0;

/// Without a long link, a leg is only added when some gap between brace or
/// link joints on the pillar exceeds this (30 mm).
const MAX_JOINT_GAP: f32 = 30.0;

/// The leg starts this far below the pillar top (2 mm).
const LEG_START_DROP: f32 = 2.0;

/// A plate leg's cone is this many R long, R = 0.5 × point spacing.
pub const LEG_CONE_R_FACTOR: f32 = 0.6;

/// Gaps between brace directions wider than this also get the two quarter
/// directions (about 148°).
const WIDE_GAP: f32 = 148.0;

/// At most this many free gaps are tried around a pillar with two or more
/// braces.
const MAX_GAPS: usize = 5;

/// Two brace directions closer than this (as a dot product) count as one.
const SAME_DIRECTION: f32 = 0.999;

/// Sizes the leg pass works with, all in millimetres.
#[derive(Clone, Copy, Debug)]
pub struct LegConfig {
    /// Pillar (and brace) radius.
    pub radius: f32,
    /// Collision radius for routing: pillar radius plus clearance.
    pub route_radius: f32,
    /// The leg cone length, 0.6 × R.
    pub cone_length: f32,
    /// Contact depth used by the leg tip's horn test.
    pub contact_depth: f32,
    /// Radius of the leg tip at its contact.
    pub tip_radius: f32,
}

/// Legs for the tall, sparsely braced pillars among `plate`, as routed
/// triplets, plus the braces that tie each leg back to its pillar.
///
/// `braces` is the brace network so far (spanning-tree braces and fork links)
/// and `fork_bottoms` the forked pillars, which are skipped.
pub fn build_legs(
    mesh: &Mesh,
    bvh: &Bvh,
    cfg: &LegConfig,
    plate: &[[Vector3<f32>; 3]],
    braces: &[Part],
    fork_bottoms: &[(usize, f32)],
) -> (Vec<[Vector3<f32>; 3]>, Vec<Part>) {
    let mut legs = Vec::new();
    let mut leg_braces = Vec::new();

    for (index, &[_, tip, joint]) in plate.iter().enumerate() {
        if fork_bottoms.iter().any(|&(i, _)| i == index) {
            continue;
        }

        let height = joint.z;
        let link = joint.xy() - tip.xy();
        let long_link = (link.magnitude() > LONG_LINK_RUN).then(|| link.normalize());
        let tall = height >= TALL_PILLAR || (long_link.is_some() && height >= TALL_LINKED_PILLAR);
        if !tall {
            continue;
        }

        let (directions, mut joints) = incident_braces(braces, joint.xy());
        if long_link.is_none() && max_gap(&mut joints, height) <= MAX_JOINT_GAP {
            continue;
        }

        let pillar = topology::Pillar::plate(joint.xy(), joint.z, tip.z, cfg.radius);
        for dir in leg_directions(&directions, long_link) {
            let Some(leg) = route_leg(mesh, bvh, cfg, joint, dir) else {
                continue;
            };

            let [_, leg_tip, leg_joint] = leg;
            let leg_pillar =
                topology::Pillar::plate(leg_joint.xy(), leg_joint.z, leg_tip.z, cfg.radius);
            if let Some(parts) = topology::brace_pair(pillar, leg_pillar, mesh, bvh) {
                leg_braces.extend(parts);
            }
            legs.push(leg);
            break;
        }
    }

    (legs, leg_braces)
}

/// Routes one leg leaving the pillar topped at `joint` in horizontal direction
/// `dir`: its contact sits on the pillar surface 2 mm below the top and its
/// cone heads 45° down.
fn route_leg(
    mesh: &Mesh,
    bvh: &Bvh,
    cfg: &LegConfig,
    joint: Vector3<f32>,
    dir: Vector2<f32>,
) -> Option<[Vector3<f32>; 3]> {
    let contact = (joint.xy() + dir * cfg.radius).push(joint.z - LEG_START_DROP);
    let normal = (dir * FRAC_1_SQRT_2).push(-FRAC_1_SQRT_2);
    let sizes = RouteSizes {
        radius: cfg.route_radius,
        margin: cfg.route_radius - cfg.radius,
        tip_radius: cfg.tip_radius,
        tip_length: cfg.cone_length,
        contact_depth: cfg.contact_depth,
    };
    let (tip, leg_joint) = route_support(mesh, bvh, &sizes, contact, normal)?;
    Some([contact, tip, leg_joint])
}

/// The distinct horizontal directions of the braces meeting the pillar at
/// `xy` (pointing away from it), and the heights where they meet it.
fn incident_braces(braces: &[Part], xy: Vector2<f32>) -> (Vec<Vector2<f32>>, Vec<f32>) {
    const SAME_XY: f32 = 1e-4;
    let mut directions: Vec<Vector2<f32>> = Vec::new();
    let mut joints = Vec::new();

    for brace in braces.iter().filter(|b| b.kind == PartKind::Brace) {
        for (here, there) in [(brace.a, brace.b), (brace.b, brace.a)] {
            if (here.xy() - xy).magnitude() > SAME_XY {
                continue;
            }
            joints.push(here.z);
            if let Some(dir) = (there.xy() - xy).try_normalize(f32::EPSILON)
                && !directions.iter().any(|d| d.dot(&dir) > SAME_DIRECTION)
            {
                directions.push(dir);
            }
        }
    }

    (directions, joints)
}

/// The largest vertical gap between consecutive joints on a plate pillar of
/// the given height, counting its top and its foot on the plate.
fn max_gap(joints: &mut Vec<f32>, height: f32) -> f32 {
    joints.push(0.0);
    joints.push(height);
    joints.sort_by(f32::total_cmp);
    joints.windows(2).map(|w| w[1] - w[0]).fold(0.0, f32::max)
}

/// Horizontal leg directions to try, in order:
/// - no braces: opposite the long link (if any), then the 8 compass directions;
/// - one brace `u`: ⊥u, −⊥u, the diagonals away from `u`, −u, then the
///   diagonals toward `u`;
/// - two or more: the bisector of each free gap between braces, widest first
///   (at most 5 gaps), plus the two quarter directions for gaps over ~148°.
fn leg_directions(braces: &[Vector2<f32>], long_link: Option<Vector2<f32>>) -> Vec<Vector2<f32>> {
    match braces {
        [] => {
            let compass = (0..8).map(|k| {
                let angle = k as f32 / 8.0 * TAU;
                Vector2::new(angle.cos(), angle.sin())
            });
            long_link.map(|l| -l).into_iter().chain(compass).collect()
        }
        [u] => {
            let perp = Vector2::new(-u.y, u.x);
            vec![
                perp,
                -perp,
                (perp - u).normalize(),
                (-perp - u).normalize(),
                -u,
                (perp + u).normalize(),
                (-perp + u).normalize(),
            ]
        }
        _ => {
            let mut angles: Vec<f32> = braces.iter().map(|d| d.y.atan2(d.x)).collect();
            angles.sort_by(f32::total_cmp);

            // Free gaps between consecutive brace directions, wrapping around.
            let mut gaps: Vec<(f32, f32)> = (0..angles.len())
                .map(|i| {
                    let start = angles[i];
                    let end = angles[(i + 1) % angles.len()];
                    let width = (end - start).rem_euclid(TAU);
                    (start, width)
                })
                .collect();
            gaps.sort_by(|a, b| b.1.total_cmp(&a.1));

            let at = |angle: f32| Vector2::new(angle.cos(), angle.sin());
            let mut directions = Vec::new();
            for (start, width) in gaps.into_iter().take(MAX_GAPS) {
                directions.push(at(start + width * 0.5));
                if width > WIDE_GAP.to_radians() {
                    directions.push(at(start + width * 0.25));
                    directions.push(at(start + width * 0.75));
                }
            }
            directions
        }
    }
}

#[cfg(test)]
mod tests {
    use common::progress::Progress;
    use nalgebra::{Vector2, Vector3};
    use slicer::geometry::{mesh::Mesh, partitioning::Bvh};

    use super::*;

    fn empty_scene() -> (Mesh, Bvh) {
        let mesh = Mesh::new(Vec::new(), Vec::new());
        let bvh = Bvh::build(mesh.inner(), Progress::new());
        (mesh, bvh)
    }

    fn config() -> LegConfig {
        LegConfig {
            radius: 0.6,
            route_radius: 1.1,
            cone_length: 1.5,
            contact_depth: 0.4,
            tip_radius: 0.2,
        }
    }

    /// A straight-drop plate support topped at `(x, y, z)`.
    fn straight(x: f32, y: f32, z: f32) -> [Vector3<f32>; 3] {
        let top = Vector3::new(x, y, z);
        [top + Vector3::z() * 2.0, top, top]
    }

    #[test]
    fn tall_isolated_pillar_gets_a_leg() {
        let (mesh, bvh) = empty_scene();
        let (legs, braces) = build_legs(
            &mesh,
            &bvh,
            &config(),
            &[straight(0.0, 0.0, 40.0)],
            &[],
            &[],
        );
        assert_eq!(legs.len(), 1);

        // First compass direction (+x): contact on the pillar surface 2 mm
        // below the top, cone 45° down, then a straight drop. The cone's outer
        // end is `contact_length − contact_depth` past the contact (the cone
        // is `cone_length` long from the sunk anchor), so `tip −
        // contact` is 1.5 − 0.4 = 1.1 mm.
        let [contact, tip, joint] = legs[0];
        assert!((contact - Vector3::new(0.6, 0.0, 38.0)).magnitude() < 1e-4);
        let cone = tip - contact;
        assert!((cone.magnitude() - 1.1).abs() < 1e-4);
        assert!((cone.x - -cone.z).abs() < 1e-4 && cone.x > 0.0);
        assert_eq!(tip, joint);

        // The leg is braced back to its pillar.
        assert!(!braces.is_empty());
        for brace in &braces {
            let run = (brace.a.xy() - brace.b.xy()).magnitude();
            assert!(((brace.a.z - brace.b.z) - run).abs() < 1e-3, "45° brace");
        }
    }

    #[test]
    fn short_pillar_gets_no_leg() {
        let (mesh, bvh) = empty_scene();
        let (legs, _) = build_legs(
            &mesh,
            &bvh,
            &config(),
            &[straight(0.0, 0.0, 25.0)],
            &[],
            &[],
        );
        assert!(legs.is_empty());
    }

    #[test]
    fn long_link_lowers_the_trigger() {
        // 25 mm tall under a 6 mm link: the leg leaves opposite the link.
        let (mesh, bvh) = empty_scene();
        let support = [
            Vector3::new(-6.0, 0.0, 33.0),
            Vector3::new(-6.0, 0.0, 31.0),
            Vector3::new(0.0, 0.0, 25.0),
        ];
        let (legs, _) = build_legs(&mesh, &bvh, &config(), &[support], &[], &[]);
        assert_eq!(legs.len(), 1);
        let [contact, ..] = legs[0];
        assert!((contact.xy() - Vector2::new(-0.6, 0.0)).magnitude() < 1e-4);
    }

    #[test]
    fn well_braced_tall_pillars_get_no_legs() {
        let (mesh, bvh) = empty_scene();
        let pillars = [
            topology::Pillar::plate(Vector2::new(0.0, 0.0), 40.0, 40.0, 0.6),
            topology::Pillar::plate(Vector2::new(10.0, 0.0), 40.0, 40.0, 0.6),
        ];
        let braces = topology::build_braces(&pillars, 30.0, &mesh, &bvh);
        let plate = [straight(0.0, 0.0, 40.0), straight(10.0, 0.0, 40.0)];
        let (legs, _) = build_legs(&mesh, &bvh, &config(), &plate, &braces, &[]);
        assert!(legs.is_empty());
    }

    #[test]
    fn one_brace_legs_leave_sideways_first() {
        let u = Vector2::new(1.0, 0.0);
        let dirs = leg_directions(&[u], None);
        assert!((dirs[0] - Vector2::new(0.0, 1.0)).magnitude() < 1e-6);
        assert!((dirs[1] - Vector2::new(0.0, -1.0)).magnitude() < 1e-6);
        assert!((dirs[4] - -u).magnitude() < 1e-6);
    }

    #[test]
    fn two_braces_bisect_the_widest_gap() {
        // Braces toward +x and +y leave a 270° gap, bisected at 225°, with
        // quarter directions at 157.5° and 292.5°.
        let dirs = leg_directions(&[Vector2::new(1.0, 0.0), Vector2::new(0.0, 1.0)], None);
        let at = |deg: f32| Vector2::new(deg.to_radians().cos(), deg.to_radians().sin());
        assert!((dirs[0] - at(225.0)).magnitude() < 1e-5);
        assert!((dirs[1] - at(157.5)).magnitude() < 1e-5);
        assert!((dirs[2] - at(292.5)).magnitude() < 1e-5);
        // The 90° gap's bisector comes last.
        assert!((dirs[3] - at(45.0)).magnitude() < 1e-5);
    }
}
