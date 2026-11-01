//! Mini supports: a short strut from an overhang straight to the model surface
//! just below it, for gaps too small for a proper support. Two tip cones meet
//! at a thin waist; there is no pillar and no foot on the plate.
//!
//! Used for supports placed by hand (and the island check), not by automatic
//! generation.

use nalgebra::Vector3;
use slicer::geometry::{mesh::Mesh, partitioning::Bvh};

use common::units::Milimeter;

use crate::supports::{
    auto::{Landing, LandingKind, Routed, route_point, tip::clamp_tip_direction},
    mesh::SupportPreset,
};

/// The longest strut a mini support may be, contact to contact.
pub const MINI_REACH: f32 = 6.0;
/// The shortest strut: below this the two surfaces print fused anyway, and the
/// contact balls just overlap.
pub const MINI_MIN_GAP: f32 = 0.1;
/// The lower surface must face back at the strut at least this much (cosine).
const MIN_FACING: f32 = 0.2;

/// Routes a support placed by hand. A gap below shorter than a tip gets a mini
/// support; otherwise the support is routed as automatic ones are (plate
/// first, then onto the model), with a mini as the last resort. `mini` asks
/// for a mini support only.
pub fn route_hand(
    mesh: &Mesh,
    bvh: &Bvh,
    support: &SupportPreset,
    contact: Vector3<f32>,
    normal: Vector3<f32>,
    mini: bool,
) -> Option<Routed> {
    let as_mini = || route_mini(mesh, bvh, support, contact, normal).map(Routed::Model);
    if mini {
        return as_mini();
    }
    let tip_length = support.tip_length.get::<Milimeter>();
    let short = directions(normal)
        .into_iter()
        .filter_map(|d| first_hit(mesh, bvh, contact, d))
        .any(|(distance, ..)| distance < tip_length);
    if short && let Some(routed) = as_mini() {
        return Some(routed);
    }
    route_point(mesh, bvh, support, contact, normal).or_else(as_mini)
}

/// A mini support from `contact` (outward normal `normal`) to the model below:
/// straight down first, else along the tip direction, else along the normal
/// itself. The waist is the slim radius, or only the tip's where that doesn't
/// fit. `None` if no direction meets the model within [`MINI_MIN_GAP`] ..
/// [`MINI_REACH`] with room for the strut.
pub fn route_mini(
    mesh: &Mesh,
    bvh: &Bvh,
    support: &SupportPreset,
    contact: Vector3<f32>,
    normal: Vector3<f32>,
) -> Option<Landing> {
    let tip_radius = support.tip_radius.get::<Milimeter>();
    let slim = support.slim_radius.get::<Milimeter>().max(tip_radius);
    let depth = support.contact_depth.get::<Milimeter>();

    directions(normal).into_iter().find_map(|d| {
        let (distance, point, lower) = first_hit(mesh, bvh, contact, d)?;
        if !(MINI_MIN_GAP..=MINI_REACH).contains(&distance) || lower.dot(&-d) < MIN_FACING {
            return None;
        }
        let waist = [slim, tip_radius]
            .into_iter()
            .find(|&w| clear(mesh, bvh, contact, d, distance, w))?;
        // The lower end sinks into the surface it rests on.
        let end = point + d * depth;
        Some(Landing {
            kind: LandingKind::Mini { radius: waist },
            contact,
            tip: contact + d * (distance / 2.0),
            foot: end,
            tails: vec![(end, lower)],
            depth,
        })
    })
}

/// Straight down, then along the tip direction, then along the normal itself
/// if it points down; each only if it differs from those before.
fn directions(normal: Vector3<f32>) -> Vec<Vector3<f32>> {
    let mut out = vec![-Vector3::z()];
    if let Some(normal) = normal.try_normalize(f32::EPSILON) {
        for d in [clamp_tip_direction(normal), normal] {
            if d.z < -0.1 && out.iter().all(|o| o.dot(&d) < 0.999) {
                out.push(d);
            }
        }
    }
    out
}

/// Where a ray from just off the contact first meets the model: the distance,
/// the point and the surface normal there (world space).
fn first_hit(
    mesh: &Mesh,
    bvh: &Bvh,
    contact: Vector3<f32>,
    d: Vector3<f32>,
) -> Option<(f32, Vector3<f32>, Vector3<f32>)> {
    const OFF: f32 = 0.02;
    let hit = bvh.intersect_ray(mesh, contact + d * OFF, d)?;
    let point = mesh.transform(&hit.position);
    let normal = mesh
        .transform_normal(&mesh.normal(hit.face))
        .try_normalize(f32::EPSILON)?;
    Some(((point - contact).norm(), point, normal))
}

/// Whether the strut's middle stays out of the model. Its axis is open (the
/// ray along it met nothing before the lower surface), so short spokes from
/// the axis out to the waist radius, in eight directions at three heights,
/// must not cross a surface. The ends are left out, where the strut meets
/// curved surfaces anyway.
fn clear(
    mesh: &Mesh,
    bvh: &Bvh,
    contact: Vector3<f32>,
    d: Vector3<f32>,
    length: f32,
    waist: f32,
) -> bool {
    let side = if d.z.abs() < 0.9 {
        Vector3::z()
    } else {
        Vector3::x()
    };
    let u = d.cross(&side).normalize();
    let v = d.cross(&u).normalize();
    let spokes = (0..8).map(|i| {
        let angle = i as f32 * std::f32::consts::FRAC_PI_4;
        (u * angle.cos() + v * angle.sin()) * waist
    });
    let spokes: Vec<_> = spokes.collect();
    [0.25, 0.5, 0.75].iter().all(|&t| {
        let axis = contact + d * (t * length);
        (spokes.iter()).all(|off| bvh.intersect_segment(mesh, axis, axis + off).is_none())
    })
}

#[cfg(test)]
mod tests {
    use common::progress::Progress;
    use nalgebra::Vector3;
    use slicer::geometry::{
        mesh::{Mesh, MeshBuilder},
        partitioning::Bvh,
    };

    use super::{MINI_REACH, route_hand, route_mini};
    use crate::supports::{
        auto::{LandingKind, Routed},
        mesh::SupportPreset,
    };

    /// A box with corners `a` and `b`, faces out.
    fn cuboid(builder: &mut MeshBuilder, a: Vector3<f32>, b: Vector3<f32>) {
        let mut v = |x: f32, y: f32, z: f32| builder.add_vertex(Vector3::new(x, y, z));
        let c = [
            v(a.x, a.y, a.z),
            v(b.x, a.y, a.z),
            v(b.x, b.y, a.z),
            v(a.x, b.y, a.z),
            v(a.x, a.y, b.z),
            v(b.x, a.y, b.z),
            v(b.x, b.y, b.z),
            v(a.x, b.y, b.z),
        ];
        for [i, j, k] in [
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [1, 2, 6],
            [1, 6, 5],
            [2, 3, 7],
            [2, 7, 6],
            [3, 0, 4],
            [3, 4, 7],
        ] {
            builder.add_face([c[i], c[j], c[k]]);
        }
    }

    /// A table top at z 10..11 over a block reaching up to `below`.
    fn shelf(below: f32) -> Mesh {
        let mut builder = MeshBuilder::new();
        cuboid(
            &mut builder,
            Vector3::new(-10.0, -10.0, 10.0),
            Vector3::new(10.0, 10.0, 11.0),
        );
        cuboid(
            &mut builder,
            Vector3::new(-10.0, -10.0, 0.0),
            Vector3::new(10.0, 10.0, below),
        );
        builder.build()
    }

    fn bvh(mesh: &Mesh) -> Bvh {
        Bvh::build(mesh.inner(), Progress::new())
    }

    #[test]
    fn short_gap_gets_a_mini() {
        let mesh = shelf(8.5);
        let bvh = bvh(&mesh);
        let preset = SupportPreset::default();
        let contact = Vector3::new(0.0, 0.0, 10.0);
        let routed = route_hand(&mesh, &bvh, &preset, contact, -Vector3::z(), false);
        let Some(Routed::Model(landing)) = routed else {
            panic!("expected a mini, got {routed:?}");
        };
        assert!(matches!(landing.kind, LandingKind::Mini { .. }));
        // The lower end is sunk into the block's top at z 8.5.
        assert!(
            landing.foot.z < 8.5 && landing.foot.z > 8.0,
            "{:?}",
            landing.foot
        );
        assert!((landing.tip.z - 9.25).abs() < 1e-3);
    }

    #[test]
    fn mini_meshes() {
        use crate::supports::{
            SupportId, SupportSet,
            mesh::{Transform, build_supports},
        };
        let mesh = shelf(8.5);
        let bvh = bvh(&mesh);
        let preset = SupportPreset::default();
        let landing = route_mini(
            &mesh,
            &bvh,
            &preset,
            Vector3::new(0.0, 0.0, 10.0),
            -Vector3::z(),
        )
        .unwrap();
        let mut set = SupportSet::default();
        set.add_landing(SupportId::new(), 0.2, 0.6, &landing);
        let (built, _) = build_supports(&set, Transform::default(), 16).unwrap();
        let (lo, hi) = built
            .vertices()
            .iter()
            .fold((f32::MAX, f32::MIN), |(lo, hi), v| {
                (lo.min(v.z), hi.max(v.z))
            });
        // Between the two surfaces, give or take the sunk ends and balls.
        assert!(lo > 7.5 && lo < 8.5 && hi > 9.9 && hi < 10.5, "{lo} {hi}");
    }

    #[test]
    fn tight_gap_gets_a_thin_mini() {
        // A 0.25 mm gap with walls 0.22 mm to either side of the contact: no
        // room for the slim waist (0.25), but for one as thick as the tip.
        let mut builder = MeshBuilder::new();
        cuboid(
            &mut builder,
            Vector3::new(-10.0, -10.0, 10.0),
            Vector3::new(10.0, 10.0, 11.0),
        );
        cuboid(
            &mut builder,
            Vector3::new(-10.0, -10.0, 0.0),
            Vector3::new(10.0, 10.0, 9.75),
        );
        cuboid(
            &mut builder,
            Vector3::new(-10.0, -10.0, 9.75),
            Vector3::new(-0.22, 10.0, 10.0),
        );
        cuboid(
            &mut builder,
            Vector3::new(0.22, -10.0, 9.75),
            Vector3::new(10.0, 10.0, 10.0),
        );
        let mesh = builder.build();
        let bvh = bvh(&mesh);
        let preset = SupportPreset::default();
        let landing = route_mini(
            &mesh,
            &bvh,
            &preset,
            Vector3::new(0.0, 0.0, 10.0),
            -Vector3::z(),
        );
        let landing = landing.expect("a mini fits");
        let tip = preset.tip_radius.get::<common::units::Milimeter>();
        assert_eq!(landing.kind, LandingKind::Mini { radius: tip });
    }

    #[test]
    fn too_far_is_no_mini() {
        let mesh = shelf(10.0 - MINI_REACH - 1.0);
        let bvh = bvh(&mesh);
        let preset = SupportPreset::default();
        let mini = route_mini(
            &mesh,
            &bvh,
            &preset,
            Vector3::new(0.0, 0.0, 10.0),
            -Vector3::z(),
        );
        assert!(mini.is_none());
    }

    #[test]
    fn open_space_routes_as_usual() {
        // Nothing below: a plate support, not a mini.
        let mut builder = MeshBuilder::new();
        cuboid(
            &mut builder,
            Vector3::new(-10.0, -10.0, 10.0),
            Vector3::new(10.0, 10.0, 11.0),
        );
        let mesh = builder.build();
        let bvh = bvh(&mesh);
        let preset = SupportPreset::default();
        let routed = route_hand(
            &mesh,
            &bvh,
            &preset,
            Vector3::new(0.0, 0.0, 10.0),
            -Vector3::z(),
            false,
        );
        assert!(matches!(routed, Some(Routed::Plate(_))));
        let forced = route_hand(
            &mesh,
            &bvh,
            &preset,
            Vector3::new(0.0, 0.0, 10.0),
            -Vector3::z(),
            true,
        );
        assert!(forced.is_none());
    }
}
