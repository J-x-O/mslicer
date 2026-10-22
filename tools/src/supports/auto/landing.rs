//! Model-landing supports (plan §2.4).
//!
//! A point with no route to the plate drops a vertical pillar onto the model
//! below it instead, and the pillar ends in a short tail cone sunk into that
//! lower surface. The tail is the tip cone mirrored: wide at the pillar foot,
//! narrow where it enters the model.
//!
//! The order follows CHITUBOX's `genNearFarModelRelated`: probe the contact
//! depth, try three tip directions, drop a vertical pillar onto the model, then
//! try a single tail to the best-aligned surface spot and fall back to a claw
//! foot. Drops too short for a pillar and tail get a slim pillar instead
//! (§2.4 step 7). Bracing between landing pillars (§2.4 step 8) lives with the
//! rest of the brace network in `topology`.

use std::f32::consts::FRAC_1_SQRT_2;

use nalgebra::{Vector2, Vector3};
use slicer::geometry::{mesh::Mesh, partitioning::Bvh};

use crate::supports::auto::{collision, grid, tip};

/// How far above the contact the model thickness is probed (§2.4 step 1: at
/// most 2 mm).
const THICKNESS_PROBE: f32 = 2.0;

/// The longest slim pillar (`supportmaxsmallpillarlength`, 3 mm). A landing
/// drop must be at least this long (§2.4 step 3).
const MAX_SLIM_LENGTH: f32 = 3.0;

/// Extra clearance in the minimum drop, `2 × cone length + 0.1 mm` (§2.4 step 3).
const MIN_DROP_EXTRA: f32 = 0.1;

/// The landing floor sits this far below the pillar Ø under the drop's first
/// hit (§2.4 step 3: `hit z − pillar Ø − 0.01 mm`).
const FLOOR_EXTRA: f32 = 0.01;

/// A tail candidate needs a clear line to a point this far above its surface
/// sample (§2.4 step 4: 0.05 mm).
const SAMPLE_LIFT: f32 = 0.05;

/// How many of the best-scored tail candidates are tried (§2.4 step 4).
const TAIL_CANDIDATES: usize = 3;

/// The claw foot top sits this many cone lengths above the floor after a
/// failed single tail (§2.4 step 6: 1.5, or 0.8 after a failed single tail;
/// the default contact-point option always tries the single tail first).
const CLAW_FOOT_CONE_FACTOR: f32 = 0.8;

/// The claw foot top stays at least this far below the tip end (§2.4 step 6).
const CLAW_FOOT_BELOW_TIP: f32 = 0.1;

/// The claw foot must be at least this many cone lengths below the tip end
/// (§2.4 step 6: 0.8 × cone length).
const CLAW_MIN_DROP_CONE_FACTOR: f32 = 0.8;

/// Claw probe reach, sideways and down, as multiples of D² with D the pillar Ø
/// in millimetres (§2.4 step 6: 2.2 × D² and 6.6 × D²).
const CLAW_PROBE_SIDE: f32 = 2.2;
const CLAW_PROBE_DOWN: f32 = 6.6;

/// A slim needs a drop of at least this many slim Øs (§2.4 step 7: 4 × slim Ø).
const SLIM_MIN_DROP_DIAMETERS: f32 = 4.0;

/// The shortest slim body, in slim Øs (§2.4 step 7: slim length 2 × slim Ø).
const SLIM_MIN_LENGTH_DIAMETERS: f32 = 2.0;

/// The slim's cross test samples lines at this many slim Øs off its axis
/// (§2.4 step 7: 0.6 × slim Ø).
const SLIM_CROSS_DIAMETERS: f32 = 0.6;

/// A horn test is skipped for cones shorter than this many pillar Øs (§2.7).
const MIN_HORN_DIAMETERS: f32 = 0.5;

/// Sizes the landing router works with, all in millimetres.
#[derive(Clone, Copy, Debug)]
pub struct LandingConfig {
    /// Collision radius: pillar radius plus the clearance from the model.
    pub radius: f32,
    /// Pillar Ø, without clearance. Sets the landing floor and claw probes.
    pub pillar_diameter: f32,
    /// Tip cone length; the tail is the same cone, mirrored.
    pub cone_length: f32,
    /// Contact depth of the tip (`supporttiptouchdepth`).
    pub contact_depth: f32,
    /// How far a tail sinks into the lower surface (`supporttailtouchdepth`).
    pub tail_depth: f32,
    /// Slim pillar radius. A single tail leans at most 2 × slim Ø sideways
    /// (§2.4 step 4), and short drops get a slim of this radius (step 7).
    pub slim_radius: f32,
    /// How far a slim's lower end sinks into the model
    /// (`supportmiddlesmallpillarlowercontactdepth`).
    pub slim_depth: f32,
}

impl LandingConfig {
    /// Farthest a single tail may lean sideways: 2 × slim Ø (§2.4 step 4).
    fn max_lean(&self) -> f32 {
        4.0 * self.slim_radius
    }
}

/// What carries a landing support from its tip end down to the model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LandingKind {
    /// A vertical pillar ending in one or more tails.
    Pillar,
    /// A slim pillar of this radius for a drop too short for pillar and tail
    /// (§2.4 step 7). Its lower end ball is centred on `foot`.
    Slim { radius: f32 },
}

/// A support that lands on the model: tip cone from `contact` to `tip`,
/// vertical pillar (or slim) from `tip` down to `foot`, then for a pillar one
/// tail per entry of `tails` from `foot` into the lower surface.
#[derive(Clone, Debug)]
pub struct Landing {
    pub kind: LandingKind,
    pub contact: Vector3<f32>,
    pub tip: Vector3<f32>,
    /// The pillar bottom, where the tail(s) start, or the slim's lower end.
    pub foot: Vector3<f32>,
    /// Each tail's sunk end inside the lower surface, with that surface's
    /// outward normal. A slim has one entry, its own sunk end, and no tails.
    pub tails: Vec<(Vector3<f32>, Vector3<f32>)>,
    /// The tip's contact depth after the thickness probe (§2.4 step 1).
    pub depth: f32,
}

/// A point sunk into the lower surface, with that surface's outward normal.
type SunkEnd = (Vector3<f32>, Vector3<f32>);

/// A landing pillar's foot and the tails leaving it.
type LandingFoot = (Vector3<f32>, Vec<SunkEnd>);

/// Routes a support at `contact` (outward normal `normal`) onto the model
/// below it (§2.4). Meant for points [`super::route_support`] couldn't take to
/// the plate. Returns `None` when no tip direction leads to a long enough drop
/// with a valid tail or claw.
pub fn land_support(
    mesh: &Mesh,
    bvh: &Bvh,
    cfg: &LandingConfig,
    contact: Vector3<f32>,
    normal: Vector3<f32>,
) -> Option<Landing> {
    let depth = contact_depth(mesh, bvh, contact, cfg.contact_depth);
    let sink = depth + cfg.radius.max(cfg.cone_length * 0.3);
    let min_drop = MAX_SLIM_LENGTH.max(2.0 * cfg.cone_length + MIN_DROP_EXTRA);

    // Tip directions in order: the surface normal, then halfway to straight
    // down, then straight down (§2.4 step 2).
    let down = -Vector3::z();
    let normal = normal.normalize();
    let halfway = normal + down;
    let mut directions = vec![tip::clamp_tip_direction(normal)];
    if halfway.magnitude() > f32::EPSILON {
        directions.push(tip::clamp_tip_direction(halfway.normalize()));
    }
    directions.push(down);

    for dir in directions {
        let tip = contact + dir * cfg.cone_length;

        // The tip end must be in free space, and the cone must clear the model.
        if collision::segment_hits(mesh, bvh, contact + dir * 0.01, tip)
            || collision::horn_test(mesh, bvh, contact, tip, cfg.radius, sink)
        {
            continue;
        }

        // Drop onto the model; the drop must be long enough for a pillar plus
        // a tail (§2.4 step 3). Shorter gaps are the slim pillar's job.
        let bottom = tip.xy().push(0.0);
        let Some(hit_z) = collision::main_first_hit(mesh, bvh, tip, bottom, cfg.radius) else {
            continue;
        };
        if tip.z - hit_z < min_drop {
            if let Some((foot, end)) = slim(mesh, bvh, cfg, tip) {
                return Some(Landing {
                    kind: LandingKind::Slim {
                        radius: cfg.slim_radius,
                    },
                    contact,
                    tip,
                    foot,
                    tails: vec![end],
                    depth,
                });
            }
            continue;
        }

        let floor = hit_z - cfg.pillar_diameter - FLOOR_EXTRA;
        let landing =
            single_tail(mesh, bvh, cfg, tip, floor).or_else(|| claw(mesh, bvh, cfg, tip, floor));
        if let Some((foot, tails)) = landing {
            return Some(Landing {
                kind: LandingKind::Pillar,
                contact,
                tip,
                foot,
                tails,
                depth,
            });
        }
    }

    None
}

/// The slim pillar (§2.4 step 7), for drops too short for a pillar and tail. It
/// runs straight down from the tip end to the upward-facing surface below.
/// Its lower end sinks the slim contact depth into that surface and is then
/// pulled back by one slim radius, where its end ball sits (§3.1.1 Q3), so the
/// ball's outer surface touches the sunk point.
///
/// Returns the lower end (ball centre) and the sunk point with its surface
/// normal.
fn slim(
    mesh: &Mesh,
    bvh: &Bvh,
    cfg: &LandingConfig,
    tip: Vector3<f32>,
) -> Option<(Vector3<f32>, SunkEnd)> {
    let diameter = 2.0 * cfg.slim_radius;
    let hit = bvh.intersect_segment(mesh, tip, tip.xy().push(0.0))?;
    let surface = mesh.transform(&hit.position);
    let normal = mesh.transform_normal(&mesh.normal(hit.face)).normalize();
    if normal.z < 0.0 || tip.z - surface.z < SLIM_MIN_DROP_DIAMETERS * diameter {
        return None;
    }

    let sunk = surface - Vector3::z() * cfg.slim_depth;
    let end = sunk + Vector3::z() * cfg.slim_radius;
    let length = tip.z - end.z;
    if !(SLIM_MIN_LENGTH_DIAMETERS * diameter..=MAX_SLIM_LENGTH).contains(&length) {
        return None;
    }

    // Cross test down to just above the surface, so the landing face itself
    // isn't counted.
    let above = surface + Vector3::z() * SAMPLE_LIFT;
    if collision::link_test(mesh, bvh, tip, above, SLIM_CROSS_DIAMETERS * diameter) {
        return None;
    }

    Some((end, (sunk, normal)))
}

/// The tip's contact depth: the touch depth, but no deeper than the model is
/// thick above the contact (§2.4 step 1). Thickness is the farthest exit face
/// within [`THICKNESS_PROBE`] straight up; no exit means the model is thicker.
fn contact_depth(mesh: &Mesh, bvh: &Bvh, contact: Vector3<f32>, touch_depth: f32) -> f32 {
    let up = Vector3::z();
    let hits = grid::collapse_coplanar(bvh.intersect_ray_all(mesh, contact + up * 1e-3, up));

    // Going up from inside the model, crossings alternate exit, entry, exit...
    hits.iter()
        .step_by(2)
        .map(|hit| mesh.transform(&hit.position).z - contact.z)
        .filter(|&d| d <= THICKNESS_PROBE)
        .reduce(f32::max)
        .map_or(touch_depth, |thickness| touch_depth.min(thickness))
}

/// The single tail (§2.4 step 4). Samples upward-facing surface spots at or
/// above `floor` within `max_lean` of the tip end's XY, scores each by how
/// well its normal lines up with the tail, and takes the first of the best
/// three whose pillar and tail both clear the model.
///
/// Returns the pillar foot and the tail's sunk end with its surface normal.
fn single_tail(
    mesh: &Mesh,
    bvh: &Bvh,
    cfg: &LandingConfig,
    tip: Vector3<f32>,
    floor: f32,
) -> Option<LandingFoot> {
    let length = cfg.cone_length;
    let mut candidates = Vec::new();

    for offset in lean_offsets(cfg.max_lean()) {
        let xy = tip.xy() + offset;
        let Some(hit) = bvh.intersect_segment(mesh, xy.push(tip.z), xy.push(floor)) else {
            continue;
        };
        let surface = mesh.transform(&hit.position);
        let normal = mesh.transform_normal(&mesh.normal(hit.face)).normalize();
        if normal.z < 0.0 {
            continue;
        }

        // The tail is a full cone length long, so its foot sits above the
        // sample by whatever the lean leaves of that length.
        let lean = offset.magnitude();
        if lean >= length {
            continue;
        }
        let foot = tip
            .xy()
            .push(surface.z + (length * length - lean * lean).sqrt());
        if foot.z >= tip.z
            || collision::segment_hits(mesh, bvh, foot, surface + Vector3::z() * SAMPLE_LIFT)
        {
            continue;
        }

        let axis = (foot - surface).normalize();
        candidates.push((normal.dot(&axis), surface, normal, foot, axis));
    }

    candidates.sort_by(|a, b| b.0.total_cmp(&a.0));
    candidates
        .into_iter()
        .take(TAIL_CANDIDATES)
        .find(|&(_, surface, _, foot, _)| {
            !collision::main_test(mesh, bvh, tip, foot, cfg.radius)
                && tail_clear(mesh, bvh, cfg, surface, foot)
        })
        .map(|(_, surface, normal, foot, axis)| {
            (foot, vec![(surface - axis * cfg.tail_depth, normal)])
        })
}

/// The claw foot (§2.4 step 6), the fallback after a failed single tail. The
/// pillar stops just above the floor and eight compass probes look for
/// surface below; each hit becomes a finger (a tail from the hit up to the
/// foot). With the default contact-point option one finger is kept: the one
/// best aligned with its surface normal.
fn claw(
    mesh: &Mesh,
    bvh: &Bvh,
    cfg: &LandingConfig,
    tip: Vector3<f32>,
    floor: f32,
) -> Option<LandingFoot> {
    let foot_z = (floor + CLAW_FOOT_CONE_FACTOR * cfg.cone_length).min(tip.z - CLAW_FOOT_BELOW_TIP);
    if tip.z - foot_z < CLAW_MIN_DROP_CONE_FACTOR * cfg.cone_length {
        return None;
    }

    let foot = tip.xy().push(foot_z);
    if collision::main_test(mesh, bvh, tip, foot, cfg.radius) {
        return None;
    }

    // Probe reach scales with D², D the pillar Ø in mm. Diagonal probes reuse
    // the unnormalised compass offsets, so they reach √2 farther sideways.
    let d2 = cfg.pillar_diameter * cfg.pillar_diameter;
    let (side, drop) = (CLAW_PROBE_SIDE * d2, CLAW_PROBE_DOWN * d2);

    let mut best: Option<(f32, Vector3<f32>, Vector3<f32>)> = None;
    for (dx, dy) in [
        (1.0, 0.0),
        (1.0, 1.0),
        (0.0, 1.0),
        (-1.0, 1.0),
        (-1.0, 0.0),
        (-1.0, -1.0),
        (0.0, -1.0),
        (1.0, -1.0),
    ] {
        let end = foot + Vector3::new(dx * side, dy * side, -drop);
        let Some(hit) = bvh.intersect_segment(mesh, foot, end) else {
            continue;
        };
        let surface = mesh.transform(&hit.position);
        let normal = mesh.transform_normal(&mesh.normal(hit.face)).normalize();

        let finger = foot - surface;
        if finger.magnitude() <= f32::EPSILON {
            continue;
        }
        let axis = finger.normalize();
        let score = normal.dot(&axis);
        if score <= 0.0 || !tail_clear(mesh, bvh, cfg, surface, foot) {
            continue;
        }

        if best.is_none_or(|(s, ..)| score > s) {
            best = Some((score, surface - axis * cfg.tail_depth, normal));
        }
    }

    best.map(|(_, end, normal)| (foot, vec![(end, normal)]))
}

/// Horn test for a tail or claw finger running from the lower `surface` up to
/// the pillar `foot`. Mirrors the tip's horn test with the tail depth in place
/// of the contact depth, and is skipped for very short cones (§2.7).
fn tail_clear(
    mesh: &Mesh,
    bvh: &Bvh,
    cfg: &LandingConfig,
    surface: Vector3<f32>,
    foot: Vector3<f32>,
) -> bool {
    if (foot - surface).magnitude() < MIN_HORN_DIAMETERS * cfg.pillar_diameter {
        return true;
    }

    let sink = cfg.tail_depth + cfg.radius.max(cfg.cone_length * 0.3);
    !collision::horn_test(mesh, bvh, surface, foot, cfg.radius, sink)
}

/// XY offsets at which single-tail surface samples are taken: the centre,
/// then the 8 compass directions at half and full `max_lean`.
fn lean_offsets(max_lean: f32) -> impl Iterator<Item = Vector2<f32>> {
    let compass = [
        (1.0, 0.0),
        (FRAC_1_SQRT_2, FRAC_1_SQRT_2),
        (0.0, 1.0),
        (-FRAC_1_SQRT_2, FRAC_1_SQRT_2),
        (-1.0, 0.0),
        (-FRAC_1_SQRT_2, -FRAC_1_SQRT_2),
        (0.0, -1.0),
        (FRAC_1_SQRT_2, -FRAC_1_SQRT_2),
    ];
    std::iter::once(Vector2::zeros()).chain([0.5, 1.0].into_iter().flat_map(move |frac| {
        compass
            .into_iter()
            .map(move |(x, y)| Vector2::new(x, y) * max_lean * frac)
    }))
}

#[cfg(test)]
mod tests {
    use common::progress::Progress;
    use nalgebra::Vector3;
    use slicer::geometry::{mesh::Mesh, partitioning::Bvh};

    use super::*;

    /// Appends an axis-aligned box with outward facing triangles.
    fn add_box(
        verts: &mut Vec<Vector3<f32>>,
        faces: &mut Vec<[u32; 3]>,
        min: Vector3<f32>,
        max: Vector3<f32>,
    ) {
        let base = verts.len() as u32;
        for i in 0..8 {
            let pick = |bit, lo: f32, hi: f32| if i & bit == 0 { lo } else { hi };
            verts.push(Vector3::new(
                pick(1, min.x, max.x),
                pick(2, min.y, max.y),
                pick(4, min.z, max.z),
            ));
        }

        #[rustfmt::skip]
        let quads = [
            [0, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4],
            [2, 6, 7, 3], [0, 4, 6, 2], [1, 3, 7, 5],
        ];
        for [a, b, c, d] in quads {
            faces.push([base + a, base + b, base + c]);
            faces.push([base + a, base + c, base + d]);
        }
    }

    /// A 60 × 60 mm table (top at z = 10) with a 10 × 10 mm ledge floating at
    /// z = `ledge_z`. Every plate route from under the ledge is blocked by the
    /// table, so its points must land on the table top.
    fn ledge(ledge_z: f32) -> (Mesh, Bvh) {
        let (mut verts, mut faces) = (Vec::new(), Vec::new());
        let v = Vector3::new;
        add_box(
            &mut verts,
            &mut faces,
            v(-30.0, -30.0, 5.0),
            v(30.0, 30.0, 10.0),
        );
        add_box(
            &mut verts,
            &mut faces,
            v(-5.0, -5.0, ledge_z),
            v(5.0, 5.0, ledge_z + 2.0),
        );
        let mesh = Mesh::new(verts, faces);
        let bvh = Bvh::build(mesh.inner(), Progress::new());
        (mesh, bvh)
    }

    fn config() -> LandingConfig {
        LandingConfig {
            radius: 1.1,
            pillar_diameter: 1.2,
            cone_length: 2.0,
            contact_depth: 0.4,
            tail_depth: 0.2,
            slim_radius: 0.25,
            slim_depth: 0.3,
        }
    }

    #[test]
    fn lands_on_the_surface_below() {
        let (mesh, bvh) = ledge(20.0);
        let contact = Vector3::new(1.0, 1.0, 20.0);
        let landing = land_support(&mesh, &bvh, &config(), contact, -Vector3::z())
            .expect("a point above the table must land on it");
        assert_eq!(landing.kind, LandingKind::Pillar);

        // Straight-down tip, vertical pillar onto the table.
        assert!((landing.tip - Vector3::new(1.0, 1.0, 18.0)).magnitude() < 1e-4);
        assert_eq!(landing.foot.xy(), landing.tip.xy());
        assert!(landing.foot.z > 10.0 && landing.foot.z < landing.tip.z);

        // One tail, sunk exactly the tail depth into the table top.
        assert_eq!(landing.tails.len(), 1);
        let (end, normal) = landing.tails[0];
        assert!(
            (10.0 - end.z - 0.2).abs() < 1e-3,
            "tail sinks 0.2 mm, end at {end:?}"
        );
        assert!(normal.z > 0.99);

        // The tail is the tip cone mirrored: a full cone length long, plus sink.
        assert!(((landing.foot - end).magnitude() - 2.2).abs() < 1e-3);

        // The ledge is thicker than the touch depth, so the depth is unchanged.
        assert!((landing.depth - 0.4).abs() < 1e-6);
    }

    #[test]
    fn short_gap_gets_a_slim() {
        // The tip end sits 2.5 mm above the table, under the 4.1 mm minimum
        // drop for a pillar and tail, so a slim bridges the gap.
        let (mesh, bvh) = ledge(14.5);
        let contact = Vector3::new(1.0, 1.0, 14.5);
        let landing = land_support(&mesh, &bvh, &config(), contact, -Vector3::z())
            .expect("a short gap gets a slim");

        assert_eq!(landing.kind, LandingKind::Slim { radius: 0.25 });
        assert_eq!(landing.foot.xy(), landing.tip.xy());
        // Sunk 0.3 mm, then pulled back by the 0.25 mm slim radius.
        assert!((landing.foot.z - 9.95).abs() < 1e-4);
        let (sunk, normal) = landing.tails[0];
        assert!((sunk.z - 9.7).abs() < 1e-4);
        assert!(normal.z > 0.99);
    }

    #[test]
    fn tiny_gap_does_not_land() {
        // The tip end would sit 1 mm above the table: under 4 × slim Ø, too
        // short even for a slim.
        let (mesh, bvh) = ledge(13.0);
        let contact = Vector3::new(1.0, 1.0, 13.0);
        assert!(land_support(&mesh, &bvh, &config(), contact, -Vector3::z()).is_none());
    }

    #[test]
    fn gap_between_slim_and_pillar_does_not_land() {
        // The tip end sits 3.5 mm above the table: too long for a 3 mm slim,
        // too short for a pillar and tail.
        let (mesh, bvh) = ledge(15.5);
        let contact = Vector3::new(1.0, 1.0, 15.5);
        assert!(land_support(&mesh, &bvh, &config(), contact, -Vector3::z()).is_none());
    }

    #[test]
    fn claw_reaches_the_surface() {
        let (mesh, bvh) = ledge(20.0);
        let cfg = config();
        let tip = Vector3::new(1.0, 1.0, 18.0);
        let floor = 10.0 - cfg.pillar_diameter - FLOOR_EXTRA;
        let (foot, fingers) = claw(&mesh, &bvh, &cfg, tip, floor).expect("claw over a flat top");

        // The foot sits 0.8 cone lengths above the floor.
        assert!((foot.z - (floor + 1.6)).abs() < 1e-4);
        assert_eq!(fingers.len(), 1);

        // The finger sinks the tail depth along its own axis into the table.
        let (end, normal) = fingers[0];
        let axis = (foot - end).normalize();
        let sunk = (10.0 - end.z) / axis.z;
        assert!((sunk - 0.2).abs() < 1e-3, "finger sinks {sunk} mm");
        assert!(normal.z > 0.99);
    }

    #[test]
    fn contact_depth_is_capped_by_thickness() {
        // A 0.25 mm sheet: the tip can only sink as deep as the sheet is thick.
        let (mut verts, mut faces) = (Vec::new(), Vec::new());
        add_box(
            &mut verts,
            &mut faces,
            Vector3::new(-5.0, -5.0, 10.0),
            Vector3::new(5.0, 5.0, 10.25),
        );
        let mesh = Mesh::new(verts, faces);
        let bvh = Bvh::build(mesh.inner(), Progress::new());

        let depth = contact_depth(&mesh, &bvh, Vector3::new(1.0, 1.0, 10.0), 0.4);
        assert!((depth - 0.25).abs() < 1e-3, "depth {depth}");
    }
}
