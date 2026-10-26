//! Plate routing: from a contact point to a pillar standing on the plate.
//!
//! A support first tries to drop straight down from its tip. When that is
//! blocked, it looks for a free pillar site nearby ([`candidate_sites`]) and
//! reaches it with a 45° link from the tip end. Each try runs the tip, link
//! and pillar collision tests; the first that passes all three wins.

use std::f32::consts::FRAC_1_SQRT_2;

use common::units::Milimeter;
use nalgebra::{Vector2, Vector3};
use slicer::geometry::{mesh::Mesh, partitioning::Bvh};

use crate::supports::{
    auto::{collision, tip},
    mesh::SupportPreset,
};

/// The sizes plate routing works with, in millimetres.
#[derive(Clone, Copy, Debug)]
pub struct RouteSizes {
    /// Pillar radius plus `margin`: the collision radius of links and pillars.
    pub radius: f32,
    /// Clearance from the model.
    pub margin: f32,
    /// Radius of the tip at the contact.
    pub tip_radius: f32,
    /// Cone length.
    pub tip_length: f32,
    /// How far the tip sinks into the model.
    pub contact_depth: f32,
}

impl RouteSizes {
    pub fn from_preset(support: &SupportPreset) -> Self {
        Self {
            radius: (support.radius + support.margin).get::<Milimeter>(),
            margin: support.margin.get::<Milimeter>(),
            tip_radius: support.tip_radius.get::<Milimeter>(),
            tip_length: support.tip_length.get::<Milimeter>(),
            contact_depth: support.contact_depth.get::<Milimeter>(),
        }
    }

    /// Whether a tip leaving `contact` along `dir` hits the model. The cone
    /// runs from the sunk tip head `contact − dir × depth` a cone length on.
    /// Its sampled shell is 0.6 × pillar Ø at the end, tapered by tip Ø /
    /// pillar Ø toward the head, and each line starts
    /// `depth + max(0.5 × pillar Ø, 0.3 × cone length)` in, so the sunk part
    /// of the tip doesn't count.
    pub(super) fn horn_hits(
        &self,
        mesh: &Mesh,
        bvh: &Bvh,
        contact: Vector3<f32>,
        dir: Vector3<f32>,
    ) -> bool {
        let pillar_radius = self.radius - self.margin;
        let head = contact - dir * self.contact_depth;
        let end = head + dir * self.tip_length;
        let end_radius = 0.6 * 2.0 * pillar_radius;
        let head_radius = end_radius * self.tip_radius / pillar_radius;
        let shift = self.contact_depth + pillar_radius.max(0.3 * self.tip_length);
        collision::horn_cone(mesh, bvh, head, end, head_radius, end_radius, shift)
    }
}

/// Returns `Some((tip_end, joint))` for a support at `contact` with outward
/// normal `normal`. `tip_end` is where the tip cone ends and `joint` is where
/// the 45° link meets the vertical pillar (which then drops to the plate).
///
/// Order of attempts:
/// 1. Straight drop: the tip points down (clamped) and a vertical pillar from
///    `tip_end` to the plate passes. `joint == tip_end`.
/// 2. Lateral search: the candidate pillar sites of [`candidate_sites`],
///    nearest to the contact first. For each site we vary the tip direction
///    ([`tip_directions`]) and try a 45° link from `tip_end` down to the
///    pillar top, then a vertical pillar. The first combination that passes
///    every collision test wins.
pub fn route_support(
    mesh: &Mesh,
    bvh: &Bvh,
    sizes: &RouteSizes,
    contact: Vector3<f32>,
    normal: Vector3<f32>,
) -> Option<(Vector3<f32>, Vector3<f32>)> {
    let normal = tip::clamp_tip_direction(normal.normalize());
    let RouteSizes {
        radius,
        margin,
        tip_length,
        contact_depth,
        ..
    } = *sizes;

    // 1. Straight drop. The cone runs `tip_length` from the sunk anchor
    // `contact − normal × contact_depth`, so its end is
    // `normal × (tip_length − contact_depth)` from the contact. The pillar is
    // coaxial with the cone end, not the sunk anchor.
    let tip_end = contact + normal * (tip_length - contact_depth);
    if !sizes.horn_hits(mesh, bvh, contact, normal)
        && !collision::main_test(mesh, bvh, tip_end, tip_end.xy().to_homogeneous(), radius)
    {
        return Some((tip_end, tip_end));
    }

    // 2. Lateral search.
    for site in candidate_sites(mesh, bvh, contact, radius, margin, tip_length) {
        let offset = site - contact.xy();
        for dir in tip_directions(normal, offset) {
            let candidate_tip = contact + dir * (tip_length - contact_depth);
            // The joint is at the site's XY, dropped by the link's 45° run.
            let run = (site - candidate_tip.xy()).magnitude();
            let joint = site.push(candidate_tip.z - run);
            if joint.z <= 0.0 {
                continue;
            }

            // Tip, link, pillar.
            if sizes.horn_hits(mesh, bvh, contact, dir)
                || collision::link_test(mesh, bvh, candidate_tip, joint, radius)
                || collision::main_test(mesh, bvh, joint, joint.xy().to_homogeneous(), radius)
            {
                continue;
            }

            return Some((candidate_tip, joint));
        }
    }

    None
}

/// The maximum lateral offset from the contact to search for a pillar site, in
/// millimetres.
const MAX_OFFSET: f32 = 15.0;

/// The step unit of the site search, in mm: the finest placement grid cell.
const CELL_SIZE: f32 = 0.1;

/// The 8 walk directions, counter-clockwise from +x.
const WALK_DIRS: [(f32, f32); 8] = [
    (1.0, 0.0),
    (FRAC_1_SQRT_2, FRAC_1_SQRT_2),
    (0.0, 1.0),
    (-FRAC_1_SQRT_2, FRAC_1_SQRT_2),
    (-1.0, 0.0),
    (-FRAC_1_SQRT_2, -FRAC_1_SQRT_2),
    (0.0, -1.0),
    (FRAC_1_SQRT_2, -FRAC_1_SQRT_2),
];

/// Where a point sits relative to the plate: open to it (nothing below), in
/// an air pocket with model below, or inside the model.
#[derive(PartialEq)]
enum Ground {
    Open,
    Pocket,
    Solid,
}

fn ground_at(mesh: &Mesh, bvh: &Bvh, point: Vector3<f32>) -> Ground {
    match bvh.intersect_ray(mesh, point, -Vector3::z()) {
        None => Ground::Open,
        Some(hit) if mesh.transform_normal(&mesh.normal(hit.face)).z < 0.0 => Ground::Solid,
        Some(_) => Ground::Pocket,
    }
}

/// Candidate pillar sites (XY), nearest to the contact first. Both searches
/// start from `origin = contact − (0, 0, 0.8 × cone length)`:
/// * [`clear_squares`]: open squares of side
///   `reach = min(0.8 × MAX_OFFSET, 2 × radius)`, the one nearest to the
///   origin and to each of the 8 points `reach` away;
/// * per direction, a walk down at 45° that stops in the model or below the
///   pillar Ø, and emits one site once 11 consecutive steps are open to the
///   plate.
///
/// A contact deep in a pocket thus gets no site: its origin, or the walk's
/// first steps, are already inside the material below.
fn candidate_sites(
    mesh: &Mesh,
    bvh: &Bvh,
    contact: Vector3<f32>,
    radius: f32,
    margin: f32,
    tip_length: f32,
) -> Vec<Vector2<f32>> {
    let origin = contact - Vector3::z() * (0.8 * tip_length);
    let mut sites = clear_squares(mesh, bvh, origin, radius);

    let lead = 0.5 * radius + CELL_SIZE;
    let step = lead / 5.0;
    let steps = ((MAX_OFFSET - lead) / step) as usize;
    // The walk ends a pillar diameter above the plate.
    let floor = 2.0 * (radius - margin);
    for (dx, dy) in WALK_DIRS {
        let dir = Vector2::new(dx, dy);
        let mut pos = origin;
        let mut previous = origin;
        let (mut run, mut crossed) = (0, false);
        for _ in 0..steps {
            pos += (dir * step).push(-step);
            if pos.z < floor {
                break;
            }
            match ground_at(mesh, bvh, pos) {
                Ground::Solid => break,
                Ground::Pocket => (run, crossed) = (0, true),
                Ground::Open => {
                    run += 1;
                    if run == 11 {
                        // Back off from the last step; further after crossing
                        // a pocket.
                        let back = if crossed {
                            (margin + 0.1).min(previous.z - floor)
                        } else {
                            0.1
                        };
                        sites.push(previous.xy() + dir * back);
                        break;
                    }
                }
            }
            previous = pos;
        }
    }

    let dist = |s: &Vector2<f32>| (s - contact.xy()).magnitude();
    sites.sort_by(|a, b| dist(a).total_cmp(&dist(b)));
    sites
}

/// A summed-area table over the cells around `origin` marks columns whose
/// lowest model surface is at or below `origin.z`; every fully clear square
/// of `side` cells yields its centre (half a square back from the far cell).
/// The result is the centre nearest (Manhattan) to the origin and to each of
/// the 8 points `reach` away, without repeats (0.1 mm per axis).
fn clear_squares(mesh: &Mesh, bvh: &Bvh, origin: Vector3<f32>, radius: f32) -> Vec<Vector2<f32>> {
    let reach = (0.8 * MAX_OFFSET).min(2.0 * radius);
    let side = (reach / CELL_SIZE + 1.0) as isize;
    let n = 2 * side + 3;
    let cell = |i: isize| (i - side - 1) as f32 * CELL_SIZE;
    // `blocked[i][j]` for cells i, j in 1..n; row/column 0 is the zero border.
    let below = origin.z - 1e4;
    let mut table = vec![0i32; (n * n) as usize];
    let at = |i: isize, j: isize| (i * n + j) as usize;
    for i in 1..n {
        for j in 1..n {
            let xy = origin.xy() + Vector2::new(cell(i), cell(j));
            let blocked = bvh
                .intersect_ray(mesh, xy.push(below), Vector3::z())
                .is_some_and(|hit| mesh.transform(&hit.position).z <= origin.z);
            table[at(i, j)] = blocked as i32 + table[at(i - 1, j)] + table[at(i, j - 1)]
                - table[at(i - 1, j - 1)];
        }
    }
    let shift = Vector2::repeat(0.5 * side as f32 * CELL_SIZE);
    let mut centres = Vec::new();
    for i in side..n {
        for j in side..n {
            let sum = table[at(i, j)] - table[at(i - side, j)] - table[at(i, j - side)]
                + table[at(i - side, j - side)];
            if sum == 0 {
                centres.push(origin.xy() + Vector2::new(cell(i), cell(j)) - shift);
            }
        }
    }

    let targets = std::iter::once(origin.xy()).chain(
        WALK_DIRS
            .iter()
            .map(|&(dx, dy)| origin.xy() + Vector2::new(dx, dy) * reach),
    );
    let mut out: Vec<Vector2<f32>> = Vec::new();
    for target in targets {
        let manhattan = |c: &Vector2<f32>| (c - target).abs().sum();
        let nearest = centres
            .iter()
            .fold(None::<&Vector2<f32>>, |best, c| match best {
                Some(b) if manhattan(b) <= manhattan(c) => Some(b),
                _ => Some(c),
            });
        if let Some(&c) = nearest
            && out.iter().all(|q| (q - c).abs().max() > 0.1)
        {
            out.push(c);
        }
    }
    out
}

/// The tip directions to try for a candidate site. `offset = site − contact`
/// in XY, `dist = |offset|`. The two tilted directions are
/// `normalize(offset, −k·dist)` for k = 1 (45°) and k = 0.5 (≈26.6°). If the
/// site lies away from the normal's horizontal part
/// (`dot(offset, n.xy) < 0`) we try straight down then the 45° tilt;
/// otherwise the clamped normal, then 45°, then the shallower tilt, then
/// straight down.
fn tip_directions(normal: Vector3<f32>, offset: Vector2<f32>) -> Vec<Vector3<f32>> {
    let down = -Vector3::z();

    // A site directly below the contact: only straight down makes sense.
    let dist = offset.magnitude();
    if dist <= f32::EPSILON {
        return vec![down, normal];
    }

    let toward_45 = offset.push(-dist).normalize();
    let toward_shallow = offset.push(-0.5 * dist).normalize();

    if offset.dot(&normal.xy()) < 0.0 {
        vec![down, toward_45]
    } else {
        vec![normal, toward_45, toward_shallow, down]
    }
}
