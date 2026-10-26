//! Edge re-seat of placed points.
//!
//! Runs once per placed point, between placement and routing, on every point
//! except islands. A point within about one grid cell of a crease is moved
//! onto the most downward-facing face touching it, one contact radius inside
//! that face's edge, and takes the face's normal. Points inside a face, on a
//! short gap, or near only steep faces are left alone.

use nalgebra::Vector3;
use slicer::geometry::{mesh::Mesh, partitioning::Bvh};

use crate::supports::auto::points::PlacedPoint;

/// Two normals lie on the same plane when their dot product exceeds this.
const SAME_PLANE: f32 = 0.99;
/// The most downward candidate face must have `n.z` at or below this,
/// `−cos 46°` (a fixed constant, not derived from the support angle).
const MAX_NORMAL_Z: f32 = -0.694_658_4;
/// The edge march takes this many steps.
const MARCH_STEPS: usize = 20;
/// March step as a fraction of `1.9 × pillar Ø`.
const MARCH_STEP_FACTOR: f32 = 0.01 * 1.9;
/// Half-length of the segment tested across the face at each march step.
const PROBE: f32 = 0.1;
/// Intersection tolerance of the probe segment.
const PROBE_TOLERANCE: f32 = 0.002;
/// The point's gap is looked up this far below the point.
pub(super) const LOOKUP_DROP: f32 = 0.01;
/// A run of on-plane neighbours this long (inclusive) marks a face border.
const RUN: std::ops::RangeInclusive<usize> = 3..=6;

/// Sizes the re-seat works with.
#[derive(Clone, Copy, Debug)]
pub struct EdgeConfig {
    /// Pillar diameter; the march step is `0.019 ×` this.
    pub pillar_diameter: f32,
    /// How far inside the edge the point ends up: the tip cone's radius at the
    /// contact depth, `top r + (depth / cone length) × (bottom r − top r)`.
    pub inset: f32,
    /// Points over a gap shorter than this (the longest slim support, 3 mm)
    /// are left alone, unless nothing is below them.
    pub min_gap: f32,
}

/// The interval grid the re-seat walks: placement's grid and links.
/// Directions are the 8 slots of [`placement::DIRS`](super::placement::DIRS).
pub(super) trait EdgeGrid {
    /// One gap (interval) of one column.
    type Node: Copy;
    /// The gap holding `point` (looked up [`LOOKUP_DROP`] below it),
    /// unless it is a gap shorter than `min_gap` with material below.
    fn own(&self, point: Vector3<f32>, min_gap: f32) -> Option<Self::Node>;
    /// The gap's face point (its upper crossing) and that face's normal.
    fn face(&self, node: Self::Node) -> (Vector3<f32>, Vector3<f32>);
    /// The gap linked to `node` in direction slot `dir`.
    fn link(&self, node: Self::Node, dir: usize) -> Option<Self::Node>;
    /// The xy centre of the cell one step from `node`'s cell in slot `dir`.
    fn neighbour_centre(&self, node: Self::Node, dir: usize) -> Option<Vector3<f32>>;
    /// The xy centre of the cell holding `point`.
    fn centre_of(&self, point: Vector3<f32>) -> Option<Vector3<f32>>;
}

/// Re-seats every non-island point in `points` (see the module docs).
/// Returns how many points moved.
pub(super) fn adjust_points(
    points: &mut [PlacedPoint],
    grid: &impl EdgeGrid,
    mesh: &Mesh,
    bvh: &Bvh,
    config: &EdgeConfig,
) -> usize {
    let mut moved = 0;
    for point in points.iter_mut().filter(|p| !p.island) {
        if let Some((pos, normal)) = adjust(point.point, grid, mesh, bvh, config) {
            point.point = pos;
            point.normal = normal;
            moved += 1;
        }
    }
    moved
}

/// The re-seated position and normal of one point, or `None` to keep it.
pub(super) fn adjust<G: EdgeGrid>(
    point: Vector3<f32>,
    grid: &G,
    mesh: &Mesh,
    bvh: &Bvh,
    config: &EdgeConfig,
) -> Option<(Vector3<f32>, Vector3<f32>)> {
    let own = grid.own(point, config.min_gap)?;
    let point_centre = grid.centre_of(point)?;

    // Candidate faces: the point's own, then each linked neighbour whose
    // normal differs from every candidate so far.
    let first = grid.face(own).1;
    let mut candidates = vec![own];
    for dir in 0..8 {
        let Some(link) = grid.link(own, dir) else {
            continue;
        };
        let normal = grid.face(link).1;
        let distinct = first.dot(&normal) < SAME_PLANE
            && (candidates[1..].iter()).all(|&c| grid.face(c).1.dot(&normal) <= SAME_PLANE);
        if distinct {
            candidates.push(link);
        }
    }

    while !candidates.is_empty() {
        // Most downward candidate (first wins ties).
        let (index, &centre) = (candidates.iter().enumerate())
            .min_by(|a, b| {
                let (za, zb) = (grid.face(*a.1).1.z, grid.face(*b.1).1.z);
                za.total_cmp(&zb).then(a.0.cmp(&b.0))
            })
            .unwrap();
        let normal = grid.face(centre).1;
        if normal.z > MAX_NORMAL_Z {
            return None;
        }
        if let Some(result) = reseat(grid, centre, normal, point_centre, mesh, bvh, config) {
            return Some(result);
        }
        candidates.remove(index);
    }
    None
}

/// One candidate face: the border test, the march to the edge, the inset, and
/// the final re-hit.
fn reseat<G: EdgeGrid>(
    grid: &G,
    centre: G::Node,
    normal: Vector3<f32>,
    point_centre: Vector3<f32>,
    mesh: &Mesh,
    bvh: &Bvh,
    config: &EdgeConfig,
) -> Option<(Vector3<f32>, Vector3<f32>)> {
    let step = MARCH_STEP_FACTOR * config.pillar_diameter;
    // Which of the centre's neighbours lie on the same plane, and where.
    let mut on_plane = [false; 8];
    let mut offsets = [Vector3::zeros(); 8];
    for dir in 0..8 {
        if let Some(link) = grid.link(centre, dir) {
            on_plane[dir] = grid.face(link).1.dot(&normal) > SAME_PLANE;
        }
        if let Some(c) = grid.neighbour_centre(centre, dir) {
            offsets[dir] = c - point_centre;
        }
    }

    // The first off→on transition, then the unbroken run of on-plane
    // directions after it. A border has a run of 3 to 6.
    let start = (0..8).find(|&k| !on_plane[k] && on_plane[(k + 1) % 8])? + 1;
    let mut run = 0;
    let mut sum = Vector3::zeros();
    while run < 8 && on_plane[(start + run) % 8] {
        sum += offsets[(start + run) % 8];
        run += 1;
    }
    if !RUN.contains(&run) {
        return None;
    }

    // Into the face, in its plane.
    let flat = Vector3::new(sum.x, sum.y, 0.0);
    let into = normal
        .cross(&flat.cross(&normal))
        .try_normalize(f32::EPSILON)?;

    // March out to the edge; `edge` is the last step still on the face.
    let start = grid.face(centre).0;
    let mut edge = start;
    for k in 1..=MARCH_STEPS {
        let probe = start - into * (step * k as f32);
        match plane_hit(probe, normal, mesh, bvh) {
            Some((pos, _)) => edge = pos,
            None => break,
        }
    }

    // One contact radius back into the face, then onto its surface.
    let target = edge + into * (config.inset - 0.5 * step);
    plane_hit(target, normal, mesh, bvh)
}

/// Where the segment `at ± 0.1·normal` crosses a triangle on the plane of
/// `normal` (normals within `SAME_PLANE`), with that triangle's normal.
fn plane_hit(
    at: Vector3<f32>,
    normal: Vector3<f32>,
    mesh: &Mesh,
    bvh: &Bvh,
) -> Option<(Vector3<f32>, Vector3<f32>)> {
    let origin = at + normal * PROBE;
    let reach = 2.0 * PROBE + PROBE_TOLERANCE;
    bvh.intersect_ray_all(mesh, origin, -normal)
        .into_iter()
        .filter_map(|hit| {
            let pos = mesh.transform(&hit.position);
            let face = mesh.transform_normal(&mesh.normal(hit.face)).normalize();
            let distance = (pos - origin).magnitude();
            (distance <= reach && face.dot(&normal) > SAME_PLANE).then_some((distance, pos, face))
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, pos, face)| (pos, face))
}
