//! Multi-line collision tests (§2.7).
//!
//! Every part a support builds — tip cone, link, pillar, vertical drop — is
//! collision-tested against the model by intersecting *several* offset line
//! segments, not just the centre line. This is what stops thin walls from
//! slipping through between samples: a part of radius `r` is sampled around its
//! whole circumference.
//!
//! All tests are pure functions over the model's BVH and are independent of how
//! a route was found, so routing (M3) and meshing can share them.

use std::f32::consts::TAU;

use nalgebra::Vector3;
use slicer::geometry::{mesh::Mesh, partitioning::Bvh};

/// Radius of the sampled shell for the horn (tip cone) test, as a fraction of
/// the cone's bottom diameter (§2.7). "0.6 × cone bottom Ø".
const HORN_SHELL: f32 = 0.6;
/// Number of lines to sample around a part's circumference for horn/link tests.
const SHELL_LINES: usize = 8;

/// Whether a straight segment from `a` to `b` intersects the model. This is the
/// cheapest test and corresponds to §2.7's "Vertical" test: an exact
/// segment-versus-triangle query (the BVH does the bbox prefilter internally).
pub fn segment_hits(mesh: &Mesh, bvh: &Bvh, a: Vector3<f32>, b: Vector3<f32>) -> bool {
    bvh.intersect_segment(mesh, a, b).is_some()
}

/// Horn (tip cone) test. The cone runs from `start` (sphere centre / sunk
/// contact) to `end` (tip end) with a bottom radius `cone_radius`. Eight lines
/// sample the shell at 45° steps, each at the sampled radius
/// `r = HORN_SHELL × cone bottom Ø` = 0.6 × cone bottom Ø (§2.7). Each line is
/// shifted toward the tip by `sink` (touch depth + max(0.5 × pillar Ø, 0.3 ×
/// cone length)) so the sunk part of the tip inside the model isn't falsely
/// reported as a collision.
pub fn horn_test(
    mesh: &Mesh,
    bvh: &Bvh,
    start: Vector3<f32>,
    end: Vector3<f32>,
    cone_radius: f32,
    sink: f32,
) -> bool {
    // Sampled shell radius: 0.6 × cone bottom Ø (§2.7).
    shell_test(mesh, bvh, start, end, cone_radius * HORN_SHELL, sink)
}

/// Main (vertical pillar) test. A vertical pillar of radius `radius` is tested
/// with lines at the centre and at 0.5 and 1.0 of the radius, from `top` down
/// to `bottom` (§2.7: "vertical lines at the centre and at 0.5 and 1.0 of the
/// radius").
pub fn main_test(
    mesh: &Mesh,
    bvh: &Bvh,
    top: Vector3<f32>,
    bottom: Vector3<f32>,
    radius: f32,
) -> bool {
    let axis = bottom - top;
    let len = axis.magnitude();
    if len <= f32::EPSILON {
        return false;
    }

    let [u, v] = slicer::geometry::mesh::orthogonal_basis(axis / len);

    // Centre line.
    if bvh.intersect_segment(mesh, top, bottom).is_some() {
        return true;
    }

    // 0.5 and 1.0 of the radius, around the circumference.
    for &frac in &[0.5f32, 1.0f32] {
        for i in 0..SHELL_LINES {
            let t = i as f32 / SHELL_LINES as f32 * TAU;
            let offset = (u * t.cos() + v * t.sin()) * radius * frac;
            if bvh
                .intersect_segment(mesh, top + offset, bottom + offset)
                .is_some()
            {
                return true;
            }
        }
    }

    false
}

/// Link test. A 45° link or brace from `start` to `end` of radius `radius` is
/// tested with the same multi-line shell approach as the horn test, but at the
/// link's full radius (no 0.6 factor) (§2.7).
pub fn link_test(
    mesh: &Mesh,
    bvh: &Bvh,
    start: Vector3<f32>,
    end: Vector3<f32>,
    radius: f32,
) -> bool {
    shell_test(mesh, bvh, start, end, radius, 0.0)
}

/// The shared multi-line shell test: `n` offset lines around the axis of a
/// segment, each shifted along the axis by `sink` toward the end.
fn shell_test(
    mesh: &Mesh,
    bvh: &Bvh,
    start: Vector3<f32>,
    end: Vector3<f32>,
    radius: f32,
    sink: f32,
) -> bool {
    let axis = end - start;
    let len = axis.magnitude();
    if len <= f32::EPSILON {
        return false;
    }

    let direction = axis / len;
    let [u, v] = slicer::geometry::mesh::orthogonal_basis(direction);
    // Shift the sample start toward the tip end by `sink` so the sunk contact
    // portion isn't reported as a collision.
    let start = start + direction * sink;

    for i in 0..SHELL_LINES {
        let t = i as f32 / SHELL_LINES as f32 * TAU;
        let offset = (u * t.cos() + v * t.sin()) * radius;
        if bvh
            .intersect_segment(mesh, start + offset, end + offset)
            .is_some()
        {
            return true;
        }
    }

    false
}

#[cfg(test)]
mod tests {
    use common::progress::Progress;
    use nalgebra::Vector3;
    use slicer::geometry::{mesh::Mesh, partitioning::Bvh};

    use super::*;

    /// A horizontal thin wall (slab) centered at z = 5, 0.2 mm thick, in a small
    /// XY area. Any vertical pillar passing through z = 5 in that area must be
    /// caught by the main test.
    fn thin_wall() -> (Mesh, Bvh) {
        let v = Vector3::new;
        let verts = vec![
            v(-10.0, -10.0, 4.9),
            v(10.0, -10.0, 4.9),
            v(10.0, 10.0, 4.9),
            v(-10.0, 10.0, 4.9),
            v(-10.0, -10.0, 5.1),
            v(10.0, -10.0, 5.1),
            v(10.0, 10.0, 5.1),
            v(-10.0, 10.0, 5.1),
        ];
        // Outward-facing quads (bottom, top, four sides), split into triangles.
        let faces = vec![
            [0, 2, 1],
            [0, 3, 2], // bottom (down)
            [4, 5, 6],
            [4, 6, 7], // top (up)
            [0, 1, 5],
            [0, 5, 4], // -y side
            [1, 2, 6],
            [1, 6, 5], // +x side
            [2, 3, 7],
            [2, 7, 6], // +y side
            [3, 0, 4],
            [3, 4, 7], // -x side
        ];
        let mesh = Mesh::new(verts, faces);
        let bvh = Bvh::build(mesh.inner(), Progress::new());
        (mesh, bvh)
    }

    #[test]
    fn main_test_catches_pillar_through_wall() {
        let (mesh, bvh) = thin_wall();
        // A vertical pillar from above the wall down through it.
        let top = Vector3::new(0.0, 0.0, 8.0);
        let bottom = Vector3::new(0.0, 0.0, 0.0);
        assert!(main_test(&mesh, &bvh, top, bottom, 0.6));
    }

    #[test]
    fn main_test_passes_in_free_space() {
        let (mesh, bvh) = thin_wall();
        // A pillar beside the wall (outside its XY extent) is clear.
        let top = Vector3::new(20.0, 0.0, 8.0);
        let bottom = Vector3::new(20.0, 0.0, 0.0);
        assert!(!main_test(&mesh, &bvh, top, bottom, 0.6));
    }

    #[test]
    fn segment_hits_wall() {
        let (mesh, bvh) = thin_wall();
        assert!(segment_hits(
            &mesh,
            &bvh,
            Vector3::new(0.0, 0.0, 8.0),
            Vector3::new(0.0, 0.0, 0.0),
        ));
        assert!(!segment_hits(
            &mesh,
            &bvh,
            Vector3::new(20.0, 0.0, 8.0),
            Vector3::new(20.0, 0.0, 0.0),
        ));
    }
}
