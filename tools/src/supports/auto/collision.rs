//! Multi-line collision tests.
//!
//! Every part a support builds — tip cone, link, pillar, vertical drop — is
//! collision-tested against the model by intersecting *several* offset line
//! segments, not just the centre line. This is what stops thin walls from
//! slipping through between samples: a part of radius `r` is sampled around its
//! whole circumference.
//!
//! All tests are pure functions over the model's BVH and are independent of how
//! a route was found, so routing and meshing can share them.

use std::f32::consts::TAU;

use nalgebra::Vector3;
use slicer::geometry::{mesh::Mesh, partitioning::Bvh};

/// Radius of the sampled shell for the horn (tip cone) test, as a fraction of
/// the cone's bottom diameter.
const HORN_SHELL: f32 = 0.6;
/// Number of lines to sample around a part's circumference for horn/link tests.
const SHELL_LINES: usize = 8;

/// Whether a straight segment from `a` to `b` intersects the model. This is the
/// cheapest test: an exact segment-versus-triangle query (the BVH does the bbox prefilter internally).
pub fn segment_hits(mesh: &Mesh, bvh: &Bvh, a: Vector3<f32>, b: Vector3<f32>) -> bool {
    bvh.intersect_segment(mesh, a, b).is_some()
}

/// Horn (tip cone) test. The cone runs from `start` (sphere centre / sunk
/// contact) to `end` (tip end) with a bottom radius `cone_radius`. Eight lines
/// sample the shell at 45° steps, each at the sampled radius
/// `r = HORN_SHELL × cone bottom Ø`. Each line is
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
    shell_test(mesh, bvh, start, end, cone_radius * HORN_SHELL, sink)
}

/// The routing horn test: 8 lines at 45° steps around the tip axis from the sunk tip head
/// `head` to the tip `end`, tapered from `head_radius` to `end_radius`. Each
/// line's start moves `shift` along the line toward the end, so the sunk
/// contact region isn't reported. A line the shift reaches past is skipped.
pub fn horn_cone(
    mesh: &Mesh,
    bvh: &Bvh,
    head: Vector3<f32>,
    end: Vector3<f32>,
    head_radius: f32,
    end_radius: f32,
    shift: f32,
) -> bool {
    let axis = end - head;
    let len = axis.magnitude();
    if len <= f32::EPSILON {
        return false;
    }
    let [u, v] = slicer::geometry::mesh::orthogonal_basis(axis / len);
    (0..SHELL_LINES).any(|i| {
        let t = i as f32 / SHELL_LINES as f32 * TAU;
        let perp = u * t.cos() + v * t.sin();
        let (a, b) = (head + perp * head_radius, end + perp * end_radius);
        let line = b - a;
        let length = line.magnitude();
        length > shift
            && bvh
                .intersect_segment(mesh, a + line * (shift / length), b)
                .is_some()
    })
}

/// Main (vertical pillar) test. A vertical pillar of radius `radius` is tested
/// with lines at the centre and at 0.5 and 1.0 of the radius, from `top` down
/// to `bottom`.
pub fn main_test(
    mesh: &Mesh,
    bvh: &Bvh,
    top: Vector3<f32>,
    bottom: Vector3<f32>,
    radius: f32,
) -> bool {
    main_lines(top, bottom, radius).any(|(a, b)| bvh.intersect_segment(mesh, a, b).is_some())
}

/// The highest point (world z) at which a vertical pillar of radius `radius`
/// dropped from `top` to `bottom` first touches the model, sampled with the
/// same lines as [`main_test`]. `None` when the drop is clear. Model-landing
/// supports use this to find the surface their pillar lands on.
pub fn main_first_hit(
    mesh: &Mesh,
    bvh: &Bvh,
    top: Vector3<f32>,
    bottom: Vector3<f32>,
    radius: f32,
) -> Option<f32> {
    main_lines(top, bottom, radius)
        .filter_map(|(a, b)| bvh.intersect_segment(mesh, a, b))
        .map(|hit| mesh.transform(&hit.position).z)
        .reduce(f32::max)
}

/// The sample lines of the main test: the centre line, then lines at 0.5 and
/// 1.0 of the radius around the circumference.
fn main_lines(
    top: Vector3<f32>,
    bottom: Vector3<f32>,
    radius: f32,
) -> impl Iterator<Item = (Vector3<f32>, Vector3<f32>)> {
    let axis = bottom - top;
    let len = axis.magnitude();
    let basis = (len > f32::EPSILON).then(|| slicer::geometry::mesh::orthogonal_basis(axis / len));

    basis.into_iter().flat_map(move |[u, v]| {
        let shell = [0.5f32, 1.0f32].into_iter().flat_map(move |frac| {
            (0..SHELL_LINES).map(move |i| {
                let t = i as f32 / SHELL_LINES as f32 * TAU;
                let offset = (u * t.cos() + v * t.sin()) * radius * frac;
                (top + offset, bottom + offset)
            })
        });
        std::iter::once((top, bottom)).chain(shell)
    })
}

/// Link test. A 45° link or brace from `start` to `end` of radius `radius` is
/// tested with the same multi-line shell approach as the horn test, but at the
/// link's full radius (no 0.6 factor).
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

    // A sink that reaches past the end leaves nothing to test (and would
    // otherwise flip the sampled segment beyond the end).
    if sink >= len {
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
