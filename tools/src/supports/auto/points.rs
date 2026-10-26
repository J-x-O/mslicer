//! The placed point type and the island candidates (the lowest mesh
//! vertices), used by [`placement`](crate::supports::auto::placement).

use nalgebra::{Vector2, Vector3};
use slicer::geometry::{mesh::Mesh, partitioning::Bvh};

/// A placed support contact point: world-space position and outward normal.
#[derive(Clone, Copy, Debug)]
pub struct PlacedPoint {
    pub point: Vector3<f32>,
    pub normal: Vector3<f32>,
    /// Placed as an island. Islands skip the edge re-seat ([`edge`](crate::supports::auto::edge)).
    pub island: bool,
}

/// A hanging-feature candidate: a world-space mesh vertex and its outward
/// (area-weighted) vertex normal.
#[derive(Clone, Copy, Debug)]
pub struct LowVertex {
    pub pos: Vector3<f32>,
    pub normal: Vector3<f32>,
}

/// Gathers the mesh's lowest vertices, the island candidates.
///
/// The domain is the mesh vertices in **world space** (the model transform is
/// applied, so a rotated model is tested in its printed orientation). A vertex
/// is a candidate iff no sibling in any of its triangles is strictly lower in
/// z (`<=`-inclusive, so ties survive). Unlike the ray-grid fill this test
/// has no face-angle gate, so it sees the lowest corner of a tilted feature
/// whose faces are too steep for the support-angle gate.
///
/// A minimum buried inside another shell (the first crossing straight below it
/// is a downward-facing exit) is dropped, since it has material under it.
pub fn gather_lowest_vertices(mesh: &Mesh, bvh: &Bvh) -> Vec<LowVertex> {
    let vertices: Vec<Vector3<f32>> = mesh.vertices().iter().map(|v| mesh.transform(v)).collect();
    let faces = mesh.faces();

    let mut survivor = vec![true; vertices.len()];
    let mut normals = vec![Vector3::zeros(); vertices.len()];
    for &[i, j, k] in faces {
        let (a, b, c) = (
            vertices[i as usize],
            vertices[j as usize],
            vertices[k as usize],
        );
        for (vi, vz, w1, w2) in [(i, a.z, b.z, c.z), (j, b.z, c.z, a.z), (k, c.z, a.z, b.z)] {
            if w1 < vz || w2 < vz {
                survivor[vi as usize] = false;
            }
        }
        // Same winding as `Mesh::normal`, unnormalised so it is area-weighted.
        let n = (c - b).cross(&(a - b));
        for v in [i, j, k] {
            normals[v as usize] += n;
        }
    }

    (0..vertices.len())
        .filter(|&i| survivor[i])
        .filter_map(|i| {
            let pos = vertices[i];
            let normal = normals[i].try_normalize(f32::EPSILON)?;
            // Start just below the vertex: a true minimum has air there.
            let below = bvh.intersect_ray(mesh, pos - Vector3::z() * 1e-3, -Vector3::z());
            let buried =
                below.is_some_and(|hit| mesh.transform_normal(&mesh.normal(hit.face)).z < 0.0);
            (!buried).then_some(LowVertex { pos, normal })
        })
        .collect()
}

/// The island filter: bucket, z-sort, cap the runs.
///
/// Candidates are bucketed on a **1.0 mm** XY grid anchored at the model's XY
/// minimum `origin` with a half-cell (**0.5 mm**) offset,
/// each bucket z-sorted lowest-first, and split into runs terminated when a
/// point is **strictly more than 1.0 mm above the run's anchor** (its lowest
/// point). A run of **≥ 11** points is a dense flat fan and is dropped
/// entirely; a shorter run emits **every** point. The final island pitch is
/// set afterwards by the `0.75×n` coverage disc, not here.
pub(super) fn filter_low_points<T: Copy>(
    candidates: &[T],
    origin: Vector2<f32>,
    pos: impl Fn(&T) -> Vector3<f32>,
) -> Vec<T> {
    const BUCKET: f32 = 1.0;
    const OFFSET: f32 = 0.5;
    const Z_GAP: f32 = 1.0;
    const RUN_CAP: usize = 11;

    let mut buckets: std::collections::HashMap<(i64, i64), Vec<T>> =
        std::collections::HashMap::new();
    for &c in candidates {
        let p = pos(&c);
        let kx = ((p.x - origin.x + OFFSET) / BUCKET).floor() as i64;
        let ky = ((p.y - origin.y + OFFSET) / BUCKET).floor() as i64;
        buckets.entry((kx, ky)).or_default().push(c);
    }

    let mut kept: Vec<T> = Vec::new();
    for bucket in buckets.values_mut() {
        bucket.sort_by(|a, b| pos(a).z.total_cmp(&pos(b).z));
        let mut start = 0usize;
        while start < bucket.len() {
            let anchor_z = pos(&bucket[start]).z;
            let mut end = start + 1;
            while end < bucket.len() && pos(&bucket[end]).z - anchor_z <= Z_GAP {
                end += 1;
            }
            if end - start < RUN_CAP {
                kept.extend_from_slice(&bucket[start..end]);
            }
            start = end;
        }
    }

    kept
}

#[cfg(test)]
mod tests {
    use common::progress::Progress;
    use nalgebra::Vector3;

    use super::*;

    /// Pushes an axis-aligned box rotated about the x axis by `tilt` radians
    /// around `centre`, outward-wound.
    fn add_tilted_box(
        verts: &mut Vec<Vector3<f32>>,
        faces: &mut Vec<[u32; 3]>,
        centre: Vector3<f32>,
        half: Vector3<f32>,
        tilt: f32,
    ) {
        let rot = nalgebra::Rotation3::from_axis_angle(&Vector3::x_axis(), tilt);
        let base = verts.len() as u32;
        for i in 0..8 {
            let corner = Vector3::new(
                if i & 1 == 0 { -half.x } else { half.x },
                if i & 2 == 0 { -half.y } else { half.y },
                if i & 4 == 0 { -half.z } else { half.z },
            );
            verts.push(centre + rot * corner);
        }
        for [a, b, c, d] in [
            [0, 2, 3, 1],
            [4, 5, 7, 6],
            [0, 1, 5, 4],
            [2, 6, 7, 3],
            [0, 4, 6, 2],
            [1, 3, 7, 5],
        ] {
            faces.push([base + a, base + b, base + c]);
            faces.push([base + a, base + c, base + d]);
        }
    }

    #[test]
    fn lowest_vertex_finds_tilted_feature_corner() {
        // A tall brick tilted 20° from vertical: every face is steeper than the
        // 45° support-angle gate, yet its lowest corner is a hanging island.
        let (mut verts, mut faces) = (Vec::new(), Vec::new());
        add_tilted_box(
            &mut verts,
            &mut faces,
            Vector3::new(0.0, 0.0, 20.0),
            Vector3::new(2.0, 2.0, 8.0),
            20f32.to_radians(),
        );
        let mesh = Mesh::new(verts, faces);
        let bvh = Bvh::build(mesh.inner(), Progress::new());

        let low = gather_lowest_vertices(&mesh, &bvh);
        let min_z = mesh
            .vertices()
            .iter()
            .map(|v| v.z)
            .fold(f32::INFINITY, f32::min);
        assert!(!low.is_empty());
        assert!(low.iter().all(|v| (v.pos.z - min_z).abs() < 1e-4));
        // No incident face passes the angle gate, but the vertex normal points
        // down, so the tip still has a sensible direction.
        assert!(low.iter().all(|v| v.normal.z < 0.0));
    }

    #[test]
    fn lowest_vertex_drops_minimum_buried_in_another_shell() {
        // A small box fully inside a large one: the inner box's bottom corners
        // are minima of their own triangles but have material under them.
        let (mut verts, mut faces) = (Vec::new(), Vec::new());
        add_tilted_box(
            &mut verts,
            &mut faces,
            Vector3::new(0.0, 0.0, 10.0),
            Vector3::new(5.0, 5.0, 5.0),
            0.0,
        );
        add_tilted_box(
            &mut verts,
            &mut faces,
            Vector3::new(0.0, 0.0, 10.0),
            Vector3::new(1.0, 1.0, 1.0),
            0.0,
        );
        let mesh = Mesh::new(verts, faces);
        let bvh = Bvh::build(mesh.inner(), Progress::new());

        let low = gather_lowest_vertices(&mesh, &bvh);
        assert!(!low.is_empty());
        assert!(low.iter().all(|v| (v.pos.z - 5.0).abs() < 1e-4));
    }

    #[test]
    fn filter_low_points_emits_whole_runs_and_drops_dense_fans() {
        let p = |x: f32, z: f32| Vector3::new(x, 0.0, z);
        // Bucket at x = 0: a 3-point run within 1 mm of its anchor, then a
        // point 1.5 mm above the anchor that starts a new run of 1.
        let mut candidates = vec![p(0.0, 0.0), p(0.1, 0.5), p(0.2, 1.0), p(0.3, 1.5)];
        // Bucket at x = 5: an 11-point flat fan, dropped entirely.
        candidates.extend((0..11).map(|i| p(5.0 + i as f32 * 0.01, 2.0)));

        let mut kept = filter_low_points(&candidates, Vector2::zeros(), |v| *v);
        kept.sort_by(|a, b| a.z.total_cmp(&b.z));
        assert_eq!(kept, candidates[..4].to_vec());
    }
}
