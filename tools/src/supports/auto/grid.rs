//! Stage 1 ray grid: rasterise the model onto a fine vertical-ray grid and
//! collect the downward-facing surface samples that will drive point
//! placement.
//!
//! This replaces the three independent detectors (`place_face_supports`,
//! `place_edge_supports`, `place_point_supports`) with a single coverage
//! source. Every grid column casts a vertical line through the mesh; crossings
//! are paired into inside-the-model intervals, and the bottom of each interval
//! is a sample on a downward-facing surface.

use common::progress::Progress;
use nalgebra::Vector3;
use slicer::geometry::{Hit, bounds::Bounds3D, mesh::Mesh, partitioning::Bvh};

/// Target number of cells over the model footprint (from the plan, §1.1).
const TARGET_CELLS: f32 = 200_000.0;
/// Minimum cell size, in millimetres.
const MIN_CELL_SIZE: f32 = 0.1;
/// Empty-cell padding on each side of the footprint.
const PADDING: usize = 32;

/// A sample on a downward-facing surface: its position in world space, the
/// normal (world space, pointing away from the surface), the face it came
/// from, and its grid cell.
#[derive(Clone, Copy, Debug)]
pub struct SurfaceSample {
    pub pos: Vector3<f32>,
    pub normal: Vector3<f32>,
    pub face: usize,
    pub col: usize,
    pub row: usize,
}

/// The model's grid footprint: origin (world xy of the lowest cell corner),
/// cell size, and grid dimensions in cells.
#[derive(Clone, Copy, Debug)]
pub struct GridFootprint {
    pub origin: Vector3<f32>,
    pub cell_size: f32,
    pub width: usize,
    pub height: usize,
}

impl GridFootprint {
    /// Cell centre in world space for a (column, row) cell index.
    #[inline]
    pub fn cell_centre(&self, col: usize, row: usize) -> Vector3<f32> {
        Vector3::new(
            self.origin.x + (col as f32 + 0.5) * self.cell_size,
            self.origin.y + (row as f32 + 0.5) * self.cell_size,
            0.0,
        )
    }

    #[inline]
    pub fn cell_count(&self) -> usize {
        self.width * self.height
    }
}

/// Sizes the grid from the model's transformed footprint. Returns `None` if the
/// footprint is degenerate (a zero-area model).
pub fn footprint(mesh: &Mesh) -> Option<GridFootprint> {
    let bounds = Bounds3D::from_mesh(mesh);
    let (min, max) = (bounds.min, bounds.max);
    let (w, h) = (max.x - min.x, max.y - min.y);
    let area = w * h;
    if area <= f32::EPSILON {
        return None;
    }

    let cell_size = (area / TARGET_CELLS).sqrt().max(MIN_CELL_SIZE);
    let width = ((w / cell_size).ceil() as usize) + PADDING * 2;
    let height = ((h / cell_size).ceil() as usize) + PADDING * 2;

    // Cell (0,0) sits at the padding offset from the model's min corner.
    let origin = Vector3::new(
        min.x - PADDING as f32 * cell_size,
        min.y - PADDING as f32 * cell_size,
        0.0,
    );

    Some(GridFootprint {
        origin,
        cell_size,
        width,
        height,
    })
}

/// Casts every grid ray and collects the downward-facing surface samples.
///
/// A ray travels from above the mesh straight down. It crosses the model an
/// even number of times (once in, once out, per solid chunk). Crossings sorted
/// by distance pair into (entry, exit); the *exit* surface of each pair — where
/// the ray leaves the solid on its way down — faces downward and is a sample.
pub fn sample_surface(
    mesh: &Mesh,
    bvh: &Bvh,
    grid: &GridFootprint,
    progress: &Progress,
) -> Vec<SurfaceSample> {
    let bounds = Bounds3D::from_mesh(mesh);
    let z_top = bounds.max.z + 1.0;

    progress.set_total(grid.cell_count() as u64);

    let mut samples = Vec::new();
    for row in 0..grid.height {
        for col in 0..grid.width {
            let xy = grid.cell_centre(col, row);
            let origin = Vector3::new(xy.x, xy.y, z_top);

            // Hits arrive sorted by `t` (distance); even indices are entries
            // (top surfaces), odd indices are exits (downward surfaces).
            // Hits arrive sorted by `t` (distance). A ray that grazes the
            // shared diagonal of a quads hits both of its triangles at the same
            // `t`, so collapse near-duplicate crossings before pairing into
            // (entry, exit) intervals. Even indices are entries (top surfaces),
            // odd indices are exits (downward surfaces).
            let raw = bvh.intersect_ray_all(mesh, origin, -Vector3::z());
            for exit in collapse_coplanar(raw).into_iter().skip(1).step_by(2) {
                samples.push(SurfaceSample {
                    pos: mesh.transform(&exit.position),
                    normal: mesh.transform_normal(&mesh.normal(exit.face)).normalize(),
                    face: exit.face,
                    col,
                    row,
                });
            }

            progress.add_complete(1);
        }
    }

    samples
}

/// Collapses hits that share (nearly) the same distance along the ray, which
/// happens when the ray grazes the diagonal an `add_quad` face and hits both of
/// its triangles. Keeps the first of each near-duplicate cluster, preserving
/// the sorted order, so crossings pair cleanly into enter/exit intervals.
fn collapse_coplanar(hits: Vec<Hit>) -> Vec<Hit> {
    const EPS: f32 = 1e-3;
    let mut out: Vec<Hit> = Vec::with_capacity(hits.len());
    for hit in hits {
        if let Some(prev) = out.last() {
            if (hit.t - prev.t).abs() < EPS {
                continue;
            }
        }
        out.push(hit);
    }
    out
}

#[cfg(test)]
mod tests {
    use common::progress::Progress;
    use nalgebra::Vector3;
    use slicer::geometry::{mesh::Mesh, partitioning::Bvh};

    use super::*;

    fn box_mesh(min: Vector3<f32>, max: Vector3<f32>) -> Mesh {
        let mut verts = Vec::new();
        let mut faces = Vec::new();
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
            faces.push([a, b, c]);
            faces.push([a, c, d]);
        }
        Mesh::new(verts, faces)
    }

    #[test]
    fn samples_are_on_downward_faces() {
        let mut mesh = box_mesh(
            Vector3::new(-10.0, -10.0, 0.0),
            Vector3::new(10.0, 10.0, 2.0),
        );
        mesh.set_position(Vector3::new(0.0, 0.0, 5.0));

        let bvh = Bvh::build(mesh.inner(), Progress::new());
        let grid = footprint(&mesh).expect("footprint");
        let progress = Progress::new();
        let samples = sample_surface(&mesh, &bvh, &grid, &progress);

        // The box's bottom face is downward-facing; its samples must have
        // downward-pointing world normals.
        assert!(!samples.is_empty());
        assert!(samples.iter().all(|s| s.normal.z < 0.0));

        // Every sample should sit near z = 5 (the lifted bottom face).
        assert!(samples.iter().all(|s| (s.pos.z - 5.0).abs() < 0.01));
    }
}
