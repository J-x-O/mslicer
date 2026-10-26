//! The placement grid's footprint: the cell size and where the cells sit over
//! the model, plus ray helpers shared by the column casts.

use nalgebra::Vector3;
use slicer::geometry::{Hit, bounds::Bounds3D, mesh::Mesh};

/// Target number of cells over the model footprint.
const TARGET_CELLS: f32 = 200_000.0;
/// Minimum cell size, in millimetres.
const MIN_CELL_SIZE: f64 = 0.1;
/// Empty-cell padding on each side of the footprint.
const PADDING: usize = 32;

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

/// Start of a column's downward ray above the cell centre `xy`. Cell centres
/// sit on round coordinates, as do the vertices of many models, so a ray
/// through a centre can run exactly along a triangle edge and miss both
/// triangles (the sphere's diagonal columns lost their bottom crossing). A
/// sub-micron sideways nudge, with unequal x and y so it also leaves 45°
/// edges, moves it off such edges.
#[inline]
pub fn column_origin(xy: Vector3<f32>, z: f32) -> Vector3<f32> {
    const NUDGE_X: f32 = 1.13e-4;
    const NUDGE_Y: f32 = 0.71e-4;
    Vector3::new(xy.x + NUDGE_X, xy.y + NUDGE_Y, z)
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

    // The cell size stays in f64 for the origin maths, clamped to exactly 0.1.
    let cell = ((area / TARGET_CELLS).sqrt() as f64).max(MIN_CELL_SIZE);
    let cell_size = cell as f32;
    // Centre the grid on the footprint: `⌊w / cell⌋` cells, the leftover split
    // evenly, one extra cell plus the padding on each side. Where the cell
    // centres fall decides where a flat face's lattice lands, so a grid
    // anchored at the bounding box corner would shift every lattice.
    let axis = |min: f32, extent: f32| {
        let c = cell;
        let k = (extent as f64 / c) as usize;
        let leftover = extent as f64 - k as f64 * c;
        let origin = min as f64 - (c - leftover * 0.5) - PADDING as f64 * c;
        (origin as f32, k + PADDING * 2 + 2)
    };
    let (origin_x, width) = axis(min.x, w);
    let (origin_y, height) = axis(min.y, h);
    let origin = Vector3::new(origin_x, origin_y, 0.0);

    Some(GridFootprint {
        origin,
        cell_size,
        width,
        height,
    })
}

/// Collapses hits that share (nearly) the same distance along the ray, which
/// happens when the ray grazes the diagonal of a quad and hits both of its
/// triangles. Keeps the first of each near-duplicate cluster, preserving the
/// sorted order, so crossings pair cleanly into enter/exit intervals.
pub(super) fn collapse_coplanar(hits: Vec<Hit>) -> Vec<Hit> {
    const EPS: f32 = 1e-3;
    let mut out: Vec<Hit> = Vec::with_capacity(hits.len());
    for hit in hits {
        if out.last().is_none_or(|prev| (hit.t - prev.t).abs() >= EPS) {
            out.push(hit);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;
    use slicer::geometry::mesh::Mesh;

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
    fn footprint_is_centred_on_the_model() {
        // 10.05 mm wide: 100 whole cells and a 0.05 mm leftover, split evenly.
        let mesh = box_mesh(Vector3::new(1.0, 2.0, 0.0), Vector3::new(11.05, 4.0, 1.0));
        let grid = footprint(&mesh).unwrap();
        assert_eq!(grid.cell_size, 0.1);
        assert_eq!(grid.width, 100 + 2 * PADDING + 2);
        let centre_x = grid.origin.x + grid.width as f32 * grid.cell_size / 2.0;
        let centre_y = grid.origin.y + grid.height as f32 * grid.cell_size / 2.0;
        assert!((centre_x - 6.025).abs() < 1e-4, "x centre {centre_x}");
        assert!((centre_y - 3.0).abs() < 1e-4, "y centre {centre_y}");
    }
}
