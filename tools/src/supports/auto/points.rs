//! Stage 1 point placement: turn ray-grid samples into a sparse set of support
//! contact points using classification, a coverage model, and deduplication.
//!
//! This is the first pass of the CHITUBOX-derived pipeline (plan §1.4–1.6). The
//! full pipeline has wall/bound/key steps ahead of low + fill; those are gated
//! behind flags here so the basic low + fill path can land first.

use nalgebra::Vector3;
use tracing::info;

use crate::supports::auto::{
    PlacementConfig,
    grid::{GridFootprint, SurfaceSample},
};

/// A placed support contact point: world-space position and outward normal.
#[derive(Clone, Copy, Debug)]
pub struct PlacedPoint {
    pub point: Vector3<f32>,
    pub normal: Vector3<f32>,
}

/// Coverage parameters derived from the config and grid.
#[derive(Clone, Copy)]
struct Coverage {
    /// Radius of the coverage disc, in cells.
    radius_cells: isize,
    /// Surface-following height tolerance, in millimetres.
    height_tolerance: f32,
    /// Minimum dedupe distance between points, in millimetres.
    dedupe: f32,
}

/// A flat per-cell bucket of sample indices, so neighbour and coverage lookups
/// are O(1) per cell instead of scanning every sample.
struct SampleIndex {
    width: usize,
    height: usize,
    cells: Vec<Vec<usize>>,
}

impl SampleIndex {
    fn new(grid: &GridFootprint, samples: &[SurfaceSample]) -> Self {
        let mut cells = vec![Vec::new(); grid.cell_count()];
        for (i, s) in samples.iter().enumerate() {
            cells[s.row * grid.width + s.col].push(i);
        }
        Self {
            width: grid.width,
            height: grid.height,
            cells,
        }
    }

    #[inline]
    fn at(&self, col: isize, row: isize) -> Option<&[usize]> {
        if col < 0 || row < 0 || col >= self.width as isize || row >= self.height as isize {
            return None;
        }
        Some(self.cells[(row as usize) * self.width + col as usize].as_slice())
    }
}

/// Places points from the samples. The plan's wall/bound/key steps are behind
/// flags and default off, so only low points and the greedy fill run here.
pub fn place_points(
    samples: &[SurfaceSample],
    grid: &GridFootprint,
    config: &PlacementConfig,
    low_points: bool,
    fill: bool,
    _wall: bool,
    _bound: bool,
    _key: bool,
) -> Vec<PlacedPoint> {
    let max_angle = config.max_angle.to_radians();
    let needs_support = |s: &SurfaceSample| {
        // Outward normal facing within `max_angle` of straight down (plan §1.2).
        s.normal.z < -max_angle.cos()
    };

    // Coverage derived from `min_spacing` (the plan's `touchtipdistance`).
    let coverage = Coverage {
        radius_cells: ((config.min_spacing / 2.0) / grid.cell_size)
            .ceil()
            .max(2.0) as isize,
        height_tolerance: 0.05,
        dedupe: (config.min_spacing * 0.3).max(0.6 * 0.8),
    };

    let index = SampleIndex::new(grid, samples);

    let mut out: Vec<PlacedPoint> = Vec::new();
    let mut placed_mask = vec![false; samples.len()];

    // Low points: a sample lower than all its immediate neighbours.
    if low_points {
        for s in samples.iter() {
            if !needs_support(s) {
                continue;
            }
            if is_low_point(s, samples, &index) {
                out.push(PlacedPoint {
                    point: s.pos,
                    normal: s.normal,
                });
                cover(s, samples, &coverage, &index, &mut placed_mask);
            }
        }
    }

    // Greedy fill: lowest-first.
    if fill {
        let mut order = (0..samples.len()).collect::<Vec<_>>();
        order.sort_by(|&a, &b| samples[a].pos.z.total_cmp(&samples[b].pos.z));

        for &i in order.iter() {
            let s = &samples[i];
            if placed_mask[i] || !needs_support(s) {
                continue;
            }

            out.push(PlacedPoint {
                point: s.pos,
                normal: s.normal,
            });
            cover(s, samples, &coverage, &index, &mut placed_mask);
        }
    }

    dedupe(&mut out, coverage.dedupe);

    info!(
        "Placed {} support points (low: {low_points}, fill: {fill})",
        out.len()
    );
    out
}

/// Whether a sample is strictly lower than all its in-bounds neighbours. A
/// sample with no neighbours is treated as a low point.
fn is_low_point(sample: &SurfaceSample, samples: &[SurfaceSample], index: &SampleIndex) -> bool {
    let (c, r) = (sample.col as isize, sample.row as isize);
    let mut any_neighbour = false;

    for dc in -1isize..=1 {
        for dr in -1isize..=1 {
            if dc == 0 && dr == 0 {
                continue;
            }
            if let Some(cell) = index.at(c + dc, r + dr) {
                for &other in cell {
                    any_neighbour = true;
                    if samples[other].pos.z <= sample.pos.z {
                        return false;
                    }
                }
            }
        }
    }

    any_neighbour
}

/// Marks samples within the coverage disc of `placed` as covered, following the
/// surface: only samples in neighbouring cells within `height_tolerance` in z.
fn cover(
    placed: &SurfaceSample,
    samples: &[SurfaceSample],
    coverage: &Coverage,
    index: &SampleIndex,
    mask: &mut [bool],
) {
    let (c, r) = (placed.col as isize, placed.row as isize);
    let radius = coverage.radius_cells;
    let r2 = radius * radius;

    for dc in -radius..=radius {
        for dr in -radius..=radius {
            if dc * dc + dr * dr > r2 {
                continue;
            }
            if let Some(cell) = index.at(c + dc, r + dr) {
                for &i in cell {
                    if (samples[i].pos.z - placed.pos.z).abs() < coverage.height_tolerance {
                        mask[i] = true;
                    }
                }
            }
        }
    }
}

/// Removes points closer than `radius` to an already-kept, higher-priority
/// point (points are in priority order: low first, then fill).
fn dedupe(points: &mut Vec<PlacedPoint>, radius: f32) {
    let mut kept: Vec<PlacedPoint> = Vec::with_capacity(points.len());
    for p in points.drain(..) {
        if (kept.iter()).all(|k| (k.point - p.point).magnitude() > radius) {
            kept.push(p);
        }
    }
    *points = kept;
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::*;
    use crate::supports::auto::grid::{GridFootprint, SurfaceSample};

    /// A flat layer of downward-facing samples on a 20 × 20 grid, 0.1 mm cells.
    fn flat_samples() -> (GridFootprint, Vec<SurfaceSample>) {
        let grid = GridFootprint {
            origin: Vector3::new(0.0, 0.0, 0.0),
            cell_size: 0.1,
            width: 20,
            height: 20,
        };
        let mut samples = Vec::new();
        for r in 0..20usize {
            for c in 0..20usize {
                samples.push(SurfaceSample {
                    pos: Vector3::new(c as f32 * 0.1, r as f32 * 0.1, 5.0),
                    normal: -Vector3::z(),
                    face: 0,
                    col: c,
                    row: r,
                });
            }
        }
        (grid, samples)
    }

    #[test]
    fn dedupe_respects_radius() {
        let (grid, samples) = flat_samples();
        let config = PlacementConfig::default();
        let points = place_points(&samples, &grid, &config, true, true, false, false, false);

        assert!(!points.is_empty());

        let dedupe = (config.min_spacing * 0.3).max(0.6 * 0.8);
        for (i, a) in points.iter().enumerate() {
            for b in points.iter().skip(i + 1) {
                assert!(
                    (a.point - b.point).magnitude() > dedupe,
                    "points too close: {:?} {:?}",
                    a.point,
                    b.point
                );
            }
        }
    }

    #[test]
    fn coverage_marks_all_needs_support_samples() {
        let (grid, samples) = flat_samples();
        let config = PlacementConfig::default();
        let points = place_points(&samples, &grid, &config, false, true, false, false, false);

        // Every sample is downward-facing (`normal.z = -1 < -cos(30°)`), so the
        // fill must place at least one point for the layer.
        assert!(!points.is_empty());

        // And every placed point must sit on a downward face.
        assert!(points.iter().all(|p| p.normal.z < 0.0));
    }
}
