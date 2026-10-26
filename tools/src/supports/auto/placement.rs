//! Contact point placement on a grid of vertical rays.
//!
//! The model is cast onto a grid of downward rays, one per cell. Each
//! column's crossings, sorted by z, pair up into *intervals*. An interval
//! whose lower crossing faces up and whose upper crossing faces down is a
//! *gap*: air under an overhang. A gap carries its upper crossing (the
//! overhang underside) as its point and normal, and every placement step
//! works on gaps and on a table linking each gap to its neighbours in the 8
//! surrounding columns.
//!
//! The steps, in order:
//!
//! 1. Build the intervals, link the gaps, and classify each gap as flat
//!    (needs support) or not from its averaged normal.
//! 2. Mark gaps beside a wall: a gap with solid material in the neighbouring
//!    column above and below it is held by that wall.
//! 3. Mark coplanar patches along the model's outline, so a boundary row of
//!    identical cells gets one point at its centre instead of a row of them.
//! 4. Find key points, the minima along straight lines through the links.
//! 5. Place islands on the lowest mesh vertices.
//! 6. Place supports on the key points, lowest first, then sweep every
//!    remaining flat gap from the bottom up. Each placed support covers a
//!    disc around it, and covered gaps take no further support.
//! 7. Thin out points that ended up too close to a lower one.
//!
//! Afterwards [`edge`] moves points that sit next to a crease onto the more
//! downward-facing face, on the same grid.
//!
//! Several orders matter for the result: the link slot order, the
//! column-major cell order, and the tie order of the sort and the queue
//! ([`order`]). On a flat face they decide where the lattice of points lands.

use std::f32::consts::SQRT_2;

use bitflags::bitflags;
use common::progress::Progress;
use nalgebra::{Vector2, Vector3};
use rayon::prelude::*;
use slicer::geometry::{bounds::Bounds3D, mesh::Mesh, partitioning::Bvh};
use tracing::info;

use crate::supports::auto::{
    PlacementConfig, edge,
    grid::{GridFootprint, collapse_coplanar, column_origin, footprint},
    order::{MinHeap, introsort},
    points::{PlacedPoint, filter_low_points, gather_lowest_vertices},
};

bitflags! {
    /// Per-interval flags.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
    struct Flags: u16 {
        /// The topmost gap of a column: the open sky above the model. Never
        /// linked and never a candidate.
        const SKY = 1 << 0;
        /// The averaged normal is within the support angle of straight down:
        /// a support candidate.
        const FLAT = 1 << 1;
        /// Not flat (exactly one of `FLAT` / `NOT_FLAT` is set on a
        /// classified gap).
        const NOT_FLAT = 1 << 2;
        /// On the outline: some direction has an empty column, a neighbour
        /// that reaches the solid above this gap, or a steep, diverging
        /// neighbour.
        const BOUNDARY = 1 << 3;
        /// Held by a neighbouring wall.
        const WALL = 1 << 4;
        /// Claimed as an island.
        const ISLAND = 1 << 5;
        /// Inside some support's coverage disc.
        const COVERED = 1 << 6;
        /// Inside a key point's coverage disc.
        const KEY_COVERED = 1 << 7;
        /// Part of a coplanar outline patch: no sweep support, no key point.
        const PATCH = 1 << 8;
        /// The patch's centre gap, which stays a candidate.
        const PATCH_CENTRE = 1 << 9;
        /// Claimed as a support.
        const CLAIMED = 1 << 10;
        /// Covered by an existing support (region auto-support). The only
        /// coverage that also keeps islands away.
        const EXISTING = 1 << 11;
    }
}

impl Flags {
    /// Gaps the key point pass skips.
    const KEY_SKIP: Self = Self::CLAIMED
        .union(Self::KEY_COVERED)
        .union(Self::WALL)
        .union(Self::SKY);
    /// Gaps the sweep skips.
    const SWEEP_SKIP: Self = Self::CLAIMED
        .union(Self::COVERED)
        .union(Self::WALL)
        .union(Self::SKY);
    /// Gaps that cannot seed a key point line.
    const KEY_SEED_SKIP: Self = Self::EXISTING.union(Self::PATCH).union(Self::SKY);
}

/// How far past the model the sentinel crossings sit, in mm.
const SENTINEL: f32 = 5.0;
/// Shortest interval kept, in mm.
const MIN_INTERVAL: f32 = 0.01;
/// How far below its seed a height-limited flood may still spread, in mm.
const FLOOD_DROP: f32 = 0.05;

/// The 8 link directions as a ring starting at −x, so slots `d` and `d + 4`
/// are opposite and even slots are axial. The order is behaviour: it is the
/// flood's visiting order, the order neighbours are tried in, and it pairs up
/// the key point lines.
pub(super) const DIRS: [(isize, isize); 8] = [
    (-1, 0),
    (-1, -1),
    (0, -1),
    (1, -1),
    (1, 0),
    (1, 1),
    (0, 1),
    (-1, 1),
];

#[derive(Clone, Debug)]
struct Interval {
    z_min: f32,
    /// The upper crossing's z.
    z_max: f32,
    point: Vector3<f32>,
    normal: Vector3<f32>,
    /// Air under an overhang (lower crossing faces up, upper faces down).
    gap: bool,
    flags: Flags,
    /// Number of links.
    degree: u8,
    /// A two-colour visit mark the key point lines flip between passes, so
    /// each line is walked once per pass.
    mark: u8,
    links: [Option<u32>; 8],
    cell: u32,
}

/// One crossing of a column ray: world position and outward normal.
#[derive(Clone, Copy)]
struct Crossing {
    pos: Vector3<f32>,
    normal: Vector3<f32>,
}

/// The cells within a radius of a centre cell: offsets with
/// `dx² + dy² < (r + 0.5)²`, `r` truncated to whole cells first.
struct Disc {
    /// Half the side of the bounding square, in cells.
    reach: isize,
    /// Membership over the bounding square, row by row.
    inside: Vec<bool>,
    len: usize,
}

impl Disc {
    fn new(r: f32) -> Self {
        let r = r.max(0.0).trunc();
        let reach = r as isize + 1;
        let limit = (r + 0.5) * (r + 0.5);
        let inside: Vec<bool> = (-reach..=reach)
            .flat_map(|dy| (-reach..=reach).map(move |dx| ((dx * dx + dy * dy) as f32) < limit))
            .collect();
        let len = inside.iter().filter(|&&i| i).count();
        Self { reach, inside, len }
    }

    /// The index of offset `(dx, dy)` in the bounding square, if it lies in
    /// the disc.
    fn index(&self, dx: isize, dy: isize) -> Option<usize> {
        let side = 2 * self.reach + 1;
        let i = (dx.abs() <= self.reach && dy.abs() <= self.reach)
            .then(|| ((dy + self.reach) * side + dx + self.reach) as usize)?;
        self.inside[i].then_some(i)
    }
}

/// The gaps a flood reached, in visiting order, and which of them survived
/// the ridge prune.
struct Walk {
    order: Vec<usize>,
    kept: Vec<bool>,
}

impl Walk {
    fn kept(&self) -> impl Iterator<Item = usize> + '_ {
        (self.order.iter().zip(&self.kept)).filter_map(|(&i, &keep)| keep.then_some(i))
    }
}

/// The two placement passes, which differ in coverage and re-centring.
#[derive(Clone, Copy)]
enum Pass {
    /// Key points: always re-centred, cover the larger disc.
    Keys,
    /// The sweep: only straight-down gaps are re-centred.
    Sweep,
}

struct Placer<'a> {
    grid: GridFootprint,
    intervals: Vec<Interval>,
    /// Per cell: the range of its intervals in `intervals`, ascending z.
    cells: Vec<(u32, u32)>,
    mesh: &'a Mesh,
    bvh: &'a Bvh,
    /// Point spacing in cells, `n`.
    spacing_cells: isize,
    /// 0.75 × point spacing, in mm: the height unit of the small-disc test.
    wall_band: f32,
    tan_angle: f32,
    /// Pillar and contact diameters (mm); they size the outline patch disc.
    pillar_diameter: f32,
    tip_diameter: f32,
    /// Coverage discs: `0.8 n` around key points, `0.75 n` elsewhere, and
    /// `0.375 n` next to a wall.
    disc_key: Disc,
    disc_cover: Disc,
    disc_small: Disc,
    /// Islands with no matching gap, supported straight down.
    loose_islands: Vec<PlacedPoint>,
    /// Height of the top sentinel crossing.
    top: f32,
}

/// Removes points crowding a lower one. In ascending z, each surviving point
/// removes every later, not lower point closer than `0.3 × spacing` (an
/// island target: `min(0.3 × spacing, 0.6 × pillar Ø)`). Both the per-axis box
/// and the distance must pass.
fn remove_crowded(
    points: Vec<PlacedPoint>,
    spacing: f32,
    pillar_diameter: f32,
) -> Vec<PlacedPoint> {
    let radius = 0.3 * spacing;
    let island_radius = radius.min(0.6 * pillar_diameter);
    let mut points = points;
    points.sort_by(|a, b| a.point.z.total_cmp(&b.point.z));
    let mut removed = vec![false; points.len()];
    for i in 0..points.len() {
        if removed[i] {
            continue;
        }
        let p = points[i].point;
        for j in i + 1..points.len() {
            let q = points[j].point;
            if removed[j] || q.z < p.z {
                continue;
            }
            let r = if points[j].island {
                island_radius
            } else {
                radius
            };
            let d = q - p;
            if d.x.abs() <= r && d.y.abs() <= r && d.z.abs() <= r && d.magnitude_squared() < r * r {
                removed[j] = true;
            }
        }
    }
    (points.into_iter().zip(removed))
        .filter_map(|(p, removed)| (!removed).then_some(p))
        .collect()
}

/// Places support points over the model (see the module docs), then re-seats
/// those next to a crease ([`edge`]). `seed` are existing contacts whose
/// surroundings count as covered (region auto-support).
pub fn place_points(
    mesh: &Mesh,
    bvh: &Bvh,
    config: &PlacementConfig,
    tip_diameter: f32,
    seed: &[Vector3<f32>],
    edge: &edge::EdgeConfig,
    progress: &Progress,
) -> Vec<PlacedPoint> {
    let Some(grid) = footprint(mesh) else {
        return Vec::new();
    };
    let pillar_diameter = edge.pillar_diameter;
    let mut placer = Placer::new(
        mesh,
        bvh,
        &grid,
        config,
        pillar_diameter,
        tip_diameter,
        progress,
    );
    placer.link();
    placer.classify_slopes(config.max_angle);
    placer.mark_wall_gaps();
    placer.mark_outline_patches();
    for &pos in seed {
        placer.cover_seed(pos);
    }
    let keys = placer.key_points();
    let islands = placer.place_islands();
    let on_keys = placer.place_on_keys(&keys);
    let swept = placer.sweep();

    // The loose islands come first, then every claimed gap in column-major
    // cell order; the thinning pass sees them in that order.
    let mut points = std::mem::take(&mut placer.loose_islands);
    for col in 0..grid.width {
        for row in 0..grid.height {
            for i in placer.column(row * grid.width + col) {
                let iv = &placer.intervals[i];
                if iv.flags.intersects(Flags::CLAIMED | Flags::ISLAND) {
                    points.push(PlacedPoint {
                        point: iv.point,
                        normal: iv.normal,
                        island: iv.flags.contains(Flags::ISLAND),
                    });
                }
            }
        }
    }
    let mut points = remove_crowded(points, config.min_spacing, pillar_diameter);
    let moved = edge::adjust_points(&mut points, &placer, mesh, bvh, edge);
    info!("Edge re-seat moved {moved} of {} points", points.len());
    info!(
        "Placement: {} key points, {islands} islands, {on_keys} on keys, {swept} swept, {} points",
        keys.len(),
        points.len()
    );
    points
}

impl<'a> Placer<'a> {
    /// Casts one ray per cell centre and pairs the sorted crossings into
    /// intervals, with a sentinel crossing below and above the model: an
    /// up-facing one under a down-facing lowest crossing, so the space under
    /// the model is a gap, and a down-facing one on top, so the sky is a gap
    /// too (flagged, never a candidate).
    fn new(
        mesh: &'a Mesh,
        bvh: &'a Bvh,
        grid: &GridFootprint,
        config: &PlacementConfig,
        pillar_diameter: f32,
        tip_diameter: f32,
        progress: &Progress,
    ) -> Self {
        let bounds = Bounds3D::from_mesh(mesh);
        let (bottom, top) = (bounds.min.z - SENTINEL, bounds.max.z + SENTINEL);

        progress.set_total(grid.cell_count() as u64);
        let columns: Vec<Vec<Interval>> = (0..grid.cell_count())
            .into_par_iter()
            .map(|cell| {
                let (col, row) = (cell % grid.width, cell / grid.width);
                let xy = grid.cell_centre(col, row);
                let origin = column_origin(xy, top);
                let hits = collapse_coplanar(bvh.intersect_ray_all(mesh, origin, -Vector3::z()));

                // An empty column has no intervals at all.
                if hits.is_empty() {
                    progress.add_complete(1);
                    return Vec::new();
                }
                let mut crossings: Vec<Crossing> = (hits.iter().rev())
                    .map(|hit| Crossing {
                        pos: mesh.transform(&hit.position),
                        normal: mesh.transform_normal(&mesh.normal(hit.face)).normalize(),
                    })
                    .collect();
                if crossings[0].normal.z < 0.0 {
                    crossings.insert(
                        0,
                        Crossing {
                            pos: Vector3::new(xy.x, xy.y, bottom),
                            normal: Vector3::z(),
                        },
                    );
                }
                crossings.push(Crossing {
                    pos: Vector3::new(xy.x, xy.y, top),
                    normal: -Vector3::z(),
                });

                let mut column: Vec<Interval> = (crossings.windows(2))
                    .filter(|pair| pair[1].pos.z - pair[0].pos.z > MIN_INTERVAL)
                    .map(|pair| Interval {
                        z_min: pair[0].pos.z,
                        z_max: pair[1].pos.z,
                        point: pair[1].pos,
                        normal: pair[1].normal,
                        gap: pair[0].normal.z > 0.0 && pair[1].normal.z < 0.0,
                        flags: Flags::empty(),
                        degree: 0,
                        mark: 0,
                        links: [None; 8],
                        cell: cell as u32,
                    })
                    .collect();
                if let Some(last) = column.last_mut().filter(|iv| iv.gap) {
                    last.flags = Flags::SKY;
                }
                progress.add_complete(1);
                column
            })
            .collect();

        let mut intervals = Vec::new();
        let mut cells = Vec::with_capacity(columns.len());
        for column in columns {
            cells.push((intervals.len() as u32, column.len() as u32));
            intervals.extend(column);
        }

        let spacing_cells = ((config.min_spacing / grid.cell_size).round()).max(4.0) as isize;
        let n = spacing_cells as f32;
        let tan_angle = if config.max_angle >= 89.9 {
            572.96594
        } else {
            (config.max_angle + 0.1).to_radians().tan()
        };
        Self {
            grid: *grid,
            intervals,
            cells,
            mesh,
            bvh,
            spacing_cells,
            wall_band: 0.75 * config.min_spacing,
            tan_angle,
            pillar_diameter,
            tip_diameter,
            disc_key: Disc::new(0.8 * n),
            disc_cover: Disc::new(0.75 * n),
            disc_small: Disc::new(0.375 * n),
            loose_islands: Vec::new(),
            top,
        }
    }

    /// The column and row holding `point`'s xy, possibly off the grid.
    fn col_row_at(&self, point: Vector3<f32>) -> (isize, isize) {
        let col = ((point.x - self.grid.origin.x) / self.grid.cell_size).floor() as isize;
        let row = ((point.y - self.grid.origin.y) / self.grid.cell_size).floor() as isize;
        (col, row)
    }

    /// The cell holding `point`'s xy.
    fn cell_of(&self, point: Vector3<f32>) -> Option<usize> {
        let (col, row) = self.col_row_at(point);
        self.cell_at(col, row)
    }

    fn cell_at(&self, col: isize, row: isize) -> Option<usize> {
        let (w, h) = (self.grid.width as isize, self.grid.height as isize);
        (col >= 0 && row >= 0 && col < w && row < h).then(|| (row * w + col) as usize)
    }

    fn col_row(&self, cell: u32) -> (isize, isize) {
        let w = self.grid.width as u32;
        ((cell % w) as isize, (cell / w) as isize)
    }

    fn column(&self, cell: usize) -> std::ops::Range<usize> {
        let (start, len) = self.cells[cell];
        start as usize..(start + len) as usize
    }

    /// The interval of `cell` whose `[z_min, z_max]` contains `z`.
    fn containing(&self, cell: usize, z: f32) -> Option<usize> {
        self.find_overlap(cell, z, z)
    }

    /// Bisects the z-sorted column for an interval overlapping the closed
    /// range `[lo, hi]`. Where two intervals share a boundary, the one
    /// returned depends on where the bisection lands; callers rely on exactly
    /// that one, so this is not replaced by a linear scan.
    fn find_overlap(&self, cell: usize, lo: f32, hi: f32) -> Option<usize> {
        let column = self.column(cell);
        let (base, mut low, mut high) = (column.start, 0isize, column.len() as isize - 1);
        while low <= high {
            let mid = (low + high) >> 1;
            let iv = &self.intervals[base + mid as usize];
            if hi < iv.z_min {
                high = mid - 1;
            } else if lo <= iv.z_max {
                return Some(base + mid as usize);
            } else {
                low = mid + 1;
            }
        }
        None
    }

    /// The gap in `cell` that `seed` links to. Starting from the bisection hit
    /// for the seed's z range:
    /// * walk up, skipping solid intervals, until one starts above the seed
    ///   or overlaps it by no more than a cell; the last gap passed wins, so
    ///   the link prefers the top of an overlapping slab;
    /// * if that found nothing, walk down and take the first gap overlapping
    ///   by more than a cell.
    fn select_neighbour(&self, seed: usize, cell: usize) -> Option<usize> {
        let seed_zmin = self.intervals[seed].z_min;
        let seed_zmax = self.intervals[seed].z_max;
        let cell_mm = self.grid.cell_size;

        let start = self.find_overlap(cell, seed_zmin, seed_zmax)? - self.column(cell).start;

        let mut sel = None;
        for j in self.column(cell).skip(start) {
            let n = &self.intervals[j];
            if n.z_min > seed_zmax {
                break; // starts above the seed: nothing higher overlaps
            }
            if !n.gap {
                continue;
            }
            let depth = n.z_max.min(seed_zmax) - n.z_min.max(seed_zmin);
            if depth <= cell_mm {
                break;
            }
            sel = Some(j);
        }
        if let Some(j) = sel {
            return Some(j);
        }

        for j in self.column(cell).take(start).rev() {
            let n = &self.intervals[j];
            if n.z_max < seed_zmin {
                break; // ends below the seed: nothing lower overlaps
            }
            if !n.gap {
                continue;
            }
            let depth = n.z_max.min(seed_zmax) - n.z_min.max(seed_zmin);
            if depth <= cell_mm {
                continue; // unlike the upward walk, keep going
            }
            return Some(j);
        }
        None
    }

    /// Links every gap to its neighbours ([`Self::select_neighbour`]). A gap
    /// is on the outline (`BOUNDARY`, no link) when the neighbour column is
    /// empty or the neighbour reaches the solid above this gap. A neighbour
    /// that rises more than 3 steps per step, faces up more and diverges by
    /// over 60° is linked and counted but also marks the outline.
    fn link(&mut self) {
        let cell_size = self.grid.cell_size;
        let found: Vec<(usize, Flags, [Option<u32>; 8], u8)> = (0..self.intervals.len())
            .into_par_iter()
            .filter(|&i| self.intervals[i].gap && !self.intervals[i].flags.contains(Flags::SKY))
            .map(|i| {
                let iv = &self.intervals[i];
                let (col, row) = self.col_row(iv.cell);
                let z_limit = self.intervals[i + 1].z_max + cell_size;
                let mut links = [None; 8];
                let (mut flags, mut degree) = (Flags::empty(), 0u8);
                for (d, &(dx, dy)) in DIRS.iter().enumerate() {
                    // Slots 4..8 step √2 by index, so +x and +y count as
                    // diagonal here.
                    let step = if d < 4 { cell_size } else { cell_size * SQRT_2 };
                    let Some(cell) = self
                        .cell_at(col + dx, row + dy)
                        .filter(|&cell| !self.column(cell).is_empty())
                    else {
                        flags |= Flags::BOUNDARY;
                        continue;
                    };
                    let Some(j) = self.select_neighbour(i, cell) else {
                        continue;
                    };
                    let n = &self.intervals[j];
                    if n.flags.contains(Flags::SKY) || n.z_max >= z_limit {
                        flags |= Flags::BOUNDARY;
                        continue;
                    }
                    links[d] = Some(j as u32);
                    degree += 1;
                    if (n.z_max - iv.z_max) / step > 3.0
                        && iv.normal.z < n.normal.z
                        && iv.normal.dot(&n.normal) < 0.5
                    {
                        flags |= Flags::BOUNDARY;
                    }
                }
                (i, flags, links, degree)
            })
            .collect();
        for (i, flags, links, degree) in found {
            let iv = &mut self.intervals[i];
            iv.flags |= flags;
            iv.links = links;
            iv.degree = degree;
        }
    }

    /// Averages each gap's normal with its linked neighbours'. Flat when
    /// `z ≤ −cos(α + 0.1°)` (with α at least 3°), else not flat.
    fn classify_slopes(&mut self, max_angle: f32) {
        let angle = (max_angle + 0.1).to_radians();
        let flat_z = (-angle.cos()).max(-(3f32.to_radians().cos()));
        // A gap topped by the sentinel is open sky, not a surface.
        let top = self.top;
        let updates: Vec<(usize, Flags)> = (0..self.intervals.len())
            .filter(|&i| self.intervals[i].gap && self.intervals[i].z_max < top)
            .map(|i| {
                let iv = &self.intervals[i];
                let mut sum = iv.normal;
                let mut count = 1;
                for j in iv.links.iter().flatten() {
                    sum += self.intervals[*j as usize].normal;
                    count += 1;
                }
                let z = if count > 1 { sum.normalize().z } else { sum.z };
                let flag = if z <= flat_z {
                    Flags::FLAT
                } else {
                    Flags::NOT_FLAT
                };
                (i, flag)
            })
            .collect();
        for (i, flag) in updates {
            self.intervals[i].flags |= flag;
        }
    }

    /// For every solid interval and direction, lists the neighbour column's
    /// solid intervals that overlap it in z. Two consecutive entries two
    /// apart enclose a gap beside solid material here: that gap is held by
    /// the wall. Wall gaps that are not flat, or link to at least two other
    /// wall gaps, spread the mark over a `0.4 n` disc; every wall gap gets it
    /// itself.
    fn mark_wall_gaps(&mut self) {
        let mut walls = Vec::new();
        for col in 0..self.grid.width {
            for row in 0..self.grid.height {
                let cell = row * self.grid.width + col;
                for seed in self.column(cell) {
                    if self.intervals[seed].gap {
                        continue;
                    }
                    let (z_min, z_max) = (self.intervals[seed].z_min, self.intervals[seed].z_max);
                    for &(dx, dy) in &DIRS {
                        let Some(ncell) = self.cell_at(col as isize + dx, row as isize + dy) else {
                            continue;
                        };
                        let list: Vec<usize> = self
                            .column(ncell)
                            .filter(|&j| {
                                let n = &self.intervals[j];
                                !n.gap && n.z_max.min(z_max) - n.z_min.max(z_min) > 0.0
                            })
                            .collect();
                        for pair in list.windows(2) {
                            let mid = pair[0] + 1;
                            if pair[1] == pair[0] + 2
                                && self.intervals[mid].gap
                                && self.intervals[mid].mark == 0
                            {
                                self.intervals[mid].mark = 1;
                                walls.push(mid);
                            }
                        }
                    }
                }
            }
        }

        let spreads: Vec<bool> = (walls.iter())
            .map(|&m| {
                let iv = &self.intervals[m];
                let linked = (iv.links.iter().flatten())
                    .filter(|&&j| self.intervals[j as usize].mark != 0)
                    .count();
                iv.flags.contains(Flags::NOT_FLAT) || linked >= 2
            })
            .collect();
        for &m in &walls {
            self.intervals[m].flags |= Flags::WALL;
            self.intervals[m].mark = 0;
        }
        let disc = Disc::new(0.4 * self.spacing_cells as f32);
        for (&m, _) in walls.iter().zip(&spreads).filter(|(_, spread)| **spread) {
            let walk = self.walk(m, &disc, true);
            self.mark(&walk, Flags::WALL);
        }
    }

    /// Outline patches. Each outline gap floods (with no height limit) a disc
    /// of `2·min(pillar Ø, tip Ø)` (at most `n` cells) and keeps the reached
    /// gaps whose normal matches its own within 0.01 per axis: the coplanar
    /// patch around it. A patch counts when it fills at least 40 % of the disc
    /// and either faces straight down (`n.z < −0.99`) or reaches a cell height
    /// below the seed. Then the patch is marked (no sweep point, no key point)
    /// except the gap at its centroid cell, which stays a candidate.
    fn mark_outline_patches(&mut self) {
        const MIN_FILL: f32 = 0.4;
        const SAME_NORMAL: f32 = 0.01;
        let cell_spacing = self.grid.cell_size;
        let r_cells = (2.0 * self.pillar_diameter.min(self.tip_diameter) / cell_spacing) as isize;
        let disc = Disc::new(r_cells.min(self.spacing_cells) as f32);

        let seeds: Vec<usize> = (0..self.intervals.len())
            .filter(|&i| {
                let iv = &self.intervals[i];
                iv.gap && iv.flags.contains(Flags::BOUNDARY) && !iv.flags.contains(Flags::SKY)
            })
            .collect();

        for seed in seeds {
            let normal = self.intervals[seed].normal;
            let seed_top = self.intervals[seed].z_max;
            let reached = self.walk(seed, &disc, false).order;

            let same: Vec<usize> = (reached.iter().copied())
                .filter(|&i| {
                    let d = self.intervals[i].normal - normal;
                    d.x.abs() <= SAME_NORMAL && d.y.abs() <= SAME_NORMAL && d.z.abs() <= SAME_NORMAL
                })
                .collect();
            if same.is_empty() || (same.len() as f32) < MIN_FILL * disc.len as f32 {
                continue;
            }
            let lowest = (same.iter())
                .map(|&i| self.intervals[i].z_max)
                .fold(f32::INFINITY, f32::min);
            if normal.z >= -0.99 && seed_top - lowest < cell_spacing {
                continue;
            }

            let (mut sum_c, mut sum_r) = (0.0f32, 0.0f32);
            for &i in &same {
                let (c, r) = self.col_row(self.intervals[i].cell);
                sum_c += c as f32;
                sum_r += r as f32;
            }
            let n = same.len() as f32;
            let centre = self
                .cell_at((sum_c / n + 0.5) as isize, (sum_r / n + 0.5) as isize)
                .and_then(|cell| self.column(cell).find(|j| reached.contains(j)));
            let Some(centre) = centre else {
                continue;
            };
            self.intervals[centre].flags |= Flags::PATCH_CENTRE;
            for &i in &same {
                self.intervals[i].flags |= Flags::PATCH;
            }
        }
    }

    /// Covers the surroundings of an existing contact (region auto-support),
    /// so neither supports nor islands land next to it.
    fn cover_seed(&mut self, pos: Vector3<f32>) {
        let Some(i) = self.interval_at_point(pos, 0.5) else {
            return;
        };
        let walk = self.walk(i, &self.disc_key, true);
        self.mark(&walk, Flags::KEY_COVERED | Flags::COVERED | Flags::EXISTING);
    }

    /// The gap of `pos`'s column whose point lies within `tolerance` of `pos`.
    fn interval_at_point(&self, pos: Vector3<f32>, tolerance: f32) -> Option<usize> {
        let cell = self.cell_of(pos)?;
        self.column(cell)
            .filter(|&i| self.intervals[i].gap)
            .map(|i| (i, (self.intervals[i].point - pos).magnitude()))
            .filter(|&(_, d)| d <= tolerance)
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(i, _)| i)
    }

    /// Floods `disc` around `seed`. The disc only bounds the flood: the walk
    /// starts at the seed and follows links into cells inside the disc,
    /// entering each cell once. With `height_limited`, a link is followed only
    /// to a neighbour above `seed z − FLOOD_DROP`, so the flood spreads level
    /// or upward, never down, and afterwards drops cells hidden behind a
    /// ridge: from the first quarter of the visit order on, a cell is dropped
    /// when a still-kept cell at 1/6 … 5/6 of the way from the seed is more
    /// than 0.1 mm above both the seed and the cell.
    fn walk(&self, seed: usize, disc: &Disc, height_limited: bool) -> Walk {
        const RIDGE_SAMPLES: [f32; 5] = [1.0 / 6.0, 1.0 / 3.0, 0.5, 2.0 / 3.0, 5.0 / 6.0];
        const RIDGE_RISE: f32 = 0.1;
        const UNSEEN: usize = usize::MAX;
        let seed_z = self.intervals[seed].z_max;
        let (col, row) = self.col_row(self.intervals[seed].cell);

        // Per disc cell: its position in `order`, once entered.
        let mut slot = vec![UNSEEN; disc.inside.len()];
        let offset_of = |i: usize| {
            let (c, r) = self.col_row(self.intervals[i].cell);
            (c - col, r - row)
        };
        slot[disc.index(0, 0).expect("the centre is in every disc")] = 0;
        let mut order = vec![seed];
        let mut next = 0;
        while next < order.len() {
            let i = order[next];
            next += 1;
            for j in self.intervals[i].links.into_iter().flatten() {
                let j = j as usize;
                let (dx, dy) = offset_of(j);
                let Some(s) = disc.index(dx, dy).filter(|&s| slot[s] == UNSEEN) else {
                    continue;
                };
                if height_limited && seed_z >= FLOOD_DROP + self.intervals[j].z_max {
                    continue;
                }
                slot[s] = order.len();
                order.push(j);
            }
        }

        let mut kept = vec![true; order.len()];
        if height_limited {
            for k in order.len() / 4..order.len() {
                let floor = self.intervals[order[k]].z_max.max(seed_z);
                let (dc, dr) = offset_of(order[k]);
                let (dc, dr) = (dc as f32, dr as f32);
                let hidden = RIDGE_SAMPLES.iter().any(|&f| {
                    // In absolute cells, so the rounding matches the grid.
                    let sc = (col as f32 + 0.5 + dc * f).floor() as isize;
                    let sr = (row as f32 + 0.5 + dr * f).floor() as isize;
                    (disc.index(sc - col, sr - row))
                        .map(|s| slot[s])
                        .filter(|&m| m != UNSEEN)
                        .is_some_and(|m| {
                            kept[m] && self.intervals[order[m]].z_max > floor + RIDGE_RISE
                        })
                });
                if hidden {
                    kept[k] = false;
                }
            }
        }
        Walk { order, kept }
    }

    /// Sets `mask` on the gaps a flood kept.
    fn mark(&mut self, walk: &Walk, mask: Flags) {
        for i in walk.kept() {
            self.intervals[i].flags |= mask;
        }
    }

    /// Key points: four passes, each along lines through opposite link slots
    /// (0/4, 2/6, 1/5, 3/7). Every unvisited flat gap, in cell order, walks
    /// its line both ways, marking each gap visited and tracking the lowest
    /// (first wins ties). That minimum is a key unless its plateau (line
    /// cells within 0.05 mm of it) spans 11 cells or more. Keys are returned
    /// in discovery order; a gap that is the minimum of several lines appears
    /// once per line.
    fn key_points(&mut self) -> Vec<usize> {
        const LINES: [(usize, usize); 4] = [(0, 4), (2, 6), (1, 5), (3, 7)];
        const PLATEAU: f32 = 0.05;
        const MAX_PLATEAU: usize = 11;
        let mut minima = Vec::new();
        for (pass, &(d1, d2)) in LINES.iter().enumerate() {
            let unseen = (pass & 1) as u8;
            let seen = unseen ^ 1;
            for col in 0..self.grid.width {
                for row in 0..self.grid.height {
                    let cell = row * self.grid.width + col;
                    for seed in self.column(cell) {
                        let iv = &self.intervals[seed];
                        if !iv.gap
                            || iv.flags.intersects(Flags::KEY_SEED_SKIP)
                            || iv.mark != unseen
                            || !iv.flags.contains(Flags::FLAT)
                        {
                            continue;
                        }
                        self.intervals[seed].mark = seen;
                        let mut min = seed;
                        for d in [d1, d2] {
                            let mut cur = seed;
                            while let Some(next) = self.intervals[cur].links[d] {
                                cur = next as usize;
                                self.intervals[cur].mark = seen;
                                if self.intervals[cur].z_max < self.intervals[min].z_max {
                                    min = cur;
                                }
                            }
                        }
                        let floor = self.intervals[min].z_max;
                        let mut plateau = 1;
                        for d in [d1, d2] {
                            let mut cur = min;
                            while let Some(next) = self.intervals[cur].links[d] {
                                cur = next as usize;
                                if self.intervals[cur].z_max - floor >= PLATEAU {
                                    break;
                                }
                                plateau += 1;
                            }
                        }
                        if plateau < MAX_PLATEAU {
                            minima.push(min);
                        }
                    }
                }
            }
        }

        (minima.into_iter())
            .filter(|&i| {
                let iv = &self.intervals[i];
                iv.gap
                    && !iv.flags.intersects(Flags::KEY_SEED_SKIP)
                    && iv.flags.contains(Flags::FLAT)
            })
            .collect()
    }

    /// Islands: the lowest mesh vertices, bucket-filtered, each kept when it
    /// is a local minimum on the grid ([`Self::is_grid_minimum`]) and placed
    /// by [`Self::claim_island`]. Returns how many were placed.
    fn place_islands(&mut self) -> usize {
        let candidates = gather_lowest_vertices(self.mesh, self.bvh);
        let origin: Vector2<f32> = Bounds3D::from_mesh(self.mesh).min.xy();
        let mut islands = filter_low_points(&candidates, origin, |v| v.pos);
        islands.sort_by(|a, b| {
            (a.pos.z.total_cmp(&b.pos.z))
                .then(a.pos.x.total_cmp(&b.pos.x))
                .then(a.pos.y.total_cmp(&b.pos.y))
        });
        let mut placed = 0;
        for v in islands {
            if self.is_grid_minimum(v.pos) && self.claim_island(v.pos) {
                placed += 1;
            }
        }
        placed
    }

    /// Whether `pos` is a local minimum on the grid. Start at the gap at
    /// `pos.z − 0.01` in `pos`'s column when its point is `pos` (per axis
    /// within 0.1 mm); solid there rejects. Otherwise start from the first
    /// non-empty neighbour column (none at all: an isolated tip, accept),
    /// which must be a gap at that height. Then follow the start's links 3
    /// steps in every direction: no step may lie more than 0.01 mm below
    /// `pos` or below the previous step, and some step must rise more than
    /// 0.01 mm above `pos`. A chain that ends early is fine unless the next
    /// column is solid at that height.
    fn is_grid_minimum(&self, pos: Vector3<f32>) -> bool {
        const TOL: f32 = 0.01;
        let (col, row) = self.col_row_at(pos);
        let Some(cell) = self
            .cell_at(col, row)
            .filter(|&c| !self.column(c).is_empty())
        else {
            return false;
        };
        let z = pos.z - TOL;
        let same_point = |q: Vector3<f32>| (q - pos).iter().all(|d| d.abs() <= 0.1);
        let start = match self.containing(cell, z) {
            Some(i) if !self.intervals[i].gap => return false,
            Some(i) if same_point(self.intervals[i].point) => (i, col, row),
            _ => {
                let Some((c, r, next)) = DIRS.iter().find_map(|&(dx, dy)| {
                    let next = self.cell_at(col + dx, row + dy)?;
                    (!self.column(next).is_empty()).then_some((col + dx, row + dy, next))
                }) else {
                    return true;
                };
                match self.containing(next, z) {
                    Some(j) if self.intervals[j].gap => (j, c, r),
                    _ => return false,
                }
            }
        };
        let solid_at = |c: isize, r: isize| {
            self.cell_at(c, r)
                .and_then(|cell| self.containing(cell, z))
                .is_some_and(|k| !self.intervals[k].gap)
        };
        for (d, &(dx, dy)) in DIRS.iter().enumerate() {
            let (mut cur, mut c, mut r) = start;
            let mut previous = pos.z;
            let mut higher = false;
            let mut ended = false;
            for _ in 0..3 {
                (c, r) = (c + dx, r + dy);
                let Some(next) = self.intervals[cur].links[d] else {
                    if solid_at(c, r) {
                        return false;
                    }
                    ended = true;
                    break;
                };
                cur = next as usize;
                let nz = self.intervals[cur].z_max;
                if nz < pos.z - TOL || nz < previous - TOL {
                    return false;
                }
                higher |= nz > pos.z + TOL;
                previous = nz;
            }
            if !ended && !higher {
                return false;
            }
        }
        true
    }

    /// Claims the island at `p`: the non-sky gap at `p.z − 0.01` in `p`'s
    /// column (else its 8 neighbours, in slot order) whose point matches `p`
    /// per axis within 0.1 mm. Unless an existing support covers it, it
    /// floods its coverage disc (the small one next to a wall) and is
    /// claimed. With no match anywhere, `p` becomes a loose island supported
    /// straight down. Returns whether a point resulted.
    fn claim_island(&mut self, p: Vector3<f32>) -> bool {
        let z = p.z - 0.01;
        let (col, row) = self.col_row_at(p);
        let matched = |c: isize, r: isize| {
            let cell = self.cell_at(c, r)?;
            let i = self.containing(cell, z)?;
            let iv = &self.intervals[i];
            let same = (iv.point - p).iter().all(|d| d.abs() <= 0.1);
            (iv.gap && !iv.flags.contains(Flags::SKY) && same).then_some(i)
        };
        let hit = matched(col, row).or_else(|| {
            DIRS.iter()
                .find_map(|&(dx, dy)| matched(col + dx, row + dy))
        });
        match hit {
            Some(i) if self.intervals[i].flags.contains(Flags::EXISTING) => false,
            Some(i) => {
                let disc = if self.beside_wall(i) {
                    &self.disc_small
                } else {
                    &self.disc_cover
                };
                let walk = self.walk(i, disc, true);
                self.mark(&walk, Flags::COVERED);
                self.intervals[i].flags |= Flags::CLAIMED | Flags::ISLAND;
                true
            }
            None => {
                self.loose_islands.push(PlacedPoint {
                    point: p,
                    normal: -Vector3::z(),
                    island: true,
                });
                true
            }
        }
    }

    /// Whether a support at `seed` sits low beside a wall and so covers only
    /// the small disc. Over the full disc's flood (no height limit), with
    /// `R = 0.75 × spacing`: some gap more than `0.9 R` above the seed, and
    /// none more than `0.4 R` below it.
    fn beside_wall(&self, seed: usize) -> bool {
        let r = self.wall_band;
        let seed_z = self.intervals[seed].z_max;
        let reached = self.walk(seed, &self.disc_cover, false).order;
        let mut above = false;
        for &i in &reached {
            let dz = self.intervals[i].z_max - seed_z;
            if dz > 0.9 * r {
                above = true;
            }
            if dz < -0.4 * r {
                return false;
            }
        }
        above
    }

    /// Whether the space under a gap's point is unusable: a segment from
    /// 0.01 mm below the point down past the model. Among the triangles it
    /// crosses that are not vertical (`|n.z| ≥ 0.01`), the highest hit decides:
    /// facing down (the point sits inside other material) or closer than 2 mm
    /// (no room for a support) rejects. Nothing below is fine.
    fn no_room_below(&self, i: usize) -> bool {
        const MIN_GAP: f32 = 2.0;
        let p = self.intervals[i].point;
        let origin = Vector3::new(p.x, p.y, p.z - 0.01);
        let below = (self
            .bvh
            .intersect_ray_all(self.mesh, origin, -Vector3::z())
            .into_iter())
        .map(|hit| {
            let n = self.mesh.transform_normal(&self.mesh.normal(hit.face));
            (self.mesh.transform(&hit.position).z, n.z)
        })
        .filter(|&(_, nz)| nz.abs() >= 0.01)
        .max_by(|a, b| a.0.total_cmp(&b.0));
        below.is_some_and(|(z, nz)| nz <= 0.0 || p.z - z < MIN_GAP)
    }

    /// Whether a steep face just beside and below the gap holds the spot. An
    /// 8-direction fan of up to `2n/3` cells (2 when `2n ≤ 8`) looks up each
    /// neighbour column at the point's height; an empty column, no interval
    /// there (above the column's top surface) or a gap ends the direction.
    /// Inside material, the drop to that interval's bottom over the walked
    /// distance is compared with `tan(α + 0.1°)`; steeper rejects.
    fn near_steep_surface(&self, i: usize) -> bool {
        let p = self.intervals[i].point;
        let (col, row) = self.col_row(self.intervals[i].cell);
        let max_steps = if 8 < 2 * self.spacing_cells {
            (2 * self.spacing_cells) / 3
        } else {
            2
        };
        for (d, &(dx, dy)) in DIRS.iter().enumerate() {
            // Slots 4..8 step √2 by index, as in `link`.
            let step = if d < 4 {
                self.grid.cell_size
            } else {
                self.grid.cell_size * SQRT_2
            };
            let mut dist = 0.0;
            for k in 1..=max_steps {
                let Some(cell) = self
                    .cell_at(col + dx * k, row + dy * k)
                    .filter(|&cell| !self.column(cell).is_empty())
                else {
                    break;
                };
                let Some(j) = self.containing(cell, p.z) else {
                    break;
                };
                if self.intervals[j].gap {
                    break;
                }
                dist += step;
                if (p.z - self.intervals[j].z_min) / dist > self.tan_angle {
                    return true;
                }
            }
        }
        false
    }

    /// Re-centres a support on its level patch: floods the `0.75 n` disc,
    /// takes the centroid cell of the reached gaps within ±0.05 mm of its z,
    /// and returns the first gap in that cell the flood kept.
    fn recentre(&self, i: usize) -> Option<usize> {
        let walk = self.walk(i, &self.disc_cover, true);
        let z = self.intervals[i].z_max;
        let (mut sum_c, mut sum_r, mut count) = (0.0f64, 0.0f64, 0usize);
        for &j in &walk.order {
            let n = &self.intervals[j];
            if (n.z_max - z).abs() < 0.05 {
                let (c, r) = self.col_row(n.cell);
                sum_c += c as f64;
                sum_r += r as f64;
                count += 1;
            }
        }
        if count == 0 {
            return None;
        }
        let c = (sum_c / count as f64 + 0.5).floor() as isize;
        let r = (sum_r / count as f64 + 0.5).floor() as isize;
        let cell = self.cell_at(c, r)?;
        self.column(cell).find(|&j| walk.kept().any(|k| k == j))
    }

    /// Tries to place a support at gap `i` in `pass`: the reject tests, the
    /// re-centring, then the coverage flood (the pass's disc, or the small
    /// one beside a wall) and the claim.
    fn accept(&mut self, i: usize, pass: Pass) -> bool {
        if self.no_room_below(i) || self.near_steep_surface(i) {
            return false;
        }
        let z = self.intervals[i].z_max;
        let recentre = match pass {
            Pass::Keys => true,
            Pass::Sweep => self.intervals[i].normal.z < -0.99,
        };
        let target = recentre
            .then(|| self.recentre(i))
            .flatten()
            .filter(|&j| {
                let n = &self.intervals[j];
                !n.flags.contains(Flags::COVERED) && n.z_max - z < 0.01
            })
            .unwrap_or(i);
        let (disc, mask) = match pass {
            Pass::Keys => (&self.disc_key, Flags::KEY_COVERED | Flags::COVERED),
            Pass::Sweep => (&self.disc_cover, Flags::COVERED),
        };
        let disc = if self.beside_wall(target) {
            &self.disc_small
        } else {
            disc
        };
        let walk = self.walk(target, disc, true);
        self.mark(&walk, mask);
        self.intervals[target].flags |= Flags::CLAIMED;
        true
    }

    /// Supports on the key points, lowest first, each re-centred and covering
    /// the `0.8 n` disc.
    fn place_on_keys(&mut self, keys: &[usize]) -> usize {
        let mut order = keys.to_vec();
        introsort(&mut order, |&a, &b| {
            self.intervals[a].z_max < self.intervals[b].z_max
        });
        let mut placed = 0;
        for i in order {
            if self.intervals[i].flags.intersects(Flags::KEY_SKIP) {
                continue;
            }
            if self.accept(i, Pass::Keys) {
                placed += 1;
            }
        }
        placed
    }

    /// Whether the sweep considers gap `i`: flat, not skipped, not a plain
    /// outline patch gap (its centre is fine), and linked on more than 4
    /// sides.
    fn sweep_candidate(&self, i: usize) -> bool {
        let iv = &self.intervals[i];
        let patch = iv.flags.contains(Flags::PATCH) && !iv.flags.contains(Flags::PATCH_CENTRE);
        iv.gap
            && !iv.flags.intersects(Flags::SWEEP_SKIP)
            && iv.flags.contains(Flags::FLAT)
            && !patch
            && iv.degree > 4
    }

    /// The sweep: a min-heap on z seeded with each column's lowest candidate,
    /// in column-major cell order. Each pop pushes the next candidate above it
    /// in the same column, then tries the popped gap if it is still clear.
    /// Only straight-down gaps (`normal.z < −0.99`) are re-centred; coverage
    /// is the `0.75 n` disc.
    fn sweep(&mut self) -> usize {
        let mut heap = MinHeap::default();
        for col in 0..self.grid.width {
            for row in 0..self.grid.height {
                let cell = row * self.grid.width + col;
                if let Some(i) = self.column(cell).find(|&i| self.sweep_candidate(i)) {
                    heap.push(i, self.intervals[i].z_max);
                }
            }
        }
        let mut placed = 0;
        while let Some(i) = heap.pop() {
            let cell = self.intervals[i].cell as usize;
            if let Some(j) = (i + 1..self.column(cell).end).find(|&j| self.sweep_candidate(j)) {
                heap.push(j, self.intervals[j].z_max);
            }
            if self.intervals[i].flags.intersects(Flags::SWEEP_SKIP) {
                continue;
            }
            if self.accept(i, Pass::Sweep) {
                placed += 1;
            }
        }
        placed
    }
}

impl edge::EdgeGrid for Placer<'_> {
    type Node = usize;

    fn own(&self, point: Vector3<f32>, min_gap: f32) -> Option<usize> {
        let cell = self.cell_of(point)?;
        let i = self.containing(cell, point.z - edge::LOOKUP_DROP)?;
        let iv = &self.intervals[i];
        let lowest = i == self.column(cell).start;
        (iv.gap && (lowest || iv.z_max - iv.z_min >= min_gap)).then_some(i)
    }

    fn face(&self, node: usize) -> (Vector3<f32>, Vector3<f32>) {
        (self.intervals[node].point, self.intervals[node].normal)
    }

    fn link(&self, node: usize, dir: usize) -> Option<usize> {
        self.intervals[node].links[dir].map(|j| j as usize)
    }

    fn neighbour_centre(&self, node: usize, dir: usize) -> Option<Vector3<f32>> {
        let (col, row) = self.col_row(self.intervals[node].cell);
        let (dc, dr) = DIRS[dir];
        self.cell_at(col + dc, row + dr)?;
        Some(
            self.grid
                .cell_centre((col + dc) as usize, (row + dr) as usize),
        )
    }

    fn centre_of(&self, point: Vector3<f32>) -> Option<Vector3<f32>> {
        let (col, row) = self.col_row(self.cell_of(point)? as u32);
        Some(self.grid.cell_centre(col as usize, row as usize))
    }
}

/// The edge re-seat ([`edge`]) on the placement grid it runs on.
#[cfg(test)]
mod edge_tests {
    use common::progress::Progress;
    use nalgebra::Vector3;
    use slicer::geometry::{mesh::Mesh, partitioning::Bvh};

    use super::Placer;
    use crate::supports::auto::{
        PlacementConfig,
        edge::{EdgeConfig, adjust, adjust_points},
        grid::footprint,
        points::PlacedPoint,
    };

    /// Middle preset: Ø 1.2 pillar, inset = 0.2 + 0.2 × (0.6 − 0.2) = 0.28.
    const CONFIG: EdgeConfig = EdgeConfig {
        pillar_diameter: 1.2,
        inset: 0.28,
        min_gap: 3.0,
    };

    /// A 20 × 20 × 4 mm box lifted to z = 5, with a 45° chamfer along its
    /// bottom +x edge (the bottom face ends at x = 8, the chamfer rises to
    /// x = 10, z = 7).
    fn chamfered_box() -> Mesh {
        let v = Vector3::new;
        #[rustfmt::skip]
        let verts = vec![
            v(-10.0, -10.0, 5.0), v(8.0, -10.0, 5.0), v(8.0, 10.0, 5.0), v(-10.0, 10.0, 5.0),
            v(10.0, -10.0, 7.0), v(10.0, 10.0, 7.0),
            v(-10.0, -10.0, 9.0), v(10.0, -10.0, 9.0), v(10.0, 10.0, 9.0), v(-10.0, 10.0, 9.0),
        ];
        #[rustfmt::skip]
        let faces = vec![
            [0, 2, 1], [0, 3, 2],             // bottom (down)
            [1, 2, 5], [1, 5, 4],             // chamfer (down and +x)
            [6, 7, 8], [6, 8, 9],             // top (up)
            [4, 5, 8], [4, 8, 7],             // +x side
            [3, 0, 6], [3, 6, 9],             // −x side
            [0, 1, 4], [0, 4, 7], [0, 7, 6],  // −y side
            [3, 9, 8], [3, 8, 5], [3, 5, 2],  // +y side
        ];
        Mesh::new(verts, faces)
    }

    /// Runs `f` with the box's linked placement grid.
    fn with_placer<R>(mesh: &Mesh, f: impl FnOnce(&Placer, &Bvh) -> R) -> R {
        let bvh = Bvh::build(mesh.inner(), Progress::new());
        let grid = footprint(mesh).unwrap();
        let config = PlacementConfig::default();
        let progress = Progress::new();
        let mut placer = Placer::new(mesh, &bvh, &grid, &config, 1.2, 0.4, &progress);
        placer.link();
        f(&placer, &bvh)
    }

    fn adjust_one(mesh: &Mesh, at: Vector3<f32>) -> Option<(Vector3<f32>, Vector3<f32>)> {
        with_placer(mesh, |placer, bvh| adjust(at, placer, mesh, bvh, &CONFIG))
    }

    #[test]
    fn rim_point_moves_inside_the_edge() {
        // On the bottom face right at the crease: pulled 0.28 mm inside it.
        let mesh = chamfered_box();
        let (pos, normal) = adjust_one(&mesh, Vector3::new(7.98, 0.0, 5.0)).expect("moved");
        assert!((pos.z - 5.0).abs() < 1e-3);
        assert!(normal.z < -0.99, "takes the bottom face's normal");
        assert!(pos.x < 8.0 && pos.x > 8.0 - 0.4, "x = {}", pos.x);
    }

    #[test]
    fn chamfer_point_moves_onto_the_bottom_face() {
        // On the chamfer just above the crease: the bottom face is more
        // downward, so the point moves onto it.
        let mesh = chamfered_box();
        let (pos, normal) = adjust_one(&mesh, Vector3::new(8.03, 0.0, 5.03)).expect("moved");
        assert!(normal.z < -0.99);
        assert!((pos.z - 5.0).abs() < 1e-3 && pos.x < 8.0);
    }

    #[test]
    fn interior_point_stays() {
        let mesh = chamfered_box();
        assert!(adjust_one(&mesh, Vector3::new(0.0, 0.0, 5.0)).is_none());
    }

    #[test]
    fn islands_are_skipped() {
        let mesh = chamfered_box();
        let mut points = [PlacedPoint {
            point: Vector3::new(7.98, 0.0, 5.0),
            normal: -Vector3::z(),
            island: true,
        }];
        let moved = with_placer(&mesh, |placer, bvh| {
            adjust_points(&mut points, placer, &mesh, bvh, &CONFIG)
        });
        assert_eq!(moved, 0);
        assert_eq!(points[0].point, Vector3::new(7.98, 0.0, 5.0));
    }
}
