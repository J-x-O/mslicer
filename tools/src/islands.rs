//! Island detection: the bottom layers of parts that start in mid-air.
//!
//! The mesh is cut into contours on evenly spaced horizontal planes. A
//! cross-section is an island when it shares no area with any cross-section on
//! the plane below but does share area with one on the plane above: it is the
//! first layer of a part with nothing under it. A cross-section with nothing
//! above it either (one plane thick) is ignored.
//!
//! All shells are merged per plane first, so parts that overlap or touch count
//! as one: they print fused. Overhangs that are part of the same cross-section
//! as the part holding them up are not islands; those are the support
//! generator's business. This check only finds regions that start
//! disconnected.

use std::collections::HashMap;

use common::progress::Progress;
use i_overlay::{
    core::{fill_rule::FillRule, overlay_rule::OverlayRule},
    float::{simplify::SimplifyShape, single::SingleFloatOverlay},
    mesh::float::{outline::offset::OutlineOffset, style::OutlineStyle},
};
use i_triangle::float::triangulatable::Triangulatable;
use nalgebra::{Isometry3, Point3, Vector2, Vector3};
use rayon::iter::{IntoParallelIterator, ParallelIterator};
use slicer::geometry::{
    bounds::Bounds3D,
    mesh::{Mesh, MeshBuilder},
    partitioning::Bvh,
};

/// One closed ring. Outer rings run counter-clockwise, holes clockwise.
pub type Contour = Vec<[f32; 2]>;
/// An outer ring followed by its holes.
pub type Shape = Vec<Contour>;

/// Open contour ends closer than this are joined.
const CLOSE_GAP: f32 = 0.1;
/// Contour points closer than this are merged.
const MIN_SEGMENT: f32 = 1e-4;
/// Cross-sections thinner than this (mean width, 2 · area / perimeter) are
/// dropped: slivers from degenerate faces, too thin to print or support. A
/// part that starts as a sliver is found on the first plane where it is wider.
const MIN_WIDTH: f32 = 0.02;
/// How far past the tip a contact still counts as touching an island, and the
/// extra height band around the island's layer.
const COVER_MARGIN: f32 = 0.15;

/// How far the overlay reaches past an island's outline, so it shows around
/// the model's underside.
pub const OVERLAY_GROW: f32 = 0.15;

/// Where the cutting planes sit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Planes {
    /// Distance between planes: the layer height, or a multiple of it.
    pub spacing: f32,
    /// Height of plane 0. Planes sit at `origin + k * spacing`; with the
    /// origin on the plate they match the slicer's layers.
    pub origin: f32,
}

impl Planes {
    /// Planes on the slicer's layers, `step` layers apart.
    pub fn layers(layer_height: f32, step: u32) -> Self {
        Self {
            spacing: layer_height * step.max(1) as f32,
            origin: 0.0,
        }
    }

    fn z(&self, k: i64) -> f32 {
        self.origin + k as f32 * self.spacing
    }
}

#[derive(Clone, Debug)]
pub struct Island {
    /// Height of the plane the island was found on. The part's real bottom is
    /// between this and one spacing lower.
    pub z: f32,
    /// The cross-section on that plane, in world xy.
    pub shape: Shape,
    pub area: f32,
    pub min: Vector2<f32>,
    pub max: Vector2<f32>,
    /// Starts within one spacing of the plate, so the plate holds it.
    pub on_plate: bool,
    /// Contact point on the island's underside (world space): the first of
    /// [`Island::contacts`].
    pub anchor: Vector3<f32>,
    /// Surface normal at the anchor (world space).
    pub normal: Vector3<f32>,
    /// Places a support could hold the island from, best first, each with its
    /// surface normal: the lowest vertex under the cross-section, and points
    /// across the underside. A flat underside puts its inner points first, so
    /// a support doesn't land on an edge.
    pub contacts: Vec<(Vector3<f32>, Vector3<f32>)>,
}

impl Island {
    /// Whether `xy` is on the island or within `reach` of its outline.
    pub fn near(&self, xy: Vector2<f32>, reach: f32) -> bool {
        xy.x >= self.min.x - reach
            && xy.x <= self.max.x + reach
            && xy.y >= self.min.y - reach
            && xy.y <= self.max.y + reach
            && near_shape(&self.shape, xy, reach)
    }
}

#[derive(Clone, Debug, Default)]
pub struct Detection {
    pub islands: Vec<Island>,
    pub spacing: f32,
    /// Planes whose contours did not close (holes or non-manifold edges in
    /// the mesh). Their results may be incomplete.
    pub open_planes: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cover {
    Supported,
    OnPlate,
    Unsupported,
}

/// Finds the islands of `mesh` (world space, at its current pose).
pub fn detect(mesh: &Mesh, bvh: &Bvh, planes: Planes, progress: &Progress) -> Detection {
    let bounds = Bounds3D::from_mesh(mesh);
    let spacing = planes.spacing;
    if spacing <= 0.0 || bounds.max.z <= bounds.min.z {
        progress.set_finished();
        return Detection {
            spacing,
            ..Default::default()
        };
    }

    // Plane indices from the last one at or below the bottom to the last one
    // below the top. The first is empty (nothing is below it).
    let first = (((bounds.min.z - planes.origin) / spacing).floor() as i64).max(0);
    let mut last = ((bounds.max.z - planes.origin) / spacing).ceil() as i64;
    while last > first && planes.z(last) >= bounds.max.z {
        last -= 1;
    }
    let count = (last - first + 1) as usize;
    progress.set_total(count as u64 * 2);

    let points: Vec<Vector3<f32>> = (mesh.vertices().iter())
        .map(|v| mesh.transform(v))
        .collect();
    let faces = bin_faces(mesh, &points, planes, first, count);
    let low = bin_vertices(&points, planes, first, count);

    let sections: Vec<(Vec<Section>, bool)> = (0..count)
        .into_par_iter()
        .map(|i| {
            let z = planes.z(first + i as i64);
            let section = cut(mesh, &points, &faces[i], z);
            progress.add_complete(1);
            section
        })
        .collect();
    let open_planes = sections.iter().filter(|(_, open)| *open).count();
    let sections: Vec<Vec<Section>> = sections.into_iter().map(|(s, _)| s).collect();

    let empty = Vec::new();
    let islands: Vec<Island> = (0..count)
        .into_par_iter()
        .flat_map_iter(|i| {
            let below = if i == 0 { &empty } else { &sections[i - 1] };
            let above = sections.get(i + 1).unwrap_or(&empty);
            let z = planes.z(first + i as i64);
            let found: Vec<Island> = (sections[i].iter())
                .filter(|s| !touches(s, below) && touches(s, above))
                .map(|s| island(mesh, bvh, s, z, spacing, &points, &low[i]))
                .collect();
            progress.add_complete(1);
            found
        })
        .collect();

    progress.set_finished();
    Detection {
        islands,
        spacing,
        open_planes,
    }
}

/// How each island is held: by the plate, by one of `contacts` (support tips
/// on this model, world space, each with its tip radius), or not at all. A
/// contact holds an island when it is within its layer band and within its tip
/// radius (plus a margin) of the cross-section.
pub fn coverage(detection: &Detection, contacts: &[(Vector3<f32>, f32)]) -> Vec<Cover> {
    let band = detection.spacing + COVER_MARGIN;
    (detection.islands.iter())
        .map(|island| {
            if island.on_plate {
                return Cover::OnPlate;
            }
            let held = contacts.iter().any(|&(c, tip_radius)| {
                let reach = tip_radius + COVER_MARGIN;
                (c.z - island.z).abs() <= band
                    && c.x >= island.min.x - reach
                    && c.x <= island.max.x + reach
                    && c.y >= island.min.y - reach
                    && c.y <= island.max.y + reach
                    && near_shape(&island.shape, c.xy(), reach)
            });
            if held {
                Cover::Supported
            } else {
                Cover::Unsupported
            }
        })
        .collect()
}

/// A slab per island, for drawing: the cross-section grown by
/// [`OVERLAY_GROW`], from a little below the plane underneath up to the
/// island's own plane, so it covers wherever the underside lies in between.
pub fn overlay_mesh<'a>(islands: impl IntoIterator<Item = &'a Island>, spacing: f32) -> Mesh {
    let mut builder = MeshBuilder::new();
    for island in islands {
        let (z0, z1) = (island.z - spacing - OVERLAY_GROW / 2.0, island.z);
        for shape in island.shape.outline(&OutlineStyle::new(OVERLAY_GROW)) {
            let tri = shape.triangulate().to_triangulation::<u32>();
            for z in [z0, z1] {
                let base = builder.next_idx();
                for &[x, y] in &tri.points {
                    builder.add_vertex(Vector3::new(x, y, z));
                }
                for t in tri.indices.as_chunks::<3>().0 {
                    builder.add_face(t.map(|i| base + i));
                }
            }
            for ring in &shape {
                let n = ring.len();
                for i in 0..n {
                    let ([px, py], [qx, qy]) = (ring[i], ring[(i + 1) % n]);
                    let p0 = builder.add_vertex(Vector3::new(px, py, z0));
                    let q0 = builder.add_vertex(Vector3::new(qx, qy, z0));
                    let q1 = builder.add_vertex(Vector3::new(qx, qy, z1));
                    let p1 = builder.add_vertex(Vector3::new(px, py, z1));
                    builder.add_face([p0, q0, q1]);
                    builder.add_face([p0, q1, p1]);
                }
            }
        }
    }
    builder.build()
}

impl Detection {
    /// Carries the islands along a rigid motion that keeps heights (a move
    /// across the plate or a turn about z).
    pub fn transform(&mut self, motion: &Isometry3<f32>) {
        let flat = |xy: [f32; 2]| {
            let p = motion.transform_point(&Point3::new(xy[0], xy[1], 0.0));
            [p.x, p.y]
        };
        for island in &mut self.islands {
            for ring in &mut island.shape {
                ring.iter_mut().for_each(|p| *p = flat(*p));
            }
            (island.min, island.max) = shape_bounds(&island.shape);
            island.anchor = motion.transform_point(&island.anchor.into()).coords;
            island.normal = motion.transform_vector(&island.normal);
            for (p, n) in &mut island.contacts {
                *p = motion.transform_point(&(*p).into()).coords;
                *n = motion.transform_vector(n);
            }
        }
    }
}

/// A cross-section on one plane.
struct Section {
    shape: Shape,
    min: Vector2<f32>,
    max: Vector2<f32>,
    /// A few points just inside the outline, for a quick overlap test.
    probes: Vec<Vector2<f32>>,
}

/// The vertices between each plane and the one below it: where a part found
/// on that plane has its bottom.
fn bin_vertices(
    points: &[Vector3<f32>],
    planes: Planes,
    first: i64,
    count: usize,
) -> Vec<Vec<u32>> {
    let mut bins = vec![Vec::new(); count];
    for (v, p) in points.iter().enumerate() {
        // The first plane above the vertex, by the cut's rule (a vertex at a
        // plane counts as above it).
        let mut k = ((p.z - planes.origin) / planes.spacing).floor() as i64 + 1;
        while planes.z(k) <= p.z {
            k += 1;
        }
        while planes.z(k - 1) > p.z {
            k -= 1;
        }
        if let Some(bin) = usize::try_from(k - first)
            .ok()
            .and_then(|i| bins.get_mut(i))
        {
            bin.push(v as u32);
        }
    }
    bins
}

/// The faces that cross each plane, i.e. have a vertex below it and one at or
/// above it.
fn bin_faces(
    mesh: &Mesh,
    points: &[Vector3<f32>],
    planes: Planes,
    first: i64,
    count: usize,
) -> Vec<Vec<u32>> {
    let mut bins = vec![Vec::new(); count];
    for (face, verts) in mesh.faces().iter().enumerate() {
        let zs = verts.map(|v| points[v as usize].z);
        let (lo, hi) = (zs[0].min(zs[1]).min(zs[2]), zs[0].max(zs[1]).max(zs[2]));
        if lo == hi {
            continue;
        }
        let k_lo = ((lo - planes.origin) / planes.spacing).floor() as i64;
        let k_hi = ((hi - planes.origin) / planes.spacing).ceil() as i64;
        for k in k_lo.max(first)..=k_hi.min(first + count as i64 - 1) {
            let z = planes.z(k);
            if lo < z && z <= hi {
                bins[(k - first) as usize].push(face as u32);
            }
        }
    }
    bins
}

/// One contour piece across a face: from where the surface goes down through
/// the plane to where it comes back up, so the material is on its left.
struct Piece {
    from: u64,
    to: u64,
    start: [f32; 2],
    end: [f32; 2],
}

/// Cuts the plane at `z`. Returns its cross-sections and whether any contour
/// was left open.
fn cut(mesh: &Mesh, points: &[Vector3<f32>], faces: &[u32], z: f32) -> (Vec<Section>, bool) {
    // A vertex at the plane counts as above it.
    let crossing = |a: u32, b: u32| -> (u64, [f32; 2]) {
        // Key and point from the ordered edge, so both faces on the edge
        // agree on them exactly.
        let (u, v) = if a < b { (a, b) } else { (b, a) };
        let (pu, pv) = (points[u as usize], points[v as usize]);
        let t = (z - pu.z) / (pv.z - pu.z);
        let p = pu.xy() + (pv.xy() - pu.xy()) * t;
        (((u as u64) << 32) | v as u64, [p.x, p.y])
    };

    let mut pieces = Vec::with_capacity(faces.len());
    for &face in faces {
        let verts = *mesh.face(face as usize);
        let above = verts.map(|v| points[v as usize].z >= z);
        let (mut down, mut up) = (None, None);
        for i in 0..3 {
            let (a, b) = (verts[i], verts[(i + 1) % 3]);
            let (ia, ib) = (above[i], above[(i + 1) % 3]);
            if ia && !ib {
                down = Some(crossing(a, b));
            } else if !ia && ib {
                up = Some(crossing(a, b));
            }
        }
        if let (Some((from, start)), Some((to, end))) = (down, up) {
            pieces.push(Piece {
                from,
                to,
                start,
                end,
            });
        }
    }

    let (mut rings, open) = chain(&pieces);
    let left_open = !open.is_empty();
    rings.extend(join_open(chain_open(&pieces, &open)));

    let rings: Vec<Contour> = (rings.into_iter())
        .map(dedupe)
        .filter(|r| r.len() >= 3)
        .collect();
    if rings.is_empty() {
        return (Vec::new(), left_open);
    }

    let sections = (rings.simplify_shape(FillRule::NonZero).into_iter())
        .filter(|shape| {
            let area = shape_area(shape);
            area > 0.0 && 2.0 * area / shape_perimeter(shape) >= MIN_WIDTH
        })
        .map(|shape| {
            let (min, max) = shape_bounds(&shape);
            let probes = probes(&shape);
            Section {
                shape,
                min,
                max,
                probes,
            }
        })
        .collect();
    (sections, left_open)
}

/// Follows pieces end to start into closed rings. Also returns the pieces
/// that are on chains which never close.
fn chain(pieces: &[Piece]) -> (Vec<Contour>, Vec<usize>) {
    let mut starts: HashMap<u64, Vec<usize>> = HashMap::with_capacity(pieces.len());
    for (i, piece) in pieces.iter().enumerate() {
        starts.entry(piece.from).or_default().push(i);
    }

    let mut used = vec![false; pieces.len()];
    let mut rings = Vec::new();
    let mut open = Vec::new();
    for first in 0..pieces.len() {
        if used[first] {
            continue;
        }
        let mut ring = Vec::new();
        let mut members = Vec::new();
        let mut at = first;
        let closed = loop {
            used[at] = true;
            members.push(at);
            ring.push(pieces[at].start);
            let key = pieces[at].to;
            if key == pieces[first].from {
                break true;
            }
            let next = (starts.get(&key).into_iter().flatten()).find(|&&j| !used[j]);
            match next {
                Some(&j) => at = j,
                None => break false,
            }
        };
        if closed {
            rings.push(ring);
        } else {
            open.extend(members);
        }
    }
    (rings, open)
}

/// Re-chains the `open` pieces into point lists from first to last point.
fn chain_open(pieces: &[Piece], open: &[usize]) -> Vec<Contour> {
    if open.is_empty() {
        return Vec::new();
    }

    // Re-chain only the open pieces, starting from pieces nothing leads into.
    let subset: Vec<&Piece> = open.iter().map(|&i| &pieces[i]).collect();
    let mut starts: HashMap<u64, Vec<usize>> = HashMap::new();
    for (i, piece) in subset.iter().enumerate() {
        starts.entry(piece.from).or_default().push(i);
    }
    let mut led_into = vec![false; subset.len()];
    for piece in &subset {
        for &j in starts.get(&piece.to).into_iter().flatten() {
            led_into[j] = true;
        }
    }

    let mut used = vec![false; subset.len()];
    let mut chains = Vec::new();
    let order = (0..subset.len())
        .filter(|&i| !led_into[i])
        .chain(0..subset.len());
    for first in order.collect::<Vec<_>>() {
        if used[first] {
            continue;
        }
        let mut chain = Vec::new();
        let mut at = first;
        loop {
            used[at] = true;
            chain.push(subset[at].start);
            let next = (starts.get(&subset[at].to).into_iter().flatten()).find(|&&j| !used[j]);
            match next {
                Some(&j) => at = j,
                None => {
                    chain.push(subset[at].end);
                    break;
                }
            }
        }
        chains.push(chain);
    }
    chains
}

/// Joins open chains whose ends are within [`CLOSE_GAP`] into rings, greedily
/// by nearest end. A chain with no partner is closed straight across its gap,
/// so a missing triangle doesn't lose the whole cross-section.
fn join_open(mut chains: Vec<Contour>) -> Vec<Contour> {
    let dist = |a: [f32; 2], b: [f32; 2]| ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt();
    let mut rings = Vec::new();
    while let Some(mut chain) = chains.pop() {
        loop {
            let (start, end) = (chain[0], *chain.last().unwrap());
            if chain.len() >= 3 && dist(start, end) <= CLOSE_GAP {
                rings.push(chain);
                break;
            }
            let next = (chains.iter().enumerate())
                .map(|(i, c)| (i, dist(end, c[0])))
                .filter(|&(_, d)| d <= CLOSE_GAP)
                .min_by(|a, b| a.1.total_cmp(&b.1));
            match next {
                Some((i, _)) => chain.extend(chains.swap_remove(i)),
                None => {
                    if chain.len() >= 3 {
                        rings.push(chain);
                    }
                    break;
                }
            }
        }
    }
    rings
}

fn dedupe(ring: Contour) -> Contour {
    let close = |p: [f32; 2], q: [f32; 2]| {
        (p[0] - q[0]).abs() <= MIN_SEGMENT && (p[1] - q[1]).abs() <= MIN_SEGMENT
    };
    let mut out: Contour = Vec::with_capacity(ring.len());
    for p in ring {
        if out.last().is_none_or(|&q| !close(p, q)) {
            out.push(p);
        }
    }
    while out.len() > 1 && close(out[0], out[out.len() - 1]) {
        out.pop();
    }
    out
}

/// Whether `section` shares area with any of `others`.
fn touches(section: &Section, others: &[Section]) -> bool {
    let near: Vec<&Section> = (others.iter())
        .filter(|o| {
            o.min.x < section.max.x
                && o.max.x > section.min.x
                && o.min.y < section.max.y
                && o.max.y > section.min.y
        })
        .collect();
    if near.is_empty() {
        return false;
    }
    // A point inside both shapes means they share area. This settles the
    // common case, a cross-section stacked on a similar one, without the
    // boolean operation.
    let probed = |a: &Section, b: &Section| a.probes.iter().any(|&p| contains(&b.shape, p));
    if near
        .iter()
        .any(|o| probed(section, o) || probed(o, section))
    {
        return true;
    }
    let near: Vec<Shape> = near.iter().map(|o| o.shape.clone()).collect();
    let common = (section.shape).overlay(&near, OverlayRule::Intersect, FillRule::NonZero);
    common.iter().any(|shape| shape_area(shape) > 0.0)
}

fn island(
    mesh: &Mesh,
    bvh: &Bvh,
    section: &Section,
    z: f32,
    spacing: f32,
    points: &[Vector3<f32>],
    low: &[u32],
) -> Island {
    /// Inner points tried per island.
    const SAMPLES: usize = 6;
    /// How far outside the cross-section the lowest vertex may lie: a sloped
    /// underside reaches past the cross-section above it.
    const LOW_REACH: f32 = 0.2;

    // Straight up from the plane below to the part's underside.
    let up = |xy: Vector2<f32>| {
        let hit = bvh.intersect_ray(mesh, xy.push(z - spacing), Vector3::z())?;
        let p = mesh.transform(&hit.position);
        let n = mesh.transform_normal(&mesh.normal(hit.face));
        (p.z <= z + 1e-3).then(|| (p, n.try_normalize(f32::EPSILON).unwrap_or(-Vector3::z())))
    };
    let inner: Vec<_> = (inner_points(&section.shape).into_iter().take(SAMPLES))
        .filter_map(up)
        .collect();
    let lowest = (low.iter().map(|&v| points[v as usize]))
        .filter(|p| near_shape(&section.shape, p.xy(), LOW_REACH))
        .min_by(|a, b| a.z.total_cmp(&b.z))
        .map(|p| (p, -Vector3::z()));

    // Within about 18° of horizontal.
    let flat = inner.first().is_some_and(|(_, n)| n.z < -0.95);
    let mut contacts: Vec<_> = if flat {
        inner.into_iter().chain(lowest).collect()
    } else {
        lowest.into_iter().chain(inner).collect()
    };
    if contacts.is_empty() {
        let centre = (section.min + section.max) / 2.0;
        contacts.push((centre.push(z), -Vector3::z()));
    }
    let (anchor, normal) = contacts[0];

    Island {
        z,
        shape: section.shape.clone(),
        area: shape_area(&section.shape),
        min: section.min,
        max: section.max,
        on_plate: z - spacing <= 1e-4,
        anchor,
        normal,
        contacts,
    }
}

/// Points well inside the shape, best first: the centroids of its
/// triangles, largest first.
fn inner_points(shape: &Shape) -> Vec<Vector2<f32>> {
    let tri = shape.triangulate().to_triangulation::<u32>();
    let mut triangles: Vec<[Vector2<f32>; 3]> = (tri.indices.as_chunks::<3>().0.iter())
        .map(|t| t.map(|i| Vector2::from(tri.points[i as usize])))
        .collect();
    triangles.sort_by(|a, b| triangle_area(b).total_cmp(&triangle_area(a)));
    triangles
        .iter()
        .map(|[a, b, c]| (a + b + c) / 3.0)
        .collect()
}

fn triangle_area([a, b, c]: &[Vector2<f32>; 3]) -> f32 {
    ((b - a).perp(&(c - a)) / 2.0).abs()
}

fn ring_area(ring: &Contour) -> f32 {
    let n = ring.len();
    (0..n)
        .map(|i| {
            let ([ax, ay], [bx, by]) = (ring[i], ring[(i + 1) % n]);
            ax * by - bx * ay
        })
        .sum::<f32>()
        / 2.0
}

fn shape_area(shape: &Shape) -> f32 {
    shape.iter().map(ring_area).sum::<f32>().abs()
}

fn shape_perimeter(shape: &Shape) -> f32 {
    (shape.iter())
        .flat_map(|ring| {
            let n = ring.len();
            (0..n).map(move |i| (Vector2::from(ring[i]) - Vector2::from(ring[(i + 1) % n])).norm())
        })
        .sum()
}

fn shape_bounds(shape: &Shape) -> (Vector2<f32>, Vector2<f32>) {
    let mut min = Vector2::repeat(f32::MAX);
    let mut max = Vector2::repeat(f32::MIN);
    for &[x, y] in shape.iter().flatten() {
        min = min.inf(&Vector2::new(x, y));
        max = max.sup(&Vector2::new(x, y));
    }
    (min, max)
}

fn edges(shape: &Shape) -> impl Iterator<Item = (Vector2<f32>, Vector2<f32>)> + '_ {
    shape.iter().flat_map(|ring| {
        let n = ring.len();
        (0..n).map(move |i| (Vector2::from(ring[i]), Vector2::from(ring[(i + 1) % n])))
    })
}

/// Whether `p` is inside the shape (even-odd over all rings).
fn contains(shape: &Shape, p: Vector2<f32>) -> bool {
    let crossings = edges(shape)
        .filter(|(a, b)| {
            (a.y > p.y) != (b.y > p.y) && p.x < a.x + (p.y - a.y) / (b.y - a.y) * (b.x - a.x)
        })
        .count();
    crossings % 2 == 1
}

/// Up to four points a hair inside the outer ring, off the midpoints of
/// evenly spread edges. Material is on the left of every ring, so the left
/// normal points in; each point is checked to really be inside.
fn probes(shape: &Shape) -> Vec<Vector2<f32>> {
    const INSET: f32 = MIN_WIDTH / 8.0;
    let ring = &shape[0];
    let n = ring.len();
    (0..4)
        .map(|i| i * n / 4)
        .filter_map(|i| {
            let (a, b) = (Vector2::from(ring[i]), Vector2::from(ring[(i + 1) % n]));
            let along = (b - a).try_normalize(f32::EPSILON)?;
            let p = (a + b) / 2.0 + Vector2::new(-along.y, along.x) * INSET;
            contains(shape, p).then_some(p)
        })
        .collect()
}

/// Whether `p` is inside the shape or within `reach` of its outline.
fn near_shape(shape: &Shape, p: Vector2<f32>, reach: f32) -> bool {
    contains(shape, p)
        || edges(shape).any(|(a, b)| {
            let ab = b - a;
            let t = ((p - a).dot(&ab) / ab.norm_squared().max(f32::EPSILON)).clamp(0.0, 1.0);
            (a + ab * t - p).norm() <= reach
        })
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use common::progress::Progress;
    use i_triangle::float::triangulatable::Triangulatable;
    use nalgebra::Vector3;
    use slicer::geometry::{
        mesh::{Mesh, MeshBuilder},
        partitioning::Bvh,
    };

    use super::{Cover, Detection, Planes, coverage, detect};

    const H: f32 = 0.1;

    /// Welds triangles by exact position, like the STL loader.
    fn weld(triangles: &[[Vector3<f32>; 3]]) -> Mesh {
        let mut index = HashMap::new();
        let mut vertices = Vec::new();
        let faces = (triangles.iter())
            .map(|t| {
                t.map(|v| {
                    *index.entry(v.map(f32::to_bits)).or_insert_with(|| {
                        vertices.push(v);
                        vertices.len() as u32 - 1
                    })
                })
            })
            .collect();
        Mesh::new(vertices, faces)
    }

    /// A counter-clockwise profile in the xz plane, extruded over `y0..y1`.
    fn prism(profile: &[[f32; 2]], (y0, y1): (f32, f32), flip: bool) -> Vec<[Vector3<f32>; 3]> {
        let at = |[x, z]: [f32; 2], y: f32| Vector3::new(x, y, z);
        let mut out = Vec::new();
        let n = profile.len();
        for i in 0..n {
            let (p, q) = (profile[i], profile[(i + 1) % n]);
            out.push([at(p, y0), at(q, y1), at(q, y0)]);
            out.push([at(p, y0), at(p, y1), at(q, y1)]);
        }
        let tri = vec![profile.to_vec()]
            .triangulate()
            .to_triangulation::<u32>();
        for t in tri.indices.as_chunks::<3>().0 {
            let [a, b, c] = t.map(|i| tri.points[i as usize]);
            let ccw = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) > 0.0;
            let (a, b, c) = if ccw { (a, b, c) } else { (a, c, b) };
            out.push([at(a, y0), at(b, y0), at(c, y0)]);
            out.push([at(a, y1), at(c, y1), at(b, y1)]);
        }
        if flip {
            for t in &mut out {
                t.swap(1, 2);
            }
        }
        out
    }

    fn rect([x0, z0]: [f32; 2], [x1, z1]: [f32; 2]) -> Vec<[f32; 2]> {
        vec![[x0, z0], [x1, z0], [x1, z1], [x0, z1]]
    }

    fn cuboid(min: [f32; 3], max: [f32; 3]) -> Vec<[Vector3<f32>; 3]> {
        prism(
            &rect([min[0], min[2]], [max[0], max[2]]),
            (min[1], max[1]),
            false,
        )
    }

    fn run(mesh: &Mesh) -> Detection {
        let bvh = Bvh::build(mesh.inner(), Progress::new());
        detect(mesh, &bvh, Planes::layers(H, 1), &Progress::new())
    }

    fn floating(d: &Detection) -> Vec<f32> {
        d.islands
            .iter()
            .filter(|i| !i.on_plate)
            .map(|i| i.z)
            .collect()
    }

    #[test]
    fn cube_on_plate() {
        let d = run(&weld(&cuboid([0.0; 3], [10.0; 3])));
        assert_eq!(d.islands.len(), 1);
        assert!(d.islands[0].on_plate);
        assert!((d.islands[0].area - 100.0).abs() < 1e-2);
        assert_eq!(d.open_planes, 0);
    }

    #[test]
    fn floating_cube() {
        let d = run(&weld(&cuboid([0.0, 0.0, 5.0], [10.0, 10.0, 10.0])));
        assert_eq!(d.islands.len(), 1);
        let island = &d.islands[0];
        assert!(!island.on_plate);
        assert!(
            island.z > 5.0 && island.z <= 5.0 + H + 1e-4,
            "z = {}",
            island.z
        );
        assert!((island.area - 100.0).abs() < 1e-2);
        // The anchor is on the cube's underside, pointing down.
        assert!((island.anchor.z - 5.0).abs() < 1e-3);
        assert!(island.normal.z < -0.99);
    }

    #[test]
    fn thin_slab() {
        let d = run(&weld(&cuboid([0.0, 0.0, 5.05], [10.0, 10.0, 5.55])));
        assert_eq!(floating(&d).len(), 1);
    }

    #[test]
    fn one_plane_speck_is_ignored() {
        // Only the plane at z = 5.1 cuts it.
        let d = run(&weld(&cuboid([0.0, 0.0, 5.05], [10.0, 10.0, 5.15])));
        assert!(d.islands.is_empty());
    }

    #[test]
    fn connected_overhang_is_not_an_island() {
        let t = [
            [4.0, 0.0],
            [6.0, 0.0],
            [6.0, 5.0],
            [10.0, 5.0],
            [10.0, 7.0],
            [0.0, 7.0],
            [0.0, 5.0],
            [4.0, 5.0],
        ];
        let d = run(&weld(&prism(&t, (0.0, 5.0), false)));
        assert_eq!(d.open_planes, 0);
        assert_eq!(d.islands.len(), 1);
        assert!(d.islands[0].on_plate);
    }

    #[test]
    fn overlapping_parts_are_connected() {
        let mut tris = cuboid([4.0, 0.0, 0.0], [6.0, 5.0, 7.0]);
        tris.extend(cuboid([0.0, 0.0, 5.0], [10.0, 5.0, 7.0]));
        let d = run(&weld(&tris));
        assert!(floating(&d).is_empty());
    }

    #[test]
    fn touching_parts_are_connected() {
        // An arm sharing only a face with the pillar still prints fused.
        let mut tris = cuboid([0.0, 0.0, 0.0], [2.0, 2.0, 10.0]);
        tris.extend(cuboid([2.0, 0.0, 5.0], [10.0, 10.0, 6.0]));
        let d = run(&weld(&tris));
        assert!(floating(&d).is_empty());
    }

    #[test]
    fn part_beside_a_gap_is_an_island() {
        let mut tris = cuboid([0.0, 0.0, 0.0], [2.0, 2.0, 10.0]);
        tris.extend(cuboid([2.5, 0.0, 5.0], [10.0, 10.0, 6.0]));
        let d = run(&weld(&tris));
        let z = floating(&d);
        assert_eq!(z.len(), 1);
        assert!((z[0] - 5.0).abs() <= H + 1e-4);
    }

    #[test]
    fn cavity_makes_no_island() {
        let mut tris = cuboid([0.0; 3], [10.0; 3]);
        tris.extend(prism(&rect([2.0, 2.0], [8.0, 8.0]), (2.0, 8.0), true));
        let d = run(&weld(&tris));
        assert_eq!(d.islands.len(), 1);
        assert!(d.islands[0].on_plate);
    }

    #[test]
    fn inside_out_mesh() {
        let d = run(&weld(&prism(
            &rect([0.0, 5.0], [10.0, 10.0]),
            (0.0, 10.0),
            true,
        )));
        assert_eq!(floating(&d).len(), 1);
    }

    #[test]
    fn floating_sphere() {
        let mut builder = MeshBuilder::new();
        builder.add_sphere(Vector3::new(0.0, 0.0, 10.0), 5.0, 32);
        let d = run(&builder.build());
        assert_eq!(d.open_planes, 0);
        let z = floating(&d);
        assert_eq!(z.len(), 1);
        assert!((z[0] - 5.0).abs() <= H + 1e-3, "z = {}", z[0]);
        // Radius √(5² − 4.9²) ≈ 1 on the first plane above the bottom.
        let area = d.islands.iter().find(|i| !i.on_plate).unwrap().area;
        assert!(area < 4.0, "area = {area}");
    }

    #[test]
    fn open_mesh_does_not_fail() {
        // Drop one triangle of a side wall: every plane's ring has a gap.
        let mut tris = cuboid([0.0, 0.0, 5.0], [10.0, 10.0, 10.0]);
        tris.remove(2);
        let d = run(&weld(&tris));
        assert!(d.open_planes > 0);
        assert_eq!(floating(&d).len(), 1);
    }

    #[test]
    fn t_junctions_close() {
        // Split the x = 10 wall with a vertex on its edge to the y = 0 wall,
        // which that wall doesn't share: the contours break at the junction.
        let mut tris = cuboid([0.0, 0.0, 5.0], [10.0, 10.0, 10.0]);
        tris.retain(|t| !t.iter().all(|v| v.x == 10.0));
        let v = |y: f32, z: f32| Vector3::new(10.0, y, z);
        let mid = v(0.0, 7.5);
        for mut t in [
            [v(0.0, 5.0), v(10.0, 5.0), mid],
            [mid, v(10.0, 5.0), v(10.0, 10.0)],
            [mid, v(10.0, 10.0), v(0.0, 10.0)],
        ] {
            if (t[1] - t[0]).cross(&(t[2] - t[0])).x < 0.0 {
                t.swap(1, 2);
            }
            tris.push(t);
        }
        let d = run(&weld(&tris));
        assert!(d.open_planes > 0);
        let island = d.islands.iter().find(|i| !i.on_plate).unwrap();
        assert!((island.area - 100.0).abs() < 1e-2);
    }

    #[test]
    fn hanging_point_is_held_at_its_tip() {
        // An upside-down pyramid: apex at (5, 5, 5), base square at z = 8.
        let apex = Vector3::new(5.0, 5.0, 5.0);
        let c = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]
            .map(|[x, y]| Vector3::new(x, y, 8.0));
        let mut tris = Vec::new();
        for i in 0..4 {
            tris.push([apex, c[(i + 1) % 4], c[i]]);
        }
        tris.push([c[0], c[1], c[2]]);
        tris.push([c[0], c[2], c[3]]);
        let d = run(&weld(&tris));
        let island = d.islands.iter().find(|i| !i.on_plate).unwrap();
        assert!(
            (island.contacts[0].0 - apex).norm() < 1e-4,
            "{:?}",
            island.contacts[0]
        );
        assert!(island.contacts.len() > 1);
    }

    #[test]
    fn flat_underside_starts_inside() {
        let d = run(&weld(&cuboid([0.0, 0.0, 5.0], [10.0, 10.0, 10.0])));
        let (p, n) = d.islands[0].contacts[0];
        assert!(p.x > 0.5 && p.x < 9.5 && p.y > 0.5 && p.y < 9.5, "{p:?}");
        assert!(n.z < -0.99);
    }

    #[test]
    fn overlay_covers_the_island() {
        let d = run(&weld(&cuboid([0.0, 0.0, 5.0], [10.0, 10.0, 10.0])));
        let mesh = super::overlay_mesh(&d.islands, d.spacing);
        assert!(mesh.face_count() > 0);
        let (lo, hi) = mesh
            .vertices()
            .iter()
            .fold((f32::MAX, f32::MIN), |(lo, hi), v| {
                (lo.min(v.x), hi.max(v.x))
            });
        assert!(lo < -0.1 && hi > 10.1);
        assert!(d.islands[0].near(nalgebra::Vector2::new(10.1, 5.0), 0.2));
        assert!(!d.islands[0].near(nalgebra::Vector2::new(10.5, 5.0), 0.2));
    }

    #[test]
    fn contacts_hold_islands() {
        let d = run(&weld(&cuboid([0.0, 0.0, 5.0], [10.0, 10.0, 10.0])));
        let anchor = d.islands[0].anchor;
        let tip = |p: Vector3<f32>| [(p, 0.2)];
        assert_eq!(coverage(&d, &tip(anchor)), [Cover::Supported]);
        // Just outside the edge, within the tip radius plus margin.
        let edge = Vector3::new(10.3, 5.0, 5.0);
        assert_eq!(coverage(&d, &tip(edge)), [Cover::Supported]);
        let far = Vector3::new(11.0, 5.0, 5.0);
        assert_eq!(coverage(&d, &tip(far)), [Cover::Unsupported]);
        let high = anchor + Vector3::z() * 2.0;
        assert_eq!(coverage(&d, &tip(high)), [Cover::Unsupported]);

        let plate = run(&weld(&cuboid([0.0; 3], [10.0; 3])));
        assert_eq!(coverage(&plate, &[]), [Cover::OnPlate]);
    }

    #[test]
    fn islands_follow_moves() {
        use nalgebra::{Isometry3, Translation3, UnitQuaternion};
        let mut d = run(&weld(&cuboid([0.0, 0.0, 5.0], [10.0, 10.0, 10.0])));
        let turn = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f32::consts::FRAC_PI_2);
        d.transform(&Isometry3::from_parts(
            Translation3::new(20.0, 0.0, 0.0),
            turn,
        ));
        let island = &d.islands[0];
        // (x, y) -> (20 - y, x): the square spans x 10..20, y 0..10.
        assert!((island.min.x - 10.0).abs() < 1e-3 && (island.max.x - 20.0).abs() < 1e-3);
        assert!((island.min.y - 0.0).abs() < 1e-3 && (island.max.y - 10.0).abs() < 1e-3);
        assert!((island.anchor.z - 5.0).abs() < 1e-3);
    }
}
