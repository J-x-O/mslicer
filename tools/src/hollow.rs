//! Hollowing: the cavity left inside a model whose walls are kept a set
//! thickness, and drain holes through those walls.
//!
//! Nothing here changes the model's surface. The cavity and the holes are
//! closed volumes that slicing takes out of the model's material, so the
//! outside stays exactly as imported and supports keep sitting on it.
//!
//! The cavity is found on a grid of samples: a sample is inside the model when
//! a ray along x has crossed the surface an unbalanced number of times, its
//! depth is the distance to the nearest sample outside (exact near the wall
//! thickness, from the surface itself), and the cavity is where the depth
//! exceeds the wall, contoured with marching cubes.

use std::{
    collections::HashMap,
    sync::atomic::{AtomicBool, Ordering},
};

use common::progress::Progress;
use nalgebra::Vector3;
use rayon::prelude::*;
use slicer::geometry::{mesh::Mesh, partitioning::Bvh};

use crate::reconstruct_mesh::marching_cubes::{
    EDGE_CONNECTIONS, EDGE_TABLE, GRID_POINTS, TRIANGULATION_TABLE,
};

/// The most grid samples a cavity is computed on; a finer spacing is
/// coarsened to fit.
const MAX_SAMPLES: usize = 40_000_000;
/// Cavity pieces smaller than this (mm³) are left solid: they save next to
/// no resin, would each need a drain, and are mostly sampling noise in thin
/// parts or pockets too tight to drill into.
const MIN_PART_VOLUME: f32 = 10.0;
/// How far a drain hole reaches past the wall into the cavity.
pub const HOLE_OVERSHOOT: f32 = 0.5;
/// How far a drain hole starts outside the surface, so it opens cleanly where
/// the surface curves away.
pub const HOLE_LEAD: f32 = 1.0;
const HOLE_SEGMENTS: usize = 24;

/// The inside of a hollowed model, in the space of the mesh it was computed
/// from (world space).
#[derive(Clone, Debug, Default)]
pub struct Cavity {
    /// A closed surface, faces pointing out of the cavity (into the walls).
    pub vertices: Vec<Vector3<f32>>,
    pub faces: Vec<[u32; 3]>,
    /// Each separate void.
    pub parts: Vec<CavityPart>,
    /// A drain hole for each part, straight up through the floor below its
    /// lowest point.
    pub drains: Vec<Hole>,
    /// The sample spacing used (mm).
    pub cell: f32,
}

/// One void of a cavity.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CavityPart {
    pub volume: f32,
    pub lowest: Vector3<f32>,
    pub min: Vector3<f32>,
    pub max: Vector3<f32>,
}

/// A round hole drilled into a model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hole {
    /// Where the hole's axis meets the surface.
    pub point: Vector3<f32>,
    /// Unit direction of the drill, into the model.
    pub direction: Vector3<f32>,
    pub diameter: f32,
    /// How deep the hole goes, from the surface.
    pub depth: f32,
}

/// How deep a hole must go to reach a cavity behind a wall this thick.
pub fn through_depth(wall: f32) -> f32 {
    wall + HOLE_OVERSHOOT
}

/// The cavity inside `mesh` (as placed: its transform applied) when its walls
/// are kept `wall` mm thick, sampled every `cell` mm (coarser for very large
/// models). Empty if the model is too thin anywhere to leave a void; `None`
/// if `cancel` was set meanwhile.
pub fn cavity(
    mesh: &Mesh,
    wall: f32,
    cell: f32,
    diameter: f32,
    progress: &Progress,
    cancel: &AtomicBool,
) -> Option<Cavity> {
    let cancelled = || cancel.load(Ordering::Relaxed);
    let vertices = mesh.vertices().iter().map(|v| mesh.transform(v)).collect();
    let world = Mesh::new(vertices, mesh.faces().to_vec());
    let bvh = Bvh::build(world.inner(), Progress::new());
    let Some(grid) = Grid::around(world.vertices(), cell) else {
        progress.set_finished();
        return Some(Cavity::default());
    };

    progress.set_total(grid.n[2] as u64 * 2 + 1);
    let inside = inside(&world, &bvh, &grid, progress, cancel);
    if cancelled() {
        return None;
    }
    let field = depth_field(&world, &bvh, &grid, &inside, wall, cancel);
    if cancelled() {
        return None;
    }
    progress.add_complete(1);
    let (vertices, faces) = contour(&grid, &field, progress, cancel);
    if cancelled() {
        return None;
    }

    let mut cavity = into_parts(vertices, faces, grid.cell);
    cavity.cell = grid.cell;
    cavity.drains = drains(&world, &bvh, &grid, &field, diameter);
    progress.set_finished();
    Some(cavity)
}

/// Adds a closed cylinder for `hole` to a mesh: from [`HOLE_LEAD`] outside the
/// surface to the hole's depth, faces pointing out.
pub fn hole_mesh(hole: &Hole, vertices: &mut Vec<Vector3<f32>>, faces: &mut Vec<[u32; 3]>) {
    let Some(d) = hole.direction.try_normalize(f32::EPSILON) else {
        return;
    };
    let side = if d.z.abs() < 0.9 {
        Vector3::z()
    } else {
        Vector3::x()
    };
    let u = d.cross(&side).normalize();
    let v = d.cross(&u);
    let (a, b) = (hole.point - d * HOLE_LEAD, hole.point + d * hole.depth);
    let r = hole.diameter / 2.0;

    let base = vertices.len() as u32;
    let n = HOLE_SEGMENTS as u32;
    for i in 0..HOLE_SEGMENTS {
        let angle = i as f32 / HOLE_SEGMENTS as f32 * std::f32::consts::TAU;
        let off = (u * angle.cos() + v * angle.sin()) * r;
        vertices.extend([a + off, b + off]);
    }
    vertices.extend([a, b]);
    let (ca, cb) = (base + 2 * n, base + 2 * n + 1);
    for i in 0..n {
        let j = (i + 1) % n;
        let (ai, bi, aj, bj) = (
            base + 2 * i,
            base + 2 * i + 1,
            base + 2 * j,
            base + 2 * j + 1,
        );
        faces.extend([[ai, aj, bj], [ai, bj, bi], [ca, aj, ai], [cb, bi, bj]]);
    }
}

/// Vertices and faces of a triangle mesh.
pub type TriangleMesh = (Vec<Vector3<f32>>, Vec<[u32; 3]>);

/// Cuts `holes` into a closed mesh (its faces pointing out): the material
/// within each hole's cylinder is removed. All holes are joined first, so
/// overlapping holes are fine. Fails if the mesh isn't closed and free of
/// self-intersections.
pub fn drill(
    vertices: &[Vector3<f32>],
    faces: &[[u32; 3]],
    holes: &[Hole],
) -> Result<TriangleMesh, String> {
    use boolmesh::prelude::{Manifold, OpType, compute_boolean};

    let manifold = |vertices: &[Vector3<f32>], faces: &[[u32; 3]]| {
        let positions: Vec<f64> = (vertices.iter())
            .flat_map(|v| [v.x as f64, v.y as f64, v.z as f64])
            .collect();
        let indices: Vec<usize> = faces.iter().flatten().map(|&i| i as usize).collect();
        Manifold::new(&positions, &indices)
    };
    let mut cut: Option<Manifold> = None;
    for hole in holes {
        let (mut hv, mut hf) = (Vec::new(), Vec::new());
        hole_mesh(hole, &mut hv, &mut hf);
        let cylinder = manifold(&hv, &hf)?;
        cut = Some(match cut {
            Some(cut) => compute_boolean(&cut, &cylinder, OpType::Add)?,
            None => cylinder,
        });
    }
    let Some(cut) = cut else {
        return Ok((vertices.to_vec(), faces.to_vec()));
    };
    let result = compute_boolean(&manifold(vertices, faces)?, &cut, OpType::Subtract)?;

    let vertices = (result.ps.iter())
        .map(|p| Vector3::new(p.x as f32, p.y as f32, p.z as f32))
        .collect();
    let faces = (result.hs.as_chunks::<3>().0.iter())
        .map(|h| [h[0].tail, h[1].tail, h[2].tail].map(|i| i as u32))
        .collect();
    Ok((vertices, faces))
}

/// How far around a hole's rim [`drill_local`] takes the mesh in (mm).
const LOCAL_MARGIN: f32 = 1.0;
/// How far past the hole's ends along its axis the patch may reach (mm).
const LOCAL_DEPTH: f32 = 5.0;
/// Sides of the prism the mesh is cut along around a hole.
const LOCAL_SIDES: usize = 32;
/// Points this close to a side of the prism are on it (mm).
const ON_SIDE: f64 = 1e-4;

/// Like [`drill`], but each hole only touches the mesh around it. The mesh
/// is cut exactly along a prism about the hole's axis (its faces split where
/// they cross the prism's sides), the pieces inside it where the axis passes
/// through (a disc of the outer wall and one of the cavity's) are joined by
/// the prism's wall into a small closed slab, and only that slab is drilled
/// and put back. The rest of the mesh never goes through the boolean, so it
/// needn't be clean, and the time doesn't grow with the model. Fails for a
/// hole whose surroundings aren't two simple discs (a hole without a cavity
/// behind it, or one too close to an edge of the wall).
pub fn drill_local(
    vertices: &[Vector3<f32>],
    faces: &[[u32; 3]],
    holes: &[Hole],
) -> Result<TriangleMesh, String> {
    let (mesh, results) = drill_each(vertices, faces, holes);
    results.into_iter().collect::<Result<Vec<_>, _>>()?;
    Ok(mesh)
}

/// [`drill_local`], hole by hole: the holes that can be drilled are, and
/// each result says why a hole couldn't be.
pub fn drill_each(
    vertices: &[Vector3<f32>],
    faces: &[[u32; 3]],
    holes: &[Hole],
) -> (TriangleMesh, Vec<Result<(), String>>) {
    let (mut vertices, mut faces) = weld_near(vertices, faces);
    let results = (holes.iter())
        .map(|hole| {
            (vertices, faces) = drill_one(&vertices, &faces, hole)?;
            Ok(())
        })
        .collect();
    ((vertices, faces), results)
}

/// Points closer than this are welded before drilling (mm): meshes are often
/// saved with the corners of neighbouring faces a rounding error apart.
const WELD: f32 = 1e-5;

/// Merges vertices less than [`WELD`] apart, each into the first one seen
/// near it, and drops the faces that collapse.
fn weld_near(vertices: &[Vector3<f32>], faces: &[[u32; 3]]) -> TriangleMesh {
    // Cells much larger than the weld distance: only a point that close to
    // a cell's side looks into the cell beyond.
    const CELL: f32 = WELD * 64.0;
    let mut grid: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
    let mut welded: Vec<Vector3<f32>> = Vec::new();
    let remap: Vec<u32> = (vertices.iter())
        .map(|v| {
            let at = v.map(|x| x / CELL);
            let c = at.map(|x| x.floor() as i64);
            let reach = |k: usize| {
                let frac = at[k] - at[k].floor();
                let low = if frac * CELL < WELD { -1 } else { 0 };
                let high = if (1.0 - frac) * CELL < WELD { 1 } else { 0 };
                low..=high
            };
            let near = (reach(0))
                .flat_map(|x| reach(1).flat_map(move |y| reach(2).map(move |z| (x, y, z))))
                .filter_map(|(x, y, z)| grid.get(&(c.x + x, c.y + y, c.z + z)))
                .flatten()
                .find(|&&j| (welded[j as usize] - v).norm_squared() <= WELD * WELD);
            match near {
                Some(&j) => j,
                None => {
                    welded.push(*v);
                    let id = welded.len() as u32 - 1;
                    grid.entry((c.x, c.y, c.z)).or_default().push(id);
                    id
                }
            }
        })
        .collect();
    let faces = (faces.iter())
        .map(|f| f.map(|i| remap[i as usize]))
        .filter(|[a, b, c]| a != b && b != c && a != c)
        .collect();
    (welded, faces)
}

/// Merges vertices at exactly the same position.
fn weld(vertices: &[Vector3<f32>], faces: &[[u32; 3]]) -> TriangleMesh {
    let mut index = HashMap::new();
    let mut welded = Vec::new();
    let remap: Vec<u32> = (vertices.iter())
        .map(|v| {
            let key = v.map(f32::to_bits);
            *index.entry((key.x, key.y, key.z)).or_insert_with(|| {
                welded.push(*v);
                welded.len() as u32 - 1
            })
        })
        .collect();
    let faces = (faces.iter())
        .map(|f| f.map(|i| remap[i as usize]))
        .filter(|[a, b, c]| a != b && b != c && a != c)
        .collect();
    (welded, faces)
}

/// A prism about a hole's axis: [`LOCAL_SIDES`] planes along the axis, each
/// `radius` from it. Side `k` faces the angle `k · step` about the axis; the
/// corner between sides `k` and `k + 1` is at `(k + ½) · step`.
struct Prism {
    origin: Vector3<f64>,
    u: Vector3<f64>,
    v: Vector3<f64>,
    radius: f64,
    /// Each side's outward normal, as (cos, sin) of its angle.
    sides: Vec<(f64, f64)>,
}

impl Prism {
    fn new(origin: Vector3<f32>, u: Vector3<f32>, v: Vector3<f32>, radius: f32) -> Self {
        let sides = (0..LOCAL_SIDES)
            .map(|k| (k as f64 * Self::step()).sin_cos())
            .map(|(s, c)| (c, s))
            .collect();
        Self {
            origin: origin.cast(),
            u: u.cast(),
            v: v.cast(),
            radius: radius as f64,
            sides,
        }
    }

    fn step() -> f64 {
        std::f64::consts::TAU / LOCAL_SIDES as f64
    }

    /// A point's coordinates across the axis.
    fn flat(&self, p: &Vector3<f32>) -> (f64, f64) {
        let rel = p.cast::<f64>() - self.origin;
        (rel.dot(&self.u), rel.dot(&self.v))
    }

    /// How far a point is outside side `k` (negative inside).
    fn distance(&self, p: &Vector3<f32>, k: usize) -> f64 {
        let (x, y) = self.flat(p);
        let (c, s) = self.sides[k];
        x * c + y * s - self.radius
    }

    /// How far a point is outside the prism (negative inside).
    fn outside(&self, p: &Vector3<f32>) -> f64 {
        (0..LOCAL_SIDES)
            .map(|k| self.distance(p, k))
            .fold(f64::MIN, f64::max)
    }

    /// How far the prism's corners are from the axis.
    fn circumradius(&self) -> f64 {
        self.radius / (Self::step() / 2.0).cos()
    }

    /// Which corner a point is on, if any: where two neighbouring sides meet.
    fn corner(&self, p: &Vector3<f32>) -> Option<usize> {
        let at = (self.angle(p) / Self::step() - 0.5).round();
        let c = (at as i64).rem_euclid(LOCAL_SIDES as i64) as usize;
        let on = |k: usize| self.distance(p, k % LOCAL_SIDES).abs() <= ON_SIDE;
        (on(c) && on(c + 1)).then_some(c)
    }

    /// The angle of a point about the axis.
    fn angle(&self, p: &Vector3<f32>) -> f64 {
        let (x, y) = self.flat(p);
        y.atan2(x)
    }
}

/// What a polygon edge, while a face is clipped, runs along: an edge of the
/// face (its two corners), or a side of the prism.
#[derive(Clone, Copy)]
enum Along {
    Edge(u32, u32),
    Side(usize),
}

/// A point made by clipping: where a mesh edge (lower index first) crosses a
/// side, or where two sides meet inside a face.
#[derive(Hash, PartialEq, Eq)]
enum Cut {
    Edge(u32, u32, usize),
    Corner(usize, usize, usize),
}

/// Splits the faces that cross the prism's sides, so every face is either
/// inside the prism or outside it. Faces that share an edge split it at the
/// same points, so the mesh stays connected.
fn clip(
    vertices: &[Vector3<f32>],
    faces: &[[u32; 3]],
    prism: &Prism,
) -> Result<TriangleMesh, String> {
    let n = LOCAL_SIDES;
    let (outer, inner) = (prism.circumradius() + ON_SIDE, prism.radius - ON_SIDE);
    let flat: Vec<(f64, f64)> = vertices.iter().map(|p| prism.flat(p)).collect();
    let mut out_v = vertices.to_vec();
    let mut out_f = Vec::with_capacity(faces.len());
    let mut cuts: HashMap<Cut, u32> = HashMap::new();
    // For each point made on a mesh edge, that edge.
    let mut on_edge: HashMap<u32, (u32, u32)> = HashMap::new();
    // The faces that reach the prism's sides, each with its part inside them
    // if that isn't all of it or nothing.
    let mut clipped = Vec::new();
    let side_of = |d: f64| match d {
        d if d > ON_SIDE => 1_i8,
        d if d < -ON_SIDE => -1,
        _ => 0,
    };

    for (fi, f) in faces.iter().enumerate() {
        let pts = f.map(|i| flat[i as usize]);
        let (mut lo, mut hi) = ((f64::MAX, f64::MAX), (f64::MIN, f64::MIN));
        for (x, y) in pts {
            (lo.0, lo.1, hi.0, hi.1) = (lo.0.min(x), lo.1.min(y), hi.0.max(x), hi.1.max(y));
        }
        let near = (0.0f64.clamp(lo.0, hi.0), 0.0f64.clamp(lo.1, hi.1));
        let far = near.0 * near.0 + near.1 * near.1 > outer * outer;
        let within = pts.iter().all(|(x, y)| x * x + y * y < inner * inner);
        if far || within {
            out_f.push(*f);
            continue;
        }

        // The part inside the prism, clipped side by side (Sutherland–
        // Hodgman), each vertex with the edge that leaves it.
        let mut poly: Vec<(u32, Along)> = (0..3)
            .map(|k| (f[k], Along::Edge(f[k], f[(k + 1) % 3])))
            .collect();
        for j in 0..n {
            let s: Vec<i8> = (poly.iter())
                .map(|&(p, _)| side_of(prism.distance(&out_v[p as usize], j)))
                .collect();
            if s.iter().all(|&x| x <= 0) {
                continue;
            }
            if s.iter().all(|&x| x >= 0) {
                poly.clear();
                break;
            }
            let m = poly.len();
            let mut next = Vec::with_capacity(m + 1);
            for k in 0..m {
                let (p, along) = poly[k];
                let (q, sq) = (poly[(k + 1) % m].0, s[(k + 1) % m]);
                if s[k] <= 0 {
                    let along = match s[k] == 0 && sq > 0 {
                        true => Along::Side(j),
                        false => along,
                    };
                    next.push((p, along));
                }
                if s[k] * sq >= 0 {
                    continue;
                }
                let x = match along {
                    Along::Edge(a, b) => {
                        let (a, b) = (a.min(b), a.max(b));
                        *cuts.entry(Cut::Edge(a, b, j)).or_insert_with(|| {
                            let (pa, pb) = (out_v[a as usize], out_v[b as usize]);
                            let (da, db) = (prism.distance(&pa, j), prism.distance(&pb, j));
                            let t = da / (da - db);
                            let at = pa.cast::<f64>() + (pb - pa).cast::<f64>() * t;
                            out_v.push(at.cast());
                            let id = out_v.len() as u32 - 1;
                            on_edge.insert(id, (a, b));
                            id
                        })
                    }
                    Along::Side(i) => {
                        let key = Cut::Corner(fi, i.min(j), i.max(j));
                        *cuts.entry(key).or_insert_with(|| {
                            let (pp, pq) = (out_v[p as usize], out_v[q as usize]);
                            let (dp, dq) = (prism.distance(&pp, j), prism.distance(&pq, j));
                            let t = dp / (dp - dq);
                            let at = pp.cast::<f64>() + (pq - pp).cast::<f64>() * t;
                            out_v.push(at.cast());
                            out_v.len() as u32 - 1
                        })
                    }
                };
                let along = if s[k] < 0 { Along::Side(j) } else { along };
                next.push((x, along));
            }
            poly = next;
        }
        let inside: Vec<u32> = poly.iter().map(|&(p, _)| p).collect();
        let whole = inside.len() == 3 && inside.iter().all(|p| f.contains(p));
        clipped.push((*f, (inside.len() >= 3 && !whole).then_some(inside)));
    }

    // The points kept on each mesh edge: a face that isn't split itself
    // still takes them on its edges, so its neighbours meet it there.
    let mut kept: HashMap<(u32, u32), Vec<u32>> = HashMap::new();
    for p in clipped
        .iter()
        .filter_map(|(_, inside)| inside.as_ref())
        .flatten()
    {
        if let Some(&edge) = on_edge.get(p) {
            let points = kept.entry(edge).or_default();
            if !points.contains(p) {
                points.push(*p);
            }
        }
    }

    for (f, inside) in clipped {
        // The face's outline with the points kept on its edges, and the part
        // inside the prism: both convex, one within the other.
        let mut outline = Vec::new();
        for k in 0..3 {
            let (a, b) = (f[k], f[(k + 1) % 3]);
            outline.push(a);
            let mut on = kept.get(&(a.min(b), a.max(b))).cloned().unwrap_or_default();
            let from = out_v[a as usize];
            on.sort_by(|p, q| {
                let (p, q) = (out_v[*p as usize] - from, out_v[*q as usize] - from);
                p.norm_squared().total_cmp(&q.norm_squared())
            });
            outline.extend(on);
        }
        let Some(inside) = inside else {
            match outline.len() {
                3 => out_f.push(f),
                m => {
                    // Points on its edges only: a fan from its middle.
                    let centre = (outline.iter())
                        .map(|&p| out_v[p as usize].cast::<f64>())
                        .sum::<Vector3<f64>>()
                        / m as f64;
                    out_v.push(centre.cast());
                    let mid = out_v.len() as u32 - 1;
                    out_f.extend((0..m).map(|k| [mid, outline[k], outline[(k + 1) % m]]));
                }
            }
            continue;
        };
        let [a, b, c] = f.map(|i| out_v[i as usize].cast::<f64>());
        let Some(normal) = (b - a).cross(&(c - a)).try_normalize(0.0) else {
            return Err("a face around the hole is degenerate".into());
        };
        let e1 = (b - a).normalize();
        let e2 = normal.cross(&e1);
        let centre = inside
            .iter()
            .map(|&p| out_v[p as usize].cast::<f64>())
            .sum::<Vector3<f64>>()
            / inside.len() as f64;
        let plane = |p: u32| {
            let rel = out_v[p as usize].cast::<f64>() - centre;
            (rel.dot(&e1), rel.dot(&e2))
        };
        // A face's height over its longest edge, negative if it's turned
        // over.
        let height = |t: &[u32; 3]| {
            let [p, q, r] = t.map(plane);
            let turn = (q.0 - p.0) * (r.1 - p.1) - (q.1 - p.1) * (r.0 - p.0);
            let longest = [(p, q), (q, r), (r, p)]
                .map(|(x, y)| (y.0 - x.0).hypot(y.1 - x.1))
                .into_iter()
                .fold(0.0, f64::max);
            turn / longest.max(f64::MIN_POSITIVE)
        };

        // The rest of the face, between the two outlines.
        let rest = zip(&inside, &outline, plane).ok_or("couldn't split a face around the hole")?;
        if rest.iter().any(|t| height(t) < -ON_SIDE) {
            return Err("couldn't split a face around the hole".into());
        }
        // The part inside: a fan, or one from its centre where the fan
        // would have flat faces.
        let fan: Vec<[u32; 3]> = (1..inside.len() - 1)
            .map(|k| [inside[0], inside[k], inside[k + 1]])
            .collect();
        let flat = fan.iter().any(|t| height(t) <= ON_SIDE);
        out_f.extend(rest);
        if flat {
            out_v.push(centre.cast());
            let mid = out_v.len() as u32 - 1;
            let m = inside.len();
            out_f.extend((0..m).map(|k| [mid, inside[k], inside[(k + 1) % m]]));
        } else {
            out_f.extend(fan);
        }
    }
    Ok((out_v, out_f))
}

/// Triangles filling the ring between two convex outlines, `inner` within
/// `outer`, both counter-clockwise at `pos` (2D) and possibly sharing points.
/// Faces wind the outlines' way. Each outer point takes the inner edges it
/// can see, then hands over to the next.
fn zip(inner: &[u32], outer: &[u32], pos: impl Fn(u32) -> (f64, f64)) -> Option<Vec<[u32; 3]>> {
    // Whether `r` is clearly right of the line from `p` to `q`: an inner
    // edge along the outer outline is not in view.
    let right = |p: u32, q: u32, r: u32| {
        let (p, q, r) = (pos(p), pos(q), pos(r));
        let turn = (q.0 - p.0) * (r.1 - p.1) - (q.1 - p.1) * (r.0 - p.0);
        turn < -ON_SIDE * (q.0 - p.0).hypot(q.1 - p.1)
    };
    let (n, m) = (inner.len(), outer.len());
    // Start where the outlines meet, or else at the first outer point and
    // the inner point where its view of the inner outline begins.
    let (si, so) = match (0..m).find_map(|k| Some((inner.iter().position(|&p| p == outer[k])?, k)))
    {
        Some(start) => start,
        None => {
            let sees = |k: usize| right(inner[k % n], inner[(k + 1) % n], outer[0]);
            ((0..n).find(|&k| !sees(k + n - 1) && sees(k))?, 0)
        }
    };
    let ci: Vec<u32> = (0..=n).map(|k| inner[(si + k) % n]).collect();
    let co: Vec<u32> = (0..=m).map(|k| outer[(so + k) % m]).collect();
    let mut faces = Vec::new();
    let (mut a, mut b) = (0, 0);
    while a < n || b < m {
        if a < n && b < m && ci[a] == co[b] && ci[a + 1] == co[b + 1] {
            (a, b) = (a + 1, b + 1);
            continue;
        }
        let inner_next = b == m || (a < n && right(ci[a], ci[a + 1], co[b]));
        let face = if inner_next {
            a += 1;
            [ci[a], ci[a - 1], co[b]]
        } else {
            b += 1;
            [co[b - 1], co[b], ci[a]]
        };
        if face[0] != face[1] && face[1] != face[2] && face[0] != face[2] {
            faces.push(face);
        }
    }
    Some(faces)
}

fn drill_one(
    vertices: &[Vector3<f32>],
    faces: &[[u32; 3]],
    hole: &Hole,
) -> Result<TriangleMesh, String> {
    use boolmesh::prelude::{Manifold, OpType, compute_boolean};

    let d = hole.direction.normalize();
    let side = if d.z.abs() < 0.9 {
        Vector3::z()
    } else {
        Vector3::x()
    };
    let u = d.cross(&side).normalize();
    let v = d.cross(&u);
    let prism = Prism::new(hole.point, u, v, hole.diameter / 2.0 + LOCAL_MARGIN);
    let (vertices, faces) = clip(vertices, faces, &prism)?;
    let (vertices, faces) = (&vertices[..], &faces[..]);

    // Generous along the axis: walls and cavities curve away from the hole.
    // Up to just past the hole's end: a thin cavity's ceiling is no business
    // of the drill.
    let (low, high) = (-HOLE_LEAD - LOCAL_DEPTH, hole.depth + LOCAL_MARGIN);
    let inside: Vec<bool> = (vertices.iter())
        .map(|p| {
            let t = (p - hole.point).dot(&d);
            (low..=high).contains(&t) && prism.outside(p) <= ON_SIDE * 10.0
        })
        .collect();
    let candidate = |f: &[u32; 3]| f.iter().all(|&i| inside[i as usize]);

    // The patch: the surfaces the hole's axis passes through on its way in,
    // each taken as far round as the prism.
    let start = hole.point - d * HOLE_LEAD;
    let length = HOLE_LEAD + hole.depth;
    let mut seeds: Vec<usize> = (0..faces.len())
        .filter(|&i| candidate(&faces[i]))
        .filter(|&i| {
            let tri = faces[i].map(|k| vertices[k as usize]);
            crossing(start, d, &tri).is_some_and(|t| (0.0..=length).contains(&t))
        })
        .collect();
    let mut by_edge: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (i, f) in faces.iter().enumerate().filter(|(_, f)| candidate(f)) {
        for k in 0..3 {
            let (a, b) = (f[k], f[(k + 1) % 3]);
            by_edge.entry((a.min(b), a.max(b))).or_default().push(i);
        }
    }
    let mut taken = vec![false; faces.len()];
    seeds.iter().for_each(|&i| taken[i] = true);
    while let Some(i) = seeds.pop() {
        let f = faces[i];
        for k in 0..3 {
            let (a, b) = (f[k], f[(k + 1) % 3]);
            for &j in &by_edge[&(a.min(b), a.max(b))] {
                if !taken[j] {
                    taken[j] = true;
                    seeds.push(j);
                }
            }
        }
    }
    let (mut patch, mut rest) = (Vec::new(), Vec::new());
    for (i, f) in faces.iter().enumerate() {
        if taken[i] {
            patch.push(*f)
        } else {
            rest.push(*f)
        }
    }

    // The patch's boundary: its edges no other patch face runs back along.
    let mut edges = HashMap::new();
    for f in &patch {
        for (a, b) in [(f[0], f[1]), (f[1], f[2]), (f[2], f[0])] {
            edges.insert((a, b), ());
        }
    }
    let mut next = HashMap::new();
    for &(a, b) in edges.keys() {
        if !edges.contains_key(&(b, a)) && next.insert(a, b).is_some() {
            return Err("the mesh around the hole isn't a simple disc".into());
        }
    }
    let mut loops = Vec::new();
    while let Some(&start) = next.keys().next() {
        let mut ring = vec![start];
        let mut at = next.remove(&start).unwrap();
        while at != start {
            ring.push(at);
            at = next
                .remove(&at)
                .ok_or("the hole's surroundings don't close up")?;
        }
        loops.push(ring);
    }
    if loops.len() != 2 {
        return Err(format!(
            "expected the outer wall and the cavity around the hole, found {} rims",
            loops.len()
        ));
    }
    // Both rims must run round the prism's wall: where they don't, the
    // surface turns along the drill within reach of the hole.
    if loops
        .iter()
        .flatten()
        .any(|&i| prism.outside(&vertices[i as usize]) < -ON_SIDE * 10.0)
    {
        return Err("the surface around the hole turns along the drill".into());
    }
    let depth = |ring: &[u32]| {
        ring.iter()
            .map(|&i| (vertices[i as usize] - hole.point).dot(&d))
            .sum::<f32>()
            / ring.len() as f32
    };
    loops.sort_by(|a, b| depth(a).total_cmp(&depth(b)));

    // The rims' edges as the patch's faces run along them; the strip's faces
    // must run the other way along each.
    let rim_edges: std::collections::HashSet<(u32, u32)> = (loops.iter())
        .flat_map(|ring| (0..ring.len()).map(|k| (ring[k], ring[(k + 1) % ring.len()])))
        .collect();
    // Both rims walked the same way round the axis from the prism's first
    // corner, each point placed by how far round it is: corners exactly, the
    // points between by their angle along their side.
    let step = Prism::step();
    let mut corner = vec![None; vertices.len()];
    for &i in loops.iter().flatten() {
        corner[i as usize] = prism.corner(&vertices[i as usize]);
    }
    let walk = |ring: &[u32]| -> Result<(Vec<u32>, Vec<f64>), String> {
        // Where the ring passes each corner: the first of the points on it,
        // if a few are.
        let marks = |ring: &[u32]| -> Vec<(usize, usize)> {
            let mut marks: Vec<(usize, usize)> = Vec::new();
            for (k, &i) in ring.iter().enumerate() {
                if let Some(c) = corner[i as usize]
                    && marks.last().is_none_or(|m| m.1 != c)
                {
                    marks.push((k, c));
                }
            }
            while marks.len() > 1 && marks[0].1 == marks[marks.len() - 1].1 {
                marks.pop();
            }
            marks
        };
        let forward = |marks: &[(usize, usize)]| {
            marks.len() == LOCAL_SIDES
                && (0..LOCAL_SIDES)
                    .all(|k| marks[(k + 1) % LOCAL_SIDES].1 == (marks[k].1 + 1) % LOCAL_SIDES)
        };
        let mut ring = ring.to_vec();
        if !forward(&marks(&ring)) {
            ring.reverse();
            if !forward(&marks(&ring)) {
                return Err("a rim around the hole doesn't go once round it".into());
            }
        }
        let n = ring.len();
        let at = marks(&ring);
        let first = at.iter().find(|m| m.1 == 0).unwrap().0;
        let ordered: Vec<u32> = (0..=n).map(|k| ring[(first + k) % n]).collect();
        let corners: HashMap<usize, usize> = (at.iter())
            .map(|&(k, c)| ((k + n - first) % n, c))
            .collect();
        // The last corner passed.
        let mut last = 0;
        let s = (0..=n)
            .map(|k| match corners.get(&k) {
                _ if k == n => LOCAL_SIDES as f64 * step,
                Some(&c) => {
                    last = c;
                    c as f64 * step
                }
                None => {
                    let from = (last as f64 + 0.5) * step;
                    let along = wrap(prism.angle(&vertices[ordered[k] as usize]) - from);
                    last as f64 * step + along.clamp(0.0, step)
                }
            })
            .collect();
        Ok((ordered, s))
    };
    let (o, so) = walk(&loops[0])?;
    let (i, si) = walk(&loops[1])?;
    // A strip face along rim edge x–y with apex z on the other rim.
    let face = |x: u32, y: u32, z: u32| match rim_edges.contains(&(x, y)) {
        true => [y, x, z],
        false => [x, y, z],
    };
    let mut strip = Vec::new();
    let (mut a, mut b) = (0, 0);
    while a + 1 < o.len() || b + 1 < i.len() {
        let advance_outer = b + 1 >= i.len() || (a + 1 < o.len() && so[a + 1] <= si[b + 1]);
        if advance_outer {
            strip.push(face(o[a], o[a + 1], i[b]));
            a += 1;
        } else {
            strip.push(face(i[b], i[b + 1], o[a]));
            b += 1;
        }
    }

    // The slab: the two discs and the strip between them, the strip tagged.
    let mut used: Vec<u32> = patch.iter().flatten().copied().collect();
    used.sort_unstable();
    used.dedup();
    let local: HashMap<u32, usize> = used.iter().enumerate().map(|(k, &g)| (g, k)).collect();
    let positions: Vec<f64> = (used.iter())
        .flat_map(|&g| {
            let v = vertices[g as usize];
            [v.x as f64, v.y as f64, v.z as f64]
        })
        .collect();
    let slab_faces: Vec<[u32; 3]> = patch.iter().chain(&strip).copied().collect();
    let indices: Vec<usize> = slab_faces.iter().flatten().map(|g| local[g]).collect();
    let tags: Vec<f64> = (0..slab_faces.len())
        .flat_map(|k| [(k >= patch.len()) as u8 as f64; 3])
        .collect();
    let slab = Manifold::new_with_values(&positions, &indices, &tags)?;

    let (mut hv, mut hf) = (Vec::new(), Vec::new());
    hole_mesh(hole, &mut hv, &mut hf);
    let hp: Vec<f64> = hv
        .iter()
        .flat_map(|v| [v.x as f64, v.y as f64, v.z as f64])
        .collect();
    let hi: Vec<usize> = hf.iter().flatten().map(|&x| x as usize).collect();
    let cylinder = Manifold::new_with_values(&hp, &hi, &vec![0.0; hi.len()])?;
    let cut = compute_boolean(&slab, &cylinder, OpType::Subtract)?;

    // Back into the mesh, minus the strip; the rims are where they were, so
    // their vertices weld onto the rest again.
    let mut out_vertices = vertices.to_vec();
    let base = out_vertices.len() as u32;
    out_vertices.extend(
        cut.ps
            .iter()
            .map(|p| Vector3::new(p.x as f32, p.y as f32, p.z as f32)),
    );
    let mut out_faces = rest;
    let mut dropped = 0;
    for (f, h) in cut.hs.as_chunks::<3>().0.iter().enumerate() {
        if cut.vals.get(3 * f).is_some_and(|&t| t > 0.5) {
            dropped += 1;
            continue;
        }
        out_faces.push([h[0].tail, h[1].tail, h[2].tail].map(|x| x as u32 + base));
    }
    // The strip has to come through whole: if the drill cut into it, the
    // cavity is narrower than the hole there and the patch can't go back.
    if dropped != strip.len() {
        return Err(format!(
            "the drill cut the seam ({dropped} of {} seam faces came back)",
            strip.len()
        ));
    }
    Ok(weld(&out_vertices, &out_faces))
}

/// Where a ray from `origin` along unit `dir` crosses a triangle: the
/// distance along it.
fn crossing(origin: Vector3<f32>, dir: Vector3<f32>, [a, b, c]: &[Vector3<f32>; 3]) -> Option<f32> {
    let (e1, e2) = (b - a, c - a);
    let p = dir.cross(&e2);
    let det = e1.dot(&p);
    if det.abs() < 1e-12 {
        return None;
    }
    let s = origin - a;
    let u = s.dot(&p) / det;
    let q = s.cross(&e1);
    let v = dir.dot(&q) / det;
    (u >= 0.0 && v >= 0.0 && u + v <= 1.0).then(|| e2.dot(&q) / det)
}

/// An angle difference brought into −π..π.
fn wrap(a: f64) -> f64 {
    use std::f64::consts::{PI, TAU};
    (a + PI).rem_euclid(TAU) - PI
}

/// Whether a mesh is closed: every edge is shared by exactly two faces, run
/// in opposite directions.
pub fn is_closed(faces: &[[u32; 3]]) -> bool {
    let mut edges = HashMap::new();
    for f in faces {
        for (a, b) in [(f[0], f[1]), (f[1], f[2]), (f[2], f[0])] {
            *edges.entry((a, b)).or_insert(0) += 1;
        }
    }
    edges
        .iter()
        .all(|(&(a, b), &n)| n == 1 && edges.get(&(b, a)) == Some(&1))
}

/// The volume a closed mesh encloses: positive when its faces point out.
pub fn signed_volume(vertices: &[Vector3<f32>], faces: &[[u32; 3]]) -> f32 {
    // In double precision: summed in single, large meshes are off by tenths
    // of a mm³, depending on face order.
    let volume: f64 = (faces.iter())
        .map(|f| {
            let [a, b, c] = f.map(|i| vertices[i as usize].cast::<f64>());
            a.dot(&b.cross(&c)) / 6.0
        })
        .sum();
    volume as f32
}

/// Sample points `origin + (i, j, k) · cell`, with a margin all round.
struct Grid {
    origin: Vector3<f32>,
    cell: f32,
    n: [usize; 3],
}

impl Grid {
    fn around(points: &[Vector3<f32>], cell: f32) -> Option<Self> {
        const MARGIN: usize = 2;
        // Off the round numbers models are often built on, so samples rarely
        // sit exactly on a face.
        const JITTER: [f32; 3] = [0.137, 0.291, 0.443];

        let (min, max) = points.iter().fold(
            (Vector3::repeat(f32::MAX), Vector3::repeat(f32::MIN)),
            |(lo, hi), p| (lo.inf(p), hi.sup(p)),
        );
        if points.is_empty() || cell.is_nan() || cell <= 0.0 {
            return None;
        }
        let mut cell = cell;
        loop {
            let n = (max - min).map(|x| (x / cell).ceil() as usize + 2 * MARGIN + 2);
            if n.x * n.y * n.z <= MAX_SAMPLES {
                let origin = min - Vector3::from(JITTER).map(|j| (MARGIN as f32 + j) * cell);
                return Some(Self {
                    origin,
                    cell,
                    n: [n.x, n.y, n.z],
                });
            }
            cell *= 1.1;
        }
    }

    fn len(&self) -> usize {
        self.n[0] * self.n[1] * self.n[2]
    }

    fn index(&self, i: usize, j: usize, k: usize) -> usize {
        i + self.n[0] * (j + self.n[1] * k)
    }

    fn point(&self, index: usize) -> Vector3<f32> {
        let i = index % self.n[0];
        let j = (index / self.n[0]) % self.n[1];
        let k = index / (self.n[0] * self.n[1]);
        self.origin + Vector3::new(i, j, k).cast::<f32>() * self.cell
    }
}

/// Which samples lie inside the model: along each row, the surface crossings
/// counted with the side they face (nonzero winding).
fn inside(
    world: &Mesh,
    bvh: &Bvh,
    grid: &Grid,
    progress: &Progress,
    cancel: &AtomicBool,
) -> Vec<bool> {
    let [nx, ny, nz] = grid.n;
    (0..ny * nz)
        .into_par_iter()
        .flat_map_iter(|row| {
            let (j, k) = (row % ny, row / ny);
            let inside = match cancel.load(Ordering::Relaxed) {
                true => vec![false; nx],
                false => row_inside(world, bvh, grid, j, k),
            };
            (j + 1 == ny).then(|| progress.add_complete(1));
            debug_assert_eq!(inside.len(), nx);
            inside
        })
        .collect()
}

fn row_inside(world: &Mesh, bvh: &Bvh, grid: &Grid, j: usize, k: usize) -> Vec<bool> {
    let nx = grid.n[0];
    let mut row = vec![false; nx];
    // A ray grazing an edge can count a crossing twice; nudge it and retry
    // if the crossings don't balance.
    for attempt in 0..3 {
        let nudge = attempt as f32 * 1e-3 * grid.cell;
        let y = grid.origin.y + j as f32 * grid.cell + nudge;
        let z = grid.origin.z + k as f32 * grid.cell + nudge * 0.7;
        let start = Vector3::new(grid.origin.x - grid.cell, y, z);
        let crossings: Vec<(f32, i32)> = (bvh.intersect_ray_all(world, start, Vector3::x()))
            .into_iter()
            .filter_map(|hit| {
                let side = world.normal(hit.face).x;
                (side != 0.0).then(|| (hit.position.x, side.signum() as i32))
            })
            .collect();
        let balance: i32 = crossings.iter().map(|c| c.1).sum();
        if balance != 0 && attempt < 2 {
            continue;
        }

        let (mut winding, mut next) = (0, 0);
        for (i, inside) in row.iter_mut().enumerate() {
            let x = grid.origin.x + i as f32 * grid.cell;
            while next < crossings.len() && crossings[next].0 < x {
                winding += crossings[next].1;
                next += 1;
            }
            *inside = winding != 0;
        }
        break;
    }
    row
}

/// For each sample: how far inside the cavity it is, `depth − wall` (positive
/// in the cavity), where depth is the distance to the model's surface.
fn depth_field(
    world: &Mesh,
    bvh: &Bvh,
    grid: &Grid,
    inside: &[bool],
    wall: f32,
    cancel: &AtomicBool,
) -> Vec<f32> {
    let [nx, ny, nz] = grid.n;
    // Squared distance, in samples, to the nearest sample outside.
    let mut squared: Vec<f32> = (inside.iter())
        .map(|&x| if x { f32::INFINITY } else { 0.0 })
        .collect();

    squared.par_chunks_mut(nx).for_each(distance_1d);
    squared.par_chunks_mut(nx * ny).for_each(|slab| {
        let mut line = vec![0.0; ny];
        for i in 0..nx {
            (0..ny).for_each(|j| line[j] = slab[i + nx * j]);
            distance_1d(&mut line);
            (0..ny).for_each(|j| slab[i + nx * j] = line[j]);
        }
    });
    let columns: Vec<Vec<f32>> = (0..nx * ny)
        .into_par_iter()
        .map(|ij| {
            let mut line: Vec<f32> = (0..nz).map(|k| squared[ij + nx * ny * k]).collect();
            distance_1d(&mut line);
            line
        })
        .collect();
    for (ij, line) in columns.into_iter().enumerate() {
        (0..nz).for_each(|k| squared[ij + nx * ny * k] = line[k]);
    }

    let cell = grid.cell;
    (0..grid.len())
        .into_par_iter()
        .map(|index| {
            if !inside[index] || cancel.load(Ordering::Relaxed) {
                return -(wall + cell);
            }
            // The surface lies about half a sample short of the nearest
            // outside sample; near the wall thickness, measure it exactly.
            let mut depth = squared[index].sqrt() * cell - cell / 2.0;
            if (depth - wall).abs() < 1.5 * cell
                && let Some(hit) = bvh.closest(world, grid.point(index))
            {
                depth = hit.t;
            }
            depth - wall
        })
        .collect()
}

/// The squared distance transform of one line, in place: each value becomes
/// the least `(q − p)² + value[p]` over the line (the lower envelope of
/// parabolas). Infinite values are never nearest.
fn distance_1d(line: &mut [f32]) {
    let n = line.len();
    let sources: Vec<usize> = (0..n).filter(|&q| line[q].is_finite()).collect();
    if sources.is_empty() {
        return;
    }
    let f = |q: usize| line[q] + (q * q) as f32;
    // Parabola apexes on the envelope, and where each takes over.
    let mut apex = Vec::with_capacity(sources.len());
    let mut from = Vec::with_capacity(sources.len() + 1);
    for &q in &sources {
        let mut start = f32::NEG_INFINITY;
        while let Some(&p) = apex.last() {
            let s = (f(q) - f(p)) / (2.0 * (q as f32 - p as f32));
            if s <= *from.last().unwrap() {
                apex.pop();
                from.pop();
            } else {
                start = s;
                break;
            }
        }
        apex.push(q);
        from.push(start);
    }

    let values: Vec<f32> = sources.iter().map(|&q| line[q]).collect();
    let value = |p: usize| values[sources.binary_search(&p).unwrap()];
    let mut k = 0;
    let out: Vec<f32> = (0..n)
        .map(|q| {
            while k + 1 < apex.len() && from[k + 1] < q as f32 {
                k += 1;
            }
            let p = apex[k];
            let d = q as f32 - p as f32;
            d * d + value(p)
        })
        .collect();
    line.copy_from_slice(&out);
}

/// The surface where `field` crosses zero, by marching cubes. Vertices on the
/// same grid edge are shared, so the result is closed.
fn contour(
    grid: &Grid,
    field: &[f32],
    progress: &Progress,
    cancel: &AtomicBool,
) -> (Vec<Vector3<f32>>, Vec<[u32; 3]>) {
    let [nx, ny, nz] = grid.n;
    type Slab = (Vec<[u64; 3]>, Vec<(u64, Vector3<f32>)>);
    let slabs: Vec<Slab> = (0..nz - 1)
        .into_par_iter()
        .map(|k| {
            let mut triangles = Vec::new();
            let mut points = Vec::new();
            if cancel.load(Ordering::Relaxed) {
                return (triangles, points);
            }
            for j in 0..ny - 1 {
                for i in 0..nx - 1 {
                    let corner = |c: usize| {
                        let o = GRID_POINTS[c];
                        grid.index(i + o.x as usize, j + o.y as usize, k + o.z as usize)
                    };
                    let corners: [usize; 8] = std::array::from_fn(corner);
                    let mut case = 0;
                    for (c, &index) in corners.iter().enumerate() {
                        case |= ((field[index] < 0.0) as usize) << c;
                    }
                    let edges = EDGE_TABLE[case];
                    if edges == 0 {
                        continue;
                    }

                    let mut keys = [0_u64; 12];
                    for (e, &(a, b)) in EDGE_CONNECTIONS.iter().enumerate() {
                        if edges & (1 << e) == 0 {
                            continue;
                        }
                        let (a, b) = (corners[a], corners[b]);
                        let (lo, hi) = (a.min(b), a.max(b));
                        let axis = match hi - lo {
                            1 => 0,
                            d if d == nx => 1,
                            _ => 2,
                        };
                        let key = lo as u64 * 3 + axis;
                        keys[e] = key;
                        let (pa, pb) = (grid.point(a), grid.point(b));
                        let (va, vb) = (field[a], field[b]);
                        let t = (-va / (vb - va)).clamp(0.0, 1.0);
                        points.push((key, pa + (pb - pa) * t));
                    }
                    for t in TRIANGULATION_TABLE[case].chunks(3) {
                        triangles.push([
                            keys[t[0] as usize],
                            keys[t[2] as usize],
                            keys[t[1] as usize],
                        ]);
                    }
                }
            }
            progress.add_complete(1);
            (triangles, points)
        })
        .collect();

    let mut index = HashMap::new();
    let mut vertices = Vec::new();
    for (_, points) in &slabs {
        for &(key, point) in points {
            index.entry(key).or_insert_with(|| {
                vertices.push(point);
                vertices.len() as u32 - 1
            });
        }
    }
    let faces = (slabs.iter())
        .flat_map(|(triangles, _)| triangles.iter())
        .map(|t| t.map(|key| index[&key]))
        .filter(|[a, b, c]| a != b && b != c && a != c)
        .collect();
    (vertices, faces)
}

/// Splits a contoured cavity into its separate voids, drops the tiny ones,
/// and turns the faces to point out of the cavity.
fn into_parts(vertices: Vec<Vector3<f32>>, mut faces: Vec<[u32; 3]>, cell: f32) -> Cavity {
    if signed_volume(&vertices, &faces) < 0.0 {
        faces.iter_mut().for_each(|f| f.swap(1, 2));
    }

    let mut parent: Vec<u32> = (0..vertices.len() as u32).collect();
    fn root(parent: &mut [u32], mut x: u32) -> u32 {
        while parent[x as usize] != x {
            parent[x as usize] = parent[parent[x as usize] as usize];
            x = parent[x as usize];
        }
        x
    }
    for f in &faces {
        let r = root(&mut parent, f[0]);
        for &v in &f[1..] {
            let s = root(&mut parent, v);
            parent[s as usize] = r;
        }
    }

    let mut groups: HashMap<u32, Vec<usize>> = HashMap::new();
    for (i, f) in faces.iter().enumerate() {
        groups.entry(root(&mut parent, f[0])).or_default().push(i);
    }
    let mut groups: Vec<Vec<usize>> = groups.into_values().collect();
    groups.sort_by_key(|g| g[0]);

    let mut cavity = Cavity::default();
    let mut remap = vec![u32::MAX; vertices.len()];
    for group in groups {
        let part_faces: Vec<[u32; 3]> = group.iter().map(|&i| faces[i]).collect();
        let volume = signed_volume(&vertices, &part_faces);
        if volume < MIN_PART_VOLUME {
            continue;
        }
        let mut part = CavityPart {
            volume,
            lowest: Vector3::repeat(f32::MAX),
            min: Vector3::repeat(f32::MAX),
            max: Vector3::repeat(f32::MIN),
        };
        let first = cavity.vertices.len();
        for face in part_faces {
            cavity.faces.push(face.map(|v| {
                if remap[v as usize] == u32::MAX {
                    let p = vertices[v as usize];
                    remap[v as usize] = cavity.vertices.len() as u32;
                    cavity.vertices.push(p);
                    part.min = part.min.inf(&p);
                    part.max = part.max.sup(&p);
                }
                remap[v as usize]
            }));
        }
        part.lowest = lowest(&cavity.vertices[first..], part.min.z + cell);
        cavity.parts.push(part);
    }
    cavity
}

/// Where to drain a void from: of its points lower than `below`, the one
/// nearest their middle, so a flat floor drains from its centre rather than a
/// corner against a wall.
fn lowest(points: &[Vector3<f32>], below: f32) -> Vector3<f32> {
    let low: Vec<_> = points.iter().filter(|p| p.z <= below).collect();
    let middle = low.iter().fold(Vector3::zeros(), |sum, p| sum + **p) / low.len() as f32;
    let flat = |p: &Vector3<f32>| (p - middle).xy().norm_squared();
    **low
        .iter()
        .min_by(|a, b| flat(a).total_cmp(&flat(b)))
        .unwrap()
}

/// A drain for each void, straight up through the floor. It goes in below
/// the void's lowest point that is at least as wide as the hole and the
/// mesh around it ([`LOCAL_MARGIN`]), so it opens into the void rather than
/// a crack at its bottom; a void that is nowhere that wide drains from its
/// lowest point. The field holds each sample's distance to the cavity's wall.
fn drains(world: &Mesh, bvh: &Bvh, grid: &Grid, field: &[f32], diameter: f32) -> Vec<Hole> {
    let need = diameter / 2.0 + LOCAL_MARGIN;
    let [nx, ny, _] = grid.n;

    // The voids, as groups of face-adjacent cavity samples.
    let mut parent: Vec<u32> = (0..grid.len() as u32).collect();
    fn root(parent: &mut [u32], mut x: u32) -> u32 {
        while parent[x as usize] != x {
            parent[x as usize] = parent[parent[x as usize] as usize];
            x = parent[x as usize];
        }
        x
    }
    for index in (0..grid.len()).filter(|&i| field[i] > 0.0) {
        let (i, j) = (index % nx, (index / nx) % ny);
        let steps = [(i + 1 < nx, 1), (j + 1 < ny, nx), (true, nx * ny)];
        for (ok, step) in steps {
            let other = index + step;
            if ok && other < grid.len() && field[other] > 0.0 {
                let (a, b) = (
                    root(&mut parent, index as u32),
                    root(&mut parent, other as u32),
                );
                parent[a as usize] = b;
            }
        }
    }
    let mut voids: HashMap<u32, Vec<usize>> = HashMap::new();
    for index in (0..grid.len()).filter(|&i| field[i] > 0.0) {
        voids
            .entry(root(&mut parent, index as u32))
            .or_default()
            .push(index);
    }

    let cell_volume = grid.cell.powi(3);
    let mut voids: Vec<Vec<usize>> = voids.into_values().collect();
    voids.sort_by_key(|v| v[0]);
    (voids.into_iter())
        .filter(|samples| samples.len() as f32 * cell_volume >= MIN_PART_VOLUME)
        .filter_map(|samples| {
            let wide: Vec<usize> = samples
                .iter()
                .copied()
                .filter(|&i| field[i] >= need)
                .collect();
            let pool = if wide.is_empty() { samples } else { wide };
            let points: Vec<Vector3<f32>> = pool.iter().map(|&i| grid.point(i)).collect();
            let at = lowest(
                &points,
                points.iter().map(|p| p.z).fold(f32::MAX, f32::min) + grid.cell / 2.0,
            );
            let hit = bvh.intersect_ray(world, at, -Vector3::z())?;
            Some(Hole {
                point: hit.position,
                direction: Vector3::z(),
                diameter,
                depth: at.z - hit.position.z,
            })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use common::progress::Progress;
    use nalgebra::Vector3;
    use slicer::geometry::mesh::{Mesh, MeshBuilder};

    use std::sync::atomic::AtomicBool;

    use super::{
        Cavity, HOLE_LEAD, Hole, distance_1d, drill, drill_local, hole_mesh, is_closed,
        signed_volume,
    };

    fn cavity(mesh: &Mesh, wall: f32, cell: f32, diameter: f32, progress: &Progress) -> Cavity {
        super::cavity(
            mesh,
            wall,
            cell,
            diameter,
            progress,
            &AtomicBool::new(false),
        )
        .unwrap()
    }

    /// A box with corners `a` and `b`, faces out.
    fn cuboid(builder: &mut MeshBuilder, a: Vector3<f32>, b: Vector3<f32>) {
        let mut v = |x: f32, y: f32, z: f32| builder.add_vertex(Vector3::new(x, y, z));
        let c = [
            v(a.x, a.y, a.z),
            v(b.x, a.y, a.z),
            v(b.x, b.y, a.z),
            v(a.x, b.y, a.z),
            v(a.x, a.y, b.z),
            v(b.x, a.y, b.z),
            v(b.x, b.y, b.z),
            v(a.x, b.y, b.z),
        ];
        for [i, j, k] in [
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [1, 2, 6],
            [1, 6, 5],
            [2, 3, 7],
            [2, 7, 6],
            [3, 0, 4],
            [3, 4, 7],
        ] {
            builder.add_face([c[i], c[j], c[k]]);
        }
    }

    fn cube(size: f32) -> Mesh {
        let mut builder = MeshBuilder::new();
        cuboid(&mut builder, Vector3::zeros(), Vector3::repeat(size));
        builder.build()
    }

    #[test]
    fn distance_transform_of_a_line() {
        let inf = f32::INFINITY;
        let mut line = [0.0, inf, inf, inf, 0.0, inf, inf];
        distance_1d(&mut line);
        assert_eq!(line, [0.0, 1.0, 4.0, 1.0, 0.0, 1.0, 4.0]);
    }

    #[test]
    fn cube_cavity_keeps_the_walls() {
        let mesh = cube(20.0);
        let cavity = cavity(&mesh, 2.0, 0.5, 2.0, &Progress::new());
        assert_eq!(cavity.parts.len(), 1);
        let part = cavity.parts[0];
        // A 16 mm cube, rounded a little at its edges by the sampling.
        assert!(
            (part.volume - 4096.0).abs() < 4096.0 * 0.06,
            "{}",
            part.volume
        );
        for axis in 0..3 {
            assert!((part.min[axis] - 2.0).abs() < 0.3, "{:?}", part.min);
            assert!((part.max[axis] - 18.0).abs() < 0.3, "{:?}", part.max);
        }
        let volume = signed_volume(&cavity.vertices, &cavity.faces);
        assert!((volume - part.volume).abs() < 1e-1);

        // One drain, up through the floor into the void.
        assert_eq!(cavity.drains.len(), 1);
        let drain = cavity.drains[0];
        assert!(drain.point.z.abs() < 1e-3, "{:?}", drain.point);
        // Up through the 2 mm floor to where the void is wide enough.
        assert!(drain.depth > 2.0 && drain.depth < 5.0, "{}", drain.depth);
    }

    #[test]
    fn cancelled_run_gives_up() {
        let cancel = AtomicBool::new(true);
        let found = super::cavity(&cube(20.0), 2.0, 0.5, 2.0, &Progress::new(), &cancel);
        assert!(found.is_none());
    }

    #[test]
    fn thin_model_has_no_cavity() {
        let mesh = cube(3.0);
        let cavity = cavity(&mesh, 2.0, 0.5, 2.0, &Progress::new());
        assert!(cavity.parts.is_empty() && cavity.faces.is_empty());
    }

    #[test]
    fn placed_model_cavity_is_in_world_space() {
        let mut mesh = cube(20.0);
        mesh.set_position(Vector3::new(50.0, -30.0, 5.0));
        let cavity = cavity(&mesh, 2.0, 0.5, 2.0, &Progress::new());
        let part = cavity.parts[0];
        let center = (part.min + part.max) / 2.0;
        let expected = mesh.transform(&Vector3::repeat(10.0));
        assert!((center - expected).norm() < 0.3, "{center} vs {expected}");
    }

    /// The cube with its cavity's wall facing into the cavity, as printed.
    fn hollowed(cavity: &Cavity) -> (Vec<Vector3<f32>>, Vec<[u32; 3]>) {
        let outer = cube(20.0);
        let mut vertices = outer.vertices().to_vec();
        let mut faces = outer.faces().to_vec();
        let offset = vertices.len() as u32;
        vertices.extend(&cavity.vertices);
        faces.extend((cavity.faces.iter()).map(|f| [f[0], f[2], f[1]].map(|i| i + offset)));
        (vertices, faces)
    }

    #[test]
    fn drill_opens_the_cavity() {
        let cavity = cavity(&cube(20.0), 2.0, 0.5, 2.0, &Progress::new());
        let (vertices, faces) = hollowed(&cavity);
        assert!(is_closed(&faces));
        let before = signed_volume(&vertices, &faces);

        let (vertices, faces) = drill(&vertices, &faces, &cavity.drains).unwrap();
        assert!(is_closed(&faces));
        // The floor loses a 2 mm hole through its 2 mm, at its centre: a
        // 24-sided prism.
        let removed = before - signed_volume(&vertices, &faces);
        let prism = 12.0 * (std::f32::consts::TAU / 24.0).sin() * 2.0;
        assert!((removed - prism).abs() < 0.2, "{removed} vs {prism}");
    }

    #[test]
    fn local_drill_matches_the_whole_mesh_drill() {
        let cavity = cavity(&cube(20.0), 2.0, 0.5, 2.0, &Progress::new());
        let (vertices, faces) = hollowed(&cavity);
        let before = signed_volume(&vertices, &faces);
        let (whole_v, whole_f) = drill(&vertices, &faces, &cavity.drains).unwrap();
        let (local_v, local_f) = drill_local(&vertices, &faces, &cavity.drains).unwrap();
        assert!(is_closed(&local_f));
        let whole = before - signed_volume(&whole_v, &whole_f);
        let local = before - signed_volume(&local_v, &local_f);
        assert!((whole - local).abs() < 1e-3, "{whole} vs {local}");
    }

    #[test]
    fn clipping_keeps_the_mesh_closed() {
        let cavity = cavity(&cube(20.0), 2.0, 0.5, 2.0, &Progress::new());
        let (vertices, faces) = super::weld(&hollowed(&cavity).0, &hollowed(&cavity).1);
        let before = signed_volume(&vertices, &faces);
        // Tilted and off the sampling grid, and on it (sides along the
        // cavity's edges, corners on them).
        let tilted = Vector3::new(0.2, -0.1, 1.0).normalize();
        for (point, d) in [
            (Vector3::new(9.7, 10.3, 0.0), tilted),
            (Vector3::new(10.0, 10.0, 0.0), Vector3::z()),
        ] {
            let u = d.cross(&Vector3::x()).normalize();
            let prism = super::Prism::new(point, u, d.cross(&u), 2.0);
            let (v, f) = super::clip(&vertices, &faces, &prism).unwrap();
            assert!(is_closed(&f));
            assert!((signed_volume(&v, &f) - before).abs() < 1e-2);
            // A face reaching inside the prism is all inside it.
            for face in &f {
                let d = face.map(|i| prism.outside(&v[i as usize]));
                let across = d.iter().any(|&x| x > 1e-3) && d.iter().any(|&x| x < -1e-3);
                assert!(!across, "{face:?} {d:?}");
            }
        }
    }

    #[test]
    fn hole_is_a_closed_outward_cylinder() {
        let hole = Hole {
            point: Vector3::new(1.0, 2.0, 3.0),
            direction: Vector3::new(0.3, -0.2, 1.0).normalize(),
            diameter: 2.0,
            depth: 4.0,
        };
        let (mut vertices, mut faces) = (Vec::new(), Vec::new());
        hole_mesh(&hole, &mut vertices, &mut faces);
        let volume = signed_volume(&vertices, &faces);
        // A 24-sided prism: a little under the round cylinder.
        let round = std::f32::consts::PI * (HOLE_LEAD + hole.depth);
        assert!(
            volume > round * 0.97 && volume < round,
            "{volume} vs {round}"
        );
    }
}
