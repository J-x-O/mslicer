//! Brace network.
//!
//! Once every support has been routed to a vertical pillar, neighbouring
//! pillars are tied together with 45° braces so tall, sparsely supported
//! pillars don't stand alone:
//!
//! * inverted-V forks along *chains* of nearly collinear pillars: chains follow
//!   the Delaunay edges (nearest neighbour within 9 × Ø, turning at most 45°,
//!   each edge used once) and only chains of 4 or more pillars fork; a forked
//!   middle stops at its fork point and is carried by its two neighbours;
//! * a Delaunay + minimum spanning tree over the remaining (unforked) pillars,
//!   every tree edge up to 30 mm braced with
//!   45° diagonals that fill the height between the brace floor and the lower
//!   pillar top without overlapping, crossed into an X when the pillars are more
//!   than 5 × Ø apart;
//! * model-landing pillars get the same bracing, but only along edges shorter
//!   than 1.3 × R.
//!
//! Every brace and fork link is link-tested at the part radius plus the model
//! clearance, and a tree edge is skipped when a brace this pass already made
//! leaves either pillar within 20° of it.
//!
//! The pillar pre-merge ([`merge`](super::merge)) is not part of this; it is
//! off by default.

use std::collections::HashSet;

use nalgebra::{Vector2, Vector3};
use slicer::geometry::{mesh::Mesh, partitioning::Bvh};
use spade::{DelaunayTriangulation, Point2, Triangulation};

use crate::supports::auto::collision;
use crate::supports::{Part, PartKind, Shape};

/// Maximum XY spacing between two pillars that are still braced together.
const MAX_CROSS_SPACING_XY: f32 = 30.0;

/// How far below the lower pillar top the braces may start (0.15 mm).
const FIRST_BRACE_DROP: f32 = 0.15;

/// Minimum vertical gap between braces down a pillar pair (2 mm).
const MIN_BRACE_STEP_Z: f32 = 2.0;

/// The other term in the brace gap formula: `13.5 ×` the brace Ø, which is the
/// pillar Ø.
const BRACE_STEP_DIAMETER_FACTOR: f32 = 13.5;

/// Pillars further apart than this many pillar Øs (Manhattan XY distance) get
/// a mirrored second diagonal at every brace level, making an X.
const CROSS_BRACE_DIAMETERS: f32 = 5.0;

/// Braces stay this far above the pillar feet (3 mm),
/// on top of the base and boss for plate pillars.
pub const CONN_MIN_HEIGHT: f32 = 3.0;

/// Clearance added under every brace floor (0.1 mm above the foot).
const FLOOR_EXTRA: f32 = 0.1;

/// A pillar added after generation is braced to at most this many neighbours,
/// trying at most [`MAX_NEW_TRIES`] of the nearest.
const MAX_NEW_LINKS: usize = 2;
const MAX_NEW_TRIES: usize = 8;

/// The brace floor of a plate pillar with the Middle preset's 1 mm base and
/// 1 mm boss and no raft: `0.1 + base + boss + CONN_MIN_HEIGHT`.
pub const DEFAULT_PLATE_FLOOR: f32 = FLOOR_EXTRA + 1.0 + 1.0 + CONN_MIN_HEIGHT;

/// The model clearance (0.6 mm): braces and fork links are collision tested at
/// the part radius plus this.
pub const DEFAULT_CLEARANCE: f32 = 0.6;

/// Fork chain step limit, in pillar Øs (9 × Ø).
const CHAIN_STEP_DIAMETERS: f32 = 9.0;

/// Shortest chain that forks.
const MIN_CHAIN: usize = 4;

/// Inverted-V fork gap limit, as a multiple of the pillar Ø (each gap ≤ 4 ×
/// pillar Ø).
const FORK_GAP_DIAMETERS: f32 = 4.0;

/// How far below the pillar top the fork point sits, in pillar Øs.
const FORK_DROP_DIAMETERS: f32 = 2.0;

/// The cone-length term of the fork drop (`0.3 × cone length`).
const FORK_DROP_CONE_FACTOR: f32 = 0.3;

/// The most the tip end may sit above the fork point, as a multiple of
/// `cone length + pillar Ø` (`2.5 × (cone length + pillar Ø)`).
const FORK_TIP_MAX_DIAMETERS: f32 = 2.5;

/// `sin(20°)`: a new brace is dropped when one already leaves either pillar
/// within 20° of its direction.
const SIN_20: f32 = 0.342;

/// The landing-pillar brace edge limit, as a multiple of R = 0.5 × point
/// spacing (1.3 × R).
pub const LANDING_EDGE_R_FACTOR: f32 = 1.3;

/// `cos(45°)`: a chain turns at most 45° from one step to the next.
const COS_45: f32 = std::f32::consts::FRAC_1_SQRT_2;

/// A pillar as seen by the brace network: its XY position, the height of its
/// top (where the tip/link ends and the vertical pillar begins) and of its tip
/// end, its radius, and the band braces may use.
#[derive(Clone, Copy, Debug)]
pub struct Pillar {
    pub xy: Vector2<f32>,
    pub top_z: f32,
    /// The tip end (horn end) height. Equal to `top_z` for a straight drop, and
    /// higher when a 45° link runs from the tip end down to the pillar top.
    pub tip_z: f32,
    /// The pillar radius, in millimetres. Braces are as thick as the pillars
    /// they join (`0.8`/`1.2`/`1.5` Ø).
    pub radius: f32,
    /// The lowest height a brace may reach on this pillar.
    pub floor_z: f32,
    /// Model clearance added to `radius` for the brace collision test.
    pub clearance: f32,
}

impl Pillar {
    /// A pillar standing on the plate, with the default (Middle, no raft)
    /// brace floor. See [`Pillar::with_floor`].
    pub fn plate(xy: Vector2<f32>, top_z: f32, tip_z: f32, radius: f32) -> Self {
        Self {
            xy,
            top_z,
            tip_z,
            radius,
            floor_z: DEFAULT_PLATE_FLOOR,
            clearance: DEFAULT_CLEARANCE,
        }
    }

    /// A model-landing pillar from its tip end `top_z` down to `foot_z`.
    /// Braces stay `0.1 + supportconnminheight` above the foot.
    pub fn landing(xy: Vector2<f32>, top_z: f32, foot_z: f32, radius: f32) -> Self {
        Self {
            xy,
            top_z,
            tip_z: top_z,
            radius,
            floor_z: foot_z + FLOOR_EXTRA + CONN_MIN_HEIGHT,
            clearance: DEFAULT_CLEARANCE,
        }
    }

    /// The same pillar with brace floor `floor_z`.
    pub fn with_floor(self, floor_z: f32) -> Self {
        Self { floor_z, ..self }
    }

    /// The same pillar with model clearance `clearance` for brace tests.
    pub fn with_clearance(self, clearance: f32) -> Self {
        Self { clearance, ..self }
    }
}

/// The brace floor of a plate pillar standing at z = 0: `0.1` + the raft
/// thickness (with a raft) or the base thickness (with a base) + the boss
/// height + [`CONN_MIN_HEIGHT`].
pub fn plate_floor(base_height: f32, boss_height: f32) -> f32 {
    FLOOR_EXTRA + base_height + boss_height + CONN_MIN_HEIGHT
}

/// The complete brace network: 45° spanning-tree braces plus inverted-V fork
/// links, and the pillar truncations the forks impose.
pub struct BraceNetwork {
    pub braces: Vec<Part>,
    /// `(support_index, bottom_z)`: the vertical pillar of support
    /// `support_index` must end at `bottom_z` (a fork point) instead of
    /// reaching the plate.
    pub fork_bottoms: Vec<(usize, f32)>,
    /// Stabilising legs as routed `[contact, tip, joint]` plate
    /// triplets. Their braces back to the pillars are in `braces`.
    pub legs: Vec<[Vector3<f32>; 3]>,
}

/// Builds the set of brace parts for a set of pillars.
///
/// The Delaunay / MST is computed over the XY positions, keeping only edges no
/// longer than `max_spacing` along which at least one brace fits; each kept
/// edge is braced by [`brace_pair`]. An edge is skipped when a brace already
/// leaves either of its pillars within 20° of the edge's direction. Used for model-landing pillars and stabilising-leg tests.
pub fn build_braces(pillars: &[Pillar], max_spacing: f32, mesh: &Mesh, bvh: &Bvh) -> Vec<Part> {
    tree_braces(pillars, max_spacing, true, mesh, bvh)
}

/// The spanning-tree braces over `pillars`. `braceable_only` drops edges that
/// can't hold a single brace before building the tree (see
/// [`spanning_tree_edges`]).
fn tree_braces(
    pillars: &[Pillar],
    max_spacing: f32,
    braceable_only: bool,
    mesh: &Mesh,
    bvh: &Bvh,
) -> Vec<Part> {
    if pillars.len() < 2 {
        return Vec::new();
    }

    let mut leaving: Vec<Vec<Vector2<f32>>> = vec![Vec::new(); pillars.len()];
    let mut braces = Vec::new();
    for (i, j) in spanning_tree_edges(pillars, max_spacing, braceable_only) {
        let Some(dir) = (pillars[j].xy - pillars[i].xy).try_normalize(f32::EPSILON) else {
            continue;
        };
        if near_parallel(&leaving[i], dir) || near_parallel(&leaving[j], -dir) {
            continue;
        }

        if let Some(parts) = brace_pair(pillars[i], pillars[j], mesh, bvh) {
            leaving[i].push(dir);
            leaving[j].push(-dir);
            braces.extend(parts);
        }
    }
    braces
}

/// Braces a pillar added after generation into the existing pillars: to the
/// nearest ones within the cross-brace spacing, at most [`MAX_NEW_LINKS`] of
/// them and in clearly different directions, each as [`brace_pair`] would.
pub fn brace_into(pillar: Pillar, others: &[Pillar], mesh: &Mesh, bvh: &Bvh) -> Vec<Part> {
    let distance = |p: &Pillar| (p.xy - pillar.xy).magnitude();
    let mut near: Vec<&Pillar> = (others.iter())
        .filter(|p| distance(p) <= MAX_CROSS_SPACING_XY)
        .collect();
    near.sort_by(|a, b| distance(a).total_cmp(&distance(b)));

    let mut leaving = Vec::new();
    let mut parts = Vec::new();
    for other in near.into_iter().take(MAX_NEW_TRIES) {
        if leaving.len() >= MAX_NEW_LINKS {
            break;
        }
        let Some(dir) = (other.xy - pillar.xy).try_normalize(f32::EPSILON) else {
            continue;
        };
        if near_parallel(&leaving, dir) {
            continue;
        }
        if let Some(braces) = brace_pair(pillar, *other, mesh, bvh) {
            leaving.push(dir);
            parts.extend(braces);
        }
    }
    parts
}

/// Whether any of the brace directions `existing` leaves within 20° of `dir`
/// (dot > 0 and |cross| < sin 20°). All are unit vectors.
fn near_parallel(existing: &[Vector2<f32>], dir: Vector2<f32>) -> bool {
    existing
        .iter()
        .any(|u| u.dot(&dir) > 0.0 && u.perp(&dir).abs() < SIN_20)
}

/// Builds the full plate brace network over `pillars`: the chain forks, then
/// the spanning-tree braces over the pillars that were not forked.
///
/// `cone_length` is the tip cone length, used to place the
/// fork point; `fork_floor` is the lowest allowed fork point (see
/// [`fork_floor`]). `mesh`/`bvh` are the model, used to link-test everything.
pub fn build_network(
    pillars: &[Pillar],
    mesh: &Mesh,
    bvh: &Bvh,
    cone_length: f32,
    fork_floor: f32,
) -> BraceNetwork {
    let mut braced = pillars.to_vec();
    let forks = build_forks(&mut braced, cone_length, fork_floor, mesh, bvh);

    // Forked middles leave the tree; their neighbours keep the brace tops the
    // forks lowered.
    let forked: HashSet<usize> = forks.bottoms.iter().map(|&(i, _)| i).collect();
    let tree: Vec<Pillar> = (0..braced.len())
        .filter(|i| !forked.contains(i))
        .map(|i| braced[i])
        .collect();
    let mut braces = tree_braces(&tree, MAX_CROSS_SPACING_XY, false, mesh, bvh);
    braces.extend(forks.parts);
    BraceNetwork {
        braces,
        fork_bottoms: forks.bottoms,
        legs: Vec::new(),
    }
}

/// The lowest allowed fork point for pillar diameter `diameter`: `pillar Ø +
/// 0.1 + base or raft thickness + supportconnminheight` above the plate (5.3 mm
/// for the Middle preset without a raft).
pub fn fork_floor(diameter: f32, base_height: f32) -> f32 {
    diameter + FLOOR_EXTRA + base_height + CONN_MIN_HEIGHT
}

/// The output of [`build_forks`]: fork links and joints, and the truncation of
/// each forked middle pillar.
struct Forks {
    parts: Vec<Part>,
    bottoms: Vec<(usize, f32)>,
}

/// Inverted-V forks along pillar chains.
///
/// Chains run along the Delaunay edges: from an unused edge (i, j) the chain
/// grows backward from i and then forward from j, each step to the nearest
/// Delaunay neighbour within 9 × Ø whose direction turns at most 45°, stopping
/// at an edge another chain already took. Chains of 4 or more pillars are
/// forked longest first, from the far end: the middle of each triple stops at
/// its fork point and leaves the chain, and the walk moves two pillars on, so
/// neighbouring forks share an anchor.
///
/// Updates `pillars` in place: a forked middle's top becomes the fork point,
/// and each anchor's top drops to where its fork link lands, so later braces
/// stay below the links.
fn build_forks(
    pillars: &mut [Pillar],
    cone_length: f32,
    floor: f32,
    mesh: &Mesh,
    bvh: &Bvh,
) -> Forks {
    let mut forks = Forks {
        parts: Vec::new(),
        bottoms: Vec::new(),
    };
    if pillars.len() < 3 {
        return forks;
    }

    let neighbours = delaunay_neighbours(pillars);
    let mut chains = chains(pillars, &neighbours);
    // Longest first; equal lengths keep their discovery order.
    chains.sort_by_key(|chain| std::cmp::Reverse(chain.len()));

    let mut forked = vec![false; pillars.len()];
    for mut chain in chains {
        let mut k = chain.len() as isize - 2;
        while k >= 1 {
            let (a, b, c) = (
                chain[k as usize - 1],
                chain[k as usize],
                chain[k as usize + 1],
            );
            if forked[c] || forked[b] {
                k -= 2;
                continue;
            }
            if forked[a] {
                k -= 3;
                continue;
            }
            if let Some(fork) = fork_at(pillars, [a, b, c], cone_length, floor, mesh, bvh) {
                forked[b] = true;
                pillars[b].top_z = fork.z;
                pillars[a].top_z = pillars[a].top_z.min(fork.ends[0]);
                pillars[c].top_z = pillars[c].top_z.min(fork.ends[1]);
                forks.parts.extend(fork.parts);
                forks.bottoms.push((b, fork.z));
                chain.remove(k as usize);
            }
            k -= 2;
        }
    }
    forks
}

/// Each pillar's Delaunay neighbours (indices), from the XY triangulation.
fn delaunay_neighbours(pillars: &[Pillar]) -> Vec<Vec<usize>> {
    let mut neighbours = vec![Vec::new(); pillars.len()];
    let points: Vec<Point2<f32>> = pillars
        .iter()
        .map(|p| Point2::new(p.xy.x, p.xy.y))
        .collect();
    let Ok(tri) = DelaunayTriangulation::<Point2<f32>>::bulk_load_stable(points.clone()) else {
        return neighbours;
    };
    for edge in tri.undirected_edges() {
        let [a, b] = edge.positions();
        if let (Some(a), Some(b)) = (point_index(&points, a), point_index(&points, b))
            && a != b
        {
            neighbours[a].push(b);
            neighbours[b].push(a);
        }
    }
    neighbours
}

/// The chains of nearly collinear pillars (see [`build_forks`]), each at least
/// [`MIN_CHAIN`] long, in discovery order.
fn chains(pillars: &[Pillar], neighbours: &[Vec<usize>]) -> Vec<Vec<usize>> {
    let edge = |a: usize, b: usize| (a.min(b), a.max(b));
    let mut used: HashSet<(usize, usize)> = HashSet::new();
    let mut out = Vec::new();

    for i in 0..pillars.len() {
        for &j in &neighbours[i] {
            if used.contains(&edge(i, j)) {
                continue;
            }
            used.insert(edge(i, j));
            let Some(dir) = (pillars[j].xy - pillars[i].xy).try_normalize(f32::EPSILON) else {
                continue;
            };
            let step = CHAIN_STEP_DIAMETERS * 2.0 * pillars[i].radius;

            let mut chain = std::collections::VecDeque::from([i, j]);
            // Backward from i, heading away from j; then forward from j.
            for (start, prev, heading, front) in [(i, j, -dir, true), (j, i, dir, false)] {
                let (mut current, mut prev, mut heading) = (start, prev, heading);
                loop {
                    let next = (neighbours[current].iter().copied())
                        .filter(|&n| n != prev)
                        .filter_map(|n| {
                            let offset = pillars[n].xy - pillars[current].xy;
                            let dist = offset.magnitude();
                            (dist < step && dist > f32::EPSILON).then(|| (n, dist, offset / dist))
                        })
                        .filter(|&(_, _, d)| heading.dot(&d) > COS_45)
                        .min_by(|a, b| a.1.total_cmp(&b.1));
                    let Some((n, _, d)) = next else {
                        break;
                    };
                    if !used.insert(edge(current, n)) {
                        break;
                    }
                    if front {
                        chain.push_front(n);
                    } else {
                        chain.push_back(n);
                    }
                    (prev, current, heading) = (current, n, d);
                }
            }

            if chain.len() >= MIN_CHAIN {
                out.push(chain.into_iter().collect());
            }
        }
    }
    out
}

/// A placed fork: its point's height, where its two links land on the
/// neighbours, and its parts (two links and a joint).
struct Fork {
    z: f32,
    ends: [f32; 2],
    parts: Vec<Part>,
}

/// Tries to fork the middle pillar `b` onto its chain neighbours `a` and `c`.
fn fork_at(
    pillars: &[Pillar],
    [a, b, c]: [usize; 3],
    cone_length: f32,
    floor: f32,
    mesh: &Mesh,
    bvh: &Bvh,
) -> Option<Fork> {
    let middle = pillars[b];
    let diameter = 2.0 * middle.radius;

    // The fork point sits on B's pillar, `2 × Ø + 0.3 × cone length` below its
    // top; it must stay above the floor, and B's tip end may sit at most
    // `2.5 × (cone length + Ø)` above it.
    let fork_z =
        middle.top_z - (FORK_DROP_DIAMETERS * diameter + FORK_DROP_CONE_FACTOR * cone_length);
    if fork_z < floor || middle.tip_z - fork_z > FORK_TIP_MAX_DIAMETERS * (cone_length + diameter) {
        return None;
    }

    let from = middle.xy.push(fork_z);
    let mut parts = Vec::with_capacity(3);
    let mut ends = [0.0; 2];
    for (k, end) in [a, c].into_iter().enumerate() {
        let anchor = pillars[end];
        let run = (anchor.xy - middle.xy).magnitude();
        if run <= f32::EPSILON || run > FORK_GAP_DIAMETERS * diameter {
            return None;
        }
        // The link lands below the neighbour's tip end and above the floor.
        // Also keep it on the pillar itself (below its top) so it never ends
        // beside a sideways link.
        let to_z = fork_z - run;
        if to_z >= anchor.tip_z || to_z > anchor.top_z || to_z <= floor {
            return None;
        }
        let to = anchor.xy.push(to_z);
        let radius = middle.radius.min(anchor.radius);
        if collision::link_test(mesh, bvh, from, to, radius + middle.clearance) {
            return None;
        }
        ends[k] = to_z;
        parts.push(round(PartKind::Brace, from, to, radius));
    }
    parts.push(round(PartKind::Joint, from, from, middle.radius));

    Some(Fork {
        z: fork_z,
        ends,
        parts,
    })
}

fn round(kind: PartKind, a: Vector3<f32>, b: Vector3<f32>, radius: f32) -> Part {
    Part {
        kind,
        a,
        b,
        radius_a: radius,
        radius_b: radius,
        shape: Shape::Round,
    }
}

/// Delaunay triangulation over pillar XY positions, reduced to a minimum
/// spanning forest over the edges no longer than `max_spacing`. Returns
/// `(a, b)` index pairs into `pillars`.
///
/// With `braceable_only` an edge must also hold at least one 45° brace (without
/// it, an edge without room just gets no brace). It is used for model-landing
/// pillars, where a short pillar high above two tall ones would otherwise cut
/// them off from each other.
fn spanning_tree_edges(
    pillars: &[Pillar],
    max_spacing: f32,
    braceable_only: bool,
) -> Vec<(usize, usize)> {
    let points: Vec<Point2<f32>> = pillars
        .iter()
        .map(|p| Point2::new(p.xy.x, p.xy.y))
        .collect();

    // Bulk-load the Delaunay. Duplicate XY positions collapse to the lowest
    // index; two co-located pillars share one vertex and never brace apart.
    let tri = match DelaunayTriangulation::<Point2<f32>>::bulk_load_stable(points.clone()) {
        Ok(t) => t,
        Err(_) => return Vec::new(),
    };

    // Collect Delaunay edges as (u, v, weight) with weight = squared distance.
    let mut edges = Vec::new();
    for edge in tri.undirected_edges() {
        let [a, b] = edge.positions();
        let w = (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
        edges.push((a, b, w));
    }

    // Kruskal MST over the Delaunay edges.
    edges.sort_by(|x, y| x.2.total_cmp(&y.2));
    let n = points.len();
    let mut parent: Vec<usize> = (0..n).collect();
    let mut rank = vec![0usize; n];

    fn find(parent: &mut [usize], x: usize) -> usize {
        let mut root = x;
        while parent[root] != root {
            root = parent[root];
        }
        // Path halving.
        let mut cur = x;
        while parent[cur] != cur {
            let next = parent[cur];
            parent[cur] = root;
            cur = next;
        }
        root
    }

    let mut kept = Vec::new();
    for (a, b, _w) in edges {
        let (Some(ia), Some(ib)) = (point_index(&points, a), point_index(&points, b)) else {
            continue;
        };
        let (pa, pb) = (pillars[ia], pillars[ib]);
        if (pa.xy - pb.xy).magnitude() > max_spacing || (braceable_only && !can_brace(pa, pb)) {
            continue;
        }

        let ra = find(&mut parent, ia);
        let rb = find(&mut parent, ib);
        if ra == rb {
            continue;
        }
        if rank[ra] < rank[rb] {
            parent[ra] = rb;
        } else if rank[ra] > rank[rb] {
            parent[rb] = ra;
        } else {
            parent[rb] = ra;
            rank[ra] += 1;
        }

        kept.push((ia, ib));

        if kept.len() == n - 1 {
            break;
        }
    }

    kept
}

/// Whether a full 45° brace fits between two pillars: starting just below the
/// lower top, it must end above both floors. Collisions aren't
/// considered here; `brace_pair` link-tests each brace.
fn can_brace(a: Pillar, b: Pillar) -> bool {
    let run = (a.xy - b.xy).magnitude();
    run > f32::EPSILON && a.top_z.min(b.top_z) - FIRST_BRACE_DROP - run > a.floor_z.max(b.floor_z)
}

/// Maps a `Point2` back to its index in `points` by exact bit-for-bit match.
/// Duplicate positions collapsed by the Delaunay map to the same (first) index.
fn point_index(points: &[Point2<f32>], p: Point2<f32>) -> Option<usize> {
    points.iter().position(|q| q.x == p.x && q.y == p.y)
}

/// Braces a pillar pair with 45° diagonals. Each brace is link-tested at the part radius
/// plus the model clearance, and a failing one is skipped. Returns `None` when
/// the pair is too close or too short to brace, or every brace fails.
///
/// The braces fill the height between the floor and `top = lower top − 0.15`
/// without overlapping: each drops `run` (45°), and the count is how many fit
/// with at least `gap = max(2 mm, 13.5 × Ø − 2 × run)` between them, plus one
/// more when over `run` is left below the last. They are then spread evenly, the
/// lowest ending on the floor and the highest starting at `top`. A single brace
/// sits on the floor. Sides alternate from the top down, starting on the higher
/// pillar. Pillars more than 5 × Ø apart (Manhattan) also get the mirrored
/// diagonal at every level, making an X.
pub(super) fn brace_pair(a: Pillar, b: Pillar, mesh: &Mesh, bvh: &Bvh) -> Option<Vec<Part>> {
    let higher = if a.top_z >= b.top_z { a } else { b };
    let lower = if a.top_z >= b.top_z { b } else { a };

    // The diagonal drops exactly as much as it travels sideways: 45°.
    let run = (higher.xy - lower.xy).magnitude();
    if run <= f32::EPSILON {
        return None;
    }

    let radius = higher.radius.min(lower.radius);
    let test_radius = radius + higher.clearance.max(lower.clearance);
    let gap = MIN_BRACE_STEP_Z.max(BRACE_STEP_DIAMETER_FACTOR * (2.0 * radius) - 2.0 * run);
    let top = lower.top_z - FIRST_BRACE_DROP;
    let floor = lower.floor_z.max(higher.floor_z);
    let offset = higher.xy - lower.xy;
    let crossed = offset.x.abs() + offset.y.abs() > CROSS_BRACE_DIAMETERS * 2.0 * radius;

    let mut parts = Vec::new();
    for (k, low) in brace_heights(top, floor, run, gap).into_iter().enumerate() {
        let (from, to) = if k % 2 == 0 {
            (higher.xy, lower.xy)
        } else {
            (lower.xy, higher.xy)
        };
        let diagonals = [
            (from.push(low + run), to.push(low)),
            (from.push(low), to.push(low + run)),
        ];
        for (start, end) in diagonals.into_iter().take(if crossed { 2 } else { 1 }) {
            if !collision::link_test(mesh, bvh, start, end, test_radius) {
                parts.push(round(PartKind::Brace, start, end, radius));
            }
        }
    }

    (!parts.is_empty()).then_some(parts)
}

/// The lower-end heights of the braces between `floor` and `top`, highest
/// first. Each brace spans `run` vertically; see [`brace_pair`].
fn brace_heights(top: f32, floor: f32, run: f32, gap: f32) -> Vec<f32> {
    if top - run <= floor {
        return Vec::new();
    }

    // How many fit with at least `gap` between them, top down.
    let mut count = 0usize;
    let mut bottom = top - run;
    let mut last_bottom;
    loop {
        last_bottom = bottom;
        count += 1;
        bottom -= gap + run;
        if bottom <= floor {
            break;
        }
    }
    if last_bottom - floor > run {
        count += 1;
    }

    // Spread evenly from the floor up to `top`.
    let spacing = if count > 1 {
        ((top - floor) - count as f32 * run) / (count - 1) as f32
    } else {
        0.0
    };
    (0..count)
        .rev()
        .map(|i| floor + i as f32 * (run + spacing))
        .collect()
}

#[cfg(test)]
mod tests {
    use common::progress::Progress;
    use nalgebra::{Vector2, Vector3};
    use slicer::geometry::{mesh::Mesh, partitioning::Bvh};

    use super::*;

    /// Middle preset: Ø 1.2, cone 2 mm, no raft, 1 mm base.
    const FORK_FLOOR: f32 = 1.2 + 0.1 + 1.0 + CONN_MIN_HEIGHT;

    fn pillar(x: f32, y: f32, z: f32) -> Pillar {
        Pillar::plate(Vector2::new(x, y), z, z, 0.6)
    }

    fn row(xs: &[f32], z: f32) -> Vec<Pillar> {
        xs.iter().map(|&x| pillar(x, 0.0, z)).collect()
    }

    /// Plate bracing over an empty scene, with the default 30 mm span.
    fn plate_braces(pillars: &[Pillar]) -> Vec<Part> {
        let (mesh, bvh) = empty_scene();
        tree_braces(pillars, MAX_CROSS_SPACING_XY, false, &mesh, &bvh)
    }

    fn forks(pillars: &[Pillar]) -> Forks {
        let (mesh, bvh) = empty_scene();
        build_forks(&mut pillars.to_vec(), 2.0, FORK_FLOOR, &mesh, &bvh)
    }

    fn network(pillars: &[Pillar]) -> BraceNetwork {
        let (mesh, bvh) = empty_scene();
        build_network(pillars, &mesh, &bvh, 2.0, FORK_FLOOR)
    }

    fn links(parts: &[Part]) -> Vec<&Part> {
        parts.iter().filter(|p| p.kind == PartKind::Brace).collect()
    }

    /// A model with no triangles, so every collision test passes (returns no
    /// hit).
    fn empty_scene() -> (Mesh, Bvh) {
        let mesh = Mesh::new(Vec::new(), Vec::new());
        let bvh = Bvh::build(mesh.inner(), Progress::new());
        (mesh, bvh)
    }

    #[test]
    fn added_pillar_is_braced_to_its_two_nearest_neighbours() {
        let (mesh, bvh) = empty_scene();
        let others = [
            pillar(-4.0, 0.0, 20.0),
            pillar(4.0, 0.0, 20.0),
            pillar(5.0, 0.0, 20.0),
        ];
        let braces = brace_into(pillar(0.0, 0.0, 20.0), &others, &mesh, &bvh);

        let ends = |x: f32| {
            (links(&braces).iter()).any(|b| (b.a.x - x).abs() < 1e-4 || (b.b.x - x).abs() < 1e-4)
        };
        assert!(ends(-4.0) && ends(4.0));
        // The third lies in the same direction as the second, so it is skipped.
        assert!(!ends(5.0));
    }

    fn assert_45(brace: &Part) {
        let run = (brace.a.xy() - brace.b.xy()).magnitude();
        let drop = (brace.a.z - brace.b.z).abs();
        assert!((drop - run).abs() < 1e-3, "run {run} drop {drop}");
    }

    #[test]
    fn no_braces_for_single_pillar() {
        assert!(plate_braces(&[pillar(0.0, 0.0, 10.0)]).is_empty());
    }

    #[test]
    fn close_pair_gets_braced() {
        let braces = plate_braces(&[pillar(0.0, 0.0, 30.0), pillar(4.0, 0.0, 30.0)]);
        assert!(!braces.is_empty(), "a close, tall pair must be braced");
        for brace in &braces {
            assert_eq!(brace.kind, PartKind::Brace);
            assert_45(brace);
        }
    }

    #[test]
    fn successive_braces_zigzag() {
        // A tall, close pair takes several braces; each leaves from the other
        // pillar than the one above it.
        let mut braces = plate_braces(&[pillar(0.0, 0.0, 40.0), pillar(2.0, 0.0, 40.0)]);
        assert!(braces.len() >= 3, "got {} braces", braces.len());
        braces.sort_by(|x, y| y.a.z.total_cmp(&x.a.z));
        for w in braces.windows(2) {
            assert_ne!(w[0].a.xy(), w[1].a.xy(), "braces must alternate sides");
        }
    }

    #[test]
    fn braces_never_overlap_in_height() {
        // 5.8 mm apart: the gap (16.2 − 11.6 = 4.6 mm) is shorter than the
        // 5.8 mm each brace drops, yet the braces must stack end to end. (Under
        // 6 mm Manhattan, so no X.)
        let mut braces = plate_braces(&[pillar(0.0, 0.0, 50.0), pillar(5.8, 0.0, 50.0)]);
        assert!(braces.len() >= 2, "got {} braces", braces.len());
        braces.sort_by(|x, y| y.a.z.total_cmp(&x.a.z));
        for w in braces.windows(2) {
            assert!(
                w[1].a.z <= w[0].b.z + 1e-4,
                "{} overlaps {}",
                w[1].a.z,
                w[0].b.z
            );
        }
    }

    #[test]
    fn braces_span_floor_to_top() {
        // The highest brace starts 0.15 mm below the lower top, on the higher
        // pillar; the lowest ends on the floor.
        let mut braces = plate_braces(&[pillar(0.0, 0.0, 40.0), pillar(5.0, 0.0, 30.0)]);
        braces.sort_by(|x, y| y.a.z.total_cmp(&x.a.z));
        let (first, last) = (braces[0], braces[braces.len() - 1]);
        assert!((first.a.z - 29.85).abs() < 1e-4);
        assert_eq!(first.a.xy(), Vector2::new(0.0, 0.0));
        assert!((last.b.z - DEFAULT_PLATE_FLOOR).abs() < 1e-4);
    }

    #[test]
    fn short_pair_is_not_braced() {
        // Tops at 7.4 mm, 3 mm apart: a brace would end at 4.25 mm, below the
        // 5.1 mm floor (0.1 + base + boss + supportconnminheight).
        assert!(plate_braces(&[pillar(0.0, 0.0, 7.4), pillar(3.0, 0.0, 7.4)]).is_empty());
    }

    #[test]
    fn far_pair_gets_crossed_braces() {
        // 10 mm apart is over 5 × Ø = 6 mm: every level is an X.
        let braces = plate_braces(&[pillar(0.0, 0.0, 40.0), pillar(10.0, 0.0, 40.0)]);
        assert!(!braces.is_empty() && braces.len().is_multiple_of(2));
        for pair in braces.chunks(2) {
            let (a, b) = (pair[0], pair[1]);
            assert_45(&a);
            assert_45(&b);
            assert_eq!(a.a.xy(), b.a.xy(), "both start on the same pillar");
            assert!((a.a.z - b.b.z).abs() < 1e-4 && (a.b.z - b.a.z).abs() < 1e-4);
        }
    }

    #[test]
    fn brace_count_and_spacing() {
        // top 20, floor 0, run 3, gap 5: braces end at 17, 9, 1, leaving
        // 1 ≤ run below, so three. Spread evenly: spacing (20 − 9) / 2 = 5.5,
        // lower ends at 17, 8.5, 0.
        assert_eq!(brace_heights(20.0, 0.0, 3.0, 5.0), vec![17.0, 8.5, 0.0]);
        // With 4 mm left below the third (> run), a fourth fits:
        // spacing (23 − 12) / 3.
        let heights = brace_heights(23.0, 0.0, 3.0, 5.0);
        assert_eq!(heights.len(), 4);
        assert!((heights[2] - (3.0 + 11.0 / 3.0)).abs() < 1e-4);
        // Not even one brace fits.
        assert!(brace_heights(2.5, 0.0, 3.0, 5.0).is_empty());
    }

    #[test]
    fn far_pair_is_not_braced() {
        // Two pillars 40 mm apart exceed the 30 mm max spacing.
        assert!(plate_braces(&[pillar(0.0, 0.0, 30.0), pillar(40.0, 0.0, 30.0)]).is_empty());
    }

    #[test]
    fn braces_stay_above_the_floor() {
        for brace in plate_braces(&[pillar(0.0, 0.0, 30.0), pillar(10.0, 0.0, 30.0)]) {
            assert!(brace.a.z.min(brace.b.z) >= DEFAULT_PLATE_FLOOR - 1e-3);
        }
    }

    /// An axis-aligned box mesh with outward facing triangles.
    fn box_scene(min: Vector3<f32>, max: Vector3<f32>) -> (Mesh, Bvh) {
        let mut verts = Vec::new();
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
        let faces = quads
            .iter()
            .flat_map(|&[a, b, c, d]| [[a, b, c], [a, c, d]])
            .collect();
        let mesh = Mesh::new(verts, faces);
        let bvh = Bvh::build(mesh.inner(), Progress::new());
        (mesh, bvh)
    }

    #[test]
    fn braces_through_the_model_are_skipped() {
        // A wall between two tall pillars blocks every brace.
        let (mesh, bvh) = box_scene(Vector3::new(4.0, -5.0, 0.0), Vector3::new(6.0, 5.0, 40.0));
        let pillars = [pillar(0.0, 0.0, 30.0), pillar(10.0, 0.0, 30.0)];
        assert!(build_braces(&pillars, MAX_CROSS_SPACING_XY, &mesh, &bvh).is_empty());
    }

    #[test]
    fn braces_keep_the_model_clearance() {
        // A wall 0.5 mm beside the brace line: clear of the 0.6 mm brace
        // itself, but inside its 1.2 mm test radius (radius + clearance).
        let (mesh, bvh) = box_scene(Vector3::new(0.5, 1.1, 0.0), Vector3::new(3.5, 3.0, 40.0));
        let pillars = [pillar(0.0, 0.0, 30.0), pillar(4.0, 0.0, 30.0)];
        assert!(build_braces(&pillars, MAX_CROSS_SPACING_XY, &mesh, &bvh).is_empty());
    }

    #[test]
    fn landing_braces_stay_above_the_feet() {
        let (mesh, bvh) = empty_scene();
        let landing = |x, foot| Pillar::landing(Vector2::new(x, 0.0), 20.0, foot, 0.6);

        // 2 mm apart, feet at 8 mm: floor 11.1 mm. Two braces, the lowest
        // ending on the floor, the highest starting at 19.85.
        let mut braces = build_braces(&[landing(0.0, 8.0), landing(2.0, 8.0)], 2.6, &mesh, &bvh);
        braces.sort_by(|x, y| y.a.z.total_cmp(&x.a.z));
        assert_eq!(braces.len(), 2);
        assert!((braces[0].a.z - 19.85).abs() < 1e-4);
        assert!((braces[1].b.z - 11.1).abs() < 1e-4);

        // A foot at 16.5 mm (floor 19.6) leaves no room for a brace.
        let pillars = [landing(0.0, 8.0), landing(2.0, 16.5)];
        assert!(build_braces(&pillars, 2.6, &mesh, &bvh).is_empty());

        // 3 mm apart exceeds the 2.6 mm landing edge limit.
        let pillars = [landing(0.0, 8.0), landing(3.0, 8.0)];
        assert!(build_braces(&pillars, 2.6, &mesh, &bvh).is_empty());
    }

    #[test]
    fn tree_skips_unbraceable_edges() {
        // B is a short landing pillar high above A and C: XY-nearest to both,
        // but no brace fits. A and C must still be braced to each other.
        let (mesh, bvh) = empty_scene();
        let pillars = [
            Pillar::landing(Vector2::new(0.0, 0.0), 30.0, 10.0, 0.6),
            Pillar::landing(Vector2::new(1.2, 0.3), 48.0, 46.0, 0.6),
            Pillar::landing(Vector2::new(2.5, 0.0), 30.0, 10.0, 0.6),
        ];
        let braces = build_braces(&pillars, 2.6, &mesh, &bvh);
        assert!(!braces.is_empty());
        assert!(braces.iter().all(|b| b.a.z < 30.0 && b.b.z >= 13.1 - 1e-4));
    }

    #[test]
    fn near_parallel_braces_are_detected() {
        let existing = [Vector2::new(1.0, 0.0)];
        // 15° off: within 20°.
        let a = 15f32.to_radians();
        assert!(near_parallel(&existing, Vector2::new(a.cos(), a.sin())));
        // 25° off, or pointing the other way: allowed.
        let b = 25f32.to_radians();
        assert!(!near_parallel(&existing, Vector2::new(b.cos(), b.sin())));
        assert!(!near_parallel(&existing, Vector2::new(-1.0, 0.0)));
    }

    #[test]
    fn chain_of_four_forks_from_the_far_end() {
        // Four pillars 4 mm apart (≤ 4 × Ø = 4.8 mm). The chain is walked from
        // its far end, so the third pillar forks onto the second and fourth.
        let forks = forks(&row(&[0.0, 4.0, 8.0, 12.0], 20.0));
        assert_eq!(forks.bottoms.len(), 1, "one pillar is truncated");
        assert_eq!(forks.bottoms[0].0, 2);
        // F = 20 − (2 × 1.2 + 0.3 × 2.0) = 17.
        assert!((forks.bottoms[0].1 - 17.0).abs() < 1e-3);
        let links = links(&forks.parts);
        assert_eq!(links.len(), 2, "a fork emits two links");
        for link in links {
            assert_45(link);
        }
        let joint = forks
            .parts
            .iter()
            .find(|p| p.kind == PartKind::Joint)
            .unwrap();
        assert_eq!(joint.a, Vector3::new(8.0, 0.0, 17.0));
    }

    #[test]
    fn short_chain_does_not_fork() {
        // Three in a row are not a chain (it needs four).
        assert!(forks(&row(&[0.0, 4.0, 8.0], 20.0)).bottoms.is_empty());
    }

    #[test]
    fn long_chain_alternates_anchors_and_middles() {
        // Seven in a row: middles at 5, 3 and 1, anchors shared between them.
        let forks = forks(&row(&[0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 24.0], 30.0));
        let mut middles: Vec<usize> = forks.bottoms.iter().map(|&(i, _)| i).collect();
        middles.sort();
        assert_eq!(middles, vec![1, 3, 5]);
    }

    /// Asserts every brace end touches a pillar of `pillars` between its top
    /// and its (possibly forked) bottom.
    fn assert_no_floating_ends(pillars: &[Pillar], network: &BraceNetwork) {
        let bottom = |i: usize| {
            (network.fork_bottoms.iter())
                .find(|&&(j, _)| j == i)
                .map_or(0.0, |&(_, z)| z)
        };
        for brace in &network.braces {
            for end in [brace.a, brace.b] {
                let ok = pillars.iter().enumerate().any(|(i, p)| {
                    (p.xy - end.xy()).magnitude() < 1e-4
                        && end.z <= p.top_z + 1e-4
                        && end.z >= bottom(i) - 1e-4
                });
                assert!(ok, "brace end {end:?} floats in mid-air");
            }
        }
    }

    #[test]
    fn forks_never_float() {
        // A long collinear row on a slope: no link may land beside a lower
        // neighbour, and no brace may run below a fork point.
        let pillars: Vec<Pillar> = (0..9)
            .map(|i| pillar(i as f32 * 4.0, 0.0, 30.0 - i as f32 * 1.5))
            .collect();
        let network = network(&pillars);
        assert!(!network.fork_bottoms.is_empty());
        assert_no_floating_ends(&pillars, &network);
    }

    #[test]
    fn fork_skips_a_neighbour_too_low() {
        // The last pillar's tip is below where the fork link would land.
        let mut pillars = row(&[0.0, 4.0, 8.0], 20.0);
        pillars.push(pillar(12.0, 0.0, 10.0));
        assert!(forks(&pillars).bottoms.is_empty());
    }

    #[test]
    fn far_chain_is_not_forked() {
        // Gaps of 5 mm exceed 4 × Ø = 4.8 mm.
        assert!(
            forks(&row(&[0.0, 5.0, 10.0, 15.0], 20.0))
                .bottoms
                .is_empty()
        );
    }

    #[test]
    fn bent_chain_is_not_forked() {
        // A right-angle corner splits the row into chains shorter than four.
        let pillars = [
            pillar(0.0, 4.0, 20.0),
            pillar(0.0, 0.0, 20.0),
            pillar(4.0, 0.0, 20.0),
            pillar(8.0, 0.0, 20.0),
        ];
        assert!(forks(&pillars).bottoms.is_empty());
    }

    #[test]
    fn low_chain_is_not_forked() {
        // The fork point would sit below the 5.3 mm floor.
        assert!(forks(&row(&[0.0, 4.0, 8.0, 12.0], 6.0)).bottoms.is_empty());
    }

    #[test]
    fn forked_middle_leaves_the_tree() {
        // Only the fork links and its joint touch the forked pillar, and the
        // anchors' tree brace stays below the fork links (13 mm).
        let pillars = row(&[0.0, 4.0, 8.0, 12.0], 20.0);
        let network = network(&pillars);
        assert_eq!(network.fork_bottoms, vec![(2, 17.0)]);
        for brace in links(&network.braces) {
            let touches_middle = [brace.a, brace.b].iter().any(|e| (e.x - 8.0).abs() < 1e-4);
            if touches_middle {
                assert!(
                    (brace.a.z - 17.0).abs() < 1e-4,
                    "only fork links reach pillar 2"
                );
            } else if brace.a.x == 4.0 && brace.b.x == 12.0 || brace.a.x == 12.0 && brace.b.x == 4.0
            {
                assert!(brace.a.z.max(brace.b.z) <= 13.0 - 0.15 + 1e-4);
            }
        }
    }
}
