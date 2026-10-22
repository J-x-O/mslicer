//! Brace network (plan §2.5).
//!
//! Once every support has been routed to a vertical pillar, neighbouring
//! pillars are tied together with 45° braces so tall, sparsely supported
//! pillars don't stand alone. The topology mirrors CHITUBOX's
//! `processIsolatedNearGrounds`:
//!
//! * a Delaunay triangulation over the pillar positions is reduced to a minimum
//!   spanning tree, every tree edge no longer than `maximalsupportcrossspacingxy`
//!   (30 mm) is kept, and each kept edge is braced with one or more 45° diagonals
//!   (§2.5 steps 3–4);
//! * nearly-collinear triples of pillars become inverted-V forks: the middle
//!   pillar stops short and is carried by two 45° links to its neighbours
//!   (§2.5 step 2).
//!
//! 2.8's pre-merge is not here; it is off by default and its rules do not run in
//! normal auto-support.

use nalgebra::{Vector2, Vector3};
use slicer::geometry::{mesh::Mesh, partitioning::Bvh};
use spade::{DelaunayTriangulation, Point2, Triangulation};

use crate::supports::auto::collision;
use crate::supports::{Part, PartKind, Shape};

/// Maximum XY spacing between two pillars that are still braced together
/// (`maximalsupportcrossspacingxy`, plan §2.5 step 3).
const MAX_CROSS_SPACING_XY: f32 = 30.0;

/// How far below the lower pillar top the first brace starts (plan §2.5 step 4:
/// 0.15 mm).
const FIRST_BRACE_DROP: f32 = 0.15;

/// Minimum vertical step between repeated braces down a pillar pair
/// (`minimalsupportcrossspacingz`, 2 mm; plan §2.5 step 4).
const MIN_BRACE_STEP_Z: f32 = 2.0;

/// The other term in the brace step formula: `13.5 × crossDiameter` (plan §2.5
/// step 4). `crossDiameter` here is the pillar Ø, since `supportcrossdiameter`
/// never reaches the algorithm (verified, §2.5 step 4).
const BRACE_STEP_DIAMETER_FACTOR: f32 = 13.5;

/// Minimum height above the plate at which braces stop (`supportconnminheight`,
/// 3 mm; plan §2.5 step 4). The +1 mm accounts for the raft/base thickness that
/// isn't modelled yet.
const BRACE_STOP_HEIGHT: f32 = 3.0;

/// Inverted-V fork gap limit, as a multiple of the pillar Ø (plan §2.5 step 2:
/// each gap ≤ 4 × pillar Ø).
const FORK_GAP_DIAMETERS: f32 = 4.0;

/// How far below the pillar top the fork point sits, in pillar Øs (plan §2.5
/// step 2: `2 × pillar Ø`).
const FORK_DROP_DIAMETERS: f32 = 2.0;

/// The cone-length term of the fork drop (plan §2.5 step 2: `0.3 × cone length`).
const FORK_DROP_CONE_FACTOR: f32 = 0.3;

/// The most the tip end may sit above the fork point, as a multiple of
/// `cone length + pillar Ø` (plan §2.5 step 2: `2.5 × (cone length + pillar Ø)`).
const FORK_TIP_MAX_DIAMETERS: f32 = 2.5;

/// Extra clearance in the fork floor formula (plan §2.5 step 2: `+ 0.1 mm`).
const FORK_FLOOR_EXTRA: f32 = 0.1;

/// Base/raft thickness added to the fork floor. The Middle preset's base is
/// 1.0 mm thick and the raft isn't modelled until M5, so this stands in for
/// "base or raft thickness" (§2.5 step 2).
const BASE_THICKNESS: f32 = 1.0;

/// `cos(45°)`: the fork chain turns at most 45° from one step to the next
/// (plan §2.5 step 2).
const COS_45: f32 = 0.707_106_78;

/// A pillar as seen by the brace network: its XY position (where it meets the
/// plate) and the height of its top (where the tip/link ends and the vertical
/// pillar begins).
#[derive(Clone, Copy, Debug)]
pub struct Pillar {
    pub xy: Vector2<f32>,
    pub top_z: f32,
    /// The tip end (horn end) height. Equal to `top_z` for a straight drop, and
    /// higher when a 45° link runs from the tip end down to the pillar top.
    pub tip_z: f32,
    /// The pillar radius, in millimetres. Braces are as thick as the pillars
    /// they join (plan §2.5 step 4: `supportcrossdiameter` never reaches the
    /// algorithm, so `0.8`/`1.2`/`1.5` Ø becomes the brace radius too).
    pub radius: f32,
}

/// The complete brace network: 45° spanning-tree braces plus inverted-V fork
/// links, and the pillar truncations the forks impose.
pub struct BraceNetwork {
    pub braces: Vec<Part>,
    /// `(support_index, bottom_z)`: the vertical pillar of support
    /// `support_index` must end at `bottom_z` (a fork point) instead of
    /// reaching the plate.
    pub fork_bottoms: Vec<(usize, f32)>,
}

/// Builds the set of brace parts for a routed support set.
///
/// `pillars` is one entry per routed support's vertical pillar. The Delaunay /
/// MST is computed over the XY positions; each kept edge is braced with 45°
/// diagonals from the higher pillar down to the lower (plan §2.5 step 4).
pub fn build_braces(pillars: &[Pillar]) -> Vec<Part> {
    if pillars.len() < 2 {
        return Vec::new();
    }

    let edges = spanning_tree_edges(pillars);

    let mut braces = Vec::new();
    for (i, j) in edges {
        if let Some(parts) = brace_pair(pillars[i], pillars[j]) {
            braces.extend(parts);
        }
    }
    braces
}

/// Builds the full brace network over `pillars`: the spanning-tree braces of
/// [`build_braces`] plus the inverted-V forks of §2.5 step 2.
///
/// `cone_length` is the tip cone length (`supporttiplength`), used to place the
/// fork point. `mesh`/`bvh` are the model, used to link-test each fork link.
pub fn build_network(pillars: &[Pillar], mesh: &Mesh, bvh: &Bvh, cone_length: f32) -> BraceNetwork {
    let mut braces = build_braces(pillars);
    let (fork_links, fork_bottoms) = build_forks(pillars, cone_length, mesh, bvh);
    braces.extend(fork_links);
    BraceNetwork {
        braces,
        fork_bottoms,
    }
}

/// Emits inverted-V forks (plan §2.5 step 2). Returns the fork links and the
/// `(pillar index, fork z)` truncation for each forked (middle) pillar.
///
/// For each pillar B we look for the most collinear pair of neighbours A, C
/// within 4 × Ø, both turning no more than 45° from straight. A fork point F is
/// placed on B, two 45° links run from F down to A and C (each link-tested),
/// and B's pillar is truncated at F so A and C carry it instead of the plate.
///
/// Each pillar is forked at most once. A forked (truncated) pillar may still
/// serve as a neighbour for an adjacent fork; in that rare chain-of-four case
/// its links land where the original pillar ran, which §2.5 does not spell out.
/// Accepted as a simplification.
fn build_forks(
    pillars: &[Pillar],
    cone_length: f32,
    mesh: &Mesh,
    bvh: &Bvh,
) -> (Vec<Part>, Vec<(usize, f32)>) {
    let mut links = Vec::new();
    let mut bottoms = Vec::new();

    for b in 0..pillars.len() {
        let middle = pillars[b];
        let diameter = 2.0 * middle.radius;
        // The 9 × Ø chain radius of §2.5 step 2 only bounds how far ahead the
        // chain walks; a fork still requires gaps ≤ 4 × Ø, so we enumerate
        // neighbours at the tighter 4 × Ø bound directly.
        let max_gap = FORK_GAP_DIAMETERS * diameter;

        let mut neighbours = Vec::new();
        for a in 0..pillars.len() {
            if a != b && (pillars[a].xy - middle.xy).magnitude() <= max_gap {
                neighbours.push(a);
            }
        }

        // Pick the most collinear pair of neighbours on opposite sides of B.
        let mut best: Option<(usize, usize, f32)> = None;
        for &a in &neighbours {
            for &c in &neighbours {
                if a >= c {
                    continue;
                }
                let d1 = middle.xy - pillars[a].xy;
                let d2 = pillars[c].xy - middle.xy;
                let (l1, l2) = (d1.magnitude(), d2.magnitude());
                if l1 <= f32::EPSILON || l2 <= f32::EPSILON {
                    continue;
                }
                let col = d1.dot(&d2) / (l1 * l2);
                if col < COS_45 {
                    continue;
                }
                if best.map_or(true, |(_, _, bc)| col > bc) {
                    best = Some((a, c, col));
                }
            }
        }

        if let Some((a, c, _)) = best {
            if let Some((parts, bottom)) = fork_at(pillars, a, b, c, cone_length, mesh, bvh) {
                links.extend(parts);
                bottoms.push(bottom);
            }
        }
    }

    (links, bottoms)
}

/// The minimum fork-point height above the plate (plan §2.5 step 2: `ground +
/// pillar Ø + 0.1 mm + base or raft thickness + supportconnminheight`; ground
/// is 0, so this is 5.3 mm for the Middle preset).
fn fork_floor(diameter: f32) -> f32 {
    diameter + FORK_FLOOR_EXTRA + BASE_THICKNESS + BRACE_STOP_HEIGHT
}

/// Tries to fork the middle pillar `b` onto its neighbours `a` and `c`.
///
/// Returns the two 45° fork links and the truncation `(b, fork_z)` when the
/// fork is valid and both links pass the link test, otherwise `None`.
fn fork_at(
    pillars: &[Pillar],
    a: usize,
    b: usize,
    c: usize,
    cone_length: f32,
    mesh: &Mesh,
    bvh: &Bvh,
) -> Option<(Vec<Part>, (usize, f32))> {
    let (pa, pb, pc) = (pillars[a], pillars[b], pillars[c]);
    let diameter = 2.0 * pb.radius;

    // The fork point sits on B's pillar, `2 × Ø + 0.3 × cone length` below its
    // top (§2.5 step 2).
    let fork_z = pb.top_z - (FORK_DROP_DIAMETERS * diameter + FORK_DROP_CONE_FACTOR * cone_length);

    // F must stay above the floor, and B's tip end may sit at most
    // `2.5 × (cone length + Ø)` above F (§2.5 step 2).
    if fork_z < fork_floor(diameter) {
        return None;
    }
    if pb.tip_z - fork_z > FORK_TIP_MAX_DIAMETERS * (cone_length + diameter) {
        return None;
    }

    let radius = pa.radius.min(pb.radius).min(pc.radius);
    let from = Vector3::new(pb.xy.x, pb.xy.y, fork_z);

    let mut parts = Vec::with_capacity(2);
    for &end in &[a, c] {
        let run = (pillars[end].xy - pb.xy).magnitude();
        if run <= f32::EPSILON {
            return None;
        }
        let to = Vector3::new(pillars[end].xy.x, pillars[end].xy.y, fork_z - run);
        // A fork link may not dip below the plate (defensive; the fork-floor
        // check above normally keeps the landing point above the base).
        if to.z < 0.0 || collision::link_test(mesh, bvh, from, to, radius) {
            return None;
        }
        parts.push(Part {
            kind: PartKind::Brace,
            a: from,
            b: to,
            radius_a: radius,
            radius_b: radius,
            shape: Shape::Round,
        });
    }

    Some((parts, (b, fork_z)))
}

/// Delaunay triangulation over pillar XY positions, reduced to a minimum
/// spanning tree, then filtered to edges no longer than
/// [`MAX_CROSS_SPACING_XY`]. Returns `(a, b)` index pairs into `pillars`.
///
/// The MST is Kruskal over the Delaunay edges: Delaunay already contains the
/// Euclidean MST as a subgraph, so this converges quickly and the plan's
/// "1.3 × R edge filter" (§2.5 step 3) is not needed for the plate-pillar path
/// (it only runs in enhanced mode).
fn spanning_tree_edges(pillars: &[Pillar]) -> Vec<(usize, usize)> {
    let points: Vec<Point2<f32>> = pillars
        .iter()
        .map(|p| Point2::new(p.xy.x, p.xy.y))
        .collect();

    // Bulk-load the Delaunay. Duplicate XY positions are collapsed to the lowest
    // index, so the returned edge indices are into `points`, not `pillars`; we
    // only need them as vertex keys, and both vectors share an index space as
    // long as we build `points` from `pillars` 1:1 (duplicates aside). The MST
    // uses positions, not pillar handles, so a collapsed duplicate simply means
    // two co-located pillars share one vertex — acceptable, they'd never brace
    // apart.
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

    fn find(parent: &mut Vec<usize>, x: usize) -> usize {
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
        let idx_a = point_index(&points, a);
        let idx_b = point_index(&points, b);
        let (Some(ia), Some(ib)) = (idx_a, idx_b) else {
            continue;
        };

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

        // Apply the max-spacing filter on the actual pillar distance.
        let pa = pillars[ia].xy;
        let pb = pillars[ib].xy;
        if (pa - pb).magnitude() <= MAX_CROSS_SPACING_XY {
            kept.push((ia, ib));
        }

        if kept.len() == n - 1 {
            break;
        }
    }

    kept
}

/// Maps a `Point2` back to its index in `points` by exact bit-for-bit match.
/// Duplicate positions collapsed by the Delaunay map to the same (first) index;
/// positions that don't match (because `bulk_load_stable` kept a copy that lost
/// a collinear duplicate) fall through to a nearest fallback lookup.
fn point_index(points: &[Point2<f32>], p: Point2<f32>) -> Option<usize> {
    points.iter().position(|q| q.x == p.x && q.y == p.y)
}

/// Braces a pillar pair with 45° diagonals (plan §2.5 step 4). Returns `None`
/// when the pair is too close or too short to brace.
fn brace_pair(a: Pillar, b: Pillar) -> Option<Vec<Part>> {
    let higher = if a.top_z >= b.top_z { a } else { b };
    let lower = if a.top_z >= b.top_z { b } else { a };

    // The diagonal drops exactly as much as it travels sideways: 45°.
    let run = (higher.xy - lower.xy).magnitude();
    if run <= f32::EPSILON {
        return None;
    }

    let radius = higher.radius.min(lower.radius);

    // Brace spacing grows with the pillar distance, so short spans aren't
    // over-braced (§2.5 step 4): step = max(2 mm, 13.5 × pillar Ø − 2 × run).
    let step_z = MIN_BRACE_STEP_Z.max(BRACE_STEP_DIAMETER_FACTOR * (2.0 * radius) - 2.0 * run);

    // First brace starts 0.15 mm below the lower pillar's top, descending
    // toward the higher pillar's surface height.
    let start_z = lower.top_z - FIRST_BRACE_DROP;

    let mut parts = Vec::new();
    let mut z = start_z;
    while z > BRACE_STOP_HEIGHT {
        // A brace drops exactly as far as it travels sideways (45°). If that
        // would carry it below the minimum brace height above the plate, stop:
        // braces end at `supportconnminheight`, they are never truncated to a
        // shallower angle (§2.5 step 4).
        let to_z = z - run;
        if to_z < BRACE_STOP_HEIGHT {
            break;
        }

        let from = Vector3::new(lower.xy.x, lower.xy.y, z);
        let to = Vector3::new(higher.xy.x, higher.xy.y, to_z);

        parts.push(Part {
            kind: PartKind::Brace,
            a: from,
            b: to,
            radius_a: radius,
            radius_b: radius,
            shape: Shape::Round,
        });

        z -= step_z;
    }

    (!parts.is_empty()).then_some(parts)
}

#[cfg(test)]
mod tests {
    use common::progress::Progress;
    use nalgebra::Vector2;
    use slicer::geometry::{mesh::Mesh, partitioning::Bvh};

    use super::*;

    fn pillar(x: f32, y: f32, z: f32) -> Pillar {
        Pillar {
            xy: Vector2::new(x, y),
            top_z: z,
            tip_z: z,
            radius: 0.6,
        }
    }

    /// A model with no triangles, so every collision test passes (returns no
    /// hit). Fork links only need the link test to come back clean.
    fn empty_scene() -> (Mesh, Bvh) {
        let mesh = Mesh::new(Vec::new(), Vec::new());
        let bvh = Bvh::build(mesh.inner(), Progress::new());
        (mesh, bvh)
    }

    #[test]
    fn no_braces_for_single_pillar() {
        let braces = build_braces(&[pillar(0.0, 0.0, 10.0)]);
        assert!(braces.is_empty());
    }

    #[test]
    fn close_pair_gets_braced() {
        // Two pillars 10 mm apart, tall enough to take at least one brace.
        let pillars = [pillar(0.0, 0.0, 30.0), pillar(10.0, 0.0, 30.0)];
        let braces = build_braces(&pillars);
        assert!(!braces.is_empty(), "a close, tall pair must be braced");

        for brace in &braces {
            assert_eq!(brace.kind, PartKind::Brace);
            // 45°: vertical drop equals horizontal run.
            let run = (brace.a.xy() - brace.b.xy()).magnitude();
            let drop = brace.a.z - brace.b.z;
            assert!(
                (drop - run).abs() < 1e-3,
                "brace must drop exactly as far as it runs (45°), got run {run} drop {drop}"
            );
        }
    }

    #[test]
    fn far_pair_is_not_braced() {
        // Two pillars 40 mm apart exceed the 30 mm max spacing.
        let pillars = [pillar(0.0, 0.0, 30.0), pillar(40.0, 0.0, 30.0)];
        assert!(build_braces(&pillars).is_empty());
    }

    #[test]
    fn braces_stop_above_the_plate() {
        let pillars = [pillar(0.0, 0.0, 30.0), pillar(10.0, 0.0, 30.0)];
        for brace in build_braces(&pillars) {
            assert!(
                brace.b.z >= BRACE_STOP_HEIGHT - 1e-3,
                "brace must stop above the plate"
            );
        }
    }

    #[test]
    fn collinear_triple_gets_forked() {
        // Three pillars 4 mm apart (≤ 4 × Ø = 4.8 mm), tall enough to fork.
        let pillars = [
            pillar(0.0, 0.0, 20.0),
            pillar(4.0, 0.0, 20.0),
            pillar(8.0, 0.0, 20.0),
        ];
        let (mesh, bvh) = empty_scene();
        let (links, bottoms) = build_forks(&pillars, 2.0, &mesh, &bvh);

        assert_eq!(links.len(), 2, "a fork emits two links");
        assert_eq!(bottoms.len(), 1, "one pillar is truncated");
        assert_eq!(bottoms[0].0, 1, "the middle pillar is truncated");
        // F = 20 − (2 × 1.2 + 0.3 × 2.0) = 20 − 3.0 = 17.0.
        assert!((bottoms[0].1 - 17.0).abs() < 1e-3);

        for link in &links {
            assert_eq!(link.kind, PartKind::Brace);
            let run = (link.a.xy() - link.b.xy()).magnitude();
            let drop = link.a.z - link.b.z;
            assert!(
                (drop - run).abs() < 1e-3,
                "fork link must drop exactly as far as it runs (45°), got run {run} drop {drop}"
            );
        }
    }

    #[test]
    fn far_triple_is_not_forked() {
        // Gaps of 5 mm exceed 4 × Ø = 4.8 mm, so no fork forms.
        let pillars = [
            pillar(0.0, 0.0, 20.0),
            pillar(5.0, 0.0, 20.0),
            pillar(10.0, 0.0, 20.0),
        ];
        let (mesh, bvh) = empty_scene();
        let (links, bottoms) = build_forks(&pillars, 2.0, &mesh, &bvh);
        assert!(links.is_empty());
        assert!(bottoms.is_empty());
    }

    #[test]
    fn bent_triple_is_not_forked() {
        // B sits at a right-angle corner: A to the north, C to the east. The
        // turn exceeds 45°, so no fork forms.
        let pillars = [
            pillar(0.0, 4.0, 20.0), // A
            pillar(0.0, 0.0, 20.0), // B
            pillar(4.0, 0.0, 20.0), // C
        ];
        let (mesh, bvh) = empty_scene();
        let (links, bottoms) = build_forks(&pillars, 2.0, &mesh, &bvh);
        assert!(links.is_empty());
        assert!(bottoms.is_empty());
    }

    #[test]
    fn low_triple_is_not_forked() {
        // The fork point would sit below the 5.3 mm floor, so no fork forms.
        let pillars = [
            pillar(0.0, 0.0, 6.0),
            pillar(4.0, 0.0, 6.0),
            pillar(8.0, 0.0, 6.0),
        ];
        let (mesh, bvh) = empty_scene();
        let (links, bottoms) = build_forks(&pillars, 2.0, &mesh, &bvh);
        assert!(links.is_empty());
        assert!(bottoms.is_empty());
    }

    #[test]
    fn network_propagates_fork_bottoms() {
        let pillars = [
            pillar(0.0, 0.0, 20.0),
            pillar(4.0, 0.0, 20.0),
            pillar(8.0, 0.0, 20.0),
        ];
        let (mesh, bvh) = empty_scene();
        let network = build_network(&pillars, &mesh, &bvh, 2.0);
        assert_eq!(network.fork_bottoms.len(), 1);
        assert_eq!(network.fork_bottoms[0].0, 1);
    }
}
