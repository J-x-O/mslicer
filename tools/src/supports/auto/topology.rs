//! Brace network (plan §2.5 steps 3–6).
//!
//! Once every support has been routed to a vertical pillar, neighbouring
//! pillars are tied together with 45° braces so tall, sparsely supported
//! pillars don't stand alone. The topology mirrors CHITUBOX's
//! `processIsolatedNearGrounds`: a Delaunay triangulation over the pillar
//! positions is reduced to a minimum spanning tree, every tree edge no longer
//! than `maximalsupportcrossspacingxy` (30 mm) is kept, and each kept edge is
//! braced with one or more 45° diagonals.
//!
//! 2.8's pre-merge and 2.5's inverted-V forks are not here yet; this is the
//! spanning-tree + brace slice only.

use nalgebra::{Vector2, Vector3};
use spade::{DelaunayTriangulation, Point2, Triangulation};

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

/// A pillar as seen by the brace network: its XY position (where it meets the
/// plate) and the height of its top (where the tip/link ends and the vertical
/// pillar begins).
#[derive(Clone, Copy, Debug)]
pub struct Pillar {
    pub xy: Vector2<f32>,
    pub top_z: f32,
    /// The pillar radius, in millimetres. Braces are as thick as the pillars
    /// they join (plan §2.5 step 4: `supportcrossdiameter` never reaches the
    /// algorithm, so `0.8`/`1.2`/`1.5` Ø becomes the brace radius too).
    pub radius: f32,
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
    use nalgebra::Vector2;

    use super::*;

    fn pillar(x: f32, y: f32, z: f32) -> Pillar {
        Pillar {
            xy: Vector2::new(x, y),
            top_z: z,
            radius: 0.6,
        }
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
}
