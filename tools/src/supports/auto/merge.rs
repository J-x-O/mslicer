//! Pillar pre-merge and its manual "merge selected" form.
//!
//! Auto-support routes every contact to its own pillar; the pre-merge joins
//! neighbouring pillars into trees before basing, so a cluster over a small
//! overhang shares one trunk instead of standing on many parallel columns. The
//! same rules power the manual "merge selected" command over supports that
//! already exist.
//!
//! The pre-merge is off by default; the manual merge command runs the rules
//! unconditionally on the chosen pillars.

use nalgebra::{Vector2, Vector3};
use slicer::geometry::{mesh::Mesh, partitioning::Bvh};

use crate::supports::{
    Part, PartKind, Shape, Support,
    auto::{collision, topology::Pillar},
};

/// The merge radius `R` (0.5 × point spacing, i.e. 2.0 mm at the default
/// 4.0 mm spacing). This is the neighbourhood the grouping passes grow from.
pub const MERGE_R: f32 = 2.0;
/// Neighbour XY bound within a group: 0.6 × R.
const GROUP_RADIUS_FACTOR: f32 = 0.6;
/// Link collision radius, 1.2 × pillar radius.
const LINK_RADIUS_FACTOR: f32 = 1.2;
/// Radius below which a near-cylinder is kept as a slim support and skipped by
/// the merge (0.31 mm).
const SLIM_RADIUS: f32 = 0.31;
/// Tolerance for a pillar's ends being vertical (0.004 mm).
const VERTICAL_TOLERANCE: f32 = 0.004;
/// Tip-end z tolerance for "merge selected" grouping (0.1 mm).
const GROUP_Z_TOLERANCE: f32 = 0.1;
/// Joint sphere inflation over the merged pillar's radius (1.04 ×).
const JOINT_RADIUS_FACTOR: f32 = 1.04;

/// The pillars of a `SupportSet` eligible for the manual merge:
/// only vertical pillar segments count, and near-cylinders thinner than the
/// slim radius are skipped. Returns the extracted `Pillar` geometry plus the
/// source support index for each, in the same order.
pub fn extract_merge_candidates(supports: &[Support]) -> Vec<(usize, Pillar)> {
    let mut out = Vec::new();
    for (i, support) in supports.iter().enumerate() {
        // The vertical pillar part: its top end is the joint, its bottom end the
        // plate. A support's tip end (`tip_z`) sits above the joint on a lateral
        // link, or equals it on a straight drop.
        let Some(pillar) = support.parts.iter().find(|p| p.kind == PartKind::Pillar) else {
            continue;
        };
        // Vertical check: XY ends within tolerance.
        if (pillar.a.xy() - pillar.b.xy()).magnitude() > VERTICAL_TOLERANCE {
            continue;
        }
        // Slim skip: near-cylinders thinner than 0.31 mm radius.
        if pillar.radius_a < SLIM_RADIUS || pillar.radius_b < SLIM_RADIUS {
            continue;
        }

        let tip_z = support
            .parts
            .iter()
            .find(|p| p.kind == PartKind::TipCone)
            .map(|t| t.b.z)
            .unwrap_or(pillar.a.z);
        let top_z = pillar.a.z;

        out.push((
            i,
            Pillar::plate(pillar.a.xy(), top_z, tip_z, pillar.radius_a),
        ));
    }
    out
}

/// The result of merging a set of pillars: the 45° links that connect members
/// to their group's trunk, and the truncation for every pillar whose own
/// column now stops at a merge point instead of reaching the plate.
#[derive(Clone, Debug, Default)]
pub struct Merged {
    /// 45° links from a member's merge point down to its group's trunk.
    pub links: Vec<Part>,
    /// `(pillar_index, bottom_z)`: the pillar at `pillar_index` ends at
    /// `bottom_z` (a merge point) instead of the plate.
    pub trunk_bottoms: Vec<(usize, f32)>,
}

/// Groups the given pillars within the merge radius `r` and joins each group
/// into a tree. Returns the links and trunk truncations. Pillars farther
/// than `0.6 × r` from any fellow member stay unmerged.
///
/// The merge point of a group is the member nearest its centre, at the lowest
/// height a 45° link from any member can reach on it: `min over members of
/// (tip_z − XY distance to the representative)`. Members whose link fails the
/// 1.2 × radius
/// collision test keep their own pillar.
pub fn merge_pillars(pillars: &[Pillar], r: f32, mesh: &Mesh, bvh: &Bvh) -> Merged {
    // Single-linkage clusters over the 0.6 × R neighbourhood: its transitivity
    // gives the same chained growth as repeated grouping passes.
    let group_radius = GROUP_RADIUS_FACTOR * r;
    let mut parent: Vec<usize> = (0..pillars.len()).collect();
    let mut rank = vec![0usize; pillars.len()];

    fn find(parent: &mut [usize], x: usize) -> usize {
        let mut root = x;
        while parent[root] != root {
            root = parent[root];
        }
        let mut cur = x;
        while parent[cur] != cur {
            let next = parent[cur];
            parent[cur] = root;
            cur = next;
        }
        root
    }

    for a in 0..pillars.len() {
        for b in (a + 1)..pillars.len() {
            if (pillars[a].xy - pillars[b].xy).magnitude() > group_radius {
                continue;
            }
            let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
            if ra == rb {
                continue;
            }
            if rank[ra] < rank[rb] {
                parent[ra] = rb;
            } else {
                parent[rb] = ra;
                if rank[ra] == rank[rb] {
                    rank[ra] += 1;
                }
            }
        }
    }

    let mut groups: std::collections::HashMap<usize, Vec<usize>> = std::collections::HashMap::new();
    for i in 0..pillars.len() {
        groups.entry(find(&mut parent, i)).or_default().push(i);
    }

    let mut merged = Merged::default();
    for members in groups.values() {
        if members.len() < 2 {
            continue;
        }
        if let Some(group) = merge_group(pillars, members, mesh, bvh) {
            merged.links.extend(group.links);
            merged.trunk_bottoms.extend(group.trunk_bottoms);
        }
    }

    merged
}

/// One group's merge: a tree with a representative trunk and 45° links from the
/// other members.
struct Group {
    links: Vec<Part>,
    trunk_bottoms: Vec<(usize, f32)>,
}

/// Merges one group of members. The representative is the member nearest the
/// group centre; the merge height is the lowest height a 45° link from any
/// member can reach.
fn merge_group(pillars: &[Pillar], members: &[usize], mesh: &Mesh, bvh: &Bvh) -> Option<Group> {
    // Group centre and the representative member nearest it.
    let centre = members
        .iter()
        .map(|&i| pillars[i].xy)
        .fold(Vector2::zeros(), |acc, xy| acc + xy)
        / members.len() as f32;

    let &representative = members.iter().min_by(|&&a, &&b| {
        (pillars[a].xy - centre)
            .magnitude_squared()
            .total_cmp(&(pillars[b].xy - centre).magnitude_squared())
    })?;

    let rep = pillars[representative];
    let diameter = 2.0 * rep.radius;

    // Merge height: the lowest height any member's 45° link reaches on the
    // representative's pillar (min over members of member z − XY distance).
    let mut merge_z = f32::INFINITY;
    for &i in members {
        let dist = (pillars[i].xy - rep.xy).magnitude();
        merge_z = merge_z.min(pillars[i].tip_z - dist);
    }

    // Cancel below ground + max(2 × D + 5 mm, base height + 2 × D).
    let floor = (2.0 * diameter + 5.0).max(1.0 + 2.0 * diameter);
    if merge_z < floor {
        return None;
    }

    let radius = members
        .iter()
        .map(|&i| pillars[i].radius)
        .fold(f32::INFINITY, f32::min);

    let mut links = Vec::new();
    let mut trunk_bottoms = Vec::new();

    for &i in members {
        // The representative is the trunk: it keeps its full pillar to the
        // plate, so it is neither truncated nor linked to itself.
        if i == representative {
            continue;
        }

        // A member's 45° link runs from its tip end down to the representative's
        // pillar at `tip_z − run`.
        let run = (pillars[i].xy - rep.xy).magnitude();
        let from = Vector3::new(pillars[i].xy.x, pillars[i].xy.y, pillars[i].tip_z);
        let to = Vector3::new(rep.xy.x, rep.xy.y, pillars[i].tip_z - run);

        // Collision test at 1.2 × radius; a failing member keeps its own pillar
        //.
        if collision::link_test(mesh, bvh, from, to, radius * LINK_RADIUS_FACTOR) {
            continue;
        }

        links.push(Part {
            kind: PartKind::Link,
            a: from,
            b: to,
            radius_a: radius,
            radius_b: radius,
            shape: Shape::Round,
        });
        // The member's own column stops at the merge point; the trunk carries
        // it from there.
        trunk_bottoms.push((i, merge_z));
    }

    Some(Group {
        links,
        trunk_bottoms,
    })
}

/// The "merge selected" result: rebuilt parts that replace the old supports in
/// a merged group. Distinct from [`merge_pillars`]: here the members stack
/// near-vertically and the parts are rebuilt, not truncated and linked at a
/// fixed 45°.
#[derive(Clone, Debug, Default)]
pub struct PostMerged {
    /// Rebuilt link parts (one per non-representative member), from the member's
    /// top down to the merge point.
    pub links: Vec<Part>,
    /// Joint spheres at each merge point, at 1.04 × the merged pillar's radius.
    pub joints: Vec<Part>,
    /// New pillar parts from each merge point down to the old support's
    /// top.
    pub pillars: Vec<Part>,
}

/// Runs the "merge selected" merge over the extracted candidates,
/// grouping by the 0.6 × R neighbourhood and a 0.1 mm tip-end z tolerance, then
/// building the rebuild parts for each group. Returns `None` groups omitted.
///
/// Merge height: `zm = min(rep_top, member_top)`, capped by
/// `min over members of (member.z − XY distance to the representative)`.
pub fn post_merge(candidates: &[(usize, Pillar)], r: f32, mesh: &Mesh, bvh: &Bvh) -> PostMerged {
    let group_radius = GROUP_RADIUS_FACTOR * r;
    let pillars: Vec<Pillar> = candidates.iter().map(|&(_, p)| p).collect();

    // Group by 0.6 × R in XY and within 0.1 mm in tip-end z.
    let mut parent: Vec<usize> = (0..pillars.len()).collect();
    let mut rank = vec![0usize; pillars.len()];

    fn find(parent: &mut [usize], x: usize) -> usize {
        let mut root = x;
        while parent[root] != root {
            root = parent[root];
        }
        let mut cur = x;
        while parent[cur] != cur {
            let next = parent[cur];
            parent[cur] = root;
            cur = next;
        }
        root
    }

    for a in 0..pillars.len() {
        for b in (a + 1)..pillars.len() {
            if (pillars[a].xy - pillars[b].xy).magnitude() > group_radius {
                continue;
            }
            if (pillars[a].tip_z - pillars[b].tip_z).abs() > GROUP_Z_TOLERANCE {
                continue;
            }
            let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
            if ra != rb {
                if rank[ra] < rank[rb] {
                    parent[ra] = rb;
                } else {
                    parent[rb] = ra;
                    if rank[ra] == rank[rb] {
                        rank[ra] += 1;
                    }
                }
            }
        }
    }

    let mut groups: std::collections::HashMap<usize, Vec<usize>> = std::collections::HashMap::new();
    for i in 0..pillars.len() {
        groups.entry(find(&mut parent, i)).or_default().push(i);
    }

    let mut out = PostMerged::default();
    for members in groups.values() {
        if members.len() < 2 {
            continue;
        }
        if let Some(group) = post_merge_group(&pillars, members, mesh, bvh) {
            out.links.extend(group.links);
            out.joints.extend(group.joints);
            out.pillars.extend(group.pillars);
        }
    }
    out
}

/// One group's rebuild. The representative is the member nearest the group
/// centre; the merge height stacks the members' tops and is capped by the 45°
/// reach of the lowest member.
fn post_merge_group(
    pillars: &[Pillar],
    members: &[usize],
    mesh: &Mesh,
    bvh: &Bvh,
) -> Option<PostMerged> {
    let centre = members
        .iter()
        .map(|&i| pillars[i].xy)
        .fold(Vector2::zeros(), |acc, xy| acc + xy)
        / members.len() as f32;
    let &representative = members.iter().min_by(|&&a, &&b| {
        (pillars[a].xy - centre)
            .magnitude_squared()
            .total_cmp(&(pillars[b].xy - centre).magnitude_squared())
    })?;
    let rep = pillars[representative];

    // The 45° lower bound: the lowest z any member can reach on the trunk.
    let mut cap = f32::INFINITY;
    for &i in members {
        let dist = (pillars[i].xy - rep.xy).magnitude();
        cap = cap.min(pillars[i].top_z - dist);
    }

    let radius = members
        .iter()
        .map(|&i| pillars[i].radius)
        .fold(f32::INFINITY, f32::min);

    let mut out = PostMerged::default();
    for &i in members {
        if i == representative {
            continue;
        }
        // Merge height: min(rep_top, member_top) capped by the 45° bound.
        let zm = rep.top_z.min(pillars[i].top_z).min(cap);
        let from = Vector3::new(pillars[i].xy.x, pillars[i].xy.y, pillars[i].top_z);
        let to = Vector3::new(rep.xy.x, rep.xy.y, zm);

        // Link collision test at 1.2 × radius.
        if collision::link_test(mesh, bvh, from, to, radius * LINK_RADIUS_FACTOR) {
            continue;
        }

        out.links.push(Part {
            kind: PartKind::Link,
            a: from,
            b: to,
            radius_a: radius,
            radius_b: radius,
            shape: Shape::Round,
        });
        out.joints.push(Part {
            kind: PartKind::Joint,
            a: to,
            b: to,
            radius_a: radius * JOINT_RADIUS_FACTOR,
            radius_b: radius * JOINT_RADIUS_FACTOR,
            shape: Shape::Round,
        });
        // The new pillar runs from the merge point down to the old
        // support's top. The representative's own column top is
        // that top.
        out.pillars.push(Part {
            kind: PartKind::Pillar,
            a: to,
            b: Vector3::new(rep.xy.x, rep.xy.y, rep.top_z),
            radius_a: radius,
            radius_b: radius,
            shape: Shape::Round,
        });
    }

    (!out.links.is_empty()).then_some(out)
}

#[cfg(test)]
mod tests {
    use common::progress::Progress;
    use nalgebra::{Vector2, Vector3};
    use slicer::geometry::{mesh::Mesh, partitioning::Bvh};

    use super::*;

    fn empty_mesh() -> (Mesh, Bvh) {
        // A tiny box far from the pillars so no link collides.
        let mut verts = Vec::new();
        let mut faces = Vec::new();
        let v = Vector3::new;
        let base = verts.len() as u32;
        for i in 0..8 {
            let pick = |bit, lo: f32, hi: f32| if i & bit == 0 { lo } else { hi };
            verts.push(v(pick(1, -1.0, 1.0), pick(2, -1.0, 1.0), pick(4, 0.0, 1.0)));
        }
        #[rustfmt::skip]
        let quads = [
            [0u32, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4],
            [2, 6, 7, 3], [0, 4, 6, 2], [1, 3, 7, 5],
        ];
        for [a, b, c, d] in quads {
            faces.push([base + a, base + b, base + c]);
            faces.push([base + a, base + c, base + d]);
        }
        let mesh = Mesh::new(verts, faces);
        let bvh = Bvh::build(mesh.inner(), Progress::new());
        (mesh, bvh)
    }

    #[test]
    fn no_merge_when_pillars_are_far_apart() {
        let (mesh, bvh) = empty_mesh();
        let pillars = vec![
            Pillar::plate(Vector2::new(0.0, 0.0), 20.0, 20.0, 0.6),
            Pillar::plate(Vector2::new(50.0, 0.0), 20.0, 20.0, 0.6),
        ];
        let merged = merge_pillars(&pillars, MERGE_R, &mesh, &bvh);
        assert!(merged.links.is_empty());
        assert!(merged.trunk_bottoms.is_empty());
    }

    #[test]
    fn nearby_pillars_merge_into_a_tree() {
        let (mesh, bvh) = empty_mesh();
        // Three pillars within the 0.6 × R (1.2 mm) neighbourhood.
        let pillars = vec![
            Pillar::plate(Vector2::new(0.0, 0.0), 20.0, 20.0, 0.6),
            Pillar::plate(Vector2::new(1.0, 0.0), 20.0, 20.0, 0.6),
            Pillar::plate(Vector2::new(0.5, 0.86), 20.0, 20.0, 0.6),
        ];
        let merged = merge_pillars(&pillars, MERGE_R, &mesh, &bvh);
        // Three members: two non-representative members stop at the merge point
        // and link into the trunk; the representative keeps its full pillar.
        assert_eq!(merged.trunk_bottoms.len(), 2);
        assert_eq!(merged.links.len(), 2);
        assert!(merged.links.iter().all(|p| p.kind == PartKind::Link));
        // Every link is a 45° descent: drop equals run.
        for link in &merged.links {
            let run = (link.a.xy() - link.b.xy()).magnitude();
            let drop = link.a.z - link.b.z;
            assert!((drop - run).abs() < 1e-3, "link not 45°: {drop} vs {run}");
        }
    }

    #[test]
    fn extract_keeps_vertical_pillars_and_skips_slims() {
        use crate::supports::{SupportId, SupportSet};

        let mut set = SupportSet::default();
        // A normal routed support (vertical pillar, radius 0.6).
        set.add_routed(
            SupportId::new(),
            0.2,
            0.6,
            0.4,
            [
                Vector3::new(0.0, 0.0, 20.0),
                Vector3::new(0.0, 0.0, 18.0),
                Vector3::new(0.0, 0.0, 18.0),
            ],
        );
        // A second routed support slightly offset in XY.
        set.add_routed(
            SupportId::new(),
            0.2,
            0.6,
            0.4,
            [
                Vector3::new(1.0, 0.0, 20.0),
                Vector3::new(1.0, 0.0, 18.0),
                Vector3::new(1.0, 0.0, 18.0),
            ],
        );

        let candidates = extract_merge_candidates(&set.supports);
        assert_eq!(candidates.len(), 2);
        for (src, pillar) in &candidates {
            assert_eq!(set.supports[*src].parts.len(), 3); // tip, joint, pillar
            assert!(pillar.radius > SLIM_RADIUS);
        }
    }

    #[test]
    fn post_merge_stacks_near_vertically() {
        let (mesh, bvh) = empty_mesh();
        // Two nearby pillars with a lateral-link offset: the tip end (tip_z)
        // sits above the pillar top (top_z). "Merge selected" stacks from
        // `top_z` (the near-common pillar-joint height), not the pre-merge's
        // 45° slope from `tip_z`.
        let pillars = [
            Pillar::plate(Vector2::new(0.0, 0.0), 18.0, 20.0, 0.6),
            Pillar::plate(Vector2::new(1.0, 0.0), 18.0, 20.0, 0.6),
        ];
        let candidates: Vec<(usize, Pillar)> = pillars.iter().copied().enumerate().collect();
        let merged = post_merge(&candidates, MERGE_R, &mesh, &bvh);
        assert_eq!(merged.links.len(), 1);
        assert_eq!(merged.joints.len(), 1);
        assert_eq!(merged.pillars.len(), 1);

        // The link starts at the pillar TOP (18.0), not the tip end (20.0):
        // the stack uses `top_z`, unlike the pre-merge's 45° link from `tip_z`.
        let link = &merged.links[0];
        assert!(
            (link.a.z - 18.0).abs() < 1e-3,
            "link starts at top_z: {:?}",
            link.a.z
        );
        // The joint is at 1.04 × the pillar radius.
        let joint = &merged.joints[0];
        assert!((joint.radius_a - 0.6 * JOINT_RADIUS_FACTOR).abs() < 1e-6);
        // The new pillar runs from the merge point down to the representative's
        // top (18.0), not to the plate at z = 0.
        let pillar = &merged.pillars[0];
        assert!(
            (pillar.b.z - 18.0).abs() < 1e-3,
            "pillar ends at rep top: {:?}",
            pillar.b.z
        );
    }
}
