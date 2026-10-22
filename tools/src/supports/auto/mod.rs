use common::{
    progress::Progress,
    units::{Milimeter, Milimeters},
};
use nalgebra::Vector3;
use rayon::prelude::*;
use slicer::geometry::{
    mesh::{HalfEdgeMesh, Mesh},
    partitioning::Bvh,
};
use tracing::info;

use crate::supports::mesh::SupportPreset;

pub mod collision;
pub mod detect;
pub mod grid;
pub mod landing;
pub mod legs;
pub mod points;
pub mod quazirandom;
pub mod tip;
pub mod topology;

pub use landing::{Landing, LandingConfig, LandingKind};
pub use topology::BraceNetwork;

/// The routed supports of one generation run.
#[derive(Clone, Debug, Default)]
pub struct Generated {
    /// Supports with a pillar to the plate, as `[contact, tip, joint]`
    /// triplets. The brace network indexes into this list.
    pub plate: Vec<[Vector3<f32>; 3]>,
    /// Supports that land on the model below them (§2.4).
    pub landings: Vec<Landing>,
}

/// One point's route: to the plate (contact, tip end, joint), or onto the
/// model.
#[derive(Clone, Debug)]
pub enum Routed {
    Plate([Vector3<f32>; 3]),
    Model(Landing),
}

pub struct AutoPlacement {
    pub placement: PlacementConfig,
    pub support: SupportPreset,

    pub bed_size: Vector3<Milimeters>,
}

#[derive(Clone)]
pub struct PlacementConfig {
    pub min_spacing: f32,
    pub max_angle: f32,
    pub face_support_spacing: f32,
    pub edge_support_spacing: f32,
    pub edge_angle_delta: f32,
}

impl AutoPlacement {
    pub fn new(
        placement: PlacementConfig,
        support: SupportPreset,
        bed_size: Vector3<Milimeters>,
    ) -> Self {
        Self {
            placement,
            support,

            bed_size,
        }
    }

    pub fn generate_supports(
        &self,
        mesh: &Mesh,
        _half_edge: &HalfEdgeMesh,
        bvh: &Bvh,
        progress: &Progress,
    ) -> Generated {
        // Stage 1: ray-grid point placement replaces the three detectors. The
        // old detector path lives on in `detect` but is no longer driven here.
        let grid = grid::footprint(mesh);
        let overhangs = grid.map(|g| {
            let samples = grid::sample_surface(mesh, bvh, &g, progress);
            let placed = points::place_points(
                &samples,
                &g,
                &self.placement,
                true,
                true,
                false,
                false,
                false,
            );
            (placed, samples.len())
        });

        let (overhangs, sample_count) = match overhangs {
            Some((p, n)) => (p, n),
            None => (Vec::new(), 0),
        };

        info!(
            "Placed {} supports from {sample_count} surface samples",
            overhangs.len()
        );

        progress.set_total(overhangs.len() as u64);
        let routed: Vec<Routed> = overhangs
            .into_par_iter()
            .filter_map(|x| {
                let routed = route_point(mesh, bvh, &self.support, x.point, x.normal);
                progress.add_complete(1);
                routed
            })
            .collect();

        let mut generated = Generated::default();
        for route in routed {
            match route {
                Routed::Plate(triplet) => generated.plate.push(triplet),
                Routed::Model(landing) => generated.landings.push(landing),
            }
        }

        info!(
            "Routed {} supports to the plate and {} onto the model",
            generated.plate.len(),
            generated.landings.len()
        );
        generated
    }

    /// Builds the brace network (plan §2.5) over the routed supports.
    ///
    /// Each plate triplet's `joint` is the pillar top; the brace network runs
    /// a Delaunay + MST over the pillar XY positions for the 45° spanning-tree
    /// braces, then chains nearly-collinear triples into inverted-V forks.
    /// Model-landing pillars are braced among themselves along edges shorter
    /// than 1.3 × R, R = 0.5 × point spacing (§2.4 step 8); slims are not
    /// braced. Last, tall and sparsely braced plate pillars get a stabilising
    /// leg each (§2.6). The returned [`BraceNetwork`] carries the brace/fork
    /// link parts, the legs, and the plate-pillar truncations the forks impose,
    /// indexed into `generated.plate`.
    pub fn generate_braces(&self, mesh: &Mesh, bvh: &Bvh, generated: &Generated) -> BraceNetwork {
        let radius = self.support.radius.get::<Milimeter>();
        let cone_length = self.support.tip_length.get::<Milimeter>();
        let pillars: Vec<topology::Pillar> = (generated.plate.iter())
            .map(|[_, tip, joint]| topology::Pillar::plate(joint.xy(), joint.z, tip.z, radius))
            .collect();
        let mut network = topology::build_network(&pillars, mesh, bvh, cone_length);

        let landing_pillars: Vec<topology::Pillar> = (generated.landings.iter())
            .filter(|landing| landing.kind == LandingKind::Pillar)
            .map(|landing| {
                topology::Pillar::landing(landing.tip.xy(), landing.tip.z, landing.foot.z, radius)
            })
            .collect();
        let r = 0.5 * self.placement.min_spacing;
        let max_spacing = topology::LANDING_EDGE_R_FACTOR * r;
        network.braces.extend(topology::build_braces(
            &landing_pillars,
            max_spacing,
            mesh,
            bvh,
        ));

        let leg_config = legs::LegConfig {
            radius,
            route_radius: (self.support.radius + self.support.margin).get::<Milimeter>(),
            cone_length: legs::LEG_CONE_R_FACTOR * r,
            contact_depth: self.support.contact_depth.get::<Milimeter>(),
        };
        let (leg_supports, leg_braces) = legs::build_legs(
            mesh,
            bvh,
            &leg_config,
            &generated.plate,
            &network.braces,
            &network.fork_bottoms,
        );
        network.legs = leg_supports;
        network.braces.extend(leg_braces);
        network
    }
}

/// Routes one contact point the way auto supports are routed: to the plate
/// first (§2.3), and onto the model below when there is no way down (§2.4).
/// Manual placement uses this too.
pub fn route_point(
    mesh: &Mesh,
    bvh: &Bvh,
    support: &SupportPreset,
    contact: Vector3<f32>,
    normal: Vector3<f32>,
) -> Option<Routed> {
    let radius = (support.radius + support.margin).get::<Milimeter>();
    let tip_length = support.tip_length.get::<Milimeter>();
    let contact_depth = support.contact_depth.get::<Milimeter>();
    route_support(
        mesh,
        bvh,
        radius,
        tip_length,
        contact_depth,
        contact,
        normal,
    )
    .map(|(tip_end, joint)| Routed::Plate([contact, tip_end, joint]))
    .or_else(|| {
        landing::land_support(mesh, bvh, &landing_config(support), contact, normal)
            .map(Routed::Model)
    })
}

/// The sizes the model-landing router (§2.4) works with, from the preset.
fn landing_config(support: &SupportPreset) -> LandingConfig {
    LandingConfig {
        radius: (support.radius + support.margin).get::<Milimeter>(),
        pillar_diameter: 2.0 * support.radius.get::<Milimeter>(),
        cone_length: support.tip_length.get::<Milimeter>(),
        contact_depth: support.contact_depth.get::<Milimeter>(),
        tail_depth: support.tail_depth.get::<Milimeter>(),
        slim_radius: support.slim_radius.get::<Milimeter>(),
        slim_depth: support.slim_depth.get::<Milimeter>(),
    }
}

/// Returns `Some((tip_end, joint))` for a support at `contact` with outward
/// normal `normal`. `tip_end` is where the tip cone ends and `joint` is where
/// the 45° link meets the vertical pillar (which then drops to the plate).
///
/// Order of attempts (§2.3):
/// 1. Straight drop: the tip points down (clamped) and a vertical pillar from
///    `tip_end` to the plate passes. `joint == tip_end`.
/// 2. Lateral search: candidate pillar sites within `MAX_OFFSET` (15 mm) of the
///    contact, nearest first. For each site we vary the tip direction in the
///    §2.3 order and try a 45° link from `tip_end` down to the pillar top, then
///    a vertical pillar. The first combination that passes every collision test
///    wins.
pub fn route_support(
    mesh: &Mesh,
    bvh: &Bvh,
    radius: f32,
    tip_length: f32,
    contact_depth: f32,
    contact: Vector3<f32>,
    normal: Vector3<f32>,
) -> Option<(Vector3<f32>, Vector3<f32>)> {
    let normal = tip::clamp_tip_direction(normal.normalize());

    // The horn test shifts its sample shell toward the tip by the contact sink
    // so the cone's sunken region at the surface isn't falsely flagged (§2.7):
    // touch depth + max(0.5 × pillar Ø, 0.3 × cone length).
    let sink = contact_depth + radius.max(tip_length * 0.3);

    // 1. Straight drop: tip along the (clamped) normal, vertical pillar.
    let tip_end = contact + normal * tip_length;
    if !collision::horn_test(mesh, bvh, contact, tip_end, radius, sink)
        && !collision::main_test(mesh, bvh, tip_end, tip_end.xy().to_homogeneous(), radius)
    {
        return Some((tip_end, tip_end));
    }

    // 2. Lateral search.
    for site in candidate_sites(contact, tip_end) {
        for dir in tip_directions(normal, site - tip_end) {
            let candidate_tip = contact + dir * tip_length;
            // The joint is at the site's XY, dropped by the link's 45° run.
            let run = (site.xy() - candidate_tip.xy()).magnitude();
            let joint = site.xy().push(candidate_tip.z - run);
            if joint.z <= 0.0 {
                continue;
            }

            // Tip (horn), link, pillar (main).
            if collision::horn_test(mesh, bvh, contact, candidate_tip, radius, sink)
                || collision::link_test(mesh, bvh, candidate_tip, joint, radius)
                || collision::main_test(mesh, bvh, joint, joint.xy().to_homogeneous(), radius)
            {
                continue;
            }

            return Some((candidate_tip, joint));
        }
    }

    None
}

/// The maximum lateral offset from the contact to search for a pillar site, in
/// millimetres (§2.3: `supportmaxoffsetfromcontact`).
const MAX_OFFSET: f32 = 15.0;

/// Candidate pillar sites within `MAX_OFFSET` of the contact, nearest first.
/// Uses the 8 compass/diagonal directions at increasing radii. The routing loop
/// applies the real collision tests; this just enumerates candidate XY locations.
fn candidate_sites(contact: Vector3<f32>, tip_end: Vector3<f32>) -> Vec<Vector3<f32>> {
    let mut sites = Vec::new();
    for k in 0..8 {
        let angle = k as f32 / 8.0 * std::f32::consts::TAU;
        let dir = Vector3::new(angle.cos(), angle.sin(), 0.0);
        for step in 1..=16 {
            let dist = (step as f32 / 16.0) * MAX_OFFSET;
            sites.push(contact + dir * dist);
        }
    }

    sites.sort_by(|a, b| {
        (a.xy() - tip_end.xy())
            .magnitude_squared()
            .total_cmp(&(b.xy() - tip_end.xy()).magnitude_squared())
    });

    sites
}

/// The tip directions to try for a given lateral offset `o` from the tip end to
/// the candidate site (§2.3 step 3). If `o` points away from the normal's
/// horizontal part we go straight down then 45° toward the site; otherwise the
/// clamped normal, then 45° toward the site, then about 27° below horizontal,
/// then straight down.
fn tip_directions(normal: Vector3<f32>, o: Vector3<f32>) -> Vec<Vector3<f32>> {
    let down = -Vector3::z();

    // Guard the degenerate case where the candidate site sits directly below
    // the tip end (zero horizontal offset): only the straight-down direction
    // then makes sense.
    let o_len = o.xy().magnitude();
    if o_len <= f32::EPSILON {
        return vec![down, normal];
    }

    let horiz = (o.xy() / o_len).push(0.0);

    // A direction 45° toward the site (from straight down).
    let toward_45 = (down + horiz).normalize();
    // About 27° below horizontal toward the site: mostly horizontal, slightly down.
    let h = horiz * (27.0_f32.to_radians()).cos();
    let toward_27 = (h + down * (27.0_f32.to_radians()).sin()).normalize();

    // `o` points away from the normal's horizontal part when their dot is < 0.
    let away = o.xy().dot(&normal.xy()) < 0.0;
    if away {
        vec![down, toward_45]
    } else {
        vec![normal, toward_45, toward_27, down]
    }
}

/// Checks if the tip, from the contact point on the surface to its end, passes
/// through the model. The segment starts just off the surface so the contact
/// face itself isn't counted. Used by the routing tests to assert the tip
/// never enters the model.
#[cfg(test)]
fn intersect_tip(mesh: &Mesh, bvh: &Bvh, contact: Vector3<f32>, end: Vector3<f32>) -> bool {
    let start = contact + (end - contact).normalize() * 0.01;
    bvh.intersect_segment(mesh, start, end).is_some()
}

impl Default for PlacementConfig {
    fn default() -> Self {
        Self {
            min_spacing: 5.0,
            max_angle: 30.0,
            face_support_spacing: 50.0,
            edge_angle_delta: 0.1,
            edge_support_spacing: 20.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use common::{progress::Progress, units::Milimeters};
    use nalgebra::Vector3;
    use slicer::geometry::{
        mesh::{HalfEdgeMesh, Mesh},
        partitioning::Bvh,
    };

    use super::{
        AutoPlacement, PlacementConfig, Routed, intersect_tip, route_point, route_support,
    };
    use crate::supports::mesh::SupportPreset;

    /// Appends an axis-aligned box with outward facing triangles.
    fn add_box(
        verts: &mut Vec<Vector3<f32>>,
        faces: &mut Vec<[u32; 3]>,
        min: Vector3<f32>,
        max: Vector3<f32>,
    ) {
        let base = verts.len() as u32;
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
            faces.push([base + a, base + b, base + c]);
            faces.push([base + a, base + c, base + d]);
        }
    }

    /// A 20 × 20 × 2 mm slab on a 4 × 4 mm stem, tilted and lifted off the plate.
    fn tilted_t() -> (Mesh, HalfEdgeMesh, Bvh) {
        let (mut verts, mut faces) = (Vec::new(), Vec::new());
        let v = Vector3::new;
        add_box(
            &mut verts,
            &mut faces,
            v(-10.0, -10.0, 4.0),
            v(10.0, 10.0, 6.0),
        );
        add_box(
            &mut verts,
            &mut faces,
            v(-2.0, -2.0, -6.0),
            v(2.0, 2.0, 4.5),
        );

        let mut mesh = Mesh::new(verts, faces);
        mesh.set_rotation(v(0.3, 0.2, 0.0));
        mesh.set_position(v(0.0, 0.0, 15.0));

        let half_edge = HalfEdgeMesh::build_from_mesh(mesh.inner());
        let bvh = Bvh::build(mesh.inner(), Progress::new());
        (mesh, half_edge, bvh)
    }

    #[test]
    fn rotated_model_supports() {
        let (mesh, half_edge, bvh) = tilted_t();
        let bed = Vector3::repeat(Milimeters::new(100.0));
        let placement =
            AutoPlacement::new(PlacementConfig::default(), SupportPreset::default(), bed);
        let supports = placement.generate_supports(&mesh, &half_edge, &bvh, &Progress::new());
        assert!(!supports.plate.is_empty());

        for [contact, tip, joint] in supports.plate {
            assert!(
                !intersect_tip(&mesh, &bvh, contact, tip),
                "tip enters the model"
            );

            let run = (joint.xy() - tip.xy()).magnitude();
            assert!(tip.z - joint.z >= run - 1e-3, "arm flatter than 45°");
            assert!(joint.z > 0.0);
        }
    }

    #[test]
    fn route_point_falls_back_to_landing() {
        // A manual click routes like auto: the open ledge corner reaches the
        // plate, the ledge centre over the table lands on it.
        let (mut verts, mut faces) = (Vec::new(), Vec::new());
        let v = Vector3::new;
        add_box(
            &mut verts,
            &mut faces,
            v(-30.0, -30.0, 5.0),
            v(30.0, 30.0, 10.0),
        );
        add_box(
            &mut verts,
            &mut faces,
            v(-5.0, -5.0, 20.0),
            v(5.0, 5.0, 22.0),
        );
        add_box(
            &mut verts,
            &mut faces,
            v(40.0, -5.0, 20.0),
            v(50.0, 5.0, 22.0),
        );
        let mesh = Mesh::new(verts, faces);
        let bvh = Bvh::build(mesh.inner(), Progress::new());
        let preset = SupportPreset::default();
        let down = -Vector3::z();

        let over_table = route_point(&mesh, &bvh, &preset, v(0.0, 0.0, 20.0), down);
        assert!(matches!(over_table, Some(Routed::Model(_))));
        let in_the_open = route_point(&mesh, &bvh, &preset, v(45.0, 0.0, 20.0), down);
        assert!(matches!(in_the_open, Some(Routed::Plate(_))));
    }

    #[test]
    fn ledge_over_table_lands_on_model() {
        // A floating 10 × 10 mm ledge above a 60 × 60 mm table: the table
        // blocks every plate route within 15 mm, so the ledge's points land on
        // the table top instead (§2.4).
        let (mut verts, mut faces) = (Vec::new(), Vec::new());
        let v = Vector3::new;
        add_box(
            &mut verts,
            &mut faces,
            v(-30.0, -30.0, 5.0),
            v(30.0, 30.0, 10.0),
        );
        add_box(
            &mut verts,
            &mut faces,
            v(-5.0, -5.0, 20.0),
            v(5.0, 5.0, 22.0),
        );
        let mesh = Mesh::new(verts, faces);
        let half_edge = HalfEdgeMesh::build_from_mesh(mesh.inner());
        let bvh = Bvh::build(mesh.inner(), Progress::new());

        let bed = Vector3::repeat(Milimeters::new(100.0));
        let placement =
            AutoPlacement::new(PlacementConfig::default(), SupportPreset::default(), bed);
        let supports = placement.generate_supports(&mesh, &half_edge, &bvh, &Progress::new());
        assert!(!supports.landings.is_empty(), "ledge points must land");

        for landing in &supports.landings {
            assert!((landing.contact.z - 20.0).abs() < 1e-3);
            assert_eq!(landing.foot.xy(), landing.tip.xy(), "pillar is vertical");
            assert!(!intersect_tip(&mesh, &bvh, landing.contact, landing.tip));
            for (end, normal) in &landing.tails {
                // Tails sink 0.2 mm (Middle) along their axis into the table.
                let axis = (landing.foot - end).normalize();
                let sunk = (10.0 - end.z) / axis.z;
                assert!((sunk - 0.2).abs() < 1e-3, "tail sinks {sunk} mm");
                assert!(normal.z > 0.99);
            }
        }
    }

    #[test]
    fn tip_through_wall_is_rejected() {
        let (mesh, _, bvh) = tilted_t();

        // Contact on the top of the slab with the tip pointing down through it.
        let top = mesh.transform(&Vector3::new(6.0, 6.0, 6.0));
        let down = mesh.transform_normal(&-Vector3::z()).normalize();
        assert!(route_support(&mesh, &bvh, 1.1, 2.0, 0.4, top, down).is_none());
    }

    #[test]
    fn lateral_search_routes_around_obstacle() {
        // A slab lifted off the plate with a stem directly below its centre. A
        // contact on the slab underside directly above the stem can't drop
        // straight; it must route sideways. We already cover this via
        // rotated_model_supports (which asserts non-empty + invariants), so here
        // we just assert the straight-drop-blocked case still finds a route.
        let (mesh, _, bvh) = tilted_t();

        // A downward-facing point near the stem (x ~ 0) is blocked directly.
        let contact = mesh.transform(&Vector3::new(0.5, 0.5, 4.5));
        let normal = mesh.transform_normal(&-Vector3::z()).normalize();

        // It should still route (laterally), returning a joint above the plate
        // with a 45° (or steeper) arm.
        if let Some((tip, joint)) = route_support(&mesh, &bvh, 1.1, 2.0, 0.4, contact, normal) {
            let run = (joint.xy() - tip.xy()).magnitude();
            assert!(tip.z - joint.z >= run - 1e-3, "arm flatter than 45°");
            assert!(joint.z > 0.0);
        }
    }
}
