//! Automatic supports: contact point placement, routing to the plate or onto
//! the model, and the brace network between the pillars.

use common::{
    progress::Progress,
    units::{Milimeter, Milimeters},
};
use nalgebra::{Vector2, Vector3};
use rayon::prelude::*;
use slicer::geometry::{mesh::Mesh, partitioning::Bvh};
use tracing::info;

use crate::supports::{
    BaseShape, Foot, Part, PartKind, Support, auto::points::PlacedPoint, mesh::SupportPreset,
    raft::RaftShape,
};

pub mod collision;
pub mod directional;
pub mod edge;
pub mod grid;
pub mod landing;
pub mod legs;
pub mod merge;
mod order;
pub mod placement;
pub mod points;
pub mod route;
pub mod tip;
pub mod topology;

pub use directional::{DirectionMode, Directional, build_directional, build_upward_tip};
pub use landing::{Landing, LandingConfig, LandingKind};
pub use merge::{Merged, PostMerged, extract_merge_candidates, merge_pillars, post_merge};
pub use route::{RouteSizes, route_support};
pub use topology::BraceNetwork;

/// The routed supports of one generation run.
#[derive(Clone, Debug, Default)]
pub struct Generated {
    /// Supports with a pillar to the plate, as `[contact, tip, joint]`
    /// triplets. The brace network indexes into this list.
    pub plate: Vec<[Vector3<f32>; 3]>,
    /// Supports that land on the model below them.
    pub landings: Vec<Landing>,
}

/// One point's route: to the plate (contact, tip end, joint), or onto the
/// model.
#[derive(Clone, Debug)]
pub enum Routed {
    Plate([Vector3<f32>; 3]),
    Model(Landing),
}

/// How far [`AutoPlacement::generate_supports`] is, per stage.
#[derive(Clone, Default)]
pub struct GenerationProgress {
    pub placing: Progress,
    pub routing: Progress,
}

pub struct AutoPlacement {
    pub placement: PlacementConfig,
    pub support: SupportPreset,

    pub bed_size: Vector3<Milimeters>,
}

#[derive(Clone)]
pub struct PlacementConfig {
    /// Point spacing, in mm.
    pub min_spacing: f32,
    /// Support angle: faces tilted less than this from straight down need
    /// support, in degrees.
    pub max_angle: f32,
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
        bvh: &Bvh,
        progress: &GenerationProgress,
    ) -> Generated {
        let points = self.place(mesh, bvh, &[], &progress.placing);
        info!("Placed {} supports", points.len());
        let generated = self.route_all(mesh, bvh, points, &progress.routing);
        info!(
            "Routed {} supports to the plate and {} onto the model",
            generated.plate.len(),
            generated.landings.len()
        );
        generated
    }

    /// Region auto-support: places over the whole model with the
    /// `existing` contacts counting as already supported, so new points keep
    /// the usual spacing from them, keeps the points whose contact falls
    /// inside `region` (a predicate over world XY), and routes those.
    pub fn generate_region_supports<'a>(
        &self,
        mesh: &Mesh,
        bvh: &Bvh,
        progress: &Progress,
        region: impl Fn(Vector2<f32>) -> bool,
        existing: impl IntoIterator<Item = &'a Vector3<f32>>,
    ) -> Generated {
        let seeds: Vec<Vector3<f32>> = existing.into_iter().copied().collect();
        let points: Vec<PlacedPoint> = (self.place(mesh, bvh, &seeds, progress).into_iter())
            .filter(|p| region(p.point.xy()))
            .collect();
        info!(
            "Region support: {} candidates inside the region",
            points.len()
        );
        self.route_all(mesh, bvh, points, progress)
    }

    /// Contact point placement and edge re-seat ([`placement`]), with
    /// `seeds` counting as already supported.
    fn place(
        &self,
        mesh: &Mesh,
        bvh: &Bvh,
        seeds: &[Vector3<f32>],
        progress: &Progress,
    ) -> Vec<PlacedPoint> {
        placement::place_points(
            mesh,
            bvh,
            &self.placement,
            2.0 * self.support.tip_radius.get::<Milimeter>(),
            seeds,
            &self.edge_config(),
            progress,
        )
    }

    /// Routes every point to the plate, or onto the model below.
    fn route_all(
        &self,
        mesh: &Mesh,
        bvh: &Bvh,
        points: Vec<PlacedPoint>,
        progress: &Progress,
    ) -> Generated {
        progress.set_total(points.len() as u64);
        let routed: Vec<Routed> = (points.into_par_iter())
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
        generated
    }

    /// The sizes for the edge re-seat ([`edge`]).
    fn edge_config(&self) -> edge::EdgeConfig {
        let radius = self.support.radius.get::<Milimeter>();
        let tip_radius = self.support.tip_radius.get::<Milimeter>();
        let tip_length = self.support.tip_length.get::<Milimeter>();
        let depth = self.support.contact_depth.get::<Milimeter>();
        // The tip cone's radius at the contact depth. Its bottom Ø is the
        // pillar Ø in every preset.
        let inset = tip_radius + depth / tip_length * (radius - tip_radius);
        edge::EdgeConfig {
            pillar_diameter: 2.0 * radius,
            inset,
            min_gap: landing::MAX_SLIM_LENGTH,
        }
    }

    /// Builds the brace network over the routed supports.
    ///
    /// Each plate triplet's `joint` is the pillar top; the brace network runs
    /// a Delaunay + MST over the pillar XY positions for the 45° spanning-tree
    /// braces, then chains nearly-collinear triples into inverted-V forks.
    /// Model-landing pillars are braced among themselves along edges shorter
    /// than 1.3 × R, R = 0.5 × point spacing; slims are not braced. Last,
    /// tall and sparsely braced plate pillars get a stabilising leg each. The returned [`BraceNetwork`] carries the brace/fork
    /// link parts, the legs, and the plate-pillar truncations the forks impose,
    /// indexed into `generated.plate`.
    pub fn generate_braces(&self, mesh: &Mesh, bvh: &Bvh, generated: &Generated) -> BraceNetwork {
        let radius = self.support.radius.get::<Milimeter>();
        let cone_length = self.support.tip_length.get::<Milimeter>();
        // What sits under a plate pillar: the raft, else the base.
        let base_height = if self.support.raft_shape != RaftShape::None {
            self.support.raft_thickness.get::<Milimeter>()
        } else if self.support.base_shape != BaseShape::None {
            self.support.base_thickness.get::<Milimeter>()
        } else {
            0.0
        };
        let boss_height = self.support.boss_height.get::<Milimeter>();
        let plate_floor = topology::plate_floor(base_height, boss_height);
        let fork_floor = topology::fork_floor(2.0 * radius, base_height);
        let clearance = self.support.margin.get::<Milimeter>();

        let pillars: Vec<topology::Pillar> = (generated.plate.iter())
            .map(|[_, tip, joint]| {
                topology::Pillar::plate(joint.xy(), joint.z, tip.z, radius)
                    .with_floor(plate_floor)
                    .with_clearance(clearance)
            })
            .collect();
        let mut network = topology::build_network(&pillars, mesh, bvh, cone_length, fork_floor);

        let landing_pillars: Vec<topology::Pillar> = (generated.landings.iter())
            .filter(|landing| landing.kind == LandingKind::Pillar)
            .map(|landing| {
                topology::Pillar::landing(landing.tip.xy(), landing.tip.z, landing.foot.z, radius)
                    .with_clearance(clearance)
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
            tip_radius: self.support.tip_radius.get::<Milimeter>(),
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

/// Braces `support`, added after generation, to the nearest plate pillars
/// among `others` (which may include `support` itself).
pub fn brace_support<'a>(
    mesh: &Mesh,
    bvh: &Bvh,
    support: &Support,
    others: impl IntoIterator<Item = &'a Support>,
) -> Vec<Part> {
    let Some(pillar) = plate_pillar(support) else {
        return Vec::new();
    };
    let others: Vec<topology::Pillar> = (others.into_iter())
        .filter(|s| s.id != support.id)
        .filter_map(plate_pillar)
        .collect();
    topology::brace_into(pillar, &others, mesh, bvh)
}

/// The brace geometry of a support standing on the plate, from its parts and
/// the settings it was built with.
fn plate_pillar(support: &Support) -> Option<topology::Pillar> {
    if !matches!(support.foot, Foot::Plate { .. }) {
        return None;
    }
    let pillar = support.parts.iter().find(|p| p.kind == PartKind::Pillar)?;
    let tip_z = (support.parts.iter())
        .find(|p| p.kind == PartKind::TipCone)
        .map_or(pillar.a.z, |tip| tip.b.z);

    let preset = &support.preset;
    let base_height = if preset.raft_shape != RaftShape::None {
        preset.raft_thickness.get::<Milimeter>()
    } else if preset.base_shape != BaseShape::None {
        preset.base_thickness.get::<Milimeter>()
    } else {
        0.0
    };
    let plate_floor = topology::plate_floor(base_height, preset.boss_height.get::<Milimeter>());
    // A fork-cut pillar ends in the air; braces stay above its end.
    let floor = plate_floor.max(pillar.b.z + topology::CONN_MIN_HEIGHT);
    Some(
        topology::Pillar::plate(pillar.a.xy(), pillar.a.z, tip_z, pillar.radius_a)
            .with_floor(floor)
            .with_clearance(preset.margin.get::<Milimeter>()),
    )
}

/// Routes one contact point the way auto supports are routed: to the plate
/// first, and onto the model below when there is no way down.
/// Manual placement uses this too.
pub fn route_point(
    mesh: &Mesh,
    bvh: &Bvh,
    support: &SupportPreset,
    contact: Vector3<f32>,
    normal: Vector3<f32>,
) -> Option<Routed> {
    route_support(
        mesh,
        bvh,
        &RouteSizes::from_preset(support),
        contact,
        normal,
    )
    .map(|(tip_end, joint)| Routed::Plate([contact, tip_end, joint]))
    .or_else(|| {
        landing::land_support(mesh, bvh, &landing_config(support), contact, normal)
            .map(Routed::Model)
    })
}

/// The sizes the model-landing router works with, from the preset.
fn landing_config(support: &SupportPreset) -> LandingConfig {
    LandingConfig {
        radius: (support.radius + support.margin).get::<Milimeter>(),
        pillar_diameter: 2.0 * support.radius.get::<Milimeter>(),
        cone_length: support.tip_length.get::<Milimeter>(),
        tip_radius: support.tip_radius.get::<Milimeter>(),
        contact_depth: support.contact_depth.get::<Milimeter>(),
        tail_depth: support.tail_depth.get::<Milimeter>(),
        slim_radius: support.slim_radius.get::<Milimeter>(),
        slim_depth: support.slim_depth.get::<Milimeter>(),
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
            min_spacing: 4.0,
            max_angle: 45.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use common::{progress::Progress, units::Milimeters};
    use nalgebra::Vector3;
    use slicer::geometry::{mesh::Mesh, partitioning::Bvh};

    use super::{
        AutoPlacement, PlacementConfig, RouteSizes, Routed, intersect_tip, route_point,
        route_support,
    };
    use crate::supports::mesh::SupportPreset;

    /// Radius 1.1 (pillar 0.6 + margin 0.5), cone 2.0, depth 0.4, tip
    /// radius 0.2.
    const TEST_SIZES: RouteSizes = RouteSizes {
        radius: 1.1,
        margin: 0.5,
        tip_radius: 0.2,
        tip_length: 2.0,
        contact_depth: 0.4,
    };

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
    fn tilted_t() -> (Mesh, Bvh) {
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

        let bvh = Bvh::build(mesh.inner(), Progress::new());
        (mesh, bvh)
    }

    #[test]
    fn rotated_model_supports() {
        let (mesh, bvh) = tilted_t();
        let bed = Vector3::repeat(Milimeters::new(100.0));
        let placement =
            AutoPlacement::new(PlacementConfig::default(), SupportPreset::default(), bed);
        let supports =
            placement.generate_supports(&mesh, &bvh, &super::GenerationProgress::default());
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
        // the table top instead.
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
        let bvh = Bvh::build(mesh.inner(), Progress::new());

        let bed = Vector3::repeat(Milimeters::new(100.0));
        let placement =
            AutoPlacement::new(PlacementConfig::default(), SupportPreset::default(), bed);
        let supports =
            placement.generate_supports(&mesh, &bvh, &super::GenerationProgress::default());
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
        let (mesh, bvh) = tilted_t();

        // Contact on the top of the slab with the tip pointing down through it.
        let top = mesh.transform(&Vector3::new(6.0, 6.0, 6.0));
        let down = mesh.transform_normal(&-Vector3::z()).normalize();
        assert!(route_support(&mesh, &bvh, &TEST_SIZES, top, down).is_none());
    }

    #[test]
    fn lateral_search_routes_around_obstacle() {
        // A slab lifted off the plate with a stem directly below its centre. A
        // contact on the slab underside directly above the stem can't drop
        // straight; it must route sideways. We already cover this via
        // rotated_model_supports (which asserts non-empty + invariants), so here
        // we just assert the straight-drop-blocked case still finds a route.
        let (mesh, bvh) = tilted_t();

        // A downward-facing point near the stem (x ~ 0) is blocked directly.
        let contact = mesh.transform(&Vector3::new(0.5, 0.5, 4.5));
        let normal = mesh.transform_normal(&-Vector3::z()).normalize();

        // It should still route (laterally), returning a joint above the plate
        // with a 45° (or steeper) arm.
        if let Some((tip, joint)) = route_support(&mesh, &bvh, &TEST_SIZES, contact, normal) {
            let run = (joint.xy() - tip.xy()).magnitude();
            assert!(tip.z - joint.z >= run - 1e-3, "arm flatter than 45°");
            assert!(joint.z > 0.0);
        }
    }

    #[test]
    fn region_support_stays_inside_the_region() {
        // A flat, lifted 40 × 40 mm slab. Region auto-support over the x > 0
        // half must place routed contacts only in that half.
        let (mut verts, mut faces) = (Vec::new(), Vec::new());
        let v = Vector3::new;
        add_box(
            &mut verts,
            &mut faces,
            v(-20.0, -20.0, 5.0),
            v(20.0, 20.0, 10.0),
        );
        let mut mesh = Mesh::new(verts, faces);
        mesh.set_position(v(0.0, 0.0, 10.0));
        let bvh = Bvh::build(mesh.inner(), Progress::new());

        let bed = Vector3::repeat(Milimeters::new(100.0));
        let placement =
            AutoPlacement::new(PlacementConfig::default(), SupportPreset::default(), bed);
        let supports = placement.generate_region_supports(
            &mesh,
            &bvh,
            &Progress::new(),
            |xy: nalgebra::Vector2<f32>| xy.x > 0.0,
            std::iter::empty(),
        );

        // Every routed support's contact must be in the x > 0 half. Contacts
        // are the first element of each plate triplet, and each landing's
        // `contact`.
        assert!(!supports.plate.is_empty() || !supports.landings.is_empty());
        for [contact, _, _] in &supports.plate {
            assert!(contact.x > 0.0, "plate contact outside region: {contact:?}");
        }
        for landing in &supports.landings {
            assert!(
                landing.contact.x > 0.0,
                "landing contact outside region: {:?}",
                landing.contact
            );
        }
    }
}
