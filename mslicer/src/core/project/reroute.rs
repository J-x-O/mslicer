//! Supports following their model to a new orientation, scale or height.
//!
//! A support's contact belongs to the model: it is a point on the model's
//! surface, in model space. When the model is turned, lifted or scaled, the
//! contact goes along, and the support is routed again from there:
//! hand-placed supports from their own contact, auto supports by generating
//! them again with the settings they were made with. A hand-placed support
//! that finds no route (its contact now faces up, lies elsewhere on a changed
//! surface, or nothing can reach it) keeps its old route and is marked as
//! stranded, so no work is lost silently.
//!
//! Nothing here moves the model: the auto supports are regenerated where the
//! model is, without the lift the Generate button applies.

use std::{collections::HashSet, sync::Arc};

use common::progress::Progress;
use slicer::geometry::{
    mesh::{Mesh, MeshId},
    partitioning::Bvh,
};
use tools::supports::{
    SupportId, SupportSet,
    auto::{AutoPlacement, GenerationProgress, brace_support, route_hand},
};

use crate::{
    core::project::supports::{Pose, auto_set, single},
    engine::CancelToken,
};

/// How far (mm) a contact may be from the surface after the change and still
/// count as on it.
const ON_SURFACE: f32 = 0.5;

/// What a re-route is for: the pose the supports were stored for, the pose
/// they go to, and the surface they are routed on (by identity).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RerouteKey {
    pub from: Pose,
    pub to: Pose,
    pub surface: MeshId,
}

/// The supports in the new pose.
pub struct Rerouted {
    pub auto: SupportSet,
    pub manual: SupportSet,
    /// Hand-placed supports that kept their old route.
    pub stranded: Vec<SupportId>,
}

pub struct RerouteInput {
    pub auto: Arc<SupportSet>,
    pub manual: Arc<SupportSet>,
    pub key: RerouteKey,
    /// The surface supports are routed on, placed at `key.to`.
    pub surface: Mesh,
    pub bvh: Option<Arc<Bvh>>,
    /// Regenerates the auto supports, if there were any.
    pub generator: Option<AutoPlacement>,
}

impl Pose {
    /// An empty mesh placed at this pose, for its transform.
    pub fn placed(&self) -> Mesh {
        let mut mesh = Mesh::new(Vec::new(), Vec::new());
        mesh.set_position_unchecked(self.position);
        mesh.set_scale_unchecked(self.scale);
        mesh.set_rotation_unchecked(self.rotation);
        mesh.update_transformation_matrix();
        mesh
    }
}

/// Routes the supports again for the new pose. `None` if cancelled.
pub fn reroute(input: RerouteInput, cancel: &CancelToken) -> Option<Rerouted> {
    let RerouteInput {
        auto,
        manual,
        key,
        surface,
        bvh,
        generator,
    } = input;
    let bvh = bvh.unwrap_or_else(|| Arc::new(Bvh::build(surface.inner(), Progress::new())));
    let (from, to) = (key.from.placed(), key.to.placed());
    cancel.check().ok()?;

    // The auto supports, generated again where the model is now.
    let mut new_auto = SupportSet::default();
    if let Some(generator) = generator.filter(|_| !auto.supports.is_empty()) {
        let progress = GenerationProgress {
            cancel: cancel.shared(),
            ..Default::default()
        };
        let generated = generator.generate_supports(&surface, &bvh, &progress);
        cancel.check().ok()?;
        let network = generator.generate_braces(&surface, &bvh, &generated);
        new_auto = auto_set(&generator.support, generated, network);
    }

    // The hand-placed ones, from their own contacts.
    let mut new_manual = SupportSet {
        lift: manual.lift,
        ..SupportSet::default()
    };
    let mut stranded = Vec::new();
    for support in &manual.supports {
        cancel.check().ok()?;
        let model = from.inv_transform(&support.point.pos);
        let moved = to.transform(&model);
        let routed = bvh.closest(&surface, moved).and_then(|hit| {
            let on = surface.transform(&hit.position);
            let normal = surface
                .transform_normal(&surface.normal(hit.face))
                .try_normalize(f32::EPSILON)?;
            // Only a face looking down needs holding up.
            let close = (on - moved).norm() <= ON_SURFACE;
            (close && normal.z < 0.0)
                .then(|| route_hand(&surface, &bvh, &support.preset, on, normal, false))
                .flatten()
        });
        match routed.and_then(|routed| single(&support.preset, &routed)) {
            Some(mut new) => {
                new.id = support.id;
                new_manual.supports.push(new);
            }
            None => {
                stranded.push(support.id);
                new_manual.supports.push(support.clone());
            }
        }
    }

    // Brace the hand-placed supports to their new neighbours.
    let stuck: HashSet<_> = stranded.iter().copied().collect();
    for support in new_manual
        .supports
        .iter()
        .filter(|s| !stuck.contains(&s.id))
    {
        let others = (new_auto.supports.iter()).filter(|o| o.id != support.id);
        let braces = brace_support(&surface, &bvh, support, others);
        new_manual.braces.extend(braces);
    }

    Some(Rerouted {
        auto: new_auto,
        manual: new_manual,
        stranded,
    })
}

#[cfg(test)]
mod tests {
    use std::{f32::consts::PI, time::Instant};

    use nalgebra::Vector3;
    use tools::supports::auto::{AutoPlacement, PlacementConfig};

    use super::*;
    use crate::{
        core::project::{Project, model::Model},
        regression::{load_prepared, project_path},
    };

    /// Re-routes the first model's supports now, as the app would once the
    /// pose settled.
    fn reroute_now(project: &mut Project) {
        let platform = project.slice_config.platform_size.map(|x| x.convert());
        let model: &mut Model = &mut project.models[0];
        let (surface, bvh) = model.support_surface();
        let later = Instant::now() + std::time::Duration::from_secs(10);
        // A move waits to settle; a new surface doesn't.
        let ticket = (model
            .supports
            .poll_reroute(surface.mesh_id(), Instant::now()))
        .or_else(|| model.supports.poll_reroute(surface.mesh_id(), later))
        .expect("settled");
        let generator = AutoPlacement::new(
            PlacementConfig::default(),
            project.support_config.preset.clone(),
            platform,
        );
        let input = model
            .supports
            .reroute_input(ticket.key, surface, bvh, Some(generator));
        let out = reroute(input, &ticket.token);
        assert!(model.supports.finish_reroute(&ticket.key, out));
    }

    fn ids(model: &Model) -> Vec<SupportId> {
        let (_, manual) = model.supports.world_sets();
        manual.supports.iter().map(|s| s.id).collect()
    }

    #[test]
    fn tilting_a_supported_model_re_routes_and_loses_nothing() {
        let mut project = load_prepared(&project_path("supported"));
        let platform = project.slice_config.platform_size;
        let manual = ids(&project.models[0]);
        let model = &mut project.models[0];
        model.set_rotation(&platform, Vector3::new(0.3, 0.0, 0.0));
        let up = model.mesh.position() + Vector3::z() * 2.0;
        model.set_position(&platform, up);
        assert!(model.supports.needs_reroute());
        assert!(!model.ready(), "slicing waits for the re-route");

        reroute_now(&mut project);
        let model = &project.models[0];
        assert!(!model.supports.needs_reroute() && model.ready());
        // The hand-placed support is still there (re-routed or marked).
        assert_eq!(ids(model), manual);
        let (auto, _) = model.supports.world_sets();
        assert!(!auto.supports.is_empty(), "auto supports regenerated");
        // Regenerating didn't move the model.
        assert!((model.mesh.position() - up).norm() < 1e-6);
    }

    #[test]
    fn a_contact_that_now_faces_up_is_kept_and_marked() {
        let mut project = load_prepared(&project_path("supported"));
        let platform = project.slice_config.platform_size;
        let manual = ids(&project.models[0]);
        let model = &mut project.models[0];
        // Upside down: the hand-placed support's face now looks up.
        model.set_rotation(&platform, Vector3::new(PI, 0.0, 0.0));
        let up = model.mesh.position() + Vector3::z() * 3.0;
        model.set_position(&platform, up);

        reroute_now(&mut project);
        let model = &project.models[0];
        assert_eq!(ids(model), manual, "kept");
        assert_eq!(model.supports.stranded(), Some(&manual[..]), "marked");
        let kinds: Vec<_> = crate::core::issues::model_issues(model, 60.0)
            .into_iter()
            .map(|i| i.kind)
            .collect();
        assert!(kinds.contains(&crate::core::issues::Kind::StrandedSupports));

        // "Remove all" takes them away, and the issue with them.
        let model = &mut project.models[0];
        let snapshot = model.supports.snapshot();
        model.supports.remove(&manual.iter().copied().collect());
        assert!(ids(model).is_empty() && model.supports.stranded().is_none());
        // Undo puts them back, marked again.
        model.supports.restore_snapshot(snapshot);
        assert_eq!(ids(model), manual);
    }

    #[test]
    fn a_new_surface_re_routes_the_supports() {
        let mut project = load_prepared(&project_path("supported"));
        let platform = project.slice_config.platform_size;
        let manual = ids(&project.models[0]);
        let model = &mut project.models[0];
        // Seen once on its surface, as the app does every frame.
        let (surface, _) = model.support_surface();
        let _ = model
            .supports
            .poll_reroute(surface.mesh_id(), Instant::now());
        assert!(!model.supports.needs_reroute());

        // The same shape as a new mesh (as a reload gives).
        let copy = slicer::geometry::mesh::MeshInner {
            vertices: model.source.vertices.clone(),
            faces: model.source.faces.clone(),
        };
        let mut mesh = Mesh::from_inner(Arc::new(copy));
        mesh.set_position_unchecked(model.mesh.position());
        mesh.update_transformation_matrix();
        model.replace_mesh(mesh, None, &platform);
        model.bvh = None;
        assert!(!model.supports.needs_reroute() || model.supports.stranded().is_none());

        reroute_now(&mut project);
        let model = &project.models[0];
        assert_eq!(ids(model), manual);
        assert_eq!(model.supports.stranded(), None, "it still fits");
    }
}
