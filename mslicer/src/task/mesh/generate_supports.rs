use std::sync::{
    Arc,
    atomic::{AtomicU64, Ordering},
};

use common::{progress::Progress, units::Milimeters};
use nalgebra::Vector3;
use slicer::geometry::{mesh::Mesh, partitioning::Bvh};
use tools::supports::{
    auto::{AutoPlacement, BraceNetwork, Generated, GenerationProgress, PlacementConfig},
    mesh::SupportPreset,
};

use crate::{
    core::project::{
        model::{Model, ModelId},
        supports::Pose,
    },
    engine::CancelToken,
    task::{PollResult, Task, TaskApp, TaskStatus, store_bvh, thread::TaskThread},
};

/// Tells generations apart, so a newer one or an undo can cancel an older one.
static NEXT_TOKEN: AtomicU64 = AtomicU64::new(0);

/// The routed supports plus the brace network computed over them, and the
/// surface they were routed on.
struct RoutedWithBraces {
    mesh: Box<Mesh>,
    /// Routed on the model's own mesh, not a hollowed one's print mesh.
    own: bool,
    bvh: Arc<Bvh>,
    generated: Generated,
    network: BraceNetwork,
}

pub struct GenerateSupports {
    model: ModelId,
    token: u64,
    preset: SupportPreset,

    progress: Stages,
    state: State,
    /// Waiting for the model's cavity or holes.
    hollowing: bool,
}

/// Progress per stage. Each counts for its share of the bar, by how long it
/// roughly takes; building the acceleration structures only when this task
/// does it.
#[derive(Clone, Default)]
struct Stages {
    bvh: Option<Progress>,
    generation: GenerationProgress,
    bracing: Progress,
}

impl Stages {
    const BVH: f32 = 1.7;
    const PLACING: f32 = 1.1;
    const ROUTING: f32 = 0.3;
    const BRACING: f32 = 0.3;
    const MESHING: f32 = 0.1;

    fn progress(&self, meshed: bool) -> f32 {
        let bvh = self.bvh.as_ref().map(|p| (Self::BVH, p.progress()));
        let stages = bvh.into_iter().chain([
            (Self::PLACING, self.generation.placing.progress()),
            (Self::ROUTING, self.generation.routing.progress()),
            (Self::BRACING, self.bracing.progress()),
            (Self::MESHING, if meshed { 1.0 } else { 0.0 }),
        ]);
        let (done, total) = stages.fold((0.0, 0.0), |(done, total), (weight, progress)| {
            (done + weight * progress.min(1.0), total + weight)
        });
        done / total
    }

    /// What is being done now.
    fn stage(&self) -> &'static str {
        if self.bvh.as_ref().is_some_and(|p| !p.complete()) {
            "Building acceleration structures"
        } else if !self.generation.placing.complete() {
            "Placing points"
        } else if !self.generation.routing.complete() {
            "Routing"
        } else if !self.bracing.complete() {
            "Bracing"
        } else {
            "Building the mesh"
        }
    }
}

enum State {
    /// Waiting for the model's hollowing, or its acceleration structures.
    Waiting(Option<AutoPlacement>),
    Running {
        at: Pose,
        handle: TaskThread<RoutedWithBraces>,
    },
    /// Generated; waiting for the support mesh to show.
    Meshing,
}

impl GenerateSupports {
    /// Generates supports for `model`. Without acceleration structures it
    /// waits for them, building them first if nothing else is.
    pub fn new(
        platform: Vector3<Milimeters>,
        placement: PlacementConfig,
        support: SupportPreset,
        model: &mut Model,
    ) -> Self {
        let token = NEXT_TOKEN.fetch_add(1, Ordering::Relaxed);
        let cancel = CancelToken::new();
        model.supports.begin_generation(token, cancel.clone());
        let generator = AutoPlacement::new(placement, support.clone(), platform);
        let mut progress = Stages::default();
        progress.generation.cancel = cancel.shared();
        Self {
            model: model.id,
            token,
            preset: support,
            progress,
            state: State::Waiting(Some(generator)),
            hollowing: false,
        }
    }
}

impl Task for GenerateSupports {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        match &mut self.state {
            State::Waiting(generator) => {
                let Some(model) = app.project.models.iter().find(|x| x.id == self.model) else {
                    return PollResult::complete();
                };
                if !model.supports.is_pending(self.token) {
                    return PollResult::complete();
                }

                // A hollowed model is supported as it prints, inside too:
                // once its cavity and holes are done.
                if !model.ready() {
                    self.hollowing = true;
                    return PollResult::pending();
                }
                self.hollowing = false;
                let (mesh, bvh) = model.support_surface();
                let own = mesh.mesh_id() == model.mesh.mesh_id();

                // Without acceleration structures, build them here unless a
                // preparation task already is; then wait for it.
                if own && bvh.is_none() && app.state.preparing.contains(&model.mesh.mesh_id()) {
                    return PollResult::pending();
                }

                let at = Pose::of(&model.mesh);
                let inner = mesh.inner().clone();
                let Some(generator) = generator.take() else {
                    return PollResult::complete();
                };
                let generate = move |bvh: Arc<Bvh>, progress: Stages| {
                    let generated = generator.generate_supports(&mesh, &bvh, &progress.generation);
                    progress.bracing.set_total(1);
                    // Cancelled: the result is dropped anyway.
                    let network = match progress.generation.is_cancelled() {
                        true => BraceNetwork::default(),
                        false => generator.generate_braces(&mesh, &bvh, &generated),
                    };
                    progress.bracing.set_finished();
                    RoutedWithBraces {
                        mesh: Box::new(mesh),
                        own,
                        bvh,
                        generated,
                        network,
                    }
                };
                let handle = match bvh {
                    Some(bvh) => {
                        let progress = self.progress.clone();
                        TaskThread::spawn(move || generate(bvh, progress))
                    }
                    None => {
                        if own {
                            app.state.preparing.insert(model.mesh.mesh_id());
                        }
                        let bvh_progress = Progress::new();
                        self.progress.bvh = Some(bvh_progress.clone());
                        let progress = self.progress.clone();
                        TaskThread::spawn(move || {
                            let bvh = Arc::new(Bvh::build(&inner, bvh_progress));
                            generate(bvh, progress)
                        })
                    }
                };
                self.state = State::Running { at, handle };
                PollResult::pending()
            }
            State::Running { at, handle } => {
                // Lifting the model for the generation sends the hand-placed
                // supports to be routed again: the new auto supports join them
                // once they have.
                let rerouting = (app.project.models.iter())
                    .find(|x| x.id == self.model)
                    .is_some_and(|m| m.supports.needs_reroute());
                if rerouting {
                    return PollResult::pending();
                }
                let at = *at;
                let result = handle.poll(app, "Failed to Generate Supports");
                let (model_id, token) = (self.model, self.token);
                let (preset, state) = (&self.preset, &mut self.state);
                result.into_poll_result(|out| {
                    let mesh_id = (app.project.models.iter())
                        .find(|x| x.id == model_id)
                        .map(|x| x.mesh.mesh_id());
                    match (mesh_id, out.own) {
                        (Some(mesh_id), true) => {
                            store_bvh(&mut app.project.models, mesh_id, &out.bvh);
                            app.state.preparing.remove(&mesh_id);
                        }
                        (Some(_), false) => {
                            if let Some(model) = app.project.model(model_id) {
                                let print = (out.mesh.mesh_id(), out.bvh.clone());
                                model.hollow.print_bvh = Some(print);
                            }
                        }
                        (None, _) => {}
                    }

                    let taken = app.project.model(model_id).is_some_and(|model| {
                        model.supports.finish_generation(
                            token,
                            at,
                            preset,
                            (out.generated, out.network),
                            &out.mesh,
                            &out.bvh,
                        )
                    });
                    if !taken {
                        return PollResult::complete();
                    }
                    *state = State::Meshing;
                    PollResult::pending()
                })
            }
            // Done once the mesh is up, or the supports went some other way.
            State::Meshing => {
                let model = app.project.models.iter().find(|x| x.id == self.model);
                PollResult::from_bool(model.is_none_or(|m| m.supports.is_current()))
            }
        }
    }

    fn status(&self) -> Option<TaskStatus<'_>> {
        let details = match self.state {
            State::Waiting(_) if self.hollowing => "Waiting for hollowing",
            State::Waiting(_) => "Waiting for acceleration structures",
            _ => self.progress.stage(),
        };
        Some(TaskStatus {
            name: "Generating Supports".into(),
            details: Some(details.into()),
            progress: (self.progress).progress(matches!(self.state, State::Meshing)),
        })
    }
}
