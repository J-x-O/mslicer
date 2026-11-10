use std::sync::Arc;

use clone_macro::clone;
use common::{progress::Progress, units::Milimeter};
use slicer::geometry::{
    mesh::{Mesh, MeshId},
    partitioning::Bvh,
};
use tools::islands::{Detection, Planes, detect};

use crate::{
    core::{
        islands::spacing,
        project::{model::ModelId, supports::Pose},
    },
    task::{PollResult, Task, TaskApp, TaskStatus, thread::TaskThread},
};

/// Detects one model's islands for the running island check.
pub struct DetectIslands {
    model: ModelId,
    pose: Pose,
    source: MeshId,
    progress: Progress,
    handle: TaskThread<Detection>,
}

impl DetectIslands {
    /// Detects the islands of `mesh` (the model's print mesh), building its
    /// hit-testing structure first if `bvh` is `None`.
    pub fn new(
        model: ModelId,
        mesh: Mesh,
        bvh: Option<Arc<Bvh>>,
        planes: Planes,
        pose: Pose,
    ) -> Self {
        let progress = Progress::new();
        Self {
            model,
            pose,
            source: mesh.mesh_id(),
            handle: TaskThread::spawn(clone!([progress], move || {
                let bvh =
                    bvh.unwrap_or_else(|| Arc::new(Bvh::build(mesh.inner(), Progress::new())));
                detect(&mesh, &bvh, planes, &progress)
            })),
            progress,
        }
    }
}

impl Task for DetectIslands {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        let (id, pose, source) = (self.model, self.pose, self.source);
        let result = self.handle.poll(app, "Failed to Detect Islands");
        result.into_poll_result(|detection| {
            let layer = app.project.slice_config.slice_height.get::<Milimeter>();
            let spacing = spacing(layer, &app.state.island_settings);
            let now = app
                .project
                .models
                .iter()
                .find(|m| m.id == id)
                .map(|m| (Pose::of(&m.mesh), m.print_mesh().mesh_id()));
            if let (Some(check), Some(now)) = (&mut app.state.island_check, now) {
                check.finished(id, (pose, source), detection, now, spacing);
            }
            PollResult::complete()
        })
    }

    fn status(&self) -> Option<TaskStatus<'_>> {
        Some(TaskStatus {
            name: "Detecting Islands".into(),
            details: None,
            progress: self.progress.progress(),
        })
    }
}
