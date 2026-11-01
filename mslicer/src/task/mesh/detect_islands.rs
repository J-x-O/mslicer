use std::sync::Arc;

use clone_macro::clone;
use common::{progress::Progress, units::Milimeter};
use slicer::geometry::partitioning::Bvh;
use tools::islands::{Detection, Planes, detect};

use crate::{
    core::{
        islands::spacing,
        project::{
            model::{Model, ModelId},
            supports::Pose,
        },
    },
    task::{PollResult, Task, TaskApp, TaskStatus, thread::TaskThread},
};

/// Detects one model's islands for the running island check.
pub struct DetectIslands {
    model: ModelId,
    pose: Pose,
    progress: Progress,
    handle: TaskThread<Detection>,
}

impl DetectIslands {
    pub fn new(model: &Model, bvh: Arc<Bvh>, planes: Planes, pose: Pose) -> Self {
        let progress = Progress::new();
        let mesh = model.mesh.clone();
        Self {
            model: model.id,
            pose,
            handle: TaskThread::spawn(clone!([progress], move || {
                detect(&mesh, &bvh, planes, &progress)
            })),
            progress,
        }
    }
}

impl Task for DetectIslands {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        let (id, pose) = (self.model, self.pose);
        let result = self.handle.poll(app, "Failed to Detect Islands");
        result.into_poll_result(|detection| {
            let layer = app.project.slice_config.slice_height.get::<Milimeter>();
            let spacing = spacing(layer, &app.state.island_settings);
            let now = app
                .project
                .models
                .iter()
                .find(|m| m.id == id)
                .map(|m| Pose::of(&m.mesh));
            if let (Some(check), Some(now)) = (&mut app.state.island_check, now) {
                check.finished(id, pose, detection, now, spacing);
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
