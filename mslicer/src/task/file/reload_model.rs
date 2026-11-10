use std::{mem, path::PathBuf};

use anyhow::Result;
use clone_macro::clone;
use common::progress::Progress;

use slicer::geometry::mesh::Mesh;
use tracing::info;

use super::mesh::read_mesh;
use crate::{
    core::project::model::ModelId,
    task::{MeshAnalysis, PollResult, Task, TaskApp, TaskStatus, thread::TaskThread},
};

pub struct ReloadModel {
    progress: Progress,
    handle: TaskThread<Result<Mesh>>,

    model: ModelId,
    path: PathBuf,
    name: String,
}

impl ReloadModel {
    pub fn new(model: ModelId, name: String, path: PathBuf) -> Self {
        let progress = Progress::new();
        Self {
            handle: TaskThread::spawn(clone!([progress, path], move || {
                let ext = path.extension().unwrap_or_default().to_string_lossy();
                read_mesh(&path, &ext, &progress)
            })),
            progress,

            model,
            path,
            name,
        }
    }
}

impl Task for ReloadModel {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        (self.handle.poll_result(app, "Failed to Load Model")).into_poll_result(|mesh| {
            info!(
                "Reloaded model `{}` with {} faces",
                self.name,
                mesh.face_count()
            );

            let platform_size = app.project.slice_config.platform_size;
            if let Some(model) = app.project.model(self.model) {
                model.replace_mesh(mesh, Some(mem::take(&mut self.path)), &platform_size);
                PollResult::complete().with_task(MeshAnalysis::new(model))
            } else {
                PollResult::complete()
            }
        })
    }

    fn status(&self) -> Option<TaskStatus<'_>> {
        Some(TaskStatus {
            name: "Reloading Model".into(),
            details: Some(format!("Loading `{}`", self.name)),
            progress: self.progress.progress(),
        })
    }
}
