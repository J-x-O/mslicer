use std::{
    collections::HashSet,
    fs::File,
    io::BufReader,
    path::{Path, PathBuf},
};

use anyhow::{Context, Result};
use clone_macro::clone;
use common::{
    fs::write_atomic,
    progress::Progress,
    serde::{ReaderDeserializer, WriterSerializer},
};
use tracing::info;

use super::file_name;
use crate::{
    core::project::Project,
    task::{MeshAnalysis, PollResult, Task, TaskApp, TaskStatus, thread::TaskThread},
};

type Callback = Box<dyn FnOnce(&mut TaskApp) -> Vec<Box<dyn Task>>>;

pub struct ProjectLoad {
    progress: Progress,
    path: PathBuf,
    handle: TaskThread<Result<Project>>,
}

pub struct ProjectSave {
    progress: Progress,
    path: PathBuf,

    handle: TaskThread<Result<()>>,
    callback: Option<Callback>,
}

impl ProjectLoad {
    pub fn new(path: PathBuf) -> Self {
        let progress = Progress::new();

        info!("Loading project from `{}`", path.display());
        let handle = TaskThread::spawn(clone!([progress, path], move || { load(&path, progress) }));

        Self {
            progress,
            path,
            handle,
        }
    }
}

impl ProjectSave {
    pub fn new(project: Project, path: PathBuf) -> Self {
        let progress = Progress::new();

        info!("Saving project to `{}`", path.display());
        let handle = TaskThread::spawn(clone!([progress, path], move || {
            save(&project, &path, progress)
        }));

        Self {
            progress,
            path,

            handle,
            callback: None,
        }
    }

    pub fn with_callback(
        self,
        callback: impl FnOnce(&mut TaskApp) -> Vec<Box<dyn Task>> + 'static,
    ) -> Self {
        Self {
            callback: Some(Box::new(callback)),
            ..self
        }
    }
}

impl Task for ProjectLoad {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        (self.handle.poll_result(app, "Failed to Load Project")).into_poll_result(|project| {
            app.config.add_recent_project(self.path.to_path_buf());
            *app.project = project;
            app.history.clear();
            app.state.anisotropic_aa = app.project.slice_config.supersample.is_anisotropic();

            let mut result = PollResult::complete();
            let count = app.project.models.len();
            let mut seen_meshes = HashSet::new();
            for (i, model) in app.project.models.iter_mut().enumerate() {
                model.update_oob(&app.project.slice_config.platform_size);

                // Only spawn these tasks one per mesh, not per instance
                if seen_meshes.insert(model.mesh.mesh_id()) {
                    result = result.with_task(MeshAnalysis::new(model));
                }

                info!(
                    " {} Loaded model `{}` with {} faces",
                    if i + 1 < count { "│" } else { "└" },
                    model.name,
                    model.mesh.face_count()
                );
            }

            result
        })
    }

    fn status(&self) -> Option<TaskStatus<'_>> {
        Some(TaskStatus {
            name: "Loading Project".into(),
            details: Some(format!("Loading `{}`", file_name(&self.path))),
            progress: self.progress.progress(),
        })
    }
}

impl Task for ProjectSave {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        (self.handle.poll_result(app, "Failed to Save Project")).into_poll_result(|_| {
            app.config.add_recent_project(self.path.to_path_buf());
            if let Some(callback) = self.callback.take() {
                PollResult::complete().with_tasks(callback(app))
            } else {
                PollResult::complete()
            }
        })
    }

    fn status(&self) -> Option<TaskStatus<'_>> {
        Some(TaskStatus {
            name: "Saving Project".into(),
            details: Some(format!("Saving `{}`", file_name(&self.path))),
            progress: self.progress.progress(),
        })
    }
}

fn load(path: &Path, progress: Progress) -> Result<Project> {
    let file = File::open(path).with_context(|| format!("Couldn't open `{}`", path.display()))?;
    let mut des = ReaderDeserializer::new(BufReader::new(file));
    let project = Project::deserialize(&mut des, progress);
    if let Some(error) = des.error() {
        anyhow::bail!("Couldn't read `{}`: {error}", path.display());
    }
    Ok(project?.with_path(path.to_path_buf()))
}

/// Writes the project next to `path` and renames it over: a failed save
/// leaves the old file.
fn save(project: &Project, path: &Path, progress: Progress) -> Result<()> {
    write_atomic(path, |out| {
        let mut ser = WriterSerializer::new(out);
        project.serialize(&mut ser, progress);
        ser.finish().map(drop)
    })
    .with_context(|| format!("Couldn't save `{}`", path.display()))
}
