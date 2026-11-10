use std::{
    fs::File,
    io::BufReader,
    mem,
    path::{Path, PathBuf},
    sync::Arc,
};

use anyhow::{Context, Result};
use clone_macro::clone;
use common::{
    fs::write_atomic,
    progress::Progress,
    serde::{ReaderDeserializer, SliceDeserializer, WriterSerializer},
};
use mesh_format::{Format, load_mesh, save_mesh};

use slicer::geometry::mesh::{Mesh, MeshInner};
use tracing::info;

use crate::{
    core::{history::Action, project::model::Model},
    task::{MeshAnalysis, PollResult, Task, TaskApp, TaskStatus, thread::TaskThread},
};

pub struct MeshLoad {
    progress: Progress,
    handle: TaskThread<Result<Mesh>>,

    name: String,
    file: Option<PathBuf>,
}

impl MeshLoad {
    pub fn file(path: PathBuf, name: String, format: String) -> Self {
        let progress = Progress::new();
        Self {
            handle: TaskThread::spawn(clone!([progress, path], move || {
                read_mesh(&path, &format, &progress)
            })),
            progress,

            name,
            file: Some(path),
        }
    }

    pub fn buffer(buffer: &'static [u8], name: String, format: String) -> Self {
        let progress = Progress::new();
        Self {
            handle: TaskThread::spawn(clone!([progress], move || {
                let format = Format::from_extension(&format).context("Unknown mesh format")?;
                let mesh = load_mesh(SliceDeserializer::new(buffer), format, &progress)?;
                Ok(Mesh::new_boxed_centered(mesh.verts, mesh.faces))
            })),
            progress,

            name,
            file: None,
        }
    }

    pub fn complete(name: String, mesh: Mesh) -> Self {
        Self {
            progress: Progress::already_complete(),
            handle: TaskThread::complete(Ok(mesh)),

            name,
            file: None,
        }
    }
}

impl Task for MeshLoad {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        (self.handle.poll_result(app, "Failed to Load Model")).into_poll_result(|mesh| {
            info!(
                "Loaded model `{}` with {} faces",
                self.name,
                mesh.face_count()
            );

            let mut model = Model::from_mesh(mesh)
                .with_name(mem::take(&mut self.name))
                .width_file(self.file.take())
                .with_random_color(app.config.ui.warning_color);
            app.config.default_stack.apply(&mut model);
            model.update_oob(&app.project.slice_config.platform_size);
            let result = PollResult::complete().with_task(MeshAnalysis::new(&model));
            app.history.track(Action::ModelAdded { id: model.id });
            app.project.models.push(model);
            result
        })
    }

    fn status(&self) -> Option<TaskStatus<'_>> {
        Some(TaskStatus {
            name: "Loading Model".into(),
            details: Some(format!("Loading `{}`", self.name)),
            progress: self.progress.progress(),
        })
    }
}

pub struct MeshSave {
    progress: Progress,
    handle: TaskThread<Result<()>>,
}

impl MeshSave {
    pub fn new(path: PathBuf, format: Format, mesh: Arc<MeshInner>) -> Self {
        // SAFETY: Both MeshInner and mesh_format::Mesh have the same layout.
        let mesh = unsafe { mem::transmute::<Arc<MeshInner>, Arc<mesh_format::Mesh>>(mesh) };
        let progress = Progress::new();

        let handle = TaskThread::spawn(clone!([progress], move || {
            write_atomic(&path, |out| {
                let mut ser = WriterSerializer::new(out);
                save_mesh(&mut ser, format, &progress, &mesh);
                ser.finish().map(drop)
            })
            .with_context(|| format!("Couldn't save `{}`", path.display()))
        }));

        Self { progress, handle }
    }
}

impl Task for MeshSave {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        self.handle
            .poll_result(app, "Failed to Save Mesh")
            .into_poll_result(|_| PollResult::complete())
    }

    fn status(&self) -> Option<TaskStatus<'_>> {
        Some(TaskStatus {
            name: "Saving Mesh".into(),
            details: None, // todo: use file name?
            progress: self.progress.progress(),
        })
    }
}

/// Reads a mesh file, centered. `format` is its extension.
pub fn read_mesh(path: &Path, format: &str, progress: &Progress) -> Result<Mesh> {
    let format = Format::from_extension(format)
        .with_context(|| format!("`{}` isn't a mesh format mslicer reads", path.display()))?;
    let file = File::open(path).with_context(|| format!("Couldn't open `{}`", path.display()))?;
    let des = ReaderDeserializer::new(BufReader::new(file));
    let mesh = load_mesh(des, format, progress)
        .with_context(|| format!("Couldn't read `{}`", path.display()))?;
    Ok(Mesh::new_boxed_centered(mesh.verts, mesh.faces))
}
