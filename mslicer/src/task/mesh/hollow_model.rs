use std::sync::{Arc, atomic::AtomicBool};

use clone_macro::clone;
use common::progress::Progress;
use tools::hollow::cavity;

use crate::{
    core::project::{
        hollow::{CavityKey, DrillInput, Drilled, ModelCavity},
        model::{Model, ModelId},
    },
    task::{PollResult, Task, TaskApp, TaskStatus, thread::TaskThread},
};

/// Computes the cavity of a hollowed model. Gives up early once `cancel` is
/// set.
pub struct HollowModel {
    model: ModelId,
    key: CavityKey,
    progress: Progress,
    handle: TaskThread<Option<ModelCavity>>,
}

impl HollowModel {
    /// Starts computing `model`'s cavity for `key`, with drain holes of
    /// `diameter` suggested.
    pub fn new(model: &Model, key: CavityKey, cancel: Arc<AtomicBool>, diameter: f32) -> Self {
        let progress = Progress::new();
        let mesh = model.mesh.clone();
        let shell = key.shell;
        Self {
            model: model.id,
            key,
            handle: TaskThread::spawn(clone!([progress], move || {
                let found = cavity(&mesh, shell.wall, shell.cell, diameter, &progress, &cancel)?;
                Some(ModelCavity::new(found, &mesh, shell))
            })),
            progress,
        }
    }
}

impl Task for HollowModel {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        let (id, key) = (self.model, self.key);
        let result = self.handle.poll(app, "Failed to Hollow Model");
        result.into_poll_result(|cavity| {
            if let Some(model) = app.project.models.iter_mut().find(|m| m.id == id) {
                let hollow = &mut model.hollow;
                if hollow.pending.as_ref().is_some_and(|(k, _)| *k == key) {
                    hollow.pending = None;
                }
                // Settings changed meanwhile: the next run starts from them.
                if let Some(cavity) = cavity
                    && hollow.key(&model.mesh) == Some(key)
                {
                    hollow.cavity = Some(Arc::new(cavity));
                }
            }
            PollResult::complete()
        })
    }

    fn status(&self) -> Option<TaskStatus<'_>> {
        Some(TaskStatus {
            name: "Hollowing".into(),
            details: None,
            progress: self.progress.progress(),
        })
    }
}

/// Drills a hollowed model's holes into its print mesh.
pub struct DrillHoles {
    model: ModelId,
    handle: TaskThread<Drilled>,
}

impl DrillHoles {
    pub fn new(model: ModelId, input: DrillInput) -> Self {
        Self {
            model,
            handle: TaskThread::spawn(move || Drilled::drill(input)),
        }
    }
}

impl Task for DrillHoles {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        let id = self.model;
        let result = self.handle.poll(app, "Failed to Drill Holes");
        result.into_poll_result(|drilled| {
            if let Some(model) = app.project.models.iter_mut().find(|m| m.id == id) {
                // Kept even if the holes changed meanwhile: it is checked
                // against them before use, and the next run starts from them.
                model.hollow.drilling = false;
                model.hollow.drilled = Some(Arc::new(drilled));
            }
            PollResult::complete()
        })
    }

    fn status(&self) -> Option<TaskStatus<'_>> {
        Some(TaskStatus {
            name: "Drilling Holes".into(),
            details: None,
            progress: 0.0,
        })
    }
}
