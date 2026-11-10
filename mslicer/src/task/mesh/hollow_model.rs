use clone_macro::clone;
use common::progress::Progress;
use tools::hollow::cavity;

use crate::{
    core::project::{
        hollow::{CavityKey, DrillInput, DrillKey, Drilled, ModelCavity},
        model::{Model, ModelId},
    },
    engine::Ticket,
    task::{PollResult, Task, TaskApp, TaskStatus, thread::TaskThread},
};

/// Computes the cavity of a hollowed model: the job of its cavity slot. Gives
/// up early once its ticket is cancelled.
pub struct HollowModel {
    model: ModelId,
    key: CavityKey,
    progress: Progress,
    handle: TaskThread<Option<ModelCavity>>,
}

impl HollowModel {
    /// Starts computing `model`'s cavity for the ticket's key, with drain
    /// holes of `diameter` suggested.
    pub fn new(model: &Model, ticket: Ticket<CavityKey>, diameter: f32) -> Self {
        let progress = Progress::new();
        let mesh = model.mesh.clone();
        let Ticket { key, token } = ticket;
        let shell = key.shell;
        Self {
            model: model.id,
            key,
            handle: TaskThread::spawn(clone!([progress], move || {
                let cancel = token.flag();
                let found = cavity(&mesh, shell.wall, shell.cell, diameter, &progress, cancel)?;
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
        // A failed run keeps the slot busy, so it isn't retried (with an
        // error popup) every frame.
        result.into_poll_result(|cavity| {
            if let Some(model) = app.project.models.iter_mut().find(|m| m.id == id) {
                let hollow = &mut model.hollow;
                // Settings changed meanwhile: dropped, and the next run
                // starts from them.
                let current = hollow.key(&model.mesh) == Some(key);
                hollow.cavity.finish(&key, cavity.filter(|_| current));
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

/// Drills a hollowed model's holes into its print mesh: the job of its
/// drilling slot.
pub struct DrillHoles {
    model: ModelId,
    key: DrillKey,
    handle: TaskThread<Option<Drilled>>,
}

impl DrillHoles {
    pub fn new(model: ModelId, ticket: Ticket<DrillKey>, input: DrillInput) -> Self {
        let Ticket { key, token } = ticket;
        Self {
            model,
            key,
            handle: TaskThread::spawn(move || Drilled::drill_unless(input, &token)),
        }
    }
}

impl Task for DrillHoles {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        let (id, key) = (self.model, &self.key);
        let result = self.handle.poll(app, "Failed to Drill Holes");
        // A failed run keeps the slot busy, so it isn't retried (with an
        // error popup) every frame.
        result.into_poll_result(|drilled| {
            // Dropped if the holes changed meanwhile; the next run starts
            // from them.
            if let Some(model) = app.project.models.iter_mut().find(|m| m.id == id) {
                model.hollow.drilled.finish(key, drilled);
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
