use clone_macro::clone;
use common::progress::Progress;

use crate::{
    core::project::{
        geometry::{Evaluated, GeometryKey, evaluate},
        model::{Model, ModelId},
    },
    engine::Ticket,
    task::{PollResult, Task, TaskApp, TaskStatus, thread::TaskThread},
};

/// Evaluates a model's geometry rows (Mesh Repair, Flip Winding Order) on its
/// source: the job of its surface slot. Gives up between rows once its
/// ticket is cancelled.
pub struct EvaluateGeometry {
    model: ModelId,
    name: String,
    key: GeometryKey,
    progress: Progress,
    handle: TaskThread<Option<Evaluated>>,
}

impl EvaluateGeometry {
    pub fn new(model: &Model, ticket: Ticket<GeometryKey>) -> Self {
        let Ticket { key, token } = ticket;
        let progress = Progress::new();
        let (source, steps) = (key.source.clone(), key.steps.clone());
        Self {
            model: model.id,
            name: model.name.clone(),
            key,
            handle: TaskThread::spawn(clone!([progress], move || {
                evaluate(&source, &steps, &progress, &token)
            })),
            progress,
        }
    }
}

impl Task for EvaluateGeometry {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        let (id, key) = (self.model, &self.key);
        let result = self.handle.poll(app, "Failed to Evaluate Model Geometry");
        // A failed run keeps the slot busy, so it isn't retried (with an
        // error popup) every frame.
        result.into_poll_result(|evaluated| {
            // Dropped if the stack changed meanwhile; the next run starts
            // from it. The stack update puts the result in place.
            if let Some(model) = app.project.model(id) {
                model.geometry.surface.finish(key, evaluated);
            }
            PollResult::complete()
        })
    }

    fn status(&self) -> Option<TaskStatus<'_>> {
        Some(TaskStatus {
            name: "Evaluating Operations".into(),
            details: Some(format!("For `{}`", self.name)),
            progress: self.progress.progress(),
        })
    }
}
