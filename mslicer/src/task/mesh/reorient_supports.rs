use crate::{
    core::project::{
        model::ModelId,
        reroute::{RerouteInput, RerouteKey, Rerouted, reroute},
    },
    engine::Ticket,
    task::{PollResult, Task, TaskApp, TaskStatus, thread::TaskThread},
};

/// Routes a model's supports again after it was turned, lifted or scaled:
/// the job of its supports' re-route slot. Gives up once superseded.
pub struct ReorientSupports {
    model: ModelId,
    name: String,
    key: RerouteKey,
    handle: TaskThread<Option<Rerouted>>,
}

impl ReorientSupports {
    pub fn new(
        model: ModelId,
        name: String,
        ticket: Ticket<RerouteKey>,
        input: RerouteInput,
    ) -> Self {
        let Ticket { key, token } = ticket;
        Self {
            model,
            name,
            key,
            handle: TaskThread::spawn(move || reroute(input, &token)),
        }
    }
}

impl Task for ReorientSupports {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        let (id, key) = (self.model, &self.key);
        let result = self.handle.poll(app, "Failed to Re-route Supports");
        // A failed run keeps the slot busy, so it isn't retried (with an
        // error popup) every frame.
        result.into_poll_result(|rerouted| {
            if let Some(model) = app.project.model(id) {
                model.supports.finish_reroute(key, rerouted);
            }
            PollResult::complete()
        })
    }

    fn status(&self) -> Option<TaskStatus<'_>> {
        Some(TaskStatus {
            name: "Re-routing Supports".into(),
            details: Some(format!("For `{}`", self.name)),
            progress: 0.0,
        })
    }
}
