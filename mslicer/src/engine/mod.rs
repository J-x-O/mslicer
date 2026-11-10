//! The evaluation engine: cached, demand-driven, cancellable computation.
//!
//! Every derived result (a model's cavity, its drilled print mesh, its
//! support mesh, a mesh's acceleration structures, a sliced layer, the full
//! slice) lives in a [`Slot`]: the cached value plus the key it was computed
//! from. A key is the node's inputs, or an [`InputHash`] of them, and it
//! includes the identity of the upstream results it used, so a change
//! upstream changes every key downstream of it. That *is* the dirty
//! propagation: nothing is recomputed or even visited on a change; a slot is
//! dirty exactly when the key a consumer asks for differs from the one it
//! holds.
//!
//! Work starts only when a consumer asks ([`Slot::request`]): the viewport
//! drawing a mesh, slicing, a panel showing a value. Jobs run on the existing
//! task system (they are ordinary [`Task`]s, so the task monitor shows them)
//! and carry a [`CancelToken`] they check between chunks of work. Asking for
//! other inputs cancels a running job, and a result for inputs nobody wants
//! any more is discarded ([`Slot::finish`]). [`Debounce`] keeps dragged values
//! from asking for work on every step.
//!
//! [`Task`]: crate::task::Task

pub mod debounce;
pub mod hash;
pub mod layer;
pub mod slot;
#[cfg(test)]
mod tests;

pub use debounce::Debounce;
pub use hash::{InputHash, InputHasher};
pub use slot::{CancelToken, Slot, SlotState, Ticket};

use crate::core::project::Project;

/// What the engine is doing, over the whole project.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Status {
    /// Results being computed for what is wanted now.
    pub computing: usize,
    /// Results wanted but neither cached nor being computed (waiting to
    /// settle, or for a cancelled job to stop).
    pub stale: usize,
}

impl Status {
    pub fn of(project: &Project) -> Self {
        let mut status = Self::default();
        for model in &project.models {
            for state in model.slot_states() {
                status.add(state);
            }
        }
        status
    }

    pub fn add(&mut self, state: SlotState) {
        match state {
            SlotState::Computing => self.computing += 1,
            SlotState::Stale => self.stale += 1,
            SlotState::Fresh | SlotState::Idle => {}
        }
    }
}
