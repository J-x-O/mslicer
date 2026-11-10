//! Slicing one layer on demand, for the layer under the scrubber.

use std::{
    sync::Arc,
    thread::{self, JoinHandle},
};

use common::slice::{Layer, SliceMode};
use slicer::slicer::Slicer;

use crate::{
    core::{
        project::Project,
        slice::{
            key::slice_key,
            runner::{SlicePart, platform_models},
        },
    },
    engine::{CancelToken, InputHash, Slot, SlotState},
};

/// Which layer of which project inputs.
pub type LayerKey = (InputHash, u32);

/// The node for one sliced layer: the layer last asked for, sliced on its
/// own in the background. Asking for another layer (or the project changing)
/// cancels a slice still running; at most one runs.
///
/// The layer is as the slicer produces it, before post-processing (which
/// works on the whole stack): exact unless variable layer height or the
/// elephant foot fixer is on.
#[derive(Default)]
pub struct LayerPreview {
    slot: Slot<LayerKey, Layer>,
    job: Option<(LayerKey, JoinHandle<Option<Layer>>)>,
}

impl LayerPreview {
    /// Asks for layer `index` (0 is the first) of `project` as it is now.
    /// Returns it once it is sliced. Meanwhile [`LayerPreview::latest`] is
    /// the last one sliced. Nothing starts while a model's hollowing or
    /// support mesh is still being computed, or in vector mode.
    pub fn request(&mut self, project: &Project, index: u32) -> Option<Arc<Layer>> {
        self.poll();
        let key = (slice_key(project), index);
        if let Some(layer) = self.slot.get(&key) {
            return Some(layer.clone());
        }

        let visible = || project.models.iter().filter(|m| !m.hidden);
        let ready = visible().all(|m| m.ready() && m.supports.is_current());
        if !ready || project.slice_config.mode != SliceMode::Raster {
            self.slot.supersede(key);
            return None;
        }

        let ticket = self.slot.request(key)?;
        let parts: Vec<_> = visible()
            .map(|m| SlicePart {
                print: m.print_mesh(),
                supports: (m.active_supports())
                    .and_then(|s| s.mesh().as_ref())
                    .map(|(mesh, _)| mesh.clone()),
                exposure: m.exposure,
            })
            .collect();
        let config = project.slice_config.clone();
        let token: CancelToken = ticket.token;
        let handle = thread::spawn(move || {
            token.check().ok()?;
            let (models, _) = platform_models(&config, parts);
            let slicer = Slicer::new(config, models);
            token.check().ok()?;
            (index < slicer.layer_count()).then(|| slicer.slice_raster_layer(index))
        });
        self.job = Some((ticket.key, handle));
        None
    }

    /// Takes in a finished slice.
    pub fn poll(&mut self) {
        if self.job.as_ref().is_some_and(|(_, h)| h.is_finished()) {
            if let Some((key, handle)) = self.job.take() {
                self.slot.finish(&key, handle.join().ok().flatten());
            }
        }
    }

    /// The last layer sliced, for whatever inputs.
    pub fn latest(&self) -> Option<&Arc<Layer>> {
        self.slot.latest()
    }

    pub fn state(&self) -> SlotState {
        self.slot.state()
    }

    /// Waits for the running slice, for tests.
    #[cfg(test)]
    pub fn wait(&mut self) {
        if let Some((key, handle)) = self.job.take() {
            self.slot.finish(&key, handle.join().ok().flatten());
        }
    }
}
