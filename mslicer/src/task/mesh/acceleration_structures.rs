use std::sync::Arc;

use clone_macro::clone;
use common::progress::Progress;
use slicer::geometry::{
    mesh::{MeshId, MeshInner},
    partitioning::Bvh,
};

use crate::{
    core::{project::model::Model, state::UiState},
    task::{PollResult, Task, TaskApp, TaskStatus, thread::TaskThread},
};

pub struct BuildAccelerationStructures {
    mesh_id: MeshId,
    name: Option<String>,

    progress: Progress,
    handle: TaskThread<Arc<Bvh>>,
}

impl BuildAccelerationStructures {
    /// Starts building the structures for `model`'s mesh, unless they exist
    /// or are already being built.
    pub fn request(state: &mut UiState, model: &Model) -> Option<Self> {
        let id = model.mesh.mesh_id();
        (!model.has_acceleration_structures() && state.preparing.insert(id)).then(|| {
            let mut task = Self::new_mesh(id, model.mesh.inner().clone());
            task.name = Some(model.name.clone());
            task
        })
    }

    fn new_mesh(mesh_id: MeshId, inner: Arc<MeshInner>) -> Self {
        let progress = Progress::new();
        Self {
            mesh_id,
            name: None,
            handle: TaskThread::spawn(clone!([progress], move || {
                Arc::new(Bvh::build(&inner, progress))
            })),
            progress,
        }
    }
}

/// Hands a finished BVH to every model that shares its mesh.
pub fn store_bvh(models: &mut [Model], mesh_id: MeshId, bvh: &Arc<Bvh>) {
    for model in models.iter_mut().filter(|x| x.mesh.mesh_id() == mesh_id) {
        model.bvh.get_or_insert_with(|| bvh.clone());
    }
}

impl Task for BuildAccelerationStructures {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        const FAILURE: &str = "Failed to Build Acceleration Structure";
        // A failed build stays in `preparing`, so it isn't retried every frame.
        (self.handle.poll(app, FAILURE)).into_poll_result(|bvh| {
            store_bvh(&mut app.project.models, self.mesh_id, &bvh);
            app.state.preparing.remove(&self.mesh_id);
            PollResult::complete()
        })
    }

    fn status(&self) -> Option<TaskStatus<'_>> {
        Some(TaskStatus {
            name: "Building Acceleration Structures".into(),
            details: self.name.as_ref().map(|name| format!("For `{name}`")),
            progress: self.progress.progress(),
        })
    }
}
