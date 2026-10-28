use std::{collections::HashMap, sync::Arc};

use common::progress::Progress;
use nalgebra::Vector3;
use rayon::prelude::*;
use slicer::geometry::{mesh::Mesh, partitioning::Bvh};
use tools::supports::{SupportId, SupportSet, auto::route_point, mesh::SupportPreset};

use crate::{
    core::{
        project::{
            model::{Model, ModelId},
            supports::{replace, surface_normal},
        },
        selected::SelectedSupport,
        state::UiState,
    },
    task::{
        PollResult, Task, TaskApp, TaskStatus,
        thread::{TaskResult, TaskThread},
    },
};

type Sets = (Arc<SupportSet>, Arc<SupportSet>);

/// Re-routes the supports of a settings edit with its newest values, away
/// from the UI thread. See [`crate::core::state::SupportEdit`].
pub struct RerouteSupports {
    handle: TaskThread<Vec<Rerouted>>,
    progress: Progress,
    count: usize,
}

/// One model's sets after the re-route, and the sets they were made from.
struct Rerouted {
    model: ModelId,
    base: Sets,
    sets: (SupportSet, SupportSet),
    /// Supports no route was found for; they keep their old shape.
    unrouted: Vec<SupportId>,
}

struct Job {
    model: ModelId,
    base: Sets,
    mesh: Mesh,
    bvh: Arc<Bvh>,
    changes: Vec<(SupportId, SupportPreset, Vector3<f32>, Vector3<f32>)>,
}

impl RerouteSupports {
    /// Starts a run with the edit's current values, if there is an edit.
    pub fn start(state: &mut UiState, models: &mut [Model]) -> Option<Self> {
        let edit = state.support_edit.as_mut()?;
        edit.running = true;
        edit.dirty = false;

        let mut changes = HashMap::<ModelId, Vec<_>>::new();
        for support in &edit.supports {
            let change = (
                support.id,
                edit.preset_for(support),
                support.contact,
                support.tip,
            );
            changes.entry(support.model).or_default().push(change);
        }

        let jobs: Vec<Job> = (changes.into_iter())
            .filter_map(|(id, changes)| {
                let model = models.iter_mut().find(|m| m.id == id)?;
                Some(Job {
                    model: id,
                    bvh: model.bvh.clone()?,
                    base: model.supports.edit_base(),
                    mesh: model.mesh.clone(),
                    changes,
                })
            })
            .collect();

        // Each support is routed, then put in.
        let count = jobs.iter().map(|job| job.changes.len()).sum::<usize>();
        let progress = Progress::new();
        progress.set_total(2 * count as u64);
        let worker = progress.clone();
        Some(Self {
            handle: TaskThread::spawn(move || {
                let out = jobs.into_iter().map(|job| run(job, &worker)).collect();
                worker.set_finished();
                out
            }),
            progress,
            count,
        })
    }
}

fn run(job: Job, progress: &Progress) -> Rerouted {
    let (mut auto, mut manual) = ((*job.base.0).clone(), (*job.base.1).clone());
    // Routing is the slow part, and each support's is its own: route them
    // all in parallel, then put them in one by one.
    let routes: Vec<_> = (job.changes.into_par_iter())
        .map(|(id, preset, contact, tip)| {
            let normal = surface_normal(&job.mesh, &job.bvh, contact, tip);
            let routed = route_point(&job.mesh, &job.bvh, &preset, contact, normal);
            progress.add_complete(1);
            (id, preset, routed)
        })
        .collect();
    let mut unrouted = Vec::new();
    for (id, preset, routed) in routes {
        progress.add_complete(1);
        match routed {
            Some(routed) => replace(
                &mut auto,
                &mut manual,
                id,
                &preset,
                &routed,
                &job.mesh,
                &job.bvh,
            ),
            None => unrouted.push(id),
        }
    }
    Rerouted {
        model: job.model,
        base: job.base,
        sets: (auto, manual),
        unrouted,
    }
}

impl Task for RerouteSupports {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        let rerouted = match self.handle.poll(app, "Failed to Re-route Supports") {
            TaskResult::Pending => return PollResult::pending(),
            TaskResult::Completed(rerouted) => rerouted,
            // Nothing to apply. End the edit, or it would wait on this run
            // forever and take no further values.
            TaskResult::Failed | TaskResult::Empty => {
                app.state.support_edit = None;
                return PollResult::complete();
            }
        };
        let Some(edit) = app.state.support_edit.as_mut() else {
            return PollResult::complete();
        };
        for out in rerouted {
            let Some(model) = app.project.model(out.model) else {
                continue;
            };
            // Supports changed some other way meanwhile: run again.
            if model.supports.swap_sets(&out.base, out.sets) {
                edit.unrouted.retain(|x| x.model != out.model);
                (edit.unrouted)
                    .extend((out.unrouted.iter()).map(|&id| SelectedSupport::new(out.model, id)));
            } else {
                edit.dirty = true;
            }
        }
        edit.running = false;

        if edit.dirty {
            let next = RerouteSupports::start(app.state, &mut app.project.models);
            return next.map_or(PollResult::complete(), |t| {
                PollResult::complete().with_task(t)
            });
        }
        if edit.released {
            app.state.end_support_edit();
        }
        PollResult::complete()
    }

    fn status(&self) -> Option<TaskStatus<'_>> {
        Some(TaskStatus {
            name: "Updating Supports".into(),
            details: Some(match self.count {
                1 => "Re-routing 1 support".into(),
                n => format!("Re-routing {n} supports"),
            }),
            progress: self.progress.progress(),
        })
    }
}
