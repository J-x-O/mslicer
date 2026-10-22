use clone_macro::clone;
use common::{progress::Progress, units::Milimeters};
use nalgebra::Vector3;
use tools::supports::{
    auto::{AutoPlacement, PlacementConfig},
    mesh::SupportPreset,
};

use crate::{
    core::project::model::{Model, ModelId},
    task::{PollResult, Task, TaskApp, TaskStatus, thread::TaskThread},
};

type RoutedSupports = Vec<[Vector3<f32>; 3]>;

/// The routed triplets plus the brace network computed over them.
struct RoutedWithBraces {
    routed: RoutedSupports,
    braces: Vec<tools::supports::Part>,
}

pub struct GenerateSupports {
    model: ModelId,
    preset: SupportPreset,

    progress: Progress,
    handle: TaskThread<RoutedWithBraces>,
}

impl GenerateSupports {
    pub fn new(
        platform: Vector3<Milimeters>,
        placement: PlacementConfig,
        support: SupportPreset,
        model: &Model,
    ) -> Option<Self> {
        let progress = Progress::new();
        let generator = AutoPlacement::new(placement, support.clone(), platform);

        if let Some(half_ege) = model.half_edge.clone()
            && let Some(bvh) = model.bvh.clone()
        {
            let mesh = model.mesh.clone();
            return Some(Self {
                model: model.id,
                preset: support,

                handle: TaskThread::spawn(clone!([progress], move || {
                    let routed = generator.generate_supports(&mesh, &half_ege, &bvh, &progress);
                    let braces = generator.generate_braces(&routed);
                    RoutedWithBraces { routed, braces }
                })),
                progress,
            });
        }

        None
    }
}

impl Task for GenerateSupports {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        self.handle
            .poll(app, "Failed to Generate Supports")
            .into_poll_result(|out| {
                if let Some(model) = app.project.model(self.model) {
                    model
                        .supports
                        .replace_auto_with_braces(&self.preset, out.routed, out.braces);
                }

                PollResult::complete()
            })
    }

    fn status(&self) -> Option<TaskStatus<'_>> {
        Some(TaskStatus {
            name: "Generating Supports".into(),
            details: None,
            progress: self.progress.progress(),
        })
    }
}
