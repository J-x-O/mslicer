use clone_macro::clone;
use common::{progress::Progress, units::Milimeters};
use nalgebra::Vector3;
use tools::supports::{
    auto::{AutoPlacement, BraceNetwork, Generated, PlacementConfig},
    mesh::SupportPreset,
};

use crate::{
    core::project::model::{Model, ModelId},
    task::{PollResult, Task, TaskApp, TaskStatus, thread::TaskThread},
};

/// The routed supports plus the brace network computed over them.
struct RoutedWithBraces {
    generated: Generated,
    network: BraceNetwork,
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

        if let Some(bvh) = model.bvh.clone() {
            let mesh = model.mesh.clone();
            return Some(Self {
                model: model.id,
                preset: support,

                handle: TaskThread::spawn(clone!([progress], move || {
                    let generated = generator.generate_supports(&mesh, &bvh, &progress);
                    let network = generator.generate_braces(&mesh, &bvh, &generated);
                    RoutedWithBraces { generated, network }
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
                    model.supports.replace_auto_with_braces(
                        &self.preset,
                        out.generated,
                        out.network,
                    );
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
