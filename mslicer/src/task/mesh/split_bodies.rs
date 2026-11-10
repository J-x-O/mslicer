use std::{collections::HashMap, mem};

use clone_macro::clone;
use common::{container::ArrayCluster, progress::Progress};
use nalgebra::Vector3;
use slicer::geometry::mesh::Mesh;

use crate::{
    core::project::{
        Collection,
        hollow::HollowSpec,
        model::{MeshUnit, Model, ModelId},
        stack::Stack,
    },
    task::{MeshAnalysis, PollResult, Task, TaskApp, TaskStatus, thread::TaskThread},
};

pub struct SplitBodies {
    parent: ModelId,
    collection: Collection,
    /// What each body takes over from the model: its placement, stack and
    /// hollowing.
    template: Template,

    progress: Progress,
    handle: TaskThread<Vec<Mesh>>,
}

struct Template {
    position: Vector3<f32>,
    scale: Vector3<f32>,
    rotation: Vector3<f32>,
    stack: Stack,
    hollow: HollowSpec,
    exposure: u8,
    unit: MeshUnit,
}

impl SplitBodies {
    /// Splits the model's source into its bodies. Each becomes a model of
    /// its own, placed like the model and with a copy of its stack, so the
    /// operations run on each body. (Hand-placed and generated supports
    /// belong to the whole and aren't copied.)
    pub fn new(model: &mut Model) -> Self {
        let collection = Collection::new(model.name.to_owned());
        let mesh = Mesh::from_inner(model.source.clone());
        let template = Template {
            position: model.mesh.position(),
            scale: model.mesh.scale(),
            rotation: model.mesh.rotation(),
            stack: model.stack.clone(),
            hollow: model.hollow.spec.clone(),
            exposure: model.exposure,
            unit: model.unit,
        };

        let progress = Progress::new();
        Self {
            parent: model.id,
            handle: TaskThread::spawn(clone!([progress], move || {
                let mut clusters = ArrayCluster::new(mesh.vertex_count());
                progress.set_total(mesh.face_count() as u64 * 3);
                for [a, b, c] in mesh.faces() {
                    for (&a, &b) in [(a, b), (b, c), (c, a)] {
                        clusters.union(a, b);
                        progress.add_complete(1);
                    }
                }

                let mut out = Vec::new();
                for body in clusters.clusters() {
                    let mut verts = Vec::new();
                    let mut vert_cache = HashMap::<u32, u32>::new();
                    let mut faces = Vec::new();

                    let mut vert = |old: u32| {
                        if let Some(new) = vert_cache.get(&old) {
                            *new
                        } else {
                            let new = verts.len() as u32;
                            verts.push(mesh.vertices()[old as usize]);
                            vert_cache.insert(old, new);
                            new
                        }
                    };

                    // if {a, b, c} ∈ body
                    for f @ [a, ..] in mesh.faces() {
                        body.contains(a).then(|| faces.push(f.map(&mut vert)));
                    }

                    out.push(Mesh::new(verts, faces));
                }

                progress.set_finished();
                out
            })),
            collection,
            template,
            progress,
        }
    }
}

impl Task for SplitBodies {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        self.handle
            .poll(app, "Unexpected Error Splitting Bodies")
            .into_poll_result(|meshes| {
                let mut result = PollResult::complete();

                let collection = mem::take(&mut self.collection);
                let template = &self.template;
                for (i, mut mesh) in meshes.into_iter().enumerate() {
                    mesh.set_position_unchecked(template.position);
                    mesh.set_scale_unchecked(template.scale);
                    mesh.set_rotation_unchecked(template.rotation);
                    mesh.update_transformation_matrix();
                    let mut model = Model::from_mesh(mesh)
                        .with_name(format!("Body {i}"))
                        .with_collection(Some(collection.id))
                        .with_exposure(template.exposure)
                        .with_unit(template.unit)
                        .with_random_color(app.config.ui.warning_color);
                    model.stack = template.stack.copied();
                    model.hollow.spec = template.hollow.clone();
                    model.update_oob(&app.project.slice_config.platform_size);
                    result = result.with_task(MeshAnalysis::new(&model));
                    app.project.models.push(model);
                }

                if let Some(model) = app.project.model(self.parent) {
                    model.collection = Some(collection.id);
                    model.hidden = true;
                }

                app.project.collections.push(collection);
                result
            })
    }

    fn status(&self) -> Option<TaskStatus<'_>> {
        Some(TaskStatus {
            name: "Splitting Bodies".into(),
            details: None,
            progress: self.progress.progress(),
        })
    }
}
