use std::{sync::Arc, thread};

use clone_macro::clone;
use const_format::concatcp;
use egui_phosphor::regular::CARET_RIGHT;
use nalgebra::Vector3;
use parking_lot::Mutex;
use tracing::info;

use crate::{
    core::{
        App,
        project::PostProcessing,
        slice::{InteractivePreviews, SliceOperation, key::slice_key},
        state::PreviewModel,
    },
    engine::CancelToken,
    interface::{
        panels::Tab,
        popup::{Popup, PopupIcon},
    },
    render::camera::Camera,
    util::misc::invert_color,
};
use common::{
    progress::CombinedProgress,
    slice::{Layer, SliceConfig, SliceMode, VectorLayer},
    units::Milimeter,
};
use slicer::{
    geometry::{bounds::Bounds3D, mesh::Mesh},
    slicer::{Slicer, SlicerModel},
};

impl App {
    pub fn slice(&mut self) {
        let device = &self.render_state.device;
        let meshes = (self.project.models.iter_mut())
            .filter(|x| !x.hidden)
            .map(|x| {
                (
                    (
                        (x.stack.supports_active())
                            .then(|| x.supports.finished_mesh().as_ref().map(|x| x.0.clone()))
                            .flatten(),
                        x.supports.preview(device, invert_color(x.color)),
                    ),
                    x.clone(),
                )
            })
            .collect::<Vec<_>>();

        if meshes.is_empty() {
            const NO_MODELS_ERROR: &str = concatcp!(
                "There are no models to slice. Add one by going to File ",
                CARET_RIGHT,
                " Import Model or drag and drop a model file into the workspace."
            );
            self.popup.open(Popup::simple(
                "Empty Workspace",
                PopupIcon::Error,
                NO_MODELS_ERROR,
            ));
            return;
        }

        // A hollowed model slices only once its cavity and holes are done.
        if let Some(model) = (self.project.models.iter()).find(|m| !m.hidden && !m.ready()) {
            self.popup.open(Popup::simple(
                "Still Hollowing",
                PopupIcon::Info,
                format!(
                    "The cavity or the holes of `{}` are still being computed. Slice again once they are done.",
                    model.name
                ),
            ));
            return;
        }

        info!("Starting slicing operation");

        let slice_config = self.project.slice_config.clone();
        let mut preview_models = Vec::new();
        let mut bounds = Bounds3D::empty();
        let mut parts = Vec::new();
        for (supports, mut model) in meshes.into_iter() {
            // preview
            bounds.extend(&model.bounds);
            preview_models.push(PreviewModel::for_model(device, &mut model));

            let (support_mesh, support_preview) = match supports {
                (Some(mesh), Some(preview)) => (Some(mesh), Some(preview)),
                _ => (None, None),
            };
            preview_models.extend(support_preview);
            parts.push(SlicePart {
                print: model.print_mesh(),
                supports: support_mesh,
                exposure: model.exposure,
            });
        }
        let (out, triangles) = platform_models(&slice_config, parts);

        let preview = &self.config.render.preview;

        let aspect = preview.size.x as f32 / preview.size.y as f32;
        let (target, radius) = bounds.bounding_sphere();
        let mut camera = Camera {
            fov: preview.fov,
            target,
            ..Default::default()
        };
        camera.visible_sphere(aspect, radius);

        let previews = InteractivePreviews {
            camera: Mutex::new(camera),
            models: preview_models,
        };

        let slicer = Slicer::new(slice_config, out);
        let post_process = CombinedProgress::new();
        let slice_operation =
            SliceOperation::new(slicer.progress(), post_process.clone(), Some(previews))
                .with_key(slice_key(&self.project));
        // Latest wins: a slice still running is stopped.
        let running = slice_operation.clone();
        if let Some(old) = self.slice_operation.replace(slice_operation) {
            old.cancel.cancel();
        }
        self.panels.focus_tab(Tab::Sliced);

        let triangles = Some(triangles);
        thread::spawn(clone!(
            [
                { running } as slice_operation,
                { self.project.post_processing } as post_processing
            ],
            move || {
                let slice_operation = &slice_operation;
                let cancel = &slice_operation.cancel;
                match run_slicer(&slicer, &post_processing, post_process, cancel) {
                    // Superseded by a newer slice.
                    None => {}
                    Some(SliceOutput::Raster(layers)) => {
                        slice_operation.add_raster_result(slicer.slice_config, layers, triangles);
                    }
                    Some(SliceOutput::Vector(layers)) => {
                        slice_operation.add_vector_result(
                            slicer.slice_config,
                            Arc::new(layers),
                            triangles,
                        );
                    }
                }
            }
        ));
    }
}

/// One visible model as it goes into the slicer: its print mesh (hollowed
/// and drilled, if it is), its support mesh if it has supports, and its
/// relative exposure. Both meshes are placed in world space (mm).
pub struct SlicePart {
    pub print: Mesh,
    pub supports: Option<Mesh>,
    pub exposure: u8,
}

/// The output of a full slice, before it is wrapped into a [`SliceResult`].
///
/// [`SliceResult`]: crate::core::slice::result::SliceResult
pub enum SliceOutput {
    Raster(Vec<Layer>),
    Vector(Vec<VectorLayer>),
}

/// Moves every part from world space (mm) into platform space (px) and makes
/// a [`SlicerModel`] of each mesh, print mesh first, then its supports.
/// Returns the models and their total triangle count.
///
/// Shared by [`App::slice`] and the headless regression harness, so both
/// slice exactly the same thing.
pub fn platform_models(
    slice_config: &SliceConfig,
    parts: impl IntoIterator<Item = SlicePart>,
) -> (Vec<SlicerModel>, u64) {
    let slice_height = slice_config.slice_height.get::<Milimeter>();
    let platform_size = (slice_config.platform_size.xy()).map(|x| x.get::<Milimeter>());

    let platform = slice_config.platform_resolution.cast::<f32>();
    let mm_to_px = platform.component_div(&platform_size).push(1.0);
    let offset = (platform / 2.0).push(-slice_height / 2.0);

    let mut out = Vec::new();
    let mut triangles = 0;
    for part in parts {
        let meshes = std::iter::once(part.print).chain(part.supports);
        for mut mesh in meshes {
            transform_mesh(&mut mesh, mm_to_px, offset);
            triangles += mesh.face_count() as u64;
            let bounds = Bounds3D::from_mesh(&mesh); // todo: no need to reaclc since only translation and scale are applied
            out.push(SlicerModel {
                mesh,
                bounds,
                exposure: part.exposure,
            });
        }
    }
    (out, triangles)
}

/// Runs a full slice: slicing, then (raster only) the post-processors.
/// Gives up (`None`) once `cancel` is set: it is checked before each layer
/// and before post-processing.
pub fn run_slicer(
    slicer: &Slicer,
    post_processing: &PostProcessing,
    progress: CombinedProgress<2>,
    cancel: &CancelToken,
) -> Option<SliceOutput> {
    Some(match slicer.slice_config.mode {
        SliceMode::Raster => {
            let mut layers = slicer.slice_raster_until(&|| cancel.is_cancelled())?;
            cancel.check().ok()?;
            post_processing.process(&slicer.slice_config, &mut layers, progress);
            SliceOutput::Raster(layers)
        }
        SliceMode::Vector => SliceOutput::Vector(slicer.slice_vector()),
    })
}

fn transform_mesh(mesh: &mut Mesh, mm_to_px: Vector3<f32>, offset: Vector3<f32>) {
    mesh.set_scale_unchecked(mesh.scale().component_mul(&mm_to_px));
    mesh.set_position_unchecked(mesh.position().component_mul(&mm_to_px) + offset);
    mesh.update_transformation_matrix();
}
