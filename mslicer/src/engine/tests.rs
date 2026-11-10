//! The engine's acceptance tests, on the regression fixtures.

use std::sync::atomic::{AtomicUsize, Ordering};

use common::{progress::CombinedProgress, slice::Layer};
use nalgebra::Vector3;
use slicer::slicer::Slicer;

use crate::{
    core::{
        project::{Project, hollow::Shell},
        slice::{
            key::model_key,
            runner::{SliceOutput, SlicePart, platform_models, run_slicer},
        },
    },
    engine::{CancelToken, SlotState, layer::LayerPreview},
    regression::{load_prepared, project_path, slice},
};

fn same(a: &Layer, b: &Layer) -> bool {
    a.data == b.data && a.height == b.height && a.exposure == b.exposure && a.defects == b.defects
}

fn layer(preview: &mut LayerPreview, project: &Project, index: u32) -> Layer {
    if preview.request(project, index).is_none() {
        preview.wait();
    }
    (*preview.request(project, index).expect("sliced")).clone()
}

#[test]
fn a_single_layer_slices_as_in_the_full_slice() {
    for name in ["teapot", "hollowed", "supported", "vlh_eff"] {
        let mut project = load_prepared(&project_path(name));
        // Post-processing works on the whole stack; the single layer is the
        // slicer's own.
        project.post_processing = Default::default();
        let SliceOutput::Raster(layers) = slice(&mut project) else {
            unreachable!()
        };
        let mut preview = LayerPreview::default();
        let last = layers.len() as u32 - 1;
        for index in [0, 7, last / 2, last] {
            let single = layer(&mut preview, &project, index);
            assert!(
                same(&single, &layers[index as usize]),
                "{name} layer {index}"
            );
        }
    }
}

#[test]
fn a_superseded_full_slice_stops_early() {
    let mut project = load_prepared(&project_path("teapot"));
    let parts: Vec<_> = (project.models.iter_mut())
        .map(|m| SlicePart {
            supports: m.supports.finished_mesh().as_ref().map(|x| x.0.clone()),
            print: m.print_mesh(),
            exposure: m.exposure,
        })
        .collect();
    let (models, _) = platform_models(&project.slice_config, parts);
    let slicer = Slicer::new(project.slice_config.clone(), models);
    let total = slicer.layer_count() as usize;

    let checks = AtomicUsize::new(0);
    let result = slicer.slice_raster_until(&|| checks.fetch_add(1, Ordering::Relaxed) >= 8);
    assert!(result.is_none());
    let done = slicer.progress().complete() as usize;
    assert!(done < total / 2, "{done} of {total} layers sliced");

    // Through the app's path: a cancelled token gives nothing back.
    let cancel = CancelToken::new();
    cancel.cancel();
    let post = &project.post_processing;
    assert!(run_slicer(&slicer, post, CombinedProgress::new(), &cancel).is_none());
}

#[test]
fn a_stale_layer_is_discarded() {
    let project = load_prepared(&project_path("teapot"));
    let mut preview = LayerPreview::default();
    assert!(preview.request(&project, 3).is_none(), "starts");
    // The scrubber moves on before layer 3 is in.
    assert!(preview.request(&project, 5).is_none());
    preview.wait();
    assert!(preview.latest().is_none(), "layer 3 came in for nobody");
    assert_eq!(preview.state(), SlotState::Stale);
    let five = layer(&mut preview, &project, 5);
    assert_eq!(five.height, project.slice_config.default_height(5));
}

#[test]
fn editing_one_model_leaves_the_others_caches_alone() {
    let mut project = load_prepared(&project_path("hollowed"));
    let platform = project.slice_config.platform_size;
    let mut other = project.models[0].clone();
    let at = other.mesh.position();
    other.set_position(&platform, at + Vector3::x() * 25.0);
    project.models.push(other);
    let (a, b) = (0, 1);
    let mesh = |p: &Project, i: usize| p.models[i].mesh.clone();
    assert!(
        project.models[b]
            .hollow
            .cavity(&mesh(&project, b))
            .is_some()
    );
    let key_a = model_key(&project.models[a]);

    // A thicker wall on one model: only its cavity goes stale.
    project.models[b].hollow.spec.shell = Some(Shell {
        wall: 3.0,
        cell: 0.5,
    });
    assert!(
        project.models[b]
            .hollow
            .cavity(&mesh(&project, b))
            .is_none()
    );
    assert!(
        project.models[a]
            .hollow
            .cavity(&mesh(&project, a))
            .is_some()
    );
    assert!(
        project.models[a]
            .hollow
            .drilled(&mesh(&project, a))
            .is_some()
    );
    assert_eq!(model_key(&project.models[a]), key_a);

    // Moving a model on the plate keeps its whole stack, and changes only
    // its own contribution to the slice.
    let moved = &mut project.models[a];
    let at = moved.mesh.position();
    moved.set_position(&platform, at + Vector3::new(1.0, 2.0, 0.0));
    let moved = &project.models[a];
    assert!(moved.hollow.cavity(&moved.mesh).is_some());
    assert!(moved.hollow.drilled(&moved.mesh).is_some());
    assert_ne!(model_key(moved), key_a);
}

#[test]
fn moving_a_supported_model_keeps_its_support_mesh() {
    let mut project = load_prepared(&project_path("supported"));
    let platform = project.slice_config.platform_size;
    let model = &mut project.models[0];
    model.supports.finished_mesh();
    assert!(model.supports.is_current());
    let at = model.mesh.position();
    model.set_position(&platform, at + Vector3::new(3.0, -2.0, 0.0));
    assert!(model.supports.is_current());
}
