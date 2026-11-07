//! Hollowing in the app: keeps each hollowed model's cavity and drilled print
//! mesh up to date, edits a model's hollowing with undo, and places holes
//! where the model is clicked while hole placement is on.

use std::{
    sync::{Arc, atomic::AtomicBool},
    time::Instant,
};

use nalgebra::Vector3;

use crate::{
    core::{
        App,
        history::ModelAction,
        project::{
            hollow::{Hole, HollowSpec},
            model::ModelId,
        },
    },
    interface::panels::Tab,
    task::{BuildAccelerationStructures, DrillHoles, HollowModel, Task},
};

/// Extra reach (mm) past a hole's rim within which a click removes it.
const REMOVE_SLACK: f32 = 0.5;

/// Runs every frame: starts computing the cavities hollowed models are
/// missing (once their settings have settled) and drilling the holes their
/// print meshes are missing, and ends hole placement once the Hollowing tab
/// is closed.
pub fn update(app: &mut App) {
    if app.panels.find_tab(&Tab::Hollow).is_none() {
        app.state.hole_placement = false;
    }
    if !app.state.hole_placement {
        app.state.hole_preview = None;
    }
    if app.panels.find_tab(&Tab::Hollow).is_none() {
        app.state.hole_highlight = None;
    }

    let diameter = app.state.hole_settings.diameter;
    let now = Instant::now();
    let mut tasks: Vec<Box<dyn Task>> = Vec::new();
    for model in app.project.models.iter_mut() {
        if let Some(key) = model.hollow.poll(&model.mesh, now) {
            let cancel = Arc::new(AtomicBool::new(false));
            model.hollow.pending = Some((key, cancel.clone()));
            tasks.push(Box::new(HollowModel::new(model, key, cancel, diameter)));
        }
        let hollow = &mut model.hollow;
        if !hollow.drilling
            && hollow.wants_drilling(&model.mesh)
            && let Some(input) = hollow.drill_input(&model.mesh)
        {
            hollow.drilling = true;
            tasks.push(Box::new(DrillHoles::new(model.id, input)));
        }
    }
    tasks.into_iter().for_each(|task| app.tasks.add_boxed(task));
}

/// Changes a model's hollowing. With `track`, the change can be undone;
/// leave it off for the steps of a drag after the first.
pub fn edit(app: &mut App, id: ModelId, track: bool, change: impl FnOnce(&mut HollowSpec)) {
    let Some(model) = app.project.model(id) else {
        return;
    };
    let before = model.hollow.spec.clone();
    change(&mut model.hollow.spec);
    if track && model.hollow.spec != before {
        app.history.track_model(id, ModelAction::Hollow(before));
    }
}

/// Adds the suggested drain holes of a model's cavity, except where a hole
/// already is.
pub fn add_drains(app: &mut App, id: ModelId) {
    let diameter = app.state.hole_settings.diameter;
    let Some(model) = app.project.models.iter().find(|m| m.id == id) else {
        return;
    };
    let Some(cavity) = model.hollow.cavity(&model.mesh) else {
        return;
    };
    let mesh = &model.mesh;
    let shell = cavity.shell;
    let existing: Vec<_> = (model.hollow.spec.holes.iter())
        .map(|h| h.world(mesh, shell))
        .collect();
    let new: Vec<Hole> = (cavity.drains.iter())
        .map(|h| Hole { diameter, ..*h })
        .filter(|h| {
            let at = h.world(mesh, shell).point;
            !existing
                .iter()
                .any(|e| (e.point - at).norm() < e.diameter + diameter)
        })
        .collect();
    if !new.is_empty() {
        edit(app, id, true, |spec| spec.holes.extend(new));
    }
}

/// Points the camera into a model's hole, from a little off its axis so its
/// wall shows.
pub fn focus(app: &mut App, id: ModelId, index: usize) {
    let Some(model) = app.project.models.iter().find(|m| m.id == id) else {
        return;
    };
    let Some(hole) = model.hollow.spec.holes.get(index) else {
        return;
    };
    let Some(shell) = model.hollow.spec.shell else {
        return;
    };
    let hole = hole.world(&model.mesh, shell);
    let reach = Vector3::repeat((hole.diameter * 3.0).max(4.0));
    app.camera.frame(hole.point - reach, hole.point + reach);
    // Looking down the drill: from outside, against its direction.
    let from = -hole.direction;
    if from.xy().norm() > 1e-3 {
        app.camera.angle.x = from.y.atan2(from.x);
    }
    app.camera.angle.y = from
        .z
        .clamp(-1.0, 1.0)
        .asin()
        .clamp(-FOCUS_PITCH, FOCUS_PITCH);
    app.state.camera_focus = Some(Default::default());
    app.state.selected.select_model(id);
}

/// The steepest the camera looks into a hole (radians): straight down a
/// vertical drain would hide its wall.
const FOCUS_PITCH: f32 = 1.1;

/// While placing holes: the hole a click would drill, to preview it.
pub fn track_preview(app: &mut App) {
    app.state.hole_preview = app
        .state
        .hole_placement
        .then(|| pointer_hole(app).map(|(.., world)| world))
        .flatten();
    if app.state.hole_preview.is_none() {
        prepare_hovered(app);
    }
}

/// Starts building the hit-testing structures of the model under the
/// pointer, if it has none yet.
fn prepare_hovered(app: &mut App) {
    if let Some(hover) = app.state.hovered_geometry
        && let Some(model) = app.project.models.iter().find(|m| m.id == hover.model)
        && let Some(task) = BuildAccelerationStructures::request(&mut app.state, model)
    {
        app.tasks.add(task);
    }
}

/// A click while placing holes: drills a hole where the model is clicked, or
/// removes the hole clicked on.
pub fn click(app: &mut App) {
    let Some((id, hole, world)) = pointer_hole(app) else {
        // The model under the pointer can't be hit-tested yet.
        prepare_hovered(app);
        return;
    };
    let Some(model) = app.project.models.iter().find(|m| m.id == id) else {
        return;
    };
    let Some(shell) = model.hollow.spec.shell else {
        return;
    };
    let clicked = (model.hollow.spec.holes.iter()).position(|h| {
        let h = h.world(&model.mesh, shell);
        (h.point - world.point).norm() < h.diameter / 2.0 + REMOVE_SLACK
    });
    match clicked {
        Some(index) => edit(app, id, true, |spec| {
            spec.holes.remove(index);
        }),
        None => edit(app, id, true, |spec| spec.holes.push(hole)),
    }
}

/// Where the pointer meets the nearest model, if it is hollowed: the model,
/// a hole drilled straight in there (in the model's coordinates) and the
/// same in world space.
fn pointer_hole(app: &App) -> Option<(ModelId, Hole, tools::hollow::Hole)> {
    let (pos, dir) = app.hovered_ray()?;
    let settings = app.state.hole_settings;
    let (_, model, hit) = (app.project.models.iter())
        .filter(|m| !m.hidden)
        .filter_map(|m| {
            let hit = m.bvh.as_ref()?.intersect_ray(&m.mesh, pos, dir)?;
            let distance = (m.mesh.transform(&hit.position) - pos).norm();
            Some((distance, m, hit))
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))?;

    let shell = model.hollow.spec.shell?;
    let normal = model.mesh.normal(hit.face).try_normalize(f32::EPSILON)?;
    let hole = Hole {
        point: hit.position,
        direction: -normal,
        diameter: settings.diameter,
    };
    Some((model.id, hole, hole.world(&model.mesh, shell)))
}
