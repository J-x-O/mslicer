//! Hollowing in the app: keeps each hollowed model's cavity and drilled print
//! mesh up to date, edits a model's hollowing with undo, and the hole tool:
//! placing, selecting, dragging and removing drain holes in the viewport.

use std::{
    sync::{Arc, atomic::AtomicBool},
    time::Instant,
};

use nalgebra::{Vector2, Vector3};
use slicer::geometry::bounds::Bounds2D;

use crate::{
    core::{
        App,
        history::ModelAction,
        project::{
            hollow::{Hole, HollowSpec},
            model::{Model, ModelId},
        },
    },
    interface::panels::Tab,
    task::{BuildAccelerationStructures, DrillHoles, HollowModel, Task},
};

/// Extra reach (mm) past a hole's rim within which the pointer is on it.
const GRAB_SLACK: f32 = 0.5;

/// A hole of a model, by its place in the model's list.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HoleRef {
    pub model: ModelId,
    pub index: usize,
}

/// A hole being dragged, and the model's hollowing before, to undo the drag
/// in one step.
pub struct HoleDrag {
    pub hole: HoleRef,
    pub before: HollowSpec,
}

/// Runs every frame: starts computing the cavities hollowed models are
/// missing (once their settings have settled) and drilling the holes their
/// print meshes are missing (not while one is dragged), and forgets holes
/// that are gone.
pub fn update(app: &mut App) {
    let exists = |models: &[Model], h: &HoleRef| {
        (models.iter()).any(|m| m.id == h.model && h.index < m.hollow.spec.holes.len())
    };
    let models = &app.project.models;
    app.state.selected_holes.retain(|h| exists(models, h));
    if (app.state.hovered_hole).is_some_and(|h| !exists(models, &h)) {
        app.state.hovered_hole = None;
    }
    if app.panels.find_tab(&Tab::Hollow).is_none() {
        app.state.listed_hole = None;
    }

    let diameter = app.state.hole_settings.diameter;
    let dragged = app.state.hole_drag.as_ref().map(|d| d.hole.model);
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
            && dragged != Some(model.id)
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

/// In the hole tool, while nothing is dragged: the hole under the pointer,
/// or else the hole a click would drill there.
pub fn track_hover(app: &mut App) {
    let target = pointer_hole(app);
    app.state.hovered_hole = target
        .as_ref()
        .and_then(|(id, _, world)| hole_at(app, *id, world));
    app.state.hole_preview = match app.state.hovered_hole {
        Some(_) => None,
        None => target.map(|(.., world)| world),
    };
    if target.is_none() {
        prepare_hovered(app);
    }
}

/// Ends hovering, out of the hole tool or while the view moves.
pub fn clear_hover(app: &mut App) {
    app.state.hovered_hole = None;
    app.state.hole_preview = None;
}

/// The hole of model `id` whose rim `world` (a hole drilled where the
/// pointer is) falls within.
fn hole_at(app: &App, id: ModelId, world: &tools::hollow::Hole) -> Option<HoleRef> {
    let model = app.project.models.iter().find(|m| m.id == id)?;
    let shell = model.hollow.spec.shell?;
    let (index, _) = (model.hollow.spec.holes.iter().enumerate())
        .map(|(index, h)| (index, h.world(&model.mesh, shell)))
        .filter(|(_, h)| (h.point - world.point).norm() < h.diameter / 2.0 + GRAB_SLACK)
        .min_by(|a, b| {
            let d = |h: &tools::hollow::Hole| (h.point - world.point).norm();
            d(&a.1).total_cmp(&d(&b.1))
        })?;
    Some(HoleRef { model: id, index })
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

/// A click in the hole tool: selects the hole clicked (Shift adds it or
/// takes it out), drills a new one where a hollowed model is clicked, or
/// with nothing under the pointer clears the selection.
pub fn click(app: &mut App, shift: bool) {
    if let Some(hole) = app.state.hovered_hole {
        let selected = &mut app.state.selected_holes;
        match (shift, selected.iter().position(|h| *h == hole)) {
            (true, Some(at)) => {
                selected.remove(at);
            }
            (true, None) => selected.push(hole),
            (false, _) => *selected = vec![hole],
        }
        return;
    }
    let Some((id, hole, _)) = pointer_hole(app) else {
        if !shift {
            app.state.selected_holes.clear();
        }
        prepare_hovered(app);
        return;
    };
    edit(app, id, true, |spec| spec.holes.push(hole));
    let Some(model) = app.project.models.iter().find(|m| m.id == id) else {
        return;
    };
    let new = HoleRef {
        model: id,
        index: model.hollow.spec.holes.len() - 1,
    };
    match shift {
        true => app.state.selected_holes.push(new),
        false => app.state.selected_holes = vec![new],
    }
}

/// Starts dragging the hole under the pointer, if there is one. Returns
/// whether it did.
pub fn start_drag(app: &mut App) -> bool {
    let Some(hole) = app.state.hovered_hole else {
        return false;
    };
    let Some(model) = app.project.models.iter().find(|m| m.id == hole.model) else {
        return false;
    };
    if !app.state.selected_holes.contains(&hole) {
        app.state.selected_holes = vec![hole];
    }
    app.state.hole_preview = None;
    app.state.hole_drag = Some(HoleDrag {
        hole,
        before: model.hollow.spec.clone(),
    });
    true
}

/// While dragging a hole: moves it to where the pointer meets its model,
/// drilling straight in there. Once `released`, the drag is done and can be
/// undone as one step.
pub fn drag(app: &mut App, released: bool) {
    let Some(HoleDrag { hole, .. }) = app.state.hole_drag else {
        return;
    };
    if let Some((id, moved, _)) = pointer_hole(app)
        && id == hole.model
    {
        edit(app, id, false, |spec| {
            if let Some(at) = spec.holes.get_mut(hole.index) {
                *at = Hole {
                    diameter: at.diameter,
                    ..moved
                };
            }
        });
    }
    if released && let Some(drag) = app.state.hole_drag.take() {
        let changed =
            (app.project.models.iter()).any(|m| m.id == hole.model && m.hollow.spec != drag.before);
        if changed {
            (app.history).track_model(hole.model, ModelAction::Hollow(drag.before));
        }
    }
}

/// Stops a drag and puts the hole back. Returns whether one was under way.
pub fn cancel_drag(app: &mut App) -> bool {
    let Some(drag) = app.state.hole_drag.take() else {
        return false;
    };
    if let Some(model) = app.project.model(drag.hole.model) {
        model.hollow.spec = drag.before;
    }
    true
}

/// Removes the selected holes, each model's in one undo step.
pub fn delete_selected(app: &mut App) {
    let selected = std::mem::take(&mut app.state.selected_holes);
    let mut models: Vec<ModelId> = selected.iter().map(|h| h.model).collect();
    models.dedup();
    for id in models {
        let mut indices: Vec<usize> = (selected.iter())
            .filter(|h| h.model == id)
            .map(|h| h.index)
            .collect();
        indices.sort_unstable_by(|a, b| b.cmp(a));
        indices.dedup();
        edit(app, id, true, |spec| {
            indices.iter().for_each(|&index| {
                spec.holes.remove(index);
            });
        });
    }
}

/// The holes whose openings are inside `area` (in viewport uv).
pub fn holes_in(app: &App, area: &Bounds2D<f32>) -> Vec<HoleRef> {
    let view_projection = app.view_projection();
    let inside = |p: Vector3<f32>| {
        let clip = view_projection * p.push(1.0);
        if clip.w <= 0.0 {
            return false;
        }
        let ndc = clip.xy() / clip.w;
        let uv = Vector2::new(ndc.x + 1.0, 1.0 - ndc.y) / 2.0;
        (area.min.x..=area.max.x).contains(&uv.x) && (area.min.y..=area.max.y).contains(&uv.y)
    };
    let mut out = Vec::new();
    for model in app.project.models.iter().filter(|m| !m.hidden) {
        let Some(shell) = model.hollow.spec.shell else {
            continue;
        };
        for (index, hole) in model.hollow.spec.holes.iter().enumerate() {
            if inside(hole.world(&model.mesh, shell).point) {
                out.push(HoleRef {
                    model: model.id,
                    index,
                });
            }
        }
    }
    out
}

/// Selects every hole of every hollowed model.
pub fn select_all(app: &mut App) {
    app.state.selected_holes = (app.project.models.iter())
        .filter(|m| m.hollow.spec.shell.is_some())
        .flat_map(|m| {
            (0..m.hollow.spec.holes.len()).map(move |index| HoleRef { model: m.id, index })
        })
        .collect();
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
            let hit = m.visible_hit(pos, dir, app.state.cut_height)?;
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
