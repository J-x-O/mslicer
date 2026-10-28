use std::collections::{HashMap, HashSet};

use common::units::{Milimeter, Milimeters};
use const_format::concatcp;
use egui::{Button, ComboBox, Context, DragValue, Ui, Widget};
use egui_phosphor::regular::{ERASER, LINE_SEGMENT, SELECTION_INVERSE, SPARKLE, TRASH};
use nalgebra::Vector3;
use tools::supports::{
    BaseShape, SupportId,
    auto::{Routed, route_point},
    mesh::SupportPreset,
    raft::{RaftConfig, RaftShape},
};

use crate::{
    core::{
        App,
        history::ModelAction,
        project::model::ModelId,
        state::{EditedSupport, GeometryHit, SupportDrag, SupportEdit},
    },
    interface::components::grid,
    task::{BuildAccelerationStructures, GenerateSupports, RerouteSupports},
};

const IN_DEV_WARNING: &str = "Support generation and placement is still in development. For the time being consider using RuneBrace for support placement.";
const EDIT_HELP: &str = "Click a model to work on it (shift adds it), then click it to add a support. Click a support to select it (shift adds to the selection), drag a support to move its contact point, drag elsewhere to box select. Clicking empty space clears the selection. Delete removes the selected supports, Escape cancels a drag.\n\nRight drag orbits, middle drag pans (or shift + right drag), alt + left drag orbits too. F frames the selection.";

pub fn ui(app: &mut App, ui: &mut Ui, _ctx: &Context) {
    ui.horizontal_wrapped(|ui| {
        prepare(app, ui);

        model_action(app, ui, concatcp!(SPARKLE, " Auto"), generate_support);
        model_action(app, ui, concatcp!(ERASER, " Clear"), clear_supports);
    });

    let selected = &app.state.selected_supports;
    let selected_count = selected.count();
    if selected_count > 0 {
        ui.add_space(4.0);
        ui.separator();
        ui.add_space(4.0);

        let models = selected.model_count();
        ui.label(format!(
            "{selected_count} support{} selected across {models} model{}.",
            ["", "s"][(selected_count != 1) as usize],
            ["", "s"][(models != 1) as usize],
        ));

        ui.add_space(8.0);
        ui.horizontal_wrapped(|ui| {
            if ui.button(concatcp!(TRASH, " Delete")).clicked() {
                delete_selected(app);
            }

            (ui.button(concatcp!(SELECTION_INVERSE, " Deselect"))
                .clicked())
            .then(|| app.state.selected_supports.clear());
        });

        ui.add_space(4.0);
        ui.separator();
        ui.add_space(4.0);
    } else {
        ui.add_space(8.0);
    }

    match focused_support(app) {
        Some(preset) => selected_settings(app, ui, preset),
        None => {
            let preset = &mut app.project.support_config.preset;
            preset_choice(ui, preset.name().unwrap_or("Custom"), preset);
            ui.add_space(4.0);
            preset_groups(ui, preset);
        }
    }
    raft_settings(app, ui);

    let config = &mut app.project.support_config.auto_placement;
    ui.collapsing("General", |ui| {
        grid("auto_placement").show(ui, |ui| {
            ui.label("Default Lift");
            (app.project.support_config.default_lift)
                .with::<Milimeter, _>(|x| DragValue::new(x).suffix(" mm").ui(ui));
            ui.end_row();

            ui.label("Max Angle");
            DragValue::new(&mut config.max_angle).speed(0.01).ui(ui);
            ui.end_row();

            ui.label("Minimum Support Spacing");
            ui.horizontal(|ui| {
                DragValue::new(&mut config.min_spacing).ui(ui);
                ui.take_available_width();
            });
            ui.end_row();
        });
    });

    ui.collapsing("Mesh", |ui| {
        grid("mesh").show(ui, |ui| {
            ui.label("Resolution");
            DragValue::new(&mut app.project.support_config.mesh_resolution).ui(ui);
            ui.end_row();
        });
    });

    ui.add_space(4.0);
    ui.separator();
    ui.add_space(4.0);
    ui.label(EDIT_HELP);
    ui.add_space(4.0);
    ui.label(IN_DEV_WARNING);
}

/// The preset dropdown, showing `label`. Returns whether a preset was picked.
fn preset_choice(ui: &mut Ui, label: &str, preset: &mut SupportPreset) -> bool {
    let mut picked = false;
    ui.horizontal(|ui| {
        ui.label("Preset");
        ComboBox::from_id_salt("preset")
            .selected_text(label)
            .show_ui(ui, |ui| {
                for (name, value) in SupportPreset::PRESETS {
                    picked |= ui.selectable_value(preset, value, name).clicked();
                }
            });
    });
    picked
}

/// The settings of the focused selected support. Editing them re-routes every
/// selected support from its contact point: a picked preset replaces all of
/// their settings, an edited value changes just that value on each. The
/// re-routing runs in the background (see [`RerouteSupports`]).
fn selected_settings(app: &mut App, ui: &mut Ui, focused: SupportPreset) {
    let count = app.state.selected_supports.count();
    let before = (app.state.support_edit.as_ref()).map_or(focused, |edit| edit.shown.clone());
    let mut after = before.clone();
    let picked = preset_choice(ui, &format!("Selected ({count})"), &mut after);
    ui.add_space(4.0);
    unrouted_notice(app, ui);
    preset_groups(ui, &mut after);

    // A picked preset applies even if the shown support already has it:
    // the other selected supports may not.
    if after != before || picked {
        if app.state.support_edit.is_none() {
            begin_edit(app, before);
        }
        let Some(edit) = app.state.support_edit.as_mut() else {
            return;
        };
        edit.shown = after;
        edit.whole |= picked;
        edit.dirty = true;
        edit.released = false;
        if !edit.running
            && let Some(task) = RerouteSupports::start(&mut app.state, &mut app.project.models)
        {
            app.tasks.add(task);
        }
    } else if !ui.input(|i| i.pointer.any_down())
        && let Some(edit) = app.state.support_edit.as_mut()
    {
        edit.released = true;
        if !edit.running && !edit.dirty {
            app.state.end_support_edit();
        }
    }
}

/// Says how many supports the last edit found no route for, which keep their
/// old settings, with a way to select just them.
fn unrouted_notice(app: &mut App, ui: &mut Ui) {
    let edit = app.state.support_edit.as_ref();
    let unrouted = edit.map_or(&app.state.unrouted, |edit| &edit.unrouted);
    if unrouted.is_empty() {
        return;
    }
    let count = unrouted.len();
    // Unrouted supports are always among the selected ones.
    let all = count == app.state.selected_supports.count();
    let text = match (all, count) {
        (true, 1) => "The support found no route with these settings and kept its old ones.".into(),
        (true, _) => {
            "None of the supports found a route with these settings; they kept their old ones."
                .into()
        }
        (false, 1) => "1 support found no route with these settings and kept its old ones.".into(),
        (false, _) => {
            format!("{count} supports found no route with these settings and kept their old ones.")
        }
    };
    ui.colored_label(ui.visuals().warn_fg_color, text);
    if !all && ui.button("Select Them").clicked() {
        let unrouted = unrouted.clone();
        let selected = &mut app.state.selected_supports;
        selected.clear();
        for x in unrouted {
            selected.select_support(x.model, x.support);
        }
        app.state.unrouted.clear();
    }
    ui.add_space(4.0);
}

/// Starts an edit of the selected supports' settings: records the undo step
/// and what the supports look like now.
fn begin_edit(app: &mut App, origin: SupportPreset) {
    let mut supports = Vec::new();
    let mut models = HashSet::new();
    for selected in app.state.selected_supports.iter() {
        let Some(model) = app.project.models.iter().find(|m| m.id == selected.model) else {
            continue;
        };
        let Some(support) = model.supports.get(selected.support) else {
            continue;
        };
        if models.insert(model.id) {
            (app.history).track_model(model.id, ModelAction::Supports(model.supports.snapshot()));
        }
        supports.push(EditedSupport {
            model: model.id,
            id: support.id,
            preset: support.preset,
            contact: support.point.pos,
            tip: support.point.normal,
        });
    }
    app.state.unrouted.clear();
    app.state.support_edit = Some(SupportEdit {
        shown: origin.clone(),
        origin,
        whole: false,
        supports,
        running: false,
        dirty: false,
        released: false,
        unrouted: Vec::new(),
    });
}

/// The settings of the support the panel shows while supports are selected.
fn focused_support(app: &App) -> Option<SupportPreset> {
    let focus = app.state.selected_supports.focus()?;
    let model = app.project.models.iter().find(|m| m.id == focus.model)?;
    model.supports.get(focus.support).map(|s| s.preset)
}

/// The raft of the models Auto and Clear act on, or with none to go by, the
/// raft new supports get. A model's raft is its own: changing it only
/// re-meshes, nothing is generated again.
fn raft_settings(app: &mut App, ui: &mut Ui) {
    let targets = targets(app).filter(|t| !t.is_empty());
    let models = &app.project.models;
    let (label, before) = match &targets {
        Some(t) if t.len() == 1 => (
            format!("Raft of `{}`", models[t[0]].name),
            models[t[0]].supports.raft(),
        ),
        Some(t) => (
            format!("Raft of {} models", t.len()),
            models[t[0]].supports.raft(),
        ),
        None => (
            "Raft for new supports".into(),
            app.project.support_config.preset.raft_config(),
        ),
    };

    let mut after = before;
    ui.collapsing("Raft", |ui| {
        ui.label(label);
        raft_group(ui, &mut after);
    });

    let editing = after != before;
    match targets {
        Some(targets) if editing => {
            let record = !std::mem::replace(&mut app.state.raft_edit, true);
            for idx in targets {
                let model = &mut app.project.models[idx];
                if record {
                    (app.history)
                        .track_model(model.id, ModelAction::Supports(model.supports.snapshot()));
                }
                model.supports.set_raft(after);
            }
        }
        None if editing => app.project.support_config.preset.set_raft(&after),
        _ if !ui.input(|i| i.pointer.any_down()) => app.state.raft_edit = false,
        _ => {}
    }
}

fn raft_group(ui: &mut Ui, raft: &mut RaftConfig) {
    grid("support_raft").show(ui, |ui| {
        row(ui, "Shape", |ui| {
            let name = |shape| match shape {
                RaftShape::None => "None",
                RaftShape::Skate => "Skate",
                RaftShape::CrossGrid => "Cross Grid",
            };
            ComboBox::from_id_salt("raft_shape")
                .selected_text(name(raft.shape))
                .show_ui(ui, |ui| {
                    for shape in [RaftShape::None, RaftShape::Skate, RaftShape::CrossGrid] {
                        ui.selectable_value(&mut raft.shape, shape, name(shape));
                    }
                });
        });
        row(ui, "Area Ratio", |ui| {
            let mut percent = raft.area_ratio * 100.0;
            let edit = DragValue::new(&mut percent)
                .suffix(" %")
                .range(100.0..=200.0);
            if edit.ui(ui).changed() {
                raft.area_ratio = percent / 100.0;
            }
        });
        row(ui, "Thickness", |ui| mm_f32(ui, &mut raft.thickness));
        row(ui, "Lip Height", |ui| mm_f32(ui, &mut raft.height));
        row(ui, "Lip Angle", |ui| {
            DragValue::new(&mut raft.angle)
                .suffix("°")
                .range(5.0..=90.0)
                .ui(ui);
        });
        row(ui, "Grid Cell", |ui| mm_f32(ui, &mut raft.grid_cell));
        row(ui, "Grid Bar Width", |ui| mm_f32(ui, &mut raft.grid_width));
    });
}

fn mm_f32(ui: &mut Ui, value: &mut f32) {
    DragValue::new(value)
        .suffix(" mm")
        .speed(0.01)
        .range(0.0..=f32::MAX)
        .ui(ui);
}

/// The preset's support settings, grouped as top / middle / bottom. Diameters
/// are shown as diameters but stored as radii.
fn preset_groups(ui: &mut Ui, preset: &mut SupportPreset) {
    ui.collapsing("Top", |ui| {
        grid("support_top").show(ui, |ui| {
            row(ui, "Cone Top Diameter", |ui| {
                diameter(ui, &mut preset.tip_radius)
            });
            row(ui, "Contact Depth", |ui| mm(ui, &mut preset.contact_depth));
            row(ui, "Cone Length", |ui| mm(ui, &mut preset.tip_length));
        });
    });

    ui.collapsing("Middle", |ui| {
        grid("support_middle").show(ui, |ui| {
            row(ui, "Pillar Diameter", |ui| diameter(ui, &mut preset.radius));
            row(ui, "Model Clearance", |ui| mm(ui, &mut preset.margin));
            row(ui, "Slim Diameter", |ui| {
                diameter(ui, &mut preset.slim_radius)
            });
            row(ui, "Slim Contact Depth", |ui| {
                mm(ui, &mut preset.slim_depth)
            });
        });
    });

    ui.collapsing("Bottom", |ui| {
        grid("support_bottom").show(ui, |ui| {
            row(ui, "Base Shape", |ui| {
                ComboBox::from_id_salt("base_shape")
                    .selected_text(format!("{:?}", preset.base_shape))
                    .show_ui(ui, |ui| {
                        for shape in [
                            BaseShape::Skate,
                            BaseShape::Cone,
                            BaseShape::Cylinder,
                            BaseShape::None,
                        ] {
                            ui.selectable_value(
                                &mut preset.base_shape,
                                shape,
                                format!("{shape:?}"),
                            );
                        }
                    });
            });
            row(ui, "Base Diameter", |ui| {
                diameter(ui, &mut preset.base_radius)
            });
            row(ui, "Base Thickness", |ui| {
                mm(ui, &mut preset.base_thickness)
            });
            row(ui, "Boss Height", |ui| mm(ui, &mut preset.boss_height));
            row(ui, "Boss Bottom Diameter", |ui| {
                diameter(ui, &mut preset.boss_radius)
            });
            row(ui, "Tail Depth", |ui| mm(ui, &mut preset.tail_depth));
        });
    });
}

fn row(ui: &mut Ui, label: &str, content: impl FnOnce(&mut Ui)) {
    ui.label(label);
    ui.horizontal(|ui| {
        content(ui);
        ui.take_available_width();
    });
    ui.end_row();
}

fn mm(ui: &mut Ui, value: &mut Milimeters) {
    value.with::<Milimeter, _>(|x| {
        DragValue::new(x)
            .suffix(" mm")
            .speed(0.01)
            .range(0.0..=f32::MAX)
            .ui(ui);
    });
}

fn diameter(ui: &mut Ui, radius: &mut Milimeters) {
    let mut d = radius.get::<Milimeter>() * 2.0;
    let edit = DragValue::new(&mut d)
        .prefix("Ø ")
        .suffix(" mm")
        .speed(0.01);
    if edit.range(0.0..=f32::MAX).ui(ui).changed() {
        *radius = Milimeters::new(d / 2.0);
    }
}

/// The models the Auto and Clear buttons act on: the selected ones (or the
/// models of the selected supports), else the only model. `None` when there
/// is no selection and several models to choose from.
fn targets(app: &App) -> Option<Vec<usize>> {
    let models = &app.project.models;
    let selected: Vec<usize> = (0..models.len())
        .filter(|&i| app.state.selected.contains_model(models[i].id))
        .collect();
    match (selected.is_empty(), models.len()) {
        (false, _) => Some(selected),
        (true, 1) => Some(vec![0]),
        (true, _) => None,
    }
}

/// A button running `action` on the target models, or a menu to pick a model
/// (or all) when there is no selection to go by.
fn model_action(app: &mut App, ui: &mut Ui, label: &str, action: fn(&mut App, usize)) {
    let models = app.project.models.len();
    match targets(app) {
        Some(targets) => {
            let hint = match targets.len() {
                1 => format!("For `{}`", app.project.models[targets[0]].name),
                n => format!("For the {n} selected models"),
            };
            if ui
                .add_enabled(models > 0, Button::new(label))
                .on_hover_text(hint)
                .clicked()
            {
                targets.into_iter().for_each(|idx| action(app, idx));
            }
        }
        None => {
            ui.menu_button(label, |ui| {
                ui.set_width(200.0);
                if ui.button("All Models").clicked() {
                    (0..models).for_each(|idx| action(app, idx));
                }
                ui.separator();
                for idx in 0..models {
                    if ui.button(&app.project.models[idx].name).clicked() {
                        action(app, idx);
                    }
                }
            });
        }
    }
}

/// Removes every support of a model, as one undo step.
fn clear_supports(app: &mut App, model: usize) {
    let model = &mut app.project.models[model];
    if model.supports.is_empty() {
        return;
    }
    (app.history).track_model(model.id, ModelAction::Supports(model.supports.snapshot()));
    model.supports.clear();
}

fn generate_support(app: &mut App, model: usize) {
    let default_lift = app.project.support_config.default_lift.get::<Milimeter>();

    let model = &mut app.project.models[model];
    (app.history).track_model(
        model.id,
        ModelAction::GeneratedSupports(model.mesh.position(), model.supports.snapshot()),
    );

    // Lift so the model's *lowest point* sits at `default_lift`, mirroring
    // align-to-bed (which puts the lowest point at z = 0) then adding the lift.
    // Using the object origin (`mesh.position().z`) instead is wrong: the origin
    // is the encoded translation, not the surface, so a model whose origin is
    // already above the lift would be left sitting below the plate.
    let min_z = model.bounds.min.z;
    let pos = model.mesh.position() + Vector3::z() * (default_lift - min_z);
    model.set_position(&app.project.slice_config.platform_size, pos);

    let placement = app.project.support_config.auto_placement.clone();
    let support = app.project.support_config.preset.clone();
    let platform = app.project.slice_config.platform_size.map(|x| x.convert());

    let task = GenerateSupports::new(platform, placement, support, model);
    app.tasks.add(task);
}

/// Deletes the selected supports, one undo step per model.
pub fn delete_selected(app: &mut App) {
    let mut by_model = HashMap::<ModelId, HashSet<SupportId>>::new();
    for selected in app.state.selected_supports.iter() {
        by_model
            .entry(selected.model)
            .or_default()
            .insert(selected.support);
    }
    app.state.selected_supports.clear();

    for (id, supports) in by_model {
        if let Some(model) = app.project.models.iter_mut().find(|x| x.id == id) {
            (app.history).track_model(id, ModelAction::Supports(model.supports.snapshot()));
            model.supports.remove(&supports);
        }
    }
}

/// Routes a support from where the pointer meets a model, considering only
/// `only` if given. Returns the model and the route.
fn route_under_pointer(
    app: &App,
    config: &SupportPreset,
    only: Option<ModelId>,
) -> Option<(ModelId, Routed)> {
    let (pos, dir) = app.hovered_ray()?;

    let mut nearest = None;
    for model in (app.project.models.iter()).filter(|m| only.is_none_or(|id| m.id == id)) {
        let Some(bvh) = model.bvh.as_ref() else {
            continue;
        };
        let Some(hit) = bvh.intersect_ray(&model.mesh, pos, dir) else {
            continue;
        };
        let point = model.mesh.transform(&hit.position);
        let distance = (point - pos).magnitude();
        if nearest
            .as_ref()
            .is_some_and(|&(d, ..): &(f32, _, _, _)| d <= distance)
        {
            continue;
        }
        nearest = Some((distance, model, hit.face, point));
    }

    let (_, model, face, point) = nearest?;
    let normal = (model.mesh)
        .transform_normal(&model.mesh.normal(face))
        .normalize();
    // Only faces that look down need holding up.
    if normal.z > 0.0 {
        return None;
    }
    let bvh = model.bvh.as_ref()?;
    let routed = route_point(&model.mesh, bvh, config, point, normal)?;
    Some((model.id, routed))
}

/// Previews the support the pointer would add and adds it on click. Only
/// selected models take supports; over an existing support nothing is added,
/// as the click selects it instead.
pub fn manual_support_placement(app: &mut App, clicked: bool) {
    app.state.support_preview = None;
    let Some(hover) = app.state.hovered_geometry else {
        return;
    };
    if app.state.workspace.is_moving
        || hover.support
        || !app.state.selected.contains_model(hover.model)
    {
        return;
    }

    let config = &app.project.support_config.preset;
    let Some((id, routed)) = route_under_pointer(app, config, Some(hover.model)) else {
        return;
    };
    let config = &app.project.support_config.preset;
    let Some(model) = app.project.models.iter_mut().find(|x| x.id == id) else {
        return;
    };
    app.state.support_preview = model.supports.preview_manual(config, &routed);
    if clicked {
        (app.history).track_model(id, ModelAction::Supports(model.supports.snapshot()));
        if let Some(bvh) = model.bvh.clone() {
            model
                .supports
                .add_manual(config, &routed, &model.mesh, &bvh);
        }
    }
}

/// Starts dragging the support under the pointer, if there is one.
pub fn start_support_drag(app: &mut App, hover: Option<GeometryHit>) -> bool {
    let selected = &app.state.selected;
    let Some(hover) = hover.filter(|hit| hit.support && selected.contains_model(hit.model)) else {
        return false;
    };
    let Some(model) = app.project.model(hover.model) else {
        return false;
    };
    let Some(support) = model.supports.support_for_face(hover.face) else {
        return false;
    };
    app.state.support_drag = Some(SupportDrag {
        model: hover.model,
        support,
    });
    true
}

/// Moves the dragged support's contact to where the pointer meets its model,
/// previewing the new route with the support's own settings; `released`
/// commits it.
pub fn drag_support(app: &mut App, released: bool) {
    let Some(drag) = app.state.support_drag else {
        return;
    };
    let Some(model) = app.project.models.iter_mut().find(|x| x.id == drag.model) else {
        app.state.support_drag = None;
        return;
    };
    let Some(config) = model.supports.get(drag.support).map(|s| s.preset) else {
        app.state.support_drag = None;
        return;
    };
    let config = &config;
    let routed = route_under_pointer(app, config, Some(drag.model));
    let model = app
        .project
        .models
        .iter_mut()
        .find(|x| x.id == drag.model)
        .unwrap();

    app.state.support_preview = None;
    if let Some((_, routed)) = routed {
        app.state.support_preview = model.supports.preview_manual(config, &routed);
        if released && let Some(bvh) = model.bvh.clone() {
            (app.history).track_model(drag.model, ModelAction::Supports(model.supports.snapshot()));
            (model.supports).replace(drag.support, config, &routed, &model.mesh, &bvh);
            // The shown mesh still has the support where it was until the
            // background build is in; keep showing the preview instead.
            app.state.support_settling = Some(drag);
        }
    }
    if released {
        app.state.support_drag = None;
        if app.state.support_settling.is_none() {
            app.state.support_preview = None;
        }
    }
}

fn prepare(app: &mut App, ui: &mut Ui) {
    if app.config.ui.auto_prepare {
        return;
    }
    let active = (app.project.models.iter()).any(|model| !model.has_acceleration_structures());
    let button = Button::new(concatcp!(LINE_SEGMENT, " Prepare All"));
    if ui.add_enabled(active, button).clicked() {
        for model in app.project.models.iter() {
            if let Some(task) = BuildAccelerationStructures::request(&mut app.state, model) {
                app.tasks.add(task);
            }
        }
    }
}

// NO DRUGS
// remember that.
