use const_format::concatcp;
use egui::{Button, ComboBox, Context, DragValue, Response, Ui, Widget};
use egui_phosphor::regular::{DROP, MAGIC_WAND, SNOWFLAKE, TRASH, WARNING, X};

use crate::{
    core::{
        App,
        hollow::{self, HoleRef, edit},
        project::{model::ModelId, stack::Op},
        stack,
        state::Tool,
    },
    interface::{components::grid, panels::supports::targets},
};

const ABOUT: &str = "Hollows a model to save resin: only a wall of the set thickness is kept, and drain holes let the resin inside run out. The model itself isn't changed: slicing, export and supports use a copy with the cavity and the holes worked in.";
const FAILED_HELP: &str = "This hole couldn't be drilled here, so it is left out and drains nothing. Remove it and try a spot where the wall is plain and the cavity behind it is wide enough.";
const WALL_HELP: &str = "How thick the kept wall is. Thinner walls save more resin but are weaker.";
const DETAIL_HELP: &str =
    "How finely the cavity follows the model. Finer takes longer on large models.";
const AUTO_HELP: &str = "Drills a hole straight up into the lowest point of each void, for the way the model is oriented now. Turning the model moves these holes with the resin line; holes you placed yourself stay where they are.";
const PLACE_HELP: &str = "In the hole tool: click a hollowed model to drill a hole straight in there, click a hole to select it (Shift adds), drag a hole to move it, Delete removes the selected ones.";

/// Sample spacings offered for the cavity (mm).
const DETAIL: [(&str, f32); 3] = [("Fine", 0.3), ("Normal", 0.5), ("Coarse", 0.8)];

pub fn ui(app: &mut App, ui: &mut Ui, _ctx: &Context) {
    ui.label(ABOUT);
    ui.add_space(8.0);

    let Some(id) = target(app) else {
        ui.label("Select a model to hollow it.");
        return;
    };
    let Some(model) = app.project.models.iter().find(|m| m.id == id) else {
        return;
    };
    let name = model.name.clone();
    ui.strong(format!("`{name}`"));
    ui.add_space(4.0);

    let Some(model) = app.project.models.iter().find(|m| m.id == id) else {
        return;
    };
    let frozen = stack::is_frozen(model, |op| matches!(op, Op::Hollow(_)));
    if frozen {
        ui.label(concatcp!(
            SNOWFLAKE,
            " The Hollow operation is frozen: thaw it in the model's operations to edit it."
        ));
    }
    ui.add_enabled_ui(!frozen, |ui| {
        shell(app, ui, id);
        // Holes drain a cavity: only a hollowed model has them.
        let hollowed = (app.project.models.iter()).any(|m| m.id == id && m.is_hollowed());
        if !hollowed {
            return;
        }
        ui.add_space(8.0);
        ui.separator();
        holes(app, ui, id);
    });
}

/// The model being worked on: the selected one, or the only one.
fn target(app: &App) -> Option<ModelId> {
    match targets(app)?.as_slice() {
        [index] => Some(app.project.models[*index].id),
        _ => None,
    }
}

fn shell(app: &mut App, ui: &mut Ui, id: ModelId) {
    let Some(model) = app.project.models.iter().find(|m| m.id == id) else {
        return;
    };
    let spec = model.hollow.spec.clone();

    // The checkbox adds the Hollow operation, or bypasses it (its holes are
    // kept for when it is back).
    let mut hollow = model.is_hollowed();
    if ui.checkbox(&mut hollow, "Hollow").changed() {
        stack::set_hollowed(stack::of(app), id, hollow);
    }
    let Some(mut shell) = spec.shell.filter(|_| hollow) else {
        return;
    };

    let mut tracked = false;
    grid("hollow").show(ui, |ui| {
        ui.label("Wall Thickness").on_hover_text(WALL_HELP);
        let response = DragValue::new(&mut shell.wall)
            .suffix(" mm")
            .speed(0.05)
            .range(0.5..=10.0)
            .ui(ui);
        tracked |= starts_edit(&response);
        ui.end_row();

        ui.label("Detail").on_hover_text(DETAIL_HELP);
        let name = (DETAIL.iter())
            .find(|(_, cell)| *cell == shell.cell)
            .map_or("Custom", |(name, _)| name);
        ComboBox::from_id_salt("hollow_detail")
            .selected_text(name)
            .show_ui(ui, |ui| {
                for (name, cell) in DETAIL {
                    if ui.selectable_value(&mut shell.cell, cell, name).changed() {
                        tracked = true;
                    }
                }
            });
        ui.end_row();
    });
    if Some(shell) != spec.shell {
        edit(app, id, tracked, |spec| spec.shell = Some(shell));
    }

    let Some(model) = app.project.models.iter().find(|m| m.id == id) else {
        return;
    };
    let hollow = &model.hollow;
    match hollow.cavity(&model.mesh) {
        None => {
            ui.horizontal(|ui| {
                ui.spinner();
                ui.label("Finding the cavity…");
            });
        }
        Some(cavity) if cavity.parts.is_empty() => {
            ui.label("No room for a cavity: the model is nowhere thicker than two walls.");
        }
        Some(cavity) => {
            let voids = match cavity.parts.len() {
                1 => String::new(),
                n => format!(" in {n} voids"),
            };
            ui.label(format!(
                "Saves about {:.1} ml of resin{voids}.",
                cavity.volume() / 1000.0
            ));
        }
    }
}

fn holes(app: &mut App, ui: &mut Ui, id: ModelId) {
    ui.strong("Drain Holes");
    let Some(model) = app.project.models.iter().find(|m| m.id == id) else {
        return;
    };
    let drains = (model.hollow.cavity(&model.mesh)).is_some_and(|c| !c.drains.is_empty());
    let sealed = model.hollow.sealed_parts(&model.mesh);

    let settings = &mut app.state.hole_settings;
    grid("holes").show(ui, |ui| {
        ui.label("Diameter");
        DragValue::new(&mut settings.diameter)
            .suffix(" mm")
            .speed(0.05)
            .range(0.5..=10.0)
            .ui(ui);
        ui.end_row();
    });

    // The auto-hole rule: one hole at each void's lowest point, wherever
    // that is in the current orientation.
    let Some(model) = app.project.models.iter().find(|m| m.id == id) else {
        return;
    };
    let rule = model.stack.hollow().map(|(op, h)| (op.id, *h));
    let autos = model.hollow.auto_holes(&model.mesh).len();
    if let Some((op, mut rule)) = rule {
        let before = rule;
        ui.horizontal(|ui| {
            ui.checkbox(&mut rule.auto_holes, "Vent each void automatically")
                .on_hover_text(AUTO_HELP);
            if rule.auto_holes {
                DragValue::new(&mut rule.diameter)
                    .prefix("Ø ")
                    .suffix(" mm")
                    .speed(0.05)
                    .range(0.5..=10.0)
                    .ui(ui);
            }
        });
        if rule.auto_holes && autos > 0 {
            ui.weak(format!(
                "{autos} auto hole{} at the lowest points. Click one with the hole tool to pin it where it is.",
                if autos == 1 { "" } else { "s" }
            ));
        }
        if rule != before {
            stack::edit_merged(stack::of(app), id, "Changed Auto Holes", |model| {
                if let Some(operation) = model.stack.get_mut(op) {
                    operation.op = Op::Hollow(rule);
                }
            });
        }
    }

    ui.horizontal_wrapped(|ui| {
        let active = app.state.tool == Tool::Holes;
        if ui
            .add(Button::new(concatcp!(DROP, " Hole Tool")).selected(active))
            .on_hover_text(PLACE_HELP)
            .clicked()
        {
            app.state
                .set_tool(if active { Tool::Select } else { Tool::Holes });
        }
        if ui
            .add_enabled(
                drains,
                Button::new(concatcp!(MAGIC_WAND, " Add Drain Holes")),
            )
            .on_hover_text("A hole up into the lowest point of each void")
            .clicked()
        {
            hollow::add_drains(app, id);
        }
    });
    if app.state.tool == Tool::Holes {
        ui.label(PLACE_HELP);
    }
    if sealed > 0 {
        let text = match sealed {
            1 => "A void has no drain hole".to_owned(),
            n => format!("{n} voids have no drain hole"),
        };
        ui.label(format!(
            "{WARNING} {text}: the resin inside stays trapped and can burst the print."
        ));
    }

    let Some(model) = app.project.models.iter().find(|m| m.id == id) else {
        return;
    };
    let rows: Vec<(String, bool)> = (model.hollow.spec.holes.iter().enumerate())
        .map(|(index, h)| {
            let text = format!("Hole {}: Ø {:.1} mm", index + 1, h.diameter);
            let failed = model.hollow.failed(&model.mesh, index);
            (text, failed)
        })
        .collect();
    if rows.is_empty() {
        return;
    }
    ui.add_space(4.0);
    if model.hollow.wants_drilling(&model.mesh) {
        ui.horizontal(|ui| {
            ui.spinner();
            ui.label("Drilling the holes…");
        });
    }
    let (mut remove, mut focus, mut hovered) = (None, None, None);
    for (index, (text, failed)) in rows.into_iter().enumerate() {
        let shown = app
            .state
            .selected_holes
            .contains(&HoleRef { model: id, index });
        let row = ui.horizontal(|ui| {
            if ui
                .selectable_label(shown, text)
                .on_hover_text("Select this hole and show it")
                .clicked()
            {
                focus = Some(index);
            }
            if failed {
                ui.label(format!("{WARNING} didn't work, try another spot"))
                    .on_hover_text(FAILED_HELP);
            }
            if ui
                .small_button(X)
                .on_hover_text("Remove this hole")
                .clicked()
            {
                remove = Some(index);
            }
        });
        if row.response.contains_pointer() {
            hovered = Some(index);
        }
    }
    // The hole under the pointer stands out in the viewport.
    app.state.listed_hole = hovered.map(|index| HoleRef { model: id, index });
    if let Some(index) = focus {
        app.state.set_tool(Tool::Holes);
        app.state.selected_holes = vec![HoleRef { model: id, index }];
        hollow::focus(app, id, index);
    }
    if ui.button(concatcp!(TRASH, " Remove All")).clicked() {
        edit(app, id, true, |spec| spec.holes.clear());
    } else if let Some(index) = remove {
        edit(app, id, true, |spec| {
            spec.holes.remove(index);
        });
    }
}

/// Whether a value change should start a new undo step: the first step of a
/// drag, or a typed or clicked change.
fn starts_edit(response: &Response) -> bool {
    response.drag_started() || (response.changed() && !response.dragged())
}
