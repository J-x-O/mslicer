//! The scene tree: models, each expanding into its source and operations,
//! and the Print node with its profile, layer ops, overrides and outputs.
//! Picking a row shows it in Properties.

use egui::{
    Align, Button, Context, Id, Label, Layout, Response, RichText, ScrollArea, Sense, Stroke, Ui,
    Widget, vec2,
};
use egui_phosphor::regular::{CARET_DOWN, CARET_RIGHT, CUBE, PRINTER, WARNING};

use crate::{
    core::{
        App,
        focus::{self, Focus, PrintNode},
        issues::{self, Source},
        project::{
            model::ModelId,
            stack::{Mode, Op},
        },
    },
    interface::panels::models::{
        add_operation, add_operation_menu, apply_operation, model_entry, operation_buttons,
    },
};

const MODE_TIP: &str = "Auto: computed from its settings, recomputed when its inputs change.\nEdited: the auto result with your edits on top (placed supports, drilled holes).\nFrozen: kept as it is.";
const SHOW_UP_TO_TIP: &str = "Show the model as it is after this operation: the operations below it are left out of the viewport (not of slicing).";
const INEXACT_TIP: &str = "Can't stop here: a repair or flip after this operation is already part of the surface the viewport shows.";

pub fn ui(app: &mut App, ui: &mut Ui, _ctx: &Context) {
    let current = app.state.focus.get(&app.state.selected);
    ScrollArea::vertical().show(ui, |ui| {
        let mut n = 0;
        if app.project.models.is_empty() {
            ui.weak("No models yet: import one with File > Import or drop a file here.");
        }
        for i in 0..app.project.models.len() {
            model(app, ui, i, &mut n, current);
        }
        ui.separator();
        print(app, ui, current);

        let fill = ui.available_rect_before_wrap();
        let resp = ui.interact(fill, ui.id().with("deselect"), Sense::click());
        if resp.clicked() {
            app.state.selected.clear();
        }
    });
}

/// Whether the tree row under `id` is open, toggled by an arrow.
fn expander(ui: &mut Ui, id: Id, default: bool) -> bool {
    let mut open = ui.data_mut(|d| *d.get_temp_mut_or(id, default));
    let icon = if open { CARET_DOWN } else { CARET_RIGHT };
    if Button::new(icon).frame(false).small().ui(ui).clicked() {
        open ^= true;
        ui.data_mut(|d| d.insert_temp(id, open));
    }
    open
}

fn model(app: &mut App, ui: &mut Ui, index: usize, n: &mut usize, current: Focus) {
    let id = app.project.models[index].id;
    let open_id = Id::new("outliner_model").with(id);
    let mut open = false;
    ui.horizontal(|ui| {
        open = expander(ui, open_id, false);
        model_entry(app, ui, index, *n, false);
    });
    *n += 1;
    if !open {
        return;
    }

    ui.indent(open_id, |ui| {
        let source = ui
            .selectable_label(current == Focus::Source(id), format!("{CUBE} Source"))
            .on_hover_text("The mesh as imported. Operations never change it: they are applied on top, in order.");
        if source.clicked() {
            focus::focus(app, Focus::Source(id));
        }
        operations(app, ui, id, current);
    });
}

fn operations(app: &mut App, ui: &mut Ui, id: ModelId, current: Focus) {
    let Some(model) = app.project.models.iter().find(|m| m.id == id) else {
        return;
    };
    let issues = issues::collect(app);
    let ops = model.stack.ops();
    // The operation the viewport shows the model up to, as an index.
    let upto = (app.state.show_up_to)
        .filter(|(m, _)| *m == id)
        .and_then(|(_, op)| model.stack.index(op));
    let mut picked = None;
    let mut knob = None;
    let mut action = None;
    for (i, operation) in ops.iter().enumerate() {
        let op = operation.id;
        let flagged = (issues.iter()).any(|i| {
            i.source
                == Source::Model {
                    model: id,
                    operation: Some(op),
                }
        });
        ui.horizontal(|ui| {
            let point = Point {
                shown: upto.is_none_or(|at| i <= at),
                next_shown: i + 1 < ops.len() && upto.is_none_or(|at| i < at),
                last: i + 1 == ops.len(),
                chosen: upto == Some(i),
                exact: focus::show_up_to_exact(model, op),
            };
            if timeline_knob(ui, point).clicked() {
                knob = Some((i, op, point.exact));
            }
            let mut name = RichText::new(operation.op.name());
            if operation.bypassed {
                name = name.strikethrough().weak();
            }
            if ui
                .selectable_label(current == Focus::Operation(id, op), name)
                .clicked()
            {
                picked = Some(op);
            }
            let badge = match (&operation.op, operation.mode) {
                (Op::Orient(_), Mode::Edited) => "Manual",
                (_, mode) => mode.name(),
            };
            ui.add(Label::new(RichText::new(badge).small().weak()).selectable(false))
                .on_hover_text(MODE_TIP);
            if flagged {
                ui.label(RichText::new(WARNING).small())
                    .on_hover_text("This operation has issues: see Issues.");
            }
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                operation_buttons(ui, &model.stack, operation, true, &mut action);
            });
        });
    }
    let add = add_operation_menu(ui, &model.stack);
    let last = ops.len().saturating_sub(1);

    if let Some((i, op, exact)) = knob {
        // The chosen point or the end shows everything again.
        app.state.show_up_to = (exact && upto != Some(i) && i != last).then_some((id, op));
    }
    if let Some(op) = picked {
        focus::focus_operation(app, id, op);
    }
    apply_operation(app, id, action);
    if let Some(op) = add {
        let name = op.name();
        add_operation(app, id, op);
        // Show what was added.
        let Some(model) = app.project.models.iter().find(|m| m.id == id) else {
            return;
        };
        let added = (model.stack.ops().iter()).rfind(|o| o.op.name() == name);
        if let Some(added) = added.map(|o| o.id) {
            focus::focus_operation(app, id, added);
        }
    }
}

/// An operation's place on the stack's timeline.
#[derive(Clone, Copy)]
struct Point {
    /// The viewport shows this operation's result.
    shown: bool,
    /// And the next one's: the line down to it is lit.
    next_shown: bool,
    last: bool,
    /// The viewport shows the model up to here.
    chosen: bool,
    /// The viewport can show the model as it is after this operation.
    exact: bool,
}

/// The knob on the timeline line at the start of an operation row: click
/// it to show the model in the viewport as it is after the operation.
fn timeline_knob(ui: &mut Ui, point: Point) -> Response {
    let height = ui.spacing().interact_size.y;
    let sense = if point.exact {
        Sense::click()
    } else {
        Sense::hover()
    };
    let (rect, response) = ui.allocate_exact_size(vec2(14.0, height), sense);
    let visuals = ui.visuals();
    let lit = visuals.selection.bg_fill;
    let dim = visuals.weak_text_color();
    let center = rect.center();
    let spacing = ui.spacing().item_spacing.y;
    let painter = ui.painter();
    // The line through the rows, up to the previous knob and down to the
    // next one.
    let up = [center.x, rect.top() - spacing].into();
    let line = |shown: bool| Stroke::new(2.0_f32, if shown { lit } else { dim });
    painter.line_segment([up, center], line(point.shown));
    if !point.last {
        let down = [center.x, rect.bottom() + spacing].into();
        painter.line_segment([center, down], line(point.next_shown));
    }
    let hovered = response.hovered() && point.exact;
    let radius = if point.chosen || hovered { 5.5 } else { 4.0 };
    let fill = match (point.shown, point.exact) {
        (true, true) => lit,
        (true, false) => dim,
        (false, _) => visuals.extreme_bg_color,
    };
    painter.circle(center, radius, fill, line(point.shown));
    let tip = match (point.exact, point.chosen) {
        (false, _) => INEXACT_TIP,
        (true, true) => "Showing the model up to here. Click to show all of it again.",
        (true, false) => SHOW_UP_TO_TIP,
    };
    response.on_hover_text(tip)
}

fn print(app: &mut App, ui: &mut Ui, current: Focus) {
    let open_id = Id::new("outliner_print");
    let mut open = false;
    ui.horizontal(|ui| {
        open = expander(ui, open_id, true);
        ui.label(RichText::new(format!("{PRINTER} Print")).strong());
    });
    if !open {
        return;
    }
    ui.indent(open_id, |ui| {
        for node in PrintNode::ALL {
            let focus = Focus::Print(node);
            if ui.selectable_label(current == focus, node.name()).clicked() {
                app.state.selected.clear();
                focus::focus(app, focus);
            }
        }
    });
}
