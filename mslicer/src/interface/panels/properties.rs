//! Shows whatever is selected: the selected models (or collection), a
//! model's source or one of its operations, or a node of the print. The
//! old panels' contents, gathered by selection.

use egui::{CollapsingHeader, Context, RichText, Ui};
use egui_phosphor::regular::{CUBE, PRINTER};

use crate::{
    core::{
        App,
        focus::{self, Focus, PrintNode},
        project::{
            model::ModelId,
            stack::{Mode, Op, OperationId},
        },
        state::Tool,
    },
    interface::panels::{hollow, islands, models, remote_print, slice_config, sliced, supports},
};

pub fn ui(app: &mut App, ui: &mut Ui, ctx: &Context) {
    match app.state.focus.get(&app.state.selected) {
        Focus::Selection if app.state.selected.has_any() => models::selection(app, ui, ctx),
        Focus::Selection => {
            ui.weak("Select a model in the Outliner or the viewport, or a node of the Print.");
        }
        Focus::Source(id) => source(app, ui, id),
        Focus::Operation(id, op) => operation(app, ui, ctx, id, op),
        Focus::Print(node) => print(app, ui, ctx, node),
    }
}

fn source(app: &mut App, ui: &mut Ui, id: ModelId) {
    let Some(model) = app.project.models.iter().find(|m| m.id == id) else {
        return;
    };
    ui.heading(format!("{CUBE} {}", model.name));
    ui.label("Source: the mesh as imported. Operations never change it: they are applied on top, in order.");
    ui.add_space(4.0);
    let mesh = &model.mesh;
    ui.label(format!(
        "{} vertices, {} triangles",
        mesh.vertex_count(),
        mesh.face_count()
    ));
}

fn operation(app: &mut App, ui: &mut Ui, ctx: &Context, id: ModelId, op: OperationId) {
    let Some(model) = app.project.models.iter().find(|m| m.id == id) else {
        return;
    };
    let Some(operation) = model.stack.get(op) else {
        return;
    };
    let kind = operation.op.clone();
    let frozen = operation.mode == Mode::Frozen;
    ui.label(RichText::new(&model.name).weak());
    // Bypass, freeze, remove and "show up to here" are on the operation's
    // row in the Outliner.
    let badge = match (&kind, operation.mode) {
        (Op::Orient(_), Mode::Edited) => "Manual",
        (_, mode) => mode.name(),
    };
    ui.horizontal(|ui| {
        ui.heading(kind.name());
        ui.weak(badge);
    });
    ui.separator();

    match kind {
        Op::Repair(_) => {
            ui.label("Repairs the mesh: welds vertices, removes bad faces, closes holes, fixes the winding.");
            let model = app.project.models.iter().find(|m| m.id == id);
            if let Some(stats) = model.and_then(|model| model.repair_stats(op)) {
                for (name, n) in stats.rows() {
                    ui.label(format!("{name}: {n}"));
                }
            }
        }
        Op::FlipWinding => {
            ui.label("Turns every face inside out.");
        }
        Op::Orient(_) => {
            ui.label("The orientation: the rotation of the model, set in the model's properties, or by laying a face on the bed with the Orient tool.");
            tool_button(app, ui, Tool::Orient, "Lay a face on the bed");
            ui.add_space(4.0);
            CollapsingHeader::new("Model")
                .default_open(true)
                .show(ui, |ui| models::selection(app, ui, ctx));
        }
        Op::Hollow(_) => {
            hollow::ui(app, ui, ctx);
        }
        Op::Supports(_) => {
            if frozen {
                ui.label("The Supports operation is frozen: thaw it to edit.");
            }
            supports::ui(app, ui, ctx);
            ui.add_space(8.0);
            CollapsingHeader::new("Island check")
                .default_open(false)
                .show(ui, |ui| islands::ui(app, ui, ctx));
        }
    }
}

fn tool_button(app: &mut App, ui: &mut Ui, tool: Tool, label: &str) {
    let active = app.state.tool == tool;
    if ui
        .selectable_label(active, format!("{} {label}", tool.icon()))
        .clicked()
    {
        focus::pick_tool(app, if active { Tool::Select } else { tool });
    }
}

fn print(app: &mut App, ui: &mut Ui, ctx: &Context, node: PrintNode) {
    ui.heading(format!("{PRINTER} Print: {}", node.name()));
    ui.add_space(4.0);
    match node {
        PrintNode::Profile => slice_config::profile(app, ui),
        PrintNode::LayerOps => slice_config::layer_ops(app, ui),
        PrintNode::Overrides => {
            ui.label("Exposure for layer ranges. The first layers' exposure is set in the Profile; other ranges can't be overridden yet.");
        }
        PrintNode::Outputs => {
            sliced::ui(app, ui, ctx);
            ui.add_space(8.0);
            CollapsingHeader::new("Remote print")
                .default_open(false)
                .show(ui, |ui| remote_print::ui(app, ui, ctx));
        }
    }
}
