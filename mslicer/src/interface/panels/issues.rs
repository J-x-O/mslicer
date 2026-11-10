use egui::{Context, RichText, Sense, Ui};
use egui_phosphor::regular::{CHECK_CIRCLE, WARNING, X_CIRCLE};

use crate::core::{
    App,
    issues::{self, Severity, Source},
};

const ABOUT: &str = "Everything that may go wrong with the print, kept up to date as you work. Click an issue to go to it.";

pub fn ui(app: &mut App, ui: &mut Ui, _ctx: &Context) {
    let mut list = issues::collect(app);
    list.sort_by_key(|i| i.severity);

    if list.is_empty() {
        ui.horizontal(|ui| {
            ui.label(
                RichText::new(CHECK_CIRCLE).color(ui.visuals().widgets.active.fg_stroke.color),
            );
            ui.label("No issues.");
        });
        ui.add_space(4.0);
        ui.weak(ABOUT);
        return;
    }

    let mut jump = None;
    let mut fix = None;
    for issue in &list {
        let (icon, color) = match issue.severity {
            Severity::Error => (X_CIRCLE, ui.visuals().error_fg_color),
            Severity::Warning => (WARNING, ui.visuals().warn_fg_color),
        };
        let source = match issue.source {
            Source::Print => "Print".to_owned(),
            Source::Model { model, operation } => {
                let model = app.project.models.iter().find(|m| m.id == model);
                let name = model.map_or("?".to_owned(), |m| m.name.clone());
                let op = operation
                    .and_then(|op| model?.stack.ops().iter().find(|o| o.id == op))
                    .map(|o| format!(" › {}", o.op.name()));
                format!("{name}{}", op.unwrap_or_default())
            }
        };
        let response = ui
            .horizontal_wrapped(|ui| {
                ui.label(RichText::new(icon).color(color));
                ui.strong(source);
                ui.label(&issue.text);
            })
            .response
            .interact(Sense::click())
            .on_hover_text("Click to go there.");
        if response.clicked() {
            jump = Some(issue.jump.clone());
        }
        if let Some(f) = issue.fix() {
            ui.horizontal(|ui| {
                ui.add_space(18.0);
                if ui.small_button(f.label()).on_hover_text(f.tip()).clicked() {
                    fix = Some(f);
                }
            });
        }
        ui.add_space(2.0);
    }
    if let Some(jump) = jump {
        issues::jump(app, &jump);
    }
    if let Some(fix) = fix {
        issues::apply(app, &fix);
    }
}
