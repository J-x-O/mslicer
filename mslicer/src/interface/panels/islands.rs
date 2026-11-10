use const_format::concatcp;
use egui::{Button, Color32, ComboBox, Context, DragValue, RichText, ScrollArea, Ui, Widget};
use egui_phosphor::regular::{
    CARET_LEFT, CARET_RIGHT, CHECK, CROSSHAIR, EYE_SLASH, PLAY, PLUS, STOP, WARNING,
};
use tools::islands::Cover;

use crate::{
    core::{
        App,
        islands::{self, Accuracy, IslandRef},
        project::model::ModelId,
    },
    interface::{components::grid, panels::supports::targets},
};

const ABOUT: &str = "Finds where a model starts in mid-air: the first layer of a part with nothing under it, like a hanging tip or a detached detail. While the check runs, the islands no support holds show in the viewport. Step through them with the arrows (or N / Shift+N over the viewport), and add a support with +, or place one yourself in the support tool: the island under the pointer lights up.";
const ACCURACY_HELP: &str = "How far apart the checked layers are: every layer, every second or every fourth. Coarser is faster, but can report an island up to a few layers too high and miss parts thinner than the spacing.";
const IGNORE_HELP: &str = "Leave this island out until the next detection.";

pub fn ui(app: &mut App, ui: &mut Ui, _ctx: &Context) {
    if app.state.island_check.is_none() {
        idle(app, ui);
    } else {
        running(app, ui);
    }
}

fn idle(app: &mut App, ui: &mut Ui) {
    ui.label(ABOUT);
    ui.add_space(8.0);
    settings(app, ui);
    ui.add_space(8.0);
    start_button(app, ui);
}

fn settings(app: &mut App, ui: &mut Ui) {
    let settings = &mut app.state.island_settings;
    grid("islands").show(ui, |ui| {
        ui.label("Accuracy").on_hover_text(ACCURACY_HELP);
        ComboBox::from_id_salt("island_accuracy")
            .selected_text(settings.accuracy.name())
            .show_ui(ui, |ui| {
                for accuracy in Accuracy::ALL {
                    ui.selectable_value(&mut settings.accuracy, accuracy, accuracy.name());
                }
            });
        ui.end_row();

        ui.label("Minimum Area");
        DragValue::new(&mut settings.min_area)
            .suffix(" mm²")
            .speed(0.01)
            .range(0.0..=100.0)
            .ui(ui);
        ui.end_row();
    });
}

/// Starts a check of the selected models (or the only one), or offers a menu
/// to pick.
fn start_button(app: &mut App, ui: &mut Ui) {
    const LABEL: &str = concatcp!(PLAY, " Start Check");
    let ids = |app: &App, idx: &[usize]| -> Vec<ModelId> {
        idx.iter().map(|&i| app.project.models[i].id).collect()
    };
    match targets(app) {
        Some(targets) => {
            let hint = match targets.len() {
                1 => format!("For `{}`", app.project.models[targets[0]].name),
                n => format!("For the {n} selected models"),
            };
            let enabled = !app.project.models.is_empty();
            if ui
                .add_enabled(enabled, Button::new(LABEL))
                .on_hover_text(hint)
                .clicked()
            {
                let models = ids(app, &targets);
                islands::start(app, models);
            }
        }
        None => {
            ui.menu_button(LABEL, |ui| {
                ui.set_width(200.0);
                if ui.button("All Models").clicked() {
                    let all: Vec<usize> = (0..app.project.models.len()).collect();
                    let models = ids(app, &all);
                    islands::start(app, models);
                }
                ui.separator();
                for idx in 0..app.project.models.len() {
                    if ui.button(&app.project.models[idx].name).clicked() {
                        let models = ids(app, &[idx]);
                        islands::start(app, models);
                    }
                }
            });
        }
    }
}

fn running(app: &mut App, ui: &mut Ui) {
    settings(app, ui);
    ui.add_space(8.0);
    strip(app, ui);
    ui.add_space(4.0);

    let settings = app.state.island_settings;
    let Some(check) = app.state.island_check.as_ref() else {
        return;
    };
    let left = check.walk(&settings).len();
    ui.horizontal_wrapped(|ui| {
        let fix_all = Button::new(concatcp!(PLUS, " Fix All"));
        if ui
            .add_enabled(left > 0, fix_all)
            .on_hover_text("Add a support to every unsupported island")
            .clicked()
        {
            islands::support_all(app);
        }
        if ui
            .button(concatcp!(STOP, " Done"))
            .on_hover_text("End the check")
            .clicked()
        {
            islands::end(app);
        }
    });
    let Some(check) = &app.state.island_check else {
        return;
    };

    if let Some(at) = check.unroutable {
        let name = model_name(app, at.model);
        ui.label(format!(
            "{WARNING} No support could be routed to an island on `{name}`. Try placing one by hand in the support tool."
        ));
    }
    for (model, planes) in check.open_planes() {
        let name = model_name(app, model);
        ui.label(format!(
            "{WARNING} `{name}` has holes in its mesh: {planes} layers didn't close, so islands there may be missing."
        ));
    }

    ui.add_space(4.0);
    ui.separator();
    list(app, ui);
}

/// The navigator: how many islands are left, and stepping through them.
fn strip(app: &mut App, ui: &mut Ui) {
    enum Action {
        Step(bool),
        Support(IslandRef),
        Ignore(IslandRef),
    }

    let settings = app.state.island_settings;
    let Some(check) = app.state.island_check.as_ref() else {
        return;
    };
    let warning = app
        .config
        .ui
        .warning_color
        .map(|x| (x * 255.0).round() as u8);
    let warning = Color32::from_rgb(warning[0], warning[1], warning[2]);
    let left = check.walk(&settings).len();
    let any = !check.listed(&settings).is_empty();
    let current = check.current(&settings);
    let busy = check.busy();

    let mut action = None;
    ui.horizontal(|ui| {
        if busy && left == 0 {
            ui.spinner();
            ui.label("Detecting islands…");
            return;
        }
        if left == 0 {
            let text = if any {
                "All islands supported"
            } else {
                "No islands found"
            };
            ui.label(format!("{CHECK} {text}"));
            return;
        }

        match current {
            None => {
                let s = if left == 1 { "" } else { "s" };
                ui.label(RichText::new(format!("{left} unsupported island{s}")).color(warning));
                if ui.button(concatcp!(CROSSHAIR, " Show")).clicked() {
                    action = Some(Action::Step(true));
                }
            }
            Some((k, len, at)) => {
                if ui
                    .button(CARET_LEFT)
                    .on_hover_text("Previous (Shift+N)")
                    .clicked()
                {
                    action = Some(Action::Step(false));
                }
                let text = format!("{} / {len} unsupported", k + 1);
                ui.label(RichText::new(text).color(warning));
                if ui
                    .button(PLUS)
                    .on_hover_text("Support this island")
                    .clicked()
                {
                    action = Some(Action::Support(at));
                }
                if ui.button(CARET_RIGHT).on_hover_text("Next (N)").clicked() {
                    action = Some(Action::Step(true));
                }
                if ui.button(EYE_SLASH).on_hover_text(IGNORE_HELP).clicked() {
                    action = Some(Action::Ignore(at));
                }
            }
        }
        if busy {
            ui.spinner();
        }
    });

    match action {
        Some(Action::Step(forward)) => islands::step(app, forward),
        Some(Action::Support(at)) => {
            islands::support(app, at);
        }
        Some(Action::Ignore(at)) => islands::ignore(app, at),
        None => {}
    }
}

/// Every island, bottom up per model.
fn list(app: &mut App, ui: &mut Ui) {
    let settings = app.state.island_settings;
    let Some(check) = app.state.island_check.as_ref() else {
        return;
    };
    let current = check.current(&settings).map(|(.., at)| at);
    let several = check.models.len() > 1;
    let rows: Vec<(IslandRef, String, Cover, bool)> = (check.listed(&settings).into_iter())
        .map(|l| {
            let mut text = format!("z {:.2} mm · {:.2} mm²", l.island.z, l.island.area);
            if several {
                text += &format!(" · {}", model_name(app, l.at.model));
            }
            (l.at, text, l.cover, l.ignored)
        })
        .collect();

    let mut action = None;
    ScrollArea::vertical()
        .auto_shrink([false, true])
        .show(ui, |ui| {
            for (at, text, cover, ignored) in rows {
                ui.horizontal(|ui| {
                    let (icon, hint) = match (cover, ignored) {
                        (_, true) => (EYE_SLASH, "Ignored"),
                        (Cover::Unsupported, _) => (WARNING, "Unsupported"),
                        _ => (CHECK, "Supported"),
                    };
                    ui.label(icon).on_hover_text(hint);
                    if ui.selectable_label(current == Some(at), text).clicked() {
                        action = Some((at, false));
                    }
                    if cover == Cover::Unsupported
                        && ui
                            .small_button(PLUS)
                            .on_hover_text("Support this island")
                            .clicked()
                    {
                        action = Some((at, true));
                    }
                });
            }
        });

    match action {
        Some((at, true)) => {
            islands::support(app, at);
        }
        Some((at, false)) => islands::show(app, at),
        None => {}
    }
}

fn model_name(app: &App, id: ModelId) -> String {
    (app.project.models.iter())
        .find(|m| m.id == id)
        .map_or_else(String::new, |m| m.name.clone())
}
