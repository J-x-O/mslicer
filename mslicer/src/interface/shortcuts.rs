use std::ops::Deref;

use common::units::Milimeter;
use egui::{Context, Key, KeyboardShortcut, Modifiers, ViewportCommand};

use crate::{
    core::{App, project::Project, state::Tool},
    include_asset,
    interface::{panels::supports::delete_selected, popup::confirm_unsaved},
    task::{AutoLayout, FileDialog, LoadSliced, MeshLoad, MultiFileDialog},
};

const COMMAND_SHIFT: Modifiers = Modifiers::COMMAND.plus(Modifiers::SHIFT);

pub const IMPORT_MODEL_SHORTCUT: Shortcut = Shortcut::new(Key::I, import_model);
pub const IMPORT_SLICED_SHORTCUT: Shortcut = Shortcut::new(Key::I, load_sliced).command_shift();
pub const LOAD_TEAPOT_SHORTCUT: Shortcut = Shortcut::new(Key::T, import_teapot);
pub const SAVE_PROJECT_SHORTCUT: Shortcut = Shortcut::new(Key::S, save);
pub const SAVE_AS_PROJECT_SHORTCUT: Shortcut = Shortcut::new(Key::S, save_as).command_shift();
pub const NEW_PROJECT_SHORTCUT: Shortcut = Shortcut::new(Key::N, new);
pub const LOAD_PROJECT_SHORTCUT: Shortcut = Shortcut::new(Key::O, load);
pub const QUIT_SHORTCUT: Shortcut = Shortcut::new(Key::Q, quit);
pub const SLICE_SHORTCUT: Shortcut = Shortcut::new(Key::R, slice);
pub const UNDO_SHORTCUT: Shortcut = Shortcut::new(Key::Z, undo);
pub const REDO_SHORTCUT: Shortcut = Shortcut::new(Key::Y, redo);
pub const LAYOUT_SHORTCUT: Shortcut = Shortcut::new(Key::L, quick_layout);
pub const SELECT_ALL_SHORTCUT: Shortcut = Shortcut::new(Key::A, select_all).input_exclusive();
pub const SELECT_NONE_SHORTCUT: Shortcut = Shortcut::new(Key::D, select_none).command_shift();
pub const DELETE_SUPPORTS_SHORTCUT: Shortcut = Shortcut::new(Key::Delete, delete_supports)
    .modifier(Modifiers::NONE)
    .input_exclusive();
pub const BACKSPACE_SUPPORTS_SHORTCUT: Shortcut = Shortcut::new(Key::Backspace, delete_supports)
    .modifier(Modifiers::NONE)
    .input_exclusive();

const SHORTCUTS: &[Shortcut] = &[
    IMPORT_SLICED_SHORTCUT,
    IMPORT_MODEL_SHORTCUT,
    LOAD_TEAPOT_SHORTCUT,
    NEW_PROJECT_SHORTCUT,
    LOAD_PROJECT_SHORTCUT,
    SAVE_PROJECT_SHORTCUT,
    SAVE_AS_PROJECT_SHORTCUT,
    QUIT_SHORTCUT,
    UNDO_SHORTCUT,
    REDO_SHORTCUT,
    SLICE_SHORTCUT,
    LAYOUT_SHORTCUT,
    SELECT_ALL_SHORTCUT,
    SELECT_NONE_SHORTCUT,
    DELETE_SUPPORTS_SHORTCUT,
    BACKSPACE_SUPPORTS_SHORTCUT,
];

type ShortcutCallback = fn(&mut App, &Context);

#[derive(Clone, Copy)]
pub struct Shortcut {
    keys: KeyboardShortcut,
    pub(crate) callback: ShortcutCallback,
    input_exclusive: bool,
}

impl Shortcut {
    const fn new(key: Key, callback: ShortcutCallback) -> Self {
        Self {
            keys: KeyboardShortcut::new(Modifiers::COMMAND, key),
            callback,
            input_exclusive: false,
        }
    }

    const fn modifier(self, modifiers: Modifiers) -> Self {
        Self {
            keys: KeyboardShortcut::new(modifiers, self.keys.logical_key),
            ..self
        }
    }

    const fn command_shift(self) -> Self {
        self.modifier(COMMAND_SHIFT)
    }

    const fn input_exclusive(self) -> Self {
        Self {
            input_exclusive: true,
            ..self
        }
    }
}

impl Deref for Shortcut {
    type Target = KeyboardShortcut;

    fn deref(&self) -> &Self::Target {
        &self.keys
    }
}

pub fn handle(app: &mut App, ctx: &Context) {
    let text_field_focused = ctx.wants_keyboard_input();
    for shortcut in SHORTCUTS {
        if shortcut.input_exclusive && text_field_focused {
            continue;
        }

        if ctx.input_mut(|x| x.consume_shortcut(shortcut)) {
            (shortcut.callback)(app, ctx);
            break;
        }
    }
}

fn import_model(app: &mut App, _ctx: &Context) {
    app.tasks.add(MultiFileDialog::pick_files(
        ("Mesh", &["stl", "obj"]),
        |_app, paths, tasks| {
            for path in paths {
                let name = path.file_name().unwrap().to_str().unwrap().to_string();
                let ext = path.extension();
                let format = ext.unwrap_or_default().to_string_lossy();

                let task = MeshLoad::file(path.to_path_buf(), name, format.into()).unwrap();
                tasks.push(Box::new(task));
            }
        },
    ));
}

fn load_sliced(app: &mut App, _ctx: &Context) {
    app.tasks.add(FileDialog::pick_file(
        ("Sliced Model", &["goo", "ctb", "nanodlp"]),
        |_app, file, tasks| tasks.push(Box::new(LoadSliced::new(file.to_path_buf()))),
    ));
}

fn import_teapot(app: &mut App, _ctx: &Context) {
    app.tasks.add(MeshLoad::buffer(
        include_asset!("teapot.stl"),
        "Utah Teapot".into(),
        "stl".into(),
    ));
}

fn new(app: &mut App, _ctx: &Context) {
    confirm_unsaved(app, |app| {
        app.project.reset(&app.config.default_slice_config);
        app.history.clear();
        Vec::new()
    });
}

fn save(app: &mut App, _ctx: &Context) {
    let task = app.project.save(|_| Vec::new());
    app.tasks().add_boxed(task);
}

fn save_as(app: &mut App, _ctx: &Context) {
    let task = app.project.save_as(|_| Vec::new());
    app.tasks().add(task);
}

fn load(app: &mut App, _ctx: &Context) {
    confirm_unsaved(app, |_app| vec![Box::new(Project::load())]);
}

fn quit(_app: &mut App, ctx: &Context) {
    ctx.send_viewport_cmd(ViewportCommand::Close);
}

fn slice(app: &mut App, _ctx: &Context) {
    app.slice();
}

fn undo(app: &mut App, _ctx: &Context) {
    app.history().undo();
}

fn redo(app: &mut App, _ctx: &Context) {
    app.history().redo();
}

fn delete_supports(app: &mut App, _ctx: &Context) {
    delete_selected(app);
}

fn select_none(app: &mut App, _ctx: &Context) {
    app.state.selected.clear();
}

fn quick_layout(app: &mut App, _ctx: &Context) {
    if app.project.models.is_empty() {
        return;
    }

    let padding = app.config.ui.quick_layout_spacing.get::<Milimeter>();
    app.tasks.add(AutoLayout::new(
        &app.project.slice_config,
        &app.project.models,
        (padding, 10.0),
    ));
}

/// Selects every model, or in the support tool every support (of the selected
/// models, if any).
fn select_all(app: &mut App, _ctx: &Context) {
    // In the support tool: every support of the selected models, or with
    // none selected, every model and all their supports.
    if app.state.tool == Tool::Support {
        let state = &mut app.state;
        if !state.selected.has_models() {
            for model in &app.project.models {
                state.selected.select_model(model.id);
            }
        }
        for model in &app.project.models {
            if state.selected.contains_model(model.id) {
                for id in model.supports.ids() {
                    state.selected_supports.select_support(model.id, id);
                }
            }
        }
        return;
    }

    app.state.selected.clear();
    for model in app.project.models.iter() {
        app.state.selected.select_model(model.id);
    }
}
