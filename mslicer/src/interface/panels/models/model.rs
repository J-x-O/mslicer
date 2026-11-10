use common::{
    misc::{separate_thousands, subscript_number},
    units::Centimeter,
};
use const_format::concatcp;
use egui::{
    Align, Button, CollapsingHeader, Color32, ComboBox, Context, DragValue, Label, Layout, Popup,
    Response, RichText, Sense, TextEdit, Ui, UiBuilder, Widget, text::CCursorRange, vec2,
};
use egui_phosphor::regular::{
    ARROW_DOWN, ARROW_LINE_DOWN, ARROW_UP, ARROWS_COUNTER_CLOCKWISE, BANDAIDS, COPY, CUBE,
    CURSOR_TEXT, DICE_THREE, EYE, EYE_SLASH, FLOPPY_DISK_BACK, FOLDER_DASHED, INFO, LINE_SEGMENT,
    LINK_BREAK, LINK_SIMPLE, PLUS, SNOWFLAKE, SUBTRACT_SQUARE, SWAP, TRASH, VECTOR_THREE, WARNING,
};
use nalgebra::Vector3;

use crate::{
    core::{
        App,
        config::ui::B_PER_MIB,
        history::ModelAction,
        project::{
            Collection, RenameState,
            model::{MeshUnit, MeshWarnings, Model, ModelId},
            stack::{Mode, Op, Operation, OperationId, Repair, Section, Stack},
        },
        stack,
    },
    interface::{
        components::{being_edited, dragger, grid, history_tracked_model},
        panels::models::{
            ALIGN_SHORTCUT, COLLECT_SHORTCUT, DELETE_SHORTCUT, DUPLICATE_SHORTCUT, RENAME_SHORTCUT,
            SPLIT_SHORTCUT, UiAction, shortcut,
        },
    },
    task::{FileDialog, MeshSave, ReloadModel, SplitBodies},
};

const WARN_DEFECTIVE: &str = "This mesh is defective, it may produce unexpected results when sliced.\nConsider running it through a mesh repair tool.";
const WARN_OUT_OF_BOUNDS: &str = "This mesh extends beyond the printer volume and will be cut off.";
const SUPPORTABLE_TIP: &str =
    "Acceleration structures have been built for this mesh.\nSupports can be placed or generated.";
const UNIT_TIP: &str = "Common polygon mesh formats like .stl and .obj don't include unit information. For this reason you need to manually specify the units for each mesh, although Millimeters, the default, is often correct.";
const CUSTOM_UNIT_TIP: &str = "To use a custom unit, input the conversion factor from the input unit to Millimeters. For example if your mesh was defined in feet (for some reason) you would use 30.48.";

pub fn model_entry(
    app: &mut App,
    ui: &mut Ui,
    model: usize,
    idx: usize,
    dragged: bool,
) -> Response {
    let model = &mut app.project.models[model];
    let id = model.id;

    let (rect, response) =
        ui.allocate_exact_size(vec2(ui.available_width(), 18.0), Sense::click_and_drag());

    let selected = app.state.selected.contains_model(id);
    let color = if selected && !dragged {
        ui.visuals().selection.bg_fill
    } else if response.hovered() || dragged {
        ui.visuals().code_bg_color
    } else if idx % 2 == 1 {
        ui.visuals().faint_bg_color
    } else {
        ui.style().noninteractive().bg_fill
    };

    let rect_margin = rect.expand2(vec2(2.0, ui.spacing().item_spacing.y / 2.0));
    ui.painter().rect_filled(rect_margin, 2.0, color);

    if response.clicked() {
        app.state
            .selected
            .model_clicked(id, ui.input(|x| x.modifiers.shift));
    }

    let indent = !dragged && model.collection.is_some();
    let ui_builder =
        UiBuilder::new().max_rect(rect.with_min_x(rect.min.x + [0.0, 12.0][indent as usize]));
    ui.scope_builder(ui_builder, |ui| {
        ui.horizontal(|ui| {
            ui.visuals_mut().override_text_color = selected.then_some(Color32::WHITE);
            ui.visuals_mut().button_frame = false;

            if ui
                .button(if model.hidden { EYE_SLASH } else { EYE })
                .on_hover_text(if model.hidden { "Show" } else { "Hide" })
                .clicked()
            {
                app.history
                    .track_model(model.id, ModelAction::Hidden(model.hidden));
                model.hidden ^= true;
            }

            if matches!(model.ui.rename, RenameState::None) {
                Label::new(&model.name).selectable(false).ui(ui);
            } else {
                let mut text_edit = TextEdit::singleline(&mut model.name).show(ui);
                if matches!(model.ui.rename, RenameState::Starting) {
                    text_edit.response.request_focus();
                    (text_edit.state.cursor)
                        .set_char_range(Some(CCursorRange::select_all(&text_edit.galley)));
                    text_edit.state.store(ui.ctx(), text_edit.response.id);
                    model.ui.rename = RenameState::Editing;
                }

                let editing = being_edited(&text_edit.response);
                (!editing).then(|| model.ui.rename = RenameState::None);

                history_tracked_model(
                    (editing, ui, &mut app.history),
                    (id, || ModelAction::Name(model.name.clone())),
                )
            }

            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                let icon = |icon: &str| Label::new(icon).selectable(false);

                if !model.warnings.is_empty() {
                    let count = model.warnings.bits().count_ones();
                    let mut warn = icon(&format!("{WARNING}{}", subscript_number(count))).ui(ui);
                    for warning in model.warnings.iter() {
                        let desc = match warning {
                            MeshWarnings::Defective => WARN_DEFECTIVE,
                            MeshWarnings::OutOfBounds => WARN_OUT_OF_BOUNDS,
                            _ => continue,
                        };
                        warn = warn.on_hover_text(desc);
                    }
                }

                if model.has_acceleration_structures() {
                    icon(LINE_SEGMENT).ui(ui).on_hover_text(SUPPORTABLE_TIP);
                }
            });
        });
    });

    response
}

pub fn model_properties(
    app: &mut App,
    ui: &mut Ui,
    ctx: &Context,
    action: &mut UiAction,
    i: usize,
) {
    let model = &mut app.project.models[i];

    let platform = &app.project.slice_config.platform_size;
    let id = model.id;
    // Nothing is locked: turning, lifting or scaling a supported model
    // re-routes its supports (see `core::reroute`).
    let warning = app.config.ui.warning_color;

    ui.horizontal_wrapped(|ui| {
        if shortcut(
            ui.button(concatcp!(CURSOR_TEXT, " Rename")),
            RENAME_SHORTCUT,
        ) {
            model.ui.rename = RenameState::Starting;
        }

        if shortcut(ui.button(concatcp!(TRASH, " Delete")), DELETE_SHORTCUT) {
            *action = UiAction::Remove(i);
            app.state.selected.clear();
        }

        if shortcut(ui.button(concatcp!(COPY, " Duplicate")), DUPLICATE_SHORTCUT) {
            *action = UiAction::Duplicate(i);
        }

        if shortcut(
            ui.button(concatcp!(FOLDER_DASHED, " Collect")),
            COLLECT_SHORTCUT,
        ) {
            let collection = Collection::new_unnamed();
            model.collection = Some(collection.id);
            app.project.collections.push(collection);
        }

        let align = ui.button(concatcp!(ARROW_LINE_DOWN, " Align to Bed"));
        if shortcut(align, ALIGN_SHORTCUT) {
            let old_pos = model.mesh.position();
            app.history.track_model(
                id,
                ModelAction::Position(old_pos, model.supports.snapshot()),
            );

            model.align_to_bed();
            model.update_oob(platform);
        }

        if ui.button(concatcp!(SWAP, " Replace")).clicked() {
            let (id, name) = (model.id, model.name.clone());
            app.tasks.add(FileDialog::pick_file(
                ("Mesh", &["stl", "obj"]),
                move |_app, path, tasks| {
                    let task = ReloadModel::new(id, name, path.to_path_buf());
                    tasks.push(Box::new(task));
                },
            ));
        }

        if let Some(file) = &model.file
            && ui
                .button(concatcp!(ARROWS_COUNTER_CLOCKWISE, " Reload"))
                .clicked()
        {
            let task = ReloadModel::new(model.id, model.name.clone(), file.clone());
            app.tasks.add(task);
        }

        ui.menu_button(concatcp!(CUBE, " Mesh"), |ui| {
            if shortcut(
                ui.button(concatcp!(SUBTRACT_SQUARE, " Split Bodies")),
                SPLIT_SHORTCUT,
            ) {
                app.tasks.add(SplitBodies::new(model));
            }

            // Both are operations now: they leave the imported mesh alone.
            if ui
                .button(concatcp!(VECTOR_THREE, " Flip Winding Order"))
                .clicked()
            {
                *action = UiAction::AddOperation(i, Op::FlipWinding);
            }

            if ui.button(concatcp!(BANDAIDS, " Repair")).clicked() {
                *action = UiAction::AddOperation(i, Op::Repair(Repair::default()));
            }

            ui.menu_button(concatcp!(FLOPPY_DISK_BACK, " Export"), |ui| {
                for format in mesh_format::Format::ALL {
                    if ui
                        .button(format!("{} ({})", format.name(), format.extension()))
                        .clicked()
                    {
                        // As it prints: hollowed, if it is.
                        let mesh = model.print_mesh().inner().clone();
                        app.tasks.add(FileDialog::save_file(
                            ("Mesh", &[format.extension()]),
                            move |_app, path, tasks| {
                                let path = path.with_extension(format.extension());
                                tasks.push(Box::new(MeshSave::new(path, format, mesh)));
                            },
                        ));
                    }
                }
            });
        });
    });

    CollapsingHeader::new("Transform")
        .default_open(true)
        .show(ui, |ui| {
            grid("model_props_grid").show(ui, |ui| {
                ui.label("Position");
                ui.horizontal(|ui| {
                    let mut position = model.mesh.position();
                    let editing = dragger::vec3(ui, position.as_mut(), |x| x);
                    history_tracked_model(
                        (editing, ui, &mut app.history),
                        (id, || {
                            ModelAction::Position(model.mesh.position(), model.supports.snapshot())
                        }),
                    );
                    (editing && model.mesh.position() != position)
                        .then(|| model.set_position(platform, position));
                    ui.take_available_width();
                });
                ui.end_row();

                ui.label("Scale");

                {
                    ui.horizontal(|ui| {
                        let factor = model.unit.conversion();
                        let mut scale = model.mesh.scale() / factor;

                        let editing = if model.ui.locked_scale {
                            dragger::vec3_proportional(ui, scale.as_mut(), |x| {
                                x.speed(0.01).range(0.001..=f32::MAX)
                            })
                        } else {
                            dragger::vec3(ui, scale.as_mut(), |x| {
                                x.speed(0.01).range(0.001..=f32::MAX)
                            })
                        };
                        history_tracked_model(
                            (editing, ui, &mut app.history),
                            (id, || {
                                ModelAction::Scale(model.mesh.scale(), model.supports.snapshot())
                            }),
                        );

                        scale *= factor;
                        (editing && model.mesh.scale() != scale)
                            .then(|| model.set_scale(platform, scale));

                        model.ui.locked_scale ^= ui
                            .button([LINK_BREAK, LINK_SIMPLE][model.ui.locked_scale as usize])
                            .clicked();
                    });
                }
                ui.end_row();

                ui.label("Rotation");
                let mut rotation = rad_to_deg(model.mesh.rotation());
                let editing = dragger::vec3(ui, rotation.as_mut(), |x| x.suffix("°"));
                history_tracked_model(
                    (editing, ui, &mut app.history),
                    (id, || {
                        ModelAction::Orientation(
                            model.mesh.rotation(),
                            model.mesh.position(),
                            model.supports.snapshot(),
                        )
                    }),
                );

                if editing {
                    let rad = deg_to_rad(rotation);
                    (model.mesh.rotation() != rad).then(|| model.rotate_resting(platform, rad));
                }

                ui.end_row();

                ui.label("Color");
                ui.horizontal(|ui| {
                    let editing = Popup::is_id_open(ctx, ui.auto_id_with("popup"));
                    let original_color = model.color;
                    let mut color = model.color;
                    ui.color_edit_button_rgb(color.as_slice_mut())
                        .changed()
                        .then(|| model.color = color);
                    history_tracked_model(
                        (editing, ui, &mut app.history),
                        (id, || ModelAction::Color(original_color)),
                    );

                    if ui.button(concatcp!(DICE_THREE, " Random")).clicked() {
                        app.history.track_model(id, ModelAction::Color(model.color));
                        model.randomize_color(warning);
                    }
                });
            });
        });

    ui.collapsing("Miscellaneous", |ui| {
        grid("model_props_grid").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label("Mesh Units");
                ui.label(INFO).on_hover_text(UNIT_TIP);
            });
            ui.horizontal(|ui| {
                let last_factor = model.unit.conversion();
                ComboBox::from_id_salt("units")
                    .selected_text(model.unit.name())
                    .show_ui(ui, |ui| {
                        for unit in MeshUnit::ALL {
                            ui.selectable_value(&mut model.unit, unit, unit.name());
                        }
                    });

                if let MeshUnit::Custom(factor) = &mut model.unit {
                    DragValue::new(factor).speed(0.1).ui(ui);
                    ui.label(INFO).on_hover_text(CUSTOM_UNIT_TIP);
                }

                let factor = model.unit.conversion();
                if factor != last_factor {
                    let scale = model.mesh.scale() * factor / last_factor;
                    model.set_scale(&app.project.slice_config.platform_size, scale);
                }
            });

            ui.end_row();

            ui.label("Exposure");
            let mut value = model.exposure as f32 / 2.55;
            let editing = being_edited(
                &DragValue::new(&mut value)
                    .range(0.0..=100.0)
                    .max_decimals(0)
                    .suffix("%")
                    .ui(ui),
            );

            history_tracked_model(
                (editing, ui, &mut app.history),
                (id, || ModelAction::RelativeExposure(model.exposure)),
            );
            editing.then(|| model.exposure = (value * 2.55).round() as u8);

            ui.end_row();

            ui.label("Faces");
            ui.horizontal(|ui| {
                ui.label(format!(
                    "{} ({:.2} MiB)",
                    separate_thousands(model.mesh.face_count()),
                    model.mesh.memory_size() as f32 / B_PER_MIB as f32
                ));
                ui.take_available_width();
            });
            ui.end_row();

            if let Some(volume) = model.volume().map(|v| v.get::<Centimeter>()) {
                ui.label("Volume");
                ui.label(format!("{volume:.2} cm³"));
                ui.end_row();
            }
        });
    });
}

/// What a click in the operations list asks for.
pub enum OperationAction {
    Add(Op),
    Remove(OperationId),
    Move(OperationId, isize),
    Bypass(OperationId, bool),
    Freeze(OperationId, bool),
}

const BYPASS_TIP: &str = "Bypass: skip this operation but keep its settings.";
const FREEZE_TIP: &str = "Freeze: keep the result as it is. It is no longer recomputed when something it depends on changes, and can't be edited until thawed.";
const MODE_TIP: &str = "Auto: computed from its settings, recomputed when its inputs change.\nEdited: the auto result with your edits on top (placed supports, drilled holes).\nFrozen: kept as it is.";

/// The model's operation stack: one row per operation, in evaluation order,
/// with its mode, a bypass toggle, reordering within the geometry section,
/// freezing and removal, and adding operations.
pub fn operations(app: &mut App, ui: &mut Ui, id: ModelId) {
    let Some(model) = app.project.models.iter().find(|m| m.id == id) else {
        return;
    };
    let mut action = None;
    CollapsingHeader::new("Operations")
        .default_open(true)
        .show(ui, |ui| {
            ui.label("Source: imported mesh").on_hover_text(
                "The mesh as imported. Operations never change it: they are applied on top, in order.",
            );
            operation_rows(ui, model, &mut action);
            ui.add_space(4.0);
            if let Some(op) = add_operation_menu(ui, &model.stack) {
                action = Some(OperationAction::Add(op));
            }
        });
    apply(app, id, action);
}

fn operation_rows(ui: &mut Ui, model: &Model, action: &mut Option<OperationAction>) {
    let stack = &model.stack;
    let mut section = None;
    for operation in stack.ops() {
        if section != Some(operation.op.section()) {
            section = Some(operation.op.section());
            let heading = match operation.op.section() {
                Section::Geometry => "Geometry (model space, reorderable)",
                Section::Orientation => "Orientation",
                Section::PrintPrep => "Print preparation (on the bed)",
            };
            ui.add_space(4.0);
            ui.weak(heading);
        }
        ui.horizontal(|ui| {
            let name = RichText::new(operation.op.name());
            let name = match operation.bypassed {
                true => name.strikethrough().weak(),
                false => name,
            };
            let nodes: Vec<_> = (operation.op.nodes().iter())
                .map(|n| match n.section() {
                    Section::Geometry => format!("{} (model space)", n.name()),
                    Section::Orientation => n.name().to_owned(),
                    Section::PrintPrep => format!("{} (on the bed)", n.name()),
                })
                .collect();
            ui.label(name)
                .on_hover_text(format!("Evaluated as: {}.", nodes.join(", then ")));
            let badge = match (&operation.op, operation.mode) {
                (Op::Orient(_), Mode::Edited) => "Manual",
                (_, mode) => mode.name(),
            };
            ui.small(format!("[{badge}]")).on_hover_text(MODE_TIP);

            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                operation_buttons(ui, stack, operation, true, action);
            });
        });
        if let Some(stats) = model.repair_stats(operation.id)
            && operation.active()
        {
            let found: Vec<_> = (stats.rows().into_iter())
                .filter(|(_, n)| *n > 0)
                .map(|(name, n)| format!("{n} {}", name.to_lowercase()))
                .collect();
            match found.is_empty() {
                true => ui.weak("  No defects found."),
                false => ui.weak(format!("  Repaired {}.", found.join(", "))),
            };
        }
    }
}

/// The buttons of an operation row, right to left: remove, freeze, bypass,
/// and (with `reorder`) the arrows.
pub fn operation_buttons(
    ui: &mut Ui,
    stack: &Stack,
    operation: &Operation,
    reorder: bool,
    action: &mut Option<OperationAction>,
) {
    let op = operation.id;
    if operation.op.can_remove() && ui.small_button(TRASH).on_hover_text("Remove").clicked() {
        *action = Some(OperationAction::Remove(op));
    }
    let frozen = operation.mode == Mode::Frozen;
    if ui
        .selectable_label(frozen, SNOWFLAKE)
        .on_hover_text(FREEZE_TIP)
        .clicked()
    {
        *action = Some(OperationAction::Freeze(op, !frozen));
    }
    if operation.op.can_bypass() {
        let label = if operation.bypassed { EYE_SLASH } else { EYE };
        if ui.small_button(label).on_hover_text(BYPASS_TIP).clicked() {
            *action = Some(OperationAction::Bypass(op, !operation.bypassed));
        }
    }
    if reorder && stack.can_move(op, 1) && ui.small_button(ARROW_DOWN).clicked() {
        *action = Some(OperationAction::Move(op, 1));
    }
    if reorder && stack.can_move(op, -1) && ui.small_button(ARROW_UP).clicked() {
        *action = Some(OperationAction::Move(op, -1));
    }
}

/// The "Add Operation" menu: the operation picked, if any.
pub fn add_operation_menu(ui: &mut Ui, stack: &Stack) -> Option<Op> {
    let mut picked = None;
    ui.menu_button(concatcp!(PLUS, " Add Operation"), |ui| {
        let options = [
            (Op::Repair(Repair::default()), "Repairs the mesh: welds vertices, removes bad faces, closes holes, fixes the winding."),
            (Op::FlipWinding, "Turns every face inside out."),
            (Op::Hollow(Default::default()), "Hollows the model (see the Hollowing panel)."),
            (Op::Supports(Default::default()), "Supports (see the Supports panel)."),
        ];
        for (op, tip) in options {
            let exists = op.is_singleton()
                && (stack.ops().iter()).any(|o| o.op.name() == op.name());
            let button = ui.add_enabled(!exists, Button::new(op.name()));
            if button.on_hover_text(tip).clicked() {
                picked = Some(op);
                ui.close();
            }
        }
    });
    picked
}

/// Adds an operation picked from [`add_operation_menu`].
pub fn add_operation(app: &mut App, id: ModelId, op: Op) {
    apply(app, id, Some(OperationAction::Add(op)));
}

pub fn apply(app: &mut App, id: ModelId, action: Option<OperationAction>) {
    match action {
        Some(OperationAction::Add(Op::Hollow(_))) => stack::set_hollowed(stack::of(app), id, true),
        Some(OperationAction::Add(op)) => {
            stack::add(stack::of(app), id, op);
        }
        Some(OperationAction::Remove(op)) => stack::remove(stack::of(app), id, op),
        Some(OperationAction::Move(op, by)) => stack::move_by(stack::of(app), id, op, by),
        Some(OperationAction::Bypass(op, on)) => stack::set_bypassed(stack::of(app), id, op, on),
        Some(OperationAction::Freeze(op, on)) => stack::set_frozen(stack::of(app), id, op, on),
        None => {}
    }
}

fn rad_to_deg(pos: Vector3<f32>) -> Vector3<f32> {
    pos.map(|x| x.to_degrees())
}

fn deg_to_rad(pos: Vector3<f32>) -> Vector3<f32> {
    pos.map(|x| x.to_radians())
}
