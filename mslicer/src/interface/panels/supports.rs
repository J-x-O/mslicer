use std::collections::HashMap;

use common::units::{Milimeter, Milimeters};
use const_format::concatcp;
use egui::{Button, ComboBox, Context, DragValue, Ui, Widget};
use egui_phosphor::regular::{LINE_SEGMENT, SELECTION_INVERSE, SPARKLE, TRASH};
use nalgebra::Vector3;
use tools::supports::{BaseShape, auto::route_point, mesh::SupportPreset, raft::RaftShape};

use crate::{
    core::App,
    interface::components::grid,
    task::{BuildAccelerationStructures, GenerateSupports},
};

const IN_DEV_WARNING: &str = "Support generation and placement is still in development. For the time being consider using RuneBrace for support placement.\n\nAt the moment supports are not saved into the project file!";

pub fn ui(app: &mut App, ui: &mut Ui, _ctx: &Context) {
    ui.horizontal_wrapped(|ui| {
        prepare(app, ui);

        ui.menu_button(concatcp!(SPARKLE, " Auto"), |ui| {
            ui.set_width(200.0);

            let mut any = false;
            for idx in 0..app.project.models.len() {
                let model = &app.project.models[idx];
                if model.has_acceleration_structures() {
                    any = true;
                    if ui.button(&model.name).clicked() {
                        generate_support(app, idx);
                    }
                }
            }

            if !any {
                ui.label("No models have been prepared for supports.");
            }
        });
    });

    let selected = &mut app.state.selected_supports;
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
                for support in selected.iter() {
                    let model = app.project.model(support.model).unwrap();
                    model.supports.remove(support.support);
                    model.supports.invalidate_cache();
                }
                selected.clear();
            }

            ui.button(concatcp!(SELECTION_INVERSE, " Deselect"))
                .clicked()
                .then(|| selected.clear());
        });

        ui.add_space(4.0);
        ui.separator();
        ui.add_space(4.0);
    } else {
        ui.add_space(8.0);
    }

    ui.horizontal(|ui| {
        ui.label("Preset");
        let preset = &mut app.project.support_config.preset;
        ComboBox::from_id_salt("preset")
            .selected_text(preset.name().unwrap_or("Custom"))
            .show_ui(ui, |ui| {
                for (name, value) in SupportPreset::PRESETS {
                    ui.selectable_value(preset, value, name);
                }
            });
    });
    ui.add_space(4.0);

    preset_groups(ui, &mut app.project.support_config.preset);

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
    ui.label(IN_DEV_WARNING);
}

/// The preset's settings, grouped as top / middle / bottom / raft. Diameters
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

    ui.collapsing("Raft", |ui| {
        grid("support_raft").show(ui, |ui| {
            row(ui, "Shape", |ui| {
                let name = |shape| match shape {
                    RaftShape::None => "None",
                    RaftShape::Skate => "Skate",
                    RaftShape::CrossGrid => "Cross Grid",
                };
                ComboBox::from_id_salt("raft_shape")
                    .selected_text(name(preset.raft_shape))
                    .show_ui(ui, |ui| {
                        for shape in [RaftShape::None, RaftShape::Skate, RaftShape::CrossGrid] {
                            ui.selectable_value(&mut preset.raft_shape, shape, name(shape));
                        }
                    });
            });
            row(ui, "Area Ratio", |ui| {
                let mut percent = preset.raft_area_ratio * 100.0;
                if DragValue::new(&mut percent)
                    .suffix(" %")
                    .range(100.0..=200.0)
                    .ui(ui)
                    .changed()
                {
                    preset.raft_area_ratio = percent / 100.0;
                }
            });
            row(ui, "Thickness", |ui| mm(ui, &mut preset.raft_thickness));
            row(ui, "Lip Height", |ui| mm(ui, &mut preset.raft_height));
            row(ui, "Lip Angle", |ui| {
                DragValue::new(&mut preset.raft_angle)
                    .suffix("°")
                    .range(5.0..=90.0)
                    .ui(ui);
            });
            row(ui, "Grid Cell", |ui| mm(ui, &mut preset.raft_grid_cell));
            row(ui, "Grid Bar Width", |ui| {
                mm(ui, &mut preset.raft_grid_width)
            });
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

fn generate_support(app: &mut App, model: usize) {
    let default_lift = app.project.support_config.default_lift.get::<Milimeter>();

    let model = &mut app.project.models[model];

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

    if let Some(task) = GenerateSupports::new(platform, placement, support, model) {
        app.tasks.add(task);
    }
}

pub fn manual_support_placement(app: &mut App, clicked: bool) {
    let workspace = &app.state.workspace;
    if workspace.is_moving {
        return;
    }

    let Some((pos, dir)) = app.hovered_ray() else {
        return;
    };

    let mut preview = None;
    for model in app.project.models.iter_mut() {
        let Some(bvh) = model.bvh.as_ref() else {
            continue;
        };

        let Some(intersection) = bvh.intersect_ray(&model.mesh, pos, dir) else {
            continue;
        };

        let config = &app.project.support_config.preset;
        let normal = (model.mesh)
            .transform_normal(&model.mesh.normal(intersection.face))
            .normalize();
        if normal.z > 0.0 {
            continue;
        }

        let intersection = model.mesh.transform(&intersection.position);
        if let Some(routed) = route_point(&model.mesh, bvh, config, intersection, normal) {
            preview = model.supports.preview_manual(config, &routed);
            if clicked {
                model.supports.add_manual(config, &routed);
            }
        }
    }

    app.state.support_preview = preview;
}

fn prepare(app: &mut App, ui: &mut Ui) {
    let active = (app.project.models.iter()).any(|model| !model.has_acceleration_structures());
    let button = Button::new(concatcp!(LINE_SEGMENT, " Prepare All"));
    if ui.add_enabled(active, button).clicked() {
        let mut meshes = HashMap::new();
        for model in app.project.models.iter() {
            if !model.has_acceleration_structures() {
                meshes.entry(model.mesh.mesh_id()).or_insert(&model.mesh);
            }
        }

        for mesh in meshes.into_values() {
            app.tasks.add(BuildAccelerationStructures::new_mesh(
                mesh.mesh_id(),
                mesh.inner().clone(),
            ));
        }
    }
}

// NO DRUGS
// remember that.
