use std::collections::HashSet;

use egui::{
    Align2, Area, Button, Color32, Context, FontId, Frame, Id, Key, Order, Painter, PointerButton,
    Pos2, Rect, Response, RichText, Sense, Stroke, StrokeKind, Theme, Ui, Vec2, Widget, pos2, vec2,
};
use egui_phosphor::regular::{INFO, MAGNIFYING_GLASS};
use egui_wgpu::Callback;
use nalgebra::{Matrix4, Rotation3, Vector2, Vector3};
use slicer::geometry::bounds::Bounds2D;
use tools::supports::PartKind;

use crate::{
    core::{
        App,
        history::ModelAction,
        islands,
        project::model::Model,
        selected::SelectedSupport,
        state::{CameraFocus, GeometryHit, Tool, WorkspaceHover},
    },
    interface::{
        components::grid,
        panels::supports::{drag_support, manual_support_placement, start_support_drag},
    },
    render::{
        camera::{Limits, Pivots},
        interface::basis::BasisRenderCallback,
        workspace::{WorkspaceRenderCallback, model::pick::pick_selection},
    },
    task::BuildAccelerationStructures,
};

#[rustfmt::skip]
const NUMBER_KEYS: [Key; 10] = [Key::Num0, Key::Num1, Key::Num2, Key::Num3, Key::Num4, Key::Num5, Key::Num6, Key::Num7, Key::Num8, Key::Num9];
const CANT_SUPPORT: &str =
    "Supports can't yet be placed. Click to initialize acceleration structures.";
const TOGGLE_HOVER_TIP: &str = "Toggle hover overlay.\nShift click to toggle detailed view.";

pub fn ui(app: &mut App, ui: &mut Ui, ctx: &Context) {
    let (rect, response) = ui.allocate_exact_size(ui.available_size(), Sense::click_and_drag());

    let aspect = rect.width() / rect.height();
    let px = ui.input(|i| i.pointer.latest_pos()).unwrap_or_default();
    let uv = (px - rect.min) / rect.size();

    let modifiers = ui.input(|i| i.modifiers);
    let shift_down = modifiers.shift;
    app.state.mini_support = modifiers.command;

    // The left button belongs to the tools: dragging a support in the support
    // tool moves it, any other drag draws a selection box. Alt + left orbits.
    if response.drag_started_by(PointerButton::Primary)
        && !modifiers.alt
        && app.state.select_start.is_none()
        && app.state.support_drag.is_none()
    {
        let grabbed = app.state.tool == Tool::Support
            && !shift_down
            && start_support_drag(app, app.state.hovered_geometry);
        if !grabbed {
            app.state.select_start = Some(uv);
        }
    }

    if !response.dragged_by(PointerButton::Primary)
        && let Some(start) = app.state.select_start.take()
    {
        let bounds = Bounds2D::<f32>::new_containing(&[
            Vector2::new(start.x, start.y),
            Vector2::new(uv.x, uv.y),
        ]);
        // Shift adds to the selection, a plain box replaces it.
        if !shift_down && !bounds.is_empty() {
            match app.state.tool {
                Tool::Select => app.state.selected.clear(),
                Tool::Support => app.state.selected_supports.clear(),
                Tool::Orient => {}
            }
        }
        match app.state.tool {
            Tool::Support => {
                for x in supports_in(app, &bounds) {
                    (app.state.selected_supports).select_support(x.model, x.support);
                }
            }
            _ => pick_selection(app, bounds),
        }
    }
    // While a box is drawn, the supports it would select show as hovered.
    app.state.boxed_supports = match app.state.select_start {
        Some(start) if app.state.tool == Tool::Support => {
            let bounds = Bounds2D::<f32>::new_containing(&[
                Vector2::new(start.x, start.y),
                Vector2::new(uv.x, uv.y),
            ]);
            supports_in(app, &bounds).into_iter().collect()
        }
        _ => HashSet::new(),
    };

    let keyboard_free = !ctx.wants_keyboard_input();
    if keyboard_free && ui.input(|i| i.key_pressed(Key::Escape)) {
        // Escape cancels what is under way, else clears the selection.
        let cancelled =
            app.state.support_drag.take().is_some() || app.state.select_start.take().is_some();
        if cancelled {
            app.state.support_preview = None;
        } else {
            app.state.selected.clear();
            app.state.selected_supports.clear();
        }
    }
    if keyboard_free && response.hovered() && ui.input(|i| i.key_pressed(Key::F)) {
        frame_selection(app);
    }
    // Steps through the unsupported islands while a check runs.
    if keyboard_free
        && response.hovered()
        && app.state.island_check.is_some()
        && ui.input(|i| i.key_pressed(Key::N) && !i.modifiers.command)
    {
        islands::step(app, !ui.input(|i| i.modifiers.shift));
    }

    let dragging_area = app.state.select_start.is_some();
    let dragging_support = app.state.support_drag.is_some();
    let pivots = camera_pivots(app, &response, ui);
    let camera_moving = !dragging_area
        && !dragging_support
        && app
            .camera
            .handle_movement(&response, ui, app.config.render.projection, pivots);

    let focused = ui.input(|i| i.focused);
    let mut is_moving = app.spacenav().handle_movement(focused);
    if is_moving {
        app.state.move_timeout = ((0.025 / app.fps.avg_frame_time(None)).round() as u32).min(60);
    } else if app.state.move_timeout > 0 {
        app.state.move_timeout -= 1;
        is_moving = true;
    }
    is_moving |= camera_moving;
    app.state.workspace = WorkspaceHover::new(is_moving, aspect, uv);

    // During an island check, the support tool lights up the island under
    // the pointer. Supports still go exactly where it points.
    if app.state.tool == Tool::Support && !is_moving {
        islands::track_hover(app);
    } else {
        islands::clear_hover(app);
    }

    if app.state.tool != Tool::Support {
        app.state.support_drag = None;
        app.state.support_preview = None;
    } else if dragging_support {
        drag_support(app, !response.dragged_by(PointerButton::Primary));
    } else if dragging_area {
        app.state.support_preview = None;
    } else if app.state.support_settling.is_none() {
        manual_support_placement(app, false)
    }

    if response.clicked() && !is_moving {
        match app.state.tool {
            Tool::Select => match app.state.hovered_geometry {
                Some(hover) => app.state.selected.model_clicked(hover.model, shift_down),
                None if !shift_down => app.state.selected.clear(),
                None => {}
            },
            Tool::Support => {
                let selected = &mut app.state.selected;
                match app.state.hovered_geometry {
                    // Empty space clears everything.
                    None if !shift_down => {
                        selected.clear();
                        app.state.selected_supports.clear();
                    }
                    None => {}
                    // A model you aren't working on: work on it now. Its
                    // supports only become editable once it is selected.
                    Some(hit) if !selected.contains_model(hit.model) => {
                        selected.model_clicked(hit.model, shift_down);
                    }
                    Some(hit) if hit.support => {
                        if let Some(model) = app.project.model(hit.model)
                            && let Some(support) = model.supports.support_for_face(hit.face)
                        {
                            (app.state.selected_supports)
                                .support_clicked(model.id, support, shift_down);
                        }
                    }
                    Some(_) => manual_support_placement(app, true),
                }

                if let Some(hover) = app.state.hovered_geometry
                    && let Some(model) = app.project.models.iter().find(|x| x.id == hover.model)
                    && let Some(task) = BuildAccelerationStructures::request(&mut app.state, model)
                {
                    app.tasks.add(task);
                }
            }
            Tool::Orient => {
                let platform = app.project.slice_config.platform_size;
                if let Some(hover) = app.state.hovered_geometry
                    && let Some(model) = app.project.model(hover.model)
                {
                    let rotation = hover_rotation(model, hover.face as usize);
                    let undo = ModelAction::Orientation(
                        model.mesh.rotation(),
                        model.mesh.position(),
                        model.supports.snapshot(),
                    );
                    app.history.track_model(model.id, undo);

                    let center = model.bounds.center();
                    model.set_rotation(&platform, rotation);
                    model.set_bounds_center(&platform, center);
                    model.align_to_bed();
                }
            }
        }
    }

    let painter = ui.painter();
    let color = match app.config.ui.theme {
        Theme::Dark => Color32::from_rgb(9, 9, 9),
        Theme::Light => Color32::from_rgb(255, 255, 255),
    };
    painter.rect_filled(rect, 0.0, color);

    let scale_factor = ui.ctx().pixels_per_point();
    let size = (Vector2::new(rect.width(), rect.height()) * scale_factor).map(|x| x.round() as u32);
    let callback = app.get_workspace_render_callback(size);
    painter.add(Callback::new_paint_callback(rect, callback));

    paint_area_selection(painter, app, &rect, px);
    paint_notice(painter, app, &rect);
    paint_basis_vectors(painter, app, &rect);
    paint_toolbar(app, ctx, &rect);

    // Can't just use `response.contains_pointer()` since the hover overlay can
    // take focus away from the viewport.
    if app.state.select_start.is_none()
        && let Some(hover) = app.state.hovered_geometry
        && response.rect.contains(px)
        && ctx.layer_id_at(px) == Some(ui.layer_id())
    {
        paint_hover_overlay(ui, app, hover, px);
    }
}

/// Every support of a selected model with a point inside the screen-space
/// box `area` (in viewport uv), hidden or not.
fn supports_in(app: &App, area: &Bounds2D<f32>) -> Vec<SelectedSupport> {
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
    let selected = (app.project.models.iter()).filter(|m| app.state.selected.contains_model(m.id));
    for model in selected {
        let ids = model.supports.find_by_point(inside);
        out.extend(ids.into_iter().map(|id| SelectedSupport::new(model.id, id)));
    }
    out
}

/// What zoom and orbit go about. Normally the point under the pointer: for an
/// orbit, where the drag started. While the camera is focused on something,
/// its target, until the view is scrolled three notches out or panned.
fn camera_pivots(app: &mut App, response: &Response, ui: &Ui) -> Pivots {
    let (scroll, modifiers) = ui.input(|i| (i.smooth_scroll_delta, i.modifiers));
    // Shift turns the wheel into a horizontal scroll in egui.
    let scroll = if modifiers.shift {
        scroll.x + scroll.y
    } else {
        scroll.y
    };
    let secondary = response.dragged_by(PointerButton::Secondary);
    let panning = response.dragged_by(PointerButton::Middle) || (secondary && modifiers.shift);
    let pan_start = response.drag_started_by(PointerButton::Middle)
        || (response.drag_started_by(PointerButton::Secondary) && modifiers.shift);
    let orbit_start = (response.drag_started_by(PointerButton::Secondary) && !modifiers.shift)
        || (response.drag_started_by(PointerButton::Primary) && modifiers.alt);
    let orbiting = (secondary && !modifiers.shift)
        || (response.dragged_by(PointerButton::Primary) && modifiers.alt);

    if let Some(focus) = &mut app.state.camera_focus {
        if response.hovered() {
            focus.scrolled_out = (focus.scrolled_out - scroll).max(0.0);
        }
        if panning || focus.scrolled_out >= CameraFocus::RELEASE {
            app.state.camera_focus = None;
        }
    }
    if !orbiting {
        app.state.orbit_pivot = None;
    }
    let limits = Some(camera_limits(app));
    if app.state.camera_focus.is_some() {
        return Pivots {
            limits,
            ..Default::default()
        };
    }

    let scrolling = scroll != 0.0 && response.hovered();
    if !(scrolling || pan_start || orbit_start) {
        return Pivots {
            orbit: app.state.orbit_pivot,
            limits,
            ..Default::default()
        };
    }
    let ray = app.hovered_ray();
    let hit = ray.and_then(|ray| pointer_hit(app, ray));
    let point = ray.zip(hit).map(|((pos, dir), t)| pos + dir * t);
    // Pan and orbit work at the depth of what was grabbed.
    if (pan_start || orbit_start)
        && let Some(point) = point
    {
        app.camera.reseat(app.config.render.projection, point);
    }
    if orbit_start {
        app.state.orbit_pivot = point;
    }
    Pivots {
        ray: scrolling.then_some(ray).flatten(),
        hit: scrolling.then_some(hit).flatten(),
        orbit: app.state.orbit_pivot,
        limits,
    }
}

/// How far along `ray` the nearest visible model surface is, else the build
/// plate.
fn pointer_hit(app: &App, (pos, dir): (Vector3<f32>, Vector3<f32>)) -> Option<f32> {
    let surface = (app.project.models.iter())
        .filter(|m| !m.hidden)
        .filter_map(|m| {
            let hit = m.bvh.as_ref()?.intersect_ray(&m.mesh, pos, dir)?;
            Some((m.mesh.transform(&hit.position) - pos).norm())
        })
        .min_by(f32::total_cmp);
    surface.or_else(|| {
        let half = app.project.slice_config.platform_size.map(|x| x.raw()) / 2.0;
        let t = -pos.z / dir.z;
        let at = pos + dir * t;
        (t > 0.0 && at.x.abs() <= half.x && at.y.abs() <= half.y).then_some(t)
    })
}

/// The build volume grown by its diagonal holds the target; the camera stays
/// within ten diagonals of it.
fn camera_limits(app: &App) -> Limits {
    let size = app.project.slice_config.platform_size.map(|x| x.raw());
    let diagonal = size.norm();
    let half = Vector3::new(size.x / 2.0, size.y / 2.0, 0.0);
    Limits {
        min: -half - Vector3::repeat(diagonal),
        max: half + Vector3::new(0.0, 0.0, size.z) + Vector3::repeat(diagonal),
        max_distance: 10.0 * diagonal,
    }
}

/// Points the camera at the selected models, the models of the selected
/// supports, or else every model.
fn frame_selection(app: &mut App) {
    let models = &app.project.models;
    let selected = |id| {
        app.state.selected.contains_model(id)
            || app.state.selected_supports.for_model(id).next().is_some()
    };
    let any = models.iter().any(|m| selected(m.id));
    let mut bounds = (models.iter())
        .filter(|m| !any || selected(m.id))
        .map(|m| (m.bounds.min, m.bounds.max));
    let Some(first) = bounds.next() else {
        return;
    };
    let (min, max) = bounds.fold(first, |(min, max), (a, b)| (min.inf(&a), max.sup(&b)));
    app.camera.frame(min, max);
    app.state.camera_focus = Some(Default::default());
}

fn paint_area_selection(painter: &Painter, app: &mut App, rect: &Rect, px: Pos2) {
    if let Some(start) = app.state.select_start {
        let start = start * rect.size() + rect.min.to_vec2();
        let rect = Rect::from_two_pos(start.to_pos2(), px);

        let [r, g, b] = app
            .config
            .ui
            .accent_color
            .map(|x| (x * 255.0).round() as u8);
        let accent = Color32::from_rgb(r, g, b);
        let stroke = Stroke::new(2_f32, accent);
        let fill = accent.lerp_to_gamma(Color32::TRANSPARENT, 0.9);

        painter.rect_filled(rect, 4.0, fill);
        painter.rect_stroke(rect, 4.0, stroke, StrokeKind::Middle);
    }
}

fn paint_toolbar(app: &mut App, ctx: &Context, rect: &Rect) {
    Area::new(Id::new("toolbar"))
        .anchor(Align2::LEFT_TOP, Vec2::splat(10.0))
        .constrain_to(*rect)
        .show(ctx, |ui| {
            Frame::new()
                .inner_margin(Vec2::splat(8.0))
                .corner_radius(ui.visuals().menu_corner_radius)
                .fill(Color32::BLACK.lerp_to_gamma(Color32::TRANSPARENT, 0.25))
                .show(ui, |ui| {
                    let button = |icon: &str| {
                        Button::new(RichText::new(icon).size(20.0)).min_size(Vec2::splat(30.0))
                    };

                    for (i, tool) in Tool::ALL.into_iter().enumerate() {
                        let button = (button(tool.icon()).selected(tool == app.state.tool).ui(ui))
                            .on_hover_text(format!("{}\nShortcut: {}", tool.name(), i + 1));

                        let shortcut = ui.input(|x| x.key_pressed(NUMBER_KEYS[i + 1]))
                            && !ui.ctx().wants_keyboard_input();
                        (button.clicked() || shortcut).then(|| app.state.set_tool(tool));
                    }

                    ui.separator();
                    let hover_overlay = &mut app.config.ui.hover_overlay;
                    let hover_detail = &mut app.config.ui.hover_overlay_detail;

                    let icon = [INFO, MAGNIFYING_GLASS][*hover_detail as usize];
                    *[hover_overlay, hover_detail][ui.input(|i| i.modifiers.shift) as usize] ^=
                        (button(icon).selected(*hover_overlay).ui(ui))
                            .on_hover_text(TOGGLE_HOVER_TIP)
                            .clicked();
                })
        });
}

/// How long a notice stays up, fading out over its last second.
const NOTICE_TIME: f32 = 4.0;

/// The current notice, centred near the bottom of the viewport.
fn paint_notice(painter: &Painter, app: &mut App, rect: &Rect) {
    let Some((text, since)) = &app.state.notice else {
        return;
    };
    let age = since.elapsed().as_secs_f32();
    if age > NOTICE_TIME {
        app.state.notice = None;
        return;
    }
    painter.ctx().request_repaint();
    let alpha = (NOTICE_TIME - age).min(1.0);

    let visuals = painter.ctx().style().visuals.clone();
    let galley = painter.layout_no_wrap(
        text.clone(),
        FontId::proportional(14.0),
        visuals.text_color().gamma_multiply(alpha),
    );
    let center = rect.center_bottom() - Vec2::new(0.0, 40.0);
    let frame = Rect::from_center_size(center, galley.size() + Vec2::new(24.0, 12.0));
    painter.rect_filled(frame, 6.0, visuals.window_fill.gamma_multiply(0.9 * alpha));
    painter.galley(frame.min + Vec2::new(12.0, 6.0), galley, Color32::WHITE);
}

fn paint_basis_vectors(painter: &Painter, app: &mut App, rect: &Rect) {
    let size = app.config.render.basis_size;
    if size == 0.0 {
        return;
    }

    let pad = 4.0;
    let color = |r, g, b, a| Color32::from_rgba_unmultiplied(r, g, b, a);
    let (color_bg, color_edge) = match app.config.ui.theme {
        Theme::Dark => (color(255, 255, 255, 40), color(255, 255, 255, 180)),
        Theme::Light => (color(0, 0, 0, 40), color(0, 0, 0, 180)),
    };
    let stroke = Stroke::new(1.5_f32, color_edge);

    let rect = Rect::from_min_size(
        pos2(rect.max.x - size - pad, rect.min.y + pad),
        vec2(size, size),
    );
    painter.rect_filled(rect, size / 2.0, color_bg);
    painter.rect_stroke(rect, size / 2.0, stroke, StrokeKind::Outside);

    painter.add(Callback::new_paint_callback(
        rect.expand(-pad),
        BasisRenderCallback {
            camera: app.camera.clone(),
        },
    ));
}

fn paint_hover_overlay(ui: &mut Ui, app: &mut App, hover: GeometryHit, px: Pos2) {
    let p = Vec2::splat(8.0);
    let enabled = app.config.ui.hover_overlay;
    let detail = app.config.ui.hover_overlay_detail;

    let Some(model) = app.project.model(hover.model) else {
        return;
    };

    if !enabled && !(app.state.tool == Tool::Support && !model.has_acceleration_structures()) {
        return;
    }

    Area::new(Id::new("hover_overlay"))
        .order(Order::Tooltip)
        .fixed_pos(px + vec2(p.x, -p.y))
        .pivot(Align2::LEFT_BOTTOM)
        .interactable(false)
        .show(ui.ctx(), |ui| {
            let id = Id::new("toolbar");
            let last_size = ui.data(|d| d.get_temp::<Vec2>(id));

            // kinda jank but better than nothing
            if let Some(last_size) = last_size {
                let rect = Align2::LEFT_BOTTOM.anchor_size(px + vec2(p.x, -p.y), last_size);
                ui.shrink_clip_rect(rect);
            }

            let response = Frame::new()
                .inner_margin(p)
                .corner_radius(ui.visuals().menu_corner_radius)
                .fill(Color32::BLACK.lerp_to_gamma(Color32::TRANSPARENT, 0.25))
                .show(ui, |ui| {
                    ui.style_mut().visuals.override_text_color = Some(Color32::WHITE);
                    ui.set_max_width(300.0);

                    if enabled {
                        grid("overlay").spacing([8.0, 4.0]).show(ui, |ui| {
                            ui.label("Model");
                            ui.label(&model.name);
                            ui.end_row();

                            if detail && !hover.support {
                                ui.label("Face");
                                let [a, b, c] = model.mesh.face(hover.face as usize);
                                ui.label(format!("{} ({a}, {b}, {c})", hover.face));
                                ui.end_row();

                                ui.label("Normal");
                                let normal = model.mesh.normal(hover.face as usize);
                                let transformed_normal = model.mesh.transform_normal(&normal);
                                ui.vertical(|ui| {
                                    ui.label(format_vector(&normal));

                                    if transformed_normal != normal {
                                        ui.horizontal(|ui| {
                                            ui.label(format_vector(&transformed_normal));
                                            ui.label("(Transformed)");
                                        });
                                    }
                                });
                                ui.end_row();
                            }

                            if hover.support
                                && let Some(id) = model.supports.support_for_face(hover.face)
                                && let Some(support) = model.supports.get(id)
                            {
                                ui.label("Support");
                                ui.label(format!("#{}", id.raw()));
                                ui.end_row();

                                let tip_cone =
                                    support.parts.iter().find(|p| p.kind == PartKind::TipCone);
                                let pillar =
                                    support.parts.iter().find(|p| p.kind == PartKind::Pillar);

                                ui.label("Radius");
                                ui.label(format!(
                                    "{:.2}mm ({:.2}mm tip)",
                                    tip_cone.map(|p| p.radius_b).unwrap_or(0.0),
                                    tip_cone.map(|p| p.radius_a).unwrap_or(0.0),
                                ));
                                ui.end_row();

                                ui.label("Touch Point");
                                ui.vertical(|ui| {
                                    ui.label(format_vector(&support.point.pos));

                                    if detail {
                                        if let Some(tip) = tip_cone {
                                            ui.label(format_vector(&tip.b));
                                        }
                                        if let Some(p) = pillar {
                                            ui.label(format_vector(&p.a));
                                        }
                                    }
                                });
                                ui.end_row();
                            }

                            if app.state.tool == Tool::Orient {
                                ui.label("Rotation");
                                let rotation = hover_rotation(model, hover.face as usize)
                                    .map(|x| x.to_degrees());
                                ui.label(format!(
                                    "{:.2}° × {:.2}° × {:.2}°",
                                    rotation.x, rotation.y, rotation.z
                                ));
                                ui.end_row();
                            }
                        });
                    }

                    if app.state.tool == Tool::Support && !model.has_acceleration_structures() {
                        enabled.then(|| ui.add_space(8.0));
                        ui.label(CANT_SUPPORT);
                    }
                })
                .response;

            ui.memory_mut(|m| m.data.insert_temp(id, response.rect.size()));
        });
}

fn format_vector(normal: &Vector3<f32>) -> String {
    const SIGN: [&str; 2] = ["", "+"];
    format!(
        "‹{}{:.2}, {}{:.2}, {}{:.2}›",
        SIGN[(normal.x > 0.0) as usize],
        normal.x,
        SIGN[(normal.y > 0.0) as usize],
        normal.y,
        SIGN[(normal.z > 0.0) as usize],
        normal.z,
    )
}

impl App {
    pub fn get_workspace_render_callback(
        &mut self,
        viewport: Vector2<u32>,
    ) -> WorkspaceRenderCallback {
        WorkspaceRenderCallback {
            app: self as *mut _,
            viewport,
        }
    }

    pub fn view_projection(&self) -> Matrix4<f32> {
        let aspect = self.state.workspace.aspect;
        self.camera
            .view_projection_matrix(self.config.render.projection, aspect)
    }
}

fn hover_rotation(model: &Model, face: usize) -> Vector3<f32> {
    let normal = model.mesh.normal(face);
    let down = Vector3::new(0.0, 0.0, -1.0);
    let matrix = Rotation3::rotation_between(&normal, &down);

    if let Some(matrix) = matrix {
        let (roll, pitch, yaw) = matrix.euler_angles();
        Vector3::new(roll, pitch, yaw)
    } else {
        Vector3::zeros()
    }
}
