//! The layer scrubber on the viewport: cuts the view at a height and shows
//! the layer there, sliced on its own (a quick look while editing; the
//! Sliced tab shows the full slice result).

use std::sync::Arc;

use common::slice::{Layer, SliceMode};
use egui::{
    Align2, Area, Color32, ColorImage, Context, Frame, Id, Image, Label, Rect, RichText, Slider,
    TextureOptions, Vec2, Widget, load::SizedTexture, vec2,
};
use egui_phosphor::regular::{IMAGE, SPINNER};

use crate::core::App;

const CUT_TIP: &str = "Cut the view: nothing above this height is shown, so you can look into the models. Clicks reach what is shown. Double-click to stop.";
const LAYER_TIP: &str = "Show the layer at the cut, as the slicer makes it (before the post processors). Sliced on its own in the background: no full slice needed.";
/// The longest side of the layer preview image, in pixels.
const PREVIEW_PIXELS: u32 = 512;
/// The cut label's size: two lines of small text.
const LABEL_SIZE: Vec2 = vec2(64.0, 28.0);

/// The slider down the right side from the top of the models to the plate,
/// and, while cutting, the layer at the cut.
pub fn paint(app: &mut App, ctx: &Context, rect: &Rect) {
    let top = (app.project.models.iter())
        .filter(|m| !m.hidden)
        .map(|m| m.bounds.max.z)
        .fold(0.0_f32, f32::max);
    if top <= 0.0 {
        app.state.cut_height = None;
        return;
    }
    let layer_height = app.project.slice_config.slice_height.raw().max(1e-3);
    let layers = (top / layer_height).ceil().max(1.0) as u32;
    // The slider ends a whole layer above the last one starts: the top layer
    // can be looked at, and the end is no cut.
    let end = layers as f32 * layer_height;
    let state = &mut app.state;
    if let Some(cut) = state.cut_height
        && (state.cut_top - top).abs() > 1e-4
    {
        state.cut_height = Some(cut * top / state.cut_top.max(1e-3));
    }
    state.cut_top = top;

    let mut height = state.cut_height.unwrap_or(end).clamp(0.0, end);
    let set = |state: &mut crate::core::state::UiState, z: f32| {
        state.cut_height = (z < end - layer_height * 0.5).then_some(z);
    };
    Area::new(Id::new("cut_slider"))
        .anchor(Align2::RIGHT_CENTER, vec2(-10.0, 0.0))
        .constrain_to(*rect)
        .show(ctx, |ui| {
            overlay_frame(ui, |ui| {
                ui.vertical_centered(|ui| {
                    let text = match app.state.cut_height {
                        Some(_) => format!(
                            "{height:.2} mm\n{}/{layers}",
                            layer(height, layer_height) + 1
                        ),
                        None => "Cut\n".to_owned(),
                    };
                    // A fixed size: the slider must not move when the text
                    // changes (it would change the value under the pointer).
                    let label = Label::new(RichText::new(text).small()).extend();
                    ui.add_sized([LABEL_SIZE.x, LABEL_SIZE.y], label)
                        .on_hover_text(CUT_TIP);
                    ui.spacing_mut().slider_width = (rect.height() * 0.5).max(80.0);
                    let slider = Slider::new(&mut height, 0.0..=end)
                        .vertical()
                        .show_value(false)
                        .step_by(layer_height as f64)
                        .ui(ui)
                        .on_hover_text(CUT_TIP);
                    if slider.double_clicked() {
                        app.state.cut_height = None;
                    } else if slider.changed() {
                        set(&mut app.state, height);
                    }
                    // Arrow keys step one layer while hovering.
                    if slider.hovered() {
                        let step = ui.input(|i| {
                            i.key_pressed(egui::Key::ArrowUp) as i32
                                - i.key_pressed(egui::Key::ArrowDown) as i32
                        });
                        if step != 0 {
                            let next = (height + step as f32 * layer_height).clamp(0.0, end);
                            set(&mut app.state, next);
                        }
                    }
                    let show = &mut app.config.ui.layer_preview;
                    ui.toggle_value(show, IMAGE).on_hover_text(LAYER_TIP);
                });
            });
        });

    match app.state.cut_height {
        Some(cut) if app.config.ui.layer_preview => {
            layer_preview(app, ctx, rect, layer(cut, layer_height))
        }
        _ => app.state.layer_texture = None,
    }
}

/// The layer index at height `z` above the plate.
fn layer(z: f32, layer_height: f32) -> u32 {
    (z / layer_height).floor().max(0.0) as u32
}

fn overlay_frame(ui: &mut egui::Ui, content: impl FnOnce(&mut egui::Ui)) {
    Frame::new()
        .inner_margin(Vec2::splat(8.0))
        .corner_radius(ui.visuals().menu_corner_radius)
        .fill(Color32::BLACK.lerp_to_gamma(Color32::TRANSPARENT, 0.25))
        .show(ui, content);
}

/// The layer at the cut in the lower right corner of the viewport.
fn layer_preview(app: &mut App, ctx: &Context, rect: &Rect, index: u32) {
    let fresh = app.state.layer_preview.request(&app.project, index);
    let computing = fresh.is_none();
    if computing {
        // Poll until the background slice is in.
        ctx.request_repaint_after(std::time::Duration::from_millis(50));
    }
    let Some(layer) = fresh.or_else(|| app.state.layer_preview.latest().cloned()) else {
        if app.project.slice_config.mode != SliceMode::Raster {
            return;
        }
        return preview_frame(ctx, rect, None, "Slicing the layer…");
    };

    let resolution = app.project.slice_config.platform_resolution;
    let key = Arc::as_ptr(&layer) as usize;
    if app
        .state
        .layer_texture
        .as_ref()
        .is_none_or(|(k, _)| *k != key)
    {
        let image = downsample(&layer, resolution.x, resolution.y);
        let texture = ctx.load_texture("layer_preview", image, TextureOptions::LINEAR);
        app.state.layer_texture = Some((key, texture));
    }
    let Some((_, texture)) = &app.state.layer_texture else {
        return;
    };
    let mut status = format!("Layer {}", index + 1);
    if computing {
        status.push_str(&format!(" {SPINNER}"));
    }
    if layer.defects > 0 {
        status.push_str(&format!(" · {} defective lines", layer.defects));
    }
    preview_frame(ctx, rect, Some(SizedTexture::from_handle(texture)), &status);
}

fn preview_frame(ctx: &Context, rect: &Rect, texture: Option<SizedTexture>, status: &str) {
    let width = (rect.width() * 0.3).clamp(160.0, 360.0);
    Area::new(Id::new("layer_preview"))
        .anchor(Align2::RIGHT_BOTTOM, vec2(-70.0, -10.0))
        .constrain_to(*rect)
        .show(ctx, |ui| {
            overlay_frame(ui, |ui| {
                ui.small(status);
                if let Some(texture) = texture {
                    let aspect = texture.size.y / texture.size.x.max(1.0);
                    Image::new(texture)
                        .fit_to_exact_size(vec2(width, width * aspect))
                        .ui(ui);
                }
            });
        });
}

/// The layer as a small greyscale image, every `step`th pixel of the
/// platform in both directions.
fn downsample(layer: &Layer, width: u32, height: u32) -> ColorImage {
    let step = (width.max(height)).div_ceil(PREVIEW_PIXELS).max(1) as u64;
    let (w, h) = (width as u64, height as u64);
    let (out_w, out_h) = (w.div_ceil(step), h.div_ceil(step));
    let mut pixels = vec![Color32::BLACK; (out_w * out_h) as usize];

    let mut pos = 0_u64;
    for run in layer.data.iter() {
        let (start, end) = (pos, (pos + run.length).min(w * h));
        pos += run.length;
        if run.value == 0 || start >= end {
            continue;
        }
        let shade = Color32::from_gray(run.value);
        // Every sampled row the run covers.
        let mut row = (start / w).div_ceil(step) * step;
        while row * w < end && row < h {
            let row_start = start.max(row * w) - row * w;
            let row_end = end.min((row + 1) * w) - row * w;
            let mut col = row_start.div_ceil(step) * step;
            while col < row_end {
                pixels[((row / step) * out_w + col / step) as usize] = shade;
                col += step;
            }
            row += step;
        }
    }
    ColorImage::new([out_w as usize, out_h as usize], pixels)
}

#[cfg(test)]
mod tests {
    use common::container::rle::Run;

    use super::*;

    #[test]
    fn downsampling_keeps_the_sampled_pixels() {
        // 4x2 platform: row 0 is 0 0 255 255, row 1 is 255 0 0 0.
        let runs = vec![Run::new(2, 0), Run::new(3, 255), Run::new(3, 0)];
        let layer = Layer {
            data: Arc::new(runs),
            area: 0,
            height: Default::default(),
            defects: 0,
            unique_exposure: false,
            exposure: Default::default(),
        };
        let image = downsample(&layer, 4, 2);
        assert_eq!(image.size, [4, 2]);
        let lit: Vec<_> = image.pixels.iter().map(|p| p.r() > 0).collect();
        assert_eq!(lit, [false, false, true, true, true, false, false, false]);
    }
}
