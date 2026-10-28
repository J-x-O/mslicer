//! Colour rules for models and their supports. A model's supports are drawn
//! in the colour opposite its own, and the warning colour's hue is kept free
//! for warnings: no random model colour and no support colour lands near it.

use std::f32::consts::{PI, TAU};

use common::color::{LinearRgb, OkLab, SRgb, START_COLOR};

/// Hues closer than this (radians, about 35°) to the warning hue are kept
/// for warnings.
const WARNING_BAND: f32 = 0.61;
/// Supports sit at this lightness, well below the white of hover and
/// selection, at a chroma in this range.
const SUPPORT_LIGHTNESS: f32 = 0.62;
const SUPPORT_CHROMA: (f32, f32) = (0.07, 0.12);
/// Random model colours have a little less chroma than the chart colours:
/// whole models in full colour are loud.
const MODEL_CHROMA: f32 = 0.9;

/// The signed angle from `b` to `a`, in [-π, π).
fn hue_offset(a: f32, b: f32) -> f32 {
    (a - b + PI).rem_euclid(TAU) - PI
}

/// `hue`, pushed out of the band around `warning` to its nearer edge.
fn clear_of(hue: f32, warning: f32) -> f32 {
    let offset = hue_offset(hue, warning);
    if offset.abs() >= WARNING_BAND {
        return hue;
    }
    warning + WARNING_BAND.copysign(offset)
}

fn hue_of(srgb: [f32; 3]) -> f32 {
    let [r, g, b] = srgb;
    OkLab::from_linear_srgb(SRgb::new(r, g, b).to_linear_rgb()).hue()
}

/// The colour of the supports of a model coloured `model`, in sRGB: the
/// opposite hue (unless that is the warning hue), at a fixed lightness and
/// moderate chroma, so hover and selection white stand out on it.
pub fn support_color(model: LinearRgb<f32>, warning: [f32; 3]) -> [f32; 3] {
    let model = OkLab::from_linear_srgb(model);
    let hue = clear_of(model.hue() + PI, hue_of(warning));
    let chroma = model.chroma().clamp(SUPPORT_CHROMA.0, SUPPORT_CHROMA.1);
    let rgb = OkLab::from_lch(SUPPORT_LIGHTNESS, chroma, hue).to_linear_srgb();
    let rgb = LinearRgb::new(
        rgb.r.clamp(0.0, 1.0),
        rgb.g.clamp(0.0, 1.0),
        rgb.b.clamp(0.0, 1.0),
    );
    let srgb = rgb.to_srgb();
    [srgb.r, srgb.g, srgb.b]
}

/// A random model colour whose hue, and the opposite one its supports get,
/// both stay clear of the warning hue.
pub fn random_model_color(warning: [f32; 3]) -> LinearRgb<f32> {
    let warning = hue_of(warning);
    let base = START_COLOR;
    let chroma = MODEL_CHROMA * base.chroma();
    let clear = |hue: f32| {
        hue_offset(hue, warning).abs() >= WARNING_BAND
            && hue_offset(hue + PI, warning).abs() >= WARNING_BAND
    };
    // Two bands of about 70° are out; a few tries always find a hue.
    let hue = (0..64)
        .map(|_| rand::random::<f32>() * TAU)
        .find(|&hue| clear(hue))
        .unwrap_or(warning + PI / 2.0);
    OkLab::from_lch(base.l, chroma, hue).to_linear_srgb()
}

#[cfg(test)]
mod tests {
    use super::*;

    const RED: [f32; 3] = [1.0, 0.1, 0.1];

    #[test]
    fn supports_of_a_cyan_model_are_not_red() {
        let cyan = SRgb::new(0.0, 174.0 / 255.0, 216.0 / 255.0).to_linear_rgb();
        let support = support_color(cyan, RED);
        assert!(hue_offset(hue_of(support), hue_of(RED)).abs() >= WARNING_BAND - 1e-3);
    }

    #[test]
    fn supports_of_a_light_blue_model_are_not_near_white() {
        let blue = SRgb::new(122.0 / 255.0, 126.0 / 255.0, 249.0 / 255.0).to_linear_rgb();
        let [r, g, b] = support_color(blue, RED);
        let lab = OkLab::from_linear_srgb(SRgb::new(r, g, b).to_linear_rgb());
        assert!(lab.l < 0.7, "lightness {}", lab.l);
        assert!(lab.chroma() > 0.05);
    }

    #[test]
    fn random_colours_and_their_supports_avoid_the_warning_hue() {
        let warning = hue_of(RED);
        for _ in 0..200 {
            let color = OkLab::from_linear_srgb(random_model_color(RED));
            assert!(hue_offset(color.hue(), warning).abs() >= WARNING_BAND - 1e-3);
            assert!(hue_offset(color.hue() + PI, warning).abs() >= WARNING_BAND - 1e-3);
        }
    }
}
