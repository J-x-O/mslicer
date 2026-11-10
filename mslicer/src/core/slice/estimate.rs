//! What is known about a print without slicing it: cheap, always available.

use common::units::{CubicMilimeters, Milimeter, Seconds};

use crate::core::project::{Project, model::MeshWarnings};

/// Slice-free facts about the print. The print time is an estimate until a
/// full slice exists: exact values depend on variable layer height.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Estimate {
    /// The number of layers a full slice will have (before variable layer
    /// height merges any).
    pub layers: u32,
    pub print_time: Seconds,
    /// Resin for the visible models (their mesh volume less their cavities),
    /// supports not included. `None` until every mesh's volume is known.
    pub volume: Option<CubicMilimeters>,
    /// Visible models reaching outside the build volume.
    pub out_of_bounds: usize,
}

impl Estimate {
    pub fn of(project: &Project) -> Self {
        let config = &project.slice_config;
        let height = config.slice_height.get::<Milimeter>();
        let visible = || project.models.iter().filter(|m| !m.hidden);

        // As `Slicer::new` counts them: the models are sliced half a layer
        // lower (see `platform_models`), and the platform caps the count.
        let top = visible().map(|m| m.bounds.max.z).fold(0_f32, f32::max);
        let max_layers = (config.platform_size.z / config.slice_height).ceil() as u32;
        let layers = (((top - height / 2.0) / height).ceil().max(0.0) as u32).min(max_layers);

        let volume = visible().try_fold(CubicMilimeters::new(0.0), |total, model| {
            let cavity = (model.hollow.cavity(&model.mesh)).map_or(0.0, |c| {
                let scale = model.mesh.scale();
                c.volume() * scale.x * scale.y * scale.z
            });
            Some(total + CubicMilimeters::new(model.volume()?.raw() - cavity))
        });

        Self {
            layers,
            print_time: config.print_time(layers),
            volume,
            out_of_bounds: visible()
                .filter(|m| m.warnings.contains(MeshWarnings::OutOfBounds))
                .count(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        core::slice::runner::SliceOutput,
        regression::{load_prepared, project_path, slice},
    };

    #[test]
    fn layer_count_and_time_match_a_full_slice() {
        for name in ["teapot", "t_shape", "hollowed", "supported"] {
            let mut project = load_prepared(&project_path(name));
            let estimate = Estimate::of(&project);
            let SliceOutput::Raster(layers) = slice(&mut project) else {
                unreachable!()
            };
            assert_eq!(estimate.layers as usize, layers.len(), "{name}");
            let exact = common::slice::print_time(layers.iter()).raw();
            let rough = estimate.print_time.raw();
            assert!(
                (exact - rough).abs() < exact * 0.03,
                "{name}: {rough} vs {exact}"
            );
        }
    }
}
