use chrono::{DateTime, Duration, Utc};
use common::units::Milimeters;
use egui::Theme;
use egui_dock::Tree;
use serde::{Deserialize, Serialize};

use crate::interface::panels::Tab;

pub const B_PER_MIB: usize = 0x100000;

#[derive(Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct UiConfig {
    pub theme: Theme,
    pub panels: Option<Tree<Tab>>,
    pub about: bool,
    pub tasks: bool,
    pub history_max_mesh_size: u32, // in MiB
    pub quick_layout_spacing: Milimeters,
    pub hover_overlay: bool,
    pub hover_overlay_detail: bool,
    /// Build every model's acceleration structures as soon as it is loaded.
    pub auto_prepare: bool,
    /// The colour of the selection box and the Orient tool's face highlight,
    /// in sRGB.
    pub accent_color: [f32; 3],
    /// The colour of warnings, like a model outside the build volume, in
    /// sRGB. Model and support colours keep clear of its hue.
    pub warning_color: [f32; 3],

    pub update_check: UpdateCheckFrequency,
    pub last_update_check: Option<DateTime<Utc>>,
    pub ignore_update: Option<String>,
}

#[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum UpdateCheckFrequency {
    Never,
    EveryLaunch,
    Daily,
    Weekly,
}

// todo: dropdown enum macro?
impl UpdateCheckFrequency {
    pub const ALL: [Self; 4] = [Self::Never, Self::EveryLaunch, Self::Daily, Self::Weekly];

    pub fn name(&self) -> &'static str {
        match self {
            UpdateCheckFrequency::Never => "Never",
            UpdateCheckFrequency::EveryLaunch => "Every Launch",
            UpdateCheckFrequency::Daily => "Daily",
            UpdateCheckFrequency::Weekly => "Weekly",
        }
    }

    pub fn as_duration(&self) -> Option<Duration> {
        Some(match self {
            UpdateCheckFrequency::Never => return None,
            UpdateCheckFrequency::EveryLaunch => Duration::zero(),
            UpdateCheckFrequency::Daily => Duration::days(1),
            UpdateCheckFrequency::Weekly => Duration::days(7),
        })
    }

    pub fn enabled(&self) -> bool {
        !matches!(self, UpdateCheckFrequency::Never)
    }

    pub fn toggle(&mut self) {
        *self = match self {
            UpdateCheckFrequency::Never => UpdateCheckFrequency::EveryLaunch,
            UpdateCheckFrequency::EveryLaunch
            | UpdateCheckFrequency::Daily
            | UpdateCheckFrequency::Weekly => UpdateCheckFrequency::Never,
        };
    }
}

impl Default for UiConfig {
    fn default() -> Self {
        Self {
            theme: Theme::Dark,
            panels: None,
            about: true,
            tasks: true,
            history_max_mesh_size: 20,
            quick_layout_spacing: Milimeters::new(2.0),
            hover_overlay: true,
            hover_overlay_detail: false,
            auto_prepare: true,
            accent_color: [0.2, 0.5, 1.0],
            warning_color: [1.0, 0.1, 0.1],

            update_check: UpdateCheckFrequency::EveryLaunch,
            last_update_check: None,
            ignore_update: None,
        }
    }
}
