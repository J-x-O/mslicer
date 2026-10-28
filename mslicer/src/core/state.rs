use std::{collections::HashSet, sync::Arc, time::Instant};

use common::color::SRgb;
use egui::Vec2;
use egui_phosphor::regular::{CURSOR_CLICK, LINE_SEGMENT, TRIANGLE};
use egui_tracing::EventCollector;
use nalgebra::{Matrix4, Vector2, Vector3};
use slicer::geometry::mesh::{Mesh, MeshId};
use tools::supports::{SupportId, mesh::SupportPreset};
use wgpu::Device;

use crate::{
    core::{
        config::peripherals::Webhook,
        project::model::{Model, ModelId},
        selected::{SelectedModel, SelectedPrinter, SelectedSupport, SelectedSupports},
    },
    interface::tools::Tools,
    render::util::MeshBuffers,
    util::misc::DefaultNum,
};

// todo: organize this...
#[derive(Default)]
pub struct UiState {
    pub event_collector: EventCollector,
    pub close_requested: bool,
    pub queue_reset_ui: bool,
    pub line_support_debug: Vec<[Vector3<f32>; 2]>,
    pub tool: Tool,

    // support stuff
    pub workspace: WorkspaceHover,
    pub hovered_geometry: Option<GeometryHit>,

    // selections
    pub select_start: Option<Vec2>,
    pub selected: SelectedModel,
    pub selected_printer: SelectedPrinter,
    pub selected_supports: SelectedSupports,
    pub support_preview: Option<Mesh>,
    pub support_drag: Option<SupportDrag>,
    /// A dragged support that was let go, still hidden (with its preview
    /// showing) until the support mesh is rebuilt with it in its new place.
    pub support_settling: Option<SupportDrag>,
    /// The supports inside the selection box being drawn.
    pub boxed_supports: HashSet<SelectedSupport>,
    /// An edit of the selected supports' settings, while it runs.
    pub support_edit: Option<SupportEdit>,
    /// An edit of the raft settings is under way, and its undo step already
    /// recorded.
    pub raft_edit: bool,
    /// A short message shown over the viewport, and since when.
    pub notice: Option<(String, Instant)>,
    /// Supports the last settings edit found no route for, kept with their
    /// old settings.
    pub unrouted: Vec<SelectedSupport>,
    /// Meshes whose acceleration structures are being built.
    pub preparing: HashSet<MeshId>,
    pub selected_remap_point: Option<u8>,

    // remote send ui
    pub working_address: String,
    pub working_filename: String,
    pub remote_print_connecting: RemotePrintConnectStatus,
    pub shared_webhook: Arc<SharedPrintCompletion>,

    // slice preview
    pub preview_layer: usize,
    pub last_preview_layer: usize,
    pub preview_offset: Vector2<f32>,
    pub preview_scale: f32,
    pub layer_count: (usize, u8),

    pub preview_mode: PreviewMode,
    pub last_preview_interact: Option<Instant>,

    pub reconstruct_subsample: DefaultNum<20, u8>,

    pub anisotropic_aa: bool,

    pub tools: Tools,
    pub move_timeout: u32, // todo: having this in frames is fragile!
}

pub struct PreviewModel {
    pub mesh: MeshBuffers,
    pub faces: usize,

    pub transform: Matrix4<f32>,
    pub color: SRgb<f32>,
}

#[derive(Default, Copy, Clone, PartialEq, Eq)]
pub enum PreviewMode {
    Interactive,
    #[default]
    Static,
}

#[derive(Default, Clone, Copy, PartialEq, Eq)]
pub enum Tool {
    #[default]
    Select,
    Support,
    Orient,
}

#[derive(Default)]
pub struct SharedPrintCompletion {
    pub webhook: Webhook,
    pub alert: bool,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct GeometryHit {
    pub model: ModelId,
    pub support: bool,

    pub face: u32,
}

/// An edit of the selected supports' settings. The supports are re-routed on
/// a worker, at most one run at a time: values that come in meanwhile wait for
/// the run to finish, and only the newest is used.
pub struct SupportEdit {
    /// The shown support's settings when the edit began.
    pub origin: SupportPreset,
    /// The settings as edited now.
    pub shown: SupportPreset,
    /// A whole preset was picked, so every support takes all of `shown`.
    pub whole: bool,
    /// The edited supports as they were when the edit began.
    pub supports: Vec<EditedSupport>,
    /// A re-route is running.
    pub running: bool,
    /// `shown` changed since the running re-route started.
    pub dirty: bool,
    /// The pointer was let go; the edit ends once the last re-route is in.
    pub released: bool,
    /// Supports the newest values found no route for.
    pub unrouted: Vec<SelectedSupport>,
}

pub struct EditedSupport {
    pub model: ModelId,
    pub id: SupportId,
    pub preset: SupportPreset,
    pub contact: Vector3<f32>,
    /// The direction the tip leaves the contact.
    pub tip: Vector3<f32>,
}

impl SupportEdit {
    /// The settings `support` gets from this edit.
    pub fn preset_for(&self, support: &EditedSupport) -> SupportPreset {
        if self.whole {
            return self.shown.clone();
        }
        let mut preset = support.preset.clone();
        preset.apply_changes(&self.origin, &self.shown);
        preset
    }
}

/// A support whose contact point is being dragged.
#[derive(Debug, Copy, Clone)]
pub struct SupportDrag {
    pub model: ModelId,
    pub support: SupportId,
}

#[derive(Default)]
pub struct WorkspaceHover {
    pub is_moving: bool,
    pub aspect: f32,
    pub uv: Vector2<f32>,
}

#[derive(Default, PartialEq, Eq)]
pub enum RemotePrintConnectStatus {
    #[default]
    None,
    Connecting,
    Scanning,
}

impl PreviewModel {
    pub fn for_model(device: &Device, model: &mut Model) -> Self {
        Self {
            mesh: model.get_buffers(device).clone(),
            faces: model.mesh.face_count(),

            transform: *model.mesh.transformation_matrix(),
            color: model.color.to_srgb(),
        }
    }
}

impl WorkspaceHover {
    pub fn new(is_moving: bool, aspect: f32, uv: Vec2) -> Self {
        Self {
            is_moving,
            aspect,
            uv: Vector2::new(uv.x, uv.y),
        }
    }

    pub fn hovered(&self) -> bool {
        self.uv.x >= 0.0 && self.uv.y >= 0.0
    }
}

impl UiState {
    /// Switches tools. Support selections, drags and previews only mean
    /// something in the support tool, so leaving it drops them.
    pub fn set_tool(&mut self, tool: Tool) {
        if self.tool == Tool::Support && tool != Tool::Support {
            self.selected_supports.clear();
            self.support_drag = None;
            self.support_preview = None;
        }
        self.tool = tool;
    }

    /// Ends the settings edit, keeping what it couldn't route.
    pub fn end_support_edit(&mut self) {
        if let Some(edit) = self.support_edit.take() {
            self.unrouted = edit.unrouted;
        }
    }
}

impl PreviewMode {
    pub fn other(&self) -> Self {
        match self {
            PreviewMode::Interactive => PreviewMode::Static,
            PreviewMode::Static => PreviewMode::Interactive,
        }
    }
}

impl Tool {
    pub const ALL: [Self; 3] = [Self::Select, Self::Support, Self::Orient];

    pub fn icon(&self) -> &str {
        match self {
            Tool::Select => CURSOR_CLICK,
            Tool::Support => LINE_SEGMENT,
            Tool::Orient => TRIANGLE,
        }
    }

    pub fn name(&self) -> &str {
        match self {
            Tool::Select => "Select (Default)",
            Tool::Support => "Support Placement",
            Tool::Orient => "Orient to Face",
        }
    }
}
