//! The island check: where models start in mid-air, and which of those spots
//! no support holds. It only exists while the user runs it from the Islands
//! panel, and nothing about it is saved.

use std::collections::HashSet;

use common::units::Milimeter;
use nalgebra::Vector3;
use slicer::geometry::mesh::{Mesh, MeshId};
use tools::{
    islands::{Cover, Detection, Island, OVERLAY_GROW, Planes, coverage, overlay_mesh},
    supports::auto::route_hand,
};

use crate::{
    core::{
        App,
        history::ModelAction,
        project::{model::ModelId, supports::Pose},
        stack,
    },
    interface::panels::Tab,
    task::{BuildAccelerationStructures, DetectIslands},
};

/// How far apart the cutting planes are: every layer, every second or every
/// fourth. Finer than a layer would find nothing the printer can resolve.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Accuracy {
    #[default]
    Layer,
    Half,
    Quarter,
}

impl Accuracy {
    pub const ALL: [Self; 3] = [Self::Layer, Self::Half, Self::Quarter];

    pub fn step(self) -> u32 {
        match self {
            Self::Layer => 1,
            Self::Half => 2,
            Self::Quarter => 4,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Layer => "Layer",
            Self::Half => "Half",
            Self::Quarter => "Quarter",
        }
    }
}

/// Check settings, kept for the session.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct IslandSettings {
    pub accuracy: Accuracy,
    /// Islands smaller than this (mm²) are left out.
    pub min_area: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct IslandRef {
    pub model: ModelId,
    pub index: usize,
}

/// A running check.
#[derive(Default)]
pub struct IslandCheck {
    pub models: Vec<CheckedModel>,
    /// Where the stepping is in [`IslandCheck::walk`], once it started.
    pub cursor: Option<usize>,
    /// The island under the pointer, in the support tool.
    pub hovered: Option<IslandRef>,
    /// An island no support could be routed to.
    pub unroutable: Option<IslandRef>,
    /// Bumped whenever what the overlay shows changes.
    revision: u64,
}

pub struct CheckedModel {
    pub model: ModelId,
    pub result: Option<Checked>,
    /// The pose a detection is running for.
    pub running: Option<Pose>,
    /// Islands the user dismissed; kept until the next detection.
    pub ignored: HashSet<usize>,
}

pub struct Checked {
    pub pose: Pose,
    /// The print mesh it was found on: a new cavity calls for a new check.
    pub source: MeshId,
    pub detection: Detection,
    pub cover: Vec<Cover>,
}

/// An island as the panel lists it.
pub struct Listed<'a> {
    pub at: IslandRef,
    pub island: &'a Island,
    pub cover: Cover,
    pub ignored: bool,
}

impl IslandCheck {
    pub fn new(models: impl IntoIterator<Item = ModelId>) -> Self {
        Self {
            models: (models.into_iter())
                .map(|model| CheckedModel {
                    model,
                    result: None,
                    running: None,
                    ignored: HashSet::new(),
                })
                .collect(),
            ..Default::default()
        }
    }

    pub fn revision(&self) -> u64 {
        self.revision
    }

    fn touch(&mut self) {
        self.revision += 1;
    }

    /// Whether any model is still being detected.
    pub fn busy(&self) -> bool {
        self.models.iter().any(|m| m.result.is_none())
    }

    /// Planes that had open contours, per model with any.
    pub fn open_planes(&self) -> impl Iterator<Item = (ModelId, usize)> + '_ {
        (self.models.iter())
            .filter_map(|m| Some((m.model, m.result.as_ref()?.detection.open_planes)))
            .filter(|(_, n)| *n > 0)
    }

    pub fn island(&self, at: IslandRef) -> Option<&Island> {
        let checked = self.models.iter().find(|m| m.model == at.model)?;
        checked.result.as_ref()?.detection.islands.get(at.index)
    }

    fn spacing(&self, model: ModelId) -> Option<f32> {
        let checked = self.models.iter().find(|m| m.model == model)?;
        Some(checked.result.as_ref()?.detection.spacing)
    }

    /// The islands the panel lists: off the plate and at least the minimum
    /// area, by model, then bottom up.
    pub fn listed(&self, settings: &IslandSettings) -> Vec<Listed<'_>> {
        let mut out = Vec::new();
        for checked in &self.models {
            let Some(result) = &checked.result else {
                continue;
            };
            let start = out.len();
            for (index, (island, &cover)) in
                (result.detection.islands.iter().zip(&result.cover)).enumerate()
            {
                if cover == Cover::OnPlate || island.area < settings.min_area {
                    continue;
                }
                out.push(Listed {
                    at: IslandRef {
                        model: checked.model,
                        index,
                    },
                    island,
                    cover,
                    ignored: checked.ignored.contains(&index),
                });
            }
            out[start..].sort_by(|a, b| {
                (a.island.z.total_cmp(&b.island.z)).then(a.island.min.x.total_cmp(&b.island.min.x))
            });
        }
        out
    }

    /// The unsupported islands, in the order they are stepped through.
    pub fn walk(&self, settings: &IslandSettings) -> Vec<IslandRef> {
        (self.listed(settings).into_iter())
            .filter(|l| l.cover == Cover::Unsupported && !l.ignored)
            .map(|l| l.at)
            .collect()
    }

    /// The island the stepping is on: its 0-based position, the walk's
    /// length and the island.
    pub fn current(&self, settings: &IslandSettings) -> Option<(usize, usize, IslandRef)> {
        let walk = self.walk(settings);
        let k = self.cursor? % walk.len().max(1);
        walk.get(k).map(|&at| (k, walk.len(), at))
    }

    /// Takes in a finished detection, if the model is still where it was
    /// detected and the plane spacing still applies.
    pub fn finished(
        &mut self,
        model: ModelId,
        (pose, source): (Pose, MeshId),
        detection: Detection,
        now: (Pose, MeshId),
        spacing: f32,
    ) {
        let Some(checked) = self.models.iter_mut().find(|m| m.model == model) else {
            return;
        };
        if checked.running == Some(pose) {
            checked.running = None;
        }
        if (pose, source) != now || detection.spacing != spacing {
            return;
        }
        let cover = vec![Cover::Unsupported; detection.islands.len()];
        checked.result = Some(Checked {
            pose,
            source,
            detection,
            cover,
        });
        checked.ignored.clear();
        self.touch();
    }

    /// The overlay meshes: the islands still to fix, and the highlighted one
    /// (hovered, or the one the stepping is on).
    pub fn overlay(&self, settings: &IslandSettings) -> (Mesh, Mesh) {
        let highlight = self.hovered.or(self.current(settings).map(|(.., at)| at));
        let mut plain = Vec::new();
        let mut lit = Vec::new();
        for listed in self.listed(settings) {
            if listed.cover != Cover::Unsupported || listed.ignored {
                continue;
            }
            let spacing = self.spacing(listed.at.model).unwrap_or_default();
            let into = if Some(listed.at) == highlight {
                &mut lit
            } else {
                &mut plain
            };
            into.push((listed.island, spacing));
        }
        let mesh = |islands: &[(&Island, f32)]| {
            let spacing = islands.first().map_or(0.0, |(_, s)| *s);
            overlay_mesh(islands.iter().map(|(i, _)| *i), spacing)
        };
        (mesh(&plain), mesh(&lit))
    }
}

/// The plane spacing for the current settings.
pub fn spacing(layer_height: f32, settings: &IslandSettings) -> f32 {
    layer_height * settings.accuracy.step() as f32
}

/// Starts a check of `models`.
pub fn start(app: &mut App, models: Vec<ModelId>) {
    app.state.island_check = Some(IslandCheck::new(models));
}

/// Ends the check: the overlay goes and the results are dropped.
pub fn end(app: &mut App) {
    app.state.island_check = None;
    app.state.support_preview = None;
}

/// Keeps a running check current. Runs once a frame.
pub fn update(app: &mut App) {
    let Some(mut check) = app.state.island_check.take() else {
        return;
    };
    // The check lives in its panel (the Islands tab, or the Supports
    // operation in Properties): closing it ends the check.
    if !app.panels.is_open(Tab::Islands) && !app.panels.is_open(Tab::Properties) {
        app.state.support_preview = None;
        return;
    }

    let layer = app.project.slice_config.slice_height.get::<Milimeter>();
    let settings = app.state.island_settings;
    let spacing = spacing(layer, &settings);
    let models = &app.project.models;
    check
        .models
        .retain(|c| models.iter().any(|m| m.id == c.model));

    let mut changed = false;
    for checked in &mut check.models {
        let Some(model) = models.iter().find(|m| m.id == checked.model) else {
            continue;
        };
        let now = Pose::of(&model.mesh);
        // Islands are found on the mesh as it prints, inner walls included.
        let print = model.print_mesh();

        if checked
            .result
            .as_ref()
            .is_some_and(|r| r.source != print.mesh_id())
        {
            checked.result = None;
            checked.ignored.clear();
            changed = true;
        }
        if let Some(result) = &mut checked.result {
            match result.pose.motion_to(&now) {
                Some(motion) if result.detection.spacing == spacing => {
                    if result.pose != now {
                        result.detection.transform(&motion);
                        result.pose = now;
                        changed = true;
                    }
                }
                // Lifted, tilted, scaled or a new accuracy: detect again.
                _ => {
                    checked.result = None;
                    checked.ignored.clear();
                    changed = true;
                }
            }
        }

        match &mut checked.result {
            Some(result) => {
                let cover = coverage(&result.detection, &model.supports.contacts());
                if cover != result.cover {
                    result.cover = cover;
                    changed = true;
                }
            }
            // One detection per model at a time; a stale one is dropped when
            // it finishes and the next frame starts a fresh one.
            // A hollowed model's print mesh gets its own hit-testing
            // structure, built by the detection.
            None if checked.running.is_none() && print.mesh_id() != model.mesh.mesh_id() => {
                checked.running = Some(now);
                let planes = Planes::layers(layer, settings.accuracy.step());
                app.tasks
                    .add(DetectIslands::new(model.id, print, None, planes, now));
            }
            None if checked.running.is_none() => match &model.bvh {
                Some(bvh) => {
                    checked.running = Some(now);
                    let planes = Planes::layers(layer, settings.accuracy.step());
                    let bvh = Some(bvh.clone());
                    app.tasks
                        .add(DetectIslands::new(model.id, print, bvh, planes, now));
                }
                None => {
                    if let Some(task) = BuildAccelerationStructures::request(&mut app.state, model)
                    {
                        app.tasks.add(task);
                    }
                }
            },
            None => {}
        }
    }

    if changed {
        check.touch();
    }
    app.state.island_check = Some(check);
}

/// Steps to the next (or previous) unsupported island and frames it.
pub fn step(app: &mut App, forward: bool) {
    let settings = app.state.island_settings;
    let Some(check) = &mut app.state.island_check else {
        return;
    };
    let len = check.walk(&settings).len();
    if len == 0 {
        return;
    }
    let k = match check.cursor {
        None if forward => 0,
        None => len - 1,
        Some(k) if forward => (k % len + 1) % len,
        Some(k) => (k % len + len - 1) % len,
    };
    check.cursor = Some(k);
    check.touch();
    if let Some((.., at)) = check.current(&settings) {
        frame(app, at);
    }
}

/// Selects an island from the list: steps to it if it is in the walk, and
/// frames it either way.
pub fn show(app: &mut App, at: IslandRef) {
    let settings = app.state.island_settings;
    let Some(check) = &mut app.state.island_check else {
        return;
    };
    if let Some(k) = check.walk(&settings).iter().position(|&x| x == at) {
        check.cursor = Some(k);
        check.touch();
    }
    frame(app, at);
}

/// Points the camera at an island from a little below, so its underside
/// shows, and selects its model so the support tool can place supports on it.
fn frame(app: &mut App, at: IslandRef) {
    app.state.selected.select_model(at.model);
    let Some(island) = app.state.island_check.as_ref().and_then(|c| c.island(at)) else {
        return;
    };
    let half = ((island.max - island.min).norm() / 2.0).max(3.0);
    let centre = island.anchor;
    app.camera.frame(
        centre - Vector3::repeat(half),
        centre + Vector3::repeat(half),
    );
    app.camera.angle.y = app.camera.angle.y.min(-0.35);
    app.state.camera_focus = Some(Default::default());
}

/// Dismisses an island until the next detection.
pub fn ignore(app: &mut App, at: IslandRef) {
    let Some(check) = &mut app.state.island_check else {
        return;
    };
    if let Some(checked) = check.models.iter_mut().find(|m| m.model == at.model) {
        checked.ignored.insert(at.index);
        check.touch();
    }
}

/// Adds a support at an island's underside, as one undo step. Returns false
/// when no support can be routed there.
pub fn support(app: &mut App, at: IslandRef) -> bool {
    support_many(app, &[at]) == 1
}

/// Supports every island in the walk; one undo step per model.
pub fn support_all(app: &mut App) -> usize {
    let settings = app.state.island_settings;
    let walk = (app.state.island_check.as_ref()).map_or(Vec::new(), |c| c.walk(&settings));
    support_many(app, &walk)
}

fn support_many(app: &mut App, islands: &[IslandRef]) -> usize {
    let Some(check) = &mut app.state.island_check else {
        return 0;
    };
    let config = &app.project.support_config.preset;
    let mut tracked = HashSet::new();
    let mut added = 0;
    for &at in islands {
        let Some(island) = check.island(at) else {
            continue;
        };
        let contacts = island.contacts.clone();
        let Some(model) = app.project.models.iter_mut().find(|m| m.id == at.model) else {
            continue;
        };
        let Some(bvh) = model.bvh.clone() else {
            continue;
        };
        // The first candidate a support can be routed from.
        let routed = (contacts.iter())
            .find_map(|&(p, n)| route_hand(&model.mesh, &bvh, config, p, n, false));
        let Some(routed) = routed else {
            check.unroutable = Some(at);
            continue;
        };
        if !stack::supports_editable(model) {
            continue;
        }
        if tracked.insert(at.model) {
            let action = ModelAction::Supports(model.supports.snapshot());
            let action = stack::supports_action(model, action);
            (app.history).track_model(at.model, action);
        }
        model
            .supports
            .add_manual(config, &routed, &model.mesh, &bvh);
        added += 1;
    }
    if added > 0 {
        check.unroutable = check.unroutable.filter(|at| !islands.contains(at));
        check.touch();
    }
    added
}

/// The unsupported island at the point where the pointer meets a checked
/// model. The overlay doesn't catch the pointer: supports go where it points,
/// and the island under it only lights up.
fn hovered(app: &App) -> Option<IslandRef> {
    /// How far off an island's outline and layer band the hit may be.
    const SLACK: f32 = 0.3;
    let check = app.state.island_check.as_ref()?;
    let (pos, dir) = app.hovered_ray()?;

    let hit = (check.models.iter())
        .filter_map(|c| app.project.models.iter().find(|m| m.id == c.model))
        .filter_map(|model| {
            let hit = model.visible_hit(pos, dir, app.state.cut_height)?;
            let point = model.mesh.transform(&hit.position);
            Some(((point - pos).norm(), model.id, point))
        })
        .min_by(|a, b| a.0.total_cmp(&b.0));
    let (_, model, point) = hit?;
    let spacing = check.spacing(model)?;

    (check.listed(&app.state.island_settings).into_iter())
        .filter(|l| l.at.model == model && l.cover == Cover::Unsupported && !l.ignored)
        .filter(|l| point.z >= l.island.z - spacing - SLACK && point.z <= l.island.z + SLACK)
        .find(|l| l.island.near(point.xy(), OVERLAY_GROW + SLACK))
        .map(|l| l.at)
}

/// Tracks the island under the pointer, in the support tool.
pub fn track_hover(app: &mut App) {
    let at = hovered(app);
    if let Some(check) = &mut app.state.island_check
        && check.hovered != at
    {
        check.hovered = at;
        check.touch();
    }
}

/// Clears the hover when the pointer is not over the viewport in the support
/// tool.
pub fn clear_hover(app: &mut App) {
    if let Some(check) = &mut app.state.island_check
        && check.hovered.take().is_some()
    {
        check.touch();
    }
}
