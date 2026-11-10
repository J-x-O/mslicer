//! Issues: every semantic warning about the project, in one live list. Each
//! has a source (a model and the operation it comes from, or the print) and
//! a place to jump to.
//!
//! The list is derived, not stored: it is collected from the project's state
//! when shown, so it can't go out of date and nothing has to remember to
//! clear an issue once it is fixed.

use crate::{
    core::{
        App,
        focus::{self, Focus, PrintNode},
        history::ModelAction,
        hollow::HoleRef,
        islands::{IslandRef, IslandSettings},
        project::{
            model::{MeshWarnings, Model, ModelId},
            stack::{Op, OperationId},
        },
        selected::SelectedSupport,
        slice::result::GenericSliceData,
        state::Tool,
    },
    interface::panels::Tab,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    /// Will likely ruin the print or leave part of it out.
    Error,
    /// Worth a look.
    Warning,
}

/// What an issue is about.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Kind {
    DefectiveMesh,
    OutOfBounds,
    UnroutedSupports,
    /// Supports that couldn't follow the model to its new orientation and
    /// kept their old route.
    StrandedSupports,
    UnsupportedIslands,
    SealedVoids,
    FailedHole,
    /// A geometry operation after Hollow may close a hole.
    BlockedHole,
    /// A hand-placed hole sits above the lowest point of its void in the
    /// current orientation: resin stays trapped below it.
    HoleNotDraining,
    /// A hole is drilled where the model touches the plate, so the model no
    /// longer stands on it.
    HoleAtPlateContact,
    OrientationOutdated,
    SlicingDefects,
    ResolutionMismatch,
    /// A frozen operation no longer fits the model (e.g. frozen supports
    /// after re-orienting).
    FrozenOutdated,
}

/// Where an issue comes from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    Model {
        model: ModelId,
        operation: Option<OperationId>,
    },
    Print,
}

/// Where clicking an issue takes you.
#[derive(Clone, Debug, PartialEq)]
pub enum Jump {
    /// Selects the model and shows a panel.
    Model(ModelId, Option<Tab>),
    Hole(HoleRef),
    Island(IslandRef),
    Supports(Vec<SelectedSupport>),
    /// Shows a panel.
    Tab(Tab),
}

#[derive(Clone, Debug, PartialEq)]
pub struct Issue {
    pub kind: Kind,
    pub severity: Severity,
    pub source: Source,
    pub text: String,
    pub jump: Jump,
}

fn plural(n: usize, one: &str, many: &str) -> String {
    match n {
        1 => format!("1 {one}"),
        n => format!("{n} {many}"),
    }
}

fn op_id(model: &Model, pick: fn(&Op) -> bool) -> Option<OperationId> {
    model.stack.ops().iter().find(|o| pick(&o.op)).map(|o| o.id)
}

/// The issues of one model, from its own state.
pub fn model_issues(model: &Model, platform_z: f32) -> Vec<Issue> {
    let mut out = Vec::new();
    let mut push = |kind, severity, operation, text: String, jump| {
        out.push(Issue {
            kind,
            severity,
            source: Source::Model {
                model: model.id,
                operation,
            },
            text,
            jump,
        })
    };
    let id = model.id;
    let hollow_op = op_id(model, |op| matches!(op, Op::Hollow(_)));
    let supports_op = op_id(model, |op| matches!(op, Op::Supports(_)));

    if model.warnings.contains(MeshWarnings::Defective) {
        push(
            Kind::DefectiveMesh,
            Severity::Warning,
            None,
            "The mesh is defective and may slice wrong. Add a Mesh Repair operation.".into(),
            Jump::Model(id, Some(Tab::Models)),
        );
    }
    if model.warnings.contains(MeshWarnings::OutOfBounds) {
        push(
            Kind::OutOfBounds,
            Severity::Error,
            None,
            "Reaches outside the build volume and will be cut off.".into(),
            Jump::Model(id, Some(Tab::Models)),
        );
    }

    if model.is_hollowed() {
        let sealed = model.hollow.sealed_parts(&model.mesh);
        if sealed > 0 {
            push(
                Kind::SealedVoids,
                Severity::Error,
                hollow_op,
                format!(
                    "{} no drain hole: resin stays trapped.",
                    plural(sealed, "void has", "voids have")
                ),
                Jump::Model(id, Some(Tab::Hollow)),
            );
        }
        for index in 0..model.hollow.spec.holes.len() {
            if model.hollow.failed(&model.mesh, index) {
                push(
                    Kind::FailedHole,
                    Severity::Warning,
                    hollow_op,
                    format!("Hole {} couldn't be drilled and is left out.", index + 1),
                    Jump::Hole(HoleRef { model: id, index }),
                );
            }
        }
        for warning in model.hole_warnings() {
            let (kind, severity, text) = match warning.kind {
                HoleWarning::Blocked => (
                    Kind::BlockedHole,
                    Severity::Warning,
                    format!(
                        "Hole {} may be closed by the Mesh Repair after Hollow.",
                        warning.hole + 1
                    ),
                ),
                HoleWarning::NotDraining { void } => (
                    Kind::HoleNotDraining,
                    Severity::Warning,
                    format!(
                        "Void {} no longer drains: its hole {} is above the resin line.",
                        void + 1,
                        warning.hole + 1
                    ),
                ),
                HoleWarning::PlateContact => (
                    Kind::HoleAtPlateContact,
                    Severity::Error,
                    format!(
                        "Hole {} is drilled where the model touches the plate: the model won't stand on it. Lift the model onto supports or move the hole.",
                        warning.hole + 1
                    ),
                ),
            };
            let jump = match warning.hole < model.hollow.spec.holes.len() {
                true => Jump::Hole(HoleRef {
                    model: id,
                    index: warning.hole,
                }),
                false => Jump::Model(id, Some(Tab::Hollow)),
            };
            push(kind, severity, hollow_op, text, jump);
        }
    }

    if let Some(stranded) = model.supports.stranded().filter(|s| !s.is_empty()) {
        push(
            Kind::StrandedSupports,
            Severity::Error,
            supports_op,
            format!(
                "{} found no route after the model changed and kept the old one.",
                plural(stranded.len(), "support", "supports")
            ),
            Jump::Supports(
                stranded
                    .iter()
                    .map(|s| SelectedSupport::new(id, *s))
                    .collect(),
            ),
        );
    }
    if model.supports.frozen_outdated() {
        push(
            Kind::FrozenOutdated,
            Severity::Warning,
            supports_op,
            "The supports are frozen and no longer fit the model's orientation. Thaw them to re-route.".into(),
            Jump::Model(id, Some(Tab::Supports)),
        );
    }

    let (orient, params) = model.stack.orient_row();
    if params.auto.is_some() && params.outdated {
        push(
            Kind::OrientationOutdated,
            Severity::Warning,
            Some(orient),
            "The orientation may be outdated: something it was chosen for changed. Re-run auto-orient.".into(),
            Jump::Model(id, Some(Tab::Models)),
        );
    }
    let _ = platform_z;
    out
}

/// What a hole is warned about.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum HoleWarning {
    Blocked,
    NotDraining { void: usize },
    PlateContact,
}

/// A warning about hole `hole` (an index into the manual holes, then the
/// auto holes after them).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HoleIssue {
    pub hole: usize,
    pub kind: HoleWarning,
}

/// Every issue of the project, models first (in their order), then the
/// print.
pub fn collect(app: &App) -> Vec<Issue> {
    let project = &app.project;
    let platform_z = project.slice_config.platform_size.z.raw();
    let mut out: Vec<Issue> = (project.models.iter())
        .filter(|m| !m.hidden)
        .flat_map(|m| model_issues(m, platform_z))
        .collect();

    // Supports the last settings edit found no route for.
    let unrouted = match &app.state.support_edit {
        Some(edit) => &edit.unrouted,
        None => &app.state.unrouted,
    };
    if !unrouted.is_empty() {
        let model = unrouted[0].model;
        out.push(Issue {
            kind: Kind::UnroutedSupports,
            severity: Severity::Warning,
            source: Source::Model {
                model,
                operation: None,
            },
            text: format!(
                "{} found no route with the new settings and kept the old ones.",
                plural(unrouted.len(), "support", "supports")
            ),
            jump: Jump::Supports(unrouted.clone()),
        });
    }

    // The island check, while it runs.
    if let Some(check) = &app.state.island_check {
        let settings: &IslandSettings = &app.state.island_settings;
        let walk = check.walk(settings);
        for model in project.models.iter() {
            let open: Vec<_> = walk.iter().filter(|at| at.model == model.id).collect();
            if let Some(first) = open.first() {
                out.push(Issue {
                    kind: Kind::UnsupportedIslands,
                    severity: Severity::Error,
                    source: Source::Model {
                        model: model.id,
                        operation: None,
                    },
                    text: format!(
                        "{} with no support.",
                        plural(open.len(), "island", "islands")
                    ),
                    jump: Jump::Island(**first),
                });
            }
        }
    }

    // The last slice.
    if let Some(operation) = &app.slice_operation
        && let Some(result) = operation.result().as_ref()
        && let GenericSliceData::Raster { data, .. } = result.slice_data()
    {
        let layers = data.iter().filter(|l| l.defects > 0).count();
        if layers > 0 {
            out.push(Issue {
                kind: Kind::SlicingDefects,
                severity: Severity::Warning,
                source: Source::Print,
                text: format!(
                    "Slicing defects on {}. Try Mesh Repair and slice again.",
                    plural(layers, "layer", "layers")
                ),
                jump: Jump::Tab(Tab::Sliced),
            });
        }
    }

    if app.state.tools.sliced_diff_mismatch() {
        out.push(Issue {
            kind: Kind::ResolutionMismatch,
            severity: Severity::Warning,
            source: Source::Print,
            text: "Sliced Diff: the layer resolutions of the two files don't match.".into(),
            jump: Jump::Tab(Tab::Sliced),
        });
    }
    out
}

/// What can be done about an issue right from the list, beside going there.
#[derive(Clone, Debug, PartialEq)]
pub enum Fix {
    /// Removes the supports (one undo step per model).
    RemoveSupports(Vec<SelectedSupport>),
}

impl Fix {
    pub fn label(&self) -> &'static str {
        match self {
            Fix::RemoveSupports(_) => "Remove all",
        }
    }

    pub fn tip(&self) -> &'static str {
        match self {
            Fix::RemoveSupports(_) => "Deletes these supports. Undo brings them back.",
        }
    }
}

impl Issue {
    pub fn fix(&self) -> Option<Fix> {
        match (&self.kind, &self.jump) {
            (Kind::StrandedSupports, Jump::Supports(supports)) => {
                Some(Fix::RemoveSupports(supports.clone()))
            }
            _ => None,
        }
    }
}

/// Applies a fix.
pub fn apply(app: &mut App, fix: &Fix) {
    match fix {
        Fix::RemoveSupports(supports) => {
            let mut by_model = std::collections::HashMap::<_, std::collections::HashSet<_>>::new();
            for s in supports {
                by_model.entry(s.model).or_default().insert(s.support);
            }
            (app.state.selected_supports)
                .retain(|model, support| !supports.contains(&SelectedSupport::new(model, support)));
            for (id, ids) in by_model {
                if let Some(model) = app.project.models.iter_mut().find(|m| m.id == id) {
                    let action = ModelAction::Supports(model.supports.snapshot());
                    app.history.track_model(id, action);
                    model.supports.remove(&ids);
                }
            }
        }
    }
}

/// Takes the user to where an issue is.
pub fn jump(app: &mut App, jump: &Jump) {
    match jump {
        Jump::Model(id, tab) => {
            app.state.selected.select_model(*id);
            if let Some(tab) = tab {
                reveal(app, *tab, Some(*id));
            }
        }
        Jump::Hole(hole) => {
            app.state.set_tool(Tool::Holes);
            reveal(app, Tab::Hollow, Some(hole.model));
            crate::core::hollow::focus(app, hole.model, hole.index);
            app.state.selected_holes = vec![*hole];
        }
        Jump::Island(at) => {
            reveal(app, Tab::Islands, Some(at.model));
            crate::core::islands::show(app, *at);
        }
        Jump::Supports(supports) => {
            app.state.set_tool(Tool::Support);
            app.state.selected_supports.clear();
            for s in supports {
                app.state.selected.select_model(s.model);
                app.state
                    .selected_supports
                    .select_support(s.model, s.support);
            }
            if supports.iter().all(|s| s.model == supports[0].model) {
                reveal(app, Tab::Supports, supports.first().map(|s| s.model));
            } else {
                app.panels.focus_tab(Tab::Supports);
            }
        }
        Jump::Tab(tab) => reveal(app, *tab, None),
    }
}

/// Shows where `tab` points: the old panel when it is open, otherwise the
/// matching operation of `model` in Properties (in the new layout).
fn reveal(app: &mut App, tab: Tab, model: Option<ModelId>) {
    if app.panels.is_open(tab) || !app.panels.is_open(Tab::Properties) {
        return app.panels.focus_tab(tab);
    }
    app.panels.focus_tab(Tab::Properties);
    let pick: fn(&Op) -> bool = match tab {
        Tab::Hollow => |op| matches!(op, Op::Hollow(_)),
        Tab::Supports | Tab::Islands => |op| matches!(op, Op::Supports(_)),
        Tab::SliceConfig => return focus::focus(app, Focus::Print(PrintNode::Profile)),
        Tab::Sliced => return focus::focus(app, Focus::Print(PrintNode::Outputs)),
        _ => return,
    };
    let Some(id) = model else { return };
    let found = (app.project.models.iter())
        .find(|m| m.id == id)
        .and_then(|m| m.stack.ops().iter().find(|o| pick(&o.op)).map(|o| o.id));
    if let Some(op) = found {
        app.state.selected.clear();
        focus::focus_operation(app, id, op);
    }
}

#[cfg(test)]
mod tests {
    use std::{f32::consts::PI, sync::Arc};

    use nalgebra::Vector3;

    use super::*;
    use crate::{
        core::project::{Project, stack::HollowOp},
        regression::{load_prepared, project_path},
    };

    fn kinds(project: &Project) -> Vec<Kind> {
        let z = project.slice_config.platform_size.z.raw();
        (project.models.iter())
            .flat_map(|m| model_issues(m, z))
            .map(|i| i.kind)
            .collect()
    }

    /// Turns the model upside down, standing on the plate again.
    fn flip(project: &mut Project) {
        let platform = project.slice_config.platform_size;
        let model = &mut project.models[0];
        model.set_rotation(&platform, Vector3::x() * PI);
        model.align_to_bed();
        model.update_oob(&platform);
    }

    #[test]
    fn a_drain_at_the_plate_contact_is_an_error() {
        // The hollowed sphere is drained at the spot it stands on.
        let project = load_prepared(&project_path("hollowed"));
        assert_eq!(kinds(&project), [Kind::HoleAtPlateContact]);
    }

    #[test]
    fn a_hand_placed_hole_above_the_resin_line_is_reported_not_moved() {
        let mut project = load_prepared(&project_path("hollowed"));
        let holes = project.models[0].hollow.spec.holes.clone();
        flip(&mut project);
        let model = &project.models[0];
        // The cavity is reused (a turn doesn't change it), the holes stay
        // where they were on the model.
        assert!(model.hollow.cavity(&model.mesh).is_some());
        assert_eq!(model.hollow.spec.holes, holes);
        let found = kinds(&project);
        assert!(found.contains(&Kind::HoleNotDraining), "{found:?}");
        assert!(!found.contains(&Kind::HoleAtPlateContact));
    }

    #[test]
    fn auto_holes_follow_the_orientation_and_drain() {
        let mut project = load_prepared(&project_path("hollowed"));
        let platform = project.slice_config.platform_size;
        {
            let model = &mut project.models[0];
            model.hollow.spec.holes.clear();
            model.hollow.auto_rule = Some(HollowOp::new().diameter);
            let lift = model.mesh.position() + Vector3::z() * 3.0;
            model.set_position(&platform, lift);
        }
        let model = &project.models[0];
        let before = model.hollow.auto_holes(&model.mesh);
        assert_eq!(before.len(), 1);
        // Moving keeps them (same node result).
        let mut moved = model.mesh.clone();
        moved.set_position(moved.position() + Vector3::x());
        assert!(Arc::ptr_eq(&before, &model.hollow.auto_holes(&moved)));

        flip(&mut project);
        let model = &mut project.models[0];
        let lift = model.mesh.position() + Vector3::z() * 3.0;
        model.set_position(&platform, lift);
        let after = model.hollow.auto_holes(&model.mesh);
        assert_eq!(after.len(), 1);
        // In model space the hole moved to the other pole.
        assert!((after[0].point - before[0].point).norm() > 10.0);
        // It drains, and it is drilled from below.
        let shell = model.hollow_shell().unwrap();
        let world = after[0].world(&model.mesh, shell);
        assert!(world.direction.z > 0.99);
        assert!(kinds(&project).is_empty(), "{:?}", kinds(&project));
    }

    #[test]
    fn auto_holes_dont_drill_through_the_plate_contact() {
        let mut project = load_prepared(&project_path("hollowed"));
        let model = &mut project.models[0];
        model.hollow.spec.holes.clear();
        model.hollow.auto_rule = Some(HollowOp::new().diameter);
        // Standing on the plate: the drain would go through the contact.
        assert!(model.hollow.auto_holes(&model.mesh).is_empty());
        let found = kinds(&project);
        assert!(!found.contains(&Kind::HoleAtPlateContact), "{found:?}");
        assert!(found.contains(&Kind::SealedVoids), "{found:?}");
    }

    #[test]
    fn out_of_bounds_and_clean_projects() {
        let mut project = load_prepared(&project_path("supported"));
        assert!(kinds(&project).is_empty(), "{:?}", kinds(&project));
        let platform = project.slice_config.platform_size;
        let model = &mut project.models[0];
        let at = model.mesh.position() + Vector3::x() * 200.0;
        model.set_position(&platform, at);
        assert!(kinds(&project).contains(&Kind::OutOfBounds));
    }
}
