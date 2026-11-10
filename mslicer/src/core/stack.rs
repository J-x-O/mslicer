//! Operation stacks in the app: evaluating the geometry section, keeping the
//! modes in step with the edits, and editing stacks with undo.

use std::time::Instant;

use common::units::Milimeters;
use nalgebra::Vector3;
use tools::supports::{
    auto::{AutoPlacement, PlacementConfig},
    mesh::SupportPreset,
};

use crate::{
    core::{
        App,
        history::{Action, History, ModelAction, StackState},
        project::{
            Project, SupportConfig,
            model::{Model, ModelId},
            stack::{HollowOp, Mode, Op, OperationId, Stack, SupportAuto},
        },
    },
    interface::{
        components::grid,
        popup::{Popup, PopupIcon},
    },
    task::{EvaluateGeometry, MeshAnalysis, ReorientSupports, Task},
};

/// Runs every frame: brings each model's surface up to date with its
/// geometry rows, derives the modes from the edits, and reports a repair
/// that was asked for once it has run.
pub fn update(app: &mut App) {
    let platform = app.project.slice_config.platform_size;
    let now = Instant::now();
    let config = &app.project.support_config;
    let mut tasks: Vec<Box<dyn Task>> = Vec::new();
    for model in app.project.models.iter_mut() {
        sync_modes(model);
        if let Some(task) = evaluate_surface(model, &platform) {
            tasks.push(task);
        }
        if let Some(task) = reroute_supports(model, config, &platform, now) {
            tasks.push(task);
        }
        if let Some(id) = model.announce_repair
            && let Some(stats) = model.repair_stats(id)
            && model.surface_ready()
        {
            model.announce_repair = None;
            app.popup.open(Popup::new("Repair Report", move |_app, ui| {
                if stats.any() {
                    ui.label("The following defects were found and are repaired:");
                    ui.add_space(8.0);
                    grid("repair").show(ui, |ui| {
                        for (name, value) in stats.rows().into_iter().filter(|(_, v)| *v > 0) {
                            ui.label(name);
                            ui.horizontal(|ui| {
                                ui.label(value.to_string());
                                ui.take_available_width();
                            });
                            ui.end_row();
                        }
                    });
                } else {
                    ui.label("No defects found.");
                }
                ui.add_space(8.0);
                ui.label(
                    "The repair is an operation in the model's stack: bypass or remove it to get the mesh as imported back.",
                );
                ui.add_space(5.0);
                let mut close = false;
                ui.centered_and_justified(|ui| close = ui.button("Close").clicked());
                close
            }));
            let _ = PopupIcon::Info;
        }
    }
    app.tasks.extend_boxed(tasks);
}

/// The modes other than Frozen follow from what is there: manual holes and
/// hand-placed supports are edits, a manual orientation is an edit.
fn sync_modes(model: &mut Model) {
    // Auto-orient holds its result; an upstream change only marks it.
    let upstream = model.upstream_hash();
    if model
        .upstream
        .replace(upstream)
        .is_some_and(|last| last != upstream)
        && let Op::Orient(orient) = &mut model.stack.orient_mut().op
        && orient.auto.is_some()
    {
        orient.outdated = true;
    }
    let hollow_mode = Stack::hollow_mode(&model.hollow.spec);
    let supports_mode = Stack::supports_mode(model.supports.has_manual());
    let mut frozen_hollow = false;
    let mut frozen_supports = false;
    let mut auto_rule = None;
    for operation in model.stack.ops_mut() {
        let derived = match &operation.op {
            Op::Repair(_) | Op::FlipWinding => Mode::Auto,
            Op::Hollow(_) => hollow_mode,
            Op::Orient(orient) if orient.auto.is_some() => Mode::Auto,
            Op::Orient(_) => Mode::Edited,
            Op::Supports(_) => supports_mode,
        };
        if operation.mode != Mode::Frozen {
            operation.mode = derived;
        } else if matches!(operation.op, Op::Hollow(_)) {
            frozen_hollow = true;
        } else if matches!(operation.op, Op::Supports(_)) {
            frozen_supports = true;
        }
        if let Op::Hollow(hollow) = operation.op
            && hollow.auto_holes
            && operation.active()
        {
            auto_rule = Some(hollow.diameter);
        }
    }
    model.hollow.frozen = frozen_hollow;
    model.hollow.auto_rule = auto_rule;
    model.supports.frozen = frozen_supports;
}

/// Starts routing the model's supports again if it was turned, lifted or
/// scaled, once that has settled and its surface is ready. The auto supports
/// are regenerated with what they were generated with (or the project's
/// support settings), without lifting the model.
fn reroute_supports(
    model: &mut Model,
    config: &SupportConfig,
    platform: &Vector3<Milimeters>,
    now: Instant,
) -> Option<Box<dyn Task>> {
    if !model.surface_ready() || (model.stack.hollow_active() && !model.hollow.ready(&model.mesh)) {
        return None;
    }
    let (surface, bvh) = model.support_surface();
    let ticket = model.supports.poll_reroute(surface.mesh_id(), now)?;
    let auto = model.stack.supports().and_then(|(_, s)| s.auto.clone());
    let (preset, placement) = match auto {
        Some(auto) => (
            auto.preset,
            PlacementConfig {
                min_spacing: auto.min_spacing,
                max_angle: auto.max_angle,
            },
        ),
        None => (config.preset.clone(), config.auto_placement.clone()),
    };
    let platform = platform.map(|x| x.convert());
    let generator = AutoPlacement::new(placement, preset, platform);
    let input = (model.supports).reroute_input(ticket.key, surface, bvh, Some(generator));
    let task = ReorientSupports::new(model.id, model.name.clone(), ticket, input);
    Some(Box::new(task))
}

/// Starts evaluating the model's surface if it is out of date. Without
/// geometry rows the surface is the source itself, at once. A frozen
/// geometry row holds the surface as it is.
fn evaluate_surface(model: &mut Model, platform: &Vector3<Milimeters>) -> Option<Box<dyn Task>> {
    let key = model.geometry_key();
    if key.steps.is_empty() {
        model.geometry.surface.withdraw();
        let source = model.source.clone();
        return (model.set_surface(source, platform)).then(|| analysis(model));
    }
    let frozen = (model.stack.geometry().0.iter()).any(|o| o.mode == Mode::Frozen);
    if frozen && model.geometry.surface.latest().is_some() {
        return None;
    }
    if let Some(evaluated) = model.geometry.surface.get(&key) {
        let mesh = evaluated.mesh.clone();
        let repairs = evaluated.repairs.clone();
        model.repairs.extend(repairs);
        return (model.set_surface(mesh, platform)).then(|| analysis(model));
    }
    let ticket = model.geometry.surface.request(key)?;
    Some(Box::new(EvaluateGeometry::new(model, ticket)))
}

fn analysis(model: &Model) -> Box<dyn Task> {
    Box::new(MeshAnalysis::new(model))
}

/// Changes a model's stack (and the payloads its rows own) as one undo step,
/// if anything changed.
pub fn edit(
    (history, project): Edit,
    id: ModelId,
    label: &'static str,
    change: impl FnOnce(&mut Model),
) {
    let Some(model) = project.model(id) else {
        return;
    };
    let before = StackState::of(model);
    change(model);
    let after = StackState::of(model);
    if after != before {
        let action = ModelAction::Stack(Box::new(before), label);
        history.track_model(id, action);
    }
}

/// [`edit`] for a value being dragged: following changes with the same
/// label merge into the step already recorded, so a drag is one undo step.
pub fn edit_merged(
    (history, project): Edit,
    id: ModelId,
    label: &'static str,
    change: impl FnOnce(&mut Model),
) {
    let last = history.history.back();
    let merges = matches!(last, Some(Action::Model { id: last, action: ModelAction::Stack(_, l) })
        if *last == id && *l == label);
    if merges {
        if let Some(model) = project.model(id) {
            change(model);
        }
        history.future.clear();
        return;
    }
    edit((history, project), id, label, change);
}

/// What a stack edit works on: the history to record it in and the project.
pub type Edit<'a> = (&'a mut History, &'a mut Project);

/// The parts of the app a stack edit works on.
pub fn of(app: &mut App) -> Edit<'_> {
    (&mut app.history, &mut app.project)
}

/// Adds an operation (or finds the one there can only be one of), as one
/// undo step. Returns its id.
pub fn add(app: Edit, id: ModelId, op: Op) -> Option<OperationId> {
    let mut added = None;
    let announce = matches!(op, Op::Repair(_));
    edit(app, id, "Added Operation", |model| {
        let op_id = model.stack.add(op);
        if announce {
            model.announce_repair = Some(op_id);
        }
        added = Some(op_id);
    });
    added
}

/// Removes an operation. Its payload (Hollow's shell and holes, the support
/// sets) is cleared with it, so re-adding starts fresh; undo brings both
/// back.
pub fn remove(app: Edit, id: ModelId, op: OperationId) {
    edit(app, id, "Removed Operation", |model| {
        if let Some(removed) = model.stack.remove(op) {
            match removed.op {
                Op::Hollow(_) => model.hollow.spec = Default::default(),
                Op::Supports(_) => model.supports.clear(),
                _ => {}
            }
            model.repairs.remove(&op);
        }
    });
}

/// Moves a geometry row up (`-1`) or down (`+1`).
pub fn move_by(app: Edit, id: ModelId, op: OperationId, by: isize) {
    edit(app, id, "Reordered Operations", |model| {
        model.stack.move_by(op, by);
    });
}

pub fn set_bypassed(app: Edit, id: ModelId, op: OperationId, bypassed: bool) {
    edit(app, id, "Bypassed Operation", |model| {
        if let Some(operation) = model.stack.get_mut(op)
            && operation.op.can_bypass()
        {
            operation.bypassed = bypassed;
        }
    });
}

/// Freezes an operation, or thaws it (back to the mode its edits give it).
pub fn set_frozen(app: Edit, id: ModelId, op: OperationId, frozen: bool) {
    edit(app, id, "Changed Operation Mode", |model| {
        if let Some(operation) = model.stack.get_mut(op) {
            operation.mode = match frozen {
                true => Mode::Frozen,
                false => Mode::Auto,
            };
        }
        sync_modes(model);
    });
}

/// Whether the model's operation of this kind is frozen.
pub fn is_frozen(model: &Model, pick: fn(&Op) -> bool) -> bool {
    (model.stack.ops().iter()).any(|o| pick(&o.op) && o.mode == Mode::Frozen)
}

/// Makes sure the model has a Hollow row that takes part, with a shell, as
/// one undo step; or, with `on` false, bypasses it (keeping its holes).
pub fn set_hollowed(app: Edit, id: ModelId, on: bool) {
    edit(app, id, "Changed Hollowing", |model| {
        if on {
            let op = model.stack.add(Op::Hollow(HollowOp::new()));
            if let Some(operation) = model.stack.get_mut(op) {
                operation.bypassed = false;
            }
            model.hollow.spec.shell.get_or_insert_with(Default::default);
        } else if let Some(operation) = model.stack.hollow_mut() {
            operation.bypassed = true;
        }
    });
}

/// Makes sure the model has a Supports row that takes part, for a support
/// edit about to be made, and returns the edit's undo `action` extended to
/// take the row away again if it had to be added.
pub fn supports_action(model: &mut Model, action: ModelAction) -> ModelAction {
    if model.stack.supports_active() {
        return action;
    }
    let before = StackState::of(model);
    let op = model.stack.add(Op::Supports(Default::default()));
    if let Some(operation) = model.stack.get_mut(op) {
        operation.bypassed = false;
    }
    let row = ModelAction::Stack(Box::new(before), "Added Supports");
    ModelAction::Batch(vec![row, action])
}

/// Records what the auto supports are generated with in the Supports row, so
/// they are regenerated the same way when the model changes.
pub fn record_support_auto(model: &mut Model, preset: &SupportPreset, placement: &PlacementConfig) {
    let auto = SupportAuto {
        preset: preset.clone(),
        min_spacing: placement.min_spacing,
        max_angle: placement.max_angle,
    };
    for operation in model.stack.ops_mut() {
        if let Op::Supports(supports) = &mut operation.op {
            supports.auto = Some(auto.clone());
        }
    }
}

/// Whether the model's supports can be changed: not while frozen.
pub fn supports_editable(model: &Model) -> bool {
    !is_frozen(model, |op| matches!(op, Op::Supports(_)))
}

/// Brings the model's surface up to date at once, on this thread.
#[cfg(test)]
pub fn evaluate_now(model: &mut Model, platform: &Vector3<Milimeters>) {
    use crate::core::project::geometry::evaluate;
    sync_modes(model);
    let key = model.geometry_key();
    if !key.steps.is_empty()
        && let Some(ticket) = model.geometry.surface.request(key)
    {
        let progress = common::progress::Progress::new();
        let out = evaluate(
            &ticket.key.source,
            &ticket.key.steps,
            &progress,
            &ticket.token,
        );
        model.geometry.surface.finish(&ticket.key, out);
    }
    let _ = evaluate_surface(model, platform);
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use common::{
        progress::Progress,
        serde::{DynamicSerializer, SliceDeserializer},
    };

    use super::*;
    use crate::{
        core::project::stack::{HollowOp, Repair},
        regression::{Reference, load_prepared, project_path, slice},
    };

    fn kinds(model: &Model) -> Vec<(&'static str, Mode)> {
        (model.stack.ops().iter())
            .map(|o| (o.op.name(), o.mode))
            .collect()
    }

    #[test]
    fn v18_projects_migrate_to_stacks() {
        let teapot = load_prepared(&project_path("teapot"));
        assert_eq!(kinds(&teapot.models[0]), [("Orient", Mode::Edited)]);

        let hollowed = load_prepared(&project_path("hollowed"));
        assert_eq!(
            kinds(&hollowed.models[0]),
            [("Hollow", Mode::Edited), ("Orient", Mode::Edited)]
        );

        let supported = load_prepared(&project_path("supported"));
        assert_eq!(
            kinds(&supported.models[0]),
            [("Orient", Mode::Edited), ("Supports", Mode::Edited)]
        );
        // The stored mesh is the source, and the surface.
        let model = &supported.models[0];
        assert!(Arc::ptr_eq(&model.source, model.mesh.inner()));
    }

    fn round_trip(project: &Project) -> Project {
        let mut ser = DynamicSerializer::new();
        project.serialize(&mut ser, Progress::new());
        let data = ser.into_inner();
        assert_eq!(u16::from_be_bytes([data[0], data[1]]), 21);
        Project::deserialize(&mut SliceDeserializer::new(&data), Progress::new()).unwrap()
    }

    #[test]
    fn projects_round_trip_and_slice_the_same() {
        let dir = std::env::temp_dir().join(format!("mslicer-v19-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        for name in [
            "teapot",
            "t_shape",
            "hollowed",
            "supported",
            "vlh_eff",
            "vector",
        ] {
            let old = load_prepared(&project_path(name));
            let back = round_trip(&old);
            assert_eq!(back.models.len(), old.models.len());
            for (a, b) in old.models.iter().zip(&back.models) {
                assert_eq!(a.stack, b.stack, "{name}");
                assert_eq!(a.hollow.spec, b.hollow.spec);
                assert_eq!(
                    a.mesh.transformation_matrix(),
                    b.mesh.transformation_matrix()
                );
                assert_eq!(a.source.vertices, b.source.vertices);
                let (sa, sb) = (a.supports.world_sets(), b.supports.world_sets());
                assert_eq!(sa.0.supports.len(), sb.0.supports.len());
                assert_eq!(sa.1.supports.len(), sb.1.supports.len());
            }

            // Sliced from the v19 file, it is what the v18 file gave.
            let path = dir.join(format!("{name}.mslicer"));
            let mut ser = DynamicSerializer::new();
            old.serialize(&mut ser, Progress::new());
            std::fs::write(&path, ser.into_inner()).unwrap();
            let mut fresh = load_prepared(&path);
            let mut original = load_prepared(&project_path(name));
            let now = Reference::of(&slice(&mut fresh), &fresh);
            let then = Reference::of(&slice(&mut original), &original);
            assert_eq!(now.diff(&then), None, "{name}");
        }
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn stack_edits_undo_and_redo() {
        let mut project = load_prepared(&project_path("hollowed"));
        let mut history = History::default();
        let id = project.models[0].id;
        let start = StackState::of(&project.models[0]);
        let hollow = project.models[0].stack.hollow().unwrap().0.id;

        let flip = add((&mut history, &mut project), id, Op::FlipWinding).unwrap();
        move_by((&mut history, &mut project), id, flip, 1);
        set_bypassed((&mut history, &mut project), id, flip, true);
        set_frozen((&mut history, &mut project), id, hollow, true);
        remove((&mut history, &mut project), id, hollow);
        let model = &project.models[0];
        assert!(model.stack.hollow().is_none());
        assert!(model.hollow.spec.holes.is_empty(), "removed with its holes");
        assert_eq!(history.history.len(), 5);

        // Undone step by step...
        type Check = Box<dyn Fn(&Model) -> bool>;
        let steps: Vec<Check> = vec![
            Box::new(|m| m.hollow.spec.holes.len() == 2),
            Box::new(|m| m.stack.hollow().unwrap().0.mode != Mode::Frozen),
            Box::new(|m| m.stack.ops().iter().all(|o| !o.bypassed)),
            Box::new(move |m| m.stack.index(flip) == Some(0)),
            Box::new(|m| m.stack.ops().iter().all(|o| o.op != Op::FlipWinding)),
        ];
        for check in &steps {
            assert!(history.undo_on(&mut project));
            assert!(check(&project.models[0]));
        }
        assert_eq!(StackState::of(&project.models[0]), start);

        // ...and redone.
        for _ in &steps {
            assert!(history.redo_on(&mut project));
        }
        let model = &project.models[0];
        assert!(model.stack.hollow().is_none() && model.hollow.spec.holes.is_empty());
        assert!(model.stack.get(flip).is_some_and(|o| o.bypassed));
    }

    #[test]
    fn a_support_edit_that_adds_the_row_undoes_in_one_step() {
        let mut project = load_prepared(&project_path("t_shape"));
        let mut history = History::default();
        let model = &mut project.models[0];
        let id = model.id;
        assert!(!model.stack.supports_active());
        let action = ModelAction::Supports(model.supports.snapshot());
        let action = supports_action(model, action);
        history.track_model(id, action);
        assert!(project.models[0].stack.supports_active());

        assert!(history.undo_on(&mut project));
        assert!(project.models[0].stack.supports().is_none());
        assert!(history.redo_on(&mut project));
        assert!(project.models[0].stack.supports_active());
    }

    /// A T with its stem's faces inside out.
    fn model_with_steps(project: &mut Project, ops: &[Op]) -> usize {
        let platform = project.slice_config.platform_size;
        let model = &mut project.models[0];
        for op in ops {
            model.stack.add(op.clone());
        }
        evaluate_now(model, &platform);
        0
    }

    #[test]
    fn geometry_operations_leave_the_source_alone() {
        let mut project = load_prepared(&project_path("t_shape"));
        let platform = project.slice_config.platform_size;
        let source = project.models[0].source.clone();
        let i = model_with_steps(&mut project, &[Op::FlipWinding]);
        let model = &mut project.models[i];
        assert!(model.surface_ready());
        assert!(!Arc::ptr_eq(model.mesh.inner(), &source));
        assert_eq!(model.mesh.faces()[0], {
            let [a, b, c] = source.faces[0];
            [a, c, b]
        });
        assert_eq!(model.source.faces, source.faces, "source untouched");

        // Bypassed: the surface is the source again, at once.
        let flip = model.stack.ops()[0].id;
        model.stack.get_mut(flip).unwrap().bypassed = true;
        evaluate_now(model, &platform);
        assert!(Arc::ptr_eq(model.mesh.inner(), &source));
    }

    #[test]
    fn a_repair_reports_what_it_found() {
        let mut project = load_prepared(&project_path("teapot"));
        let i = model_with_steps(&mut project, &[Op::Repair(Repair::default())]);
        let model = &project.models[i];
        let (op, _) = model
            .stack
            .find(|op| match op {
                Op::Repair(r) => Some(r),
                _ => None,
            })
            .unwrap();
        assert!(model.repairs.contains_key(&op.id));
        assert!(model.surface_ready());
    }

    #[test]
    fn geometry_after_hollow_applies_to_the_print_mesh() {
        let mut project = load_prepared(&project_path("hollowed"));
        let platform = project.slice_config.platform_size;
        let before = project.models[0].print_mesh();
        let model = &mut project.models[0];
        let flip = model.stack.add(Op::FlipWinding);
        assert!(model.stack.move_by(flip, 1));
        // Flip sits after Hollow, so it flips the hollowed print mesh, and
        // leaves the surface alone.
        assert!(model.stack.index(flip) > model.stack.index(model.stack.hollow().unwrap().0.id));
        evaluate_now(model, &platform);
        assert!(Arc::ptr_eq(model.mesh.inner(), &model.source));
        let after = model.print_mesh();
        assert_eq!(after.face_count(), before.face_count());
        let [a, b, c] = before.faces()[0];
        assert_eq!(after.faces()[0], [a, c, b]);

        // Moved before Hollow, it flips the surface instead.
        assert!(model.stack.move_by(flip, -1));
        evaluate_now(model, &platform);
        assert!(!Arc::ptr_eq(model.mesh.inner(), &model.source));
        let _ = HollowOp::default();
    }
}

#[cfg(test)]
mod frozen_tests {
    use std::sync::Arc;

    use common::{
        progress::Progress,
        serde::{DynamicSerializer, SliceDeserializer},
    };

    use super::*;
    use crate::regression::{load_prepared, project_path};

    fn reload(project: &Project) -> Project {
        let mut ser = DynamicSerializer::new();
        project.serialize(&mut ser, Progress::new());
        let data = ser.into_inner();
        Project::deserialize(&mut SliceDeserializer::new(&data), Progress::new()).unwrap()
    }

    fn freeze(model: &mut Model, pick: fn(&Op) -> bool) {
        for operation in model.stack.ops_mut() {
            if pick(&operation.op) {
                operation.mode = Mode::Frozen;
            }
        }
        sync_modes(model);
    }

    #[test]
    fn a_frozen_hollow_is_saved_and_needs_no_recompute() {
        let mut project = load_prepared(&project_path("hollowed"));
        freeze(&mut project.models[0], |op| matches!(op, Op::Hollow(_)));
        let print = project.models[0].print_mesh();

        let mut back = reload(&project);
        let model = &mut back.models[0];
        sync_modes(model);
        // Ready at once: nothing was computed after loading.
        assert!(model.hollow.cavity(&model.mesh).is_some());
        assert!(model.hollow.drilled(&model.mesh).is_some());
        assert!(model.ready());
        assert_eq!(model.print_mesh().face_count(), print.face_count());
    }

    #[test]
    fn a_frozen_geometry_row_keeps_its_surface_across_saving() {
        let mut project = load_prepared(&project_path("t_shape"));
        let platform = project.slice_config.platform_size;
        let model = &mut project.models[0];
        model.stack.add(Op::FlipWinding);
        evaluate_now(model, &platform);
        freeze(model, |op| matches!(op, Op::FlipWinding));
        let surface = model.mesh.inner().clone();
        assert!(!Arc::ptr_eq(&surface, &model.source));

        let mut back = reload(&project);
        let model = &mut back.models[0];
        evaluate_now(model, &platform);
        assert!(model.surface_ready());
        assert_eq!(model.mesh.faces(), &surface.faces[..]);
        assert_ne!(model.mesh.faces(), &model.source.faces[..]);
    }
}
