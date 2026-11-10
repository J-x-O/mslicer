//! What the Properties panel shows: the model selection, or a part of a
//! model (its source, one operation) or a node of the print picked in the
//! Outliner. Picking a viewport tool focuses (and if needed adds) the
//! operation it edits.

use std::hash::{DefaultHasher, Hash, Hasher};

use crate::core::{
    App,
    project::{
        model::{Model, ModelId},
        stack::{Op, OperationId},
    },
    selected::SelectedModel,
    stack,
    state::Tool,
};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Focus {
    /// Whatever models (or collection) are selected.
    #[default]
    Selection,
    /// A model's imported mesh.
    Source(ModelId),
    /// One operation of a model's stack.
    Operation(ModelId, OperationId),
    /// A node of the print.
    Print(PrintNode),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PrintNode {
    /// Printer, slice mode, exposure.
    Profile,
    /// The post processors on the sliced layers.
    LayerOps,
    /// Exposure overrides for layer ranges.
    Overrides,
    /// What the print is saved or sent as.
    Outputs,
}

impl PrintNode {
    pub const ALL: [Self; 4] = [
        Self::Profile,
        Self::LayerOps,
        Self::Overrides,
        Self::Outputs,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Self::Profile => "Profile",
            Self::LayerOps => "Layer ops",
            Self::Overrides => "Overrides",
            Self::Outputs => "Outputs",
        }
    }
}

/// The focus, valid while the model selection stays what it was when it
/// was set: selecting something else (in the viewport, say) shows that.
#[derive(Default)]
pub struct FocusState {
    focus: Focus,
    selection: u64,
}

impl FocusState {
    pub fn get(&self, selected: &SelectedModel) -> Focus {
        match key(selected) == self.selection {
            true => self.focus,
            false => Focus::Selection,
        }
    }

    pub fn set(&mut self, focus: Focus, selected: &SelectedModel) {
        self.focus = focus;
        self.selection = key(selected);
    }
}

/// Identifies a model selection.
fn key(selected: &SelectedModel) -> u64 {
    let mut hasher = DefaultHasher::new();
    match selected {
        SelectedModel::None => 0_u8.hash(&mut hasher),
        SelectedModel::Models(set) => {
            let mut ids: Vec<_> = set.iter().map(|id| id.raw()).collect();
            ids.sort_unstable();
            ids.hash(&mut hasher);
        }
        SelectedModel::Collection(id) => (1_u8, id.raw()).hash(&mut hasher),
    }
    hasher.finish()
}

/// Focuses an operation of a model, selecting the model alone.
pub fn focus_operation(app: &mut App, model: ModelId, op: OperationId) {
    focus(app, Focus::Operation(model, op));
}

/// Focuses a part of a model (selecting the model alone) or a print node.
pub fn focus(app: &mut App, focus: Focus) {
    if let Focus::Source(model) | Focus::Operation(model, _) = focus
        && app.state.selected.single_model() != Some(model)
    {
        app.state.selected.clear();
        app.state.selected.select_model(model);
    }
    app.state.focus.set(focus, &app.state.selected);
}

/// What "show result up to here" leaves out of a model in the viewport.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Hidden {
    pub hollow: bool,
    pub supports: bool,
}

/// For "show result up to `op`": the hollow and supports when they come
/// after it. The viewport's surface already has the geometry rows before
/// the Hollow applied, so the toggle can't take one of those away:
/// [`show_up_to_exact`] tells.
pub fn hidden(show_up_to: Option<(ModelId, OperationId)>, model: &Model) -> Hidden {
    let Some((_, op)) = show_up_to.filter(|(id, _)| *id == model.id) else {
        return Hidden::default();
    };
    let Some(at) = model.stack.index(op) else {
        return Hidden::default();
    };
    let after =
        |o: Option<OperationId>| o.and_then(|o| model.stack.index(o)).is_some_and(|i| i > at);
    Hidden {
        hollow: after(model.stack.hollow().map(|(o, _)| o.id)),
        supports: after(model.stack.supports().map(|(o, _)| o.id)),
    }
}

/// Whether the viewport can show the model exactly as it is after `op`: no
/// active geometry row before the Hollow comes after it.
pub fn show_up_to_exact(model: &Model, op: OperationId) -> bool {
    let Some(at) = model.stack.index(op) else {
        return false;
    };
    let (before, _) = model.stack.geometry();
    (before.iter()).all(|o| model.stack.index(o.id).is_none_or(|i| i <= at))
}

/// The operation a tool edits.
fn edits(tool: Tool, op: &Op) -> bool {
    match tool {
        Tool::Select => false,
        Tool::Support => matches!(op, Op::Supports(_)),
        Tool::Orient => matches!(op, Op::Orient(_)),
        Tool::Holes => matches!(op, Op::Hollow(_)),
    }
}

/// Picks a viewport tool. With one model selected, its operation for the
/// tool is focused, and added first if the model has none (the hole tool
/// hollows the model). Adding is one undo step.
pub fn pick_tool(app: &mut App, tool: Tool) {
    app.state.set_tool(tool);
    if let Some(id) = app.state.selected.single_model()
        && let Some(op) = tool_operation(stack::of(app), id, tool)
    {
        focus_operation(app, id, op);
    }
}

/// The model's operation `tool` edits, added if missing.
fn tool_operation(edit: stack::Edit, id: ModelId, tool: Tool) -> Option<OperationId> {
    let (history, project) = edit;
    let model = project.models.iter().find(|m| m.id == id)?;
    let found = (model.stack.ops().iter())
        .find(|o| edits(tool, &o.op))
        .map(|o| (o.id, o.bypassed));
    match (tool, found) {
        (Tool::Select, _) => None,
        (Tool::Holes, Some((op, true))) => {
            stack::set_hollowed((history, project), id, true);
            Some(op)
        }
        (_, Some((op, _))) => Some(op),
        (Tool::Holes, None) => {
            stack::set_hollowed((history, project), id, true);
            let model = project.models.iter().find(|m| m.id == id)?;
            model.stack.hollow().map(|(o, _)| o.id)
        }
        (Tool::Support, None) => {
            stack::add((history, project), id, Op::Supports(Default::default()))
        }
        (Tool::Orient, None) => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        core::history::History,
        regression::{load_prepared, project_path},
    };

    #[test]
    fn a_tool_finds_or_adds_its_operation() {
        let mut project = load_prepared(&project_path("hollowed"));
        let mut history = History::default();
        let id = project.models[0].id;
        project.models[0].stack = Default::default();

        let op = tool_operation((&mut history, &mut project), id, Tool::Support).unwrap();
        let model = &project.models[0];
        assert_eq!(model.stack.supports().unwrap().0.id, op, "added");
        let steps = history.history.len();
        assert_eq!(
            tool_operation((&mut history, &mut project), id, Tool::Support),
            Some(op),
            "found again"
        );
        assert_eq!(history.history.len(), steps, "no step for finding it");

        let hole = tool_operation((&mut history, &mut project), id, Tool::Holes).unwrap();
        let model = &project.models[0];
        assert!(model.stack.hollow_active(), "the hole tool hollows");
        assert_eq!(model.stack.hollow().unwrap().0.id, hole);
        assert_eq!(
            tool_operation((&mut history, &mut project), id, Tool::Select),
            None
        );
    }

    #[test]
    fn showing_up_to_an_operation_hides_what_follows() {
        let mut project = load_prepared(&project_path("hollowed"));
        let mut history = History::default();
        let id = project.models[0].id;
        project.models[0].stack = Default::default();
        let hollow = tool_operation((&mut history, &mut project), id, Tool::Holes).unwrap();
        let supports = tool_operation((&mut history, &mut project), id, Tool::Support).unwrap();
        let model = &project.models[0];
        // Hollow is a geometry row: it comes before the orientation.
        let first = model.stack.ops()[0].id;
        assert_eq!(first, hollow);
        let orient = model.stack.orient_row().0;
        let all = Hidden {
            hollow: true,
            supports: true,
        };
        let at_hollow = hidden(Some((id, hollow)), model);
        assert_eq!(
            at_hollow,
            Hidden {
                hollow: false,
                ..all
            }
        );
        assert_eq!(hidden(Some((id, supports)), model), Hidden::default());
        let at_orient = hidden(Some((id, orient)), model);
        assert_eq!(at_orient, at_hollow, "the supports follow the orientation");
        assert_eq!(
            hidden(Some((ModelId::new(), orient)), model),
            Hidden::default(),
            "another model"
        );
        assert!(show_up_to_exact(model, hollow));
    }

    #[test]
    fn the_focus_lasts_while_the_selection_does() {
        let (a, b) = (ModelId::new(), ModelId::new());
        let mut selected = SelectedModel::default();
        selected.select_model(a);
        let mut focus = FocusState::default();
        focus.set(Focus::Print(PrintNode::Profile), &selected);
        assert_eq!(focus.get(&selected), Focus::Print(PrintNode::Profile));
        selected.model_clicked(a, false);
        assert_eq!(
            focus.get(&selected),
            Focus::Print(PrintNode::Profile),
            "same selection"
        );
        selected.model_clicked(b, false);
        assert_eq!(focus.get(&selected), Focus::Selection);
    }
}
