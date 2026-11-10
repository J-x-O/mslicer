//! The operations a newly imported model starts with (a global setting:
//! the project keeps each model's own stack).

use serde::{Deserialize, Serialize};

use crate::core::project::{
    model::Model,
    stack::{HollowOp, Op, Repair},
};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct DefaultStack {
    /// Repair the mesh.
    pub repair: bool,
    /// Hollow it (with auto holes).
    pub hollow: bool,
    /// An empty Supports operation, ready for the support tool and the
    /// auto supports.
    pub supports: bool,
}

impl DefaultStack {
    /// Adds the operations to a model that was just imported. Part of
    /// adding it: undo removes the model with them.
    pub fn apply(&self, model: &mut Model) {
        if self.repair {
            model.stack.add(Op::Repair(Repair::default()));
        }
        if self.hollow {
            model.stack.add(Op::Hollow(HollowOp::new()));
            model.hollow.spec.shell.get_or_insert_with(Default::default);
        }
        if self.supports {
            model.stack.add(Op::Supports(Default::default()));
        }
        for operation in model.stack.ops_mut() {
            operation.bypassed = false;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regression::{load_prepared, project_path};

    #[test]
    fn a_new_model_gets_the_default_operations() {
        let mut project = load_prepared(&project_path("hollowed"));
        let model = &mut project.models[0];
        model.stack = Default::default();
        model.hollow.spec = Default::default();
        DefaultStack::default().apply(model);
        assert_eq!(model.stack.len(), 1, "just the orientation");

        let all = DefaultStack {
            repair: true,
            hollow: true,
            supports: true,
        };
        all.apply(model);
        let names: Vec<_> = model.stack.ops().iter().map(|o| o.op.name()).collect();
        assert_eq!(names.len(), 4, "{names:?}");
        assert!(model.stack.hollow_active() && model.stack.supports_active());
        assert!(model.hollow.spec.shell.is_some());
    }
}
