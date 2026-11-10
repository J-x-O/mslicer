//! The inputs of a full slice, as a hash: what decides whether a sliced
//! result is still current.

use common::serde::DynamicSerializer;

use crate::{
    core::project::{Project, model::Model},
    engine::{InputHash, InputHasher},
};

/// The hash of a model's contribution to the slice: its print mesh (by
/// identity, so it changes when the cavity or holes come in), its placement,
/// its relative exposure and its support mesh's inputs.
///
/// Changing another model, or a model's stack in a way that doesn't reach
/// what it prints, leaves it alone.
pub fn model_key(model: &Model) -> InputHash {
    let mut hash = InputHasher::new();
    let print = model.print_mesh();
    hash.arc(print.inner())
        .matrix(print.transformation_matrix())
        .u64(model.exposure as u64);
    model.supports.hash_inputs(&mut hash);
    hash.input_hash()
}

/// The hash of everything a full slice of `project` depends on: the slice
/// config, the post-processing and every visible model, in order.
pub fn slice_key(project: &Project) -> InputHash {
    let mut ser = DynamicSerializer::new();
    project.slice_config.serialize(&mut ser);
    project.post_processing.serialize(&mut ser);

    let mut hash = InputHasher::new();
    hash.hash(&ser.into_inner());
    for model in project.models.iter().filter(|m| !m.hidden) {
        hash.u64(model_key(model).0);
    }
    hash.input_hash()
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::*;
    use crate::regression::{load_prepared, project_path};

    #[test]
    fn the_key_follows_what_slicing_reads() {
        let mut project = load_prepared(&project_path("vlh_eff"));
        let key = slice_key(&project);
        assert_eq!(slice_key(&project), key, "stable");

        // The hidden model doesn't count.
        project.models[2].color = common::color::LinearRgb::splat(0.3);
        project.models[2].set_position(
            &project.slice_config.platform_size,
            Vector3::new(1.0, 2.0, 0.0),
        );
        assert_eq!(slice_key(&project), key);

        // Neither does a colour, but a move, an exposure or a setting does.
        project.models[0].color = common::color::LinearRgb::splat(0.3);
        assert_eq!(slice_key(&project), key);
        let platform = project.slice_config.platform_size;
        let at = project.models[0].mesh.position();
        project.models[0].set_position(&platform, at + Vector3::x());
        let moved = slice_key(&project);
        assert_ne!(moved, key);
        project.models[1].exposure = 200;
        assert_ne!(slice_key(&project), moved);
        let exposed = slice_key(&project);
        project.post_processing.elephant_foot_fixer.inset_distance = 0.7;
        assert_ne!(slice_key(&project), exposed);
    }
}
