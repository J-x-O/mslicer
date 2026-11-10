mod file_dialog;
mod mesh;
mod project;
mod reload_model;
mod sliced;

pub use self::{
    file_dialog::{FileDialog, MultiFileDialog},
    mesh::{MeshLoad, MeshSave},
    project::{ProjectLoad, ProjectSave},
    reload_model::ReloadModel,
    sliced::{LoadSliced, SaveSliced},
};

use std::path::Path;

/// The file's name for messages, or the whole path if it has none.
pub fn file_name(path: &Path) -> String {
    match path.file_name() {
        Some(name) => name.to_string_lossy().into_owned(),
        None => path.display().to_string(),
    }
}
