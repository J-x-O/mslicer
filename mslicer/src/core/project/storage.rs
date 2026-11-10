use std::{collections::HashMap, sync::Arc};

use anyhow::{Context, Result, ensure};
use const_format::concatcp;
use nalgebra::Vector3;

use crate::core::project::{
    Collection, CollectionId, PostProcessing, Project, RenameState, SupportConfig,
    hollow::{FrozenHollow, HollowSpec, read_mesh_inner, valid_mesh, write_mesh_inner},
    model::{MeshUnit, Model},
    stack::Stack,
};
use common::{
    progress::Progress,
    serde::{Deserializer, SerdeExt, Serializer},
    slice::SliceConfig,
    units::{Milimeter, Milimeters},
};
use slicer::{
    geometry::mesh::{Mesh, MeshId, MeshInner},
    post_process::{
        elephant_foot_fixer::ElephantFootFixer, variable_layer_height::VariableLayerHeight,
    },
};
use tools::supports::{
    SupportId, SupportSet, auto::PlacementConfig, mesh::SupportPreset, raft::RaftConfig,
};

/// Project format version. Value should be incremented whenever the save format
/// changes, even in development so anyone using the dev/prerelease versions
/// don't ruin their projects.
///
/// - v16 (v0.10.0) — Added project notes and an ignored section with info about
///   the format in ASCII.
/// - v15 (v0.10.0) — Store first layers and transition layers as either a layer
///   count or a united distance.
/// - v14 (v0.10.0) — Separate XY and Z supersample antialiasing parameters. You
///   might not always want 3DAA or the (honestly slight) performance impact.
/// - v13 (v0.10.0) — Merge exposure config lift and retract distances. Since
///   these values really should always be the same.
/// - v12 (v0.9.0) — Store mesh units.
/// - v11 (v0.9.0) — Customizable variable layer height similarity threshold.
/// - v10 (v0.9.0) — Added variable layer height post processor.
/// - v9 (v0.8.0) — Added collections.
/// - v8 (v0.8.0) — Added exposure remapping curve to slice config.
/// - v7 (v0.7.0) — Added exposure delay option to exposure config.
/// - v6 (v0.7.0) — Store SliceMode (raster / vector) instead of the specific
///   Format to save to. Since slicing and encoding are now separated.
/// - v5 (v0.7.0) — Replaced the (inaccurate and incredibly slow) blur based
///   anti aliasing with a super sampling approach.
/// - v4 (v0.6.0) — Added relative exposure value to models.
/// - v3 (v0.6.0) — Added the PWM value to SliceConfig -> ExposureConfig.
/// - v2 (v0.5.0) — A complete rewrite using a custom serilizer/deserilizer
///   because of the bincode drama...
/// - v17 (v0.11.0) — Store support sets (auto and manual, each support with the
///   settings it was built with) per model, and the project's support settings.
/// - v18 (v0.11.0) — Store each model's hollowing: the wall kept and the drain
///   holes.
/// - v19 (v0.11.0) — Operation stacks: each model stores its source mesh (as
///   imported, never repaired or flipped in place) and its operations (Mesh
///   Repair, Flip Winding Order, Hollow, Orient, Supports) with their modes.
///   Older projects get a stack made from what they have (see
///   [`Stack::migrated`]); their stored mesh becomes the source.
/// - v20 (v0.11.0) — Hollow rows store their auto-hole diameter.
/// - v21 (v0.11.0) — Frozen results are saved: a frozen geometry row's
///   surface, a frozen Hollow's cavity and drilled print mesh. Supports that
///   couldn't follow their model keep their mark.
const VERSION: u16 = 21;
const DESCRIPTION: &str = concatcp!(
    "Saved by mslicer v",
    crate::VERSION,
    " (https://mslicer.com)"
);

struct ModelInfo {
    mesh: u32,
    collection: Option<CollectionId>,

    name: String,
    unit: MeshUnit,
    color: Vector3<f32>,
    exposure: u8,
    hidden: bool,

    position: Vector3<f32>,
    scale: Vector3<f32>,
    rotation: Vector3<f32>,

    supports: (SupportSet, SupportSet),
    raft: RaftConfig,
    hollow: HollowSpec,
    stack: Stack,
    stranded: Vec<SupportId>,
    frozen_outdated: bool,
    frozen_surface: Option<MeshInner>,
    frozen_hollow: Option<FrozenHollow>,
}

impl ModelInfo {
    pub fn new(mesh: u32, model: &Model) -> Self {
        Self {
            mesh,
            collection: model.collection,
            name: model.name.to_owned(),
            unit: model.unit,
            color: model.color.into(),
            exposure: model.exposure,
            hidden: model.hidden,
            position: model.mesh.position(),
            scale: model.mesh.scale(),
            rotation: model.mesh.rotation(),
            supports: model.supports.world_sets(),
            raft: model.supports.raft(),
            hollow: model.hollow.spec.clone(),
            stack: model.stack.clone(),
            stranded: model.supports.stranded().unwrap_or_default().to_vec(),
            frozen_outdated: model.supports.frozen_outdated(),
            frozen_surface: model.frozen_surface().map(|m| MeshInner {
                vertices: m.vertices.clone(),
                faces: m.faces.clone(),
            }),
            frozen_hollow: FrozenHollow::of(&model.hollow),
        }
    }

    pub fn into_model(self, inner: Arc<MeshInner>) -> Model {
        let mut mesh = Mesh::from_inner(inner);
        mesh.set_position_unchecked(self.position);
        mesh.set_scale_unchecked(self.scale);
        mesh.set_rotation_unchecked(self.rotation);
        mesh.update_transformation_matrix();

        let mut model = Model::from_mesh(mesh)
            .with_name(self.name)
            .with_color(self.color.into())
            .with_exposure(self.exposure)
            .with_hidden(self.hidden)
            .with_collection(self.collection)
            .with_unit(self.unit);

        (model.supports).restore(self.supports.0, self.supports.1, self.raft);
        model
            .supports
            .set_marks(self.stranded, self.frozen_outdated);
        model.hollow.spec = self.hollow;
        model.stack = self.stack;
        if let Some(surface) = self.frozen_surface {
            model.restore_frozen_surface(Arc::new(surface));
        }
        if let Some(frozen) = self.frozen_hollow {
            frozen.restore(&mut model.hollow, &model.mesh.clone());
        }
        model
    }

    pub fn serialize<T: Serializer>(&self, ser: &mut T) {
        // Mesh reference
        ser.write_u32_be(self.mesh);

        // Model properties
        if let Some(collection) = self.collection {
            ser.write_bool(true);
            ser.write_u32_be(collection.raw());
        } else {
            ser.write_bool(false);
        }

        ser.write_u32_be(self.name.len() as u32);
        ser.write_bytes(self.name.as_bytes());
        self.unit.serialize(ser);
        Vector3::from(self.color).serialize(ser);
        ser.write_u8(self.exposure);
        ser.write_bool(self.hidden);

        // Mesh properties
        self.position.serialize(ser);
        self.scale.serialize(ser);
        self.rotation.serialize(ser);

        // Support sets (auto, manual)
        self.supports.0.serialize(ser);
        self.supports.1.serialize(ser);
        self.raft.serialize(ser);
        self.hollow.serialize(ser);
        self.stack.serialize(ser);

        ser.write_u32_be(self.stranded.len() as u32);
        self.stranded
            .iter()
            .for_each(|id| ser.write_u32_be(id.raw()));
        ser.write_bool(self.frozen_outdated);
        ser.write_bool(self.frozen_surface.is_some());
        if let Some(surface) = &self.frozen_surface {
            write_mesh_inner(ser, surface);
        }
        ser.write_bool(self.frozen_hollow.is_some());
        if let Some(frozen) = &self.frozen_hollow {
            frozen.serialize(ser);
        }
    }

    pub fn deserialize<T: Deserializer>(des: &mut T, version: u16) -> Self {
        let mut info = Self {
            mesh: des.read_u32_be(),
            collection: if version < 9 {
                None
            } else {
                des.read_bool()
                    .then(|| CollectionId::from_raw(des.read_u32_be()))
            },
            name: {
                let name_len = des.read_u32_be();
                let data = des.read_bytes(name_len as usize);
                String::from_utf8_lossy(&data).into_owned()
            },
            unit: if version < 12 {
                MeshUnit::Millimeters
            } else {
                MeshUnit::deserialize(des)
            },
            color: Vector3::<f32>::deserialize(des),
            exposure: if version < 4 { 255 } else { des.read_u8() },
            hidden: des.read_bool(),
            position: Vector3::<f32>::deserialize(des),
            scale: Vector3::<f32>::deserialize(des),
            rotation: Vector3::<f32>::deserialize(des),
            supports: if version < 17 {
                (SupportSet::default(), SupportSet::default())
            } else {
                (SupportSet::deserialize(des), SupportSet::deserialize(des))
            },
            raft: if version < 17 {
                RaftConfig::default()
            } else {
                RaftConfig::deserialize(des)
            },
            hollow: if version < 18 {
                HollowSpec::default()
            } else {
                HollowSpec::deserialize(des)
            },
            stack: Stack::default(),
            stranded: Vec::new(),
            frozen_outdated: false,
            frozen_surface: None,
            frozen_hollow: None,
        };
        info.stack = if version < 19 {
            let (auto, manual) = &info.supports;
            Stack::migrated(
                &info.hollow,
                !auto.supports.is_empty(),
                !manual.supports.is_empty(),
            )
        } else {
            Stack::deserialize(des, version)
        };
        if version >= 21 {
            info.stranded = des.read_list(|des| SupportId::from_raw(des.read_u32_be()));
            info.frozen_outdated = des.read_bool();
            info.frozen_surface = des.read_bool().then(|| read_mesh_inner(des));
            // The cavity holds on to its source only so its key stays
            // unique while cached; the model's own mesh does that here.
            let placeholder = Arc::new(MeshInner {
                vertices: Box::new([]),
                faces: Box::new([]),
            });
            info.frozen_hollow = des
                .read_bool()
                .then(|| FrozenHollow::deserialize(des, placeholder));
        }
        info
    }
}

impl Project {
    pub fn serialize<T: Serializer>(&self, ser: &mut T, progress: Progress) {
        ser.write_u16_be(VERSION);

        ser.write_u32_be(DESCRIPTION.len() as u32);
        ser.write_bytes(DESCRIPTION.as_bytes());

        ser.write_u32_be(self.notes.len() as u32);
        ser.write_bytes(self.notes.as_bytes());

        self.slice_config.serialize(ser);
        self.post_processing.serialize(ser);

        let mut total = 0;
        let mut map = HashMap::new();
        let mut meshes = Vec::new();

        // Each model's source, once per shared source.
        ser.write_u32_be(self.models.len() as u32);
        for model in self.models.iter() {
            let id = MeshId::for_mesh_inner(&model.source);
            let mesh = match map.get(&id) {
                Some(mesh) => *mesh,
                None => {
                    let mesh = map.len() as u32;
                    meshes.push(model.source.clone());
                    total += model.source.vertices.len() + model.source.faces.len();
                    map.insert(id, mesh);
                    mesh
                }
            };

            let info = ModelInfo::new(mesh, model);
            info.serialize(ser);
        }

        progress.set_total(total as u64);
        ser.write_u32_be(meshes.len() as u32);
        (meshes.iter()).for_each(|mesh| serialize_mesh_inner(ser, mesh, &progress));

        ser.write_u32_be(self.collections.len() as u32);
        self.collections.iter().for_each(|c| c.serialize(ser));
        self.support_config.serialize(ser);
        progress.set_finished();
    }

    pub fn deserialize<T: Deserializer>(des: &mut T, progress: Progress) -> Result<Self> {
        let version = des.read_u16_be();
        ensure!(
            (2..=VERSION).contains(&version),
            "Save version not supported."
        );

        let notes = if version >= 16 {
            let description_len = des.read_u32_be() as usize;
            des.advance_by(description_len);

            let notes_len = des.read_u32_be() as usize;
            String::from_utf8_lossy(&des.read_bytes(notes_len)).into_owned()
        } else {
            String::new()
        };

        let slice_config = SliceConfig::deserialize(des, version)?;
        let post_processing = PostProcessing::deserialize(des, version);

        let models = des.read_list(|des| ModelInfo::deserialize(des, version));

        // Keep generated support ids ahead of anything already in the file.
        for info in &models {
            for set in [&info.supports.0, &info.supports.1] {
                if let Some(max_id) = set.supports.iter().map(|s| s.id.raw()).max() {
                    tools::supports::SupportId::bump_past(max_id);
                }
            }
        }

        let meshes = des.read_u32_be();
        progress.set_total(des.size().saturating_sub(des.pos()) as u64);
        let mut mesh_list = Vec::new();
        for _ in 0..meshes {
            if des.truncated() {
                break;
            }
            mesh_list.push(Arc::new(deserialize_mesh_inner(des, &progress)));
        }

        let models = (models.into_iter())
            .map(|x| {
                let mesh = mesh_list.get(x.mesh as usize).cloned();
                let mesh = mesh.with_context(|| format!("model `{}` has no mesh", x.name))?;
                Ok(x.into_model(mesh))
            })
            .collect::<Result<Vec<_>>>()?;

        let collections = des.read_list(|des| Collection::deserialize(des));

        // kinda jank but it's whatever
        if let Some(max_id) = collections.iter().map(|x| x.id.raw()).max() {
            CollectionId::bump_past(max_id);
        }

        let support_config = if version < 17 {
            SupportConfig::default()
        } else {
            SupportConfig::deserialize(des)
        };

        ensure!(
            !des.truncated(),
            "The project file is cut short or damaged."
        );
        progress.set_finished();
        Ok(Self {
            path: None,
            notes,
            slice_config,
            support_config,
            post_processing,
            models,
            collections,
        })
    }
}

impl SupportConfig {
    pub fn serialize<T: Serializer>(&self, ser: &mut T) {
        self.preset.serialize(ser);
        ser.write_f32_be(self.auto_placement.min_spacing);
        ser.write_f32_be(self.auto_placement.max_angle);
        ser.write_u32_be(self.mesh_resolution);
        ser.write_f32_be(self.default_lift.get::<Milimeter>());
    }

    pub fn deserialize<T: Deserializer>(des: &mut T) -> Self {
        Self {
            preset: SupportPreset::deserialize(des),
            auto_placement: PlacementConfig {
                min_spacing: des.read_f32_be(),
                max_angle: des.read_f32_be(),
            },
            mesh_resolution: des.read_u32_be(),
            default_lift: Milimeters::new(des.read_f32_be()),
        }
    }
}

impl Collection {
    pub fn serialize<T: Serializer>(&self, ser: &mut T) {
        ser.write_u32_be(self.id.raw());
        ser.write_u32_be(self.name.len() as u32);
        ser.write_bytes(self.name.as_bytes());
        ser.write_bool(self.collapsed);
    }

    pub fn deserialize<T: Deserializer>(des: &mut T) -> Self {
        Self {
            id: CollectionId::from_raw(des.read_u32_be()),
            name: {
                let len = des.read_u32_be();
                let data = des.read_bytes(len as usize);
                String::from_utf8_lossy(&data).into_owned()
            },
            collapsed: des.read_bool(),
            rename: RenameState::None,
        }
    }
}

impl PostProcessing {
    pub fn serialize<T: Serializer>(&self, ser: &mut T) {
        self.variable_layer_height.serialize(ser);
        self.elephant_foot_fixer.serialize(ser);
    }

    pub fn deserialize<T: Deserializer>(des: &mut T, version: u16) -> Self {
        (version < 5).then(|| des.advance_by(5));
        Self {
            variable_layer_height: if version < 10 {
                Default::default()
            } else {
                VariableLayerHeight::deserialize(des, version)
            },
            elephant_foot_fixer: ElephantFootFixer::deserialize(des),
        }
    }
}

fn serialize_mesh_inner<T: Serializer>(ser: &mut T, mesh: &Arc<MeshInner>, progress: &Progress) {
    ser.write_u32_be(mesh.vertices.len() as u32);
    for vert in mesh.vertices.iter() {
        vert.serialize(ser);
        progress.add_complete(1);
    }

    ser.write_u32_be(mesh.faces.len() as u32);
    for face in mesh.faces.iter() {
        ser.write_u32_be(face[0]);
        ser.write_u32_be(face[1]);
        ser.write_u32_be(face[2]);
        progress.add_complete(3);
    }
}

fn deserialize_mesh_inner<T: Deserializer>(des: &mut T, progress: &Progress) -> MeshInner {
    let verts = des.read_list(|des| {
        progress.add_complete(4 * 3);
        Vector3::<f32>::deserialize(des)
    });
    let faces = des.read_list(|des| {
        progress.add_complete(4 * 3);
        [des.read_u32_be(), des.read_u32_be(), des.read_u32_be()]
    });
    valid_mesh(verts, faces)
}

#[cfg(test)]
mod tests {
    use std::fs;

    use common::serde::SliceDeserializer;

    use super::*;
    use crate::regression::project_path;

    fn load(data: &[u8]) -> Result<Project> {
        Project::deserialize(&mut SliceDeserializer::new(data), Progress::new())
    }

    #[test]
    fn a_cut_short_project_fails_to_load() {
        let data = fs::read(project_path("hollowed")).unwrap();
        assert!(load(&data).is_ok());
        let step = (data.len() / 300).max(1);
        for len in (0..data.len()).step_by(step) {
            assert!(load(&data[..len]).is_err(), "cut at {len} loads");
        }
    }

    #[test]
    fn a_damaged_project_does_not_crash() {
        let data = fs::read(project_path("hollowed")).unwrap();
        let step = (data.len() / 300).max(1);
        for at in (0..data.len()).step_by(step) {
            let mut damaged = data.clone();
            let end = (at + 4).min(damaged.len());
            damaged[at..end].fill(0xff);
            // Loads or fails; it doesn't panic or run out of memory.
            let _ = load(&damaged);
        }
    }
}
