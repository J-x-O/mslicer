use common::{color::SRgb, units::Milimeter};
use nalgebra::Vector3;
use slicer::geometry::mesh::Mesh;
use tools::supports::{
    SupportId, SupportSet,
    auto::{BraceNetwork, Generated},
    mesh::{FaceMap, SupportPreset, Transform, build_supports},
};
use wgpu::Device;

use crate::{core::state::PreviewModel, render::util::MeshBuffers};

#[derive(Default)]
pub struct Supports {
    auto: SupportSet,
    manual: SupportSet,
    transform: Transform,

    resolution: u32,
    mesh: Option<(Mesh, FaceMap)>,
    buffers: Option<MeshBuffers>,
}

impl Supports {
    pub fn set_resolution(&mut self, resolution: u32) {
        self.resolution = resolution;
    }

    pub fn invalidate_cache(&mut self) {
        self.mesh.take();
        self.buffers.take();
    }

    pub fn clear(&mut self) {
        self.auto = SupportSet::default();
        self.manual = SupportSet::default();
        self.transform = Default::default();
        self.invalidate_cache();
    }

    pub fn get(&self, id: SupportId) -> Option<&tools::supports::Support> {
        (self.auto.supports.iter())
            .chain(self.manual.supports.iter())
            .find(|s| s.id == id)
    }

    pub fn replace_auto(&mut self, config: &SupportPreset, supports: Vec<[Vector3<f32>; 3]>) {
        self.invalidate_cache();
        let mut set = SupportSet::default();
        for points in supports {
            set.add_routed(
                SupportId::new(),
                config.tip_radius.get::<Milimeter>(),
                config.radius.get::<Milimeter>(),
                points,
            );
        }
        self.auto = set;
    }

    /// Replaces the stored auto supports and their brace network.
    ///
    /// `network` is the output of `AutoPlacement::generate_braces` over the same
    /// plate supports, so the pillars are tied together before meshing and any
    /// inverted-V fork truncations are applied to the middle pillars. The
    /// model-landing supports and stabilising legs go after the plate
    /// supports, which keeps the network's support indices valid.
    pub fn replace_auto_with_braces(
        &mut self,
        config: &SupportPreset,
        generated: Generated,
        network: BraceNetwork,
    ) {
        self.replace_auto(config, generated.plate);
        for landing in &generated.landings {
            self.auto.add_landing(
                SupportId::new(),
                config.tip_radius.get::<Milimeter>(),
                config.radius.get::<Milimeter>(),
                landing,
            );
        }
        for leg in network.legs {
            self.auto.add_routed(
                SupportId::new(),
                config.tip_radius.get::<Milimeter>(),
                config.radius.get::<Milimeter>(),
                leg,
            );
        }
        self.auto.braces = network.braces;
        for (index, bottom_z) in network.fork_bottoms {
            self.auto.truncate_pillar(index, bottom_z);
        }
        // Bases go on last, once forks have cut their middle pillars short.
        self.auto.add_bases(&config.base_config());
    }

    pub fn add_manual(&mut self, config: &SupportPreset, support: [Vector3<f32>; 3]) {
        self.invalidate_cache();
        self.manual.add_routed(
            SupportId::new(),
            config.tip_radius.get::<Milimeter>(),
            config.radius.get::<Milimeter>(),
            support,
        );
        self.manual.add_bases(&config.base_config());
    }

    pub fn auto(&self) -> &SupportSet {
        &self.auto
    }

    pub fn manual(&self) -> &SupportSet {
        &self.manual
    }

    /// Replaces the stored support sets, for deserialization.
    pub fn restore(&mut self, auto: SupportSet, manual: SupportSet) {
        self.auto = auto;
        self.manual = manual;
        self.invalidate_cache();
    }

    pub fn mesh(&mut self) -> &Option<(Mesh, FaceMap)> {
        if self.mesh.is_some() || (self.auto.supports.is_empty() && self.manual.supports.is_empty())
        {
            return &self.mesh;
        }

        // Build both sets into a single mesh: put every support from both sets
        // into one SupportSet so build_supports assigns contiguous, non
        // -overlapping face ranges keyed by id.
        let mut set = SupportSet {
            lift: self.auto.lift,
            ..SupportSet::default()
        };
        set.supports.extend(self.auto.supports.iter().cloned());
        set.supports.extend(self.manual.supports.iter().cloned());
        set.braces = self.auto.braces.clone();
        set.braces.extend(self.manual.braces.clone());
        if let Some(raft) = &self.auto.raft {
            set.raft = Some(raft.clone());
        }

        if let Some((mesh, map)) = build_supports(&set, self.transform, self.resolution) {
            self.mesh = Some((mesh, map));
        }

        &self.mesh
    }

    pub fn support_for_face(&mut self, face: u32) -> Option<SupportId> {
        let (_, map) = self.mesh().as_ref()?;
        (map.iter().find(|(_, r)| r.contains(&face))).map(|(id, _)| *id)
    }

    pub fn get_buffers(&mut self, device: &Device) -> &Option<MeshBuffers> {
        if self.buffers.is_none()
            && let Some((mesh, _)) = self.mesh()
        {
            self.buffers = Some(MeshBuffers::get_for(device, mesh));
        }

        &self.buffers
    }

    pub fn try_get_buffers(&self) -> Option<(&MeshBuffers, u32)> {
        self.buffers.as_ref().map(|x| {
            let face_count = self.mesh.as_ref().unwrap().0.face_count() as u32 * 3;
            (x, face_count)
        })
    }

    // kickin it old school rn
    pub fn set_transform(
        &mut self,
        position: Vector3<f32>,
        scale: Vector3<f32>,
        rotation: Vector3<f32>,
    ) {
        if let Some((mesh, _)) = &mut self.mesh {
            if mesh.rotation().xy() != rotation.xy() {
                self.clear();
                return;
            }

            mesh.set_position(position.xy().push(mesh.position().z));
            mesh.set_rotation(mesh.rotation().xy().push(rotation.z));

            let old = self.transform;
            self.transform = Transform {
                position,
                scale,
                _rotation: rotation.z,
            };

            if scale != old.scale || position.z != old.position.z {
                self.invalidate_cache();
            }
        }
    }

    pub fn remove(&mut self, id: SupportId) {
        self.auto.supports.retain(|x| x.id != id);
        self.manual.supports.retain(|x| x.id != id);
    }

    pub fn preview(&mut self, device: &Device, color: SRgb<f32>) -> Option<PreviewModel> {
        if let Some((faces, transform)) = (self.mesh().as_ref())
            .map(|(mesh, _)| (mesh.face_count(), *mesh.transformation_matrix()))
            && let Some(buffers) = self.get_buffers(device)
        {
            return Some(PreviewModel {
                mesh: buffers.clone(),
                faces,
                transform,
                color,
            });
        }

        None
    }
}
