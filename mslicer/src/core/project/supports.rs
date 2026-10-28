use std::{
    collections::HashSet,
    fmt, mem,
    sync::Arc,
    thread::{self, JoinHandle},
};

use common::{color::SRgb, units::Milimeter};
use nalgebra::{Isometry3, Rotation3, Vector3};
use slicer::geometry::{mesh::Mesh, partitioning::Bvh};
use tools::supports::{
    Support, SupportId, SupportSet,
    auto::{BraceNetwork, Generated, Routed, brace_support},
    mesh::{FaceMap, SupportPreset, Transform, build_supports},
    raft::{Raft, RaftConfig},
};
use wgpu::Device;

use crate::{core::state::PreviewModel, render::util::MeshBuffers};

/// A model's supports.
///
/// The support coordinates are stored for one model pose, the `frame`. Moving
/// the model across the plate or turning it about z carries its supports
/// along without touching them: only the pose changes, and the coordinates
/// are brought up to date the next time something needs them. Any other
/// change (lifting, tilting, scaling) would leave them hanging in the wrong
/// place, so it clears them.
///
/// Stored pillars reach the plate; their feet (a base each, or the raft) are
/// added when meshing, from the model's raft settings. The mesh is built on a
/// background thread, and the previous one stays on screen until it is done.
#[derive(Default)]
pub struct Supports {
    auto: Arc<SupportSet>,
    manual: Arc<SupportSet>,
    /// The model's raft. Generation sets it from the preset; after that it is
    /// the model's own setting.
    raft: RaftConfig,
    /// The pose the stored coordinates belong to.
    frame: Pose,
    /// The model's current pose.
    pose: Pose,
    /// The generation the model is waiting on, if any. Anything that replaces
    /// the supports in the meantime cancels it.
    pending: Option<u64>,

    resolution: u32,
    mesh: Option<(Mesh, FaceMap)>,
    /// The raft in the shown mesh.
    mesh_raft: Option<Raft>,
    /// The pose the shown mesh was built in.
    mesh_frame: Pose,
    buffers: Option<MeshBuffers>,
    /// The supports changed since the last build started.
    stale: bool,
    building: Option<Build>,
    /// A move removed the supports, and nobody was told yet.
    removed_by_move: bool,
}

/// A mesh being built in the background, for supports stored in `frame`.
struct Build {
    handle: JoinHandle<Built>,
    frame: Pose,
}

struct Built {
    mesh: Option<(Mesh, FaceMap)>,
    raft: Option<Raft>,
}

/// Where a model sits: its position, scale and rotation.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pose {
    pub position: Vector3<f32>,
    pub scale: Vector3<f32>,
    pub rotation: Vector3<f32>,
}

/// The support sets at one point in time, for the undo history. Taking one
/// copies nothing.
#[derive(Clone)]
pub struct SupportsSnapshot {
    auto: Arc<SupportSet>,
    manual: Arc<SupportSet>,
    raft: RaftConfig,
    frame: Pose,
}

impl Pose {
    pub fn of(mesh: &Mesh) -> Self {
        Self {
            position: mesh.position(),
            scale: mesh.scale(),
            rotation: mesh.rotation(),
        }
    }

    /// The motion that carries supports from this pose to `to`, if they
    /// survive it. That needs the same height, tilt and scale; a turn about z
    /// additionally needs equal x and y scale, as the scale is applied after
    /// the rotation.
    pub fn motion_to(&self, to: &Pose) -> Option<Isometry3<f32>> {
        let turn = to.rotation.z - self.rotation.z;
        let rigid = self.position.z == to.position.z
            && self.scale == to.scale
            && self.rotation.xy() == to.rotation.xy()
            && (turn == 0.0 || self.scale.x == self.scale.y);
        rigid.then(|| {
            let rotation = Rotation3::from_axis_angle(&Vector3::z_axis(), turn);
            let shift = to.position - rotation * self.position;
            Isometry3::from_parts(shift.into(), rotation.into())
        })
    }
}

impl Supports {
    pub fn set_resolution(&mut self, resolution: u32) {
        if self.resolution != resolution {
            self.resolution = resolution;
            self.invalidate_cache();
        }
    }

    /// Marks the mesh for a rebuild. The current one stays shown meanwhile.
    pub fn invalidate_cache(&mut self) {
        self.stale = true;
    }

    /// Takes in a finished background build and starts the next one if the
    /// supports changed. Runs once a frame.
    pub fn update(&mut self) {
        if self
            .building
            .as_ref()
            .is_some_and(|b| b.handle.is_finished())
        {
            let build = self.building.take().unwrap();
            if let Ok(built) = build.handle.join() {
                self.show(built, build.frame);
            }
        }
        if self.stale && self.building.is_none() {
            self.stale = false;
            if self.is_empty() {
                self.show(
                    Built {
                        mesh: None,
                        raft: None,
                    },
                    self.frame,
                );
                return;
            }
            let (auto, manual) = (self.auto.clone(), self.manual.clone());
            let (raft, resolution) = (self.raft, self.resolution);
            self.building = Some(Build {
                handle: thread::spawn(move || build(&auto, &manual, &raft, resolution)),
                frame: self.frame,
            });
        }
    }

    /// Whether the shown mesh is of the current supports.
    pub fn is_current(&self) -> bool {
        !self.stale && self.building.is_none()
    }

    /// The mesh of the current supports, waiting for it if need be. For
    /// slicing, which must not use a stale mesh.
    pub fn finished_mesh(&mut self) -> &Option<(Mesh, FaceMap)> {
        if let Some(build) = self.building.take()
            && let Ok(built) = build.handle.join()
        {
            self.show(built, build.frame);
        }
        if self.stale {
            self.stale = false;
            let built = build(&self.auto, &self.manual, &self.raft, self.resolution);
            self.show(built, self.frame);
        }
        &self.mesh
    }

    fn show(&mut self, built: Built, frame: Pose) {
        self.mesh = built.mesh;
        self.mesh_raft = built.raft;
        self.mesh_frame = frame;
        self.buffers = None;
        self.place_mesh();
    }

    /// The model's raft settings.
    pub fn raft(&self) -> RaftConfig {
        self.raft
    }

    pub fn set_raft(&mut self, raft: RaftConfig) {
        if self.raft != raft {
            self.raft = raft;
            self.invalidate_cache();
        }
    }

    pub fn is_empty(&self) -> bool {
        self.auto.supports.is_empty() && self.manual.supports.is_empty()
    }

    /// Removes every support and cancels a running generation.
    pub fn clear(&mut self) {
        self.auto = Arc::default();
        self.manual = Arc::default();
        self.frame = self.pose;
        self.pending = None;
        self.stale = false;
        self.building = None;
        self.show(
            Built {
                mesh: None,
                raft: None,
            },
            self.pose,
        );
    }

    /// Follows the model to `pose`, clearing the supports if they can't
    /// follow.
    pub fn set_pose(&mut self, pose: Pose) {
        self.pose = pose;
        if self.frame.motion_to(&pose).is_none() {
            if !self.is_empty() || self.pending.is_some() {
                self.removed_by_move |= !self.is_empty();
                self.clear();
            }
            self.frame = pose;
            self.mesh_frame = pose;
        }
        self.place_mesh();
    }

    /// Whether a move removed the supports since the last call.
    pub fn take_removed_by_move(&mut self) -> bool {
        std::mem::take(&mut self.removed_by_move)
    }

    /// Brings the stored coordinates to the current pose.
    fn bake(&mut self) {
        if self.frame == self.pose {
            return;
        }
        if let Some(motion) = self.frame.motion_to(&self.pose) {
            Arc::make_mut(&mut self.auto).transform(&motion);
            Arc::make_mut(&mut self.manual).transform(&motion);
        }
        self.frame = self.pose;
    }

    /// Moves the cached mesh from the pose it was built in to the current one.
    fn place_mesh(&mut self) {
        if let Some((mesh, _)) = &mut self.mesh
            && let Some(motion) = self.mesh_frame.motion_to(&self.pose)
        {
            let turn = self.pose.rotation.z - self.mesh_frame.rotation.z;
            mesh.set_rotation(Vector3::z() * turn);
            mesh.set_position(motion.translation.vector);
        }
    }

    pub fn snapshot(&self) -> SupportsSnapshot {
        SupportsSnapshot {
            auto: self.auto.clone(),
            manual: self.manual.clone(),
            raft: self.raft,
            frame: self.frame,
        }
    }

    /// Puts back a snapshot taken at the current pose, cancelling a running
    /// generation.
    pub fn restore_snapshot(&mut self, snapshot: SupportsSnapshot) {
        // An undo moving the model back first is no news.
        self.removed_by_move = false;
        self.auto = snapshot.auto;
        self.manual = snapshot.manual;
        self.raft = snapshot.raft;
        self.frame = snapshot.frame;
        self.pending = None;
        self.invalidate_cache();
        if self.frame.motion_to(&self.pose).is_none() {
            self.clear();
        }
    }

    /// Replaces the stored support sets and raft, for deserialization. The
    /// sets are in world space for the current pose.
    pub fn restore(&mut self, auto: SupportSet, manual: SupportSet, raft: RaftConfig) {
        self.auto = Arc::new(auto);
        self.manual = Arc::new(manual);
        self.raft = raft;
        self.frame = self.pose;
        self.invalidate_cache();
    }

    /// Both sets in world space, for saving.
    pub fn world_sets(&self) -> (SupportSet, SupportSet) {
        let (mut auto, mut manual) = ((*self.auto).clone(), (*self.manual).clone());
        if let Some(motion) = self.frame.motion_to(&self.pose)
            && self.frame != self.pose
        {
            auto.transform(&motion);
            manual.transform(&motion);
        }
        (auto, manual)
    }

    /// The supports with a point (contact or part end) in world space that
    /// `test` accepts.
    pub fn find_by_point(&self, mut test: impl FnMut(Vector3<f32>) -> bool) -> Vec<SupportId> {
        let motion = (self.frame.motion_to(&self.pose)).unwrap_or_else(Isometry3::identity);
        let mut test = |p: &Vector3<f32>| test(motion.transform_point(&(*p).into()).coords);
        (self.auto.supports.iter())
            .chain(self.manual.supports.iter())
            .filter(|s| {
                test(&s.point.pos) || (s.parts.iter()).any(|part| test(&part.a) || test(&part.b))
            })
            .map(|s| s.id)
            .collect()
    }

    /// A support in world space.
    pub fn get(&self, id: SupportId) -> Option<Support> {
        let support = (self.auto.supports.iter())
            .chain(self.manual.supports.iter())
            .find(|s| s.id == id)?;
        let mut set = SupportSet {
            supports: vec![support.clone()],
            ..SupportSet::default()
        };
        if let Some(motion) = self.frame.motion_to(&self.pose) {
            set.transform(&motion);
        }
        set.supports.pop()
    }

    pub fn ids(&self) -> impl Iterator<Item = SupportId> + '_ {
        (self.auto.supports.iter())
            .chain(self.manual.supports.iter())
            .map(|s| s.id)
    }

    pub fn contains(&self, id: SupportId) -> bool {
        (self.auto.supports.iter())
            .chain(self.manual.supports.iter())
            .any(|s| s.id == id)
    }

    /// Marks the model as waiting on generation `token` and returns the pose
    /// the generation runs in.
    pub fn begin_generation(&mut self, token: u64) -> Pose {
        self.pending = Some(token);
        self.pose
    }

    pub fn is_pending(&self, token: u64) -> bool {
        self.pending == Some(token)
    }

    /// Replaces the auto supports with the result of generation `token`, run
    /// with the model at `at`. Returns false, and changes nothing, if the
    /// generation was cancelled or the model has since moved in a way the
    /// supports can't follow.
    ///
    /// `network` is the output of `AutoPlacement::generate_braces` over the
    /// same plate supports, so the pillars are tied together before meshing
    /// and any inverted-V fork truncations are applied to the middle pillars.
    /// The model-landing supports and stabilising legs go after the plate
    /// supports, which keeps the network's support indices valid.
    pub fn finish_generation(
        &mut self,
        token: u64,
        at: Pose,
        config: &SupportPreset,
        (generated, network): (Generated, BraceNetwork),
        mesh: &Mesh,
        bvh: &Bvh,
    ) -> bool {
        if self.pending != Some(token) {
            return false;
        }
        self.pending = None;
        let Some(motion) = at.motion_to(&self.pose) else {
            return false;
        };

        let (tip_radius, radius, depth) = (
            config.tip_radius.get::<Milimeter>(),
            config.radius.get::<Milimeter>(),
            config.contact_depth.get::<Milimeter>(),
        );
        let mut set = SupportSet::default();
        for points in generated.plate {
            set.add_routed(SupportId::new(), tip_radius, radius, depth, points);
        }
        for landing in &generated.landings {
            set.add_landing(SupportId::new(), tip_radius, radius, landing);
        }
        for leg in network.legs {
            set.add_routed(SupportId::new(), tip_radius, radius, depth, leg);
        }
        set.braces = network.braces;
        for (index, bottom_z) in network.fork_bottoms {
            set.truncate_pillar(index, bottom_z);
        }
        set.set_preset(config);
        set.transform(&motion);

        self.bake();
        let old = mem::replace(&mut self.auto, Arc::new(set));
        self.raft = config.raft_config();

        // Hand-placed supports were braced to the old auto supports; brace
        // them to the new ones instead.
        if !self.manual.supports.is_empty() {
            let manual = Arc::make_mut(&mut self.manual);
            manual.remove_braces_on(&old.supports);
            for support in &manual.supports {
                let braces = brace_support(mesh, bvh, support, &self.auto.supports);
                manual.braces.extend(braces);
            }
        }
        self.invalidate_cache();
        true
    }

    /// Adds a hand-placed support, routed like an auto one (to the plate, or
    /// onto the model below), and braces it to its nearest neighbours.
    pub fn add_manual(&mut self, config: &SupportPreset, routed: &Routed, mesh: &Mesh, bvh: &Bvh) {
        self.bake();
        let support = single(config, routed);
        let (auto, manual) = (
            Arc::make_mut(&mut self.auto),
            Arc::make_mut(&mut self.manual),
        );
        insert(auto, manual, false, support, mesh, bvh);
        self.invalidate_cache();
    }

    /// Removes the given supports and every brace that ends on them, in
    /// either set.
    pub fn remove(&mut self, ids: &HashSet<SupportId>) {
        self.bake();
        let (auto, manual) = (
            Arc::make_mut(&mut self.auto),
            Arc::make_mut(&mut self.manual),
        );
        remove(auto, manual, ids);
        self.invalidate_cache();
    }

    /// Re-routes support `id` from a new contact, keeping its id and set, and
    /// braces it into its new neighbours.
    pub fn replace(
        &mut self,
        id: SupportId,
        config: &SupportPreset,
        routed: &Routed,
        mesh: &Mesh,
        bvh: &Bvh,
    ) {
        self.bake();
        let (auto, manual) = (
            Arc::make_mut(&mut self.auto),
            Arc::make_mut(&mut self.manual),
        );
        replace(auto, manual, id, config, routed, mesh, bvh);
        self.invalidate_cache();
    }

    /// The current sets at the current pose, for an edit made on another
    /// thread. Hand the result to [`Self::swap_sets`].
    pub fn edit_base(&mut self) -> (Arc<SupportSet>, Arc<SupportSet>) {
        self.bake();
        (self.auto.clone(), self.manual.clone())
    }

    /// Takes in sets edited from `base`, unless the supports changed since.
    /// Returns whether they were taken.
    pub fn swap_sets(
        &mut self,
        base: &(Arc<SupportSet>, Arc<SupportSet>),
        (auto, manual): (SupportSet, SupportSet),
    ) -> bool {
        let unchanged = Arc::ptr_eq(&self.auto, &base.0)
            && Arc::ptr_eq(&self.manual, &base.1)
            && self.frame == self.pose;
        if unchanged {
            self.auto = Arc::new(auto);
            self.manual = Arc::new(manual);
            self.invalidate_cache();
        }
        unchanged
    }

    /// Meshes what [`Self::add_manual`] would add, for the placement preview.
    pub fn preview_manual(&mut self, config: &SupportPreset, routed: &Routed) -> Option<Mesh> {
        self.bake();
        // The shown raft only lines up with the new support if the model
        // hasn't moved since it was built.
        let raft = self
            .mesh_raft
            .as_ref()
            .filter(|_| self.mesh_frame == self.frame);
        let set = SupportSet {
            supports: vec![single(config, routed)],
            ..SupportSet::default()
        }
        .with_feet(raft);
        build_supports(&set, Transform::default(), self.resolution).map(|(mesh, _)| mesh)
    }

    /// Roughly how many bytes the support sets occupy.
    pub fn memory_size(&self) -> usize {
        self.auto.memory_size() + self.manual.memory_size()
    }

    /// The shown mesh, which may lag the supports by a background build.
    pub fn mesh(&mut self) -> &Option<(Mesh, FaceMap)> {
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

    /// A copy of these supports for a duplicated model.
    pub fn duplicate(&self) -> Self {
        Self {
            auto: self.auto.clone(),
            manual: self.manual.clone(),
            raft: self.raft,
            frame: self.frame,
            pose: self.pose,
            resolution: self.resolution,
            stale: true,
            ..Self::default()
        }
    }
}

/// A support routed as `routed`, built with `config`.
fn single(config: &SupportPreset, routed: &Routed) -> Support {
    let (tip_radius, radius) = (
        config.tip_radius.get::<Milimeter>(),
        config.radius.get::<Milimeter>(),
    );
    let mut set = SupportSet::default();
    match routed {
        Routed::Plate(points) => set.add_routed(
            SupportId::new(),
            tip_radius,
            radius,
            config.contact_depth.get::<Milimeter>(),
            *points,
        ),
        Routed::Model(landing) => set.add_landing(SupportId::new(), tip_radius, radius, landing),
    }
    set.set_preset(config);
    set.supports.pop().unwrap()
}

/// Removes the given supports from either set, and every brace in either set
/// that ends on them.
fn remove(auto: &mut SupportSet, manual: &mut SupportSet, ids: &HashSet<SupportId>) {
    let mut removed = auto.remove(ids);
    removed.extend(manual.remove(ids));
    auto.remove_braces_on(&removed);
    manual.remove_braces_on(&removed);
}

/// Puts `support` into the auto or the manual set, braced to its nearest
/// neighbours in both.
fn insert(
    auto: &mut SupportSet,
    manual: &mut SupportSet,
    into_auto: bool,
    support: Support,
    mesh: &Mesh,
    bvh: &Bvh,
) {
    let others = auto.supports.iter().chain(manual.supports.iter());
    let braces = brace_support(mesh, bvh, &support, others);
    let set = if into_auto { auto } else { manual };
    set.supports.push(support);
    set.braces.extend(braces);
}

/// Re-routes support `id` as `routed` with `config`, keeping its id and set.
pub fn replace(
    auto: &mut SupportSet,
    manual: &mut SupportSet,
    id: SupportId,
    config: &SupportPreset,
    routed: &Routed,
    mesh: &Mesh,
    bvh: &Bvh,
) {
    let into_auto = auto.supports.iter().any(|s| s.id == id);
    if !into_auto && !manual.supports.iter().any(|s| s.id == id) {
        return;
    }
    let mut support = single(config, routed);
    support.id = id;
    remove(auto, manual, &HashSet::from([id]));
    insert(auto, manual, into_auto, support, mesh, bvh);
}

/// The normal of the model surface at a support's contact, found by casting
/// back along the tip direction `tip`. Falls back to the tip direction.
pub fn surface_normal(
    mesh: &Mesh,
    bvh: &Bvh,
    contact: Vector3<f32>,
    tip: Vector3<f32>,
) -> Vector3<f32> {
    let start = contact + tip * 0.1;
    (bvh.intersect_ray(mesh, start, -tip))
        .map(|hit| mesh.transform_normal(&mesh.normal(hit.face)).normalize())
        .unwrap_or(-tip)
}

/// Meshes both sets as one, with their feet: the raft under every plate
/// pillar when the raft is on, else a base each. One set gives every support
/// its own contiguous face range, keyed by id.
fn build(auto: &SupportSet, manual: &SupportSet, raft: &RaftConfig, resolution: u32) -> Built {
    let mut set = SupportSet {
        lift: auto.lift,
        ..SupportSet::default()
    };
    set.supports.extend(auto.supports.iter().cloned());
    set.supports.extend(manual.supports.iter().cloned());
    set.braces = auto.braces.clone();
    set.braces.extend(manual.braces.iter().cloned());

    let feet: Vec<_> = set.plate_feet().collect();
    let raft = Raft::build(raft, &feet);
    let mesh = build_supports(
        &set.with_feet(raft.as_ref()),
        Transform::default(),
        resolution,
    );
    Built { mesh, raft }
}

impl SupportsSnapshot {
    pub fn memory_size(&self) -> usize {
        self.auto.memory_size() + self.manual.memory_size()
    }
}

/// Snapshots ride along with other changes and never decide on their own
/// whether an action changed anything, so any two compare equal.
impl PartialEq for SupportsSnapshot {
    fn eq(&self, _other: &Self) -> bool {
        true
    }
}

impl fmt::Debug for SupportsSnapshot {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SupportsSnapshot")
            .field("auto", &self.auto.supports.len())
            .field("manual", &self.manual.supports.len())
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use std::f32::consts::PI;

    use common::progress::Progress;

    use super::*;

    fn pose(position: [f32; 3], rotation: [f32; 3]) -> Pose {
        Pose {
            position: position.into(),
            scale: Vector3::repeat(1.0),
            rotation: rotation.into(),
        }
    }

    /// One support under (2, 0) with the model at `at`.
    fn supported(at: Pose) -> Supports {
        let mut set = SupportSet::default();
        let joint = Vector3::new(2.0, 0.0, 10.0);
        let contact = joint + Vector3::z();
        set.add_routed(SupportId::new(), 0.2, 0.5, 0.3, [contact, joint, joint]);

        let mut supports = Supports::default();
        supports.set_pose(at);
        supports.restore(set, SupportSet::default(), RaftConfig::default());
        supports
    }

    fn contact(supports: &Supports) -> Vector3<f32> {
        supports.world_sets().0.supports[0].point.pos
    }

    #[test]
    fn plate_moves_and_turns_carry_supports_along() {
        let mut supports = supported(pose([0.0, 0.0, 5.0], [0.3, 0.0, 0.0]));
        supports.set_pose(pose([10.0, 0.0, 5.0], [0.3, 0.0, 0.0]));
        assert!((contact(&supports) - Vector3::new(12.0, 0.0, 11.0)).magnitude() < 1e-5);

        // A quarter turn about z swings the contact around the model origin.
        supports.set_pose(pose([10.0, 0.0, 5.0], [0.3, 0.0, 0.5 * PI]));
        assert!((contact(&supports) - Vector3::new(10.0, 2.0, 11.0)).magnitude() < 1e-5);

        // Edits bake the motion in; the result must not move twice.
        supports.remove(&HashSet::new());
        assert!((contact(&supports) - Vector3::new(10.0, 2.0, 11.0)).magnitude() < 1e-5);
    }

    #[test]
    fn lifting_tilting_or_scaling_clears_supports() {
        let at = pose([0.0, 0.0, 5.0], [0.0; 3]);
        let changes = [
            pose([0.0, 0.0, 6.0], [0.0; 3]),
            pose([0.0, 0.0, 5.0], [0.1, 0.0, 0.0]),
            Pose {
                scale: Vector3::repeat(2.0),
                ..at
            },
        ];
        for change in changes {
            let mut supports = supported(at);
            supports.set_pose(change);
            assert!(supports.is_empty());
            assert!(supports.take_removed_by_move());
            assert!(!supports.take_removed_by_move());
        }
    }

    #[test]
    fn snapshot_brings_back_cleared_supports() {
        let at = pose([0.0, 0.0, 5.0], [0.0; 3]);
        let mut supports = supported(at);
        let snapshot = supports.snapshot();

        supports.set_pose(pose([0.0, 0.0, 9.0], [0.0; 3]));
        assert!(supports.is_empty());

        // Undo: the model goes back first, then the snapshot. That is no
        // news to report.
        supports.set_pose(at);
        supports.restore_snapshot(snapshot);
        assert!(!supports.take_removed_by_move());
        assert!((contact(&supports) - Vector3::new(2.0, 0.0, 11.0)).magnitude() < 1e-5);
    }

    #[test]
    fn replacing_a_support_keeps_its_id_and_records_its_settings() {
        let mut supports = supported(pose([0.0, 0.0, 5.0], [0.0; 3]));
        let id = supports.world_sets().0.supports[0].id;
        let joint = Vector3::new(4.0, 0.0, 10.0);
        let routed = Routed::Plate([joint + Vector3::z(), joint, joint]);

        let mesh = Mesh::new(
            vec![
                Vector3::new(50.0, 50.0, 0.0),
                Vector3::new(51.0, 50.0, 0.0),
                Vector3::new(50.0, 51.0, 0.0),
            ],
            vec![[0, 1, 2]],
        );
        let bvh = Bvh::build(mesh.inner(), Progress::new());
        supports.replace(id, &SupportPreset::HEAVY, &routed, &mesh, &bvh);
        let support = supports.get(id).unwrap();
        assert_eq!(support.preset, SupportPreset::HEAVY);
        assert_eq!(support.point.pos, joint + Vector3::z());
        assert_eq!(supports.world_sets().0.supports.len(), 1);
    }

    #[test]
    fn cancelled_generation_is_dropped() {
        let at = pose([0.0, 0.0, 5.0], [0.0; 3]);
        let mut supports = supported(at);
        supports.begin_generation(1);
        let snapshot = supports.snapshot();
        supports.restore_snapshot(snapshot);
        assert!(!supports.is_pending(1));
    }
}
