//! A model's hollowing: the wall it keeps and the holes drilled into it, and
//! the cavity computed from them.
//!
//! The imported mesh is never changed. What prints is derived from it: the
//! *print mesh* is the model's surface plus the cavity's wall facing into the
//! cavity, with the holes drilled into it, and is what slicing, export and
//! the island check use. The cavity and the drilling are computed in the
//! background. Holes go through the wall into the cavity, so only a hollowed
//! model has them; one that can't be drilled is left out (and marked as
//! such).

use std::{
    sync::{Arc, Mutex, OnceLock, PoisonError},
    time::Instant,
};

use common::serde::{Deserializer, SerdeExt, Serializer};
use nalgebra::{Matrix3, Vector3};
use slicer::geometry::{
    bounds::Bounds3D,
    mesh::{Mesh, MeshId, MeshInner},
    partitioning::Bvh,
};
use tools::hollow::{self, Cavity, CavityPart, through_depth};

use crate::engine::{CancelToken, Debounce, Slot, SlotState, Ticket};

/// The wall kept when hollowing.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Shell {
    /// Wall thickness (mm).
    pub wall: f32,
    /// Sample spacing the cavity is computed at (mm).
    pub cell: f32,
}

/// A drain hole, in the model's own coordinates so it moves with it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hole {
    pub point: Vector3<f32>,
    /// Unit direction of the drill, into the model.
    pub direction: Vector3<f32>,
    /// Diameter (mm).
    pub diameter: f32,
}

/// What the user set: saved with the project and kept in the undo history.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct HollowSpec {
    pub shell: Option<Shell>,
    pub holes: Vec<Hole>,
}

/// A model's hollowing and its computed cavity.
///
/// Two evaluation nodes, each an engine [`Slot`]: the cavity (from the
/// shell, the mesh and the placement's [`Shape`]) and the drilled print mesh
/// (from the cavity and the holes).
#[derive(Clone, Default)]
pub struct Hollow {
    pub spec: HollowSpec,
    /// The cavity, in the model's coordinates. The last one computed stays
    /// (as [`Slot::latest`]) to show while a new one is computed.
    pub cavity: Slot<CavityKey, ModelCavity>,
    /// Waits for the shell and scale to settle before asking for a cavity.
    pub settle: Debounce<CavityKey>,
    /// The print mesh with the holes drilled in.
    pub drilled: Slot<DrillKey, Drilled>,
    /// The Hollow row is frozen: the last cavity and drilling are kept
    /// whatever changes, and nothing is recomputed while there are some.
    pub frozen: bool,
    /// The auto-hole rule, from the Hollow row: each void is vented at its
    /// lowest point in the current orientation, with holes of this diameter.
    pub auto_rule: Option<f32>,
    /// The auto holes last found, and what for.
    auto: AutoHolesCache,
    /// The print mesh's hit-testing structure, once built, and which print
    /// mesh it is for.
    pub print_bvh: Option<(MeshId, Arc<Bvh>)>,
}

/// The print mesh with the holes drilled in. What it was drilled for is the
/// key it is cached under ([`DrillKey`]).
pub struct Drilled {
    /// In the model's coordinates, like its mesh.
    pub mesh: Arc<MeshInner>,
    /// For each hole, why it couldn't be drilled, if it couldn't: such a hole
    /// is left out.
    pub failed: Vec<Option<String>>,
}

/// What a model's placement does to its shape, and nothing else.
///
/// Each stage derived from a model (the cavity, the drilled print mesh) keeps
/// what it was computed from and is reused while that still holds, so an
/// interaction only redoes the stages it affects. From the placement, a
/// stage depends on this: moving a model leaves it alone, and so does
/// turning one scaled evenly, but scaling changes it, and so does turning a
/// model scaled unevenly (the scale is along the world's axes, after the
/// turn).
///
/// It is `Lᵀ·L` for the placement's linear part `L`: two placements have
/// the same one exactly when they differ by a move and a turn.
#[derive(Clone, Copy, Debug)]
pub struct Shape(Matrix3<f32>);

impl Shape {
    pub fn of(mesh: &Mesh) -> Self {
        let linear = mesh
            .transformation_matrix()
            .fixed_view::<3, 3>(0, 0)
            .into_owned();
        Self(linear.transpose() * linear)
    }
}

impl PartialEq for Shape {
    fn eq(&self, other: &Self) -> bool {
        let size = self.0.abs().max().max(other.0.abs().max());
        (self.0 - other.0).abs().max() <= size * 1e-5
    }
}

/// The auto-hole node's cache: the cavity (by identity), the linear part of
/// the placement and the diameter it is for, and the holes.
#[derive(Default)]
struct AutoHolesCache(Mutex<Option<AutoHolesEntry>>);

/// The cavity's address, the placement's linear part and the diameter (as
/// bits), and the auto holes found for them, with how far below the model's
/// position its lowest point and each hole's mouth are.
type AutoHolesEntry = (usize, [u32; 9], u32, Arc<Vec<Hole>>, AutoHeights);

/// The model's lowest point and each auto hole's mouth, as heights relative
/// to the model's position (they depend on the orientation only).
#[derive(Clone, Default)]
struct AutoHeights {
    lowest: f32,
    mouths: Vec<f32>,
}

/// A model whose lowest point is this close to the plate stands on it.
const ON_PLATE: f32 = 0.05;
/// A hole whose mouth is this close to the model's lowest point would be
/// drilled through the contact with the plate.
const AT_CONTACT: f32 = 0.2;

impl Clone for AutoHolesCache {
    fn clone(&self) -> Self {
        Self(Mutex::new(
            self.0
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .clone(),
        ))
    }
}

/// A void as it lies now: its lowest point (world space) and its box.
#[derive(Clone, Copy, Debug)]
pub struct PlacedVoid {
    pub lowest: Vector3<f32>,
    pub min: Vector3<f32>,
    pub max: Vector3<f32>,
}

/// What a cavity is computed from; changing any of it calls for a new one.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CavityKey {
    pub shell: Shell,
    pub shape: Shape,
    pub mesh: MeshId,
}

/// What holes are drilled from: the cavity (by identity), the holes and the
/// placement's shape.
#[derive(Clone)]
pub struct DrillKey {
    pub cavity: Arc<ModelCavity>,
    pub holes: Vec<Hole>,
    pub shape: Shape,
}

impl PartialEq for DrillKey {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.cavity, &other.cavity)
            && self.holes == other.holes
            && self.shape == other.shape
    }
}

/// A cavity in the model's coordinates, and what it was computed for.
pub struct ModelCavity {
    pub shell: Shell,
    /// The model's mesh it was computed from. Only held, so the mesh id in
    /// the cavity's key can't be reused by another mesh while it is cached.
    _source: Arc<MeshInner>,
    pub mesh: Mesh,
    pub parts: Vec<CavityPart>,
    /// Suggested drain holes, one per part.
    pub drains: Vec<Hole>,
    /// The print mesh, once asked for.
    print: OnceLock<Arc<MeshInner>>,
}

impl Default for Shell {
    fn default() -> Self {
        Self {
            wall: 2.0,
            cell: 0.5,
        }
    }
}

impl Hole {
    /// The hole as placed with the model, in world space: through a wall of
    /// `shell` into the cavity.
    pub fn world(&self, mesh: &Mesh, shell: Shell) -> hollow::Hole {
        let point = mesh.transform(&self.point);
        let ahead = mesh.transform(&(self.point + self.direction));
        hollow::Hole {
            point,
            direction: (ahead - point).normalize(),
            diameter: self.diameter,
            depth: through_depth(shell.wall),
        }
    }

    /// The hole in `mesh`'s coordinates, from one in world space.
    pub fn local(hole: &hollow::Hole, mesh: &Mesh) -> Self {
        let point = mesh.inv_transform(&hole.point);
        let ahead = mesh.inv_transform(&(hole.point + hole.direction));
        Self {
            point,
            direction: (ahead - point).normalize(),
            diameter: hole.diameter,
        }
    }

    pub fn serialize<T: Serializer>(&self, ser: &mut T) {
        self.point.serialize(ser);
        self.direction.serialize(ser);
        ser.write_f32_be(self.diameter);
    }

    pub fn deserialize<T: Deserializer>(des: &mut T) -> Self {
        let point = Vector3::<f32>::deserialize(des);
        let direction = Vector3::<f32>::deserialize(des);
        let diameter = des.read_f32_be();
        Self {
            point,
            direction,
            diameter,
        }
    }
}

impl HollowSpec {
    pub fn serialize<T: Serializer>(&self, ser: &mut T) {
        ser.write_bool(self.shell.is_some());
        let shell = self.shell.unwrap_or_default();
        ser.write_f32_be(shell.wall);
        ser.write_f32_be(shell.cell);
        ser.write_u32_be(self.holes.len() as u32);
        self.holes.iter().for_each(|h| h.serialize(ser));
    }

    pub fn deserialize<T: Deserializer>(des: &mut T) -> Self {
        let hollow = des.read_bool();
        let wall = des.read_f32_be();
        let cell = des.read_f32_be();
        let holes = des.read_list(|des| Hole::deserialize(des));
        Self {
            shell: hollow.then_some(Shell { wall, cell }),
            holes,
        }
    }
}

impl Hollow {
    /// What the cavity has to be computed from now, if the model is hollowed.
    pub fn key(&self, mesh: &Mesh) -> Option<CavityKey> {
        Some(CavityKey {
            shell: self.spec.shell?,
            shape: Shape::of(mesh),
            mesh: mesh.mesh_id(),
        })
    }

    /// The cavity, if it is up to date with the shell, the scale and the mesh.
    pub fn cavity(&self, mesh: &Mesh) -> Option<&ModelCavity> {
        self.cavity_arc(mesh).map(|c| &**c)
    }

    fn cavity_arc(&self, mesh: &Mesh) -> Option<&Arc<ModelCavity>> {
        let key = self.key(mesh)?;
        match self.frozen {
            true => self.cavity.latest(),
            false => self.cavity.get(&key),
        }
    }

    /// Stores a cavity computed for the model as it is now.
    #[cfg(test)]
    pub fn insert_cavity(&mut self, mesh: &Mesh, cavity: ModelCavity) {
        if let Some(key) = self.key(mesh) {
            self.cavity.insert(key, Arc::new(cavity));
        }
    }

    /// Called every frame: stops a run whose inputs changed, and once the
    /// inputs have settled and nothing runs, hands out the ticket to compute
    /// the cavity for them.
    pub fn poll(&mut self, mesh: &Mesh, now: Instant) -> Option<Ticket<CavityKey>> {
        if self.frozen && self.cavity.latest().is_some() {
            return None;
        }
        let Some(key) = self.key(mesh) else {
            self.cavity.withdraw();
            self.settle.reset();
            return None;
        };
        self.cavity.supersede(key);
        if self.cavity.get(&key).is_some() {
            self.settle.reset();
            return None;
        }
        match self.settle.settled(key, now) {
            true => self.cavity.request(key),
            false => None,
        }
    }

    /// The voids as the model is placed now: lowest point and box, in world
    /// space. The lowest point is where resin collects, so it moves with the
    /// orientation.
    pub fn placed_voids(&self, mesh: &Mesh) -> Vec<PlacedVoid> {
        let Some(cavity) = self.cavity(mesh) else {
            return Vec::new();
        };
        let vertices = cavity.mesh.vertices();
        (cavity.parts.iter())
            .map(|part| {
                // The range comes from the project file.
                let part_vertices = vertices.get(part.vertices.clone()).unwrap_or_default();
                let world: Vec<_> = (part_vertices.iter()).map(|v| mesh.transform(v)).collect();
                let (min, max) = bounds(&world);
                PlacedVoid {
                    lowest: lowest_middle(&world, min.z + cavity.shell.cell),
                    min,
                    max,
                }
            })
            .collect()
    }

    /// The auto holes (model space): with the auto-hole rule on, one per
    /// void, drilled straight up through the floor below its lowest point in
    /// the current orientation. Recomputed only when the orientation, scale
    /// or cavity changes; moving the model keeps them. On a model standing
    /// on the plate, a hole that would go through its contact with the plate
    /// is left out (the void then shows as sealed).
    pub fn auto_holes(&self, mesh: &Mesh) -> Arc<Vec<Hole>> {
        let (holes, heights) = self.auto_holes_any_height(mesh);
        let z = mesh.position().z;
        let standing = z + heights.lowest < ON_PLATE;
        let contact = |&mouth: &f32| mouth < heights.lowest + AT_CONTACT;
        if !standing || !heights.mouths.iter().any(contact) {
            return holes;
        }
        let kept = (holes.iter().zip(&heights.mouths))
            .filter(|(_, mouth)| !contact(mouth))
            .map(|(hole, _)| *hole);
        Arc::new(kept.collect())
    }

    fn auto_holes_any_height(&self, mesh: &Mesh) -> (Arc<Vec<Hole>>, AutoHeights) {
        let (Some(diameter), Some(cavity)) = (self.auto_rule, self.cavity_arc(mesh)) else {
            return Default::default();
        };
        let linear = mesh
            .transformation_matrix()
            .fixed_view::<3, 3>(0, 0)
            .into_owned();
        let linear: [u32; 9] = std::array::from_fn(|i| linear.as_slice()[i].to_bits());
        let key = (Arc::as_ptr(cavity) as usize, linear, diameter.to_bits());
        let mut cache = self.auto.0.lock().unwrap_or_else(PoisonError::into_inner);
        if let Some((ptr, lin, d, holes, heights)) = cache.as_ref()
            && (*ptr, *lin, *d) == key
        {
            return (holes.clone(), heights.clone());
        }
        let z = mesh.position().z;
        let mut heights = AutoHeights {
            lowest: Bounds3D::from_mesh(mesh).min.z - z,
            mouths: Vec::new(),
        };
        let holes: Vec<Hole> = (self.placed_voids(mesh).iter())
            .filter_map(|void| {
                // Straight down from the void's lowest point to the outside.
                let origin = mesh.inv_transform(&void.lowest);
                let down = mesh.inv_transform(&(void.lowest - Vector3::z())) - origin;
                let t = first_hit(mesh, origin, down.normalize())?;
                let surface = mesh.transform(&(origin + down.normalize() * t));
                let depth = void.lowest.z - surface.z;
                heights.mouths.push(surface.z - z);
                Some(Hole::local(
                    &hollow::Hole {
                        point: surface,
                        direction: Vector3::z(),
                        diameter,
                        depth,
                    },
                    mesh,
                ))
            })
            .collect();
        let holes = Arc::new(holes);
        *cache = Some((key.0, key.1, key.2, holes.clone(), heights.clone()));
        (holes, heights)
    }

    /// The holes that are drilled: the manual ones, then the auto ones.
    pub fn all_holes(&self, mesh: &Mesh) -> Vec<Hole> {
        let mut holes = self.spec.holes.clone();
        holes.extend(self.auto_holes(mesh).iter().copied());
        holes
    }

    /// The states of the hollowing's evaluation nodes.
    pub fn slot_states(&self) -> [SlotState; 2] {
        [self.cavity.state(), self.drilled.state()]
    }

    /// The hit-testing structure of `print` (from [`Hollow::print_mesh`]),
    /// if built.
    pub fn print_bvh(&self, print: &Mesh) -> Option<Arc<Bvh>> {
        let (id, bvh) = self.print_bvh.as_ref()?;
        (*id == print.mesh_id()).then(|| bvh.clone())
    }

    /// Whether everything the model's slicing needs is ready.
    pub fn ready(&self, mesh: &Mesh) -> bool {
        self.spec.shell.is_none() || (self.cavity(mesh).is_some() && !self.wants_drilling(mesh))
    }

    /// Whether holes are to be drilled into the print mesh: the model is
    /// hollowed, its cavity is up to date, and the holes drilled last aren't
    /// these.
    pub fn wants_drilling(&self, mesh: &Mesh) -> bool {
        let Some(cavity) = self.cavity(mesh) else {
            return false;
        };
        !cavity.parts.is_empty() && !self.all_holes(mesh).is_empty() && self.drilled(mesh).is_none()
    }

    /// The holes drilled into the print mesh, if they are the model's holes
    /// as it is now.
    pub fn drilled(&self, mesh: &Mesh) -> Option<&Drilled> {
        let key = self.drill_key(mesh)?;
        match self.frozen {
            true => self.drilled.latest().map(|d| &**d),
            false => self.drilled.get(&key).map(|d| &**d),
        }
    }

    /// What the print mesh's holes are drilled from now, once the cavity is
    /// up to date.
    pub fn drill_key(&self, mesh: &Mesh) -> Option<DrillKey> {
        Some(DrillKey {
            cavity: self.cavity_arc(mesh)?.clone(),
            holes: self.all_holes(mesh),
            shape: Shape::of(mesh),
        })
    }

    /// Stores holes drilled for the model as it is now.
    #[cfg(test)]
    pub fn insert_drilled(&mut self, mesh: &Mesh, drilled: Drilled) {
        if let Some(key) = self.drill_key(mesh) {
            self.drilled.insert(key, Arc::new(drilled));
        }
    }

    /// Called every frame: keeps the drilling node's wanted inputs current,
    /// and hands out the ticket to drill once there is something to drill
    /// and nothing runs. With `hold` (a hole is being dragged) nothing
    /// starts.
    pub fn poll_drilling(
        &mut self,
        mesh: &Mesh,
        hold: bool,
    ) -> Option<(Ticket<DrillKey>, DrillInput)> {
        if self.frozen && self.drilled.latest().is_some() {
            return None;
        }
        let Some(key) = self.drill_key(mesh) else {
            self.drilled.withdraw();
            return None;
        };
        if !self.wants_drilling(mesh) {
            self.drilled.supersede(key);
            return None;
        }
        if hold {
            self.drilled.supersede(key);
            return None;
        }
        let input = self.drill_input(mesh)?;
        self.drilled.request(key).map(|ticket| (ticket, input))
    }

    /// The holes to drill, as `Drilled::drill` takes them.
    pub fn drill_input(&self, mesh: &Mesh) -> Option<DrillInput> {
        let cavity = self.cavity_arc(mesh)?.clone();
        let shell = cavity.shell;
        Some(DrillInput {
            print: self.undrilled(&cavity, mesh),
            world: (self.all_holes(mesh).iter())
                .map(|h| h.world(mesh, shell))
                .collect(),
            mesh: placed(mesh),
        })
    }

    /// The mesh as it prints: the model's surface plus, when hollowed, the
    /// cavity's wall facing into the cavity, with the holes drilled in once
    /// they are. Placed like `mesh`. Without an up-to-date cavity, `mesh`
    /// itself.
    pub fn print_mesh(&self, mesh: &Mesh) -> Mesh {
        let inner = match (self.drilled(mesh), self.cavity_arc(mesh)) {
            (Some(drilled), _) => drilled.mesh.clone(),
            (None, Some(cavity)) if !cavity.parts.is_empty() => self.undrilled(cavity, mesh),
            _ => return mesh.clone(),
        };
        let mut print = Mesh::from_inner(inner);
        print.set_position_unchecked(mesh.position());
        print.set_scale_unchecked(mesh.scale());
        print.set_rotation_unchecked(mesh.rotation());
        print.update_transformation_matrix();
        print
    }

    /// The model's surface plus the cavity's wall facing into the cavity, in
    /// the model's coordinates.
    fn undrilled(&self, cavity: &ModelCavity, mesh: &Mesh) -> Arc<MeshInner> {
        let inner = cavity.print.get_or_init(|| {
            let (outer, inside) = (mesh.inner(), &cavity.mesh);
            let offset = outer.vertices.len() as u32;
            let vertices = (outer.vertices.iter())
                .chain(inside.vertices())
                .copied()
                .collect();
            // The cavity's faces point into the walls; the material's surface
            // there points into the cavity.
            let faces = (outer.faces.iter().copied())
                .chain(
                    inside
                        .faces()
                        .iter()
                        .map(|f| [f[0], f[2], f[1]].map(|i| i + offset)),
                )
                .collect();
            Mesh::new(vertices, faces).inner().clone()
        });
        inner.clone()
    }

    /// Whether hole `index` couldn't be drilled, as far as is known yet.
    pub fn failed(&self, mesh: &Mesh, index: usize) -> bool {
        (self.drilled(mesh)).is_some_and(|d| d.failed.get(index).is_some_and(Option::is_some))
    }

    /// The parts of the cavity no hole reaches into: resin stays trapped in
    /// them. Holes that couldn't be drilled don't count.
    pub fn sealed_parts(&self, mesh: &Mesh) -> usize {
        let Some(cavity) = self.cavity(mesh) else {
            return 0;
        };
        let world: Vec<_> = (self.all_holes(mesh).iter().enumerate())
            .filter(|(index, _)| !self.failed(mesh, *index))
            .map(|(_, h)| h.world(mesh, cavity.shell))
            .collect();
        let margin = Vector3::repeat(cavity.shell.cell * 2.0);
        (cavity.parts.iter())
            .filter(|part| {
                let (min, max) = world_box(mesh, part);
                let (min, max) = (min - margin, max + margin);
                !world.iter().any(|h| {
                    let end = h.point + h.direction * h.depth;
                    (min.zip_map(&end, |a, b| a <= b) == Vector3::repeat(true))
                        && (end.zip_map(&max, |a, b| a <= b) == Vector3::repeat(true))
                })
            })
            .count()
    }
}

impl ModelCavity {
    /// Moves a cavity computed in world space with `mesh` into its coordinates.
    pub fn new(cavity: Cavity, mesh: &Mesh, shell: Shell) -> Self {
        let vertices = cavity.vertices.iter().map(|v| mesh.inv_transform(v));
        let local = |p: &Vector3<f32>| mesh.inv_transform(p);
        let parts = (cavity.parts.iter())
            .map(|part| {
                let corners = corners(part.min, part.max).map(|c| local(&c));
                let (min, max) = bounds(&corners);
                CavityPart {
                    volume: part.volume,
                    lowest: local(&part.lowest),
                    min,
                    max,
                    vertices: part.vertices.clone(),
                }
            })
            .collect();
        Self {
            shell,
            _source: mesh.inner().clone(),
            mesh: Mesh::new(vertices.collect(), cavity.faces),
            parts,
            drains: (cavity.drains.iter())
                .map(|h| Hole::local(h, mesh))
                .collect(),
            print: OnceLock::new(),
        }
    }

    /// The total volume of the voids (mm³).
    pub fn volume(&self) -> f32 {
        self.parts.iter().map(|p| p.volume).sum()
    }
}

/// The frozen results of a Hollow row, as saved with the project.
pub struct FrozenHollow {
    pub cavity: Arc<ModelCavity>,
    pub drilled: Option<Arc<Drilled>>,
}

fn write_inner<T: Serializer>(ser: &mut T, mesh: &MeshInner) {
    ser.write_u32_be(mesh.vertices.len() as u32);
    mesh.vertices.iter().for_each(|v| v.serialize(ser));
    ser.write_u32_be(mesh.faces.len() as u32);
    for face in mesh.faces.iter() {
        face.iter().for_each(|i| ser.write_u32_be(*i));
    }
}

fn read_inner<T: Deserializer>(des: &mut T) -> MeshInner {
    let vertices = des.read_list(|des| Vector3::<f32>::deserialize(des));
    let faces = des.read_list(|des| [des.read_u32_be(), des.read_u32_be(), des.read_u32_be()]);
    valid_mesh(vertices, faces)
}

/// A mesh from a file, without the faces that point past its vertices (a
/// damaged file): everything after would index out of bounds.
pub fn valid_mesh(vertices: Vec<Vector3<f32>>, mut faces: Vec<[u32; 3]>) -> MeshInner {
    let count = vertices.len();
    faces.retain(|face| face.iter().all(|&v| (v as usize) < count));
    MeshInner {
        vertices: vertices.into(),
        faces: faces.into(),
    }
}

impl FrozenHollow {
    /// The frozen results of a model's hollowing, if the row is frozen and
    /// has some.
    pub fn of(hollow: &Hollow) -> Option<Self> {
        if !hollow.frozen {
            return None;
        }
        Some(Self {
            cavity: hollow.cavity.latest()?.clone(),
            drilled: hollow.drilled.latest().cloned(),
        })
    }

    pub fn serialize<T: Serializer>(&self, ser: &mut T) {
        let cavity = &self.cavity;
        ser.write_f32_be(cavity.shell.wall);
        ser.write_f32_be(cavity.shell.cell);
        write_inner(ser, cavity.mesh.inner());
        ser.write_u32_be(cavity.parts.len() as u32);
        for part in &cavity.parts {
            ser.write_f32_be(part.volume);
            part.lowest.serialize(ser);
            part.min.serialize(ser);
            part.max.serialize(ser);
            ser.write_u32_be(part.vertices.start as u32);
            ser.write_u32_be(part.vertices.end as u32);
        }
        ser.write_u32_be(cavity.drains.len() as u32);
        cavity.drains.iter().for_each(|h| h.serialize(ser));
        ser.write_bool(self.drilled.is_some());
        if let Some(drilled) = &self.drilled {
            write_inner(ser, &drilled.mesh);
            ser.write_u32_be(drilled.failed.len() as u32);
            for failed in &drilled.failed {
                let text = failed.as_deref().unwrap_or_default();
                ser.write_bool(failed.is_some());
                ser.write_u32_be(text.len() as u32);
                ser.write_bytes(text.as_bytes());
            }
        }
    }

    pub fn deserialize<T: Deserializer>(des: &mut T, source: Arc<MeshInner>) -> Self {
        let shell = Shell {
            wall: des.read_f32_be(),
            cell: des.read_f32_be(),
        };
        let mesh = Mesh::from_inner(Arc::new(read_inner(des)));
        let parts = des.read_list(|des| CavityPart {
            volume: des.read_f32_be(),
            lowest: Vector3::<f32>::deserialize(des),
            min: Vector3::<f32>::deserialize(des),
            max: Vector3::<f32>::deserialize(des),
            vertices: des.read_u32_be() as usize..des.read_u32_be() as usize,
        });
        let drains = des.read_list(|des| Hole::deserialize(des));
        let cavity = Arc::new(ModelCavity {
            shell,
            _source: source,
            mesh,
            parts,
            drains,
            print: OnceLock::new(),
        });
        let drilled = des.read_bool().then(|| {
            let mesh = Arc::new(read_inner(des));
            let failed = des.read_list(|des| {
                let some = des.read_bool();
                let len = des.read_u32_be() as usize;
                let text = String::from_utf8_lossy(&des.read_bytes(len)).into_owned();
                some.then_some(text)
            });
            Arc::new(Drilled { mesh, failed })
        });
        Self { cavity, drilled }
    }

    /// Puts the frozen results back as the model's latest (which is what a
    /// frozen row uses).
    pub fn restore(self, hollow: &mut Hollow, mesh: &Mesh) {
        let key = CavityKey {
            shell: self.cavity.shell,
            shape: Shape::of(mesh),
            mesh: mesh.mesh_id(),
        };
        let cavity = self.cavity.clone();
        hollow.cavity.insert(key, self.cavity);
        if let Some(drilled) = self.drilled {
            let key = DrillKey {
                cavity,
                holes: hollow.spec.holes.clone(),
                shape: Shape::of(mesh),
            };
            hollow.drilled.insert(key, drilled);
        }
        hollow.frozen = true;
    }
}

pub fn write_mesh_inner<T: Serializer>(ser: &mut T, mesh: &MeshInner) {
    write_inner(ser, mesh);
}

pub fn read_mesh_inner<T: Deserializer>(des: &mut T) -> MeshInner {
    read_inner(des)
}

/// What drilling holes into a print mesh takes, gathered on the main thread.
pub struct DrillInput {
    /// The print mesh without holes, in the model's coordinates.
    print: Arc<MeshInner>,
    /// The holes as placed.
    world: Vec<hollow::Hole>,
    /// Where the model is.
    mesh: Mesh,
}

impl Drilled {
    /// [`Drilled::drill`], unless `cancel` is set before it starts.
    pub fn drill_unless(input: DrillInput, cancel: &CancelToken) -> Option<Self> {
        cancel.check().ok()?;
        Some(Self::drill(input))
    }

    /// Drills the holes into the print mesh. Holes are round in world space,
    /// so this happens there (but for the model's position), and the result
    /// is brought back into the model's coordinates.
    pub fn drill(input: DrillInput) -> Self {
        let DrillInput { print, world, mesh } = input;
        let at = mesh.position();
        let vertices: Vec<_> = (print.vertices.iter())
            .map(|v| mesh.transform(v) - at)
            .collect();
        let world: Vec<_> = (world.into_iter())
            .map(|h| hollow::Hole {
                point: h.point - at,
                ..h
            })
            .collect();
        let ((vertices, faces), results) = hollow::drill_each(&vertices, &print.faces, &world);
        let vertices = vertices
            .iter()
            .map(|v| mesh.inv_transform(&(v + at)))
            .collect();
        Self {
            mesh: Mesh::new(vertices, faces).inner().clone(),
            failed: results.into_iter().map(Result::err).collect(),
        }
    }
}

/// A mesh placed like `mesh`, without its geometry.
fn placed(mesh: &Mesh) -> Mesh {
    let mut placed = Mesh::new(Vec::new(), Vec::new());
    placed.set_position_unchecked(mesh.position());
    placed.set_scale_unchecked(mesh.scale());
    placed.set_rotation_unchecked(mesh.rotation());
    placed.update_transformation_matrix();
    placed
}

/// Of the points lower than `below`, the one nearest their middle, so a flat
/// floor drains from its centre (as `tools::hollow` picks drains).
fn lowest_middle(points: &[Vector3<f32>], below: f32) -> Vector3<f32> {
    let low: Vec<_> = points.iter().filter(|p| p.z <= below).collect();
    if low.is_empty() {
        return Vector3::zeros();
    }
    let middle = low.iter().fold(Vector3::zeros(), |sum, p| sum + **p) / low.len() as f32;
    let flat = |p: &Vector3<f32>| (p - middle).xy().norm_squared();
    (low.iter())
        .min_by(|a, b| flat(a).total_cmp(&flat(b)))
        .map_or(middle, |p| **p)
}

/// Distance along `direction` (unit, model space) to the nearest face of
/// `mesh` from `origin`, either side facing.
pub fn first_hit(mesh: &Mesh, origin: Vector3<f32>, direction: Vector3<f32>) -> Option<f32> {
    let vertices = mesh.vertices();
    let mut best: Option<f32> = None;
    for face in mesh.faces() {
        let [a, b, c] = face.map(|i| vertices[i as usize]);
        let (e1, e2) = (b - a, c - a);
        let p = direction.cross(&e2);
        let det = e1.dot(&p);
        if det.abs() < 1e-12 {
            continue;
        }
        let inv = 1.0 / det;
        let s = origin - a;
        let u = s.dot(&p) * inv;
        if !(0.0..=1.0).contains(&u) {
            continue;
        }
        let q = s.cross(&e1);
        let v = direction.dot(&q) * inv;
        if v < 0.0 || u + v > 1.0 {
            continue;
        }
        let t = e2.dot(&q) * inv;
        if t > 1e-6 && best.is_none_or(|b| t < b) {
            best = Some(t);
        }
    }
    best
}

fn corners(min: Vector3<f32>, max: Vector3<f32>) -> [Vector3<f32>; 8] {
    std::array::from_fn(|i| {
        Vector3::new(
            if i & 1 == 0 { min.x } else { max.x },
            if i & 2 == 0 { min.y } else { max.y },
            if i & 4 == 0 { min.z } else { max.z },
        )
    })
}

fn bounds(points: &[Vector3<f32>]) -> (Vector3<f32>, Vector3<f32>) {
    points.iter().fold(
        (Vector3::repeat(f32::MAX), Vector3::repeat(f32::MIN)),
        |(lo, hi), p| (lo.inf(p), hi.sup(p)),
    )
}

/// A part's box in world space (loose: the box of its turned corners).
fn world_box(mesh: &Mesh, part: &CavityPart) -> (Vector3<f32>, Vector3<f32>) {
    bounds(&corners(part.min, part.max).map(|c| mesh.transform(&c)))
}

#[cfg(test)]
mod tests {
    use common::{progress::Progress, slice::SliceConfig, units::Milimeters};
    use nalgebra::{Vector2, Vector3};
    use slicer::{
        geometry::{
            bounds::Bounds3D,
            mesh::{Mesh, MeshBuilder},
        },
        slicer::{Slicer, SlicerModel},
    };
    use tools::hollow::cavity;

    use super::{Drilled, Hole, Hollow, HollowSpec, ModelCavity, Shell};

    fn hollowed(spec: HollowSpec, cavity: ModelCavity, mesh: &Mesh) -> Hollow {
        let mut hollow = Hollow {
            spec,
            ..Default::default()
        };
        hollow.insert_cavity(mesh, cavity);
        hollow
    }

    #[test]
    fn spec_round_trips() {
        use common::serde::{DynamicSerializer, SliceDeserializer};
        let spec = HollowSpec {
            shell: Some(Shell {
                wall: 1.5,
                cell: 0.3,
            }),
            holes: vec![
                Hole {
                    point: Vector3::new(1.0, 2.0, 3.0),
                    direction: Vector3::z(),
                    diameter: 2.5,
                },
                Hole {
                    point: Vector3::new(-4.0, 0.5, 9.0),
                    direction: -Vector3::x(),
                    diameter: 1.0,
                },
            ],
        };
        for spec in [spec, HollowSpec::default()] {
            let mut ser = DynamicSerializer::new();
            spec.serialize(&mut ser);
            let data = ser.into_inner();
            let back = HollowSpec::deserialize(&mut SliceDeserializer::new(&data));
            assert_eq!(back, spec);
        }
    }

    /// A 20 mm cube standing on the plate, centred.
    fn cube() -> Mesh {
        let mut builder = MeshBuilder::new();
        let mut v = |x: f32, y: f32, z: f32| builder.add_vertex(Vector3::new(x, y, z) * 20.0);
        let c = [
            v(0., 0., 0.),
            v(1., 0., 0.),
            v(1., 1., 0.),
            v(0., 1., 0.),
            v(0., 0., 1.),
            v(1., 0., 1.),
            v(1., 1., 1.),
            v(0., 1., 1.),
        ];
        for [i, j, k] in [
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [1, 2, 6],
            [1, 6, 5],
            [2, 3, 7],
            [2, 7, 6],
            [3, 0, 4],
            [3, 4, 7],
        ] {
            builder.add_face([c[i], c[j], c[k]]);
        }
        let mut mesh = builder.build();
        mesh.set_position(Vector3::new(-10.0, -10.0, 0.0));
        mesh
    }

    /// Lit pixels of each layer, slicing at 5 px/mm with the runner's
    /// transform into platform space.
    fn slice(mesh: &Mesh, hollow: &Hollow) -> Vec<u64> {
        let config = SliceConfig {
            platform_resolution: Vector2::new(200, 200),
            platform_size: Vector3::new(40.0, 40.0, 30.0).map(Milimeters::new),
            ..Default::default()
        };
        let mm_to_px = Vector3::new(5.0, 5.0, 1.0);
        let offset = Vector3::new(100.0, 100.0, -config.slice_height.raw() / 2.0);
        let to_platform = |mut mesh: Mesh| {
            mesh.set_scale_unchecked(mesh.scale().component_mul(&mm_to_px));
            mesh.set_position_unchecked(mesh.position().component_mul(&mm_to_px) + offset);
            mesh.update_transformation_matrix();
            mesh
        };
        let mesh = to_platform(hollow.print_mesh(mesh));
        let model = SlicerModel {
            bounds: Bounds3D::from_mesh(&mesh),
            mesh,
            exposure: 255,
        };
        let layers = Slicer::new(config, vec![model]).slice_raster();
        (layers.iter())
            .map(|l| {
                l.data
                    .iter()
                    .filter(|r| r.value > 0)
                    .map(|r| r.length)
                    .sum()
            })
            .collect()
    }

    #[test]
    fn cavity_waits_for_settings_to_settle_and_stale_runs_stop() {
        use std::time::{Duration, Instant};

        let mut mesh = cube();
        let mut hollow = Hollow::default();
        let t0 = Instant::now();
        let ms = |n| t0 + Duration::from_millis(n);
        assert!(hollow.poll(&mesh, t0).is_none(), "not hollowed");

        hollow.spec.shell = Some(Shell::default());
        assert!(hollow.poll(&mesh, ms(0)).is_none());
        assert!(hollow.poll(&mesh, ms(200)).is_none());
        // A change restarts the wait.
        hollow.spec.shell = Some(Shell {
            wall: 2.5,
            ..Shell::default()
        });
        assert!(hollow.poll(&mesh, ms(250)).is_none());
        assert!(hollow.poll(&mesh, ms(500)).is_none());
        let key = hollow.poll(&mesh, ms(600)).expect("settled");
        assert_eq!(key.key.shell.wall, 2.5);

        // Running: nothing more starts, and scaling stops the run.
        assert!(hollow.poll(&mesh, ms(1000)).is_none());
        assert!(!key.token.is_cancelled());
        mesh.set_scale(Vector3::repeat(1.5));
        assert!(hollow.poll(&mesh, ms(1100)).is_none());
        assert!(key.token.is_cancelled());
        // Its result is dropped when it comes in: it is for the old scale.
        assert!(!hollow.cavity.finish(&key.key, None));
    }

    #[test]
    fn print_mesh_encloses_only_the_material() {
        let mesh = cube();
        let shell = Shell::default();
        let cancel = std::sync::atomic::AtomicBool::new(false);
        let found = cavity(
            &mesh,
            shell.wall,
            shell.cell,
            2.0,
            &Progress::new(),
            &cancel,
        )
        .unwrap();
        let cavity = ModelCavity::new(found, &mesh, shell);
        let voids = cavity.volume();
        let hollow = hollowed(
            HollowSpec {
                shell: Some(shell),
                holes: Vec::new(),
            },
            cavity,
            &mesh,
        );
        let print = hollow.print_mesh(&mesh);
        assert_eq!(print.position(), mesh.position());
        let volume = tools::hollow::signed_volume(print.vertices(), print.faces());
        assert!((volume - (8000.0 - voids)).abs() < 1.0, "{volume}");
        // Asking again gives the same mesh, not a new one.
        assert_eq!(hollow.print_mesh(&mesh).mesh_id(), print.mesh_id());
        // Unhollowed, it is the model's own mesh.
        assert_eq!(
            Hollow::default().print_mesh(&mesh).mesh_id(),
            mesh.mesh_id()
        );
    }

    #[test]
    fn supports_reach_into_the_cavity() {
        use common::units::Milimeters;
        use slicer::geometry::partitioning::Bvh;
        use tools::supports::{
            auto::{AutoPlacement, GenerationProgress, PlacementConfig},
            mesh::SupportPreset,
        };

        // Lifted, so the outside needs supports too.
        let mut mesh = cube();
        mesh.set_position(Vector3::new(-10.0, -10.0, 5.0));
        let shell = Shell::default();
        let cancel = std::sync::atomic::AtomicBool::new(false);
        let found = cavity(
            &mesh,
            shell.wall,
            shell.cell,
            2.0,
            &Progress::new(),
            &cancel,
        );
        let hollow = hollowed(
            HollowSpec {
                shell: Some(shell),
                holes: Vec::new(),
            },
            ModelCavity::new(found.unwrap(), &mesh, shell),
            &mesh,
        );
        let print = hollow.print_mesh(&mesh);
        let bvh = Bvh::build(print.inner(), Progress::new());
        let platform = Vector3::new(200.0, 200.0, 200.0).map(Milimeters::new);
        let generator = AutoPlacement::new(
            PlacementConfig::default(),
            SupportPreset::default(),
            platform,
        );
        let generated = generator.generate_supports(&print, &bvh, &GenerationProgress::default());
        // The cavity's ceiling, 2 mm under the top, is held up from its floor.
        let ceiling = 5.0 + 18.0;
        let inside = (generated.landings.iter())
            .filter(|l| (l.contact.z - ceiling).abs() < 0.5 && l.foot.z > 5.0)
            .count();
        assert!(inside > 0, "{} landings", generated.landings.len());
        // And the outside from the plate, as before.
        assert!(generated.plate.iter().any(|s| (s[0].z - 5.0).abs() < 0.5));
    }

    #[test]
    fn a_hole_that_cant_be_drilled_is_left_out() {
        let mesh = cube();
        let shell = Shell::default();
        let cancel = std::sync::atomic::AtomicBool::new(false);
        let found = cavity(
            &mesh,
            shell.wall,
            shell.cell,
            2.0,
            &Progress::new(),
            &cancel,
        );
        let cavity = ModelCavity::new(found.unwrap(), &mesh, shell);
        let good = cavity.drains[0];
        // Pointing out of the model: nothing to drill.
        let bad = Hole {
            point: Vector3::new(10.0, 10.0, 20.0),
            direction: Vector3::z(),
            diameter: 2.0,
        };
        let mut hollow = hollowed(
            HollowSpec {
                shell: Some(shell),
                holes: vec![bad],
            },
            cavity,
            &mesh,
        );
        let drill = |hollow: &Hollow| Drilled::drill(hollow.drill_input(&mesh).unwrap());
        let d = drill(&hollow);
        hollow.insert_drilled(&mesh, d);
        assert!(hollow.failed(&mesh, 0));
        // It drains nothing.
        assert_eq!(hollow.sealed_parts(&mesh), 1);
        hollow.spec.holes.push(good);
        let d = drill(&hollow);
        hollow.insert_drilled(&mesh, d);
        assert!(!hollow.failed(&mesh, 1));
        assert_eq!(hollow.sealed_parts(&mesh), 0);
    }

    #[test]
    fn hollowed_cube_slices_with_its_cavity_and_drain() {
        let mesh = cube();
        let solid = slice(&mesh, &Hollow::default());
        let shell = Shell {
            wall: 2.0,
            cell: 0.5,
        };
        let cancel = std::sync::atomic::AtomicBool::new(false);
        let found = cavity(
            &mesh,
            shell.wall,
            shell.cell,
            2.0,
            &Progress::new(),
            &cancel,
        )
        .unwrap();
        let cavity = ModelCavity::new(found, &mesh, shell);
        let drains = cavity.drains.clone();
        let mut hollow = hollowed(
            HollowSpec {
                shell: Some(shell),
                holes: Vec::new(),
            },
            cavity,
            &mesh,
        );
        assert!(hollow.ready(&mesh));
        assert_eq!(hollow.sealed_parts(&mesh), 1);
        hollow.spec.holes = drains;
        assert_eq!(hollow.sealed_parts(&mesh), 0);
        // The holes wait to be drilled, and slicing waits for them.
        assert!(hollow.wants_drilling(&mesh) && !hollow.ready(&mesh));
        let drilled = Drilled::drill(hollow.drill_input(&mesh).unwrap());
        assert_eq!(drilled.failed, vec![None]);
        hollow.insert_drilled(&mesh, drilled);
        assert!(!hollow.wants_drilling(&mesh) && hollow.ready(&mesh));
        let print = hollow.print_mesh(&mesh);
        assert!(tools::hollow::is_closed(print.faces()));
        let hollowed = slice(&mesh, &hollow);
        // Moving the model keeps the holes drilled; changing them doesn't.
        let mut moved = mesh.clone();
        moved.set_position(mesh.position() + Vector3::new(3.0, 0.0, 0.0));
        assert!(hollow.drilled(&moved).is_some());
        // So does turning it, scaled evenly...
        moved.set_rotation(Vector3::new(0.3, 0.0, 1.0));
        assert!(hollow.drilled(&moved).is_some());
        // ...but not scaled unevenly: that changes its shape.
        let mut squashed = mesh.clone();
        squashed.set_scale(Vector3::new(1.0, 1.0, 0.5));
        let mut turned = squashed.clone();
        turned.set_rotation(Vector3::new(0.0, 1.0, 0.0));
        assert_ne!(super::Shape::of(&squashed), super::Shape::of(&turned));
        let mut spun = squashed.clone();
        spun.set_rotation(Vector3::new(0.0, 0.0, 1.0));
        assert_eq!(super::Shape::of(&squashed), super::Shape::of(&spun));
        hollow.spec.holes[0].diameter = 3.0;
        assert!(hollow.wants_drilling(&mesh));

        // Half way up: the 20 mm square less the 16 mm void.
        let middle = 200;
        assert_eq!(solid[middle], 100 * 100);
        let expected = 100 * 100 - 80 * 80;
        let got = hollowed[middle] as i64;
        assert!((got - expected).abs() < 400, "{got} vs {expected}");

        // In the floor: solid but for the 2 mm drain.
        let floor = 20;
        let drain = (solid[floor] - hollowed[floor]) as f32;
        let round = std::f32::consts::PI * 5.0 * 5.0;
        assert!((drain - round).abs() < round * 0.3, "{drain} vs {round}");
    }
}
