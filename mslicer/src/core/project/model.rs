use std::{collections::HashMap, path::PathBuf, sync::Arc};

use bitflags::bitflags;
use common::{
    color::LinearRgb,
    id_type,
    serde::{Deserializer, Serializer},
    units::{CubicMilimeters, Milimeters},
};
use nalgebra::Vector3;
use wgpu::Device;

use slicer::geometry::{
    Hit,
    bounds::Bounds3D,
    mesh::{Mesh, MeshInner},
    partitioning::Bvh,
};

use crate::{
    core::{
        issues::{HoleIssue, HoleWarning},
        palette,
        project::{
            CollectionId, RenameState,
            geometry::{Evaluated, Geometry, GeometryKey, RepairStats, Step},
            hollow::{Hollow, Shell, first_hit},
            stack::{Mode, OperationId, Stack},
            supports::{Pose, Supports},
        },
    },
    engine::{InputHash, InputHasher, SlotState},
    render::util::MeshBuffers,
};

pub struct Model {
    pub name: String,
    pub id: ModelId,
    pub collection: Option<CollectionId>,

    /// The model's source: the mesh as imported, never changed.
    pub source: Arc<MeshInner>,
    /// The operations applied to the source.
    pub stack: Stack,
    /// The geometry section's evaluation.
    pub geometry: Geometry,
    /// What each Mesh Repair found, once it ran.
    pub repairs: HashMap<OperationId, RepairStats>,
    /// A Mesh Repair was added by hand and its findings are to be shown once
    /// it has run.
    pub announce_repair: Option<OperationId>,
    /// What the orientation was last seen above: the surface and the shell.
    /// A change marks an auto-orientation as possibly outdated.
    pub upstream: Option<InputHash>,

    /// The model's surface (the source with the geometry rows before Hollow
    /// applied), placed: its transform is the model's position, scale and
    /// orientation.
    pub mesh: Mesh,
    pub bounds: Bounds3D,
    pub bvh: Option<Arc<Bvh>>,
    pub supports: Supports,
    pub hollow: Hollow,
    base_volume: Option<CubicMilimeters>,

    pub unit: MeshUnit,
    pub color: LinearRgb<f32>,
    pub exposure: u8,
    pub hidden: bool,
    pub ui: ModelUi,

    // Not serialized
    pub warnings: MeshWarnings,
    buffers: Option<MeshBuffers>,
    pub file: Option<PathBuf>,
}

#[derive(Clone)]
pub struct ModelUi {
    pub rename: RenameState,
    pub locked_scale: bool,
}

bitflags! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct MeshWarnings: u8 {
        const Defective = 1 << 0;
        const OutOfBounds = 1 << 1;
    }
}

#[derive(Clone, Copy, PartialEq)]
pub enum MeshUnit {
    Millimeters,
    Centimeters,
    Meters,
    Inches,
    Custom(f32),
}

impl Model {
    pub fn from_mesh(mesh: Mesh) -> Self {
        let mut supports = Supports::default();
        supports.set_pose(Pose::of(&mesh));
        Self {
            name: String::new(),
            id: ModelId::new(),
            collection: None,

            source: mesh.inner().clone(),
            stack: Stack::default(),
            geometry: Geometry::default(),
            repairs: HashMap::new(),
            announce_repair: None,
            upstream: None,

            base_volume: None,
            bvh: None,
            supports,
            hollow: Hollow::default(),
            bounds: Bounds3D::from_mesh(&mesh),
            mesh,

            unit: MeshUnit::Millimeters,
            color: LinearRgb::splat(1.0),
            hidden: false,
            exposure: 255,
            ui: ModelUi::default(),

            warnings: MeshWarnings::empty(),
            buffers: None,
            file: None,
        }
    }

    pub fn volume(&self) -> Option<CubicMilimeters> {
        let scale = self.mesh.scale();
        self.base_volume.map(|v| v * scale.x * scale.y * scale.z)
    }

    pub fn with_name(mut self, name: String) -> Self {
        self.name = name;
        self
    }

    pub fn with_color(mut self, color: LinearRgb<f32>) -> Self {
        self.color = color;
        self
    }

    pub fn with_exposure(mut self, exposure: u8) -> Self {
        self.exposure = exposure;
        self
    }

    pub fn with_hidden(mut self, hidden: bool) -> Self {
        self.hidden = hidden;
        self
    }

    pub fn with_collection(mut self, collection: Option<CollectionId>) -> Self {
        self.collection = collection;
        self
    }

    pub fn with_unit(mut self, unit: MeshUnit) -> Self {
        self.unit = unit;
        self
    }

    pub fn width_file(mut self, path: Option<PathBuf>) -> Self {
        self.file = path;
        self
    }

    /// A random colour clear of the `warning` colour (sRGB); see
    /// [`palette::random_model_color`].
    pub fn with_random_color(mut self, warning: [f32; 3]) -> Self {
        self.randomize_color(warning);
        self
    }

    pub fn randomize_color(&mut self, warning: [f32; 3]) -> &mut Self {
        self.color = palette::random_model_color(warning);
        self
    }

    /// Where a ray (world space) first meets the model's surface where it
    /// is shown: at or below the view's `cut` height, if there is one. Needs
    /// the hit-testing structure.
    pub fn visible_hit(
        &self,
        origin: Vector3<f32>,
        direction: Vector3<f32>,
        cut: Option<f32>,
    ) -> Option<Hit> {
        let bvh = self.bvh.as_ref()?;
        let Some(cut) = cut else {
            return bvh.intersect_ray(&self.mesh, origin, direction);
        };
        (bvh.intersect_ray_all(&self.mesh, origin, direction)
            .into_iter())
        .find(|hit| self.mesh.transform(&hit.position).z <= cut)
    }

    /// The mesh as it prints: the surface, hollowed if the model is, with
    /// the geometry rows after Hollow applied. Placed like the model.
    pub fn print_mesh(&self) -> Mesh {
        let print = match self.stack.hollow_active() {
            true => self.hollow.print_mesh(&self.mesh),
            false => self.mesh.clone(),
        };
        let (_, after) = self.stack.geometry();
        if after.is_empty() {
            return print;
        }
        let steps: Vec<_> = after.into_iter().filter_map(Step::of).collect();
        let inner = self.geometry.after_hollow(print.inner(), &steps);
        let mut mesh = Mesh::from_inner(inner);
        mesh.set_position_unchecked(print.position());
        mesh.set_scale_unchecked(print.scale());
        mesh.set_rotation_unchecked(print.rotation());
        mesh.update_transformation_matrix();
        mesh
    }

    /// What is wrong with the model's holes as it is placed now (indices into
    /// the manual holes, then the auto holes):
    /// - drilled where the model touches the plate (it won't stand on it),
    /// - above the resin line of their void (a hand-placed hole that no
    ///   longer is at the void's lowest point after re-orienting; holes are
    ///   never moved for that),
    /// - closed again by a geometry operation after Hollow.
    pub fn hole_warnings(&self) -> Vec<HoleIssue> {
        let Some(shell) = self.hollow_shell() else {
            return Vec::new();
        };
        let mesh = &self.mesh;
        let holes = self.hollow.all_holes(mesh);
        let world: Vec<_> = holes.iter().map(|h| h.world(mesh, shell)).collect();
        let failed = |i: usize| self.hollow.failed(mesh, i);
        let mut out = Vec::new();

        // Within a few layers of the plate, on a model standing on it.
        if self.bounds.min.z < 0.05 {
            for (hole, w) in world.iter().enumerate() {
                if w.point.z < self.bounds.min.z + 0.2 && !failed(hole) {
                    out.push(HoleIssue {
                        hole,
                        kind: HoleWarning::PlateContact,
                    });
                }
            }
        }

        // Resin drains down to the lowest edge of a void's lowest hole.
        let margin = shell.cell * 2.0;
        let tolerance = shell.cell * 2.0 + 0.5;
        for (void, placed) in self.hollow.placed_voids(mesh).iter().enumerate() {
            let opening = |w: &tools::hollow::Hole| {
                let end = w.point + w.direction * w.depth;
                let across = (1.0 - w.direction.z.powi(2)).max(0.0).sqrt();
                (end, end.z - w.diameter / 2.0 * across)
            };
            let into: Vec<(usize, f32)> = (world.iter().enumerate())
                .filter(|(i, _)| !failed(*i))
                .filter_map(|(i, w)| {
                    let (end, z) = opening(w);
                    let inside = (0..3).all(|k| {
                        end[k] >= placed.min[k] - margin && end[k] <= placed.max[k] + margin
                    });
                    inside.then_some((i, z))
                })
                .collect();
            if let Some(&(hole, z)) = into.iter().min_by(|a, b| a.1.total_cmp(&b.1))
                && z > placed.lowest.z + tolerance
            {
                out.push(HoleIssue {
                    hole,
                    kind: HoleWarning::NotDraining { void },
                });
            }
        }

        // A geometry row after Hollow could close a hole: look down each
        // hole's axis in the final print mesh.
        if !self.stack.geometry().1.is_empty() {
            let print = self.print_mesh();
            for (hole, h) in holes.iter().enumerate() {
                if failed(hole) {
                    continue;
                }
                let start = h.point - h.direction * 0.5;
                let open = first_hit(&print, start, h.direction)
                    .is_none_or(|t| t > 0.5 + shell.wall * 0.8);
                if !open {
                    out.push(HoleIssue {
                        hole,
                        kind: HoleWarning::Blocked,
                    });
                }
            }
        }
        out
    }

    /// What a Mesh Repair operation found, once it ran.
    pub fn repair_stats(&self, id: OperationId) -> Option<RepairStats> {
        (self.repairs.get(&id).copied()).or_else(|| self.geometry.after_hollow_repair(id))
    }

    /// Whether the model is hollowed: it has an active Hollow row with a
    /// shell.
    pub fn is_hollowed(&self) -> bool {
        self.stack.hollow_active() && self.hollow.spec.shell.is_some()
    }

    /// The shell, if the model is hollowed.
    pub fn hollow_shell(&self) -> Option<Shell> {
        self.hollow
            .spec
            .shell
            .filter(|_| self.stack.hollow_active())
    }

    /// The inputs above the orientation: the surface (by identity) and the
    /// hollow shell.
    pub fn upstream_hash(&self) -> InputHash {
        let mut hash = InputHasher::new();
        hash.arc(self.mesh.inner());
        if let Some(shell) = self.hollow_shell() {
            hash.f32(shell.wall).f32(shell.cell);
        }
        hash.input_hash()
    }

    /// What the surface is evaluated from now.
    pub fn geometry_key(&self) -> GeometryKey {
        let (before, _) = self.stack.geometry();
        GeometryKey {
            source: self.source.clone(),
            steps: before.into_iter().filter_map(Step::of).collect(),
        }
    }

    /// The surface held by a frozen geometry row, if there is one that
    /// differs from the source.
    pub fn frozen_surface(&self) -> Option<&Arc<MeshInner>> {
        let (before, _) = self.stack.geometry();
        let frozen = before.iter().any(|o| o.mode == Mode::Frozen);
        (frozen && !Arc::ptr_eq(self.mesh.inner(), &self.source)).then(|| self.mesh.inner())
    }

    /// Puts back a frozen surface saved with the project.
    pub fn restore_frozen_surface(&mut self, surface: Arc<MeshInner>) {
        let key = self.geometry_key();
        let evaluated = Evaluated {
            mesh: surface.clone(),
            repairs: Vec::new(),
        };
        self.geometry.surface.insert(key, Arc::new(evaluated));
        let (position, scale, rotation) = (
            self.mesh.position(),
            self.mesh.scale(),
            self.mesh.rotation(),
        );
        self.mesh = Mesh::from_inner(surface);
        self.mesh.set_position_unchecked(position);
        self.mesh.set_scale_unchecked(scale);
        self.mesh.set_rotation_unchecked(rotation);
        self.mesh.update_transformation_matrix();
    }

    /// Whether the surface is evaluated from the current stack.
    pub fn surface_ready(&self) -> bool {
        // A frozen geometry row holds whatever surface there is.
        let frozen = (self.stack.geometry().0.iter()).any(|o| o.mode == Mode::Frozen);
        if frozen && let Some(held) = self.geometry.surface.latest() {
            return Arc::ptr_eq(&held.mesh, self.mesh.inner());
        }
        let key = self.geometry_key();
        key.steps.is_empty() && Arc::ptr_eq(&key.source, self.mesh.inner())
            || (self.geometry.surface.get(&key))
                .is_some_and(|e| Arc::ptr_eq(&e.mesh, self.mesh.inner()))
    }

    /// Whether everything slicing the model needs is computed.
    pub fn ready(&self) -> bool {
        self.surface_ready()
            && (!self.stack.hollow_active() || self.hollow.ready(&self.mesh))
            && !self.supports.needs_reroute()
    }

    /// The support sets, if the model's supports take part.
    pub fn active_supports(&self) -> Option<&Supports> {
        self.stack.supports_active().then_some(&self.supports)
    }

    /// Makes `inner` the model's surface, keeping its placement. What was
    /// derived from the old surface (hit testing, GPU buffers, analysis) is
    /// dropped; the supports stay. Returns whether it changed.
    pub fn set_surface(&mut self, inner: Arc<MeshInner>, platform: &Vector3<Milimeters>) -> bool {
        if Arc::ptr_eq(self.mesh.inner(), &inner) {
            return false;
        }
        let (position, scale, rotation) = (
            self.mesh.position(),
            self.mesh.scale(),
            self.mesh.rotation(),
        );
        self.mesh = Mesh::from_inner(inner);
        self.mesh.set_position_unchecked(position);
        self.mesh.set_scale_unchecked(scale);
        self.mesh.set_rotation_unchecked(rotation);
        self.mesh.update_transformation_matrix();
        self.bvh = None;
        self.buffers = None;
        self.base_volume = None;
        self.warnings.remove(MeshWarnings::Defective);
        self.update_oob(platform);
        true
    }

    /// The surface supports are placed and routed on: the mesh as it
    /// prints (a hollowed model's cavity walls included, so the inside gets
    /// supports too), with its hit-testing structure if that is built.
    pub fn support_surface(&self) -> (Mesh, Option<Arc<Bvh>>) {
        let print = self.print_mesh();
        let bvh = match print.mesh_id() == self.mesh.mesh_id() {
            true => self.bvh.clone(),
            false => self.hollow.print_bvh(&print),
        };
        (print, bvh)
    }

    /// The states of the model's evaluation nodes: cavity, drilling and
    /// support mesh.
    pub fn slot_states(&self) -> [SlotState; 3] {
        let [cavity, drilled] = self.hollow.slot_states();
        [cavity, drilled, self.supports.slot_state()]
    }

    pub fn has_acceleration_structures(&self) -> bool {
        self.bvh.is_some()
    }

    pub fn try_get_buffers(&self) -> Option<&MeshBuffers> {
        self.buffers.as_ref()
    }

    pub fn get_buffers(&mut self, device: &Device) -> &MeshBuffers {
        (self.buffers).get_or_insert_with(|| MeshBuffers::get_for(device, &self.mesh))
    }

    pub fn replace_mesh(
        &mut self,
        mesh: Mesh,
        file: Option<PathBuf>,
        platform: &Vector3<Milimeters>,
    ) {
        let (position, scale, rotation) = (
            self.mesh.position(),
            self.mesh.scale(),
            self.mesh.rotation(),
        );
        self.source = mesh.inner().clone();
        self.mesh = mesh;
        self.mesh.set_position_unchecked(position);
        self.mesh.set_scale_unchecked(scale);
        self.mesh.set_rotation_unchecked(rotation);
        self.mesh.update_transformation_matrix();

        self.file = file;
        // The supports stay: each is checked against the new mesh, kept if
        // it still finds a route, else kept and marked.
        self.supports.surface_replaced();
        self.warnings = MeshWarnings::empty();
        self.bvh = None;
        self.buffers = None;
        self.base_volume = None;
        self.update_oob(platform);
    }

    pub fn set_base_volume(&mut self, volume: CubicMilimeters) {
        self.base_volume.get_or_insert(volume);
    }

    pub fn mesh_memory_size(&self) -> usize {
        let source = match Arc::ptr_eq(&self.source, self.mesh.inner()) {
            true => 0,
            false => Mesh::from_inner(self.source.clone()).memory_size(),
        };
        self.mesh.memory_size()
            + source
            + (self.bvh.as_ref().map(|x| x.memory_size())).unwrap_or_default()
    }
}

impl Model {
    pub fn align_to_bed(&mut self) {
        let Bounds3D { min, .. } = self.bounds;

        let pos = self.mesh.position() - Vector3::z() * min.z;
        self.mesh.set_position(pos);
    }

    pub fn update_oob(&mut self, platform: &Vector3<Milimeters>) {
        self.supports.set_pose(Pose::of(&self.mesh));

        let bounds @ Bounds3D { min, max } = Bounds3D::from_mesh(&self.mesh);
        let half = platform.map(|x| x.raw()) / 2.0;
        self.bounds = bounds;

        let oob = (min.x < -half.x || min.y < -half.y || min.z < 0.0)
            || (max.x > half.x || max.y > half.y || max.z > platform.z.raw());
        self.warnings.set(MeshWarnings::OutOfBounds, oob);
    }

    pub fn set_bounds_center(&mut self, platform: &Vector3<Milimeters>, center: Vector3<f32>) {
        let current = self.bounds.center();
        self.set_position(platform, self.mesh.position() + center - current);
    }

    pub fn set_position(&mut self, platform: &Vector3<Milimeters>, pos: Vector3<f32>) {
        self.mesh.set_position(pos);
        self.update_oob(platform);
    }

    pub fn set_scale(&mut self, platform: &Vector3<Milimeters>, scale: Vector3<f32>) {
        self.mesh.set_scale(scale);
        self.update_oob(platform);
    }

    pub fn set_rotation(&mut self, platform: &Vector3<Milimeters>, rotation: Vector3<f32>) {
        self.mesh.set_rotation(rotation);
        self.update_oob(platform);
    }

    /// Rotates the model and sets it down again: see [`Self::set_down`].
    pub fn rotate_resting(&mut self, platform: &Vector3<Milimeters>, rotation: Vector3<f32>) {
        let floor = self.floor();
        self.set_rotation(platform, rotation);
        self.set_down(platform, floor);
    }

    /// Where a rotated model is set down: on the bed, or, with supports,
    /// with its lowest point at the height it had (the support lift).
    pub fn floor(&self) -> f32 {
        match self.supports.is_empty() {
            true => 0.0,
            false => self.bounds.min.z,
        }
    }

    /// Moves the model up or down so its lowest point is at `floor`.
    pub fn set_down(&mut self, platform: &Vector3<Milimeters>, floor: f32) {
        let lift = Vector3::z() * (floor - self.bounds.min.z);
        self.set_position(platform, self.mesh.position() + lift);
    }
}

impl MeshUnit {
    pub const ALL: [Self; 5] = [
        Self::Millimeters,
        Self::Centimeters,
        Self::Meters,
        Self::Inches,
        Self::Custom(1.0),
    ];

    pub fn name(&self) -> &str {
        match self {
            Self::Millimeters => "Millimeters",
            Self::Centimeters => "Centimeters",
            Self::Meters => "Meters",
            Self::Inches => "Inches",
            Self::Custom(_) => "Custom",
        }
    }

    /// Conversion factor from self to millimeters
    pub fn conversion(&self) -> f32 {
        match self {
            Self::Millimeters => 1.0,
            Self::Centimeters => 10.0,
            Self::Meters => 1000.0,
            Self::Inches => 2.54,
            Self::Custom(x) => *x,
        }
    }

    pub fn ordinal(&self) -> u8 {
        match self {
            Self::Custom(_) => 0,
            Self::Millimeters => 1,
            Self::Centimeters => 2,
            Self::Meters => 3,
            Self::Inches => 4,
        }
    }

    pub fn from_ordinal(ordinal: u8) -> Option<Self> {
        Some(match ordinal {
            1 => Self::Millimeters,
            2 => Self::Centimeters,
            3 => Self::Meters,
            4 => Self::Inches,
            _ => return None,
        })
    }

    pub fn serialize<T: Serializer>(&self, ser: &mut T) {
        ser.write_u8(self.ordinal());
        if let MeshUnit::Custom(factor) = self {
            ser.write_f32_be(*factor);
        }
    }

    pub fn deserialize<T: Deserializer>(des: &mut T) -> Self {
        let ordinal = des.read_u8();
        if ordinal == 0 {
            Self::Custom(des.read_f32_be())
        } else {
            Self::from_ordinal(ordinal).unwrap_or(Self::Millimeters)
        }
    }
}

impl Clone for Model {
    fn clone(&self) -> Self {
        Self {
            name: self.name.clone(),
            id: ModelId::new(),
            collection: self.collection,

            source: self.source.clone(),
            stack: self.stack.clone(),
            geometry: self.geometry.clone(),
            repairs: self.repairs.clone(),
            announce_repair: None,
            upstream: self.upstream,
            mesh: self.mesh.clone(),
            bounds: self.bounds.clone(),
            bvh: self.bvh.clone(),
            supports: self.supports.duplicate(),
            hollow: self.hollow.clone(),
            base_volume: self.base_volume,

            unit: self.unit,
            color: self.color,
            hidden: self.hidden,
            ui: self.ui.clone(),

            exposure: self.exposure,

            warnings: self.warnings,
            buffers: None,
            file: None,
        }
    }
}

impl Default for ModelUi {
    fn default() -> Self {
        Self {
            rename: RenameState::None,
            locked_scale: true,
        }
    }
}

impl PartialEq for Model {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

id_type!(ModelId, u32);

#[cfg(test)]
mod tests {
    use crate::regression::{load_prepared, project_path};

    #[test]
    fn a_turned_model_rests_on_the_bed() {
        let mut project = load_prepared(&project_path("hollowed"));
        let platform = project.slice_config.platform_size;
        let model = &mut project.models[0];
        model.supports = Default::default();
        model.rotate_resting(&platform, nalgebra::Vector3::new(0.7, 0.3, 0.0));
        assert!(model.bounds.min.z.abs() < 1e-3, "{}", model.bounds.min.z);
    }
}
