//! The slicing regression harness.
//!
//! A handful of reference projects are kept as `.mslicer` files (format v18,
//! as saved before the redesign) in `mslicer/tests/regression/`. Each test
//! loads one, prepares it the way the app would before slicing (cavity,
//! drilled holes, support mesh), slices it headlessly through the same code
//! the Slice button uses ([`platform_models`] and [`run_slicer`]), and
//! compares a hash of every layer plus some key stats against the stored
//! reference (`<name>.ref.json`).
//!
//! - `MSLICER_BLESS=1 cargo test -p mslicer regression` rewrites the
//!   references from the current output. Only do that when a change of the
//!   slicing output is intended.
//! - `MSLICER_BLESS_PROJECTS=1` additionally rebuilds the project files from
//!   their recipes below. Avoid this once the save format changes: the
//!   committed files are the record of what old projects look like.

use std::{
    fmt::Write as _,
    fs,
    path::{Path, PathBuf},
    sync::{Arc, atomic::AtomicBool},
};

use common::{
    progress::{CombinedProgress, Progress},
    serde::{DynamicSerializer, SliceDeserializer},
    slice::{ExposureConfig, Layer, SliceConfig, SliceMode, Supersample, VectorLayer},
    units::{Milimeter, Milimeters},
};
use mesh_format::{Format, load_mesh};
use nalgebra::{Vector2, Vector3};
use serde::{Deserialize, Serialize};
use slicer::{
    geometry::{bounds::Bounds3D, mesh::Mesh, partitioning::Bvh},
    post_process::island_detection::detect_islands,
    slicer::Slicer,
};
use tools::islands::{Planes, detect};
use tools::supports::{
    auto::{AutoPlacement, GenerationProgress, route_hand},
    mesh::SupportPreset,
};

use crate::core::{
    project::{
        Collection, Project,
        hollow::{Drilled, Hole, ModelCavity, Shell},
        model::Model,
        supports::Pose,
    },
    slice::runner::{SliceOutput, SlicePart, platform_models, run_slicer},
};
use crate::engine::CancelToken;

fn dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/regression")
}

/// A small printer, so the references slice quickly: 40 µm pixels.
fn small_config() -> SliceConfig {
    SliceConfig {
        platform_resolution: Vector2::new(1920, 1080),
        platform_size: Vector3::new(76.8, 43.2, 60.0).map(Milimeters::new),
        ..Default::default()
    }
}

fn load_stl(path: &Path) -> Mesh {
    let data = fs::read(path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let mesh = load_mesh(SliceDeserializer::new(&data), Format::Stl, &Progress::new()).unwrap();
    Mesh::new_boxed_centered(mesh.verts, mesh.faces)
}

fn teapot() -> Mesh {
    load_stl(&Path::new(env!("CARGO_MANIFEST_DIR")).join("assets/teapot.stl"))
}

fn fixture_mesh(name: &str) -> Mesh {
    load_stl(&dir().join(name))
}

fn placed(project: &Project, mesh: Mesh, name: &str) -> Model {
    let platform = project.slice_config.platform_size;
    let mut model = Model::from_mesh(mesh).with_name(name.into());
    model.update_oob(&platform);
    model.align_to_bed();
    model.update_oob(&platform);
    model
}

/// Computes the cavity of a hollowed model, as the `HollowModel` task does.
fn compute_cavity(model: &mut Model) {
    let Some(shell) = model.hollow.spec.shell else {
        return;
    };
    let cancel = AtomicBool::new(false);
    let found = tools::hollow::cavity(
        &model.mesh,
        shell.wall,
        shell.cell,
        2.0,
        &Progress::new(),
        &cancel,
    )
    .expect("cavity");
    let cavity = ModelCavity::new(found, &model.mesh, shell);
    model.hollow.insert_cavity(&model.mesh.clone(), cavity);
}

/// Drills the holes, as the `DrillHoles` task does.
fn drill(model: &mut Model) {
    if model.hollow.wants_drilling(&model.mesh) {
        let input = model.hollow.drill_input(&model.mesh).unwrap();
        let drilled = Drilled::drill(input);
        model.hollow.insert_drilled(&model.mesh.clone(), drilled);
    }
}

/// Generates auto supports, as the Supports panel's Generate does: lifts the
/// model to the default lift, then places, routes and braces.
fn generate_supports(project: &mut Project, index: usize) {
    let lift = project.support_config.default_lift.get::<Milimeter>();
    let platform = project.slice_config.platform_size;
    let placement = project.support_config.auto_placement.clone();
    let preset = project.support_config.preset.clone();
    let model = &mut project.models[index];

    let pos = model.mesh.position() + Vector3::z() * (lift - model.bounds.min.z);
    model.set_position(&platform, pos);

    let (mesh, _) = model.support_surface();
    let bvh = Bvh::build(mesh.inner(), Progress::new());
    let generator = AutoPlacement::new(placement, preset.clone(), platform.map(|x| x.convert()));
    let generated = generator.generate_supports(&mesh, &bvh, &GenerationProgress::default());
    let network = generator.generate_braces(&mesh, &bvh, &generated);

    let at = Pose::of(&model.mesh);
    model
        .supports
        .begin_generation(u64::MAX, Default::default());
    let taken = (model.supports).finish_generation(
        u64::MAX,
        at,
        &preset,
        (generated, network),
        &mesh,
        &bvh,
    );
    assert!(taken);
}

/// Adds a hand-placed support under the first face that looks down and isn't
/// on the plate, routed as a click there would be.
fn add_manual_support(project: &mut Project, index: usize) {
    let preset = project.support_config.preset.clone();
    let model = &mut project.models[index];
    let bvh = Arc::new(Bvh::build(model.mesh.inner(), Progress::new()));
    model.bvh = Some(bvh.clone());
    let mesh = &model.mesh;
    let (contact, normal) = (0..mesh.face_count())
        .map(|face| {
            let verts = mesh.face_verts(face);
            let centroid = (verts[0] + verts[1] + verts[2]) / 3.0;
            let normal = mesh.transform_normal(&mesh.normal(face)).normalize();
            (centroid, normal)
        })
        .find(|(c, n)| n.z < -0.5 && c.z > model.bounds.min.z + 1.0)
        .expect("a downward face");
    let routed = route_hand(mesh, &bvh, &preset, contact, normal, false).expect("routed");
    let mesh = model.mesh.clone();
    model.supports.add_manual(&preset, &routed, &mesh, &bvh);
}

/// The recipes of the reference projects.
fn build(name: &str) -> Project {
    let mut project = Project {
        slice_config: small_config(),
        ..Default::default()
    };
    match name {
        // A plain model, as imported.
        "teapot" => {
            let model = placed(&project, teapot(), "Teapot");
            project.models.push(model);
        }
        // The T-shaped island test model: its arms start in mid-air.
        "t_shape" => {
            let model = placed(&project, fixture_mesh("t_shape.stl"), "T");
            project.models.push(model);
        }
        // A hollowed sphere with its suggested drain plus a hand-placed
        // hole in its side.
        "hollowed" => {
            let mut model = placed(&project, fixture_mesh("sphere.stl"), "Sphere");
            model.hollow.spec.shell = Some(Shell {
                wall: 2.0,
                cell: 0.5,
            });
            compute_cavity(&mut model);
            let cavity = model.hollow.cavity(&model.mesh).unwrap();
            let mut holes = cavity.drains.clone();
            let bounds = Bounds3D::from_mesh(&model.mesh);
            let center = bounds.center();
            let side = tools::hollow::Hole {
                point: Vector3::new(bounds.max.x, center.y, center.z),
                direction: -Vector3::x(),
                diameter: 2.5,
                depth: 0.0,
            };
            holes.push(Hole::local(&side, &model.mesh));
            model.hollow.spec.holes = holes;
            project.models.push(model);
        }
        // The T lifted on auto supports with a raft, plus a hand-placed one.
        "supported" => {
            project.support_config.preset = SupportPreset::MIDDLE;
            let model = placed(&project, fixture_mesh("t_shape.stl"), "T");
            project.models.push(model);
            generate_supports(&mut project, 0);
            add_manual_support(&mut project, 0);
        }
        // Post-processing and anti-aliasing: variable layer height, the
        // elephant foot fixer, 2×2 supersampling; two instances of one mesh
        // (one turned, at half exposure, in a collection) and a hidden model.
        "vlh_eff" => {
            project.slice_config.supersample = Supersample { xy: 2, z: 2 };
            project.post_processing.variable_layer_height.enabled = true;
            project.post_processing.elephant_foot_fixer.enabled = true;
            let platform = project.slice_config.platform_size;

            let mut first = placed(&project, teapot(), "Teapot");
            first.set_position(&platform, first.mesh.position() - Vector3::x() * 12.0);
            let mut second = first.clone().with_name("Teapot (turned)".into());
            second.set_rotation(&platform, Vector3::z() * std::f32::consts::FRAC_PI_2);
            second.set_position(&platform, Vector3::new(14.0, 0.0, second.mesh.position().z));
            second.align_to_bed();
            second.update_oob(&platform);
            let collection = Collection::new("Pair".into());
            second.collection = Some(collection.id);
            second.exposure = 128;
            project.collections.push(collection);

            let mut hidden = placed(&project, fixture_mesh("sphere.stl"), "Hidden");
            hidden.hidden = true;
            project.models.extend([first, second, hidden]);
        }
        // Vector (SVG) slicing.
        "vector" => {
            project.slice_config.mode = SliceMode::Vector;
            let model = placed(&project, fixture_mesh("t_shape.stl"), "T");
            project.models.push(model);
        }
        _ => panic!("unknown fixture {name}"),
    }
    project
}

fn save(project: &Project, path: &Path) {
    let mut ser = DynamicSerializer::new();
    project.serialize(&mut ser, Progress::new());
    write_atomic(path, &ser.into_inner());
}

/// Loads a project file and prepares it as the app does after loading and
/// before slicing.
pub fn load_prepared(path: &Path) -> Project {
    let data = fs::read(path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let mut project = Project::deserialize(&mut SliceDeserializer::new(&data), Progress::new())
        .expect("project loads");
    let platform = project.slice_config.platform_size;
    for model in project.models.iter_mut() {
        model.update_oob(&platform);
        crate::core::stack::evaluate_now(model, &platform);
        compute_cavity(model);
        drill(model);
        assert!(model.ready(), "{} hollowed", model.name);
    }
    project.update();
    project
}

/// Slices a prepared project the way `App::slice` does, without previews.
pub fn slice(project: &mut Project) -> SliceOutput {
    let parts: Vec<_> = (project.models.iter_mut())
        .filter(|m| !m.hidden)
        .map(|m| SlicePart {
            supports: (m.stack.supports_active())
                .then(|| m.supports.finished_mesh().as_ref().map(|x| x.0.clone()))
                .flatten(),
            print: m.print_mesh(),
            exposure: m.exposure,
        })
        .collect();
    let (models, _) = platform_models(&project.slice_config, parts);
    let slicer = Slicer::new(project.slice_config.clone(), models);
    run_slicer(
        &slicer,
        &project.post_processing,
        CombinedProgress::new(),
        &CancelToken::new(),
    )
    .expect("not cancelled")
}

/// FNV-1a, 64 bit: stable across platforms and Rust versions.
#[derive(Clone, Copy)]
struct Fnv(u64);

impl Fnv {
    fn new() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }

    fn bytes(&mut self, bytes: &[u8]) -> &mut Self {
        for b in bytes {
            self.0 ^= *b as u64;
            self.0 = self.0.wrapping_mul(0x0100_0000_01b3);
        }
        self
    }

    fn u64(&mut self, x: u64) -> &mut Self {
        self.bytes(&x.to_le_bytes())
    }

    fn f32(&mut self, x: f32) -> &mut Self {
        self.bytes(&x.to_bits().to_le_bytes())
    }

    fn hex(&self) -> String {
        format!("{:016x}", self.0)
    }
}

fn hash_exposure(hash: &mut Fnv, e: &ExposureConfig) {
    hash.f32(e.exposure_time.raw())
        .f32(e.exposure_delay.raw())
        .u64(e.pwm as u64)
        .f32(e.lift_distance.raw())
        .f32(e.lift_speed.raw())
        .f32(e.retract_speed.raw());
}

fn hash_layer(layer: &Layer) -> String {
    let mut hash = Fnv::new();
    for run in layer.data.iter() {
        hash.u64(run.length).u64(run.value as u64);
    }
    hash.f32(layer.height.raw())
        .u64(layer.defects)
        .u64(layer.unique_exposure as u64);
    hash_exposure(&mut hash, &layer.exposure);
    hash.hex()
}

/// Every polygon's points, in order.
fn hash_vector_layer(layer: &VectorLayer) -> String {
    let mut hash = Fnv::new();
    for polygon in layer {
        hash.u64(polygon.len() as u64);
        for p in polygon {
            hash.f32(p.x).f32(p.y);
        }
    }
    hash.hex()
}

/// The island check of the Islands panel over every visible model, at every
/// layer: the islands found, and how many of them start on the plate.
fn model_islands(project: &Project) -> (usize, usize) {
    let planes = Planes::layers(project.slice_config.slice_height.get::<Milimeter>(), 1);
    let (mut islands, mut on_plate) = (0, 0);
    for model in project.models.iter().filter(|m| !m.hidden) {
        let print = model.print_mesh();
        let bvh = Bvh::build(print.inner(), Progress::new());
        let found = detect(&print, &bvh, planes, &Progress::new());
        islands += found.islands.len();
        on_plate += found.islands.iter().filter(|i| i.on_plate).count();
    }
    (islands, on_plate)
}

/// What is compared: key stats plus a hash of every layer.
#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub struct Reference {
    pub layers: usize,
    /// Lit voxels over all layers (raster only).
    pub voxels: u64,
    /// Scan lines with slicing defects over all layers (raster only).
    pub defects: u64,
    /// Layers with islands, as the Sliced panel's island detection finds
    /// them (raster only).
    pub island_layers: usize,
    /// Islands the Islands panel's check finds over the visible models, and
    /// how many of them start on the plate.
    pub model_islands: (usize, usize),
    /// Print time (s), formatted to keep the file stable.
    pub print_time: String,
    pub layer_hashes: Vec<String>,
}

impl Reference {
    pub fn of(output: &SliceOutput, project: &Project) -> Self {
        let config = &project.slice_config;
        let model_islands = model_islands(project);
        match output {
            SliceOutput::Raster(layers) => {
                let voxels = (layers.iter())
                    .flat_map(|l| l.data.iter().filter(|r| r.value != 0).map(|r| r.length))
                    .sum();
                let islands = match layers.is_empty() {
                    true => Vec::new(),
                    false => {
                        detect_islands(config.platform_resolution, layers, Progress::new(), true)
                    }
                };
                Self {
                    layers: layers.len(),
                    voxels,
                    defects: layers.iter().map(|l| l.defects).sum(),
                    island_layers: islands.iter().filter(|r| r.len() > 1).count(),
                    model_islands,
                    print_time: format!("{:.3}", common::slice::print_time(layers.iter()).raw()),
                    layer_hashes: layers.iter().map(hash_layer).collect(),
                }
            }
            SliceOutput::Vector(layers) => Self {
                layers: layers.len(),
                voxels: 0,
                defects: 0,
                island_layers: 0,
                model_islands,
                print_time: String::new(),
                layer_hashes: layers.iter().map(hash_vector_layer).collect(),
            },
        }
    }

    /// Where `self` differs from `reference`, if it does.
    pub fn diff(&self, reference: &Self) -> Option<String> {
        if self == reference {
            return None;
        }
        let mut out = String::new();
        for (name, now, then) in [
            ("layers", self.layers as u64, reference.layers as u64),
            ("voxels", self.voxels, reference.voxels),
            ("defects", self.defects, reference.defects),
            (
                "island layers",
                self.island_layers as u64,
                reference.island_layers as u64,
            ),
        ] {
            if now != then {
                let _ = writeln!(out, "{name}: {now} (reference {then})");
            }
        }
        if self.model_islands != reference.model_islands {
            let _ = writeln!(
                out,
                "model islands: {:?} (reference {:?})",
                self.model_islands, reference.model_islands
            );
        }
        if self.print_time != reference.print_time {
            let _ = writeln!(
                out,
                "print time: {} (reference {})",
                self.print_time, reference.print_time
            );
        }
        let differing: Vec<_> = (self.layer_hashes.iter().zip(&reference.layer_hashes))
            .enumerate()
            .filter(|(_, (a, b))| a != b)
            .map(|(i, _)| i)
            .collect();
        if !differing.is_empty() {
            let _ = writeln!(
                out,
                "{} layers differ, first {:?}",
                differing.len(),
                &differing[..differing.len().min(10)]
            );
        }
        Some(out)
    }
}

/// Tests run in parallel and may bless the same file: write it whole or not
/// at all.
fn write_atomic(path: &Path, data: &[u8]) {
    let tmp = path.with_extension(format!("tmp{}", std::process::id()));
    fs::write(&tmp, data).unwrap();
    fs::rename(&tmp, path).unwrap();
}

fn env(name: &str) -> bool {
    std::env::var_os(name).is_some_and(|v| !v.is_empty() && v != "0")
}

/// The path of a reference project, rebuilding it first if asked to (or if
/// it is missing and blessing is on).
pub fn project_path(name: &str) -> PathBuf {
    let path = dir().join(format!("{name}.mslicer"));
    if env("MSLICER_BLESS_PROJECTS") || (!path.exists() && env("MSLICER_BLESS")) {
        save(&build(name), &path);
    }
    path
}

fn check(name: &str) {
    let mut project = load_prepared(&project_path(name));
    let output = slice(&mut project);
    let now = Reference::of(&output, &project);

    let path = dir().join(format!("{name}.ref.json"));
    if env("MSLICER_BLESS") {
        let json = serde_json::to_string_pretty(&now).unwrap() + "\n";
        write_atomic(&path, json.as_bytes());
        return;
    }
    let reference: Reference = serde_json::from_str(
        &fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("{}: {e} (bless with MSLICER_BLESS=1)", path.display())),
    )
    .unwrap();
    if let Some(diff) = now.diff(&reference) {
        panic!("`{name}` slices differently than its reference:\n{diff}");
    }
}

#[test]
fn regression_teapot() {
    check("teapot");
}

#[test]
fn regression_t_shape() {
    check("t_shape");
}

#[test]
fn regression_hollowed() {
    check("hollowed");
}

#[test]
fn regression_supported() {
    check("supported");
}

#[test]
fn regression_vlh_eff() {
    check("vlh_eff");
}

#[test]
fn regression_vector() {
    check("vector");
}

/// The fixtures are what they claim to be, so the harness covers what it
/// says it does.
#[test]
fn regression_fixtures_cover_their_features() {
    let load = |name| load_prepared(&project_path(name));

    let hollowed = load("hollowed");
    let model = &hollowed.models[0];
    assert!(model.hollow.spec.shell.is_some());
    assert_eq!(model.hollow.spec.holes.len(), 2);
    let drilled = model.hollow.drilled(&model.mesh).expect("drilled");
    assert!(drilled.failed.iter().all(Option::is_none), "holes drill");

    let supported = load("supported");
    let (auto, manual) = supported.models[0].supports.world_sets();
    assert!(!auto.supports.is_empty() && manual.supports.len() == 1);

    let vlh = load("vlh_eff");
    assert!(vlh.post_processing.variable_layer_height.enabled);
    assert!(vlh.post_processing.elephant_foot_fixer.enabled);
    assert_eq!(vlh.models.len(), 3);
    assert!(vlh.models[2].hidden);
    assert_eq!(vlh.models[0].mesh.mesh_id(), vlh.models[1].mesh.mesh_id());
}
