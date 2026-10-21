//! Support comparison harness (plan §9.4 item 3).
//!
//! Loads every `<name>.stl` in `tools/tests/models/`, runs mslicer's own
//! auto-support generator, meshes the result, and reports the metrics that are
//! recoverable from plain STL exports, side by side with the CHITUBOX reference
//! (`<name>_supports.stl` = supports only, `<name>_supported.stl` = model +
//! supports fused).
//!
//! What is *not* comparable yet: point count. CHITUBOX exports are already
//! meshed and merge neighbouring tips into shared trunks, so the number of
//! bodies in `<name>_supports.stl` is not the number of points CHITUBOX placed.
//! A meaningful ±25% point-count check only becomes possible after M4/M5 give
//! our supports the same merging structure. Until then this harness reports the
//! structural invariants that *are* comparable: support volume, triangle count,
//! model/triangle intersection count, and generation vs meshing time. `sups`
//! below is *our own* point count, reported for reference only.
//!
//! Run from the repo root:
//!   cargo run -p tools --example support_metrics -- [<model-name> ...]
//! With no arguments it processes every model that has a `<name>_supports.stl`
//! next to it.

use std::{
    fs,
    io::BufReader,
    path::{Path, PathBuf},
    time::Instant,
};

use anyhow::{Context, Result};
use mesh_format::Format;
use nalgebra::Vector3;

use common::{
    progress::Progress,
    serde::ReaderDeserializer,
    units::{Milimeter, Milimeters},
};
use slicer::geometry::{mesh::Mesh, partitioning::Bvh};
use tools::supports::{
    auto::{AutoPlacement, PlacementConfig},
    mesh::{SupportPreset, Transform, build_supports},
    parts::PartKind,
};

/// Signed volume of a watertight triangle mesh, in mm³ (via the divergence
/// theorem over the tetrahedra anchored at the origin).
fn mesh_volume(mesh: &Mesh) -> f32 {
    let mut volume = 0.0f32;
    // The divergence theorem over the tetrahedra anchored at the origin. Works
    // for any watertight mesh with outward-facing winding.
    for face in mesh.faces() {
        let a = mesh.transform(&mesh.vertices()[face[0] as usize]);
        let b = mesh.transform(&mesh.vertices()[face[1] as usize]);
        let c = mesh.transform(&mesh.vertices()[face[2] as usize]);
        volume += a.dot(&b.cross(&c)) / 6.0;
    }

    volume
}

/// Loads an STL into a `slicer::geometry::mesh::Mesh` in raw (identity) space.
fn load_stl(path: &Path) -> Result<Mesh> {
    let file = fs::File::open(path).with_context(|| format!("opening {path:?}"))?;
    let des = ReaderDeserializer::new(BufReader::new(file));
    let mesh = mesh_format::load_mesh(des, Format::Stl, &Progress::new())?;
    Ok(Mesh::new_boxed(mesh.verts, mesh.faces))
}

/// Counts support-mesh edges that pierce the model, using the model's existing
/// BVH. Each support triangle edge is a segment; if it intersects the model,
/// the support triangle overlaps the model. This is the M3 acceptance metric
/// ("zero part–model intersections") made measurable, and it reuses the
/// already-built model BVH so it stays fast on large meshes.
fn triangle_intersection_count(model: &Mesh, model_bvh: &Bvh, support_mesh: &Mesh) -> usize {
    let mut count = 0;
    for face in support_mesh.faces() {
        let a = support_mesh.transform(&support_mesh.vertices()[face[0] as usize]);
        let b = support_mesh.transform(&support_mesh.vertices()[face[1] as usize]);
        let c = support_mesh.transform(&support_mesh.vertices()[face[2] as usize]);

        for (p0, p1) in [(a, b), (b, c), (c, a)] {
            if model_bvh.intersect_segment(model, p0, p1).is_some() {
                count += 1;
                break;
            }
        }
    }

    count
}

struct ModelReport {
    name: String,
    our_supports: usize,
    our_triangles: usize,
    our_volume: f32,
    chitubox_triangles: usize,
    chitubox_volume: f32,
    part_kinds: Vec<(PartKind, usize)>,
    intersections: usize,
    generate_ms: u128,
    build_ms: u128,
}

fn process_model(model_path: &Path) -> Result<ModelReport> {
    let mut model = load_stl(model_path)?;

    // Lift the model by the default lift, exactly as the app's generate button
    // does (`generate_support` in panels/supports.rs). The models sit at z=0,
    // so without this the routing has no vertical room and drops every support.
    let default_lift = 5.0;
    let pos = model.position();
    model.set_position(pos.xy().push(pos.z.max(default_lift)));

    let progress = Progress::new();
    let bvh = Bvh::build(model.inner(), Progress::new());

    // Build the auto placement exactly as the app's generate button does.
    let bed = Vector3::repeat(Milimeters::new(218.88)); // Saturn 4 Ultra 16K xy
    let placement = AutoPlacement::new(PlacementConfig::default(), SupportPreset::default(), bed);

    let generate_start = Instant::now();
    let routed = {
        let half_edge = slicer::geometry::mesh::HalfEdgeMesh::build_from_mesh(model.inner());
        placement.generate_supports(&model, &half_edge, &bvh, &progress)
    };
    let generate_ms = generate_start.elapsed().as_millis();

    // Expand routed triplets [contact, tip, joint] into a SupportSet so we can
    // mesh them with the same code path the app uses.
    let mut set = tools::supports::SupportSet::default();
    for (i, [contact, tip, joint]) in routed.iter().enumerate() {
        set.add_routed(
            tools::supports::SupportId::from_raw(i as u32),
            placement.support.tip_radius.get::<Milimeter>(),
            placement.support.radius.get::<Milimeter>(),
            [*contact, *tip, *joint],
        );
    }

    let build_start = Instant::now();
    let (mesh, _map) = build_supports(&set, Transform::default(), 24)
        .map(|(m, map)| (m, map))
        .unwrap_or_else(|| (Mesh::default(), Default::default()));
    let build_ms = build_start.elapsed().as_millis();

    let our_volume = mesh_volume(&mesh);
    let our_triangles = mesh.face_count();

    // Part-kind histogram.
    let mut part_kinds: Vec<(PartKind, usize)> = Vec::new();
    for support in &set.supports {
        for part in &support.parts {
            if let Some((_, count)) = part_kinds.iter_mut().find(|(k, _)| *k == part.kind) {
                *count += 1;
            } else {
                part_kinds.push((part.kind, 1));
            }
        }
    }
    part_kinds.sort_by_key(|(k, _)| format!("{k:?}"));

    // CHITUBOX reference: supports-only, and model+supports fused.
    let stem = model_path.with_extension("");
    let chitubox_path = {
        let mut name = stem.file_name().unwrap().to_os_string();
        name.push("_supports.stl");
        model_path.with_file_name(name)
    };

    let (chitubox_triangles, chitubox_volume) = if chitubox_path.exists() {
        let ch = load_stl(&chitubox_path)?;
        (ch.face_count(), mesh_volume(&ch))
    } else {
        (0, f32::NAN)
    };

    // Model–support triangle intersection count. NOTE: this includes the
    // intentional contact sink — the ContactSphere and TipCone start *inside*
    // the model by design — so the number is never zero. Its value is a rough
    // "how much overlap" signal that should shrink as routing/M3 tighten the
    // geometry; a proper "unintended intersection" metric needs per-part face
    // attribution, which is a later refinement.
    let intersections = triangle_intersection_count(&model, &bvh, &mesh);

    Ok(ModelReport {
        name: model_path
            .file_stem()
            .unwrap()
            .to_string_lossy()
            .into_owned(),
        our_supports: routed.len(),
        our_triangles,
        our_volume,
        chitubox_triangles,
        chitubox_volume,
        part_kinds,
        intersections,
        generate_ms,
        build_ms,
    })
}

fn main() -> Result<()> {
    let models_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("models");

    let selected: Vec<String> = std::env::args().skip(1).collect();

    let mut reports = Vec::new();
    for entry in fs::read_dir(&models_dir)? {
        let path = entry?.path();
        if path.extension().and_then(|e| e.to_str()) != Some("stl") {
            continue;
        }
        // Skip CHITUBOX-derived exports; only process the original model.
        let name = path.file_stem().unwrap().to_string_lossy();
        if name.ends_with("_supported") || name.ends_with("_supports") {
            continue;
        }
        if !selected.is_empty() && !selected.iter().any(|s| s == &name) {
            continue;
        }

        reports.push(process_model(&path)?);
    }

    reports.sort_by(|a, b| a.name.cmp(&b.name));

    println!(
        "\n{:<28} {:>6} {:>10} {:>10} {:>10} {:>10} {:>6} {:>6} {:>6}",
        "model", "sups", "our_vol", "cb_vol", "our_tris", "cb_tris", "inter", "gen_ms", "build_ms"
    );
    for r in &reports {
        let cb_vol = if r.chitubox_volume.is_nan() {
            "–".to_string()
        } else {
            format!("{:.2}", r.chitubox_volume)
        };
        println!(
            "{:<28} {:>6} {:>10.2} {:>10} {:>10} {:>10} {:>6} {:>6} {:>6}",
            r.name,
            r.our_supports,
            r.our_volume,
            cb_vol,
            r.our_triangles,
            r.chitubox_triangles,
            r.intersections,
            r.generate_ms,
            r.build_ms,
        );

        if !r.part_kinds.is_empty() {
            let kinds = r
                .part_kinds
                .iter()
                .map(|(k, c)| format!("{k:?}:{c}"))
                .collect::<Vec<_>>()
                .join(" ");
            println!("    parts: {kinds}");
        }
    }
    println!();

    Ok(())
}
