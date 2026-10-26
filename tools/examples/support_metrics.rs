//! Support comparison harness.
//!
//! Loads every `<name>.stl` in `tools/tests/models/`, runs mslicer's own
//! auto-support generator, meshes the result, and reports the metrics that are
//! recoverable from plain STL exports, side by side with a reference export
//! next to the model (`<name>_supports.stl` = supports only,
//! `<name>_supported.stl` = model + supports fused).
//!
//! Point counts are not compared: an exported mesh merges neighbouring tips
//! into shared trunks, so its body count is not its point count. The harness
//! reports what *is* comparable: support volume, triangle count,
//! model/triangle intersection count, and generation vs meshing time. `sups`
//! below is *our own* point count.
//!
//! Run from the repo root:
//!   cargo run -p tools --example support_metrics -- [<model-name> ...]
//! With no arguments it processes every model that has a `<name>_supports.stl`
//! next to it. `PRESET=light|middle|heavy` picks the support preset (default
//! middle). `RENDER=<dir>` writes top and side depth renders of model plus
//! supports to `<dir>/<model>_{top,side}.png`, braces in red.
//! `DUMP_STL=<dir>` writes our support mesh to `<dir>/<model>_ours.stl`.
//! `DUMP_POINTS=<dir>` writes the contacts and routes as text, and
//! `ROTATE_X=<deg>` tilts the model about x first.

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
    SupportSet,
    auto::{AutoPlacement, PlacementConfig, collision},
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

/// Orthographic depth render for eyeballing output. `axes` picks the image
/// x, image y (up) and depth (towards the viewer) world axes; each mesh gets a
/// base colour, shaded by depth.
fn render(meshes: &[(&Mesh, [u8; 3])], [u, v, w]: [usize; 3], path: &Path) -> Result<()> {
    const PX: f32 = 0.1; // mm per pixel
    let points = |m: &Mesh| -> Vec<Vector3<f32>> {
        (m.vertices().iter())
            .map(|p| {
                let p = m.transform(p);
                Vector3::new(p[u], p[v], p[w])
            })
            .collect()
    };
    let all: Vec<_> = meshes.iter().map(|(m, c)| (m, points(m), *c)).collect();
    let (mut min, mut max) = (Vector3::repeat(f32::MAX), Vector3::repeat(f32::MIN));
    for p in all.iter().flat_map(|(_, p, _)| p) {
        min = min.inf(p);
        max = max.sup(p);
    }
    let (width, height) = (
        ((max.x - min.x) / PX) as u32 + 1,
        ((max.y - min.y) / PX) as u32 + 1,
    );
    let mut depth = vec![f32::MIN; (width * height) as usize];
    let mut image = image::RgbImage::new(width, height);
    for (layer, (mesh, pts, colour)) in all.iter().enumerate() {
        // Meshes past the first two are overlays of geometry already drawn
        // (e.g. the braces inside the support mesh): let them win depth ties.
        let bias = if layer >= 2 { 0.05 } else { 0.0 };
        for face in mesh.faces() {
            let [a, b, c] = face.map(|i| {
                let p = pts[i as usize];
                Vector3::new((p.x - min.x) / PX, (max.y - p.y) / PX, p.z)
            });
            let area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
            if area.abs() < 1e-9 {
                continue;
            }
            let x0 = a.x.min(b.x).min(c.x).max(0.0) as u32;
            let x1 = (a.x.max(b.x).max(c.x) as u32).min(width - 1);
            let y0 = a.y.min(b.y).min(c.y).max(0.0) as u32;
            let y1 = (a.y.max(b.y).max(c.y) as u32).min(height - 1);
            for y in y0..=y1 {
                for x in x0..=x1 {
                    let (px, py) = (x as f32 + 0.5, y as f32 + 0.5);
                    let wa = ((b.x - px) * (c.y - py) - (b.y - py) * (c.x - px)) / area;
                    let wb = ((c.x - px) * (a.y - py) - (c.y - py) * (a.x - px)) / area;
                    let wc = 1.0 - wa - wb;
                    if wa < 0.0 || wb < 0.0 || wc < 0.0 {
                        continue;
                    }
                    let z = wa * a.z + wb * b.z + wc * c.z + bias;
                    let i = (y * width + x) as usize;
                    if z > depth[i] {
                        depth[i] = z;
                        let shade = 0.35 + 0.65 * (z - min.z) / (max.z - min.z).max(1e-6);
                        let rgb = colour.map(|ch| (ch as f32 * shade) as u8);
                        image.put_pixel(x, y, image::Rgb(rgb));
                    }
                }
            }
        }
    }
    image.save(path)?;
    Ok(())
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
/// the support triangle overlaps the model. It reuses the already-built model
/// BVH so it stays fast on large meshes.
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

/// Brace and tail sanity checks, measured on the generated set.
struct Acceptance {
    /// Pillars taller than 10 mm with no brace and no stabilising leg.
    unbraced_tall: usize,
    /// Pillars taller than 10 mm, for scale.
    tall: usize,
    /// Braces (spanning-tree, fork links, leg braces) that fail the link test.
    failed_braces: usize,
    /// Brace ends that don't touch any pillar (floating in mid-air).
    floating_ends: usize,
    /// Shallowest and deepest tail sink into the lower surface, in mm.
    tail_sink: Option<(f32, f32)>,
}

/// Runs the sanity checks: every pillar over 10 mm is braced or has a
/// leg, every brace passes the link test, and tails sink 0.2–0.3 mm.
fn acceptance(model: &Mesh, bvh: &Bvh, set: &SupportSet) -> Acceptance {
    const TALL: f32 = 10.0;
    const SAME_XY: f32 = 1e-4;

    let braces: Vec<_> = (set.braces.iter())
        .filter(|b| b.kind == PartKind::Brace)
        .collect();
    // Every brace end must sit on some vertical pillar, within its height.
    let pillars: Vec<_> = (set.supports.iter())
        .flat_map(|s| s.parts.iter().filter(|p| p.kind == PartKind::Pillar))
        .collect();
    let on_pillar = |end: Vector3<f32>| {
        pillars.iter().any(|p| {
            (p.a.xy() - end.xy()).norm() < 1e-3 && end.z <= p.a.z + 1e-3 && end.z >= p.b.z - 1e-3
        })
    };
    let floating_ends = (braces.iter())
        .map(|b| !on_pillar(b.a) as usize + !on_pillar(b.b) as usize)
        .sum();

    let failed_braces = (braces.iter())
        .filter(|b| collision::link_test(model, bvh, b.a, b.b, b.radius_a))
        .count();

    // A leg's contact sits on its pillar's surface, exactly one pillar radius
    // from the axis; model contacts practically never land there.
    let contacts: Vec<_> = set.supports.iter().map(|s| s.point.pos).collect();

    let (mut tall, mut unbraced_tall) = (0, 0);
    for support in &set.supports {
        for pillar in support.parts.iter().filter(|p| p.kind == PartKind::Pillar) {
            if pillar.a.z - pillar.b.z <= TALL {
                continue;
            }
            tall += 1;
            let xy = pillar.a.xy();
            let braced = (braces.iter())
                .any(|b| (b.a.xy() - xy).norm() < SAME_XY || (b.b.xy() - xy).norm() < SAME_XY);
            let legged = (contacts.iter())
                .any(|c| ((c.xy() - xy).norm() - pillar.radius_a).abs() < 1e-3 && c.z < pillar.a.z);
            if !braced && !legged {
                unbraced_tall += 1;
            }
        }
    }

    // A tail's sink is how far its end lies past where its axis meets the
    // model.
    let tail_sink = (set.supports.iter())
        .flat_map(|s| s.parts.iter().filter(|p| p.kind == PartKind::Tail))
        .filter_map(|tail| {
            let hit = bvh.intersect_segment(model, tail.a, tail.b)?;
            Some((tail.b - model.transform(&hit.position)).norm())
        })
        .fold(None, |acc: Option<(f32, f32)>, d| {
            Some(acc.map_or((d, d), |(lo, hi)| (lo.min(d), hi.max(d))))
        });

    Acceptance {
        unbraced_tall,
        tall,
        failed_braces,
        floating_ends,
        tail_sink,
    }
}

struct ModelReport {
    name: String,
    our_supports: usize,
    our_landings: usize,
    our_triangles: usize,
    our_volume: f32,
    reference_triangles: usize,
    reference_volume: f32,
    part_kinds: Vec<(PartKind, usize)>,
    raft_islands: usize,
    intersections: usize,
    generate_ms: u128,
    build_ms: u128,
    acceptance: Acceptance,
}

fn process_model(model_path: &Path) -> Result<ModelReport> {
    let mut model = load_stl(model_path)?;

    // Optional pre-placement rotation about the buildplate X axis, so the same
    // base geometry can be stress-tested tilted (`ROTATE_X=-20` reproduces the
    // `…_20_deg` references). Applied before the lift so the rotated
    // bounding box drives the align-to-bed step, as a model rotated on the
    // plate would be.
    if let Ok(deg) = std::env::var("ROTATE_X") {
        let deg: f32 = deg.parse()?;
        let prev = model.rotation();
        model.set_rotation(Vector3::new(prev.x + deg.to_radians(), prev.y, prev.z));
    }

    // Lift so the model's *lowest point* sits at `default_lift`, mirroring the
    // app's corrected generate path (align-to-bed then lift). The STLs sit at
    // their raw coordinates with identity transform, so `bounds.min.z` is the
    // lowest world point; without this there's no vertical room to route.
    let default_lift = 5.0;
    let min_z = slicer::geometry::bounds::Bounds3D::from_mesh(&model).min.z;
    let pos = model.position();
    model.set_position(Vector3::new(pos.x, pos.y, pos.z + (default_lift - min_z)));

    let progress = Progress::new();
    let bvh = Bvh::build(model.inner(), Progress::new());

    // Build the auto placement exactly as the app's generate button does.
    let bed = Vector3::repeat(Milimeters::new(218.88)); // Saturn 4 Ultra 16K xy
    let preset = match std::env::var("PRESET").as_deref() {
        Ok("light") => SupportPreset::LIGHT,
        Ok("heavy") => SupportPreset::HEAVY,
        _ => SupportPreset::MIDDLE,
    };
    let placement = AutoPlacement::new(PlacementConfig::default(), preset, bed);

    let generate_start = Instant::now();
    let generated = { placement.generate_supports(&model, &bvh, &progress) };
    let generate_ms = generate_start.elapsed().as_millis();

    // Expand routed triplets [contact, tip, joint] into a SupportSet so we can
    // mesh them with the same code path the app uses, and add the brace network
    // computed over the pillar tops. Model-landing supports go after the plate
    // supports so the network's support indices stay valid.
    let tip_radius = placement.support.tip_radius.get::<Milimeter>();
    let radius = placement.support.radius.get::<Milimeter>();
    let contact_depth = placement.support.contact_depth.get::<Milimeter>();
    let mut set = tools::supports::SupportSet::default();
    for (i, triplet) in generated.plate.iter().enumerate() {
        set.add_routed(
            tools::supports::SupportId::from_raw(i as u32),
            tip_radius,
            radius,
            contact_depth,
            *triplet,
        );
    }
    for (i, landing) in generated.landings.iter().enumerate() {
        let id = (generated.plate.len() + i) as u32;
        set.add_landing(
            tools::supports::SupportId::from_raw(id),
            tip_radius,
            radius,
            landing,
        );
    }
    let network = placement.generate_braces(&model, &bvh, &generated);
    for (i, leg) in network.legs.iter().enumerate() {
        let id = (generated.plate.len() + generated.landings.len() + i) as u32;
        set.add_routed(
            tools::supports::SupportId::from_raw(id),
            tip_radius,
            radius,
            contact_depth,
            *leg,
        );
    }
    let brace_set = tools::supports::SupportSet {
        braces: network.braces.clone(),
        ..Default::default()
    };
    set.braces = network.braces;
    for (index, bottom_z) in network.fork_bottoms {
        set.truncate_pillar(index, bottom_z);
    }
    set.add_feet(
        &placement.support.base_config(),
        &placement.support.raft_config(),
    );

    let build_start = Instant::now();
    let (mesh, _map) = build_supports(&set, Transform::default(), 24)
        .unwrap_or_else(|| (Mesh::default(), Default::default()));
    let build_ms = build_start.elapsed().as_millis();

    if let Ok(dir) = std::env::var("RENDER") {
        let name = model_path.file_stem().unwrap().to_string_lossy();
        for (view, axes) in [("top", [0, 1, 2]), ("side", [0, 2, 1])] {
            let path = Path::new(&dir).join(format!("{name}_{view}.png"));
            let brace_mesh = build_supports(&brace_set, Transform::default(), 24)
                .map(|(m, _)| m)
                .unwrap_or_default();
            render(
                &[
                    (&model, [200, 120, 80]),
                    (&mesh, [90, 160, 230]),
                    (&brace_mesh, [230, 40, 40]),
                ],
                axes,
                &path,
            )?;
        }
    }

    // `DUMP_POINTS=<dir>`: one line per contact, `kind x y z`, world space.
    if let Ok(dir) = std::env::var("DUMP_POINTS") {
        let name = model_path.file_stem().unwrap().to_string_lossy();
        let mut out = String::new();
        for [contact, ..] in &generated.plate {
            out += &format!("plate {} {} {}\n", contact.x, contact.y, contact.z);
        }
        for landing in &generated.landings {
            let c = landing.contact;
            out += &format!("landing {} {} {}\n", c.x, c.y, c.z);
        }
        std::fs::write(Path::new(&dir).join(format!("{name}_points.txt")), out)?;
        let mut routes = String::new();
        for path in &generated.plate {
            for p in path {
                routes += &format!("{} {} {} ", p.x, p.y, p.z);
            }
            routes += "\n";
        }
        std::fs::write(Path::new(&dir).join(format!("{name}_routes.txt")), routes)?;
    }

    if let Ok(dir) = std::env::var("DUMP_STL") {
        let name = model_path.file_stem().unwrap().to_string_lossy();
        let mut out = vec![0u8; 80];
        out.extend((mesh.face_count() as u32).to_le_bytes());
        for i in 0..mesh.face_count() {
            out.extend([0u8; 12]);
            for v in mesh.face_verts(i) {
                for c in [v.x, v.y, v.z] {
                    out.extend(c.to_le_bytes());
                }
            }
            out.extend([0u8; 2]);
        }
        std::fs::write(Path::new(&dir).join(format!("{name}_ours.stl")), out)?;
    }

    let our_volume = mesh_volume(&mesh);
    let our_triangles = mesh.face_count();

    // Part-kind histogram, including the top-level brace network.
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
    for brace in &set.braces {
        if let Some((_, count)) = part_kinds.iter_mut().find(|(k, _)| *k == brace.kind) {
            *count += 1;
        } else {
            part_kinds.push((brace.kind, 1));
        }
    }
    part_kinds.sort_by_key(|(k, _)| format!("{k:?}"));

    // The reference export: supports only.
    let stem = model_path.with_extension("");
    let reference_path = {
        let mut name = stem.file_name().unwrap().to_os_string();
        // A rotated run compares against the rotation-tagged reference
        // (`…_20_deg_supports.stl`), not the unrotated one.
        if let Ok(deg) = std::env::var("ROTATE_X")
            && let Ok(deg) = deg.parse::<f32>()
        {
            let tag = format!("_{}_deg", (deg.abs() as i32));
            name.push(&tag);
        }
        name.push("_supports.stl");
        model_path.with_file_name(name)
    };

    let (reference_triangles, reference_volume) = if reference_path.exists() {
        let reference = load_stl(&reference_path)?;
        (reference.face_count(), mesh_volume(&reference))
    } else {
        (0, f32::NAN)
    };

    // Model–support triangle intersection count. NOTE: this includes the
    // intentional contact sink — the ContactSphere and TipCone start *inside*
    // the model by design — so the number is never zero. Its value is a rough
    // "how much overlap" signal; a proper "unintended intersection" metric needs per-part face
    // attribution, which is a later refinement.
    let intersections = triangle_intersection_count(&model, &bvh, &mesh);
    let acceptance = acceptance(&model, &bvh, &set);

    Ok(ModelReport {
        name: model_path
            .file_stem()
            .unwrap()
            .to_string_lossy()
            .into_owned(),
        our_supports: generated.plate.len() + generated.landings.len(),
        our_landings: generated.landings.len(),
        our_triangles,
        our_volume,
        reference_triangles,
        reference_volume,
        part_kinds,
        raft_islands: set.raft.as_ref().map_or(0, |r| r.islands.len()),
        intersections,
        acceptance,
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
        // Skip reference exports; only process the original model.
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
        "\n{:<28} {:>6} {:>6} {:>10} {:>10} {:>10} {:>10} {:>6} {:>6} {:>6}",
        "model",
        "sups",
        "land",
        "our_vol",
        "ref_vol",
        "our_tris",
        "ref_tris",
        "inter",
        "gen_ms",
        "build_ms"
    );
    for r in &reports {
        let ref_vol = if r.reference_volume.is_nan() {
            "–".to_string()
        } else {
            format!("{:.2}", r.reference_volume)
        };
        println!(
            "{:<28} {:>6} {:>6} {:>10.2} {:>10} {:>10} {:>10} {:>6} {:>6} {:>6}",
            r.name,
            r.our_supports,
            r.our_landings,
            r.our_volume,
            ref_vol,
            r.our_triangles,
            r.reference_triangles,
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
            println!("    parts: {kinds} raft_islands:{}", r.raft_islands);
        }

        let a = &r.acceptance;
        let sink = a
            .tail_sink
            .map_or("–".to_string(), |(lo, hi)| format!("{lo:.3}–{hi:.3} mm"));
        println!(
            "    accept: unbraced>10mm {}/{} failed_braces {} floating_brace_ends {} tail_sink {sink}",
            a.unbraced_tall, a.tall, a.failed_braces, a.floating_ends
        );
    }
    println!();

    Ok(())
}
