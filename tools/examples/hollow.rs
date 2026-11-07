//! Hollowing dump.
//!
//! Runs `tools::hollow::cavity` on STL files as stored and prints, per model,
//! the time taken, the voids found and their suggested drain holes.
//!
//!   cargo run --release -p tools --example hollow -- [--wall MM] [--cell MM] [--diameter MM] [--out DIR] <stl> ...
//!
//! `--out DIR` writes each cavity to `DIR/<model>_cavity.stl`.
//!
//! `--drill` also builds the mesh as printed (the model plus the cavity's
//! wall facing in), cuts the suggested drain holes into it with
//! `tools::hollow::drill` and reports the time, whether the input and the
//! result are closed, and the volume removed; with `--out`, it writes
//! `DIR/<model>_drilled.stl`.
//!
//! `--repair` runs `tools::repair` on each model first (welds near vertices,
//! drops degenerate faces); `--edges` reports the open and over-shared edges
//! of the model and of the cavity before drilling; `--local` drills with
//! `tools::hollow::drill_local` (only the mesh around each hole).

use std::{fs, io::BufReader, path::Path, time::Instant};

use anyhow::{Context, Result};
use common::{progress::Progress, serde::ReaderDeserializer};
use mesh_format::Format;
use slicer::geometry::mesh::Mesh;
use tools::hollow::{cavity, drill, drill_local, is_closed, signed_volume};

fn load_stl(path: &Path) -> Result<Mesh> {
    let file = fs::File::open(path).with_context(|| format!("opening {path:?}"))?;
    let des = ReaderDeserializer::new(BufReader::new(file));
    let mesh = mesh_format::load_mesh(des, Format::Stl, &Progress::new())?;
    Ok(Mesh::new_boxed(mesh.verts, mesh.faces))
}

fn write_stl(path: &Path, vertices: &[nalgebra::Vector3<f32>], faces: &[[u32; 3]]) -> Result<()> {
    let mut out = vec![0_u8; 80];
    out.extend((faces.len() as u32).to_le_bytes());
    for face in faces {
        let [a, b, c] = face.map(|i| vertices[i as usize]);
        let normal = (b - a)
            .cross(&(c - a))
            .try_normalize(0.0)
            .unwrap_or_default();
        for v in [normal, a, b, c] {
            v.iter().for_each(|x| out.extend(x.to_le_bytes()));
        }
        out.extend([0, 0]);
    }
    fs::write(path, out)?;
    Ok(())
}

fn main() -> Result<()> {
    let (mut wall, mut cell, mut out, mut diameter) = (2.0, 0.5, None, 2.0);
    let (mut drilling, mut repair, mut edges, mut local) = (false, false, false, false);
    let mut paths = Vec::new();
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--wall" => wall = args.next().context("wall")?.parse()?,
            "--cell" => cell = args.next().context("cell")?.parse()?,
            "--diameter" => diameter = args.next().context("diameter")?.parse()?,
            "--out" => out = Some(args.next().context("out")?),
            "--drill" => drilling = true,
            "--repair" => repair = true,
            "--edges" => edges = true,
            "--local" => local = true,
            _ => paths.push(arg),
        }
    }

    for path in paths {
        let mut mesh = load_stl(Path::new(&path))?;
        if repair {
            let repaired = tools::repair::MeshRepair {
                vertex_epsilon: 1e-5,
            }
            .repair(&mesh, &Progress::new());
            println!(
                "  repair: {} unwelded, {} degenerate, {} repeated, {} holes, {} flipped",
                repaired.unwelded_vertices,
                repaired.degenerative_faces,
                repaired.repeated_faces,
                repaired.holes,
                repaired.flipped_winding
            );
            mesh = repaired.mesh;
        }
        let start = Instant::now();
        let cancel = std::sync::atomic::AtomicBool::new(false);
        let cavity = cavity(&mesh, wall, cell, diameter, &Progress::new(), &cancel).unwrap();
        let ms = start.elapsed().as_secs_f64() * 1e3;
        let volume: f32 = cavity.parts.iter().map(|p| p.volume).sum();
        println!(
            "{path}: {ms:.0} ms, cell {:.2} mm, {} faces, {} voids, {volume:.0} mm³, {} drains",
            cavity.cell,
            cavity.faces.len(),
            cavity.parts.len(),
            cavity.drains.len()
        );
        let name = Path::new(&path).file_stem().unwrap().to_string_lossy();
        if let Some(dir) = &out {
            let file = Path::new(dir).join(format!("{name}_cavity.stl"));
            write_stl(&file, &cavity.vertices, &cavity.faces)?;
        }
        if drilling && !cavity.faces.is_empty() {
            let mut vertices = mesh.vertices().to_vec();
            let mut faces = mesh.faces().to_vec();
            let offset = vertices.len() as u32;
            vertices.extend(&cavity.vertices);
            faces.extend((cavity.faces.iter()).map(|f| [f[0], f[2], f[1]].map(|i| i + offset)));
            let before = signed_volume(&vertices, &faces);
            let closed = is_closed(&faces);
            if edges {
                let report = |label: &str, v: &[nalgebra::Vector3<f32>], f: &[[u32; 3]]| {
                    let (open, shared) = edge_report(v, f);
                    println!("  {label}: {open} open edges, {shared} over-shared edges");
                };
                report("model", mesh.vertices(), mesh.faces());
                report("cavity", &cavity.vertices, &cavity.faces);
            }

            let start = Instant::now();
            if local && cavity.drains.len() > 1 {
                for (k, drain) in cavity.drains.iter().enumerate() {
                    let start = Instant::now();
                    let one = drill_local(&vertices, &faces, std::slice::from_ref(drain));
                    let ms = start.elapsed().as_secs_f64() * 1e3;
                    match one {
                        Ok((v, f)) => println!(
                            "  hole {k} alone: {ms:.0} ms, {:?} open/over-shared, removed {:.2} mm³",
                            edge_report(&v, &f),
                            before - signed_volume(&v, &f)
                        ),
                        Err(err) => println!("  hole {k} alone: failed: {err}"),
                    }
                }
            }
            let result = match local {
                true => drill_local(&vertices, &faces, &cavity.drains),
                false => drill(&vertices, &faces, &cavity.drains),
            };
            let ms = start.elapsed().as_secs_f64() * 1e3;
            match result {
                Ok((v, f)) => {
                    println!(
                        "  drill: {ms:.0} ms, input closed {closed}, result closed {} ({:?} open/over-shared), {} -> {} faces, removed {:.2} mm³",
                        is_closed(&f),
                        edge_report(&v, &f),
                        faces.len(),
                        f.len(),
                        before - signed_volume(&v, &f)
                    );
                    if let Some(dir) = &out {
                        let file = Path::new(dir).join(format!("{name}_drilled.stl"));
                        write_stl(&file, &v, &f)?;
                    }
                }
                Err(err) => {
                    println!("  drill: failed after {ms:.0} ms (input closed {closed}): {err}")
                }
            }
        }
    }
    Ok(())
}

/// Edges (after welding equal positions) used by one face only, and by more
/// than two faces.
fn edge_report(vertices: &[nalgebra::Vector3<f32>], faces: &[[u32; 3]]) -> (usize, usize) {
    use std::collections::HashMap;
    let mut weld = HashMap::new();
    let ids: Vec<usize> = (vertices.iter())
        .map(|v| {
            let key = v.map(f32::to_bits);
            let next = weld.len();
            *weld.entry((key.x, key.y, key.z)).or_insert(next)
        })
        .collect();
    let mut edges: HashMap<(usize, usize), usize> = HashMap::new();
    for f in faces {
        let f = f.map(|i| ids[i as usize]);
        if f[0] == f[1] || f[1] == f[2] || f[0] == f[2] {
            continue;
        }
        for (a, b) in [(f[0], f[1]), (f[1], f[2]), (f[2], f[0])] {
            *edges.entry((a.min(b), a.max(b))).or_insert(0) += 1;
        }
    }
    let open = edges.values().filter(|&&n| n == 1).count();
    let shared = edges.values().filter(|&&n| n > 2).count();
    (open, shared)
}
