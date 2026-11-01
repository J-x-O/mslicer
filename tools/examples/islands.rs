//! Island detection dump.
//!
//! Runs `tools::islands::detect` on STL files as stored (no lift or rotation)
//! and prints one JSON object per model on stdout:
//! `{"model", "ms", "open_planes", "islands": [{"z", "area", "on_plate", "anchor", "min", "max"}]}`.
//!
//!   cargo run --release -p tools --example islands -- [--layer-height MM] [--model-origin] <stl> ...
//!
//! `--route` lifts each model 5 mm (as Auto does) and reports, on stderr, how
//! many floating islands a default support can be routed to: from the first
//! contact candidate, from any, and from any when mini supports are allowed.
//!
//! `--model-origin` puts the planes at `max(lowest − layer height, 0) + k·h`
//! instead of on the plate grid, so the planes follow the model rather than the plate.

use std::{fs, io::BufReader, path::Path, time::Instant};

use anyhow::{Context, Result};
use common::{progress::Progress, serde::ReaderDeserializer};
use mesh_format::Format;
use nalgebra::Vector3;
use slicer::geometry::{bounds::Bounds3D, mesh::Mesh, partitioning::Bvh};
use tools::{
    islands::{Planes, detect},
    supports::{
        auto::{route_hand, route_point},
        mesh::SupportPreset,
    },
};

fn load_stl(path: &Path) -> Result<Mesh> {
    let file = fs::File::open(path).with_context(|| format!("opening {path:?}"))?;
    let des = ReaderDeserializer::new(BufReader::new(file));
    let mesh = mesh_format::load_mesh(des, Format::Stl, &Progress::new())?;
    Ok(Mesh::new_boxed(mesh.verts, mesh.faces))
}

fn main() -> Result<()> {
    let mut layer_height = 0.05;
    let mut model_origin = false;
    let mut route = false;
    let mut paths = Vec::new();
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--layer-height" => layer_height = args.next().context("layer height")?.parse()?,
            "--model-origin" => model_origin = true,
            "--route" => route = true,
            _ => paths.push(arg),
        }
    }

    for path in paths {
        let mut mesh = load_stl(Path::new(&path))?;
        if route {
            let lowest = Bounds3D::from_mesh(&mesh).min.z;
            mesh.set_position(mesh.position() + Vector3::z() * (5.0 - lowest));
        }
        let bvh = Bvh::build(mesh.inner(), Progress::new());
        let mut planes = Planes::layers(layer_height, 1);
        if model_origin {
            let lowest = Bounds3D::from_mesh(&mesh).min.z;
            planes.origin = (lowest - layer_height).max(0.0);
        }

        let start = Instant::now();
        let detection = detect(&mesh, &bvh, planes, &Progress::new());
        let ms = start.elapsed().as_secs_f64() * 1e3;

        if route {
            let preset = SupportPreset::default();
            let floating: Vec<_> = detection.islands.iter().filter(|i| !i.on_plate).collect();
            let routes = |(p, n): &(Vector3<f32>, Vector3<f32>)| {
                route_point(&mesh, &bvh, &preset, *p, *n).is_some()
            };
            let first = floating.iter().filter(|i| routes(&i.contacts[0])).count();
            let any = (floating.iter())
                .filter(|i| i.contacts.iter().any(routes))
                .count();
            let by_hand = |(p, n): &(Vector3<f32>, Vector3<f32>)| {
                route_hand(&mesh, &bvh, &preset, *p, *n, false).is_some()
            };
            let hand = (floating.iter())
                .filter(|i| i.contacts.iter().any(by_hand))
                .count();
            eprintln!(
                "{path}: {} floating islands, routable from first contact {first}, from any {any}, with minis {hand}",
                floating.len()
            );
        }

        let islands: Vec<String> = (detection.islands.iter())
            .map(|i| {
                format!(
                    r#"{{"z":{},"area":{},"on_plate":{},"anchor":[{},{},{}],"min":[{},{}],"max":[{},{}]}}"#,
                    i.z,
                    i.area,
                    i.on_plate,
                    i.anchor.x,
                    i.anchor.y,
                    i.anchor.z,
                    i.min.x,
                    i.min.y,
                    i.max.x,
                    i.max.y
                )
            })
            .collect();
        println!(
            r#"{{"model":{:?},"ms":{ms:.1},"open_planes":{},"islands":[{}]}}"#,
            path,
            detection.open_planes,
            islands.join(",")
        );
    }
    Ok(())
}
