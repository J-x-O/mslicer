use std::f32::consts::TAU;

use common::{
    progress::Progress,
    units::{Milimeter, Milimeters},
};
use nalgebra::Vector3;
use slicer::geometry::{
    mesh::{HalfEdgeMesh, Mesh, orthogonal_basis},
    partitioning::Bvh,
};
use tracing::info;

use crate::supports::mesh::SupportPreset;

pub mod detect;
pub mod quazirandom;

pub struct AutoPlacement {
    pub placement: PlacementConfig,
    pub support: SupportPreset,

    pub bed_size: Vector3<Milimeters>,
}

#[derive(Clone)]
pub struct PlacementConfig {
    pub min_spacing: f32,
    pub max_angle: f32,
    pub face_support_spacing: f32,
    pub edge_support_spacing: f32,
    pub edge_angle_delta: f32,
}

impl AutoPlacement {
    pub fn new(
        placement: PlacementConfig,
        support: SupportPreset,
        bed_size: Vector3<Milimeters>,
    ) -> Self {
        Self {
            placement,
            support,

            bed_size,
        }
    }

    pub fn generate_supports(
        &self,
        mesh: &Mesh,
        half_edge: &HalfEdgeMesh,
        bvh: &Bvh,
        progress: &Progress,
    ) -> Vec<[Vector3<f32>; 3]> {
        let mut overhangs = Vec::new();
        let min_dist = self.placement.min_spacing;

        let overhanging_faces = self.overhanging_faces(mesh);
        let mut faces = self.place_face_supports(mesh, &overhanging_faces);
        let mut edges = self.place_edge_supports(mesh, half_edge, &overhanging_faces);
        for overhang in edges.iter() {
            // i know its n²... shut up.
            faces.retain(|x| (x.point - overhang.point).magnitude() > min_dist);
        }

        let points = self.place_point_supports(mesh, half_edge);
        for overhang in points.iter() {
            faces.retain(|x| (x.point - overhang.point).magnitude() > min_dist);
            edges.retain(|x| (x.point - overhang.point).magnitude() > min_dist);
        }

        info!(
            "Placed {} supports. {{ point: {}, face: {}, edge: {} }}",
            points.len() + faces.len() + edges.len(),
            points.len(),
            faces.len(),
            edges.len()
        );
        overhangs.extend([points, faces, edges].into_iter().flatten());

        progress.set_total(overhangs.len() as u64);
        let radius = (self.support.radius + self.support.margin).get::<Milimeter>();
        overhangs
            .into_iter()
            .filter_map(|x| {
                let tip_start = x.point + x.normal * self.support.tip_length.get::<Milimeter>();
                let mid = route_support(mesh, bvh, radius, x.point, tip_start);
                progress.add_complete(1);
                mid.map(|mid| [x.point, tip_start, mid])
            })
            .collect()
    }
}

/// Returns the middle of the three points defining a support. The final point
/// (that touches the build plate) is just this returned point projected down.
///
/// The support is rejected if the tip (`contact` → `position`) passes through
/// the model. The arm from `position` to the returned point always descends at
/// least 45°.
pub fn route_support(
    mesh: &Mesh,
    bvh: &Bvh,
    radius: f32,
    contact: Vector3<f32>,
    position: Vector3<f32>,
) -> Option<Vector3<f32>> {
    if intersect_tip(mesh, bvh, contact, position) {
        return None;
    }

    let mut point = position;
    let mut momentum = Vector3::zeros();
    let beta = 0.9;

    // todo: try searching through angles instead of the gradient decent
    for _ in 0..100 {
        let closest = bvh.closest(mesh, point).unwrap();
        let grad = point - mesh.transform(&closest.position);

        momentum = beta * momentum + (1.0 - beta) * grad;
        point += momentum.xy().push(momentum.z.min(0.0)).normalize() * closest.t.min(1.0);

        // Drop the joint until the arm descends at least 45°. The walk never
        // moves up and the run only grows, so once that is below the plate it
        // won't recover.
        let run = (point.xy() - position.xy()).magnitude();
        let mut joint = point.xy().push(point.z.min(position.z - run));
        if joint.z <= 0.0 {
            return None;
        }

        if !intersect_cylinder(mesh, bvh, radius, (position, joint))
            && !intersect_cylinder(mesh, bvh, radius, (joint, joint.xy().to_homogeneous()))
        {
            // Steepen the arm further while it stays clear of the model.
            while joint.z > 2.0
                && !intersect_cylinder(mesh, bvh, radius, (position, joint - Vector3::z()))
            {
                joint.z -= 1.0
            }

            debug_assert!(position.z - joint.z >= run - 1e-3, "arm flatter than 45°");
            return Some(joint);
        }
    }

    None
}

/// Checks if the tip, from the contact point on the surface to its end, passes
/// through the model. The segment starts just off the surface so the contact
/// face itself isn't counted.
fn intersect_tip(mesh: &Mesh, bvh: &Bvh, contact: Vector3<f32>, end: Vector3<f32>) -> bool {
    let start = contact + (end - contact).normalize() * 0.01;
    bvh.intersect_segment(mesh, start, end).is_some()
}

fn intersect_cylinder(
    mesh: &Mesh,
    bvh: &Bvh,
    radius: f32,
    (start, end): (Vector3<f32>, Vector3<f32>),
) -> bool {
    let n = 4;

    let [u, v] = orthogonal_basis((end - start).normalize());
    for i in 0..n {
        let t = i as f32 / n as f32 * TAU;
        let offset = (u * t.cos() + v * t.sin()) * radius;

        if bvh
            .intersect_segment(mesh, start + offset, end + offset)
            .is_some()
        {
            return true;
        }
    }

    false
}

impl Default for PlacementConfig {
    fn default() -> Self {
        Self {
            min_spacing: 5.0,
            max_angle: 30.0,
            face_support_spacing: 50.0,
            edge_angle_delta: 0.1,
            edge_support_spacing: 20.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use common::{progress::Progress, units::Milimeters};
    use nalgebra::Vector3;
    use slicer::geometry::{
        mesh::{HalfEdgeMesh, Mesh},
        partitioning::Bvh,
    };

    use super::{AutoPlacement, PlacementConfig, intersect_tip, route_support};
    use crate::supports::mesh::SupportPreset;

    /// Appends an axis-aligned box with outward facing triangles.
    fn add_box(
        verts: &mut Vec<Vector3<f32>>,
        faces: &mut Vec<[u32; 3]>,
        min: Vector3<f32>,
        max: Vector3<f32>,
    ) {
        let base = verts.len() as u32;
        for i in 0..8 {
            let pick = |bit, lo: f32, hi: f32| if i & bit == 0 { lo } else { hi };
            verts.push(Vector3::new(
                pick(1, min.x, max.x),
                pick(2, min.y, max.y),
                pick(4, min.z, max.z),
            ));
        }

        #[rustfmt::skip]
        let quads = [
            [0, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4],
            [2, 6, 7, 3], [0, 4, 6, 2], [1, 3, 7, 5],
        ];
        for [a, b, c, d] in quads {
            faces.push([base + a, base + b, base + c]);
            faces.push([base + a, base + c, base + d]);
        }
    }

    /// A 20 × 20 × 2 mm slab on a 4 × 4 mm stem, tilted and lifted off the plate.
    fn tilted_t() -> (Mesh, HalfEdgeMesh, Bvh) {
        let (mut verts, mut faces) = (Vec::new(), Vec::new());
        let v = Vector3::new;
        add_box(&mut verts, &mut faces, v(-10.0, -10.0, 4.0), v(10.0, 10.0, 6.0));
        add_box(&mut verts, &mut faces, v(-2.0, -2.0, -6.0), v(2.0, 2.0, 4.5));

        let mut mesh = Mesh::new(verts, faces);
        mesh.set_rotation(v(0.3, 0.2, 0.0));
        mesh.set_position(v(0.0, 0.0, 15.0));

        let half_edge = HalfEdgeMesh::build_from_mesh(mesh.inner());
        let bvh = Bvh::build(mesh.inner(), Progress::new());
        (mesh, half_edge, bvh)
    }

    #[test]
    fn rotated_model_supports() {
        let (mesh, half_edge, bvh) = tilted_t();
        let bed = Vector3::repeat(Milimeters::new(100.0));
        let placement = AutoPlacement::new(PlacementConfig::default(), SupportPreset::default(), bed);
        let supports = placement.generate_supports(&mesh, &half_edge, &bvh, &Progress::new());
        assert!(!supports.is_empty());

        for [contact, tip, joint] in supports {
            assert!(!intersect_tip(&mesh, &bvh, contact, tip), "tip enters the model");

            let run = (joint.xy() - tip.xy()).magnitude();
            assert!(tip.z - joint.z >= run - 1e-3, "arm flatter than 45°");
            assert!(joint.z > 0.0);
        }
    }

    #[test]
    fn tip_through_wall_is_rejected() {
        let (mesh, _, bvh) = tilted_t();

        // Contact on the top of the slab with the tip pointing down through it.
        let top = mesh.transform(&Vector3::new(6.0, 6.0, 6.0));
        let down = mesh.transform_normal(&-Vector3::z()).normalize();
        assert!(route_support(&mesh, &bvh, 1.1, top, top + down * 3.0).is_none());
    }
}
