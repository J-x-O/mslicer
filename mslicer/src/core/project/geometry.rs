//! The geometry section's evaluation: Mesh Repair and Flip Winding Order
//! applied, in stack order, to the model's source mesh (before Hollow) or to
//! its hollowed print mesh (after Hollow).
//!
//! Nothing here changes the source: removing or bypassing an operation
//! brings the model back to what it was.

use std::sync::{Arc, Mutex, PoisonError};

use common::progress::Progress;
use slicer::geometry::mesh::{Mesh, MeshInner};
use tools::repair::MeshRepair;

use crate::{
    core::project::stack::{Op, Operation, OperationId},
    engine::{CancelToken, Slot},
};

/// One geometry step, with what decides its result.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Step {
    Repair {
        id: OperationId,
        vertex_epsilon: f32,
    },
    FlipWinding,
}

impl Step {
    pub fn of(operation: &Operation) -> Option<Self> {
        match operation.op {
            Op::Repair(repair) => Some(Self::Repair {
                id: operation.id,
                vertex_epsilon: repair.vertex_epsilon,
            }),
            Op::FlipWinding => Some(Self::FlipWinding),
            _ => None,
        }
    }
}

/// What a repair found (and fixed).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RepairStats {
    pub unwelded_vertices: u64,
    pub degenerative_faces: u64,
    pub repeated_faces: u64,
    pub holes: u64,
    pub flipped_winding: u64,
}

impl RepairStats {
    pub fn any(&self) -> bool {
        self.unwelded_vertices
            + self.degenerative_faces
            + self.repeated_faces
            + self.holes
            + self.flipped_winding
            > 0
    }

    pub fn rows(&self) -> [(&'static str, u64); 5] {
        [
            ("Unwelded Vertices", self.unwelded_vertices),
            ("Degenerative Faces", self.degenerative_faces),
            ("Repeated Faces", self.repeated_faces),
            ("Holes", self.holes),
            ("Flipped Winding", self.flipped_winding),
        ]
    }
}

/// What the model's surface is evaluated from: the source (by identity) and
/// the steps before Hollow.
#[derive(Clone, Debug)]
pub struct GeometryKey {
    pub source: Arc<MeshInner>,
    pub steps: Vec<Step>,
}

impl PartialEq for GeometryKey {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.source, &other.source) && self.steps == other.steps
    }
}

/// The evaluated surface and what each repair found on the way.
pub struct Evaluated {
    pub mesh: Arc<MeshInner>,
    pub repairs: Vec<(OperationId, RepairStats)>,
}

/// Runs `steps` on `mesh` (model space). `None` if cancelled between steps.
pub fn evaluate(
    mesh: &Arc<MeshInner>,
    steps: &[Step],
    progress: &Progress,
    cancel: &CancelToken,
) -> Option<Evaluated> {
    let mut mesh = mesh.clone();
    let mut repairs = Vec::new();
    progress.set_total(steps.len() as u64);
    for step in steps {
        cancel.check().ok()?;
        match *step {
            Step::Repair { id, vertex_epsilon } => {
                let result = MeshRepair { vertex_epsilon }
                    .repair(&Mesh::from_inner(mesh.clone()), &Progress::new());
                let stats = RepairStats {
                    unwelded_vertices: result.unwelded_vertices,
                    degenerative_faces: result.degenerative_faces,
                    repeated_faces: result.repeated_faces,
                    holes: result.holes,
                    flipped_winding: result.flipped_winding,
                };
                // As the destructive repair did: a mesh without defects is
                // left as it is.
                if result.any_defects() {
                    mesh = result.mesh.inner().clone();
                }
                repairs.push((id, stats));
            }
            Step::FlipWinding => {
                let faces = mesh.faces.iter().map(|&[a, b, c]| [a, c, b]).collect();
                mesh = Arc::new(MeshInner {
                    vertices: mesh.vertices.clone(),
                    faces,
                });
            }
        }
        progress.add_complete(1);
    }
    progress.set_finished();
    Some(Evaluated { mesh, repairs })
}

/// The print mesh a cached after-Hollow result is for, the steps, and the
/// result.
type AfterHollow = (Arc<MeshInner>, Vec<Step>, Arc<Evaluated>);

/// The geometry nodes of a model: its surface (a background job) and the
/// steps after Hollow (applied to the print mesh when asked for, and
/// cached).
#[derive(Default)]
pub struct Geometry {
    pub surface: Slot<GeometryKey, Evaluated>,
    after: Mutex<Option<AfterHollow>>,
}

impl Clone for Geometry {
    fn clone(&self) -> Self {
        Self {
            surface: self.surface.clone(),
            after: Mutex::new(
                self.after
                    .lock()
                    .unwrap_or_else(PoisonError::into_inner)
                    .clone(),
            ),
        }
    }
}

impl Geometry {
    /// `print` with `steps` applied, cached for the last print mesh asked for.
    pub fn after_hollow(&self, print: &Arc<MeshInner>, steps: &[Step]) -> Arc<MeshInner> {
        let mut cache = self.after.lock().unwrap_or_else(PoisonError::into_inner);
        if let Some((input, cached_steps, output)) = cache.as_ref()
            && Arc::ptr_eq(input, print)
            && cached_steps == steps
        {
            return output.mesh.clone();
        }
        // Only a cancel makes `evaluate` fail, and this token is never
        // cancelled; the unhollowed mesh is the fallback all the same.
        let Some(out) = evaluate(print, steps, &Progress::new(), &CancelToken::new()) else {
            return print.clone();
        };
        let mesh = out.mesh.clone();
        *cache = Some((print.clone(), steps.to_vec(), Arc::new(out)));
        mesh
    }

    /// What a repair after Hollow found, the last time the print mesh was
    /// asked for.
    pub fn after_hollow_repair(&self, id: OperationId) -> Option<RepairStats> {
        let cache = self.after.lock().unwrap_or_else(PoisonError::into_inner);
        let (_, _, out) = cache.as_ref()?;
        (out.repairs.iter()).find_map(|(op, stats)| (*op == id).then_some(*stats))
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::*;

    fn triangle() -> Arc<MeshInner> {
        Mesh::new(
            vec![Vector3::zeros(), Vector3::x(), Vector3::y()],
            vec![[0, 1, 2]],
        )
        .inner()
        .clone()
    }

    #[test]
    fn steps_run_in_order_and_leave_the_source_alone() {
        let source = triangle();
        let flip = [Step::FlipWinding];
        let out = evaluate(&source, &flip, &Progress::new(), &CancelToken::new()).unwrap();
        assert_eq!(&*out.mesh.faces, &[[0, 2, 1]]);
        assert_eq!(&*source.faces, &[[0, 1, 2]]);
        let twice = [Step::FlipWinding, Step::FlipWinding];
        let out = evaluate(&source, &twice, &Progress::new(), &CancelToken::new()).unwrap();
        assert_eq!(out.mesh.faces, source.faces);
    }

    #[test]
    fn cancelled_between_steps() {
        let cancel = CancelToken::new();
        cancel.cancel();
        let steps = [Step::FlipWinding];
        assert!(evaluate(&triangle(), &steps, &Progress::new(), &cancel).is_none());
    }

    #[test]
    fn after_hollow_is_cached() {
        let geometry = Geometry::default();
        let print = triangle();
        let a = geometry.after_hollow(&print, &[Step::FlipWinding]);
        let b = geometry.after_hollow(&print, &[Step::FlipWinding]);
        assert!(Arc::ptr_eq(&a, &b));
    }
}
