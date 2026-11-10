use anyhow::{Result, bail, ensure};
use common::{
    progress::Progress,
    serde::{Deserializer, Serializer},
};
use nalgebra::Vector3;

mod obj;
mod stl;
mod util;

#[derive(Debug, Default)]
pub struct Mesh {
    pub verts: Box<[Vector3<f32>]>,
    pub faces: Box<[[u32; 3]]>,
}

#[derive(Clone, Copy)]
pub enum Format {
    Stl,
    Obj,
}

impl Mesh {
    pub fn face_normal(&self, face: usize) -> Vector3<f32> {
        let f = self.faces[face];
        let edge1 = self.verts[f[2] as usize] - self.verts[f[1] as usize];
        let edge2 = self.verts[f[0] as usize] - self.verts[f[1] as usize];
        edge1.cross(&edge2).normalize()
    }

    pub fn vertex_normals(&self) -> Vec<Vector3<f32>> {
        let mut normals = vec![Vector3::zeros(); self.verts.len()];
        for (i, [a, b, c]) in self.faces.iter().enumerate() {
            let normal = self.face_normal(i);
            normals[*a as usize] += normal;
            normals[*b as usize] += normal;
            normals[*c as usize] += normal;
        }

        for normal in normals.iter_mut() {
            *normal = normal.normalize();
        }

        normals
    }
}

impl Format {
    pub const ALL: [Self; 2] = [Self::Stl, Self::Obj];

    pub fn from_extension(ext: &str) -> Option<Self> {
        let format = ext.to_ascii_lowercase();
        Some(match format.as_str() {
            "stl" => Self::Stl,
            "obj" => Self::Obj,
            _ => return None,
        })
    }

    pub fn extension(&self) -> &str {
        match self {
            Format::Stl => "stl",
            Format::Obj => "obj",
        }
    }

    pub fn name(&self) -> &str {
        match self {
            Format::Stl => "Stereolithography",
            Format::Obj => "Wavefront",
        }
    }
}

pub fn load_mesh<T: Deserializer + Send>(
    mut des: T,
    format: Format,
    progress: &Progress,
) -> Result<Mesh> {
    let mesh = match format {
        Format::Stl => stl::parse(&mut des, progress),
        Format::Obj => obj::parse(&mut des, progress),
    };

    progress.set_finished();
    let mesh = mesh?;
    ensure!(!des.truncated(), "The file is cut short or unreadable.");
    mesh.validate()?;
    Ok(mesh)
}

impl Mesh {
    /// Refuses a mesh the rest of mslicer can't work with: faces that point
    /// past the vertices, coordinates that aren't finite, no faces.
    pub fn validate(&self) -> Result<()> {
        ensure!(!self.faces.is_empty(), "The model has no triangles.");
        let count = self.verts.len();
        if let Some(face) = (self.faces.iter()).find(|f| f.iter().any(|&v| v as usize >= count)) {
            bail!("A triangle refers to a missing vertex: {face:?} of {count}.");
        }
        let finite = |v: &Vector3<f32>| v.iter().all(|c| c.is_finite());
        ensure!(
            self.verts.iter().all(finite),
            "The model has a vertex that isn't a number."
        );
        Ok(())
    }
}

pub fn save_mesh<T: Serializer>(ser: &mut T, format: Format, progress: &Progress, mesh: &Mesh) {
    match format {
        Format::Stl => stl::serialize(ser, progress, mesh),
        Format::Obj => obj::serialize(ser, progress, mesh),
    }
    progress.set_finished();
}

#[cfg(test)]
mod tests {
    use common::serde::SliceDeserializer;

    use super::*;

    fn obj(text: &str) -> Result<Mesh> {
        let des = SliceDeserializer::new(text.as_bytes());
        load_mesh(des, Format::Obj, &Progress::new())
    }

    #[test]
    fn a_damaged_obj_is_refused() {
        let good = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";
        assert_eq!(obj(good).unwrap().faces.len(), 1);
        // No final newline: the last line still counts.
        assert_eq!(obj(good.trim_end()).unwrap().faces.len(), 1);
        assert!(
            obj("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 4\n").is_err(),
            "missing vertex"
        );
        assert!(
            obj("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 0 1 2\n").is_err(),
            "index 0"
        );
        assert!(obj("v 0 0 nan\nv 1 0 0\nv 0 1 0\nf 1 2 3\n").is_err());
        assert!(obj("").is_err(), "no triangles");
    }

    #[test]
    fn a_cut_short_stl_is_refused() {
        let mesh = Mesh {
            verts: [Vector3::zeros(), Vector3::x(), Vector3::y()].into(),
            faces: [[0, 1, 2]].into(),
        };
        let mut ser = common::serde::DynamicSerializer::new();
        save_mesh(&mut ser, Format::Stl, &Progress::new(), &mesh);
        let data = ser.into_inner();
        let load =
            |data: &[u8]| load_mesh(SliceDeserializer::new(data), Format::Stl, &Progress::new());
        assert_eq!(load(&data).unwrap().faces.len(), 1);
        for len in 0..data.len() {
            assert!(load(&data[..len]).is_err(), "cut at {len}");
        }
    }
}
