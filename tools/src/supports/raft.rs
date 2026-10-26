//! The raft: a plate under clusters of pillar feet.
//!
//! The outline is the union of every plate pillar's footprint, so separate
//! support clusters give separate raft islands. Each island is then meshed as
//! a Skate plate (a slab with a sloped anti-peel lip) or a cross-grid lattice.

use i_overlay::{
    core::{fill_rule::FillRule, overlay_rule::OverlayRule},
    float::{simplify::SimplifyShape, single::SingleFloatOverlay},
    mesh::float::{outline::offset::OutlineOffset, style::OutlineStyle},
};
use i_triangle::float::triangulatable::Triangulatable;
use nalgebra::{Vector2, Vector3};
use slicer::geometry::mesh::MeshBuilder;

use common::serde::{Deserializer, Serializer};

/// Segments per footprint circle.
const CIRCLE_SEGMENTS: usize = 32;
/// Outline vertices closer than this are merged.
const MIN_SEGMENT: f32 = 0.01;

/// One closed polygon ring; outer rings run counter-clockwise, holes clockwise.
pub type Contour = Vec<[f32; 2]>;
/// An outer ring followed by its holes.
pub type Island = Vec<Contour>;

/// The raft style.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RaftShape {
    None,
    /// A solid plate with a sloped, raised lip.
    Skate,
    /// A lattice of square cells.
    CrossGrid,
}

/// Raft settings. Lengths are millimetres, angles degrees.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RaftConfig {
    pub shape: RaftShape,
    /// Footprint radius around each pillar foot. Pillars on a raft lose their
    /// base, so the raft takes the base's footprint (the base radius).
    pub foot_radius: f32,
    /// Area ratio (1.1): each island is scaled by this about its centre.
    pub area_ratio: f32,
    /// Thickness (1.0).
    pub thickness: f32,
    /// Height (1.8): the crest of the Skate lip.
    pub height: f32,
    /// Angle (30°): the Skate wall's slope from the plate.
    pub angle: f32,
    /// Cross-grid cell side (2.0).
    pub grid_cell: f32,
    /// Cross-grid bar width (2.0).
    pub grid_width: f32,
}

/// A generated raft: its settings and outline, in model-local xy.
#[derive(Clone, Debug)]
pub struct Raft {
    pub config: RaftConfig,
    pub islands: Vec<Island>,
}

impl Raft {
    /// Builds the raft outline under the given pillar feet, or `None` when the
    /// shape is `None` or there are no feet.
    pub fn build(config: &RaftConfig, feet: &[Vector2<f32>]) -> Option<Self> {
        if config.shape == RaftShape::None || feet.is_empty() {
            return None;
        }

        let discs: Vec<Contour> = (feet.iter())
            .map(|xy| circle(*xy, config.foot_radius))
            .collect();
        let union = discs.simplify_shape(FillRule::NonZero);

        // Scale every island about its own centre, then re-union in case
        // grown islands now overlap.
        let scaled: Vec<Contour> = (union.iter())
            .flat_map(|island| {
                let centre = centroid(&island[0]);
                island.iter().map(move |contour| {
                    let scaled = contour.iter().map(|&[x, y]| {
                        let p = centre + (Vector2::new(x, y) - centre) * config.area_ratio;
                        [p.x, p.y]
                    });
                    dedupe(scaled.collect())
                })
            })
            .filter(|c| c.len() >= 3)
            .collect();
        let islands = scaled.simplify_shape(FillRule::NonZero);

        (!islands.is_empty()).then_some(Self {
            config: *config,
            islands,
        })
    }

    /// Whether `xy` lies inside the raft outline.
    pub fn contains(&self, xy: Vector2<f32>) -> bool {
        // Even-odd over every ring: holes sit inside their outer ring.
        let crossings = (self.islands.iter().flatten())
            .flat_map(|contour| {
                let n = contour.len();
                (0..n).map(move |i| (contour[i], contour[(i + 1) % n]))
            })
            .filter(|&([ax, ay], [bx, by])| {
                (ay > xy.y) != (by > xy.y) && xy.x < ax + (xy.y - ay) / (by - ay) * (bx - ax)
            })
            .count();
        crossings % 2 == 1
    }

    /// Height of the flat top pillars stand on.
    pub fn top(&self) -> f32 {
        self.config.thickness
    }

    /// Meshes the raft; `map` takes model-local points to output space.
    pub fn mesh(&self, builder: &mut MeshBuilder, map: impl Fn(Vector3<f32>) -> Vector3<f32>) {
        let cfg = &self.config;
        for island in &self.islands {
            match cfg.shape {
                RaftShape::None => {}
                RaftShape::Skate => skate(builder, &map, cfg, island),
                RaftShape::CrossGrid => {
                    let region = cross_grid(cfg, island);
                    slab(builder, &map, &region, 0.0, cfg.thickness);
                }
            }
        }
    }
}

/// A Skate island: the outer wall rises at `angle` from the outline on the
/// plate to the lip crest at `height`, drops to the slab top at `thickness`,
/// and the slab fills the inside.
fn skate(
    builder: &mut MeshBuilder,
    map: &impl Fn(Vector3<f32>) -> Vector3<f32>,
    cfg: &RaftConfig,
    island: &Island,
) {
    let inset = cfg.height / cfg.angle.to_radians().tan();
    let inner = island.outline(&OutlineStyle::new(-inset));
    if inner.is_empty() {
        // Too small for a lip: a plain slab.
        slab(
            builder,
            map,
            std::slice::from_ref(island),
            0.0,
            cfg.thickness,
        );
        return;
    }

    // Bottom face over the whole island.
    let outer = std::slice::from_ref(island);
    faces(builder, map, outer, |_| 0.0, false);

    // The slope: the ring between the outline and the lip crest, each vertex
    // lifted to the height of the ring it lies on.
    let ring = island.overlay(&inner, OverlayRule::Difference, FillRule::EvenOdd);
    let crest = |p: [f32; 2]| {
        let (o, i) = (distance(island, p), inner.iter().map(|s| distance(s, p)));
        if i.fold(f32::MAX, f32::min) < o {
            cfg.height
        } else {
            0.0
        }
    };
    faces(builder, map, &ring, crest, true);

    // Inside the crest: the lip's inner wall down to the slab top, and the top.
    for shape in &inner {
        for contour in shape {
            // Facing into the crest ring, away from the slope.
            wall(builder, map, contour, cfg.thickness, cfg.height, true);
        }
    }
    faces(builder, map, &inner, |_| cfg.thickness, true);
}

/// The cross-grid island: square holes of side `grid_cell` on a pitch of
/// `grid_cell + grid_width`, kept a bar width inside the outline.
fn cross_grid(cfg: &RaftConfig, island: &Island) -> Vec<Island> {
    let inner = island.outline(&OutlineStyle::new(-cfg.grid_width));
    let pitch = cfg.grid_cell + cfg.grid_width;
    let (min, max) = bounds(island);
    let start = (min / pitch).map(f32::floor) * pitch;

    let mut holes = Vec::new();
    let mut y = start.y;
    while y < max.y {
        let mut x = start.x;
        while x < max.x {
            let (x1, y1) = (x + cfg.grid_cell, y + cfg.grid_cell);
            holes.push(vec![[x, y], [x1, y], [x1, y1], [x, y1]]);
            x += pitch;
        }
        y += pitch;
    }

    let holes = inner.overlay(&holes, OverlayRule::Intersect, FillRule::NonZero);
    island.overlay(&holes, OverlayRule::Difference, FillRule::NonZero)
}

/// Extrudes the shapes between `z0` and `z1` with vertical walls.
fn slab(
    builder: &mut MeshBuilder,
    map: &impl Fn(Vector3<f32>) -> Vector3<f32>,
    shapes: &[Island],
    z0: f32,
    z1: f32,
) {
    faces(builder, map, shapes, |_| z0, false);
    faces(builder, map, shapes, |_| z1, true);
    for contour in shapes.iter().flatten() {
        wall(builder, map, contour, z0, z1, false);
    }
}

/// Triangulates the shapes, placing each vertex at height `z(p)`; faces point
/// up when `up`, else down.
fn faces(
    builder: &mut MeshBuilder,
    map: &impl Fn(Vector3<f32>) -> Vector3<f32>,
    shapes: &[Island],
    z: impl Fn([f32; 2]) -> f32,
    up: bool,
) {
    for shape in shapes {
        let tri = shape.triangulate().to_triangulation::<u32>();
        let base = builder.next_idx();
        for &p in &tri.points {
            builder.add_vertex(map(Vector3::new(p[0], p[1], z(p))));
        }
        for &t in tri.indices.as_chunks::<3>().0 {
            let [a, b, c] = t.map(|i| tri.points[i as usize]);
            let ccw = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) > 0.0;
            let [i, j, k] = t.map(|i| base + i);
            if ccw == up {
                builder.add_face([i, j, k]);
            } else {
                builder.add_face([i, k, j]);
            }
        }
    }
}

/// A vertical wall along a ring from `z0` to `z1`. Rings run with the
/// material on their left, so the wall faces right (out of the material),
/// or left when `inward`.
fn wall(
    builder: &mut MeshBuilder,
    map: &impl Fn(Vector3<f32>) -> Vector3<f32>,
    contour: &Contour,
    z0: f32,
    z1: f32,
    inward: bool,
) {
    let n = contour.len();
    for i in 0..n {
        let ([px, py], [qx, qy]) = (contour[i], contour[(i + 1) % n]);
        let p0 = builder.add_vertex(map(Vector3::new(px, py, z0)));
        let q0 = builder.add_vertex(map(Vector3::new(qx, qy, z0)));
        let q1 = builder.add_vertex(map(Vector3::new(qx, qy, z1)));
        let p1 = builder.add_vertex(map(Vector3::new(px, py, z1)));
        if inward {
            builder.add_face([p0, q1, q0]);
            builder.add_face([p0, p1, q1]);
        } else {
            builder.add_face([p0, q0, q1]);
            builder.add_face([p0, q1, p1]);
        }
    }
}

fn circle(centre: Vector2<f32>, radius: f32) -> Contour {
    (0..CIRCLE_SEGMENTS)
        .map(|i| {
            let angle = i as f32 / CIRCLE_SEGMENTS as f32 * std::f32::consts::TAU;
            [
                centre.x + radius * angle.cos(),
                centre.y + radius * angle.sin(),
            ]
        })
        .collect()
}

/// Area centroid of a ring.
fn centroid(contour: &Contour) -> Vector2<f32> {
    let n = contour.len();
    let (mut area, mut sum) = (0.0, Vector2::zeros());
    for i in 0..n {
        let (a, b) = (contour[i], contour[(i + 1) % n]);
        let cross = a[0] * b[1] - b[0] * a[1];
        area += cross;
        sum += Vector2::new(a[0] + b[0], a[1] + b[1]) * cross;
    }
    if area.abs() < f32::EPSILON {
        return Vector2::new(contour[0][0], contour[0][1]);
    }
    sum / (3.0 * area)
}

/// Drops vertices closer than [`MIN_SEGMENT`] to the previous kept one.
fn dedupe(contour: Contour) -> Contour {
    let mut out: Contour = Vec::with_capacity(contour.len());
    for p in contour {
        let near = |q: &[f32; 2]| (p[0] - q[0]).hypot(p[1] - q[1]) < MIN_SEGMENT;
        if !out.last().is_some_and(near) {
            out.push(p);
        }
    }
    if out.len() > 1
        && (out[0][0] - out[out.len() - 1][0]).hypot(out[0][1] - out[out.len() - 1][1])
            < MIN_SEGMENT
    {
        out.pop();
    }
    out
}

/// Distance from `p` to the nearest edge of any ring in the shape.
fn distance(shape: &Island, p: [f32; 2]) -> f32 {
    let p = Vector2::new(p[0], p[1]);
    let mut best = f32::MAX;
    for contour in shape {
        let n = contour.len();
        for i in 0..n {
            let a = Vector2::new(contour[i][0], contour[i][1]);
            let b = Vector2::new(contour[(i + 1) % n][0], contour[(i + 1) % n][1]);
            let ab = b - a;
            let t = ((p - a).dot(&ab) / ab.magnitude_squared().max(f32::EPSILON)).clamp(0.0, 1.0);
            best = best.min((a + ab * t - p).magnitude());
        }
    }
    best
}

fn bounds(island: &Island) -> (Vector2<f32>, Vector2<f32>) {
    let mut min = Vector2::repeat(f32::MAX);
    let mut max = Vector2::repeat(f32::MIN);
    for &[x, y] in island.iter().flatten() {
        min = min.inf(&Vector2::new(x, y));
        max = max.sup(&Vector2::new(x, y));
    }
    (min, max)
}

// --------------------------------------------------------------------------
// Serialization, used by `SupportSet`.
// --------------------------------------------------------------------------

impl RaftShape {
    const fn discriminant(self) -> u8 {
        match self {
            RaftShape::None => 0,
            RaftShape::Skate => 1,
            RaftShape::CrossGrid => 2,
        }
    }

    fn from_discriminant(x: u8) -> Self {
        match x {
            1 => RaftShape::Skate,
            2 => RaftShape::CrossGrid,
            _ => RaftShape::None,
        }
    }
}

impl Raft {
    pub fn serialize<T: Serializer>(&self, ser: &mut T) {
        let c = &self.config;
        ser.write_u8(c.shape.discriminant());
        for x in [
            c.foot_radius,
            c.area_ratio,
            c.thickness,
            c.height,
            c.angle,
            c.grid_cell,
            c.grid_width,
        ] {
            ser.write_f32_be(x);
        }
        ser.write_u32_be(self.islands.len() as u32);
        for island in &self.islands {
            ser.write_u32_be(island.len() as u32);
            for contour in island {
                ser.write_u32_be(contour.len() as u32);
                for &[x, y] in contour {
                    ser.write_f32_be(x);
                    ser.write_f32_be(y);
                }
            }
        }
    }

    pub fn deserialize<T: Deserializer>(des: &mut T) -> Self {
        let shape = RaftShape::from_discriminant(des.read_u8());
        let mut f = || des.read_f32_be();
        let config = RaftConfig {
            shape,
            foot_radius: f(),
            area_ratio: f(),
            thickness: f(),
            height: f(),
            angle: f(),
            grid_cell: f(),
            grid_width: f(),
        };
        let islands = (0..des.read_u32_be())
            .map(|_| {
                (0..des.read_u32_be())
                    .map(|_| {
                        (0..des.read_u32_be())
                            .map(|_| [des.read_f32_be(), des.read_f32_be()])
                            .collect()
                    })
                    .collect()
            })
            .collect();
        Self { config, islands }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SKATE: RaftConfig = RaftConfig {
        shape: RaftShape::Skate,
        foot_radius: 6.0,
        area_ratio: 1.1,
        thickness: 1.0,
        height: 1.8,
        angle: 30.0,
        grid_cell: 2.0,
        grid_width: 2.0,
    };

    fn volume(builder: MeshBuilder) -> f32 {
        let mesh = builder.build();
        (mesh.faces().iter())
            .map(|f| {
                let [a, b, c] = f.map(|i| mesh.vertices()[i as usize]);
                a.dot(&b.cross(&c)) / 6.0
            })
            .sum()
    }

    #[test]
    fn separate_clusters_give_separate_islands() {
        let feet = [
            Vector2::new(0.0, 0.0),
            Vector2::new(4.0, 0.0),
            Vector2::new(50.0, 0.0),
        ];
        let raft = Raft::build(&SKATE, &feet).unwrap();
        assert_eq!(raft.islands.len(), 2);
        assert!(raft.contains(Vector2::new(2.0, 0.0)));
        assert!(raft.contains(Vector2::new(50.0, 6.2)));
        assert!(!raft.contains(Vector2::new(25.0, 0.0)));
    }

    #[test]
    fn no_shape_no_raft() {
        let cfg = RaftConfig {
            shape: RaftShape::None,
            ..SKATE
        };
        assert!(Raft::build(&cfg, &[Vector2::zeros()]).is_none());
    }

    #[test]
    fn skate_is_a_closed_slab_with_a_lip() {
        let raft = Raft::build(&SKATE, &[Vector2::zeros()]).unwrap();
        let mut builder = MeshBuilder::new();
        raft.mesh(&mut builder, |v| v);
        let r = 6.0 * 1.1;
        let inset = 1.8 / 30f32.to_radians().tan();
        // Slab plus the sloped ring above it, give or take polygon facets.
        let slab = std::f32::consts::PI * (r - inset).powi(2) * 1.0;
        let v = volume(builder);
        assert!(v > slab, "volume {v} should exceed the inner slab {slab}");
        assert!(v < std::f32::consts::PI * r * r * 1.8, "volume {v}");
    }

    #[test]
    fn cross_grid_has_holes() {
        let cfg = RaftConfig {
            shape: RaftShape::CrossGrid,
            ..SKATE
        };
        let raft = Raft::build(&cfg, &[Vector2::zeros()]).unwrap();
        let mut builder = MeshBuilder::new();
        raft.mesh(&mut builder, |v| v);
        let full = std::f32::consts::PI * (6.6f32).powi(2);
        let v = volume(builder);
        assert!(v > 0.3 * full && v < 0.9 * full, "volume {v} of {full}");
    }
}
