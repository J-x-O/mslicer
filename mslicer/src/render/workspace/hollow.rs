use std::sync::Arc;

use encase::{ShaderSize, ShaderType, UniformBuffer};
use nalgebra::{Matrix4, Vector3, Vector4};
use slicer::geometry::mesh::Mesh;
use tools::hollow::{HOLE_LEAD, Hole};
use wgpu::{
    BindGroup, BlendState, Buffer, BufferDescriptor, BufferUsages, ColorTargetState, ColorWrites,
    CompareFunction, DepthStencilState, Device, Face, FragmentState, IndexFormat, MultisampleState,
    PipelineLayoutDescriptor, PrimitiveState, RenderPass, RenderPipeline, RenderPipelineDescriptor,
    ShaderModule, TextureFormat, VertexState,
};

use crate::{
    core::{
        App,
        project::{
            hollow::ModelCavity,
            model::{Model, ModelId},
        },
    },
    include_shader,
    interface::panels::Tab,
    render::{
        Gcx, VERTEX_BUFFER_LAYOUT,
        consts::{
            BASE_BIND_GROUP_LAYOUT_DESCRIPTOR, BASE_UNIFORM_DESCRIPTOR, DEPTH_STENCIL_STATE,
            bind_group,
        },
        util::{ResizingBuffer, gpu_mesh},
    },
};

/// Drain holes, and the hole a click would drill. The model's surface is cut
/// open over each hole (`in_hole` in the model shader); this draws the
/// hole's wall from where it meets the surface down to its end in the
/// cavity. The hole picked out in the
/// Hollowing tab is in the accent colour, with its rim marked so it shows
/// from any side.
pub struct HollowPipeline {
    solid: RenderPipeline,
    walls: RenderPipeline,
    holes: Batch,
    highlight: Batch,
    /// Rims of holes that couldn't be drilled.
    failed: Batch,
    preview: Batch,
}

/// Hollowed models' cavities. Each is part of its model's surface (drawn by
/// the model pass from [`CavityPass::buffers`]), so it shows from inside the
/// model and through its holes. While the Hollowing tab is open, what the
/// model hides of its hollowing shows through it too: the cavities, the
/// holes' insides and its supports, faintly. That is drawn in the model pass
/// after the models and before the supports, so supports in front of a model
/// stay in front of it.
pub struct CavityPass {
    pipeline: RenderPipeline,
    cavities: Vec<CavityBatch>,
    /// The Hollowing tab is open.
    shown: bool,
    holes: Batch,
    /// Per hollowed model, its supports seen through it.
    supports: Vec<(ModelId, Batch)>,
    device: Device,
}

struct Batch {
    bind_group: BindGroup,
    uniform_buffer: Buffer,
    vertex_buffer: ResizingBuffer,
    index_buffer: ResizingBuffer,
    index_count: u32,
}

/// One model's cavity, uploaded once in the model's coordinates.
struct CavityBatch {
    model: ModelId,
    cavity: Arc<ModelCavity>,
    batch: Batch,
}

#[derive(ShaderType)]
struct HollowUniforms {
    transform: Matrix4<f32>,
    camera_direction: Vector3<f32>,
    color: Vector4<f32>,
}

const HOLE_COLOR: [f32; 4] = [0.24, 0.24, 0.26, 1.0];
const CAVITY_ALPHA: f32 = 0.3;
/// Supports seen through their model.
const GHOST_COLOR: [f32; 4] = [0.85, 0.85, 0.87, 0.45];
/// Sides of a hole as drawn.
const SEGMENTS: usize = 32;

impl HollowPipeline {
    pub fn new(device: &Device, texture: TextureFormat) -> Self {
        // The support preview's shader: one colour, lit, with alpha.
        let shader =
            device.create_shader_module(include_shader!("workspace/support.wgsl", "common.wgsl"));
        let (layout, _) = Batch::new(device);
        let pipeline_layout = device.create_pipeline_layout(&PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[&layout],
            push_constant_ranges: &[],
        });
        let pipeline = |label, primitive: PrimitiveState| {
            device.create_render_pipeline(&RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&pipeline_layout),
                vertex: vertex_state(&shader),
                fragment: Some(FragmentState {
                    module: &shader,
                    entry_point: None,
                    targets: &[Some(ColorTargetState {
                        format: texture,
                        blend: Some(BlendState::ALPHA_BLENDING),
                        write_mask: ColorWrites::all(),
                    })],
                    compilation_options: Default::default(),
                }),
                primitive,
                depth_stencil: Some(DEPTH_STENCIL_STATE),
                multisample: MultisampleState {
                    count: 4,
                    ..Default::default()
                },
                multiview: None,
                cache: None,
            })
        };
        // A hole's wall faces its axis; from outside, where it pokes out of
        // a surface curving away, it isn't drawn.
        let inside_only = PrimitiveState {
            cull_mode: Some(Face::Back),
            ..Default::default()
        };

        Self {
            solid: pipeline("Hole Rims", PrimitiveState::default()),
            walls: pipeline("Holes", inside_only),
            holes: Batch::new(device).1,
            highlight: Batch::new(device).1,
            failed: Batch::new(device).1,
            preview: Batch::new(device).1,
        }
    }

    pub fn prepare(&mut self, gcx: &Gcx, app: &mut App) {
        let view = app.view_projection();
        let camera = app.camera.position(1.0);
        let accent = Vector3::from(app.config.ui.accent_color);

        let warning = Vector3::from(app.config.ui.warning_color);
        let holes = gather(app);
        // With the Hollowing tab open, the holes' insides are seen through
        // the model with the cavity instead (see `CavityPass`).
        let plain = match app.panels.find_tab(&Tab::Hollow) {
            Some(_) => Shape::default(),
            None => holes.plain,
        };
        self.holes.write(gcx, &plain.mesh());
        (self.holes).uniform(gcx, view, camera, Vector4::from(HOLE_COLOR));
        self.highlight.write(gcx, &holes.picked.mesh());
        (self.highlight).uniform(gcx, view, camera, accent.push(1.0));
        self.failed.write(gcx, &holes.failed.mesh());
        (self.failed).uniform(gcx, view, camera, warning.push(1.0));

        let mut preview = Shape::default();
        if let Some(hole) = &app.state.hole_preview {
            ring(hole, &mut preview);
        }
        self.preview.write(gcx, &preview.mesh());
        self.preview.uniform(gcx, view, camera, accent.push(0.8));
    }

    pub fn paint(&self, render_pass: &mut RenderPass) {
        render_pass.set_pipeline(&self.walls);
        self.holes.draw(render_pass);
        render_pass.set_pipeline(&self.solid);
        self.highlight.draw(render_pass);
        self.failed.draw(render_pass);
        self.preview.draw(render_pass);
    }
}

impl CavityPass {
    /// For the model pass's targets: its colour target, model ids and
    /// normals (left alone), and its depth (ignored).
    pub fn new(device: &Device, texture: TextureFormat) -> Self {
        let shader = device.create_shader_module(include_shader!(
            "workspace/model/cavity.wgsl",
            "common.wgsl"
        ));
        let (layout, _) = Batch::new(device);
        let pipeline_layout = device.create_pipeline_layout(&PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[&layout],
            push_constant_ranges: &[],
        });
        let untouched = |format| {
            Some(ColorTargetState {
                format,
                blend: None,
                write_mask: ColorWrites::empty(),
            })
        };
        let pipeline = device.create_render_pipeline(&RenderPipelineDescriptor {
            label: Some("Cavities"),
            layout: Some(&pipeline_layout),
            vertex: vertex_state(&shader),
            fragment: Some(FragmentState {
                module: &shader,
                entry_point: None,
                targets: &[
                    Some(ColorTargetState {
                        format: texture,
                        blend: Some(BlendState::ALPHA_BLENDING),
                        write_mask: ColorWrites::all(),
                    }),
                    untouched(TextureFormat::Rg32Uint),
                    untouched(TextureFormat::Rgba16Float),
                ],
                compilation_options: Default::default(),
            }),
            primitive: PrimitiveState::default(),
            // Seen through the model in front of them.
            depth_stencil: Some(DepthStencilState {
                format: TextureFormat::Depth32Float,
                depth_write_enabled: false,
                depth_compare: CompareFunction::Always,
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: MultisampleState::default(),
            multiview: None,
            cache: None,
        });
        Self {
            pipeline,
            cavities: Vec::new(),
            shown: false,
            holes: Batch::new(device).1,
            supports: Vec::new(),
            device: device.clone(),
        }
    }

    /// A model's cavity facing into the cavity, as part of its surface: the
    /// vertex and index buffers and the index count.
    pub fn buffers(&self, model: ModelId) -> Option<(&Buffer, &Buffer, u32)> {
        let batch = &self.cavities.iter().find(|c| c.model == model)?.batch;
        (batch.index_count > 0).then_some((
            &batch.vertex_buffer,
            &batch.index_buffer,
            batch.index_count,
        ))
    }

    /// With the model pass's view (jittered like it, for TAA).
    pub fn prepare(&mut self, gcx: &Gcx, app: &App, view: Matrix4<f32>) {
        let camera = app.camera.position(1.0);
        let accent = Vector3::from(app.config.ui.accent_color);
        self.shown = app.panels.find_tab(&Tab::Hollow).is_some();
        let wanted: Vec<_> = (app.project.models.iter())
            .filter(|m| !m.hidden)
            .filter_map(|m| {
                // The last cavity, even while a new one is computed: it is in
                // the model's coordinates, so it follows a move or a scale.
                m.hollow.spec.shell?;
                let cavity = m.hollow.cavity.clone()?;
                Some((m.id, cavity, m))
            })
            .collect();
        self.cavities.retain(|c| {
            (wanted.iter()).any(|(id, cavity, ..)| *id == c.model && Arc::ptr_eq(cavity, &c.cavity))
        });
        self.supports
            .retain(|(id, _)| wanted.iter().any(|(m, ..)| m == id));
        for (id, cavity, model) in wanted {
            let index = match self.cavities.iter().position(|c| c.model == id) {
                Some(index) => index,
                None => {
                    // Facing into the cavity, as the material around it does.
                    let mesh = &cavity.mesh;
                    let faces = mesh.faces().iter().map(|f| [f[0], f[2], f[1]]).collect();
                    let mut batch = Batch::new(&self.device).1;
                    batch.write(gcx, &Mesh::new(mesh.vertices().to_vec(), faces));
                    self.cavities.push(CavityBatch {
                        model: id,
                        cavity: cavity.clone(),
                        batch,
                    });
                    self.cavities.len() - 1
                }
            };
            if !self.shown {
                continue;
            }
            // Drawn in the model's coordinates; light it from the camera as
            // seen from there.
            let matrix = model.mesh.transformation_matrix();
            let local = model.mesh.inv_transform_normal(&camera).normalize();
            let color = accent.push(CAVITY_ALPHA);
            (self.cavities[index].batch).uniform(gcx, view * matrix, local, color);

            if let Some((mesh, _)) = model.supports.mesh() {
                let index = match self.supports.iter().position(|(m, _)| *m == id) {
                    Some(index) => index,
                    None => {
                        self.supports.push((id, Batch::new(&self.device).1));
                        self.supports.len() - 1
                    }
                };
                let matrix = mesh.transformation_matrix();
                let local = mesh.inv_transform_normal(&camera).normalize();
                let color = Vector4::from(GHOST_COLOR);
                (self.supports[index].1).uniform(gcx, view * matrix, local, color);
            }
        }

        let holes = match self.shown {
            true => gather(app).plain,
            false => Shape::default(),
        };
        self.holes.write(gcx, &holes.mesh());
        (self.holes).uniform(gcx, view, camera, accent.push(CAVITY_ALPHA));
    }

    /// With the Hollowing tab open: the hollowing seen through the models.
    pub fn paint(&self, render_pass: &mut RenderPass, app: &App) {
        if !self.shown {
            return;
        }
        render_pass.set_pipeline(&self.pipeline);
        self.cavities.iter().for_each(|c| c.batch.draw(render_pass));
        self.holes.draw(render_pass);
        for (id, batch) in &self.supports {
            let model = app.project.models.iter().find(|m| m.id == *id);
            if let Some((buffers, count)) = model.and_then(|m| m.supports.try_get_buffers()) {
                batch.draw_with(
                    render_pass,
                    &buffers.vertex_buffer,
                    &buffers.index_buffer,
                    count,
                );
            }
        }
    }
}

/// The holes as drawn: the plain ones' walls, the picked one's wall and rim,
/// and the rims of those that couldn't be drilled.
struct Holes {
    plain: Shape,
    picked: Shape,
    failed: Shape,
}

/// Holes are few and small: rebuilt every frame, so they follow the models
/// without tracking every change.
fn gather(app: &App) -> Holes {
    let mut holes = Holes {
        plain: Shape::default(),
        picked: Shape::default(),
        failed: Shape::default(),
    };
    for model in app.project.models.iter().filter(|m| !m.hidden) {
        // Holes drain a cavity: only a hollowed model has them.
        let Some(shell) = model.hollow.spec.shell else {
            continue;
        };
        for (index, hole) in model.hollow.spec.holes.iter().enumerate() {
            let world = hole.world(&model.mesh, shell);
            let picked = app.state.hole_highlight == Some((model.id, index));
            if model.hollow.failed(&model.mesh, index) {
                ring(&world, &mut holes.failed);
                continue;
            }
            let start = surface(model, &world);
            match picked {
                true => {
                    wall(&world, &start, &mut holes.picked);
                    ring(&world, &mut holes.picked);
                }
                false => wall(&world, &start, &mut holes.plain),
            }
        }
    }
    holes
}

/// Triangles being gathered.
#[derive(Default)]
struct Shape {
    vertices: Vec<Vector3<f32>>,
    faces: Vec<[u32; 3]>,
}

impl Shape {
    fn mesh(self) -> Mesh {
        Mesh::new(self.vertices, self.faces)
    }
}

/// Unit vectors across a hole's axis.
fn across(d: &Vector3<f32>) -> (Vector3<f32>, Vector3<f32>) {
    let side = if d.z.abs() < 0.9 {
        Vector3::z()
    } else {
        Vector3::x()
    };
    let u = d.cross(&side).normalize();
    (u, d.cross(&u))
}

/// How far along a hole's axis its wall meets the model's surface, round
/// the hole (0 where that can't be found yet).
fn surface(model: &Model, hole: &Hole) -> [f32; SEGMENTS] {
    let Some(d) = hole.direction.try_normalize(f32::EPSILON) else {
        return [0.0; SEGMENTS];
    };
    let (u, v) = across(&d);
    let r = hole.diameter / 2.0;
    std::array::from_fn(|i| {
        let angle = i as f32 / SEGMENTS as f32 * std::f32::consts::TAU;
        let rim = hole.point + (u * angle.cos() + v * angle.sin()) * r;
        let from = rim - d * HOLE_LEAD;
        (model.bvh.as_ref())
            .and_then(|bvh| bvh.intersect_segment(&model.mesh, from, rim + d * hole.depth))
            .map_or(0.0, |hit| {
                (model.mesh.transform(&hit.position) - hole.point).dot(&d)
            })
    })
}

/// A hole's wall, facing its axis, from the surface (`start`, round it) to
/// its end in the cavity.
fn wall(hole: &Hole, start: &[f32; SEGMENTS], out: &mut Shape) {
    let Some(d) = hole.direction.try_normalize(f32::EPSILON) else {
        return;
    };
    let (u, v) = across(&d);
    let r = hole.diameter / 2.0;
    let base = out.vertices.len() as u32;
    let end = hole.point + d * hole.depth;
    for (i, start) in start.iter().enumerate() {
        let angle = i as f32 / SEGMENTS as f32 * std::f32::consts::TAU;
        let off = (u * angle.cos() + v * angle.sin()) * r;
        out.vertices
            .extend([hole.point + d * start.min(hole.depth) + off, end + off]);
    }
    let n = SEGMENTS as u32;
    for i in 0..n {
        let j = (i + 1) % n;
        let (ai, bi, aj, bj) = (
            base + 2 * i,
            base + 2 * i + 1,
            base + 2 * j,
            base + 2 * j + 1,
        );
        out.faces.extend([[ai, bj, aj], [ai, bi, bj]]);
    }
}

/// A hole's rim, as a short tube straddling the surface: where it is, seen
/// from anywhere.
fn ring(hole: &Hole, out: &mut Shape) {
    /// How far the tube reaches out of and into the surface (mm).
    const REACH: f32 = 0.3;
    let Some(d) = hole.direction.try_normalize(f32::EPSILON) else {
        return;
    };
    let (u, v) = across(&d);
    let r = hole.diameter / 2.0;
    let base = out.vertices.len() as u32;
    for i in 0..SEGMENTS {
        let angle = i as f32 / SEGMENTS as f32 * std::f32::consts::TAU;
        let at = hole.point + (u * angle.cos() + v * angle.sin()) * r;
        out.vertices.extend([at - d * REACH, at + d * REACH]);
    }
    let n = SEGMENTS as u32;
    for i in 0..n {
        let j = (i + 1) % n;
        let (a, b, c, e) = (
            base + 2 * i,
            base + 2 * i + 1,
            base + 2 * j,
            base + 2 * j + 1,
        );
        out.faces.extend([[a, c, e], [a, e, b]]);
    }
}

fn vertex_state(shader: &ShaderModule) -> VertexState<'_> {
    VertexState {
        module: shader,
        entry_point: None,
        buffers: &[VERTEX_BUFFER_LAYOUT],
        compilation_options: Default::default(),
    }
}

impl Batch {
    fn new(device: &Device) -> (wgpu::BindGroupLayout, Self) {
        let uniform_buffer = device.create_buffer(&BufferDescriptor {
            size: HollowUniforms::SHADER_SIZE.get(),
            ..BASE_UNIFORM_DESCRIPTOR
        });
        let (layout, bind_group) = bind_group(
            device,
            BASE_BIND_GROUP_LAYOUT_DESCRIPTOR,
            [uniform_buffer.as_entire_binding()],
        );
        let batch = Batch {
            bind_group,
            uniform_buffer,
            vertex_buffer: ResizingBuffer::new(device, BufferUsages::VERTEX),
            index_buffer: ResizingBuffer::new(device, BufferUsages::INDEX),
            index_count: 0,
        };
        (layout, batch)
    }

    fn uniform(
        &self,
        gcx: &Gcx,
        transform: Matrix4<f32>,
        camera_direction: Vector3<f32>,
        color: Vector4<f32>,
    ) {
        let uniform = HollowUniforms {
            transform,
            camera_direction,
            color,
        };
        let mut buffer = UniformBuffer::new(Vec::new());
        buffer.write(&uniform).unwrap();
        gcx.queue
            .write_buffer(&self.uniform_buffer, 0, &buffer.into_inner());
    }

    fn write(&mut self, gcx: &Gcx, mesh: &Mesh) {
        self.index_count = mesh.face_count() as u32 * 3;
        if self.index_count == 0 {
            return;
        }
        let (vertices, indices) = gpu_mesh(mesh);
        self.vertex_buffer.write_slice(gcx, vertices);
        self.index_buffer.write_slice(gcx, indices);
    }

    /// Draws another mesh's buffers with this batch's uniforms.
    fn draw_with(
        &self,
        render_pass: &mut RenderPass,
        vertices: &Buffer,
        indices: &Buffer,
        count: u32,
    ) {
        render_pass.set_bind_group(0, &self.bind_group, &[]);
        render_pass.set_vertex_buffer(0, vertices.slice(..));
        render_pass.set_index_buffer(indices.slice(..), IndexFormat::Uint32);
        render_pass.draw_indexed(0..count, 0, 0..1);
    }

    fn draw(&self, render_pass: &mut RenderPass) {
        if self.index_count == 0 {
            return;
        }
        render_pass.set_bind_group(0, &self.bind_group, &[]);
        render_pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
        render_pass.set_index_buffer(self.index_buffer.slice(..), IndexFormat::Uint32);
        render_pass.draw_indexed(0..self.index_count, 0, 0..1);
    }
}
