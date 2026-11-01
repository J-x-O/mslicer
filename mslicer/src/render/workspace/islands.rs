use encase::{ShaderSize, ShaderType, UniformBuffer};
use nalgebra::{Matrix4, Vector3, Vector4};
use slicer::geometry::mesh::Mesh;
use wgpu::{
    BindGroup, BlendState, Buffer, BufferDescriptor, BufferUsages, ColorTargetState, ColorWrites,
    Device, FragmentState, IndexFormat, MultisampleState, PipelineLayoutDescriptor, PrimitiveState,
    RenderPass, RenderPipeline, RenderPipelineDescriptor, TextureFormat, VertexState,
};

use crate::{
    core::{App, islands::IslandSettings},
    include_shader,
    render::{
        Gcx, VERTEX_BUFFER_LAYOUT,
        consts::{
            BASE_BIND_GROUP_LAYOUT_DESCRIPTOR, BASE_UNIFORM_DESCRIPTOR, DEPTH_STENCIL_STATE,
            bind_group,
        },
        util::{ResizingBuffer, gpu_mesh},
    },
};

/// The island check's overlay: a see-through slab under every island still to
/// fix, in the warning colour, and the highlighted one in white.
pub struct IslandPipeline {
    render_pipeline: RenderPipeline,
    batches: [Batch; 2],
    /// What the meshes were built from.
    built: Option<(u64, IslandSettings)>,
}

struct Batch {
    bind_group: BindGroup,
    uniform_buffer: Buffer,
    vertex_buffer: ResizingBuffer,
    index_buffer: ResizingBuffer,
    index_count: u32,
}

#[derive(ShaderType)]
struct IslandUniforms {
    transform: Matrix4<f32>,
    camera_direction: Vector3<f32>,
    color: Vector4<f32>,
}

impl IslandPipeline {
    pub fn new(device: &Device, texture: TextureFormat) -> Self {
        // The support preview's shader: one colour, lit, with alpha.
        let shader =
            device.create_shader_module(include_shader!("workspace/support.wgsl", "common.wgsl"));

        let batch = || {
            let uniform_buffer = device.create_buffer(&BufferDescriptor {
                size: IslandUniforms::SHADER_SIZE.get(),
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
        };
        let (layout, plain) = batch();
        let (_, lit) = batch();

        let pipeline_layout = device.create_pipeline_layout(&PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[&layout],
            push_constant_ranges: &[],
        });

        let render_pipeline = device.create_render_pipeline(&RenderPipelineDescriptor {
            label: Some("Islands"),
            layout: Some(&pipeline_layout),
            vertex: VertexState {
                module: &shader,
                entry_point: None,
                buffers: &[VERTEX_BUFFER_LAYOUT],
                compilation_options: Default::default(),
            },
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
            primitive: PrimitiveState::default(),
            depth_stencil: Some(DEPTH_STENCIL_STATE),
            multisample: MultisampleState {
                count: 4,
                ..Default::default()
            },
            multiview: None,
            cache: None,
        });

        Self {
            render_pipeline,
            batches: [plain, lit],
            built: None,
        }
    }

    pub fn prepare(&mut self, gcx: &Gcx, app: &mut App) {
        let warning = Vector3::from(app.config.ui.warning_color);
        let colors = [warning.push(0.6), Vector3::repeat(1.0).push(0.9)];
        for (batch, color) in self.batches.iter().zip(colors) {
            let uniform = IslandUniforms {
                transform: app.view_projection(),
                camera_direction: app.camera.position(1.0),
                color,
            };
            let mut buffer = UniformBuffer::new(Vec::new());
            buffer.write(&uniform).unwrap();
            gcx.queue
                .write_buffer(&batch.uniform_buffer, 0, &buffer.into_inner());
        }

        let settings = app.state.island_settings;
        let Some(check) = &app.state.island_check else {
            self.built = None;
            self.batches.iter_mut().for_each(|b| b.index_count = 0);
            return;
        };
        let key = (check.revision(), settings);
        if self.built == Some(key) {
            return;
        }
        self.built = Some(key);

        let (plain, lit) = check.overlay(&settings);
        for (batch, mesh) in self.batches.iter_mut().zip([plain, lit]) {
            batch.write(gcx, &mesh);
        }
    }

    pub fn paint(&self, render_pass: &mut RenderPass) {
        render_pass.set_pipeline(&self.render_pipeline);
        for batch in self.batches.iter().filter(|b| b.index_count > 0) {
            render_pass.set_bind_group(0, &batch.bind_group, &[]);
            render_pass.set_vertex_buffer(0, batch.vertex_buffer.slice(..));
            render_pass.set_index_buffer(batch.index_buffer.slice(..), IndexFormat::Uint32);
            render_pass.draw_indexed(0..batch.index_count, 0, 0..1);
        }
    }
}

impl Batch {
    fn write(&mut self, gcx: &Gcx, mesh: &Mesh) {
        self.index_count = mesh.face_count() as u32 * 3;
        if self.index_count == 0 {
            return;
        }
        let (vertices, indices) = gpu_mesh(mesh);
        self.vertex_buffer.write_slice(gcx, vertices);
        self.index_buffer.write_slice(gcx, indices);
    }
}
