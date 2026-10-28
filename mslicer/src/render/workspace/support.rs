use encase::{ShaderSize, ShaderType, UniformBuffer};
use nalgebra::{Matrix4, Vector3, Vector4};
use wgpu::{
    BindGroup, BlendState, Buffer, BufferDescriptor, BufferUsages, ColorTargetState, ColorWrites,
    Device, FragmentState, IndexFormat, MultisampleState, PipelineLayoutDescriptor, PrimitiveState,
    RenderPass, RenderPipeline, RenderPipelineDescriptor, TextureFormat, VertexState,
};

use crate::{
    core::{App, palette},
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

pub struct SupportPipeline {
    render_pipeline: RenderPipeline,
    bind_group: BindGroup,

    vertex_buffer: ResizingBuffer,
    index_buffer: ResizingBuffer,
    uniform_buffer: Buffer,
    index_count: u32,
}

#[derive(ShaderType)]
struct SupportUniforms {
    transform: Matrix4<f32>,
    camera_direction: Vector3<f32>,
    color: Vector4<f32>,
}

/// The preview in its model's support colour (see
/// [`palette::support_color`]). A support
/// being dragged is drawn solid, so it seems carried; a new one is a
/// see-through proposal.
fn preview_color(app: &App) -> Vector4<f32> {
    let moving = app.state.support_drag.or(app.state.support_settling);
    let model = (moving.map(|x| x.model))
        .or(app.state.hovered_geometry.map(|hit| hit.model))
        .and_then(|id| app.project.models.iter().find(|m| m.id == id));
    let warning = app.config.ui.warning_color;
    let color = model.map_or(Vector3::repeat(1.0), |m| {
        palette::support_color(m.color, warning).into()
    });
    color.push(if moving.is_some() { 1.0 } else { 0.5 })
}

impl SupportPipeline {
    pub fn new(device: &Device, texture: TextureFormat) -> Self {
        let shader =
            device.create_shader_module(include_shader!("workspace/support.wgsl", "common.wgsl"));

        let uniform_buffer = device.create_buffer(&BufferDescriptor {
            size: SupportUniforms::SHADER_SIZE.get(),
            ..BASE_UNIFORM_DESCRIPTOR
        });

        let (bind_group_layout, bind_group) = bind_group(
            device,
            BASE_BIND_GROUP_LAYOUT_DESCRIPTOR,
            [uniform_buffer.as_entire_binding()],
        );

        let pipeline_layout = device.create_pipeline_layout(&PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[&bind_group_layout],
            push_constant_ranges: &[],
        });

        let render_pipeline = device.create_render_pipeline(&RenderPipelineDescriptor {
            label: Some("Supports"),
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
            bind_group,

            vertex_buffer: ResizingBuffer::new(device, BufferUsages::VERTEX),
            index_buffer: ResizingBuffer::new(device, BufferUsages::INDEX),
            uniform_buffer,

            index_count: 0,
        }
    }
}

impl SupportPipeline {
    pub fn prepare(&mut self, gcx: &Gcx, app: &mut App) {
        let uniform = SupportUniforms {
            transform: app.view_projection(),
            camera_direction: app.camera.position(1.0),
            color: preview_color(app),
        };

        let mut buffer = UniformBuffer::new(Vec::new());
        buffer.write(&uniform).unwrap();
        gcx.queue
            .write_buffer(&self.uniform_buffer, 0, &buffer.into_inner());

        let Some(mesh) = &app.state.support_preview else {
            self.index_count = 0;
            return;
        };

        let (vertices, indices) = gpu_mesh(mesh);
        self.vertex_buffer
            .write_slice(gcx, bytemuck::cast_slice::<_, u8>(vertices));
        self.index_buffer.write_slice(gcx, indices);
        self.index_count = mesh.face_count() as u32 * 3;
    }

    pub fn paint(&self, render_pass: &mut RenderPass, _app: &mut App) {
        render_pass.set_pipeline(&self.render_pipeline);
        render_pass.set_bind_group(0, &self.bind_group, &[]);

        if self.index_count > 0 {
            render_pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
            render_pass.set_index_buffer(self.index_buffer.slice(..), IndexFormat::Uint32);
            render_pass.draw_indexed(0..self.index_count, 0, 0..1);
        }
    }
}
