use std::num::NonZero;

use common::units::Milimeter;
use encase::{DynamicUniformBuffer, ShaderSize, ShaderType};
use nalgebra::{Matrix4, Vector2, Vector3, Vector4};
use wgpu::{
    BindGroup, BindGroupDescriptor, BindGroupEntry, BindGroupLayout, BindGroupLayoutDescriptor,
    BindGroupLayoutEntry, BindingResource, BindingType, BlendState, BufferBinding,
    BufferBindingType, BufferUsages, Color, ColorTargetState, ColorWrites, CommandEncoder,
    CompareFunction, DepthBiasState, DepthStencilState, Device, FragmentState, IndexFormat, LoadOp,
    Operations, PipelineLayoutDescriptor, RenderPass, RenderPassColorAttachment,
    RenderPassDepthStencilAttachment, RenderPassDescriptor, RenderPipeline,
    RenderPipelineDescriptor, ShaderStages, StencilFaceState, StencilState, StoreOp, TextureFormat,
    VertexState,
};

use crate::{
    core::{App, config::render::RenderStyle, palette, project::model::ModelId, state::Tool},
    include_shader,
    render::{
        Gcx, VERTEX_BUFFER_LAYOUT,
        util::ResizingBuffer,
        workspace::model::{
            MultiStage,
            selected::{FaceState, Selected},
        },
    },
};

/// What isn't being worked on is grey, its supports a little lighter. A
/// model shows its own colour when selected, and when hovered as a preview of
/// selecting it.
const GREY_MODEL: Vector3<f32> = Vector3::new(0.55, 0.55, 0.56);
const GREY_SUPPORT: Vector3<f32> = Vector3::new(0.68, 0.68, 0.69);
const WHITE: Vector3<f32> = Vector3::new(1.0, 1.0, 1.0);

pub struct BasePass {
    pipeline: RenderPipeline,
    group_layout: BindGroupLayout,
}

pub struct BaseResources {
    uniform: ResizingBuffer,
    uniform_offsets: Vec<u32>,
    selected: ResizingBuffer,
    bind_group: Option<BindGroup>,
}

#[derive(ShaderType, Clone)]
struct Uniforms {
    transform: Matrix4<f32>,
    model_transform: Matrix4<f32>,
    build_volume: Vector3<f32>,
    model_color: Vector3<f32>,
    render_style: u32,
    overhang_angle: f32,
    id: u32,
    selected: Vector2<u32>, // start, length
    accent: Vector3<f32>,
    /// What hovered and selected faces are mixed towards (rgb) and how far
    /// (w).
    hover: Vector4<f32>,
    select: Vector4<f32>,
    warning: Vector3<f32>,
}

impl BasePass {
    pub fn create(device: &Device, texture: TextureFormat) -> Self {
        let shader = device
            .create_shader_module(include_shader!("workspace/model/base.wgsl", "common.wgsl"));

        let group_layout = device.create_bind_group_layout(&BindGroupLayoutDescriptor {
            label: None,
            entries: &[
                BindGroupLayoutEntry {
                    binding: 0,
                    visibility: ShaderStages::VERTEX | ShaderStages::FRAGMENT,
                    ty: BindingType::Buffer {
                        ty: BufferBindingType::Uniform,
                        has_dynamic_offset: true,
                        min_binding_size: Some(Uniforms::SHADER_SIZE),
                    },
                    count: None,
                },
                BindGroupLayoutEntry {
                    binding: 1,
                    visibility: ShaderStages::FRAGMENT,
                    ty: BindingType::Buffer {
                        ty: BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: NonZero::new(16),
                    },
                    count: None,
                },
            ],
        });
        let pipeline_layout = device.create_pipeline_layout(&PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[&group_layout],
            push_constant_ranges: &[],
        });

        let pipeline = device.create_render_pipeline(&RenderPipelineDescriptor {
            label: Some("Model"),
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
                targets: &[
                    // target
                    Some(ColorTargetState {
                        format: texture,
                        blend: Some(BlendState::ALPHA_BLENDING),
                        write_mask: ColorWrites::all(),
                    }),
                    // model
                    Some(ColorTargetState {
                        format: TextureFormat::Rg32Uint,
                        blend: None,
                        write_mask: ColorWrites::all(),
                    }),
                    // normals
                    Some(ColorTargetState {
                        format: TextureFormat::Rgba16Float,
                        blend: None,
                        write_mask: ColorWrites::all(),
                    }),
                ],
                compilation_options: Default::default(),
            }),
            primitive: Default::default(),
            depth_stencil: Some(DepthStencilState {
                format: TextureFormat::Depth32Float,
                depth_write_enabled: true,
                depth_compare: CompareFunction::Less,
                stencil: StencilState {
                    front: StencilFaceState::IGNORE,
                    back: StencilFaceState::IGNORE,
                    read_mask: 0,
                    write_mask: 0,
                },
                bias: DepthBiasState {
                    constant: 0,
                    slope_scale: 0.0,
                    clamp: 0.0,
                },
            }),
            multisample: Default::default(),
            multiview: None,
            cache: None,
        });

        Self {
            pipeline,
            group_layout,
        }
    }

    fn render_pass<'a>(
        &self,
        encoder: &'a mut CommandEncoder,
        multi: &MultiStage,
    ) -> RenderPass<'a> {
        encoder.begin_render_pass(&RenderPassDescriptor {
            label: Some("Model"),
            color_attachments: &[
                Some(RenderPassColorAttachment {
                    view: &multi.target_a,
                    resolve_target: None,
                    ops: Operations {
                        load: LoadOp::Clear(Color::TRANSPARENT),
                        store: StoreOp::Store,
                    },
                    depth_slice: None,
                }),
                Some(RenderPassColorAttachment {
                    view: &multi.model_target,
                    resolve_target: None,
                    ops: Operations {
                        load: LoadOp::Clear(Color {
                            r: u32::MAX as f64,
                            g: u32::MAX as f64,
                            b: 0.0,
                            a: 0.0,
                        }),
                        store: StoreOp::Store,
                    },
                    depth_slice: None,
                }),
                Some(RenderPassColorAttachment {
                    view: &multi.normal_target,
                    resolve_target: None,
                    ops: Operations {
                        load: LoadOp::Clear(Color::TRANSPARENT),
                        store: StoreOp::Store,
                    },
                    depth_slice: None,
                }),
            ],
            depth_stencil_attachment: Some(RenderPassDepthStencilAttachment {
                view: &multi.depth_target,
                depth_ops: Some(Operations {
                    load: LoadOp::Clear(1.0),
                    store: StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
        })
    }

    pub fn paint(&self, encoder: &mut CommandEncoder, app: &mut App, multi: &MultiStage) {
        let resources = &multi.base;
        let Some(bind_group) = &resources.bind_group else {
            return;
        };

        let mut render_pass = self.render_pass(encoder, multi);
        render_pass.set_pipeline(&self.pipeline);

        let indexes = (app.project.models.iter().enumerate())
            .filter(|(_, x)| !x.hidden)
            .map(|(idx, _)| idx);

        let mut i = 0;
        for idx in indexes {
            render_pass.set_bind_group(0, bind_group, &[resources.uniform_offsets[i]]);
            i += 1;

            let model = &app.project.models[idx];
            let buffers = model.try_get_buffers().unwrap();
            render_pass.set_vertex_buffer(0, buffers.vertex_buffer.slice(..));
            render_pass.set_index_buffer(buffers.index_buffer.slice(..), IndexFormat::Uint32);
            render_pass.draw_indexed(0..(model.mesh.face_count() as u32 * 3), 0, 0..1);

            if let Some((buffers, count)) = model.supports.try_get_buffers() {
                render_pass.set_bind_group(0, bind_group, &[resources.uniform_offsets[i]]);
                i += 1;

                render_pass.set_vertex_buffer(0, buffers.vertex_buffer.slice(..));
                render_pass.set_index_buffer(buffers.index_buffer.slice(..), IndexFormat::Uint32);
                render_pass.draw_indexed(0..count, 0, 0..1);
            }
        }
    }

    pub fn paint_preview(&self, encoder: &mut CommandEncoder, app: &mut App, multi: &MultiStage) {
        let (Some(bind_group), Some(Some(interactive))) = (
            &multi.base.bind_group,
            (app.slice_operation.as_ref()).map(|x| x.interactive_previews.as_ref()),
        ) else {
            return;
        };

        let mut render_pass = self.render_pass(encoder, multi);
        render_pass.set_pipeline(&self.pipeline);

        for (i, model) in interactive.models.iter().enumerate() {
            render_pass.set_bind_group(0, bind_group, &[multi.base.uniform_offsets[i]]);

            let buffers = &model.mesh;
            render_pass.set_vertex_buffer(0, buffers.vertex_buffer.slice(..));
            render_pass.set_index_buffer(buffers.index_buffer.slice(..), IndexFormat::Uint32);
            render_pass.draw_indexed(0..(model.faces as u32 * 3), 0, 0..1);
        }
    }
}

impl BaseResources {
    pub fn new(gcx: &Gcx) -> Self {
        let device = &gcx.device;
        let uniform = ResizingBuffer::new(device, BufferUsages::UNIFORM | BufferUsages::COPY_DST);
        let selected = ResizingBuffer::new(device, BufferUsages::STORAGE | BufferUsages::COPY_DST);

        Self {
            uniform,
            selected,
            uniform_offsets: Vec::new(),
            bind_group: None,
        }
    }

    fn recreate_bind_group(&mut self, gcx: &Gcx, layout: &BindGroupLayout) {
        let bind_group = gcx.device.create_bind_group(&BindGroupDescriptor {
            label: None,
            layout,
            entries: &[
                BindGroupEntry {
                    binding: 0,
                    resource: BindingResource::Buffer(BufferBinding {
                        buffer: &self.uniform,
                        offset: 0,
                        size: Some(Uniforms::SHADER_SIZE),
                    }),
                },
                BindGroupEntry {
                    binding: 1,
                    resource: BindingResource::Buffer(BufferBinding {
                        buffer: &self.selected,
                        offset: 0,
                        size: None,
                    }),
                },
            ],
        });

        self.bind_group.replace(bind_group);
    }

    fn write_uniforms(
        &mut self,
        gcx: &Gcx,
        app: &mut App,
        layout: &BindGroupLayout,
        mut callback: impl FnMut(&mut App, &Gcx, ModelId) -> Vec<(Uniforms, Vec<[u32; 4]>)>,
    ) {
        self.uniform_offsets.clear();

        let mut uniform_buffer = DynamicUniformBuffer::new(Vec::new());
        let mut selected_words = Vec::new();

        let ids = (app.project.models.iter_mut())
            .filter(|x| !x.hidden)
            .map(|x| x.id)
            .collect::<Vec<_>>();

        for model_id in ids {
            let model = app.project.model(model_id).unwrap();
            model.get_buffers(&gcx.device);
            model.supports.get_buffers(&gcx.device);

            for (mut uniform, selected) in callback(app, gcx, model_id) {
                uniform.selected = Vector2::new(selected_words.len(), selected.len()).cast();
                selected_words.extend_from_slice(&selected);

                let offset = uniform_buffer.write(&uniform);
                self.uniform_offsets.push(offset.unwrap() as u32);
            }
        }

        selected_words.resize(selected_words.len().max(1), [0; 4]);
        let uniform = self.uniform.write(gcx, &uniform_buffer.into_inner());
        let selected = self.selected.write_slice(gcx, &selected_words);

        if !self.uniform_offsets.is_empty() && (uniform || selected) {
            self.recreate_bind_group(gcx, layout);
        }
    }

    pub fn prepare(
        &mut self,
        gcx: &Gcx,
        app: &mut App,
        pass: &BasePass,
        view_projection: Matrix4<f32>,
    ) {
        let render_style = app.config.render.style as u32;
        let build_volume = (app.project.slice_config)
            .platform_size
            .map(|x| x.get::<Milimeter>());

        let (show_overhang, overhang_angle) = app.config.render.overhangs;
        let overhang_angle = show_overhang.then_some(overhang_angle);
        let overhang_angle = overhang_angle
            .map(|x| x.to_radians())
            .unwrap_or(f32::from_bits(u32::MAX));

        let accent = Vector3::from(app.config.ui.accent_color);
        let warning = app.config.ui.warning_color;
        self.write_uniforms(gcx, app, &pass.group_layout, |app, gcx, model_id| {
            let (tool, hover) = (app.state.tool, app.state.hovered_geometry);
            let moving = [app.state.support_drag, app.state.support_settling];
            let moving = moving.into_iter().flatten().filter(|x| x.model == model_id);
            let selected = app.state.selected.contains_model(model_id);
            let hovered = hover.is_some_and(|hit| hit.model == model_id);

            let model = app.project.model(model_id).unwrap();
            model.get_buffers(&gcx.device);

            let own = model.color.to_srgb().into();
            let model_color = if selected || hovered { own } else { GREY_MODEL };
            let model_transform = *model.mesh.transformation_matrix();
            let base = Uniforms {
                transform: view_projection * model_transform,
                model_transform,
                build_volume,
                model_color,
                render_style,
                overhang_angle,
                id: model.id.raw(),
                selected: Vector2::zeros(),
                accent,
                // Hovering a selected model just brightens it a little.
                hover: WHITE.push(0.15),
                select: accent.push(0.75),
                warning: warning.into(),
            };

            let mut states = Selected::default();
            selected.then(|| states.set_all(FaceState::Outlined));
            match tool {
                Tool::Select => {
                    (selected && hovered).then(|| states.set_all(FaceState::Hovered));
                }
                Tool::Orient => {
                    if let Some(hit) = hover.filter(|hit| !hit.support && hit.model == model_id) {
                        states.set_face(hit.face, FaceState::Selected);
                    }
                }
                Tool::Support => {}
            }
            let mut out = vec![(base.clone(), states.into_inner())];

            if model.supports.get_buffers(&gcx.device).is_some() {
                let hovered_support = (hover.filter(|hit| hit.support && hit.model == model_id))
                    .and_then(|hit| model.supports.support_for_face(hit.face));
                let (mesh, support_faces) = model.supports.mesh().as_ref().unwrap();
                let support_color = match tool {
                    // Supports move with their model, so they wear its colour.
                    Tool::Select | Tool::Orient => {
                        if selected || hovered {
                            own
                        } else {
                            GREY_SUPPORT
                        }
                    }
                    // While you work on them, they stand out against it.
                    // See palette::support_color.
                    Tool::Support if selected || hovered => {
                        palette::support_color(model.color, warning).into()
                    }
                    Tool::Support => GREY_SUPPORT,
                };
                let model_transform = mesh.transformation_matrix();
                let uniform = Uniforms {
                    transform: view_projection * model_transform,
                    model_transform: *model_transform,
                    model_color: support_color,
                    hover: match tool {
                        Tool::Support => WHITE.push(1.0),
                        _ => base.hover,
                    },
                    select: WHITE.push(1.0),
                    id: base.id | 1 << 31,
                    overhang_angle: f32::from_bits(0xFFFFFFFF),
                    ..base
                };

                let mut states = Selected::default();
                selected.then(|| states.set_all(FaceState::Outlined));
                let mut set = |support, state| {
                    if let Some(range) = support_faces.get(&support) {
                        states.set_range(*range, state);
                    }
                };
                match tool {
                    Tool::Select => {
                        (selected && hovered).then(|| states.set_all(FaceState::Hovered));
                    }
                    Tool::Support => {
                        // White under the pointer even on a model you aren't
                        // working on yet: clicking it starts working on it.
                        if let Some(support) = hovered_support {
                            set(support, FaceState::Hovered);
                        }
                        // Only a selected model's supports can be worked on.
                        if selected {
                            for support in app.state.selected_supports.for_model(model_id) {
                                set(support, FaceState::Selected);
                            }
                            for boxed in app.state.boxed_supports.iter() {
                                (boxed.model == model_id)
                                    .then(|| set(boxed.support, FaceState::Hovered));
                            }
                            for moving in moving {
                                set(moving.support, FaceState::Hidden);
                            }
                        }
                    }
                    Tool::Orient => {}
                }

                out.push((uniform, states.into_inner()));
            }

            out
        });
    }

    pub fn prepare_preview(
        &mut self,
        gcx: &Gcx,
        app: &mut App,
        pass: &BasePass,
        view: Matrix4<f32>,
    ) {
        let Some(Some(interactive)) =
            (app.slice_operation.as_ref()).map(|x| x.interactive_previews.as_ref())
        else {
            return;
        };

        self.uniform_offsets.clear();

        let mut uniform_buffer = DynamicUniformBuffer::new(Vec::new());
        let mut selected_words = Vec::new();

        for model in interactive.models.iter() {
            let uniform = Uniforms {
                transform: view * model.transform,
                model_transform: model.transform,
                build_volume: Vector3::repeat(f32::MAX),
                model_color: model.color.into(),
                render_style: RenderStyle::Rendered as u32,
                overhang_angle: f32::from_bits(0xFFFFFFFF),
                id: 0,
                selected: Vector2::zeros(),
                accent: Vector3::zeros(),
                hover: Vector4::zeros(),
                select: Vector4::zeros(),
                warning: Vector3::zeros(),
            };

            let selected = Selected::default().into_inner();
            selected_words.extend_from_slice(&selected);

            let offset = uniform_buffer.write(&uniform);
            self.uniform_offsets.push(offset.unwrap() as u32);
        }

        selected_words.resize(selected_words.len().max(1), [0; 4]);
        let uniform = self.uniform.write(gcx, &uniform_buffer.into_inner());
        let selected = self.selected.write_slice(gcx, &selected_words);

        if !self.uniform_offsets.is_empty() && (uniform || selected) {
            self.recreate_bind_group(gcx, &pass.group_layout);
        }
    }
}
