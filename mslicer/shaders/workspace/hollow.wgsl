// Holes and see-through cavities: one colour, lit, with alpha, cut at the
// view's height like the models.
@group(0) @binding(0) var<uniform> ctx: Context;

struct Context {
    transform: mat4x4f,
    // Into world space, for the cut.
    model: mat4x4f,
    camera_direction: vec3f,
    color: vec4f,
    // Nothing above this height (world z) is drawn.
    cut: f32,
}

struct VertexOutput {
    @builtin(position) position: vec4f,
    @location(1) local_position: vec3f,
    @location(2) height: f32,
}

@vertex
fn vert(@location(0) position: vec4f) -> VertexOutput {
    return VertexOutput(ctx.transform * position, position.xyz, (ctx.model * position).z);
}

@fragment
fn frag(in: VertexOutput) -> @location(0) vec4f {
    if in.height > ctx.cut { discard; }
    let intensity = blinn_phong(screen_normal(in.local_position), ctx.camera_direction);
    return vec4(ctx.color.rgb * intensity, ctx.color.a);
}
