// A hollowed model's cavity, seen through the model in the model pass: lit,
// blended over the colour target only, so the model's own normals and ids
// stay for the passes after.
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

struct FragmentOutput {
    @location(0) color: vec4f,
    @location(1) model: vec2u,
    @location(2) normal: vec4f,
}

@vertex
fn vert(@location(0) position: vec4f) -> VertexOutput {
    return VertexOutput(ctx.transform * position, position.xyz, (ctx.model * position).z);
}

@fragment
fn frag(in: VertexOutput) -> FragmentOutput {
    if in.height > ctx.cut { discard; }
    let intensity = blinn_phong(screen_normal(in.local_position), ctx.camera_direction);
    return FragmentOutput(vec4(ctx.color.rgb * intensity, ctx.color.a), vec2u(0), vec4f(0));
}
