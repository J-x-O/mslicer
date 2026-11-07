// A hollowed model's cavity, seen through the model in the model pass: lit,
// blended over the colour target only, so the model's own normals and ids
// stay for the passes after.
@group(0) @binding(0) var<uniform> ctx: Context;

struct Context {
    transform: mat4x4f,
    camera_direction: vec3f,
    color: vec4f,
}

struct VertexOutput {
    @builtin(position) position: vec4f,
    @location(1) world_position: vec3f,
}

struct FragmentOutput {
    @location(0) color: vec4f,
    @location(1) model: vec2u,
    @location(2) normal: vec4f,
}

@vertex
fn vert(@location(0) position: vec4f) -> VertexOutput {
    return VertexOutput(ctx.transform * position, position.xyz);
}

@fragment
fn frag(in: VertexOutput) -> FragmentOutput {
    let intensity = blinn_phong(screen_normal(in.world_position), ctx.camera_direction);
    return FragmentOutput(vec4(ctx.color.rgb * intensity, ctx.color.a), vec2u(0), vec4f(0));
}
