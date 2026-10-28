@group(0) @binding(0) var<uniform> ctx: Context;
@group(0) @binding(1) var texture: texture_2d<f32>;
@group(0) @binding(2) var normal: texture_2d<f32>;
@group(0) @binding(3) var occlusion: texture_2d<f32>;
@group(0) @binding(4) var texture_sampler: sampler;
@group(0) @binding(5) var filtering_sampler: sampler;

struct Context {
    camera_position: vec3f,
    flags: u32,
    outline: vec3f,
}

/// Outline width, in pixels.
const OUTLINE: i32 = 2;

@fragment
fn frag(in: VertexOutput) -> @location(0) vec4f {
    let uv = clip_to_uv(in.position);
    let color = textureSample(texture, texture_sampler, uv);
    let sample = textureSample(normal, texture_sampler, uv);
    let normal = sample.xyz;

    // Pixels just outside an outlined object, background included, draw
    // the outline.
    if sample.w < 0.5 && near_outlined(vec2i(in.frag_coord.xy)) {
        return vec4(ctx.outline, 1.0);
    }
    if all(normal == vec3f(0)) {  discard; }

    let intensity = blinn_phong(normal, normalize(ctx.camera_position));
    let occlusion = textureSample(occlusion, filtering_sampler, uv).r;

    let ao = select(1.0, occlusion, (ctx.flags & 1) != 0);
    return vec4(color.rgb * intensity * ao, color.w);
}

fn near_outlined(pixel: vec2i) -> bool {
    let size = vec2i(textureDimensions(normal));
    for (var y = -OUTLINE; y <= OUTLINE; y++) {
        for (var x = -OUTLINE; x <= OUTLINE; x++) {
            let at = clamp(pixel + vec2i(x, y), vec2i(0), size - 1);
            if textureLoad(normal, at, 0).w > 0.5 { return true; }
        }
    }
    return false;
}
