//! Tip construction (§2.2).
//!
//! A support tip leaves the contact point along the surface normal, but it is
//! never allowed to be flatter than 20° below horizontal: directions more than
//! `maxConnectAngle` (70°) from straight down are clamped back toward vertical.
//! This is distinct from the overhang-angle threshold used in point placement
//! (`PlacementConfig::max_angle`, 30°) — that decides *which* faces need a
//! point; this decides the *tip direction* once a point is chosen.

use nalgebra::Vector3;

/// Maximum angle of a tip direction from straight down, in degrees (§2.2).
/// `cos(70°) ≈ 0.342`. A tip is never flatter than 20° below horizontal.
pub const MAX_CONNECT_ANGLE_DEG: f32 = 70.0;
/// The cosine of the maximum connect angle, `cos(70°)`.
pub const MAX_CONNECT_ANGLE_COS: f32 = 0.342;

/// Clamps an outward tip direction `d` so it is at most `maxConnectAngle` from
/// straight down (§2.2 step 1). If `d` is already steep enough it is returned
/// unchanged; otherwise its z component is set to `-cos(maxConnectAngle)` and
/// its horizontal part is rescaled to keep it a unit vector.
///
/// `d` must be a unit vector. The returned direction is unit length.
pub fn clamp_tip_direction(d: Vector3<f32>) -> Vector3<f32> {
    // Outward normal. `dot(d, -z)` is just `-d.z`.
    if -d.z >= MAX_CONNECT_ANGLE_COS {
        // Already within maxConnectAngle of straight down.
        return d;
    }

    // Rotate toward down until exactly maxConnectAngle away: d.z = -cos(70°),
    // then rescale the horizontal part so |d| stays 1.
    let z = -MAX_CONNECT_ANGLE_COS;
    let horiz_scale = (1.0 - z * z).sqrt() / d.xy().magnitude().max(f32::EPSILON);
    Vector3::new(d.x * horiz_scale, d.y * horiz_scale, z)
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::*;

    #[test]
    fn steep_direction_is_unchanged() {
        let down = -Vector3::z();
        let out = clamp_tip_direction(down);
        assert!((out - down).magnitude() < 1e-6);
    }

    #[test]
    fn flat_direction_is_clamped_to_70_degrees() {
        // A purely horizontal direction (90° from down) must be clamped to 70°.
        let horizontal = Vector3::x();
        let out = clamp_tip_direction(horizontal);
        // z should be exactly -cos(70°).
        assert!((out.z - (-MAX_CONNECT_ANGLE_COS)).abs() < 1e-6);
        // And unit length.
        assert!((out.magnitude() - 1.0).abs() < 1e-6);
        // The horizontal part keeps its direction but is rescaled.
        assert!(out.y.abs() < 1e-6);
        assert!(out.x > 0.0);
    }

    #[test]
    fn exactly_at_boundary_is_kept() {
        // A direction exactly 70° from down has -d.z == cos(70°).
        let d = Vector3::new(MAX_CONNECT_ANGLE_COS, 0.0, -MAX_CONNECT_ANGLE_COS);
        let d = d.normalize();
        let out = clamp_tip_direction(d);
        // Boundary (>=) is kept unchanged.
        assert!((out - d).magnitude() < 1e-6);
    }
}
