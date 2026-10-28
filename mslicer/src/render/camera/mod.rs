use std::{
    f32::consts::{FRAC_PI_2, FRAC_PI_4},
    ops::Neg,
};

use egui::{PointerButton, Response, Ui};
use nalgebra::{Matrix4, Vector2, Vector3};

use crate::core::config::render::Projection;

pub mod spacenav;

const EPSILON: f32 = 1e-5;
/// Radians of orbit per point of pointer movement.
const ORBIT_SPEED: f32 = 0.0092;
pub const NEAR: f32 = 0.1;
pub const FAR: f32 = 10_000.0;

#[derive(Clone, Debug)]
pub struct Camera {
    pub target: Vector3<f32>,
    pub angle: Vector2<f32>,
    pub distance: f32,
    pub fov: f32,
}

impl Camera {
    pub fn view_projection_matrix(&self, projection: Projection, aspect: f32) -> Matrix4<f32> {
        match projection {
            Projection::Perspective => {
                Matrix4::new_perspective(aspect, self.fov, NEAR, FAR)
                    * Matrix4::look_at_rh(
                        &(self.position(self.distance) + self.target).into(),
                        &self.target.into(),
                        &self.up(),
                    )
            }
            Projection::Orthographic => {
                let height = self.distance * (self.fov / 2.0).sin();
                let width = aspect * height;

                Matrix4::new_orthographic(-width, width, -height, height, -FAR, FAR)
                    * Matrix4::look_at_rh(
                        &(self.position(FAR / 2.0) + self.target).into(),
                        &self.target.into(),
                        &self.up(),
                    )
            }
        }
    }

    // returns ray pos (camera pos) and ray dir
    pub fn hovered_ray(
        &self,
        projection: Projection,
        aspect: f32,
        uv: Vector2<f32>,
    ) -> (Vector3<f32>, Vector3<f32>) {
        let camera_pos = self.position(self.distance) + self.target;
        let pos = 2.0 * Vector2::new(uv.x, 1.0 - uv.y) - Vector2::repeat(1.0);

        let forward = (self.target - camera_pos).normalize();
        let right = forward.cross(&self.up()).normalize();
        let up = right.cross(&forward).normalize();

        match projection {
            Projection::Perspective => {
                let fov_scale = (self.fov * 0.5).tan();
                let uv = pos.component_mul(&Vector2::new(aspect, 1.0)) * fov_scale;
                let dir = (forward + right * uv.x + up * uv.y).normalize();

                (camera_pos, dir)
            }
            Projection::Orthographic => {
                let height = self.distance * (self.fov / 2.0).sin();
                let width = aspect * height;

                let origin = camera_pos + right * pos.x * width + up * pos.y * height;
                (origin, forward)
            }
        }
    }

    /// Moves the camera from pointer input: right drag orbits, middle drag
    /// pans, scrolling zooms. Without a middle button, shift + right drag pans
    /// and alt + left drag orbits. The plain left button is left to the tools.
    pub fn handle_movement(
        &mut self,
        response: &Response,
        ui: &Ui,
        projection: Projection,
    ) -> bool {
        let modifiers = ui.input(|x| x.modifiers);
        let drag_delta = response.drag_delta() * if modifiers.command { 0.1 } else { 1.0 };

        let right = response.dragged_by(PointerButton::Secondary);
        let orbit = (right && !modifiers.shift)
            || (response.dragged_by(PointerButton::Primary) && modifiers.alt);
        let pan = response.dragged_by(PointerButton::Middle) || (right && modifiers.shift);

        let mut active = false;
        if orbit {
            self.angle.x -= drag_delta.x * ORBIT_SPEED;
            self.angle.y += drag_delta.y * ORBIT_SPEED;
            active |= true;
        }

        if pan {
            // Move the target as far as the view moves under the pointer at
            // the target's depth, so the grabbed point stays under it at any
            // zoom.
            let half_height = match projection {
                Projection::Perspective => self.distance * (self.fov / 2.0).tan(),
                Projection::Orthographic => self.distance * (self.fov / 2.0).sin(),
            };
            let per_point = 2.0 * half_height / response.rect.height().max(1.0);

            let facing = self.position(1.0).neg();
            let right = facing.cross(&self.up()).normalize();
            let up = right.cross(&facing).normalize();
            self.target -= (right * drag_delta.x - up * drag_delta.y) * per_point;
            active |= true;
        }

        if response.hovered() {
            let scroll = ui.input(|x| x.smooth_scroll_delta);
            self.distance = (self.distance - scroll.y * 0.1).max(EPSILON);
            active |= scroll.y != 0.0;
        }

        active
    }

    /// Points the camera at `bounds` from far enough to see all of it.
    pub fn frame(&mut self, min: Vector3<f32>, max: Vector3<f32>) {
        self.target = (min + max) / 2.0;
        let radius = (max - min).magnitude() / 2.0;
        self.distance = (radius / (self.fov / 2.0).sin()).max(EPSILON);
    }

    pub fn position(&self, distance: f32) -> Vector3<f32> {
        Vector3::new(
            self.angle.x.cos() * self.angle.y.cos(),
            self.angle.x.sin() * self.angle.y.cos(),
            self.angle.y.sin(),
        ) * distance
    }

    pub fn up(&self) -> Vector3<f32> {
        Vector3::z() * self.angle.y.cos().signum()
    }

    /// Updates the view distance so that the sphere centered at the camera
    /// target with radius r is fully visible.
    pub fn visible_sphere(&mut self, aspect: f32, r: f32) {
        // Because the orthographic projection is also based on the fov, the
        // same eq can be used for both:

        let base_size = aspect.min(1.0) * (self.fov / 2.0).sin();
        self.distance = r / base_size;
    }
}

impl Projection {
    pub const ALL: [Projection; 2] = [Projection::Perspective, Projection::Orthographic];

    pub fn name(&self) -> &'static str {
        match self {
            Projection::Perspective => "Perspective",
            Projection::Orthographic => "Orthographic",
        }
    }
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            target: Vector3::zeros(),
            angle: Vector2::new(-FRAC_PI_2, FRAC_PI_4),
            distance: 150.0,
            fov: FRAC_PI_2,
        }
    }
}
