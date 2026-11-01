use std::{
    f32::consts::{FRAC_PI_2, FRAC_PI_4},
    ops::Neg,
};

use egui::{PointerButton, Response, Ui};
use nalgebra::{Matrix4, Rotation3, Unit, Vector2, Vector3};

use crate::core::config::render::Projection;

pub mod spacenav;

const EPSILON: f32 = 1e-5;
/// Radians of orbit per point of pointer movement.
const ORBIT_SPEED: f32 = 0.0092;
/// Zoom per point of scroll, as a share of the depth zoomed into: a wheel
/// notch (50 points) closes about 13 % of it.
const ZOOM_RATE: f32 = 0.0026;
/// Shift + scroll zooms this much finer.
const FINE_ZOOM: f32 = 0.25;
/// Scroll points in a wheel notch.
const NOTCH: f32 = 50.0;
/// Zoom steps are sized as if nothing were nearer than this, so they stay
/// usable right up against a surface.
const MIN_STEP_DEPTH: f32 = 5.0;
/// The pitch stops this short of straight up or down, where the view would
/// flip over.
const MAX_PITCH: f32 = FRAC_PI_2 - 0.0175;
/// Zooming into a surface rests the camera this far in front of it; one more
/// notch takes it through. Also the shortest distance to the target.
pub const MIN_DEPTH: f32 = 1.0;
pub const NEAR: f32 = 0.1;
pub const FAR: f32 = 10_000.0;

/// What pointer input moves the camera about, where known. Without a ray,
/// zooming closes in on the target; without a pivot, orbiting turns about it.
#[derive(Clone, Copy, Debug, Default)]
pub struct Pivots {
    /// The ray under the pointer (origin, unit direction) zooming follows.
    pub ray: Option<(Vector3<f32>, Vector3<f32>)>,
    /// How far along the ray the nearest surface is.
    pub hit: Option<f32>,
    /// Orbiting turns the view about this point.
    pub orbit: Option<Vector3<f32>>,
    /// Where the camera may go.
    pub limits: Option<Limits>,
}

/// Bounds that keep the camera near the scene: the target stays in the box,
/// the camera within `max_distance` of it.
#[derive(Clone, Copy, Debug)]
pub struct Limits {
    pub min: Vector3<f32>,
    pub max: Vector3<f32>,
    pub max_distance: f32,
}

#[derive(Clone, Debug)]
pub struct Camera {
    pub target: Vector3<f32>,
    pub angle: Vector2<f32>,
    pub distance: f32,
    pub fov: f32,
    /// Scroll toward a surface the camera rests at, gathered until a notch's
    /// worth takes it through.
    pub pushed: f32,
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
    /// pans, scrolling zooms (shift + scroll finer). Without a middle button,
    /// shift + right drag pans and alt + left drag orbits. The plain left
    /// button is left to the tools. Zoom and orbit go about the `pivots`.
    pub fn handle_movement(
        &mut self,
        response: &Response,
        ui: &Ui,
        projection: Projection,
        pivots: Pivots,
    ) -> bool {
        let modifiers = ui.input(|x| x.modifiers);
        let drag_delta = response.drag_delta() * if modifiers.command { 0.1 } else { 1.0 };

        let right = response.dragged_by(PointerButton::Secondary);
        let orbit = (right && !modifiers.shift)
            || (response.dragged_by(PointerButton::Primary) && modifiers.alt);
        let pan = response.dragged_by(PointerButton::Middle) || (right && modifiers.shift);

        let mut active = false;
        if orbit {
            let (yaw, pitch) = (-drag_delta.x * ORBIT_SPEED, drag_delta.y * ORBIT_SPEED);
            self.orbit(yaw, pitch, pivots.orbit);
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
            // Shift turns the wheel into a horizontal scroll in egui.
            let scroll = ui.input(|x| x.smooth_scroll_delta);
            let (amount, rate) = match modifiers.shift {
                true => (scroll.x + scroll.y, ZOOM_RATE * FINE_ZOOM),
                false => (scroll.y, ZOOM_RATE),
            };
            if amount != 0.0 {
                match projection {
                    Projection::Perspective => self.dolly(amount, rate, &pivots),
                    Projection::Orthographic => self.scale(amount, rate, &pivots),
                }
                active = true;
            }
        }

        if active && let Some(limits) = pivots.limits {
            self.target = self.target.sup(&limits.min).inf(&limits.max);
            self.distance = self.distance.min(limits.max_distance);
        }

        active
    }

    /// Moves the camera `amount` scroll points along the pointer ray (toward
    /// the target without one), by a share of the depth it looks into, so
    /// what is under the pointer stays there. It rests [`MIN_DEPTH`] in front
    /// of a surface; another notch steps it through. The target is then put
    /// on the view axis at the depth zoomed into.
    fn dolly(&mut self, amount: f32, rate: f32, pivots: &Pivots) {
        let eye = self.eye();
        let forward = self.forward();
        let (dir, hit) = match pivots.ray {
            Some((_, dir)) => (dir, pivots.hit),
            None => (forward, None),
        };
        // How far along the ray the depth zoomed into is.
        let along = dir.dot(&forward).max(0.1);
        let reach = hit.unwrap_or(self.distance / along);

        let mut travel = amount * rate * reach.max(MIN_STEP_DEPTH);
        match hit {
            Some(hit) if travel > 0.0 && hit - travel < MIN_DEPTH => {
                if hit > MIN_DEPTH + 0.01 {
                    travel = hit - MIN_DEPTH;
                    self.pushed = 0.0;
                } else {
                    self.pushed += amount;
                    travel = 0.0;
                    if self.pushed >= NOTCH {
                        travel = hit + MIN_DEPTH;
                        self.pushed = 0.0;
                    }
                }
            }
            _ => self.pushed = 0.0,
        }
        if pivots.ray.is_none() {
            travel = travel.min(self.distance - MIN_DEPTH);
        }

        let eye = eye + dir * travel;
        let depth = (dir * reach - dir * travel).dot(&forward);
        self.distance = depth.max(MIN_DEPTH);
        self.target = eye + forward * self.distance;
    }

    /// Orthographic zoom: scales the view about the point under the pointer
    /// (the target without one), which stays put on screen.
    fn scale(&mut self, amount: f32, rate: f32, pivots: &Pivots) {
        let mut factor = (-amount * rate).exp();
        if self.distance * factor < MIN_DEPTH {
            factor = (MIN_DEPTH / self.distance).min(1.0);
        }
        let pivot = match pivots.ray {
            Some((origin, dir)) => {
                origin + dir * pivots.hit.unwrap_or((self.target - origin).dot(&dir))
            }
            None => self.target,
        };
        self.target = pivot + (self.target - pivot) * factor;
        self.distance *= factor;
    }

    /// Puts the target on the view axis at the depth of `point`, without
    /// moving the view, so panning moves `point` with the pointer and zoom
    /// steps are sized to it. Only in perspective, where the distance is not
    /// the zoom.
    pub fn reseat(&mut self, projection: Projection, point: Vector3<f32>) {
        if projection != Projection::Perspective {
            return;
        }
        let eye = self.eye();
        let forward = self.forward();
        self.distance = (point - eye).dot(&forward).max(MIN_DEPTH);
        self.target = eye + forward * self.distance;
    }

    /// Turns the view by `yaw` (about z) and `pitch` (radians), the pitch
    /// stopping short of straight up or down. With a pivot, the target is
    /// carried around it, so the pivot stays put on screen: the pitch turns
    /// about the horizontal axis across the view, the yaw about z.
    pub fn orbit(&mut self, yaw: f32, pitch: f32, pivot: Option<Vector3<f32>>) {
        let y = self.angle.y;
        let pitch = (y + pitch).clamp(-MAX_PITCH.max(-y), MAX_PITCH.max(y)) - y;
        if let Some(pivot) = pivot {
            let across = Vector3::new(self.angle.x.sin(), -self.angle.x.cos(), 0.0);
            let turn = Rotation3::from_axis_angle(&Vector3::z_axis(), yaw)
                * Rotation3::from_axis_angle(&Unit::new_normalize(across), pitch);
            self.target = pivot + turn * (self.target - pivot);
        }
        self.angle.x += yaw;
        self.angle.y += pitch;
    }

    /// Where the camera is (in perspective).
    pub fn eye(&self) -> Vector3<f32> {
        self.target + self.position(self.distance)
    }

    /// The direction the camera looks.
    pub fn forward(&self) -> Vector3<f32> {
        -self.position(1.0)
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
            pushed: 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Vector2, Vector3};

    use super::{Camera, MAX_PITCH, MIN_DEPTH, NOTCH, Pivots, ZOOM_RATE};
    use crate::core::config::render::Projection;

    /// A camera 100 mm in front of the wall x = 0, looking at it along +x.
    fn facing_wall() -> Camera {
        Camera {
            target: Vector3::new(-50.0, 0.0, 10.0),
            angle: Vector2::new(std::f32::consts::PI, 0.0),
            distance: 50.0,
            ..Default::default()
        }
    }

    /// One wheel notch toward the wall, spread over frames as smooth
    /// scrolling delivers it, along `dir` from the eye.
    fn notch(camera: &mut Camera, dir: Vector3<f32>) {
        for _ in 0..5 {
            let eye = camera.eye();
            let hit = -eye.x / dir.x;
            let pivots = Pivots {
                ray: Some((eye, dir)),
                hit: Some(hit),
                ..Default::default()
            };
            camera.dolly(NOTCH / 5.0, ZOOM_RATE, &pivots);
        }
    }

    fn on_screen(camera: &Camera, p: Vector3<f32>) -> Vector2<f32> {
        let clip = camera.view_projection_matrix(Projection::Perspective, 1.5) * p.push(1.0);
        clip.xy() / clip.w
    }

    #[test]
    fn zoom_reaches_a_wall_and_steps_through() {
        let mut camera = facing_wall();
        assert!((camera.eye().x + 100.0).abs() < 1e-3);
        let mut notches = 0;
        while -camera.eye().x > MIN_DEPTH + 1e-3 {
            let before = -camera.eye().x;
            notch(&mut camera, Vector3::x());
            // Steps never dwindle: at least the floor's worth, or the rest.
            let step = before - -camera.eye().x;
            assert!(step > 0.6 || -camera.eye().x <= MIN_DEPTH + 1e-3, "{step}");
            notches += 1;
            assert!(notches < 40, "still {} mm away", -camera.eye().x);
        }
        assert!((camera.eye().x + MIN_DEPTH).abs() < 1e-3);
        // Resting at the wall, the target sits on the view axis in front.
        assert!(camera.distance >= MIN_DEPTH);

        // One more notch goes through.
        let mut through = camera.clone();
        notch(&mut through, Vector3::x());
        assert!(
            (through.eye().x - MIN_DEPTH).abs() < 1e-3,
            "{}",
            through.eye()
        );
    }

    #[test]
    fn zoom_keeps_what_is_under_the_pointer() {
        let mut camera = facing_wall();
        let dir = Vector3::new(1.0, 0.3, -0.2).normalize();
        let point = camera.eye() + dir * (100.0 / dir.x);
        let before = on_screen(&camera, point);
        for _ in 0..6 {
            notch(&mut camera, dir);
        }
        let after = on_screen(&camera, point);
        assert!((before - after).norm() < 1e-3, "{before} -> {after}");
        assert!(camera.eye().x > -50.0);
    }

    #[test]
    fn reseat_keeps_the_view() {
        let mut camera = facing_wall();
        let view = camera.view_projection_matrix(Projection::Perspective, 1.5);
        camera.reseat(Projection::Perspective, Vector3::new(0.0, 20.0, 0.0));
        let moved = camera.view_projection_matrix(Projection::Perspective, 1.5);
        assert!((view - moved).norm() < 1e-3);
        assert!((camera.distance - 100.0).abs() < 1e-3);
    }

    #[test]
    fn pitch_stops_short_of_the_pole() {
        let mut camera = Camera::default();
        camera.orbit(0.0, 3.0, Some(Vector3::new(5.0, 5.0, 5.0)));
        assert!((camera.angle.y - MAX_PITCH).abs() < 1e-5);
        camera.orbit(0.0, -10.0, None);
        assert!((camera.angle.y + MAX_PITCH).abs() < 1e-5);
    }

    #[test]
    fn orbit_keeps_the_pivot_in_place() {
        let mut camera = Camera {
            target: Vector3::new(10.0, -5.0, 20.0),
            angle: Vector2::new(0.7, 0.4),
            distance: 80.0,
            ..Default::default()
        };
        let pivot = Vector3::new(14.0, -2.0, 16.0);
        let before = on_screen(&camera, pivot);
        camera.orbit(0.3, -0.25, Some(pivot));
        let after = on_screen(&camera, pivot);
        assert!((before - after).norm() < 1e-4, "{before} -> {after}");
    }
}
