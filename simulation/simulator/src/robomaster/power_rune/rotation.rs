use crate::robomaster::power_rune::consts::ROTATION_BASELINE_SMALL;
use crate::robomaster::power_rune::motion::{
    A_MAX, A_MIN, OMEGA_MAX, OMEGA_MIN, angle_delta, angular_speed,
};
use bevy::math::Dir3;
use bevy::prelude::{Component, GlobalTransform, Quat, Resource, Transform, Vec3};
use rand::Rng;

struct VariableRotation {
    a: f32,
    omega: f32,
    t: f32,
}

impl VariableRotation {
    pub fn random(rng: &mut impl Rng) -> Self {
        let a = rng.gen_range(A_MIN..=A_MAX);
        let omega = rng.gen_range(OMEGA_MIN..=OMEGA_MAX);
        Self { a, omega, t: 0.0 }
    }

    pub fn speed(&self) -> f32 {
        angular_speed(self.t, self.a, self.omega)
    }

    pub fn angle_delta(&mut self, dt: f32) -> f32 {
        let dt = dt.max(0.0);
        let angle = angle_delta(self.t, dt, self.a, self.omega);
        self.t += dt;
        angle
    }
}

#[derive(Resource, Default)]
pub struct SharedRuneMotion {
    variable: Option<VariableRotation>,
}

impl SharedRuneMotion {
    pub fn variable_params(&self) -> Option<(f32, f32, f32)> {
        self.variable.as_ref().map(|v| (v.a, v.omega, v.t))
    }

    pub fn sync_large_activation(&mut self, activating: bool, rng: &mut impl Rng) {
        match (activating, self.variable.is_some()) {
            (true, false) => self.variable = Some(VariableRotation::random(rng)),
            (true, true) => {}
            (false, _) => self.variable = None,
        }
    }

    pub fn current_speed(&self) -> f32 {
        self.variable
            .as_ref()
            .map_or(ROTATION_BASELINE_SMALL, VariableRotation::speed)
    }

    pub fn predicted_angle_delta(&self, dt: f32) -> f32 {
        self.variable
            .as_ref()
            .map_or(ROTATION_BASELINE_SMALL * dt.max(0.0), |variable| {
                angle_delta(variable.t, dt, variable.a, variable.omega)
            })
    }

    pub fn angle_delta(&mut self, dt: f32) -> f32 {
        if let Some(variable) = &mut self.variable {
            return variable.angle_delta(dt);
        }
        ROTATION_BASELINE_SMALL * dt.max(0.0)
    }
}

#[derive(Component)]
pub struct PowerRuneRotation {
    direction: Dir3,
    clockwise: bool,
}

impl PowerRuneRotation {
    pub fn new(clockwise: bool) -> Self {
        Self {
            direction: Dir3::from_xyz(-1.0, 0.0, -1.0).unwrap(),
            clockwise,
        }
    }

    pub fn is_clockwise(&self) -> bool {
        self.clockwise
    }

    pub fn signed_angle(&self, angle: f32) -> f32 {
        if self.clockwise { angle } else { -angle }
    }

    pub fn rotate(&self, transform: &mut Transform, angle: f32) {
        transform.rotate_local_axis(self.direction, self.signed_angle(angle));
    }

    pub fn predict_global_point(
        &self,
        rune_transform: &GlobalTransform,
        point: Vec3,
        angle: f32,
    ) -> Vec3 {
        let local_point = rune_transform.affine().inverse().transform_point3(point);
        let (scale, rotation, translation) = rune_transform.to_scale_rotation_translation();
        let future = Transform {
            translation,
            rotation: rotation
                * Quat::from_axis_angle(*self.direction, self.signed_angle(angle)),
            scale,
        };
        future.transform_point(local_point)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn small_rune_rotation_is_baseline_speed() {
        let motion = SharedRuneMotion::default();

        assert_eq!(motion.current_speed(), ROTATION_BASELINE_SMALL);
        assert!(motion.variable_params().is_none());
    }

    #[test]
    fn large_rune_activation_uses_fresh_sine_params() {
        let mut rng = rand::thread_rng();
        let mut motion = SharedRuneMotion::default();

        motion.sync_large_activation(true, &mut rng);
        let (a, omega, t) = motion.variable_params().unwrap();
        assert!((0.780..=1.045).contains(&a));
        assert!((1.884..=2.0).contains(&omega));
        assert_eq!(t, 0.0);

        let expected_initial_speed = 2.090 - a;
        assert_eq!(motion.current_speed(), expected_initial_speed);
        motion.angle_delta(0.5);
        assert_eq!(motion.variable_params().unwrap().2, 0.5);

        motion.angle_delta(0.5);
        assert_eq!(motion.variable_params().unwrap().2, 1.0);

        motion.sync_large_activation(false, &mut rng);
        assert!(motion.variable_params().is_none());
    }

    #[test]
    fn opposite_faces_share_magnitude_and_negate_direction() {
        let clockwise = PowerRuneRotation::new(true);
        let counter_clockwise = PowerRuneRotation::new(false);

        assert_eq!(clockwise.signed_angle(0.25), 0.25);
        assert_eq!(counter_clockwise.signed_angle(0.25), -0.25);
    }

    #[test]
    fn shared_motion_preserves_active_variable_rotation() {
        let mut rng = rand::thread_rng();
        let mut motion = SharedRuneMotion::default();

        motion.sync_large_activation(true, &mut rng);
        let first_params = motion.variable_params().unwrap();

        motion.sync_large_activation(true, &mut rng);
        assert_eq!(motion.variable_params().unwrap(), first_params);

        motion.sync_large_activation(false, &mut rng);
        assert!(motion.variable_params().is_none());
    }

    #[test]
    fn large_rune_angle_is_independent_of_frame_partitioning() {
        let mut whole = VariableRotation {
            a: 0.9,
            omega: 1.95,
            t: 0.0,
        };
        let mut split = VariableRotation {
            a: 0.9,
            omega: 1.95,
            t: 0.0,
        };

        let whole_angle = whole.angle_delta(1.0);
        let split_angle = split.angle_delta(0.25)
            + split.angle_delta(0.25)
            + split.angle_delta(0.25)
            + split.angle_delta(0.25);

        assert!((whole_angle - split_angle).abs() < 1e-6);
        assert!((whole.t - 1.0).abs() < f32::EPSILON);
        assert!((split.t - 1.0).abs() < f32::EPSILON);
    }
}
