pub const A_MIN: f32 = 0.780;
pub const A_MAX: f32 = 1.045;
pub const OMEGA_MIN: f32 = 1.884;
pub const OMEGA_MAX: f32 = 2.000;
pub const SPEED_SUM: f32 = 2.090;

pub fn angular_speed(t: f32, a: f32, omega: f32) -> f32 {
    a * (omega * t).sin() + SPEED_SUM - a
}

pub fn angle_delta(t0: f32, dt: f32, a: f32, omega: f32) -> f32 {
    let dt = dt.max(0.0);
    let t1 = t0 + dt;
    a / omega * ((omega * t0).cos() - (omega * t1).cos()) + (SPEED_SUM - a) * dt
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parameter_bounds_match_the_2026_rulebook() {
        assert_eq!(A_MIN, 0.780);
        assert_eq!(A_MAX, 1.045);
        assert_eq!(OMEGA_MIN, 1.884);
        assert_eq!(OMEGA_MAX, 2.000);
        assert_eq!(SPEED_SUM, 2.090);
    }

    #[test]
    fn activation_starts_at_t_zero() {
        let a = 0.9;
        assert!((angular_speed(0.0, a, 1.95) - (SPEED_SUM - a)).abs() < f32::EPSILON);
    }

    #[test]
    fn analytic_angle_is_independent_of_frame_partitioning() {
        let whole = angle_delta(0.0, 1.0, 0.9, 1.95);
        let split = angle_delta(0.0, 0.25, 0.9, 1.95)
            + angle_delta(0.25, 0.25, 0.9, 1.95)
            + angle_delta(0.5, 0.25, 0.9, 1.95)
            + angle_delta(0.75, 0.25, 0.9, 1.95);
        assert!((whole - split).abs() < 1e-6);
    }

    #[test]
    fn negative_delta_does_not_reverse_time() {
        assert_eq!(angle_delta(1.0, -0.1, 0.9, 1.95), 0.0);
    }
}
