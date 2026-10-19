pub(super) const ACTIVATION_PRIMARY_TIMEOUT: f32 = 2.5;
pub(super) const LARGE_SECONDARY_TIMEOUT: f32 = 1.0;
pub(super) const LARGE_REFRESH_BLACKOUT: f32 = 0.2;
pub(super) const INACTIVE_WAIT: f32 = 1.0;
pub(super) const FAILURE_RECOVER: f32 = LARGE_REFRESH_BLACKOUT;
pub(super) const ACTIVATED_HOLD: f32 = 6.0;
pub(super) const ACTIVATION_GLOBAL_TIMEOUT: f32 = 30.0; // 视觉闭环仿真全局激活超时
pub(super) const ROTATION_BASELINE_SMALL: f32 = std::f32::consts::PI / 3.0; // 小机关固定角速度
