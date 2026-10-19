use crate::robomaster::power_rune::common::{RuneMode, RuneTransition};
use crate::robomaster::power_rune::motion::SPEED_SUM;
use crate::robomaster::power_rune::rotation::{PowerRuneRotation, SharedRuneMotion};
use crate::robomaster::power_rune::state::MechanismState;
use crate::robomaster::power_rune::visual::PowerRuneVisuals;
use crate::robomaster::prelude::Team;
use crate::robomaster::visibility::StatefulAppearance;
use bevy::app::Update;
use bevy::log::info;
use bevy::prelude::{Component, IntoScheduleConfigs, Query, Res, ResMut, Time, Transform};

#[derive(Component, Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct PowerRune {
    team: Team,
    mode: RuneMode,
}

#[derive(Component, Debug, Clone, PartialEq)]
pub struct PowerRuneMechanism {
    state: MechanismState,
}

impl PowerRune {
    pub fn new(team: Team, mode: RuneMode) -> Self {
        Self { team, mode }
    }

    pub fn team(&self) -> Team {
        self.team
    }

    pub fn mode(&self) -> RuneMode {
        self.mode
    }
}

impl PowerRuneMechanism {
    pub fn new(mode: RuneMode) -> Self {
        Self::with_auto_activate_delay(mode, 1.0)
    }

    pub fn with_auto_activate_delay(mode: RuneMode, delay_secs: f32) -> Self {
        Self {
            state: MechanismState::inactive_with_delay(mode, delay_secs),
        }
    }

    pub fn state(&self) -> &MechanismState {
        &self.state
    }

    pub fn state_mut(&mut self) -> &mut MechanismState {
        &mut self.state
    }
}

impl Default for PowerRuneMechanism {
    fn default() -> Self {
        Self::new(RuneMode::Small)
    }
}

fn rune_activation_tick(time: Res<Time>, mut runes: Query<(&PowerRune, &mut PowerRuneMechanism)>) {
    let delta_secs = time.delta_secs();
    let mut rng = rand::thread_rng();

    for (rune, mut mechanism) in &mut runes {
        let transition = mechanism.state.tick(delta_secs, &mut rng);
        if transition != RuneTransition::None {
            info!(
                "RUNE_STATE team={:?} mode={:?} transition={:?} progress={:?}",
                rune.team,
                rune.mode,
                transition,
                mechanism.state.large_progress()
            );
        }
    }
}

fn apply_power_rune_visuals(
    mut runes: Query<(&PowerRune, &PowerRuneMechanism, &mut PowerRuneVisuals)>,
    mut appearance: StatefulAppearance,
) {
    for (rune, mechanism, mut visuals) in &mut runes {
        visuals.apply(rune.mode, mechanism.state(), &mut appearance);
    }
}

fn rune_rotation_system(
    time: Res<Time>,
    mut motion: ResMut<SharedRuneMotion>,
    mut runes: Query<(
        &PowerRune,
        &PowerRuneMechanism,
        &PowerRuneRotation,
        &mut Transform,
    )>,
) {
    let dt = time.delta_secs();
    // 大符正弦运动参数属于整个激活会话。0.2 s 失败恢复只是组内重试，
    // 不能在此期间重新随机 a/omega，否则视觉端已跟踪的相位会立即失效。
    let large_session_active = runes.iter().any(|(rune, mechanism, _, _)| {
        rune.mode == RuneMode::Large
            && !matches!(mechanism.state(), MechanismState::Inactive { .. })
    });
    let was_variable = motion.variable_params().is_some();
    motion.sync_large_activation(large_session_active, &mut rand::thread_rng());
    if large_session_active
        && !was_variable
        && let Some((a, omega, _)) = motion.variable_params()
    {
        info!(
            "Shared large-rune motion started: a={:.3}, omega={:.3}, b={:.3}",
            a,
            omega,
            SPEED_SUM - a
        );
    }
    let angle = motion.angle_delta(dt);

    for (_, _, rotation, mut transform) in &mut runes {
        rotation.rotate(&mut transform, angle);
    }
}

#[derive(Default)]
pub(super) struct PowerRuneUpdatePlugin;

impl bevy::app::Plugin for PowerRuneUpdatePlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.init_resource::<SharedRuneMotion>().add_systems(
            Update,
            (
                rune_activation_tick,
                apply_power_rune_visuals,
                rune_rotation_system,
            )
                .chain(),
        );
    }
}
