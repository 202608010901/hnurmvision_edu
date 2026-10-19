use avian3d::prelude::*;
use bevy::prelude::*;
use std::f32::consts::{FRAC_PI_2, TAU};
use crate::components::{Controlled, Infantry};
use crate::config::{SimulationConfig, TrainingConfig};
use crate::robomaster::prelude::{Team, INFANTRY_THREE_CONFIG, INFANTRY_FOUR_CONFIG, SENTINEL_ROBOT_TWO_CONFIG};
use crate::scoreboard::Board;

/// Where an unselected target is parked: high enough that no arena geometry reaches it and
/// far enough off-axis that no shot aimed at the active lane can touch it.
const PARKED_HEIGHT: f32 = 50.0;

#[derive(Component)]
pub struct TrainingTarget { pub kind: usize, pub center: Vec3 }

impl TrainingTarget {
    /// Authoritative motion shared by rendering and continuous hit queries.
    /// Selection is sampled once per fixed tick; parking is a teleport, never a sweep.
    pub fn pose_at(&self, c: &TrainingConfig, selected: usize, t: f64) -> (Vec3, Quat) {
        let parked = c.solo_target && self.kind != selected;
        let mut p = if parked {
            Vec3::new(self.center.x, c.targets_y + PARKED_HEIGHT, self.center.z)
        } else if c.solo_target {
            Vec3::new(self.center.x, self.center.y, 0.0)
        } else { self.center };
        let mut angle = FRAC_PI_2;
        if !parked {
            if self.kind == 2 { angle += c.rotation_speed * t as f32; }
            if self.kind == 1 { p.z += c.translation_amplitude * (TAU * t as f32 / c.translation_period).sin(); }
        }
        (p, Quat::from_rotation_y(angle))
    }
}

pub fn manual_targets(config: Res<SimulationConfig>) -> bool { !config.training.enabled }

pub fn spawn(commands: &mut Commands, assets: &AssetServer, c: &TrainingConfig) {
    assert!(c.translation_period > 0.0 && c.translation_amplitude >= 0.0);
    assert!(c.lane_spacing - c.translation_amplitude > 1.0,
        "Target swept envelopes require >1m center separation");
    commands.spawn((
        Name::new("TRAINING_OBSERVER"),
        WorldAssetRoot(assets.load(GltfAssetLabel::Scene(0).from_asset("vehicle.glb"))),
        Transform::from_translation(Vec3::from_array(c.observer)).with_rotation(Quat::from_rotation_y(-FRAC_PI_2)),
        Infantry::new(Team::Red, INFANTRY_THREE_CONFIG), Controlled,
    ));
    let specs = [INFANTRY_THREE_CONFIG, INFANTRY_FOUR_CONFIG, SENTINEL_ROBOT_TWO_CONFIG];
    let names = ["STATIC_3", "TRANSLATING_4", "ROTATING_2"];
    for kind in 0..3 {
        let center = Vec3::new(c.targets_x, c.targets_y, (kind as f32 - 1.0)*c.lane_spacing);
        commands.spawn((
            Name::new(names[kind]), TrainingTarget { kind, center },
            WorldAssetRoot(assets.load(GltfAssetLabel::Scene(0).from_asset("vehicle.glb"))),
            Transform::from_translation(center).with_rotation(Quat::from_rotation_y(FRAC_PI_2)),
            Infantry::new(Team::Blue, specs[kind]),
        ));
    }
}

pub fn move_targets(
    time: Res<Time<Fixed>>, config: Res<SimulationConfig>,
    board: Option<Res<Board>>,
    mut targets: Query<(&TrainingTarget, &mut Position, &mut Rotation, &mut LinearVelocity, &mut AngularVelocity, &mut Visibility)>,
    mut last_log: Local<f64>,
) {
    if !config.training.enabled { return; }
    let t = time.elapsed_secs_f64();
    let c = &config.training;
    let selected = board.as_deref().map_or(0, |b| b.0.selected);
    let emit = t - *last_log >= 1.0;
    for (target, mut pos, mut rot, mut vel, mut omega, mut visibility) in &mut targets {
        let solo = c.solo_target && target.kind != selected;
        let (p, rotation) = target.pose_at(c, selected, t);
        let angle = rotation.to_euler(EulerRot::YXZ).0;
        // Kinematic targets prescribe poses; hits cannot knock them into neighbouring lanes.
        pos.0 = p; rot.0 = rotation; vel.0 = Vec3::ZERO; omega.0 = Vec3::ZERO;
        *visibility = if solo { Visibility::Hidden } else { Visibility::Inherited };
        if emit { info!("TRAINING_TARGET kind={} t={:.3} xyz=({:.3},{:.3},{:.3}) yaw={:.3}", target.kind,t,p.x,p.y,p.z,angle); }
    }
    if emit { *last_log = t; }
}
