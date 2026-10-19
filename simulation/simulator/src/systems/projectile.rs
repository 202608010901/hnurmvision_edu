use avian3d::prelude::*;
use bevy::prelude::*;
use core::f32::consts::PI;

use crate::components::{
    Controlled, DartLaunch, DartProjectile, DartSetting, GameLayer, Infantry, InfantryChassis,
    InfantryGimbal, InfantryLaunchOffset, ProjectileCooldown, ProjectileLifetime,
    ProjectileSetting,
};
use crate::config::SimulationConfig;
use crate::robomaster::prelude::{
    Activation, PowerRune, PowerRuneMechanism, PowerRuneRotation, Projectile, RuneIndex,
    RuneShotDiagnostic, SharedRuneMotion, Team,
};
use crate::statistic::ProjectileStatistics;

pub fn setup_projectile(
    mut commands: Commands,
    config: Res<SimulationConfig>,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.insert_resource(ProjectileSetting(
        meshes.add(Sphere::new(config.projectile.diameter / 2.0)),
        materials.add(StandardMaterial {
            base_color: Color::srgba(0.132866, 1.0, 0.132869, 0.85),
            emissive: LinearRgba::new(0.132866, 1.0, 0.132869, 0.85),
            emissive_exposure_weight: -1.0,
            alpha_mode: AlphaMode::Opaque,
            ..default()
        }),
    ));
    commands.insert_resource(DartSetting(
        asset_server.load(GltfAssetLabel::Scene(0).from_asset("DART.glb")),
    ));
}

pub fn tick_projectile_cooldown(
    time: Res<Time>,
    mut cooldown: ResMut<ProjectileCooldown>,
) {
    cooldown.tick(time.delta());
}

pub fn projectile_launch(
    flight: (Res<Time<Fixed>>, ResMut<crate::ballistics::ShotIds>, ResMut<crate::ballistics::FlightTrace>),
    mut board: ResMut<crate::scoreboard::Board>,
    mut cooldown: ResMut<ProjectileCooldown>,
    mut stats: ResMut<ProjectileStatistics>,
    config: Res<SimulationConfig>,
    gravity: Res<Gravity>,
    rune_motion: Res<SharedRuneMotion>,
    _asset_server: Res<AssetServer>,
    mut commands: Commands,
    setting: Res<ProjectileSetting>,
    infantry: Single<
        (&Transform, &LinearVelocity, &AngularVelocity),
        (With<Infantry>, With<Controlled>),
    >,
    gimbal: Single<
        (&GlobalTransform, &InfantryGimbal),
        (With<Controlled>, Without<InfantryChassis>),
    >,
    launch_offset: Single<&Transform, (With<Controlled>, With<InfantryLaunchOffset>)>,
    rune_targets: Query<(&GlobalTransform, &RuneIndex)>,
    runes: Query<(
        &GlobalTransform,
        &PowerRune,
        &PowerRuneMechanism,
        &PowerRuneRotation,
    )>,
) {
    let (fixed_time, mut shot_ids, mut hit_trace) = flight;
    if !cooldown.is_finished() {
        return;
    }
    cooldown.reset();

    stats.increase_launch();
    let direction = (gimbal.0.rotation() * launch_offset.rotation)
        .mul_vec3(Vec3::Y)
        .normalize_or_zero();
    if direction == Vec3::ZERO {
        return;
    }
    let origin = infantry.0.translation + (gimbal.0.rotation() * launch_offset.translation);
    let mut best_launch_geometry: Option<(usize, f32, f32, Vec3)> = None;
    for (target_transform, target) in &rune_targets {
        let Ok((rune_transform, rune, mechanism, rotation)) = runes.get(target.rune) else {
            continue;
        };
        if rune.team() != Team::Blue
            || mechanism.state().target_states().get(target.target).copied()
                != Some(Activation::Activating)
        {
            continue;
        }

        let mut flight_time = target_transform.translation().distance(origin)
            / config.projectile.speed.max(1e-3);
        let mut predicted_target = target_transform.translation();
        for _ in 0..3 {
            let rotation_delta = rune_motion.predicted_angle_delta(flight_time);
            predicted_target = rotation.predict_global_point(
                rune_transform,
                target_transform.translation(),
                rotation_delta,
            );
            flight_time = predicted_target.distance(origin) / config.projectile.speed.max(1e-3);
        }

        let required_velocity =
            (predicted_target - origin - 0.5 * gravity.0 * flight_time * flight_time)
                / flight_time.max(1e-3);
        let required_direction = required_velocity.normalize_or_zero();
        if required_direction == Vec3::ZERO {
            continue;
        }
        let angular_error = direction.angle_between(required_direction);
        if best_launch_geometry
            .is_none_or(|(_, best_error, _, _)| angular_error < best_error)
        {
            best_launch_geometry = Some((
                target.target,
                angular_error,
                flight_time,
                predicted_target - origin,
            ));
        }
    }

    let vel = infantry.1.0 + direction * config.projectile.speed;
    let shot_id = shot_ids.next();
    if hit_trace.enabled() { hit_trace.record(serde_json::json!({"event":"launch","shot_id":shot_id,"t":fixed_time.elapsed_secs_f64(),
        "origin":origin.to_array(),"velocity":vel.to_array(),"radius":config.projectile.diameter*0.5,
        "mode":format!("{:?}",config.training.hit_detection),"selected":board.0.selected,
        "gravity":gravity.0.to_array(),"lifetime":config.projectile.lifetime})); }
    let shot = if config.training.enabled { board.0.launch() } else { None };
    if config.training.enabled && config.training.hit_detection != crate::ballistics::HitDetectionMode::Legacy {
        let shadow = config.training.hit_detection == crate::ballistics::HitDetectionMode::Shadow;
        let bullet = commands.spawn((
            crate::ballistics::BallisticProjectile {
                id: shot_id, position: origin, velocity: vel, radius: config.projectile.diameter * 0.5,
                age: 0.0, born_at: fixed_time.elapsed_secs_f64(), lifetime: config.projectile.lifetime,
                shot: if shadow { None } else { shot }, shadow, resolved: false,
            },
            Transform::from_translation(origin),
        )).id();
        if !shadow { commands.entity(bullet).insert((Mesh3d(setting.0.clone()), MeshMaterial3d(setting.1.clone()))); }
        info!("BALLISTIC_LAUNCH shot_id={shot_id} t={} origin={origin:?} velocity={vel:?} radius={} shadow={shadow}",
            fixed_time.elapsed_secs_f64(), config.projectile.diameter * 0.5);
        if !shadow { return; }
    }
    let projectile = commands.spawn((
        RigidBody::Dynamic,
        SweptCcd::default(),
        Collider::sphere(config.projectile.diameter / 2.0),
        Mass(config.projectile.mass),
        Friction::new(config.projectile.friction),
        Restitution::new(0.3),
        LinearDamping(config.projectile.linear_damping),
        GameLayer::projectile_collision_layers(true),
        Mesh3d(setting.0.clone()),
        MeshMaterial3d(setting.1.clone()),
        LinearVelocity(vel),
        AngularVelocity(infantry.2.0),
        Transform::IDENTITY.with_translation(origin),
        ProjectileLifetime(Timer::from_seconds(
            config.projectile.lifetime,
            TimerMode::Once,
        )),
        (Projectile, RuneShotDiagnostic::default()),
    )).id();
    // Required by Avian before CollisionStart/CollisionEnd observers receive
    // projectile contacts. The scoreboard still credits armor contacts only.
    commands.entity(projectile).insert(CollisionEventsEnabled);
    commands.entity(projectile).insert(crate::ballistics::LegacyShotId(shot_id));
    if let Some(shot) = shot {
        commands.entity(projectile).insert(crate::scoreboard::ScoredShot(shot));
    }
    if let Some((target, angular_error, flight_time, target_delta)) = best_launch_geometry {
        info!(
            "RUNE_SHOT_LAUNCH projectile={:?} active_target={} angle_error={:.5} miss_estimate={:.4} flight={:.4} target_delta=({:.3},{:.3},{:.3})",
            projectile,
            target,
            angular_error,
            angular_error * target_delta.length(),
            flight_time,
            target_delta.x,
            target_delta.y,
            target_delta.z
        );
    } else {
        info!(
            "RUNE_SHOT_LAUNCH projectile={:?} active_target=None",
            projectile
        );
    }
}

pub fn projectile_aerodynamics(
    config: Res<SimulationConfig>,
    mut projectiles: Query<Forces, (With<Projectile>, Without<DartProjectile>)>,
) {
    let aero = &config.projectile.aerodynamics;
    if !aero.enabled {
        return;
    }

    let diameter = config.projectile.diameter;
    if diameter <= 0.0 {
        return;
    }
    let air_density = aero.air_density.max(0.0);
    let drag_coefficient = aero.drag_coefficient.max(0.0);
    if air_density == 0.0 || drag_coefficient == 0.0 {
        return;
    }

    let area = PI * (diameter * 0.5).powi(2);
    let wind = Vec3::new(aero.wind[0], aero.wind[1], aero.wind[2]);
    let k = 0.5 * air_density * drag_coefficient * area;

    for mut forces in projectiles.iter_mut() {
        let v_rel = forces.linear_velocity() - wind;
        let speed = v_rel.length();
        if speed <= 1e-3 {
            continue;
        }
        forces.apply_force(-k * speed * v_rel);
    }
}

pub fn dart_launch(
    mut commands: Commands,
    config: Res<SimulationConfig>,
    mut stats: ResMut<ProjectileStatistics>,
    setting: Res<DartSetting>,
    launchers: Query<&GlobalTransform, With<DartLaunch>>,
) {
    const DART_FORWARD: Vec3 = Vec3::Y;
    const DART_MODEL_FORWARD: Vec3 = Vec3::NEG_Y;
    const DART_SPEED_MPS: f32 = 17.0;
    const DART_MASS_KG: f32 = 0.25;
    const DART_COLLIDER_RADIUS_M: f32 = 0.001;
    const DART_COLLIDER_LENGTH_M: f32 = 0.001;
    const DART_SPAWN_OFFSET_M: f32 = 0.00;

    let Ok(launcher) = launchers.single() else {
        return;
    };

    let direction = launcher
        .rotation()
        .mul_vec3(DART_FORWARD)
        .normalize_or_zero();
    if direction == Vec3::ZERO {
        return;
    }

    stats.increase_launch();

    let transform =
        Transform::from_translation(launcher.translation() + direction * DART_SPAWN_OFFSET_M)
            .with_rotation(
                launcher.rotation() * Quat::from_rotation_arc(DART_MODEL_FORWARD, DART_FORWARD),
            );
    let voxel = |size| {
        ColliderConstructorHierarchy::new(ColliderConstructor::VoxelizedTrimeshFromMesh {
            voxel_size: size,
            fill_mode: FillMode::FloodFill {
                detect_cavities: true,
            },
        })
        .with_default_layers(GameLayer::projectile_collision_layers(true))
    };
    commands.spawn((
        RigidBody::Dynamic,
        voxel(0.005),
        Mass(DART_MASS_KG),
        Friction::new(config.projectile.friction),
        Restitution::new(0.55),
        LinearDamping(config.projectile.linear_damping),
        GameLayer::projectile_collision_layers(true),
        WorldAssetRoot(setting.0.clone()),
        transform,
        LinearVelocity(direction * DART_SPEED_MPS),
        ProjectileLifetime(Timer::from_seconds(
            config.projectile.lifetime,
            TimerMode::Once,
        )),
        Projectile,
        DartProjectile,
    ));
}

pub fn cleanup_projectiles(
    time: Res<Time>,
    mut commands: Commands,
    mut projectiles: Query<(Entity, &mut ProjectileLifetime, Option<&RuneShotDiagnostic>)>,
) {
    for (entity, mut lifetime, diagnostic) in &mut projectiles {
        lifetime.tick(time.delta());
        if lifetime.is_finished() {
            if let Some(diagnostic) = diagnostic {
                let effective = diagnostic
                    .rune_outcome
                    .is_some_and(|outcome| outcome.is_accurate())
                    && diagnostic.hit_team == Some(Team::Blue);
                info!(
                    "RUNE_SHOT_END projectile={:?} hit_team={:?} outcome={:?} effective={} min_active_distance={:.4} nearest_team={:?} nearest_target={} delta=({:.4},{:.4},{:.4})",
                    entity,
                    diagnostic.hit_team,
                    diagnostic.rune_outcome,
                    effective,
                    diagnostic.min_active_distance,
                    diagnostic.nearest_team,
                    diagnostic.nearest_target,
                    diagnostic.closest_delta.x,
                    diagnostic.closest_delta.y,
                    diagnostic.closest_delta.z
                );
            }
            commands.entity(entity).despawn();
        }
    }
}
