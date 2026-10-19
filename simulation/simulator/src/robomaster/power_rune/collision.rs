use crate::robomaster::power_rune::common::RuneHitOutcome;
use crate::robomaster::power_rune::rune::{PowerRune, PowerRuneMechanism};
use crate::robomaster::prelude::Team;
use avian3d::prelude::{CollisionEnd, CollisionEventsEnabled};
use bevy::log::info;
use bevy::prelude::{
    ChildOf, Commands, Component, Entity, EntityEvent, GlobalTransform, On, Query, ResMut,
    Resource, Update, Vec3, With,
};
use std::collections::HashSet;

#[derive(Component)]
#[require(CollisionEventsEnabled)]
pub struct Projectile;

#[derive(Component, Debug)]
pub struct RuneShotDiagnostic {
    pub previous_position: Vec3,
    pub min_active_distance: f32,
    pub nearest_target: usize,
    pub nearest_team: Option<Team>,
    pub closest_delta: Vec3,
    pub hit_team: Option<Team>,
    pub rune_outcome: Option<RuneHitOutcome>,
}

impl Default for RuneShotDiagnostic {
    fn default() -> Self {
        Self {
            previous_position: Vec3::NAN,
            min_active_distance: f32::INFINITY,
            nearest_target: usize::MAX,
            nearest_team: None,
            closest_delta: Vec3::NAN,
            hit_team: None,
            rune_outcome: None,
        }
    }
}

#[derive(Resource, Default)]
struct ConsumedRuneProjectiles(HashSet<Entity>);

#[derive(Component, Debug, Copy, Clone)]
pub struct RuneIndex {
    pub target: usize,
    pub rune: Entity,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct HitResult {
    pub outcome: RuneHitOutcome,
}

impl HitResult {
    pub const fn accurate(self) -> bool {
        self.outcome.is_accurate()
    }
}

#[derive(EntityEvent)]
pub struct RuneActivated {
    #[event_target]
    pub rune: Entity,
}

#[derive(EntityEvent)]
pub struct RuneHit {
    #[event_target]
    pub rune: Entity,
    pub result: HitResult,
}

fn handle_rune_collision(
    event: On<CollisionEnd>,
    mut commands: Commands,
    mut consumed_projectiles: ResMut<ConsumedRuneProjectiles>,
    mut runes: Query<(&PowerRune, &mut PowerRuneMechanism)>,
    targets: Query<&RuneIndex>,
    projectiles: Query<Entity, With<Projectile>>,
    mut diagnostics: Query<&mut RuneShotDiagnostic>,
    child_of: Query<&ChildOf>,
) {
    let projectile_body1 = event.body1.and_then(|body| projectiles.get(body).ok());
    let projectile_body2 = event.body2.and_then(|body| projectiles.get(body).ok());

    let (projectile_entity, target_collider) = match (projectile_body1, projectile_body2) {
        (Some(projectile), _) => (projectile, event.collider2),
        (_, Some(projectile)) => (projectile, event.collider1),
        _ => return,
    };

    let Some(target) = find_rune_target(target_collider, &targets, &child_of) else {
        return;
    };

    let Ok((rune, mut mechanism)) = runes.get_mut(target.rune) else {
        return;
    };

    if !consumed_projectiles.0.insert(projectile_entity) {
        return;
    }

    commands
        .entity(projectile_entity)
        .remove::<CollisionEventsEnabled>();

    let mut rng = rand::thread_rng();
    let outcome = mechanism.state_mut().hit(target.target, &mut rng);
    if let Ok(mut diagnostic) = diagnostics.get_mut(projectile_entity) {
        diagnostic.hit_team = Some(rune.team());
        diagnostic.rune_outcome = Some(outcome);
    }
    info!(
        "RUNE_HIT projectile={:?} team={:?} mode={:?} target={} outcome={:?} progress={:?}",
        projectile_entity,
        rune.team(),
        rune.mode(),
        target.target,
        outcome,
        mechanism.state().large_progress()
    );

    commands.trigger(RuneHit {
        rune: target.rune,
        result: HitResult { outcome },
    });

    if outcome.activates_rune() {
        commands.trigger(RuneActivated { rune: target.rune });
    }
}

fn track_projectile_distance_to_active_rune(
    mut projectiles: Query<(&GlobalTransform, &mut RuneShotDiagnostic), With<Projectile>>,
    targets: Query<(&GlobalTransform, &RuneIndex)>,
    runes: Query<(&PowerRune, &PowerRuneMechanism)>,
) {
    for (projectile_transform, mut diagnostic) in &mut projectiles {
        let current = projectile_transform.translation();
        let previous = diagnostic.previous_position;
        diagnostic.previous_position = current;
        if previous.is_nan() {
            continue;
        }

        let segment = current - previous;
        let segment_length_squared = segment.length_squared();
        for (target_transform, target) in &targets {
            let Ok((rune, mechanism)) = runes.get(target.rune) else {
                continue;
            };
            let states = mechanism.state().target_states();
            if states.get(target.target).copied()
                != Some(crate::robomaster::visibility::Activation::Activating)
            {
                continue;
            }

            let target_position = target_transform.translation();
            let ratio = if segment_length_squared > f32::EPSILON {
                ((target_position - previous).dot(segment) / segment_length_squared).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let closest = previous + ratio * segment;
            let distance = closest.distance(target_position);
            if distance < diagnostic.min_active_distance {
                diagnostic.min_active_distance = distance;
                diagnostic.nearest_target = target.target;
                diagnostic.nearest_team = Some(rune.team());
                diagnostic.closest_delta = closest - target_position;
            }
        }
    }
}

fn cleanup_consumed_rune_projectiles(
    mut consumed_projectiles: ResMut<ConsumedRuneProjectiles>,
    projectiles: Query<(), With<Projectile>>,
) {
    consumed_projectiles
        .0
        .retain(|entity| projectiles.contains(*entity));
}

fn find_rune_target(
    entity: Entity,
    targets: &Query<&RuneIndex>,
    child_of: &Query<&ChildOf>,
) -> Option<RuneIndex> {
    if let Ok(target) = targets.get(entity) {
        return Some(*target);
    }

    child_of
        .iter_ancestors(entity)
        .find_map(|ancestor| targets.get(ancestor).ok().copied())
}

#[derive(Default)]
pub(super) struct PowerRuneCollisionPlugin;

impl bevy::app::Plugin for PowerRuneCollisionPlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.init_resource::<ConsumedRuneProjectiles>()
            .add_systems(
                Update,
                (
                    cleanup_consumed_rune_projectiles,
                    track_projectile_distance_to_active_rune,
                ),
            )
            .add_observer(handle_rune_collision);
    }
}
