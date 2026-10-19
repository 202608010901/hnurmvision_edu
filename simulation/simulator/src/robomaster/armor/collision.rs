use avian3d::prelude::CollisionStart;
use bevy::prelude::{ChildOf, Commands, Entity, On, Plugin, Query, ResMut, With};

use super::construct::Armor;
use crate::robomaster::power_rune::prelude::Projectile;
use crate::statistic::ProjectileStatistics;

fn handle_armor_collision(
    event: On<CollisionStart>,
    mut commands: Commands,
    mut stats: ResMut<ProjectileStatistics>,
    projectiles: Query<Entity, With<Projectile>>,
    armors: Query<(), With<Armor>>,
    child_of: Query<&ChildOf>,
) {
    let projectile_body1 = event.body1.and_then(|body| projectiles.get(body).ok());
    let projectile_body2 = event.body2.and_then(|body| projectiles.get(body).ok());

    let projectile_entity = match (projectile_body1, projectile_body2) {
        (Some(e), _) => e,
        (_, Some(e)) => e,
        _ => return,
    };

    let other_collider = if projectile_body1.is_some() {
        event.collider2
    } else {
        event.collider1
    };

    if armors.contains(other_collider)
        || child_of
            .iter_ancestors(other_collider)
            .any(|ancestor| armors.contains(ancestor))
    {
        // 装甲板命中在首次接触时立即结算并移除弹丸。旧实现等待 CollisionEnd，
        // 且只关闭事件，弹丸会继续穿过装甲板，视觉上像是漏检。
        stats.increase_accurate();
        commands.entity(projectile_entity).try_despawn();
    }
}

#[derive(Default)]
pub(super) struct ArmorCollisionPlugin;

impl Plugin for ArmorCollisionPlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.add_observer(handle_armor_collision);
    }
}
