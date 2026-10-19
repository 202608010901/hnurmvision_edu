use bevy::{
    asset::AssetServer,
    audio::AudioPlayer,
    ecs::{
        observer::On,
        system::{Commands, Query, Res, ResMut},
    },
    log::info,
    transform::components::Transform,
};

use crate::{
    robomaster::prelude::{PowerRune, RuneActivated, RuneHit, Team},
    statistic::ProjectileStatistics,
};

pub fn on_activate(
    ev: On<RuneActivated>,
    mut commands: Commands,
    query: Query<&PowerRune>,
    asset_server: Res<AssetServer>,
) {
    let Ok(rune) = query.get(ev.rune) else {
        return;
    };
    info!(
        "Power rune fully activated: team={:?}, mode={:?}",
        rune.team(),
        rune.mode()
    );
    commands.spawn(AudioPlayer::new(asset_server.load("rune_activated.ogg")));
}

pub fn on_hit(
    ev: On<RuneHit>,
    mut stats: ResMut<ProjectileStatistics>,
    _commands: Commands,
    query: Query<(&Transform, &PowerRune)>,
) {
    let Ok((_transform, rune)) = query.get(ev.rune) else {
        return;
    };
    // 受控机器人位于红方，只能把命中蓝方机关计为有效；弹丸穿过目标后撞到
    // 背面的红方机关，即使其规则结果为 PrimaryHit，也属于无效弹。
    if ev.result.accurate() && rune.team() == Team::Blue {
        stats.increase_accurate();
        //commands.spawn(AudioPlayer::new(asset_server.load("rune_activated.ogg")));
    }
    info!(
        "Power rune projectile result: team={:?}, mode={:?}, outcome={:?}, accurate_total={}",
        rune.team(),
        rune.mode(),
        ev.result.outcome,
        stats.accurate_count
    );
}
