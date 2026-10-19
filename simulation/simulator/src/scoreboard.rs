use avian3d::prelude::CollisionStart;
use bevy::prelude::*;
use bevy::render::view::screenshot::{Screenshot, save_to_disk};
use crate::config::SimulationConfig;
use crate::score_model::{ScoreBook, Shot};
use crate::training::TrainingTarget;
use crate::robomaster::prelude::Armor;

#[derive(Resource)]
pub struct Board(pub ScoreBook);
#[derive(Component)]
pub struct ScoredShot(pub Shot);
#[derive(Component)]
struct BoardText(usize);
#[derive(Component)]
struct Action(u8);
#[derive(Resource, Default)]
struct ExportStatus(String);

pub struct ScoreboardPlugin;
impl Plugin for ScoreboardPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(Board(ScoreBook::new(SimulationConfig::default().training.damage_per_hit)))
            .init_resource::<ExportStatus>()
            .add_systems(Startup, setup)
            .add_systems(Update, (tick, actions, update_text, capture_preview).chain())
            .add_observer(contact);
    }
}

fn setup(mut commands: Commands, assets: Res<AssetServer>, config: Res<SimulationConfig>) {
    if !config.training.enabled { return; }
    let font = assets.load("fonts/NotoSansCJKSC-Regular.otf");
    commands.spawn((Node { position_type: PositionType::Absolute, top: px(12), right: px(12), left: px(12),
        padding: UiRect::all(px(10)), flex_direction: FlexDirection::Column,
        row_gap: px(6), ..default() }, BackgroundColor(Color::srgba(0.025,0.035,0.055,0.94)),
        GlobalZIndex(100)))
    .with_children(|p| {
        p.spawn((Text::new("三靶测试 · 装甲板命中"), TextFont { font: FontSource::Handle(font.clone()), font_size: FontSize::Px(22.), ..default() }));
        p.spawn((Node { flex_direction: FlexDirection::Row, column_gap: px(6), ..default() }))
            .with_children(|row| {
                for (id,label) in [(0,"静止 3"),(1,"平移 4"),(2,"旋转 2"),(3,"开始/暂停"),(4,"清零全部"),(5,"导出 JSON")] {
                    row.spawn((Button, Action(id), Node { padding: UiRect::axes(px(8),px(5)), ..default() },
                        BackgroundColor(Color::srgb(0.12,0.20,0.30))))
                    .with_child((Text::new(label), TextFont { font: FontSource::Handle(font.clone()),font_size: FontSize::Px(16.),..default() }));
                }
            });
        p.spawn((Text::new(""), BoardText(3), TextFont { font: FontSource::Handle(font.clone()),font_size: FontSize::Px(16.),..default() }));
        p.spawn((Node { flex_direction: FlexDirection::Row, column_gap: px(10), ..default() }))
            .with_children(|row| {
                for id in 0..3 {
                    row.spawn((Node { flex_grow: 1., flex_basis: percent(0), padding: UiRect::all(px(8)), ..default() },
                        BackgroundColor(Color::srgb(0.08,0.12,0.18))))
                    .with_child((Text::new(""),BoardText(id),TextFont { font: FontSource::Handle(font.clone()),font_size: FontSize::Px(16.),..default() }));
                }
            });
        p.spawn((Text::new("F6 选靶  F7 开始/暂停  F8 清零  F9 导出　｜　命中率=本靶命中/本轮发射；误中其他车不计伤害\nDPS=总伤/该靶累计测试秒数；未开始的射击不统计；暂停后可换靶"),
            TextFont { font: FontSource::Handle(font),font_size: FontSize::Px(13.),..default() },TextColor(Color::srgb(0.7,0.8,0.9))));
    });
}

fn tick(time: Res<Time>, mut board: ResMut<Board>) { board.0.tick(time.delta_secs_f64()); }

fn actions(keys: Res<ButtonInput<KeyCode>>, buttons: Query<(&Interaction,&Action),Changed<Interaction>>,
    mut board: ResMut<Board>, mut status: ResMut<ExportStatus>) {
    let mut actions=Vec::new();
    for (interaction, action) in &buttons { if *interaction==Interaction::Pressed { actions.push(action.0); } }
    if keys.just_pressed(KeyCode::F6) { actions.push(((board.0.selected+1)%3) as u8); }
    if keys.just_pressed(KeyCode::F7) { actions.push(3); }
    if keys.just_pressed(KeyCode::F8) { actions.push(4); }
    if keys.just_pressed(KeyCode::F9) { actions.push(5); }
    for action in actions {
        match action {
            0..=2 => board.0.select(action as usize),
            3 => board.0.running=!board.0.running,
            4 => { board.0.reset(); status.0.clear(); },
            5 => {
                let rows:Vec<_>=board.0.scores.iter().enumerate().map(|(i,s)| serde_json::json!({
                    "target":i,"shots":s.shots,"hits":s.hits,"off_target":s.off_target,"damage":s.damage,
                    "test_seconds":s.seconds,"accuracy_percent":s.accuracy(),"dps":s.dps()})).collect();
                let output=serde_json::json!({"damage_per_hit":board.0.damage_per_hit,"targets":rows});
                status.0=match std::fs::write("training_scores.json",serde_json::to_string_pretty(&output).unwrap()) {
                    Ok(())=>"已导出 training_scores.json".into(), Err(e)=>format!("导出失败: {e}") };
            }, _=>{}
        }
    }
}

fn update_text(board: Res<Board>, status: Res<ExportStatus>, mut text: Query<(&BoardText,&mut Text)>) {
    let b=&board.0; let names=["静止靶 3","平移靶 4","旋转靶 2"];
    for (slot,mut t) in &mut text {
        let value=if slot.0==3 {
            format!("当前：{} · {}　单发伤害 {:.1}（教学设定）　{}",names[b.selected],if b.running {"计时中"} else {"已暂停"},b.damage_per_hit,status.0)
        } else {
            let s=&b.scores[slot.0];
            format!("{}　{:.1} 秒\n总伤 {:.1}\n命中率 {:.1}%　DPS {:.2}\n发射 {}　命中 {}　误中 {}",names[slot.0],s.seconds,s.damage,s.accuracy(),s.dps(),s.shots,s.hits,s.off_target)
        };
        if **t!=value { **t=value; }
    }
}

fn contact(event: On<CollisionStart>, mut shots: Query<&mut ScoredShot>,
    ids: Query<&crate::ballistics::LegacyShotId>,
    armors: Query<(),With<Armor>>, parents: Query<&ChildOf>, targets: Query<&TrainingTarget>,
    transforms: Query<&GlobalTransform>,
    mut board: ResMut<Board>) {
    let a=event.body1.filter(|e| shots.contains(*e));
    let b=event.body2.filter(|e| shots.contains(*e));
    let Some(projectile)=a.or(b) else { return; };
    let other=if a.is_some(){event.collider2}else{event.collider1};
    let armor_entity=if armors.contains(other) { Some(other) }
        else { parents.iter_ancestors(other).find(|e|armors.contains(*e)) };
    let armor=armor_entity.is_some();
    let target=if armor {
        targets.get(other).ok().map(|t|t.kind).or_else(||parents.iter_ancestors(other).find_map(|e|targets.get(e).ok().map(|t|t.kind)))
    } else {None};
    if let Some(armor_entity)=armor_entity {
        if let (Ok(projectile_tf),Ok(armor_tf))=
            (transforms.get(projectile),transforms.get(armor_entity)) {
            let local=armor_tf.affine().inverse().transform_point3(projectile_tf.translation());
            info!("TRAINING_ARMOR_HIT target={:?} local_xyz=({:.4},{:.4},{:.4})",
                target,local.x,local.y,local.z);
        }
    }
    if let Ok(mut shot)=shots.get_mut(projectile) {
        if !shot.0.resolved { info!("LEGACY_IMPACT shot_id={:?} target={target:?}", ids.get(projectile).ok().map(|id|id.0)); }
        board.0.contact(&mut shot.0,target);
    }
}

// Optional renderer smoke-test snapshot; absent in normal sessions.
fn capture_preview(time: Res<Time>, mut done: Local<bool>, mut commands: Commands) {
    if !*done && time.elapsed_secs()>5.0 {
        *done=true;
        if let Ok(path)=std::env::var("TRAINING_PANEL_SCREENSHOT") {
            commands.spawn(Screenshot::primary_window()).observe(save_to_disk(path));
        }
    }
}

#[cfg(test)]
#[path = "collision_repro_tests.rs"]
mod collision_repro_tests;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robomaster::prelude::{Team, INFANTRY_THREE_CONFIG};
    #[test]
    fn moving_projectile_scores_through_real_physics_events() {
        use avian3d::prelude::*;
        let mut app=App::new();
        app.add_plugins((MinimalPlugins,TransformPlugin,AssetPlugin::default(),bevy::mesh::MeshPlugin,PhysicsPlugins::default()))
            .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(std::time::Duration::from_secs_f64(1.0/60.0)))
            .insert_resource(Gravity(Vec3::ZERO))
            .insert_resource(Board(ScoreBook::new(10.0)))
            .add_observer(contact);
        let root=app.world_mut().spawn((TrainingTarget{kind:0,center:Vec3::ZERO},RigidBody::Static,Transform::default())).id();
        let spec=INFANTRY_THREE_CONFIG.armor;
        app.world_mut().spawn((ChildOf(root),Transform::default(),Collider::cuboid(0.1,0.5,0.5),
            Armor{name:"physical armor".into(),team:Team::Blue,spec,label:spec.label()}));
        let shot={let mut b=app.world_mut().resource_mut::<Board>();b.0.running=true;b.0.tick(2.0);b.0.launch().unwrap()};
        // Mirrors the collision-related components installed by projectile_launch.
        app.world_mut().spawn((RigidBody::Dynamic,SweptCcd::default(),Collider::sphere(0.02),CollisionEventsEnabled,
            Transform::from_xyz(-1.,0.,0.),LinearVelocity(Vec3::X*2.),ScoredShot(shot)));
        app.finish(); app.cleanup();
        for _ in 0..90 {app.update();}
        let s=&app.world().resource::<Board>().0.scores[0];
        assert_eq!(s.hits,1);assert_eq!(s.damage,10.0);assert_eq!(s.accuracy(),100.0);
    }
    #[test]
    fn collision_observer_resolves_nested_armors_and_deduplicates_each_car() {
        let mut app=App::new();
        app.insert_resource(Board(ScoreBook::new(10.0))).add_observer(contact);
        for kind in 0..3 {
            let root=app.world_mut().spawn(TrainingTarget{kind,center:Vec3::ZERO}).id();
            let spec=INFANTRY_THREE_CONFIG.armor;
            let armor=app.world_mut().spawn((ChildOf(root),Armor{name:"test".into(),team:Team::Blue,spec,label:spec.label()})).id();
            let collider=app.world_mut().spawn(ChildOf(armor)).id();
            let shot={let mut b=app.world_mut().resource_mut::<Board>();b.0.running=false;b.0.select(kind);b.0.running=true;b.0.tick(2.0);b.0.launch().unwrap()};
            let projectile=app.world_mut().spawn(ScoredShot(shot)).id();
            for _ in 0..2 {app.world_mut().trigger(CollisionStart{collider1:projectile,collider2:collider,body1:Some(projectile),body2:Some(root)});}
        }
        let b=&app.world().resource::<Board>().0;
        for s in &b.scores {assert_eq!(s.shots,1);assert_eq!(s.hits,1);assert_eq!(s.damage,10.0);assert_eq!(s.dps(),5.0);}
    }
    #[test]
    fn vehicle_body_contact_is_not_an_armor_hit() {
        let mut app=App::new();
        app.insert_resource(Board(ScoreBook::new(10.0))).add_observer(contact);
        let body=app.world_mut().spawn(TrainingTarget{kind:0,center:Vec3::ZERO}).id();
        let shot={let mut b=app.world_mut().resource_mut::<Board>();b.0.running=true;b.0.launch().unwrap()};
        let projectile=app.world_mut().spawn(ScoredShot(shot)).id();
        app.world_mut().trigger(CollisionStart{collider1:projectile,collider2:body,body1:Some(projectile),body2:Some(body)});
        let s=&app.world().resource::<Board>().0.scores[0];
        assert_eq!(s.shots,1);assert_eq!(s.hits,0);assert_eq!(s.damage,0.0);
    }

}
