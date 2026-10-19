//! Headless reproduction using the shipped vehicle mesh and real Avian contacts.
use super::*;
use crate::components::GameLayer;
use crate::robomaster::prelude::{INFANTRY_THREE_CONFIG, Team};
use avian3d::prelude::*;

fn armor_mesh_collider() -> Collider {
    let bytes = include_bytes!("../assets/vehicle.glb");
    let json_len = u32::from_le_bytes(bytes[12..16].try_into().unwrap()) as usize;
    let doc: serde_json::Value = serde_json::from_slice(&bytes[20..20 + json_len]).unwrap();
    let binary = &bytes[28 + json_len..];
    let node = doc["nodes"]
        .as_array()
        .unwrap()
        .iter()
        .find(|node| node["name"] == "1_ARMOR")
        .unwrap();
    let primitive = &doc["meshes"][node["mesh"].as_u64().unwrap() as usize]["primitives"][0];
    let read =
        |index: usize, components: usize| {
            let accessor = &doc["accessors"][index];
            let view = &doc["bufferViews"][accessor["bufferView"].as_u64().unwrap() as usize];
            let kind = accessor["componentType"].as_u64().unwrap();
            let size = match kind {
                5126 | 5125 => 4,
                5123 => 2,
                _ => panic!("unsupported accessor"),
            };
            let offset = view["byteOffset"].as_u64().unwrap_or(0) as usize
                + accessor["byteOffset"].as_u64().unwrap_or(0) as usize;
            let stride = view["byteStride"]
                .as_u64()
                .unwrap_or((size * components) as u64) as usize;
            (0..accessor["count"].as_u64().unwrap() as usize)
                .map(|i| {
                    (0..components)
                        .map(|j| {
                            let at = offset + i * stride + j * size;
                            match kind {
                                5126 => f32::from_le_bytes(binary[at..at + 4].try_into().unwrap())
                                    as f64,
                                5125 => u32::from_le_bytes(binary[at..at + 4].try_into().unwrap())
                                    as f64,
                                5123 => u16::from_le_bytes(binary[at..at + 2].try_into().unwrap())
                                    as f64,
                                _ => unreachable!(),
                            }
                        })
                        .collect::<Vec<_>>()
                })
                .collect::<Vec<_>>()
        };
    let vertices = read(
        primitive["attributes"]["POSITION"].as_u64().unwrap() as usize,
        3,
    )
    .iter()
    .map(|p| Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32))
    .collect();
    let indices = read(primitive["indices"].as_u64().unwrap() as usize, 1);
    let triangles = indices
        .chunks_exact(3)
        .map(|t| [t[0][0] as u32, t[1][0] as u32, t[2][0] as u32])
        .collect();
    Collider::trimesh_with_config(vertices, triangles, TrimeshFlags::MERGE_DUPLICATE_VERTICES)
}

#[test]
fn shipped_armor_mesh_projectile_reproduction() {
    for (label, speculative, margin) in [
        ("avian_default", true, f32::MAX),
        ("main_20mm", true, 0.02),
        ("zero", false, 0.0),
    ] {
        for (name, y, z) in [
            ("center", 0.0, 0.0),
            ("above_light_strip_50mm", 0.0, 0.050),
            ("near_upper_edge_60mm", 0.0, 0.060),
            ("outside_upper_edge_80mm", 0.0, 0.080),
            ("outside_upper_edge_100mm", 0.0, 0.100),
            ("outside_upper_edge_150mm", 0.0, 0.150),
            ("outside_upper_edge_300mm", 0.0, 0.300),
            ("near_side_edge_70mm", 0.070, 0.0),
            ("outside_side_edge_90mm", 0.090, 0.0),
            ("body_center", -0.115649, 0.0),
        ] {
            let mut app = App::new();
            app.add_plugins((
                MinimalPlugins,
                TransformPlugin,
                AssetPlugin::default(),
                bevy::mesh::MeshPlugin,
                PhysicsPlugins::default(),
            ))
            .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
                std::time::Duration::from_secs_f64(1.0 / 60.0),
            ))
            .insert_resource(Gravity(Vec3::ZERO))
            .insert_resource(NarrowPhaseConfig {
                default_speculative_margin: margin,
                ..default()
            })
            .insert_resource(Time::<Fixed>::from_hz(120.0))
            .insert_resource(SubstepCount(8))
            .insert_resource(Board(ScoreBook::new(10.0)))
            .add_observer(contact)
            .add_observer(
                |event: On<CollisionStart>,
                 shots: Query<(), With<ScoredShot>>,
                 contacts: Res<ContactGraph>,
                 positions: Query<&Position>| {
                    if let Some(entity) = event
                        .body1
                        .filter(|e| shots.contains(*e))
                        .or_else(|| event.body2.filter(|e| shots.contains(*e)))
                    {
                        let deepest =
                            contacts
                                .get(event.collider1, event.collider2)
                                .and_then(|(_, pair)| {
                                    pair.manifolds
                                        .iter()
                                        .flat_map(|m| m.points.iter())
                                        .map(|p| p.penetration)
                                        .reduce(f32::max)
                                });
                        println!(
                            "  CONTACT position={:?} deepest_penetration_m={deepest:?}",
                            positions.get(entity).unwrap().0
                        );
                    }
                },
            );
            let root = app
                .world_mut()
                .spawn((
                    TrainingTarget {
                        kind: 0,
                        center: Vec3::ZERO,
                    },
                    RigidBody::Static,
                    Transform::default(),
                    Collider::compound(vec![(
                        Vec3::new(0.0, -0.115649, 0.0),
                        Quat::IDENTITY,
                        Collider::cylinder(0.2593615, 0.231298),
                    )]),
                    GameLayer::vehicle_body_collision_layers(false),
                ))
                .id();
            let spec = INFANTRY_THREE_CONFIG.armor;
            // Mesh-local coordinates: X is plate normal, Y horizontal, Z vertical.
            app.world_mut().spawn((
                ChildOf(root),
                Transform::default(),
                armor_mesh_collider(),
                GameLayer::vehicle_armor_collision_layers(false),
                Armor {
                    name: "1_ARMOR".into(),
                    team: Team::Blue,
                    spec,
                    label: spec.label(),
                },
            ));
            let shot = {
                let mut board = app.world_mut().resource_mut::<Board>();
                board.0.running = true;
                board.0.launch().unwrap()
            };
            let projectile = app
                .world_mut()
                .spawn((
                    RigidBody::Dynamic,
                    SweptCcd::default(),
                    Collider::sphere(0.017 / 2.0),
                    CollisionEventsEnabled,
                    GameLayer::projectile_collision_layers(true),
                    Transform::from_xyz(0.5, y, z),
                    LinearVelocity(Vec3::NEG_X * 25.0),
                    ScoredShot(shot),
                ))
                .id();
            if !speculative {
                app.world_mut()
                    .entity_mut(projectile)
                    .insert(SpeculativeMargin(0.0));
            }
            app.finish();
            app.cleanup();
            for _ in 0..12 {
                app.update();
            }
            let score = &app.world().resource::<Board>().0.scores[0];
            println!(
                "REPRO mode={label} speculative={speculative} {name}: y={y:.3} z={z:.3} shots={} hits={} damage={}",
                score.shots, score.hits, score.damage
            );
            assert_eq!(score.shots, 1);
            if name == "center" {
                assert_eq!(score.hits, 1);
            }
            if name == "outside_upper_edge_300mm" {
                assert_eq!(score.hits, 0);
            }
            if label != "main_20mm"
                && matches!(
                    name,
                    "outside_upper_edge_80mm"
                        | "outside_upper_edge_100mm"
                        | "outside_upper_edge_150mm"
                        | "outside_side_edge_90mm"
                        | "body_center"
                )
            {
                assert_eq!(
                    score.hits,
                    u64::from(speculative),
                    "{name}, speculative={speculative}"
                );
            }
        }
    }
}
