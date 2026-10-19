use super::*;

fn object(id: u32, shape: SharedShape, position: Vec3, kind: ImpactKind) -> MovingShape {
    let aabb = shape.compute_local_aabb();
    let bound = aabb.mins.abs().max(aabb.maxs.abs()).length();
    MovingShape {
        id: Entity::from_bits(id as u64 + (1 << 32)),
        shape,
        motion: NonlinearRigidMotion::new(
            pose(position, Quat::IDENTITY),
            Vec3::ZERO,
            Vec3::ZERO,
            Vec3::ZERO,
        ),
        front: (kind == ImpactKind::Armor).then_some(Vec3::X),
        kind,
        target: Some(0),
        bound,
    }
}
fn plate(id: u32, x: f32) -> MovingShape {
    object(
        id,
        SharedShape::cuboid(0.0015, 0.0675, 0.0625),
        Vec3::X * x,
        ImpactKind::Armor,
    )
}
fn hit(p: Vec3, delta: Vec3, dt: f32, objects: &[MovingShape]) -> Option<Hit> {
    first_hit(
        p,
        delta,
        0.0085,
        dt,
        objects,
        &mut BallisticDiagnostics::default(),
    )
    .expect("query converges")
}

#[test]
fn center_and_geometric_near_misses_at_multiple_speeds() {
    for speed in [10.0, 25.0, 50.0, 100.0] {
        let dt = 2.0 / speed;
        assert_eq!(
            hit(Vec3::X, Vec3::NEG_X * 2.0, dt, &[plate(1, 0.0)])
                .unwrap()
                .kind,
            ImpactKind::Armor
        );
        for (y, z) in [(0.0, 0.080), (0.0, 0.100), (0.0, 0.150), (0.090, 0.0)] {
            assert!(
                hit(
                    Vec3::new(1.0, y, z),
                    Vec3::NEG_X * 2.0,
                    dt,
                    &[plate(1, 0.0)]
                )
                .is_none()
            );
        }
    }
}
#[test]
fn finite_radius_edges_corners_and_backface() {
    assert!(
        hit(
            Vec3::new(1.0, 0.073, 0.0),
            Vec3::NEG_X * 2.0,
            0.08,
            &[plate(1, 0.0)]
        )
        .is_some()
    );
    assert!(
        hit(
            Vec3::new(1.0, 0.074, 0.069),
            Vec3::NEG_X * 2.0,
            0.08,
            &[plate(1, 0.0)]
        )
        .is_none()
    );
    assert_eq!(
        hit(Vec3::NEG_X, Vec3::X * 2.0, 0.08, &[plate(1, 0.0)])
            .unwrap()
            .kind,
        ImpactKind::Body
    );
    assert_eq!(
        hit(Vec3::ZERO, Vec3::X, 0.04, &[plate(1, 0.0)])
            .unwrap()
            .kind,
        ImpactKind::InitialOverlap
    );
}
#[test]
fn earliest_blocker_wins_independent_of_iteration_order() {
    let armor = plate(1, 0.0);
    let wall = object(
        2,
        SharedShape::cuboid(0.01, 0.2, 0.2),
        Vec3::X * 0.5,
        ImpactKind::Environment,
    );
    for objects in [
        vec![armor.clone(), wall.clone()],
        vec![wall.clone(), armor.clone()],
    ] {
        let h = hit(Vec3::X, Vec3::NEG_X * 2.0, 0.08, &objects).unwrap();
        assert_eq!(h.collider, wall.id);
        assert_eq!(h.kind, ImpactKind::Environment);
    }
    let body = object(
        3,
        SharedShape::cuboid(0.01, 0.2, 0.2),
        Vec3::X * 0.5,
        ImpactKind::Body,
    );
    assert_eq!(
        hit(Vec3::X, Vec3::NEG_X * 2.0, 0.08, &[armor, body])
            .unwrap()
            .kind,
        ImpactKind::Body
    );
}
#[test]
fn translating_target_crosses_stationary_bullet() {
    let mut target = plate(1, 0.0);
    target.motion.start = pose(Vec3::X * 0.5, Quat::IDENTITY);
    target.motion.linvel = Vec3::NEG_X * 10.0;
    let h = hit(Vec3::ZERO, Vec3::ZERO, 0.1, &[target]).unwrap();
    assert!(h.toi > 0.04 && h.toi < 0.06);
}
#[test]
fn rotating_plate_sweeps_around_vehicle_origin() {
    // A small plate starts +X from its pivot and sweeps through +Z. A frozen
    // endpoint query misses both ends; nonlinear motion must detect the arc.
    let shape = SharedShape::cuboid(0.03, 0.03, 0.03);
    let mut target = object(1, shape, Vec3::X * 0.2, ImpactKind::Body);
    target.motion = NonlinearRigidMotion::new(
        pose(Vec3::X * 0.2, Quat::IDENTITY),
        Vec3::NEG_X * 0.2,
        Vec3::ZERO,
        Vec3::NEG_Y * std::f32::consts::PI,
    );
    target.bound = 0.26;
    assert!(hit(Vec3::Z * 0.2, Vec3::ZERO, 1.0, &[target]).is_some());
}
#[test]
fn impact_time_matches_analytic_sphere_box_and_fixed_rate() {
    let expected = (1.0 - 0.0015 - 0.0085) / 25.0;
    for hz in [60.0, 120.0, 240.0] {
        let dt = 1.0 / hz;
        let mut p = Vec3::X;
        let mut elapsed = 0.0;
        let mut found = None;
        for _ in 0..30 {
            if let Some(h) = hit(p, Vec3::NEG_X * 25.0 * dt, dt, &[plate(1, 0.0)]) {
                found = Some(elapsed + h.toi);
                break;
            }
            p += Vec3::NEG_X * 25.0 * dt;
            elapsed += dt;
        }
        assert!((found.unwrap() - expected).abs() < 0.00002);
    }
}
#[test]
fn mesh_selection_excludes_decorations() {
    for name in [
        "1_ARMOR_L_L",
        "1_ARMOR_MARKER",
        "1_ARMOR_CENTER",
        "1_ARMOR_ROOT",
        "12",
    ] {
        assert_eq!(mesh_kind(name), None);
    }
    assert_eq!(mesh_kind("1_ARMOR"), Some(ImpactKind::Armor));
    assert_eq!(mesh_kind("1_ARMOR_BASE"), Some(ImpactKind::Body));
}
#[test]
fn parking_is_not_a_motion_sweep_and_pose_matches_formula() {
    let target = TrainingTarget {
        kind: 2,
        center: Vec3::new(2.0, 0.0, 1.0),
    };
    let mut c = crate::config::TrainingConfig::default();
    c.solo_target = true;
    let (parked, _) = target.pose_at(&c, 0, 1.0);
    let (active, _) = target.pose_at(&c, 2, 1.0);
    assert!(parked.y > 40.0);
    assert_eq!(active, Vec3::new(2.0, 0.0, 0.0));
    assert_eq!(segment_count(1.0 / 120.0, 9.81), Some(8));
    assert!(segment_count(10.0, 9.81).is_none());
}

/// Load the shipped mesh and full node transforms into ECS without a renderer.
/// This exercises production registration, including real chassis/base occluders.
fn load_vehicle(app: &mut App, target: TrainingTarget) -> Entity {
    use bevy::{
        asset::RenderAssetUsages,
        mesh::{Indices, PrimitiveTopology},
    };
    let bytes = include_bytes!("../assets/vehicle.glb");
    let size = u32::from_le_bytes(bytes[12..16].try_into().unwrap()) as usize;
    let doc: serde_json::Value = serde_json::from_slice(&bytes[20..20 + size]).unwrap();
    let binary = &bytes[28 + size..];
    let read =
        |index: usize, components: usize| {
            let a = &doc["accessors"][index];
            let v = &doc["bufferViews"][a["bufferView"].as_u64().unwrap() as usize];
            let kind = a["componentType"].as_u64().unwrap();
            let size = match kind {
                5126 | 5125 => 4,
                5123 => 2,
                _ => panic!("accessor"),
            };
            let offset = v["byteOffset"].as_u64().unwrap_or(0) as usize
                + a["byteOffset"].as_u64().unwrap_or(0) as usize;
            let stride = v["byteStride"]
                .as_u64()
                .unwrap_or((size * components) as u64) as usize;
            (0..a["count"].as_u64().unwrap() as usize)
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
    let root = app.world_mut().spawn((target, Transform::default())).id();
    let mut entities = Vec::new();
    for node in doc["nodes"].as_array().unwrap() {
        let mut transform = Transform::default();
        if let Some(v) = node["translation"].as_array() {
            transform.translation = Vec3::new(
                v[0].as_f64().unwrap() as f32,
                v[1].as_f64().unwrap() as f32,
                v[2].as_f64().unwrap() as f32,
            );
        }
        if let Some(v) = node["rotation"].as_array() {
            transform.rotation = Quat::from_xyzw(
                v[0].as_f64().unwrap() as f32,
                v[1].as_f64().unwrap() as f32,
                v[2].as_f64().unwrap() as f32,
                v[3].as_f64().unwrap() as f32,
            );
        }
        if let Some(v) = node["scale"].as_array() {
            transform.scale = Vec3::new(
                v[0].as_f64().unwrap() as f32,
                v[1].as_f64().unwrap() as f32,
                v[2].as_f64().unwrap() as f32,
            );
        }
        let entity = app
            .world_mut()
            .spawn((
                Name::new(node["name"].as_str().unwrap_or("").to_owned()),
                transform,
            ))
            .id();
        if let Some(index) = node["mesh"].as_u64() {
            let p = &doc["meshes"][index as usize]["primitives"][0];
            let vertices: Vec<[f32; 3]> =
                read(p["attributes"]["POSITION"].as_u64().unwrap() as usize, 3)
                    .iter()
                    .map(|p| [p[0] as f32, p[1] as f32, p[2] as f32])
                    .collect();
            let indices: Vec<u32> = read(p["indices"].as_u64().unwrap() as usize, 1)
                .iter()
                .map(|i| i[0] as u32)
                .collect();
            let mut mesh = Mesh::new(
                PrimitiveTopology::TriangleList,
                RenderAssetUsages::default(),
            );
            mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, vertices);
            mesh.insert_indices(Indices::U32(indices));
            let handle = app.world_mut().resource_mut::<Assets<Mesh>>().add(mesh);
            app.world_mut()
                .spawn((Mesh3d(handle), Transform::default(), ChildOf(entity)));
        }
        entities.push(entity);
    }
    for (i, node) in doc["nodes"].as_array().unwrap().iter().enumerate() {
        if let Some(children) = node["children"].as_array() {
            for child in children {
                app.world_mut()
                    .entity_mut(entities[child.as_u64().unwrap() as usize])
                    .insert(ChildOf(entities[i]));
            }
        }
    }
    app.world_mut()
        .entity_mut(*entities.last().unwrap())
        .insert(ChildOf(root));
    root
}

fn app() -> App {
    let mut app = App::new();
    let mut config = SimulationConfig::default();
    config.training.enabled = true;
    config.training.solo_target = false;
    config.projectile.aerodynamics.enabled = false;
    config.projectile.linear_damping = 0.0;
    app.add_plugins((
        MinimalPlugins,
        TransformPlugin,
        AssetPlugin::default(),
        bevy::mesh::MeshPlugin,
    ))
    .insert_resource(config)
    .insert_resource(Gravity(Vec3::ZERO))
    .insert_resource(Board(crate::score_model::ScoreBook::new(10.0)))
    .init_resource::<ProjectileStatistics>()
    .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
        std::time::Duration::from_secs_f64(1.0 / 60.0),
    ))
    .insert_resource(Time::<Fixed>::from_hz(120.0))
    .add_plugins(BallisticsPlugin);
    app
}

#[test]
fn shipped_vehicle_center_surface_scores_before_real_backing_and_chassis() {
    let mut app = app();
    load_vehicle(
        &mut app,
        TrainingTarget {
            kind: 0,
            center: Vec3::ZERO,
        },
    );
    app.finish();
    app.cleanup();
    app.update();
    app.update();
    let registry = app.world().resource::<HitGeometry>();
    assert_eq!(
        registry.0.len(),
        10,
        "four armor surfaces, four bases, chassis and gimbal"
    );
    let geometries: Vec<_> = registry.0.iter().map(|(id, g)| (*id, g.clone())).collect();
    let objects: Vec<_> = geometries
        .iter()
        .map(|(id, g)| MovingShape {
            id: *id,
            shape: g.shape.clone(),
            motion: NonlinearRigidMotion::new(
                pose(Vec3::ZERO, Quat::IDENTITY),
                Vec3::ZERO,
                Vec3::ZERO,
                Vec3::ZERO,
            ),
            front: g.front,
            kind: g.kind,
            target: Some(0),
            bound: g.bound,
        })
        .collect();
    for (_, g) in &geometries {
        if g.kind != ImpactKind::Armor {
            continue;
        }
        let center = g.shape.compute_local_aabb().center();
        let front = g.front.unwrap();
        let h = hit(center + front * 0.5, -front, 0.04, &objects).unwrap();
        assert_eq!(h.kind, ImpactKind::Armor, "front={front:?} hit={h:?}");
    }
}

#[test]
fn ecs_resolves_one_impact_and_shadows_cannot_change_score() {
    for shadow in [false, true] {
        let mut app = app();
        let root = app
            .world_mut()
            .spawn((
                TrainingTarget {
                    kind: 0,
                    center: Vec3::ZERO,
                },
                Transform::default(),
            ))
            .id();
        // Give production registration a finite box with an explicit armor name.
        let mesh = Mesh::from(Cuboid::new(0.003, 0.135, 0.125));
        let handle = app.world_mut().resource_mut::<Assets<Mesh>>().add(mesh);
        app.world_mut().spawn((
            Name::new("1_ARMOR"),
            Mesh3d(handle),
            Transform::default(),
            ChildOf(root),
        ));
        let shot = {
            let mut board = app.world_mut().resource_mut::<Board>();
            board.0.running = true;
            board.0.launch().unwrap()
        };
        // Static target has a +90 degree Y rotation: local +X is world -Z.
        let bullet = app
            .world_mut()
            .spawn((
                BallisticProjectile {
                    id: 1,
                    position: Vec3::NEG_Z,
                    velocity: Vec3::Z * 25.0,
                    radius: 0.0085,
                    age: 0.0,
                    born_at: 0.0,
                    lifetime: 2.0,
                    shot: Some(shot),
                    shadow,
                    resolved: false,
                },
                Transform::from_translation(Vec3::NEG_Z),
            ))
            .id();
        app.finish();
        app.cleanup();
        for _ in 0..15 {
            app.update();
        }
        assert!(app.world().get_entity(bullet).is_err());
        let board = &app.world().resource::<Board>().0;
        assert_eq!(board.scores[0].hits, u64::from(!shadow));
        assert_eq!(app.world().resource::<BallisticDiagnostics>().impacts, 1);
        assert_eq!(app.world().resource::<BallisticDiagnostics>().failures, 0);
    }
}

#[test]
fn all_three_real_targets_score_with_frame_accumulation() {
    for kind in 0..3 {
        for frame_hz in [30.0, 60.0] {
            let mut app = app();
            app.world_mut()
                .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
                    std::time::Duration::from_secs_f64(1.0 / frame_hz),
                ));
            load_vehicle(
                &mut app,
                TrainingTarget {
                    kind,
                    center: Vec3::ZERO,
                },
            );
            app.finish();
            app.cleanup();
            app.update();
            app.update();
            let born = app.world().resource::<Time<Fixed>>().elapsed_secs_f64();
            let config = &app.world().resource::<SimulationConfig>().training;
            let (p, q) = TrainingTarget {
                kind,
                center: Vec3::ZERO,
            }
            .pose_at(config, kind, born + 0.04);
            let registry = app.world().resource::<HitGeometry>();
            let g = registry
                .0
                .values()
                .filter(|g| g.kind == ImpactKind::Armor)
                .min_by(|a, b| {
                    (q * a.front.unwrap())
                        .z
                        .total_cmp(&(q * b.front.unwrap()).z)
                })
                .unwrap();
            let center = p + q * g.shape.compute_local_aabb().center();
            let front = q * g.front.unwrap();
            let origin = center + front;
            let shot = {
                let mut board = app.world_mut().resource_mut::<Board>();
                board.0.selected = kind;
                board.0.running = true;
                board.0.launch().unwrap()
            };
            app.world_mut().spawn((
                BallisticProjectile {
                    id: 1,
                    position: origin,
                    velocity: -front * 25.0,
                    radius: 0.0085,
                    age: 0.0,
                    born_at: born,
                    lifetime: 1.0,
                    shot: Some(shot),
                    shadow: false,
                    resolved: false,
                },
                Transform::from_translation(origin),
            ));
            for _ in 0..20 {
                app.update();
            }
            let score = &app.world().resource::<Board>().0.scores[kind];
            assert_eq!(
                score.hits,
                1,
                "kind={kind} render_hz={frame_hz}, diagnostics={:?}",
                app.world().resource::<BallisticDiagnostics>()
            );
            assert_eq!(score.damage, 10.0);
            assert_eq!(app.world().resource::<BallisticDiagnostics>().impacts, 1);
            assert_eq!(app.world().resource::<BallisticDiagnostics>().failures, 0);
        }
    }
}

#[test]
fn expiry_and_clear_reject_inflight_score() {
    let mut app = app();
    let shot = {
        let mut board = app.world_mut().resource_mut::<Board>();
        board.0.running = true;
        board.0.launch().unwrap()
    };
    app.world_mut().spawn((
        BallisticProjectile {
            id: 1,
            position: Vec3::ZERO,
            velocity: Vec3::X,
            radius: 0.0085,
            age: 0.0,
            born_at: 0.0,
            lifetime: 0.02,
            shot: Some(shot),
            shadow: false,
            resolved: false,
        },
        Transform::default(),
    ));
    app.world_mut().resource_mut::<Board>().0.reset();
    app.finish();
    app.cleanup();
    for _ in 0..10 {
        app.update();
    }
    assert_eq!(app.world().resource::<BallisticDiagnostics>().impacts, 1);
    assert_eq!(app.world().resource::<BallisticDiagnostics>().failures, 0);
    assert_eq!(app.world().resource::<Board>().0.scores[0].shots, 0);
    assert_eq!(app.world().resource::<Board>().0.scores[0].hits, 0);
}

#[test]
fn shipped_mesh_rejects_original_near_miss_tracks() {
    let mut app = app();
    let root = load_vehicle(
        &mut app,
        TrainingTarget {
            kind: 0,
            center: Vec3::ZERO,
        },
    );
    app.finish();
    app.cleanup();
    app.update();
    app.update();
    let entity = {
        let world = app.world_mut();
        let mut q = world.query::<(Entity, &Name)>();
        q.iter(world)
            .find(|(_, n)| n.as_str() == "1_ARMOR")
            .unwrap()
            .0
    };
    let mut affine = Affine3A::IDENTITY;
    let mut current = entity;
    while current != root {
        affine = app
            .world()
            .get::<Transform>(current)
            .unwrap()
            .compute_affine()
            * affine;
        current = app.world().get::<ChildOf>(current).unwrap().parent();
    }
    let registry = app.world().resource::<HitGeometry>();
    let g = registry
        .0
        .iter()
        .find(|(id, _)| {
            **id == entity
                || app
                    .world()
                    .get::<ChildOf>(**id)
                    .is_some_and(|p| p.parent() == entity)
        })
        .unwrap()
        .1
        .clone();
    let shape = MovingShape {
        id: entity,
        shape: g.shape,
        motion: NonlinearRigidMotion::new(
            pose(Vec3::ZERO, Quat::IDENTITY),
            Vec3::ZERO,
            Vec3::ZERO,
            Vec3::ZERO,
        ),
        front: g.front,
        kind: ImpactKind::Armor,
        target: Some(0),
        bound: g.bound,
    };
    for speed in [10.0, 25.0, 50.0, 100.0] {
        assert_eq!(
            hit(
                affine.transform_point3(Vec3::X),
                affine.transform_vector3(Vec3::NEG_X * 2.0),
                2.0 / speed,
                &[shape.clone()]
            )
            .unwrap()
            .kind,
            ImpactKind::Armor
        );
        for (y, z) in [(0.0, 0.080), (0.0, 0.100), (0.0, 0.150), (0.090, 0.0)] {
            assert!(
                hit(
                    affine.transform_point3(Vec3::new(1.0, y, z)),
                    affine.transform_vector3(Vec3::NEG_X * 2.0),
                    2.0 / speed,
                    &[shape.clone()]
                )
                .is_none(),
                "speed={speed}, y={y}, z={z}"
            );
        }
    }
}

#[test]
fn gltf_unnamed_primitive_child_is_registered() {
    let mut app = app();
    let root = app
        .world_mut()
        .spawn((
            TrainingTarget {
                kind: 0,
                center: Vec3::ZERO,
            },
            Transform::default(),
        ))
        .id();
    let node = app
        .world_mut()
        .spawn((Name::new("1_ARMOR"), Transform::default(), ChildOf(root)))
        .id();
    let mesh = app
        .world_mut()
        .resource_mut::<Assets<Mesh>>()
        .add(Mesh::from(Cuboid::new(0.003, 0.135, 0.125)));
    app.world_mut()
        .spawn((Mesh3d(mesh), Transform::default(), ChildOf(node)));
    app.finish();
    app.cleanup();
    app.update();
    app.update();
    let r = app.world().resource::<HitGeometry>();
    assert_eq!(r.0.len(), 1);
    assert_eq!(r.0.values().next().unwrap().kind, ImpactKind::Armor);
}

#[test]
fn gravity_flight_matches_analytic_solution_until_expiry() {
    let mut app = app();
    app.world_mut()
        .resource_mut::<SimulationConfig>()
        .training
        .hit_debug = true;
    app.world_mut().insert_resource(Gravity(Vec3::NEG_Y * 9.81));
    let origin = Vec3::new(1.0, 2.0, 3.0);
    let velocity = Vec3::new(25.0, 1.0, 0.0);
    app.world_mut().spawn((
        BallisticProjectile {
            id: 1,
            position: origin,
            velocity,
            radius: 0.0085,
            age: 0.0,
            born_at: 0.0,
            lifetime: 0.1,
            shot: None,
            shadow: false,
            resolved: false,
        },
        Transform::from_translation(origin),
    ));
    app.finish();
    app.cleanup();
    for _ in 0..10 {
        app.update();
    }
    let trace = app.world().resource::<FlightTrace>();
    let impact = trace.hits.back().unwrap();
    assert_eq!(impact.kind, ImpactKind::Expired);
    let expected = origin + velocity * 0.1 + Vec3::NEG_Y * (0.5 * 9.81 * 0.1 * 0.1);
    assert!(
        impact.point_world.distance(expected) < 0.0001,
        "actual={:?} expected={expected:?}",
        impact.point_world
    );
}
#[test]
fn oblique_thin_plate_does_not_tunnel_at_high_speed() {
    let rotation = Quat::from_rotation_z(70_f32.to_radians());
    let mut p = plate(1, 0.0);
    p.motion.start = pose(Vec3::ZERO, rotation);
    let normal = rotation * Vec3::X;
    let h = hit(normal, -normal * 2.0, 0.02, &[p]).unwrap();
    assert_eq!(h.kind, ImpactKind::Armor);
    assert!(h.normal.dot(normal) > 0.99);
}

#[test]
fn environment_cache_observes_pose_shape_layer_and_removal_changes() {
    let mut app = app();
    app.world_mut()
        .resource_mut::<SimulationConfig>()
        .training
        .hit_debug = true;
    let blocker = app
        .world_mut()
        .spawn((
            Collider::cuboid(0.01, 0.5, 0.5),
            Position(Vec3::ZERO),
            Rotation(Quat::IDENTITY),
            GameLayer::environment_collision_layers(),
        ))
        .id();
    app.finish();
    app.cleanup();
    app.update();
    app.update();
    fn fire(app: &mut App, id: u64) -> ImpactKind {
        let born = app.world().resource::<Time<Fixed>>().elapsed_secs_f64();
        app.world_mut().spawn((
            BallisticProjectile {
                id,
                position: Vec3::X,
                velocity: Vec3::NEG_X * 25.0,
                radius: 0.0085,
                age: 0.0,
                born_at: born,
                lifetime: 0.08,
                shot: None,
                shadow: false,
                resolved: false,
            },
            Transform::from_translation(Vec3::X),
        ));
        for _ in 0..8 {
            app.update();
        }
        let impact = app.world().resource::<FlightTrace>().hits.back().unwrap();
        assert_eq!(impact.shot_id, id);
        impact.kind
    }
    assert_eq!(fire(&mut app, 1), ImpactKind::Environment);
    app.world_mut()
        .entity_mut(blocker)
        .insert(Position(Vec3::Y * 10.0));
    assert_eq!(fire(&mut app, 2), ImpactKind::Expired);
    app.world_mut()
        .entity_mut(blocker)
        .insert(Collider::cuboid(0.01, 40.0, 40.0));
    assert_eq!(fire(&mut app, 3), ImpactKind::Environment);
    app.world_mut()
        .entity_mut(blocker)
        .insert(Rotation(Quat::from_rotation_z(std::f32::consts::FRAC_PI_2)));
    assert_eq!(fire(&mut app, 4), ImpactKind::Expired);
    app.world_mut()
        .entity_mut(blocker)
        .insert(Rotation(Quat::IDENTITY));
    assert_eq!(fire(&mut app, 5), ImpactKind::Environment);
    app.world_mut()
        .entity_mut(blocker)
        .insert(GameLayer::vehicle_body_collision_layers(false));
    assert_eq!(fire(&mut app, 6), ImpactKind::Expired);
    app.world_mut()
        .entity_mut(blocker)
        .insert(GameLayer::environment_collision_layers());
    assert_eq!(fire(&mut app, 7), ImpactKind::Environment);
    app.world_mut().entity_mut(blocker).despawn();
    assert_eq!(fire(&mut app, 8), ImpactKind::Expired);
    assert_eq!(app.world().resource::<BallisticDiagnostics>().failures, 0);
}

/// Differential oracle: retain the former wide-prediction verification only
/// in tests, to check the faster BVH queries against unchanged mesh geometry.
fn wide_prediction_reference(
    p: Vec3,
    delta: Vec3,
    dt: f32,
    object: &MovingShape,
) -> Option<(f32, Vec3)> {
    let ball = Ball::new(0.0085);
    let motion =
        NonlinearRigidMotion::new(pose(p, Quat::IDENTITY), Vec3::ZERO, delta / dt, Vec3::ZERO);
    let cast = cast_interval(&motion, &ball, object, 0.0, dt, 4).unwrap()?;
    let mut toi = cast.time_of_impact;
    let speed = (motion.linvel - object.motion.linvel).length()
        + object.motion.angvel.length() * (object.bound + object.motion.local_center.length());
    for _ in 0..128 {
        let bp = motion.position_at_time(toi);
        let op = object.motion.position_at_time(toi);
        let prediction =
            (bp.translation - op.translation).length() + object.bound * 2.0 + 0.0085 + 1.0;
        let c = query::contact(&bp, &ball, &op, &*object.shape, prediction)
            .unwrap()
            .unwrap();
        if c.dist <= GEOMETRY_EPS {
            return Some((toi, c.point2));
        }
        if speed <= f32::EPSILON {
            return None;
        }
        toi += (c.dist / speed * 0.9).max(1e-8);
        if toi > dt {
            return None;
        }
    }
    panic!("reference did not converge")
}

#[test]
fn narrow_contact_queries_match_wide_reference_on_real_meshes() {
    let mut app = app();
    load_vehicle(
        &mut app,
        TrainingTarget {
            kind: 0,
            center: Vec3::ZERO,
        },
    );
    app.finish();
    app.cleanup();
    app.update();
    app.update();
    let geometries: Vec<_> = app
        .world()
        .resource::<HitGeometry>()
        .0
        .iter()
        .map(|(e, g)| (*e, g.clone()))
        .collect();
    let mut comparisons = 0;
    let mut hits = 0;
    // Includes the dense chassis and backing meshes which caused the hotspot.
    for (id, g) in geometries {
        let center = g.shape.compute_local_aabb().center();
        let normal = g.front.unwrap_or(Vec3::X);
        let object = MovingShape {
            id,
            shape: g.shape,
            motion: NonlinearRigidMotion::new(
                pose(Vec3::ZERO, Quat::IDENTITY),
                Vec3::ZERO,
                Vec3::ZERO,
                Vec3::ZERO,
            ),
            front: g.front,
            kind: g.kind,
            target: Some(0),
            bound: g.bound,
        };
        for offset in [-0.03, 0.0, 0.03] {
            let p = center + normal * 0.5 + Vec3::Y * offset;
            let delta = -normal;
            let old = wide_prediction_reference(p, delta, 0.04, &object);
            let new = hit(p, delta, 0.04, std::slice::from_ref(&object));
            assert_eq!(
                old.is_some(),
                new.is_some(),
                "collider={id:?} offset={offset}"
            );
            if let (Some((time, point)), Some(new)) = (old, new) {
                assert!(
                    (time - new.toi).abs() < 0.00002,
                    "collider={id:?} kind={:?} offset={offset} old={time} new={} point_old={point:?} point_new={:?} p={p:?} delta={delta:?}",
                    object.kind,
                    new.toi,
                    new.point
                );
                assert!(point.distance(new.point) < 0.0004, "collider={id:?}");
                hits += 1;
            }
            comparisons += 1;
        }
    }
    assert_eq!(comparisons, 30);
    assert!(hits >= 12);
}

#[test]
fn zero_area_cad_faces_cannot_create_distant_penetration() {
    // Actual collinear chassis face that the wide legacy query projected onto
    // the sphere center 20 cm away, incorrectly reporting dist=-sphere_radius.
    let vertices = vec![
        Vec3::new(0.04830224, -0.074864715, -0.15339227),
        Vec3::new(0.04830224, -0.07376471, -0.15339227),
        Vec3::new(0.04830224, -0.062864736, -0.15339227),
    ];
    assert!(!valid_surface_triangle(&vertices, [0, 1, 2]));
    let mut app = app();
    load_vehicle(
        &mut app,
        TrainingTarget {
            kind: 0,
            center: Vec3::ZERO,
        },
    );
    app.finish();
    app.cleanup();
    app.update();
    app.update();
    for geometry in app.world().resource::<HitGeometry>().0.values() {
        for triangle in geometry.shape.as_trimesh().unwrap().triangles() {
            assert!(
                (triangle.b - triangle.a)
                    .cross(triangle.c - triangle.a)
                    .length_squared()
                    > 0.0
            );
        }
    }
}

/// Independent analytic oracle: a sphere crossing a rectangular plate along its
/// normal contacts it iff its YZ distance to the rectangle is below its radius.
/// No Parry query is used to determine the expected result.
fn classification_matrix(segmented: bool) {
    let radius = 0.0085_f32;
    let mut tp = 0_u64;
    let mut tn = 0_u64;
    let mut fp = 0_u64;
    let mut fn_count = 0_u64;
    let mut failures = 0_u64;
    let mut excluded_boundary = 0_u64;
    let mut examples = Vec::new();
    let mut missed_reasons = std::collections::BTreeMap::<String,u64>::new();
    for angle in [0.0_f32, 35.0, 70.0] {
        let rotation = Quat::from_rotation_z(angle.to_radians());
        for speed in [10.0_f32, 25.0, 50.0, 100.0] {
            for translating in [false, true] {
                let dt = 2.0 / speed;
                let target_velocity = if translating { rotation * Vec3::X * 2.0 } else { Vec3::ZERO };
                let mut target = plate(1, 0.0);
                target.motion.start = pose(Vec3::ZERO, rotation);
                target.motion.linvel = target_velocity;
                for yi in -20_i32..=20 {
                    for zi in -20_i32..=20 {
                        let y = yi as f32 * 0.004;
                        let z = zi as f32 * 0.004;
                        let distance = Vec2::new((y.abs()-0.0675).max(0.0), (z.abs()-0.0625).max(0.0)).length();
                        // Exact tangency falls inside the detector's documented
                        // 0.1 mm tolerance; keep it out of binary error rates.
                        if (distance-radius).abs() <= GEOMETRY_EPS {
                            excluded_boundary += 1;
                            continue;
                        }
                        let expected = distance < radius;
                        let start = rotation * Vec3::new(1.0,y,z);
                        let delta = rotation * Vec3::NEG_X * 2.0 + target_velocity * dt;
                        let steps = if segmented { (dt * 960.0).ceil() as usize } else { 1 };
                        let mut result = Ok(None);
                        for index in 0..steps {
                            let fraction = index as f32 / steps as f32;
                            let mut moving = target.clone();
                            moving.motion.start = pose(target_velocity * dt * fraction, rotation);
                            result = first_hit(start + delta * fraction,delta / steps as f32,radius,dt / steps as f32,&[moving],&mut BallisticDiagnostics::default());
                            if !matches!(&result, Ok(None)) { break; }
                        }
                        let detail = format!("{:?}", result.as_ref().map(|v|v.as_ref().map(|h|(h.kind,h.toi))));
                        let actual = match result {
                            Ok(value) => value.is_some_and(|h| h.kind == ImpactKind::Armor),
                            Err(()) => { failures += 1; false }
                        };
                        match (expected,actual) {
                            (true,true)=>tp+=1, (false,false)=>tn+=1,
                            (false,true)=>fp+=1, (true,false)=>fn_count+=1,
                        }
                        if expected && !actual { *missed_reasons.entry(detail.clone()).or_default() += 1; }
                        if expected != actual && examples.len()<12 {
                            examples.push(format!("angle={angle},speed={speed},moving={translating},y={y},z={z},distance={distance},expected={expected},actual={actual},result={detail}"));
                        }
                    }
                }
            }
        }
    }
    let report=serde_json::json!({"segmented":segmented,"oracle":"analytic sphere vs rectangular plate normal crossing", "true_positive":tp,"true_negative":tn,"false_positive":fp,"false_negative":fn_count,"query_failures":failures,"missed_reasons":missed_reasons,"excluded_tolerance_boundary":excluded_boundary,"examples":examples});
    println!("FALSE_CLASSIFICATION_MATRIX {report}");
    if let Ok(path)=std::env::var("HIT_CLASSIFICATION_REPORT") {
        std::fs::write(format!("{path}-{}.json",if segmented {"segmented"} else {"long"}),serde_json::to_string_pretty(&report).unwrap()).unwrap();
    }
    assert_eq!((fp,fn_count,failures),(0,0,0),"{report}");
}

#[test]
fn analytic_false_positive_negative_matrix_long_sweep() { classification_matrix(false); }
#[test]
fn analytic_false_positive_negative_matrix_production_segments() { classification_matrix(true); }
