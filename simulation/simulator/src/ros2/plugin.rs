use crate::arc_mutex;
use crate::capture::CaptureSource;
use crate::capture::driver::{CaptureConfig, CapturedFrameKind};
use crate::components::{
    Controlled, InfantryChassis, InfantryGimbal, InfantryLaunchOffset, SubscribeAutoAim,
};
use crate::config::SimulationConfig;
use crate::robomaster::prelude::{
    ArmorRoot, PowerRune, PowerRuneMechanism, RuneHit, RuneIndex, TechCore, tech_core_state_json,
};
use crate::ros2::capture::{RosCaptureContext, RosCapturePlugin};
use crate::ros2::livox::{RosLivoxContext, RosLivoxPlugin};
use crate::ros2::prelude::AverageRateLimiter;
use crate::ros2::prelude::transform;
use crate::ros2::topic::*;
use crate::systems::{GameplaySystems, projectile_launch};
use crate::util::entity_query::HierarchyQuery;
use bevy::ecs::system::RunSystemOnce;
use bevy::prelude::*;
use bevy::render::render_resource::TextureFormat;
use r2r::ClockType::SystemTime;
use r2r::geometry_msgs::msg::{Point, Pose, Vector3};
use r2r::std_msgs::msg::{ColorRGBA, String as RosString};
use r2r::visualization_msgs::msg::Marker;
use r2r::{Clock, Context, Node, std_msgs::msg::Header, tf2_msgs::msg::TFMessage};
use std::collections::HashMap;
use std::f32::consts::PI;
use std::time::Duration;
use std::{
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, AtomicU64, Ordering},
    },
    thread::{self, JoinHandle},
};

macro_rules! res_unwrap {
    ($res:tt) => {
        $res.0.lock().unwrap()
    };
}

#[derive(Resource, Deref, DerefMut)]
struct StopSignal(Arc<AtomicBool>);

#[derive(Resource, Deref, DerefMut)]
struct SpinThreadHandle(Option<JoinHandle<()>>);

#[derive(Resource, Deref, DerefMut)]
pub struct RoboMasterClock(pub Arc<Mutex<Clock>>);

#[derive(Resource, Deref, DerefMut)]
struct FireRateLimiter(AverageRateLimiter);

#[derive(Resource, Deref, DerefMut)]
struct TechCoreStateRateLimiter(AverageRateLimiter);

#[derive(Resource, Deref, DerefMut)]
struct PowerRuneStateRateLimiter(AverageRateLimiter);

fn publish_power_rune_hit(
    ev: On<RuneHit>,
    runes: Query<(&PowerRune, &PowerRuneMechanism)>,
    publisher: Res<TopicPublisher<PowerRuneHitTopic>>,
) {
    let Ok((rune, mechanism)) = runes.get(ev.rune) else {
        return;
    };
    publisher.publish(RosString {
        data: format!(
            "{{\"team\":\"{:?}\",\"mode\":\"{:?}\",\"outcome\":\"{:?}\",\"progress\":{}}}",
            rune.team(),
            rune.mode(),
            ev.result.outcome,
            mechanism
                .state()
                .large_progress()
                .map_or("null".to_string(), |value| value.to_string())
        ),
    });
}

fn publish_power_rune_state(
    time: Res<Time>,
    mut limiter: ResMut<PowerRuneStateRateLimiter>,
    runes: Query<(&PowerRune, &PowerRuneMechanism)>,
    publisher: Res<TopicPublisher<PowerRuneStateTopic>>,
) {
    limiter.tick(time.delta());
    if !limiter.allow() {
        return;
    }

    for (rune, mechanism) in &runes {
        let state = mechanism.state();
        publisher.publish(RosString {
            data: format!(
                "{{\"team\":\"{:?}\",\"mode\":\"{:?}\",\"phase\":\"{}\",\"primary_remaining\":{},\"activation_remaining\":{},\"progress\":{}}}",
                rune.team(),
                rune.mode(),
                state.phase_name(),
                state
                    .primary_remaining()
                    .map_or("null".to_string(), |value| format!("{value:.4}")),
                state
                    .activation_remaining()
                    .map_or("null".to_string(), |value| format!("{value:.4}")),
                state
                    .large_progress()
                    .map_or("null".to_string(), |value| value.to_string())
            ),
        });
    }
}

macro_rules! tf_tree {
    (stamp: $stamp:expr;$root:literal { $($content:tt)* }) => {{
        let stamp = $stamp;
        let mut transform_stamped = vec![];
        let _parent = $root;
        let _current = $root;
        tf_tree!(@frame transform_stamped, stamp, _parent, _current, $($content)*);

        transform_stamped
    }};

    (@header $stamp:ident, $current:ident) => {
        Header {
            stamp: $stamp.clone(),
            frame_id: $current.to_string(),
        }
    };

    (@frame $tf_vec:ident, $stamp:ident, $parent:ident, $current:ident,
        $curr_name:literal as ($translation:expr, $rotation:expr) $(for $pub_:ident)?
        {$($children:tt)*}
        $($remaining:tt)*
    ) => {
        {
            let $parent = &$current;
            let $current = $curr_name;
            $crate::add_tf_frame!($tf_vec, tf_tree!(@header $stamp, $parent), $current, $translation, $rotation);
            $(
                $pub_.publish($crate::pose!(tf_tree!(@header $stamp, $current)));
            )*
            tf_tree!(@frame $tf_vec, $stamp, $parent, $current, $($children)*);
        }
        tf_tree!(@frame $tf_vec, $stamp, $parent, $current, $($remaining)*);
    };

    (@frame $tf_vec:ident, $stamp:ident, $parent:ident, $current:ident,
    $(let $p_name:ident = $p_expr:expr;)*
        for ($($elem:tt),+$(,)?) in $iter:ident {
            $(let $name:ident = $expr:expr;)*
            pub $curr_name:ident as ($translation:expr, $rotation:expr) $(for $pub_:ident)?;
            $($children:tt)*
        }
        $($remaining:tt)*
    ) => {
        $(let $p_name = $p_expr;)*
        for ($($elem),+) in $iter {
            $(let $name = $expr;)*
            let $parent = &$current;
            let $current = $curr_name;
            $crate::add_tf_frame!($tf_vec, tf_tree!(@header $stamp, $parent), $current, $translation, $rotation);
            $(
                $pub_.publish($crate::pose!(tf_tree!(@header $stamp, $current)));
            )*
            tf_tree!(@frame $tf_vec, $stamp, $parent, $current, $($children)*);
        }
        tf_tree!(@frame $tf_vec, $stamp, $parent, $current, $($remaining)*);
    };

    (@frame $tf_vec:ident, $stamp:ident, $parent:ident, $current:ident, $(;)? $(,)? $({})?) => { };
}

fn capture_rune(
    camera: Single<&GlobalTransform, With<CaptureSource>>,
    gimbal: Single<&GlobalTransform, (With<Controlled>, With<InfantryGimbal>)>,
    muzzle_offset: Single<
        (&GlobalTransform, &Transform),
        (With<InfantryLaunchOffset>, With<Controlled>),
    >,

    runes: Query<(Entity, &GlobalTransform, &PowerRune)>,
    targets: Query<(&GlobalTransform, &RuneIndex, &Name)>,

    clock_and_config: (ResMut<RoboMasterClock>, Res<SimulationConfig>),
    tf_publisher: ResMut<TopicPublisher<GlobalTransformTopic>>,
    gimbal_pose_pub: ResMut<TopicPublisher<GimbalPoseTopic>>,
    odom_pose_pub: ResMut<TopicPublisher<OdomPoseTopic>>,
    muzzle_pose_pub: ResMut<TopicPublisher<MuzzlePoseTopic>>,
    camera_pose_pub: ResMut<TopicPublisher<CameraPoseTopic>>,
    vision_recv_pub: ResMut<TopicPublisher<VisionRecvDataTopic>>,
    center: Query<(Entity, &GlobalTransform)>,
    qq: HierarchyQuery,
    armor: Query<(Entity, &GlobalTransform, &ArmorRoot)>,
    marker_pub: ResMut<TopicPublisher<OutpostMarkerTopic>>,
) {
    let (clock, sim_config) = clock_and_config;
    let cam_transform = camera.into_inner();
    let gimbal = gimbal.into_inner();
    let cam_rel = cam_transform.reparented_to(gimbal);
    let muzzle_rel = muzzle_offset.0.reparented_to(gimbal);

    // Avoid per-frame `info!` logging (it will cap FPS under ROS2 builds).
    debug!(
        "[ROS2] ODOM pos: [{:.4}, {:.4}, {:.4}]",
        gimbal.translation().x,
        gimbal.translation().y,
        gimbal.translation().z
    );
    debug!(
        "[ROS2] CAMERA_REL pos: [{:.4}, {:.4}, {:.4}]",
        cam_rel.translation.x, cam_rel.translation.y, cam_rel.translation.z
    );
    let mut targets = targets.into_iter().fold(
        HashMap::<Entity, Vec<(String, Transform)>>::new(),
        |mut map, (tf, target, name)| {
            // only use one target
            if !name.contains("_ACTIVATED") {
                return map;
            }
            let Ok((rune_entity, rune_tf, rune)) = runes.get(target.rune) else {
                return map;
            };
            map.entry(rune_entity).or_default().push((
                format!("power_rune_{:?}_{:?}", rune.mode(), target.target)
                    .to_string()
                    .to_lowercase(),
                tf.reparented_to(rune_tf),
            ));
            map
        },
    );

    debug!(
        "[ROS2] MUZZLE pos: [{:.4}, {:.4}, {:.4}]",
        muzzle_rel.translation.x, muzzle_rel.translation.y, muzzle_rel.translation.z
    );
    let rot = gimbal.rotation().to_euler(EulerRot::YXZ);
    debug!(
        "[ROS2] GIMBAL YXZ: [{:.4}, {:.4}, {:.4}]",
        rot.0.to_degrees(),
        rot.1.to_degrees(),
        rot.2.to_degrees()
    );
    let stamp = Clock::to_builtin_time(&res_unwrap!(clock).get_now().unwrap());

    // YXZ: rot.0=Y=yaw, rot.1=X=pitch, rot.2=Z=roll
    let yaw_deg = rot.0.to_degrees();
    let pitch_deg = -rot.1.to_degrees();
    let roll_deg = rot.2.to_degrees();

    vision_recv_pub.publish(r2r::hnurm_interfaces::msg::VisionRecvData {
        header: Header {
            stamp: stamp.clone(),
            frame_id: "gimbal_link".to_string(),
        },
        self_color: r2r::hnurm_interfaces::msg::SelfColor { data: 1 },
        work_mode: r2r::hnurm_interfaces::msg::WorkMode { data: 0 },
        bullet_speed: r2r::hnurm_interfaces::msg::BulletSpeed { data: sim_config.projectile.speed.round().clamp(0.0, 255.0) as u8 },
        roll: roll_deg,
        pitch: pitch_deg,
        yaw: yaw_deg,
        vel_x: 0.0,
        vel_y: 0.0,
        vel_yaw: 0.0,
        control_id: 0.0,
        game_progress: 0.0,
        stage_remain_time: 0.0,
        current_hp: 0.0,
        current_enemy_sentry_hp: 0.0,
        current_enemy_base_hp: 0.0,
        current_virtual_shield: 0.0,
        current_base_hp: 0.0,
    });

    let transform_stamped = tf_tree! {
        stamp: stamp;

        "map" {
            "odom" as (gimbal.translation(), Quat::IDENTITY) for odom_pose_pub {
                "gimbal_link" as (Vec3::ZERO, gimbal.rotation() * muzzle_offset.1.rotation * Quat::from_euler(EulerRot::ZYX, 0.0, 0.0, PI / 2.0)) for gimbal_pose_pub {
                    "muzzle" as (muzzle_rel.translation, Quat::IDENTITY) {
                        "muzzle_link" as (Vec3::ZERO, Quat::IDENTITY) for muzzle_pose_pub{}
                    }
                    "camera_link" as (cam_rel.translation, Quat::IDENTITY) for camera_pose_pub {
                        "camera_optical_frame" as (Vec3::ZERO, Quat::from_euler(EulerRot::ZYX, -PI / 2.0, PI, PI / 2.0)) {}
                    }
                }
            }
            for (rune_entity, transform, rune) in runes {
                let name = format!("power_rune_{:?}", rune.mode()).to_string().to_lowercase();
                let tf = transform.compute_transform();
                pub name as (tf.translation, tf.rotation);
                let targets = targets.remove(&rune_entity).unwrap_or_default();
                for (name, tf) in targets {
                    pub name as (tf.translation, tf.rotation);
                }
            }
            for (entity, _transform, armor) in armor {
                let name = format!("armor_{:?}", armor.id.as_usize())
                    .to_string()
                    .to_lowercase();
                let tf = qq
                    .of(entity)
                    .suffix("CENTER")
                    .any()
                    .one()
                    .and_then(|center_entity| center.get(center_entity).ok())
                    .map(|(_, transform)| transform.compute_transform())
                    .unwrap_or_else(|| _transform.compute_transform());
                pub name as (tf.translation, tf.rotation);
            }
        }
    };

    let stamp = Clock::to_builtin_time(&res_unwrap!(clock).get_now().unwrap());
    for (entity, tf, armor) in armor {
        let mut tff = qq
            .of(entity)
            .suffix("CENTER")
            .any()
            .one()
            .and_then(|center_entity| center.get(center_entity).ok())
            .map(|(_, transform)| transform.compute_transform())
            .unwrap_or_else(|| tf.compute_transform());
        tff.rotation = tf.rotation() * Quat::from_euler(EulerRot::ZYX, 0.0, 0.0, -PI / 2.0);
        let tf = transform(tff);
        marker_pub.publish(Marker {
            header: Header {
                stamp: stamp.clone(),
                frame_id: "map".to_string(),
            }
            .clone(),
            ns: "armors".to_string(),
            id: armor.id.as_usize() as i32,
            type_: Marker::CUBE as i32,
            action: Marker::ADD as i32,
            pose: Pose {
                position: Point {
                    x: tf.translation.x,
                    y: tf.translation.y,
                    z: tf.translation.z,
                },
                orientation: tf.rotation,
            },
            scale: Vector3 {
                x: 0.03,
                y: 0.15,
                z: 0.125,
            },
            color: ColorRGBA {
                r: 0.0,
                g: 1.0,
                b: 0.0,
                a: 0.0,
            },
            lifetime: r2r::builtin_interfaces::msg::Duration {
                sec: 0,
                nanosec: 300000000,
            },
            frame_locked: false,
            points: vec![],
            colors: vec![],
            texture_resource: "".to_string(),
            texture: Default::default(),
            uv_coordinates: vec![],
            text: "".to_string(),
            mesh_resource: "".to_string(),
            mesh_file: Default::default(),
            mesh_use_embedded_materials: false,
        });
    }

    tf_publisher.publish(TFMessage {
        transforms: transform_stamped,
    });
}

fn process_subscription(
    time: Res<Time>,
    sim_config: Res<SimulationConfig>,
    mut commands: Commands,
    gimbal_cmd: ResMut<TopicSubscriber<VisionSendDataTopic>>,
    mut fire_rate_limiter: ResMut<FireRateLimiter>,
    gimbal: Single<
        (&mut Transform, &mut InfantryGimbal),
        (
            With<Controlled>,
            Without<InfantryChassis>,
            Without<InfantryLaunchOffset>,
        ),
    >,
    chassis: Single<&GlobalTransform, (With<Controlled>, With<InfantryChassis>)>,
    mut target_angles: Local<Option<Vec2>>,
) {
    let (mut gimbal_transform, mut gimbal_data) = gimbal.into_inner();
    fire_rate_limiter.tick(time.delta());

    // TopicSubscriber 只保存最新消息。死区仅决定是否接受新的目标角，
    // 不能直接结束系统，否则限速云台只移动一帧便停止。
    if let Ok(Some(cmd)) = gimbal_cmd.try_recv() {
        if cmd.target_state.data == 0 || cmd.target_distance == -1.0 {
            *target_angles = None;
            return;
        }

        // The real bridge publishes FIRE as a held level while the shooter permits firing,
        // rather than as one pulse per projectile. Every newly received FIRE message may
        // therefore launch, bounded here at 10 Hz and again by ProjectileCooldown. Because
        // this runs only after try_recv() returns a fresh message, a disconnected publisher
        // cannot keep firing from stale state.
        if fire_rate_limiter.allow_if(cmd.target_state.data == 2) {
            commands.queue(|w: &mut World| {
                w.run_system_once(projectile_launch).unwrap();
            });
        }

        // send 使用 RM 约定；Bevy 的 pitch 正方向相反。
        let yaw = (cmd.yaw as f32).to_radians();
        let pitch = (-(cmd.pitch as f32).to_radians()).clamp(
            -sim_config.vehicle.gimbal_pitch_limit,
            sim_config.vehicle.gimbal_pitch_limit,
        );
        let deadband = sim_config.vehicle.gimbal_command_deadband_deg.to_radians();

        let should_update = match *target_angles {
            None => true,
            Some(last) => {
                // yaw 使用最短圆周角差，避免 179.99/-179.99 被误判为 360 度跳变。
                let yaw_delta = (yaw - last.x + PI).rem_euclid(2.0 * PI) - PI;
                yaw_delta.abs() > deadband || (pitch - last.y).abs() > deadband
            }
        };

        if should_update {
            *target_angles = Some(Vec2::new(yaw, pitch));
        }
    }

    let Some(target) = *target_angles else {
        return;
    };

    // send 是世界姿态命令；转换成云台相对底盘的局部姿态。
    let global_target = Quat::from_euler(EulerRot::YXZ, target.x, target.y, 0.0);
    let local_target = chassis.rotation().inverse() * global_target;

    // 按最大角速度、四元数最短弧逼近目标，避免延迟闭环瞬移和 +/-180 度绕远路。
    let current_rotation = gimbal_transform.rotation.normalize();
    let mut target_rotation = local_target.normalize();
    if current_rotation.dot(target_rotation) < 0.0 {
        target_rotation = -target_rotation;
    }
    let dot = current_rotation.dot(target_rotation).clamp(0.0, 1.0);
    let angle_error = 2.0 * dot.acos();
    let max_step = sim_config.vehicle.gimbal_rotation_speed * time.delta_secs();
    let interpolation = if angle_error <= 1e-5 {
        1.0
    } else {
        (max_step / angle_error).clamp(0.0, 1.0)
    };

    gimbal_transform.rotation = current_rotation.slerp(target_rotation, interpolation);
    (gimbal_data.local_yaw, gimbal_data.pitch, _) =
        gimbal_transform.rotation.to_euler(EulerRot::YXZ);
}

fn publish_tech_core_state(
    time: Res<Time>,
    clock: Res<RoboMasterClock>,
    mut limiter: ResMut<TechCoreStateRateLimiter>,
    cores: Query<&TechCore>,
    state_pub: Res<TopicPublisher<TechCoreStateTopic>>,
) {
    limiter.tick(time.delta());
    if !limiter.allow() {
        return;
    }

    let stamp = Clock::to_builtin_time(&res_unwrap!(clock).get_now().unwrap());
    state_pub.publish(RosString {
        data: tech_core_state_json(
            stamp.sec,
            stamp.nanosec,
            time.elapsed_secs_f64(),
            cores.iter(),
        ),
    });
}

fn cleanup_ros2_system(
    mut exit: MessageReader<AppExit>,
    stop_signal: Res<StopSignal>,
    mut handle_res: ResMut<SpinThreadHandle>,
) {
    if exit.read().len() > 0 {
        stop_signal.store(true, Ordering::Release);
        if let Some(handle) = handle_res.take() {
            info!("Waiting for ROS 2 spin thread to join...");
            match handle.join() {
                Ok(_) => info!("ROS 2 thread successfully joined. Safe to exit."),
                Err(_) => error!("WARNING: ROS 2 thread panicked or failed to join."),
            }
        }
    }
}

#[derive(Default)]
pub struct ROS2Plugin {}

impl Plugin for ROS2Plugin {
    fn build(&self, app: &mut App) {
        let sim_config = app
            .world()
            .get_resource::<SimulationConfig>()
            .cloned()
            .unwrap_or_default();
        let mut node = Node::create(Context::create().unwrap(), "simulator", "robomaster").unwrap();
        let signal_arc = Arc::new(AtomicBool::new(false));

        register_pub(signal_arc.clone(), app, &mut node);
        register_sub(signal_arc.clone(), app, &mut node);

        let camera_info = app
            .world_mut()
            .remove_resource::<TopicPublisher<CameraInfoTopic>>()
            .unwrap();
        let image_raw = app
            .world_mut()
            .remove_resource::<TopicPublisher<ImageRawTopic>>()
            .unwrap();
        let image_compressed = app
            .world_mut()
            .remove_resource::<TopicPublisher<ImageCompressedTopic>>()
            .unwrap();
        let livox_pointcloud = app
            .world_mut()
            .remove_resource::<TopicPublisher<LivoxPointCloudTopic>>()
            .unwrap();

        let clock = arc_mutex!(Clock::create(SystemTime).unwrap());
        let fov_y = sim_config.camera.fov.to_radians();
        let color_capture_config = CaptureConfig {
            width: sim_config.capture.color.width,
            height: sim_config.capture.color.height,
            texture_format: TextureFormat::Rgba8UnormSrgb,
            frame_kind: CapturedFrameKind::Rgb8,
        };
        let depth_capture_config = CaptureConfig {
            width: sim_config.capture.depth.width,
            height: sim_config.capture.depth.height,
            texture_format: TextureFormat::Depth32Float,
            frame_kind: CapturedFrameKind::Depth32F,
        };
        let publish_freq = sim_config.livox_ros.publish_freq.max(0.1);
        let points_per_publish =
            ((sim_config.livox_ros.points_per_second as f32) / publish_freq).max(1.0) as usize;

        app.insert_resource(RoboMasterClock(clock.clone()))
            .insert_resource(StopSignal(signal_arc.clone()))
            .insert_resource(FireRateLimiter(AverageRateLimiter::from_hz(10.0)))
            .insert_resource(TechCoreStateRateLimiter(AverageRateLimiter::from_hz(20.0)))
            .insert_resource(PowerRuneStateRateLimiter(AverageRateLimiter::from_hz(30.0)))
            .add_observer(publish_power_rune_hit)
            .add_plugins(RosCapturePlugin {
                config: color_capture_config,
                context: RosCaptureContext {
                    clock: clock.clone(),
                    fov_y,
                    publish_compressed: false,
                    noise_sigma: sim_config.vision.noise_sigma,
                    camera_info,
                    image_raw,
                    image_compressed,
                },
            })
            .add_systems(Last, cleanup_ros2_system)
            .add_systems(
                Update,
                process_subscription
                    .after(GameplaySystems::Input)
                    .run_if(|enabled: Res<SubscribeAutoAim>| enabled.load(Ordering::Acquire)),
            )
            .add_systems(Update, capture_rune)
            .add_systems(Update, publish_tech_core_state)
            .add_systems(Update, publish_power_rune_state)
            .insert_resource(SpinThreadHandle(Some(thread::spawn(move || {
                while !signal_arc.load(Ordering::Acquire) {
                    node.spin_once(Duration::from_millis(1));
                }
            }))));

        if sim_config.livox_ros.enabled {
            app.add_plugins(RosLivoxPlugin {
                config: depth_capture_config,
                context: RosLivoxContext {
                    clock,
                    frame_id: sim_config.livox_ros.frame_id.clone(),
                    fov_y,
                    near: sim_config.capture.depth.near,
                    far: sim_config.capture.depth.far,
                    publish_period_ns: (1_000_000_000.0 / publish_freq) as u64,
                    points_per_publish,
                    line_num: sim_config.livox_ros.line_num.max(1),
                    tag_default: sim_config.livox_ros.tag_default,
                    intensity_default: sim_config.livox_ros.intensity_default,
                    pointcloud: livox_pointcloud,
                    last_publish_ns: Arc::new(AtomicU64::new(0)),
                },
            });
        }
    }
}
