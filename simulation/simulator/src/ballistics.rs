//! Training bullets use finite-radius continuous queries, never solver contact events.
use crate::{
    components::GameLayer,
    config::{ProjectileConfig, SimulationConfig},
    score_model::Shot,
    scoreboard::Board,
    statistic::ProjectileStatistics,
    training::TrainingTarget,
};
use avian3d::{parry, prelude::*};
use bevy::{math::Affine3A, mesh::VertexAttributeValues, prelude::*};
use parry::{
    math::Pose,
    query::{self, NonlinearRigidMotion, ShapeCastOptions, ShapeCastStatus},
    shape::{Ball, SharedShape},
};
use serde::Deserialize;
use std::{
    collections::{HashMap, VecDeque},
    io::Write,
};

const GEOMETRY_EPS: f32 = 0.0001;
const PATH_ERROR: f32 = 0.0001;
const MAX_SEGMENTS: usize = 1024;
const GEOMETRY_VERSION: u32 = 2;

#[derive(Deserialize, Reflect, Clone, Copy, Debug, Default, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum HitDetectionMode {
    #[default]
    Swept,
    Shadow,
    Legacy,
}

#[derive(Component)]
pub struct BallisticProjectile {
    pub id: u64,
    pub position: Vec3,
    pub velocity: Vec3,
    pub radius: f32,
    pub age: f32,
    pub born_at: f64,
    pub lifetime: f32,
    pub shot: Option<Shot>,
    pub shadow: bool,
    pub resolved: bool,
}
#[derive(Resource, Default)]
pub struct ShotIds(pub u64);
#[derive(Component)]
pub struct LegacyShotId(pub u64);
impl ShotIds {
    pub fn next(&mut self) -> u64 {
        self.0 += 1;
        self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImpactKind {
    Armor,
    Body,
    Environment,
    InitialOverlap,
    Expired,
    QueryFailure,
}
#[derive(Message, Clone, Debug)]
pub struct ProjectileImpact {
    pub shot_id: u64,
    pub projectile: Entity,
    pub collider: Option<Entity>,
    pub target: Option<usize>,
    pub simulation_time: f64,
    pub point_world: Vec3,
    pub normal_world: Vec3,
    pub point_vehicle_local: Option<Vec3>,
    pub kind: ImpactKind,
    pub shadow: bool,
}
#[derive(Resource, Default, Debug)]
pub struct BallisticDiagnostics {
    pub profile_queries: bool,
    pub query_costs: HashMap<Entity, QueryCost>,
    pub objects_ms: f64,
    pub segments: u64,
    pub candidates: u64,
    pub queries: u64,
    pub failures: u64,
    pub impacts: u64,
    pub step_ms: VecDeque<f32>,
}
#[derive(Default, Debug)]
pub struct QueryCost {
    pub calls: u64,
    pub cast_ms: f64,
    pub verify_ms: f64,
}
impl BallisticDiagnostics {
    pub fn p95_ms(&self) -> f32 {
        let mut samples: Vec<_> = self.step_ms.iter().copied().collect();
        if samples.is_empty() {
            return 0.0;
        }
        samples.sort_by(f32::total_cmp);
        samples[((samples.len() as f32 * 0.95).ceil() as usize - 1).min(samples.len() - 1)]
    }
}
#[derive(Clone)]
struct Geometry {
    shape: SharedShape,
    front: Option<Vec3>,
    root: Entity,
    bound: f32,
    kind: ImpactKind,
}
#[derive(Resource, Default)]
struct HitGeometry(HashMap<Entity, Geometry>);
#[derive(Resource, Default)]
struct EnvironmentCache(HashMap<Entity, MovingShape>);

#[derive(Resource)]
pub struct FlightTrace {
    file: Option<std::fs::File>,
    trails: VecDeque<(f64, Vec3, Vec3, f32)>,
    hits: VecDeque<ProjectileImpact>,
}
impl Default for FlightTrace {
    fn default() -> Self {
        let file =
            std::env::var_os("TRAINING_HIT_LOG").and_then(|path| match std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(path)
            {
                Ok(file) => Some(file),
                Err(error) => {
                    warn!("Cannot open TRAINING_HIT_LOG: {error}");
                    None
                }
            });
        Self {
            file,
            trails: VecDeque::new(),
            hits: VecDeque::new(),
        }
    }
}
impl FlightTrace {
    pub fn enabled(&self) -> bool {
        self.file.is_some()
    }
    pub fn record(&mut self, value: serde_json::Value) {
        if let Some(file) = &mut self.file {
            if let Err(error) = writeln!(file, "{value}") {
                warn!("Hit trace write failed: {error}");
                self.file = None;
            }
        }
    }
}

pub struct BallisticsPlugin;
impl Plugin for BallisticsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<HitGeometry>()
            .init_resource::<EnvironmentCache>()
            .init_resource::<ShotIds>()
            .init_resource::<FlightTrace>()
            .init_resource::<BallisticDiagnostics>()
            .add_message::<ProjectileImpact>()
            .add_systems(
                FixedUpdate,
                (register_geometry, advance)
                    .chain()
                    .after(crate::training::move_targets),
            );
        app.world_mut()
            .resource_mut::<BallisticDiagnostics>()
            .profile_queries = std::env::var_os("TRAINING_HIT_PROFILE").is_some();
        if app.is_plugin_added::<bevy::gizmos::GizmoPlugin>() {
            app.add_systems(Update, draw_debug);
        }
    }
}

fn mesh_kind(name: &str) -> Option<ImpactKind> {
    if name.ends_with("_ARMOR") {
        Some(ImpactKind::Armor)
    } else if name.ends_with("_ARMOR_BASE") || matches!(name, "CHASSIS" | "GIMBAL_MAIN") {
        Some(ImpactKind::Body)
    } else {
        None
    }
}

fn valid_surface_triangle(vertices: &[Vec3], t: [u32; 3]) -> bool {
    let Some(a) = vertices.get(t[0] as usize) else {
        return false;
    };
    let Some(b) = vertices.get(t[1] as usize) else {
        return false;
    };
    let Some(c) = vertices.get(t[2] as usize) else {
        return false;
    };
    a.is_finite()
        && b.is_finite()
        && c.is_finite()
        && (*b - *a).cross(*c - *a).length_squared() > 0.0
}

fn register_geometry(
    config: Res<SimulationConfig>,
    meshes: Res<Assets<Mesh>>,
    nodes: Query<(Entity, &Mesh3d)>,
    names: Query<&Name>,
    hierarchy: Query<(&Transform, Option<&ChildOf>)>,
    targets: Query<(), With<TrainingTarget>>,
    mut registry: ResMut<HitGeometry>,
) {
    if !config.training.enabled {
        return;
    }
    registry.0.retain(|entity, _| nodes.contains(*entity));
    for (entity, handle) in &nodes {
        if registry.0.contains_key(&entity) {
            continue;
        }
        let mut label = None;
        let mut current = entity;
        let mut local = Affine3A::IDENTITY;
        let mut root = None;
        for _ in 0..64 {
            // Bevy's glTF loader may put the primitive Mesh3d on an unnamed
            // child of its named node. Resolve only an explicitly classified
            // node, never the broad Armor tag on arbitrary descendants.
            if label.is_none() {
                if let Ok(name) = names.get(current) {
                    if let Some(kind) = mesh_kind(name.as_str()) {
                        label = Some((kind, name.to_string()));
                    }
                }
            }
            if targets.contains(current) {
                root = Some(current);
                break;
            }
            let Ok((transform, parent)) = hierarchy.get(current) else {
                break;
            };
            local = transform.compute_affine() * local;
            let Some(parent) = parent else {
                break;
            };
            current = parent.parent();
        }
        let Some(root) = root else {
            continue;
        };
        let Some((kind, name)) = label else {
            continue;
        };
        let Some(mesh) = meshes.get(&handle.0) else {
            continue;
        };
        let Some(VertexAttributeValues::Float32x3(vertices)) =
            mesh.attribute(Mesh::ATTRIBUTE_POSITION)
        else {
            continue;
        };
        let vertices: Vec<Vec3> = vertices
            .iter()
            .map(|p| local.transform_point3(Vec3::from(*p)))
            .collect();
        let indices: Vec<u32> = mesh
            .indices()
            .map(|i| i.iter().map(|i| i as u32).collect())
            .unwrap_or_else(|| (0..vertices.len() as u32).collect());
        let triangles: Vec<[u32; 3]> = indices
            .chunks_exact(3)
            .map(|t| [t[0], t[1], t[2]])
            // CAD export contains collinear triangles. Parry's solid triangle
            // projection can return the query point itself for these faces,
            // producing a false penetration arbitrarily far from the surface.
            .filter(|t| valid_surface_triangle(&vertices, *t))
            .collect();
        let discarded = indices.len() / 3 - triangles.len();
        let bound = vertices.iter().map(|v| v.length()).fold(0.0, f32::max);
        let Ok(shape) = SharedShape::trimesh_with_flags(
            vertices,
            triangles,
            parry::shape::TriMeshFlags::MERGE_DUPLICATE_VERTICES,
        ) else {
            error!("Invalid hit geometry {name}");
            continue;
        };
        // Bake local hierarchy into vertices: rotation is around vehicle origin.
        registry.0.insert(
            entity,
            Geometry {
                shape,
                root,
                bound,
                kind,
                front: (kind == ImpactKind::Armor)
                    .then(|| local.transform_vector3(Vec3::X).normalize()),
            },
        );
        info!(
            "HIT_GEOMETRY entity={entity:?} name={name} kind={kind:?} bound={bound:.4} discarded_zero_area={discarded} version={GEOMETRY_VERSION}"
        );
    }
}

#[derive(Clone)]
pub(crate) struct MovingShape {
    pub id: Entity,
    pub shape: SharedShape,
    pub motion: NonlinearRigidMotion,
    pub front: Option<Vec3>,
    pub kind: ImpactKind,
    pub target: Option<usize>,
    pub bound: f32,
}
#[derive(Debug, Clone)]
pub(crate) struct Hit {
    pub toi: f32,
    pub collider: Entity,
    pub target: Option<usize>,
    pub kind: ImpactKind,
    pub point: Vec3,
    pub normal: Vec3,
    pub local: Vec3,
}
fn pose(position: Vec3, rotation: Quat) -> Pose {
    Pose::from_parts(position, rotation)
}

fn cast_interval(
    ball_motion: &NonlinearRigidMotion,
    ball: &Ball,
    object: &MovingShape,
    start: f32,
    end: f32,
    depth: u32,
) -> Result<Option<query::ShapeCastHit>, ()> {
    let result = if object.motion.angvel.length_squared() == 0.0 && start == 0.0 {
        query::cast_shapes(
            &ball_motion.start,
            ball_motion.linvel,
            ball,
            &object.motion.start,
            object.motion.linvel,
            &*object.shape,
            ShapeCastOptions {
                max_time_of_impact: end,
                target_distance: 0.0,
                compute_impact_geometry_on_penetration: true,
                ..default()
            },
        )
    } else {
        query::cast_shapes_nonlinear(
            ball_motion,
            ball,
            &object.motion,
            &*object.shape,
            start,
            end,
            true,
        )
    }
    .map_err(|_| ())?;
    if result.as_ref().is_some_and(|h| {
        matches!(
            h.status,
            ShapeCastStatus::Failed | ShapeCastStatus::OutOfIterations
        )
    }) {
        if depth == 0 {
            return Err(());
        }
        let mid = (start + end) * 0.5;
        if let Some(hit) = cast_interval(ball_motion, ball, object, start, mid, depth - 1)? {
            return Ok(Some(hit));
        }
        return cast_interval(ball_motion, ball, object, mid, end, depth - 1);
    }
    Ok(result)
}

pub(crate) fn first_hit(
    position: Vec3,
    displacement: Vec3,
    radius: f32,
    dt: f32,
    objects: &[MovingShape],
    diagnostics: &mut BallisticDiagnostics,
) -> Result<Option<Hit>, ()> {
    let ball = Ball::new(radius);
    let ball_motion = NonlinearRigidMotion::new(
        pose(position, Quat::IDENTITY),
        Vec3::ZERO,
        displacement / dt,
        Vec3::ZERO,
    );
    let mut best: Option<Hit> = None;
    for object in objects {
        if object.motion.angvel.length_squared() == 0.0 {
            // Exact conservative broad phase for a translating sphere. The
            // arena meshes span the whole map: a pivot-centered sphere alone
            // would send almost every bullet segment through their BVHs.
            let inverse = object.motion.start.inverse();
            let start = inverse * position;
            let delta = inverse.rotation * (displacement - object.motion.linvel * dt);
            let aabb = object.shape.compute_local_aabb();
            let padding = Vec3::splat(radius + GEOMETRY_EPS + PATH_ERROR);
            if !segment_aabb(start, delta, aabb.mins - padding, aabb.maxs + padding) {
                continue;
            }
        }
        // Spherical rotation envelope includes every intermediate orientation.
        let relative = position - object.motion.start * object.motion.local_center;
        let relative_delta = displacement - object.motion.linvel * dt;
        let alpha = if relative_delta.length_squared() > 0.0 {
            (-relative.dot(relative_delta) / relative_delta.length_squared()).clamp(0.0, 1.0)
        } else {
            0.0
        };
        if (relative + alpha * relative_delta).length() > object.bound + radius + GEOMETRY_EPS {
            continue;
        }
        diagnostics.candidates += 1;
        diagnostics.queries += 1;
        let cast_started = diagnostics.profile_queries.then(std::time::Instant::now);
        let cast = cast_interval(&ball_motion, &ball, object, 0.0, dt, 4);
        if let Some(started) = cast_started {
            let cost = diagnostics.query_costs.entry(object.id).or_default();
            cost.calls += 1;
            cost.cast_ms += started.elapsed().as_secs_f64() * 1000.0;
        }
        let Some(hit) = cast? else {
            continue;
        };
        let verify_started = diagnostics.profile_queries.then(std::time::Instant::now);
        if !hit.time_of_impact.is_finite() || hit.time_of_impact < 0.0 || hit.time_of_impact > dt {
            return Err(());
        }
        // A cast's Converged status alone is insufficient: GJK can stop while
        // the shapes are still separated. Conservatively advance using a bound
        // on relative surface speed until the measured gap is within tolerance.
        let speed_bound = (ball_motion.linvel - object.motion.linvel).length()
            + object.motion.angvel.length() * (object.bound + object.motion.local_center.length());
        let mut toi = hit.time_of_impact;
        let mut verified = None;
        for _ in 0..128 {
            let bp = ball_motion.position_at_time(toi);
            let op = object.motion.position_at_time(toi);
            // A large prediction expands the sphere's BVH query to encompass
            // almost the entire chassis mesh. Query only the contact tolerance;
            // when separated, use nearest distance to bound the contact search.
            let nearby =
                query::contact(&bp, &ball, &op, &*object.shape, GEOMETRY_EPS).map_err(|_| ())?;
            let c = if let Some(c) = nearby {
                c
            } else {
                let gap = query::distance(&bp, &ball, &op, &*object.shape).map_err(|_| ())?;
                if !gap.is_finite() {
                    return Err(());
                }
                // Distance is only a search-radius hint. It must not advance
                // time: on tessellated meshes its GJK tolerance can skip an
                // earlier edge contact. Advance only with verified contact data.
                let maximum =
                    (bp.translation - op.translation).length() + object.bound * 2.0 + radius + 1.0;
                let mut prediction = (gap + GEOMETRY_EPS).max(GEOMETRY_EPS * 2.0).min(maximum);
                let mut found = None;
                for _ in 0..16 {
                    found = query::contact(&bp, &ball, &op, &*object.shape, prediction)
                        .map_err(|_| ())?;
                    if found.is_some() {
                        break;
                    }
                    if prediction >= maximum {
                        break;
                    }
                    prediction = (prediction * 2.0).min(maximum);
                }
                found.ok_or(())?
            };
            if !c.dist.is_finite() || !c.point2.is_finite() || !c.normal2.is_finite() {
                return Err(());
            }
            if c.dist <= GEOMETRY_EPS {
                verified = Some((op, c));
                break;
            }
            if speed_bound <= f32::EPSILON {
                break;
            }
            toi += (c.dist / speed_bound * 0.9).max(1e-8);
            if toi > dt {
                break;
            }
        }
        let Some((op, contact)) = verified else {
            if let Some(started) = verify_started {
                diagnostics
                    .query_costs
                    .entry(object.id)
                    .or_default()
                    .verify_ms += started.elapsed().as_secs_f64() * 1000.0;
            }
            if toi > dt || speed_bound <= f32::EPSILON {
                continue;
            }
            return Err(());
        };
        let mut kind = object.kind;
        if let Some(front) = object.front {
            if contact.normal2.dot(op.rotation * front) < 0.01 {
                kind = ImpactKind::Body;
            }
        }
        if toi <= f32::EPSILON && contact.dist < -GEOMETRY_EPS {
            kind = ImpactKind::InitialOverlap;
        }
        let candidate = Hit {
            toi,
            collider: object.id,
            target: object.target,
            kind,
            point: contact.point2,
            normal: contact.normal2,
            local: op.inverse() * contact.point2,
        };
        if let Some(started) = verify_started {
            diagnostics
                .query_costs
                .entry(object.id)
                .or_default()
                .verify_ms += started.elapsed().as_secs_f64() * 1000.0;
        }
        if best.as_ref().is_none_or(|b| {
            candidate
                .toi
                .total_cmp(&b.toi)
                .then(candidate.collider.to_bits().cmp(&b.collider.to_bits()))
                .is_lt()
        }) {
            best = Some(candidate);
        }
    }
    Ok(best)
}

fn segment_aabb(start: Vec3, delta: Vec3, min: Vec3, max: Vec3) -> bool {
    let mut enter: f32 = 0.0;
    let mut exit: f32 = 1.0;
    for axis in 0..3 {
        if delta[axis].abs() < 1e-12 {
            if start[axis] < min[axis] || start[axis] > max[axis] {
                return false;
            }
        } else {
            let a = (min[axis] - start[axis]) / delta[axis];
            let b = (max[axis] - start[axis]) / delta[axis];
            enter = enter.max(a.min(b));
            exit = exit.min(a.max(b));
            if enter > exit {
                return false;
            }
        }
    }
    true
}

fn acceleration(velocity: Vec3, gravity: Vec3, c: &ProjectileConfig) -> Vec3 {
    let mut a = gravity - c.linear_damping.max(0.0) * velocity;
    let drag = &c.aerodynamics;
    if drag.enabled && c.mass > 0.0 {
        let relative = velocity - Vec3::from_array(drag.wind);
        let area = std::f32::consts::PI * (c.diameter * 0.5).powi(2);
        a -= relative
            * relative.length()
            * (0.5 * drag.air_density.max(0.0) * drag.drag_coefficient.max(0.0) * area / c.mass);
    }
    a
}
fn segment_count(dt: f32, acceleration: f32) -> Option<usize> {
    if !dt.is_finite() || dt <= 0.0 || !acceleration.is_finite() {
        return None;
    }
    let limit = (8.0 * PATH_ERROR / acceleration.max(1e-6))
        .sqrt()
        .min(1.0 / 960.0);
    let count = (dt / limit).ceil() as usize;
    (count <= MAX_SEGMENTS).then_some(count.max(1))
}

fn objects_at(
    t: f64,
    dt: f32,
    selected: usize,
    config: &SimulationConfig,
    registry: &HitGeometry,
    targets: &Query<&TrainingTarget>,
    environment_count: usize,
    objects: &mut Vec<MovingShape>,
) {
    objects.truncate(environment_count);
    for (entity, geometry) in &registry.0 {
        let Ok(target) = targets.get(geometry.root) else {
            continue;
        };
        if config.training.solo_target && selected != target.kind {
            continue;
        }
        let (p0, q0) = target.pose_at(&config.training, selected, t);
        let (p1, _) = target.pose_at(&config.training, selected, t + dt as f64);
        let omega = if target.kind == 2 {
            Vec3::Y * config.training.rotation_speed
        } else {
            Vec3::ZERO
        };
        objects.push(MovingShape {
            id: *entity,
            shape: geometry.shape.clone(),
            motion: NonlinearRigidMotion::new(pose(p0, q0), Vec3::ZERO, (p1 - p0) / dt, omega),
            front: geometry.front,
            kind: geometry.kind,
            target: Some(target.kind),
            bound: geometry.bound,
        });
    }
}

fn advance(
    time: Res<Time<Fixed>>,
    config: Res<SimulationConfig>,
    gravity: Res<Gravity>,
    registry: Res<HitGeometry>,
    targets: Query<&TrainingTarget>,
    colliders: Query<(Entity, &Collider, &Position, &Rotation, &CollisionLayers)>,
    mut environment_cache: ResMut<EnvironmentCache>,
    mut objects: Local<Vec<MovingShape>>,
    mut bullets: Query<(Entity, &mut BallisticProjectile, &mut Transform)>,
    mut board: ResMut<Board>,
    mut stats: ResMut<ProjectileStatistics>,
    mut diagnostics: ResMut<BallisticDiagnostics>,
    mut impacts: MessageWriter<ProjectileImpact>,
    mut commands: Commands,
    mut trace: ResMut<FlightTrace>,
) {
    let started = std::time::Instant::now();
    let selected = board.0.selected;
    let env_membership = GameLayer::environment_collision_layers().memberships.0;
    environment_cache.0.retain(|id, _| {
        colliders
            .get(*id)
            .is_ok_and(|(_, _, _, _, layers)| layers.memberships.0 & env_membership != 0)
    });
    for (id, collider, p, q, layers) in &colliders {
        if layers.memberships.0 & env_membership == 0 {
            continue;
        }
        let shape = collider.shape_scaled();
        if environment_cache.0.get(&id).is_some_and(|cached| {
            cached.motion.start.translation == p.0
                && cached.motion.start.rotation == q.0
                && std::ptr::eq(&*cached.shape, &**shape)
        }) {
            continue;
        }
        let aabb = shape.compute_local_aabb();
        let bound = aabb.mins.abs().max(aabb.maxs.abs()).length();
        environment_cache.0.insert(
            id,
            MovingShape {
                id,
                shape: shape.clone(),
                motion: NonlinearRigidMotion::new(
                    pose(p.0, q.0),
                    Vec3::ZERO,
                    Vec3::ZERO,
                    Vec3::ZERO,
                ),
                front: None,
                kind: ImpactKind::Environment,
                target: None,
                bound,
            },
        );
    }
    objects.clear();
    objects.extend(environment_cache.0.values().cloned());
    let environment_count = objects.len();
    for (entity, mut bullet, mut transform) in &mut bullets {
        if bullet.resolved {
            continue;
        }
        let start_time = (time.elapsed_secs_f64() - time.delta_secs_f64()).max(bullet.born_at);
        let available = (time.elapsed_secs_f64() - start_time).max(0.0) as f32;
        let total_dt = available.min((bullet.lifetime - bullet.age).max(0.0));
        if total_dt <= 0.0 && bullet.age < bullet.lifetime {
            continue;
        }
        let max_radius = registry.0.values().map(|g| g.bound).fold(0.0, f32::max);
        let target_accel = max_radius * config.training.rotation_speed.powi(2)
            + config.training.translation_amplitude
                * (std::f32::consts::TAU / config.training.translation_period).powi(2);
        let a0 = acceleration(bullet.velocity, gravity.0, &config.projectile);
        let count = segment_count(total_dt, a0.length() + target_accel);
        let mut terminal: Option<(ImpactKind, Option<Hit>, f64)> = None;
        if total_dt == 0.0 {
            terminal = Some((ImpactKind::Expired, None, start_time));
        } else if count.is_none()
            || !bullet.position.is_finite()
            || !bullet.velocity.is_finite()
            || !bullet.radius.is_finite()
            || bullet.radius <= 0.0
        {
            terminal = Some((ImpactKind::QueryFailure, None, start_time));
        }
        let count = count.unwrap_or(1);
        let dt = total_dt / count as f32;
        for index in 0..count {
            if terminal.is_some() {
                break;
            }
            let t = start_time + index as f64 * dt as f64;
            let a = acceleration(bullet.velocity, gravity.0, &config.projectile);
            let next_velocity = bullet.velocity + a * dt;
            let displacement = (bullet.velocity + next_velocity) * (0.5 * dt);
            let objects_started = diagnostics.profile_queries.then(std::time::Instant::now);
            objects_at(
                t,
                dt,
                selected,
                &config,
                &registry,
                &targets,
                environment_count,
                &mut objects,
            );
            if let Some(started) = objects_started {
                diagnostics.objects_ms += started.elapsed().as_secs_f64() * 1000.0;
            }
            let old_position = bullet.position;
            diagnostics.segments += 1;
            match first_hit(
                bullet.position,
                displacement,
                bullet.radius,
                dt,
                &objects,
                &mut diagnostics,
            ) {
                Ok(Some(hit)) => {
                    bullet.position += displacement * (hit.toi / dt);
                    terminal = Some((hit.kind, Some(hit.clone()), t + hit.toi as f64));
                }
                Ok(None) => {
                    bullet.position += displacement;
                    bullet.velocity = next_velocity;
                    bullet.age += dt;
                }
                Err(()) => {
                    terminal = Some((ImpactKind::QueryFailure, None, t));
                }
            }
            if trace.enabled() {
                trace.record(serde_json::json!({"event":"segment","shot_id":bullet.id,"shadow":bullet.shadow,
                "t":t,"dt":dt,"start":old_position.to_array(),"end":bullet.position.to_array(),
                "velocity":bullet.velocity.to_array(),"radius":bullet.radius,"geometry_version":GEOMETRY_VERSION,"selected":selected}));
            }
            if config.training.hit_debug {
                trace
                    .trails
                    .push_back((t, old_position, bullet.position, bullet.radius));
            }
        }
        transform.translation = bullet.position;
        if terminal.is_none() && bullet.age + f32::EPSILON >= bullet.lifetime {
            terminal = Some((ImpactKind::Expired, None, start_time + total_dt as f64));
        }
        if let Some((kind, hit, stamp)) = terminal {
            bullet.resolved = true;
            if kind == ImpactKind::QueryFailure {
                diagnostics.failures += 1;
            }
            diagnostics.impacts += 1;
            if !bullet.shadow {
                if kind == ImpactKind::Armor {
                    stats.increase_accurate();
                }
                if let Some(shot) = &mut bullet.shot {
                    board.0.contact(
                        shot,
                        hit.as_ref()
                            .filter(|h| h.kind == ImpactKind::Armor)
                            .and_then(|h| h.target),
                    );
                }
            }
            let impact = ProjectileImpact {
                shot_id: bullet.id,
                projectile: entity,
                collider: hit.as_ref().map(|h| h.collider),
                target: hit.as_ref().and_then(|h| h.target),
                simulation_time: stamp,
                point_world: hit.as_ref().map_or(bullet.position, |h| h.point),
                normal_world: hit.as_ref().map_or(Vec3::ZERO, |h| h.normal),
                point_vehicle_local: hit.as_ref().filter(|h| h.target.is_some()).map(|h| h.local),
                kind,
                shadow: bullet.shadow,
            };
            info!("PROJECTILE_IMPACT {impact:?}");
            trace.record(serde_json::json!({"event":"impact","shot_id":impact.shot_id,"shadow":impact.shadow,
                "t":impact.simulation_time,"kind":format!("{:?}",impact.kind),"target":impact.target,
                "collider":impact.collider.map(Entity::to_bits),"point":impact.point_world.to_array(),
                "normal":impact.normal_world.to_array(),"local":impact.point_vehicle_local.map(|v|v.to_array())}));
            if config.training.hit_debug {
                trace.hits.push_back(impact.clone());
            }
            impacts.write(impact);
            commands.entity(entity).try_despawn();
        }
    }
    while trace
        .trails
        .front()
        .is_some_and(|(t, _, _, _)| time.elapsed_secs_f64() - t > 2.0)
        || trace.trails.len() > 8192
    {
        trace.trails.pop_front();
    }
    while trace
        .hits
        .front()
        .is_some_and(|h| time.elapsed_secs_f64() - h.simulation_time > 2.0)
        || trace.hits.len() > 256
    {
        trace.hits.pop_front();
    }
    diagnostics
        .step_ms
        .push_back(started.elapsed().as_secs_f32() * 1000.0);
    if diagnostics.step_ms.len() > 256 {
        diagnostics.step_ms.pop_front();
    }
    if config.training.hit_debug || trace.file.is_some() {
        if diagnostics.profile_queries {
            let costs:Vec<_> = diagnostics.query_costs.iter().map(|(id,c)|serde_json::json!({
                "collider":id.to_bits(),"calls":c.calls,"cast_ms":c.cast_ms,"verify_ms":c.verify_ms})).collect();
            trace.record(
                serde_json::json!({"event":"profile","t":time.elapsed_secs_f64(),
                "objects_ms":diagnostics.objects_ms,"costs":costs}),
            );
        }
        trace.record(serde_json::json!({"event":"diagnostics","t":time.elapsed_secs_f64(),
            "p95_ms":diagnostics.p95_ms(),"candidates":diagnostics.candidates,
            "queries":diagnostics.queries,"failures":diagnostics.failures,"impacts":diagnostics.impacts}));
    }
}

fn draw_debug(
    config: Res<SimulationConfig>,
    time: Res<Time<Fixed>>,
    board: Res<Board>,
    registry: Res<HitGeometry>,
    targets: Query<&TrainingTarget>,
    trace: Res<FlightTrace>,
    mut gizmos: Gizmos,
) {
    if !config.training.hit_debug {
        return;
    }
    for (index, (_, start, end, radius)) in trace.trails.iter().enumerate() {
        gizmos.line(*start, *end, Color::srgb(0.1, 1.0, 1.0));
        if index % 16 == 0 {
            gizmos.sphere(
                Isometry3d::from_translation(*end),
                *radius,
                Color::srgb(0.1, 0.5, 0.6),
            );
        }
    }
    for hit in &trace.hits {
        gizmos.sphere(
            Isometry3d::from_translation(hit.point_world),
            0.006,
            Color::srgb(1.0, 0.0, 1.0),
        );
        gizmos.line(
            hit.point_world,
            hit.point_world + hit.normal_world * 0.1,
            Color::srgb(1.0, 0.0, 1.0),
        );
    }
    for geometry in registry.0.values() {
        let Ok(target) = targets.get(geometry.root) else {
            continue;
        };
        if config.training.solo_target && target.kind != board.0.selected {
            continue;
        }
        let (p, q) = target.pose_at(&config.training, board.0.selected, time.elapsed_secs_f64());
        if geometry.kind == ImpactKind::Armor {
            if let Some(mesh) = geometry.shape.as_trimesh() {
                for triangle in mesh.triangles() {
                    for (a, b) in [
                        (triangle.a, triangle.b),
                        (triangle.b, triangle.c),
                        (triangle.c, triangle.a),
                    ] {
                        gizmos.line(p + q * a, p + q * b, Color::srgb(0.1, 1.0, 0.1));
                    }
                }
            }
        } else {
            let aabb = geometry.shape.compute_local_aabb();
            let corners: Vec<_> = (0..8)
                .map(|i| {
                    p + q * Vec3::new(
                        if i & 1 == 0 { aabb.mins.x } else { aabb.maxs.x },
                        if i & 2 == 0 { aabb.mins.y } else { aabb.maxs.y },
                        if i & 4 == 0 { aabb.mins.z } else { aabb.maxs.z },
                    )
                })
                .collect();
            for i in 0..8 {
                for bit in [1, 2, 4] {
                    if i & bit == 0 {
                        gizmos.line(corners[i], corners[i | bit], Color::srgb(1.0, 0.3, 0.1));
                    }
                }
            }
        }
    }
}

#[cfg(test)]
#[path = "ballistics_tests.rs"]
mod tests;
