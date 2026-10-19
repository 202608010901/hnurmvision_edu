//! Camera-facing fidelity.
//!
//! The preview window is judged by a human eye; a vision stack is judged against footage
//! from a real camera. The two differ in ways that matter to a detector: armour LEDs are
//! blown out rather than tinted, they bleed a halo into their neighbours, and the sensor
//! adds noise. Everything here is knob-gated from `[vision]` in `config.toml` and is a
//! no-op at its default value.

use bevy::prelude::*;

use crate::components::Infantry;
use crate::config::SimulationConfig;
use crate::robomaster::visibility::StatefulAppearanceCreator;

#[derive(Component)]
pub struct LampEmissiveBoosted;

/// Scale the emissive term of every vehicle's materials once, so armour lamps render at
/// the brightness a real camera sees instead of the glb's own faint glow.
///
/// The lamps as shipped peak at grey ~142, under the 150 the team's classical pipeline
/// thresholds at. That stage is not what decides whether an armour is detected — the
/// YOLOv5 network is — but it is the corner-refinement pass that runs on every detection,
/// and corners it cannot find are left at the network's own (looser) estimate. At 142 the
/// refinement never fires; at ~195 it does.
///
/// Vehicles are `WorldAssetRoot` scenes: their meshes and materials only exist after the
/// glb finishes loading, and that can happen at any point after spawn. So walk the tree
/// each frame until it actually touches a material, then mark the root — each vehicle is
/// scaled exactly once, no matter how late its assets arrive.
pub fn boost_vehicle_emissive(
    config: Res<SimulationConfig>,
    mut commands: Commands,
    mut creator: StatefulAppearanceCreator,
    vehicles: Query<Entity, (With<Infantry>, Without<LampEmissiveBoosted>)>,
) {
    let scale = config.vision.armor_emissive_scale;
    if !scale.is_finite() || scale <= 0.0 || (scale - 1.0).abs() < f32::EPSILON {
        return;
    }
    let mut missed = 0;
    for vehicle in &vehicles {
        let changed = creator.scale_emissive_tree(vehicle, scale);
        if changed > 0 {
            commands.entity(vehicle).insert(LampEmissiveBoosted);
        } else {
            missed += 1;
        }
    }
    if missed == 0 {
        info!("VISION emissive bootstrapped, scale={}", scale);
    }
}

pub fn has_unboosted_vehicles(
    config: Res<SimulationConfig>,
    vehicles: Query<(), (With<Infantry>, Without<LampEmissiveBoosted>)>,
) -> bool {
    let scale = config.vision.armor_emissive_scale;
    if !scale.is_finite() || scale <= 0.0 || (scale - 1.0).abs() < f32::EPSILON {
        return false;
    }
    !vehicles.is_empty()
}

/// Per-pixel Gaussian sensor noise, in gray levels, applied in place.
///
/// A real camera's readout is never this clean, and a stack validated only on noise-free
/// frames tends to rely on a background that is exactly constant. Sampling Box-Muller per
/// channel would cost more than the rest of the capture path, so the curve is tabulated
/// once per frame and indexed with a xorshift step.
pub fn add_sensor_noise(data: &mut [u8], sigma: f32, seed: u32) {
    if !sigma.is_finite() || sigma <= 0.0 || data.is_empty() {
        return;
    }

    const TABLE: usize = 4096;
    let mut table = [0i32; TABLE];
    let mut state = seed | 1;
    let mut next = move || {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        state
    };
    for slot in table.iter_mut() {
        // Two uniforms -> one normal sample; the result is kept in 1/256 gray levels.
        let u1 = (next() >> 8) as f32 / 16_777_216.0 + f32::EPSILON;
        let u2 = (next() >> 8) as f32 / 16_777_216.0;
        let normal = (-2.0 * u1.ln()).sqrt() * (std::f32::consts::TAU * u2).cos();
        *slot = (normal * sigma * 256.0).clamp(-4096.0, 4096.0) as i32;
    }

    for value in data.iter_mut() {
        // The high bits of a xorshift step are the well-mixed ones; TABLE is a power of two.
        let delta = table[((next() >> 16) as usize) & (TABLE - 1)] >> 8;
        *value = (*value as i32 + delta).clamp(0, 255) as u8;
    }
}
