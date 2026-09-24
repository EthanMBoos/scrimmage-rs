//! Aerodynamic fixed-wing six-degree-of-freedom motion, with RK4 propagation.
//!
//! C++ counterpart: motion/FixedWing6DOF (GTRI, LGPL-3.0-or-later).
//! Motion phase: read throttle and surface angles, update entity truth.
//! Private body axes are forward/right/down; public truth remains ENU world.
//! The two-aircraft mission checks the port, not trimmed flight or ArduPilot readiness.

mod aerodynamics;
mod dynamics;

use anyhow::{Context, Result, ensure};
use nalgebra::{Matrix3, UnitQuaternion};

use crate::math::{self, KinematicState, Vec3};
use crate::plugin::{
    Frame, MotionContext, MotionModel, Plugin, PluginIo, PluginParams, Port, Ports, Unit, Update,
};

use aerodynamics::{Aerodynamics, Airframe};
use dynamics::{AircraftState, derivative, flip_yz};

// Preserve the C++ rounded limit rather than replacing it with exactly pi/6.
#[allow(clippy::approx_constant)]
const LEGACY_SURFACE_LIMIT_RAD: f64 = 0.5236;

#[derive(Clone)]
pub struct FixedWingConfig {
    mass_kg: f64,
    gravity_mps2: f64,
    inertia_kgm2: Matrix3<f64>,
    inverse_inertia: Matrix3<f64>,
    density_kgpm3: f64,
    span_m: f64,
    area_m2: f64,
    chord_m: f64,
    efficiency: f64,
    wind_world_mps: Vec3,
    throttle_min: f64,
    throttle_max: f64,
    thrust_min_n: f64,
    thrust_max_n: f64,
    elevator_limits_rad: [f64; 2],
    aileron_limits_rad: [f64; 2],
    rudder_limits_rad: [f64; 2],
    aerodynamics: Aerodynamics,
    use_ground_model: bool,
    use_launcher: bool,
    launch_time_s: f64,
    launch_accel_mps2: f64,
    launch_speed_mps: f64,
}

#[derive(Clone, Copy, Default)]
struct Control {
    throttle: f64,
    elevator_rad: f64,
    aileron_rad: f64,
    rudder_rad: f64,
}

enum LaunchState {
    Waiting,
    Launching { start_s: f64 },
    Flying,
}

pub struct FixedWing6Dof {
    config: FixedWingConfig,
    launch: LaunchState,
}

impl Plugin for FixedWing6Dof {
    type Config = FixedWingConfig;

    fn configure(params: &PluginParams<'_>) -> Result<FixedWingConfig> {
        for key in ["write_csv", "draw_vel", "draw_ang_vel"] {
            ensure!(
                !params.boolean(key, false)?,
                "FixedWing6DOF {key}=true is not supported; use the run's frames and Rerun recording"
            );
        }
        // Do not silently accept a tuning value that the C++ source ignores.
        ensure!(
            params.number("C_D_delta_elevator", 0.01)? == 0.01,
            "use C_D_delta_elevator_ (trailing underscore), the key read by C++ FixedWing6DOF"
        );
        let inertia_kgm2 = parse_inertia(params)?;
        let inverse_inertia = inertia_kgm2
            .try_inverse()
            .context("FixedWing6DOF inertia must be invertible")?;
        let config = FixedWingConfig {
            mass_kg: params.number("mass", 1.2)?,
            gravity_mps2: params.number("gravity_magnitude", 9.81)?,
            inertia_kgm2,
            inverse_inertia,
            density_kgpm3: params.number("air_density", 1.225)?,
            span_m: params.number("wing_span", 8.382)?,
            area_m2: params.number("surface_area_of_wing", 24.1548)?,
            chord_m: params.number("chord_length", 3.29184)?,
            efficiency: params.number("efficiency_factor", 0.995)?,
            wind_world_mps: Vec3::new(
                params.number("wind_E", 0.0)?,
                params.number("wind_N", 0.0)?,
                params.number("wind_U", 0.0)?,
            ),
            throttle_min: params.number("throttle_input_min", 0.0)?,
            throttle_max: params.number("throttle_input_max", 1.0)?,
            thrust_min_n: params.number("thrust_min", -100_000.0)?,
            thrust_max_n: params.number("thrust_max", 100_000.0)?,
            elevator_limits_rad: surface_limits(
                params,
                "delta_elevator_min",
                "delta_elevator_max",
                LEGACY_SURFACE_LIMIT_RAD,
            )?,
            aileron_limits_rad: surface_limits(
                params,
                "delta_aileron_min",
                "delta_aileron_max",
                LEGACY_SURFACE_LIMIT_RAD,
            )?,
            rudder_limits_rad: surface_limits(
                params,
                "delta_rudder_min",
                "delta_rudder_max",
                0.2618,
            )?,
            aerodynamics: Aerodynamics::parse(params)?,
            use_ground_model: params.boolean("use_ground_model", true)?,
            use_launcher: params.boolean("use_launcher", false)?,
            launch_time_s: params.number("launch_time", 30.0)?,
            launch_accel_mps2: params.number("launch_accel", 200.0)?,
            launch_speed_mps: params.number("launch_speed", 20.0)?,
        };
        ensure!(
            config.mass_kg > 0.0
                && config.density_kgpm3 >= 0.0
                && config.gravity_mps2 >= 0.0
                && config.span_m > 0.0
                && config.area_m2 > 0.0
                && config.chord_m > 0.0
                && config.efficiency > 0.0,
            "FixedWing6DOF requires positive mass and geometry, nonnegative air density and gravity"
        );
        ensure!(
            config.throttle_min < config.throttle_max && config.thrust_min_n <= config.thrust_max_n,
            "FixedWing6DOF has invalid throttle/thrust limits"
        );
        ensure!(
            !config.use_launcher
                || (config.launch_accel_mps2 > 0.0 && config.launch_speed_mps >= 0.0),
            "FixedWing6DOF launcher requires positive acceleration and nonnegative speed"
        );
        Ok(config)
    }

    fn new(config: &FixedWingConfig) -> Self {
        Self {
            config: config.clone(),
            launch: if config.use_launcher {
                LaunchState::Waiting
            } else {
                LaunchState::Flying
            },
        }
    }

    fn ports(_config: &FixedWingConfig) -> Ports {
        Ports::default()
            .input(Port::new("throttle", Unit::Dimensionless, Frame::None))
            .input(Port::new("elevator", Unit::Radians, Frame::Model))
            .input(Port::new("aileron", Unit::Radians, Frame::Model))
            .input(Port::new("rudder", Unit::Radians, Frame::Model))
    }
}

impl MotionModel for FixedWing6Dof {
    fn step(&mut self, context: &mut MotionContext<'_>, io: &mut PluginIo) -> Result<Update> {
        let control = Control {
            throttle: io.read("throttle")?,
            elevator_rad: io.read("elevator")?,
            aileron_rad: io.read("aileron")?,
            rudder_rad: io.read("rudder")?,
        };
        self.integrate(
            context.truth,
            control,
            context.time.time_s,
            context.time.dt_s,
        )?;
        Ok(Update::Applied)
    }
}

impl FixedWing6Dof {
    fn integrate(
        &mut self,
        truth: &mut KinematicState,
        control: Control,
        time_s: f64,
        dt_s: f64,
    ) -> Result<()> {
        let catapult_force_n = self.advance_launcher(time_s);
        let config = &self.config;
        let state = AircraftState::from_truth(truth);
        let rotation = UnitQuaternion::new_unchecked(state.attitude_model);
        let control = Control {
            throttle: control.throttle.clamp(-1.0, 1.0),
            elevator_rad: control
                .elevator_rad
                .clamp(config.elevator_limits_rad[0], config.elevator_limits_rad[1]),
            aileron_rad: control
                .aileron_rad
                .clamp(config.aileron_limits_rad[0], config.aileron_limits_rad[1]),
            rudder_rad: control
                .rudder_rad
                .clamp(config.rudder_limits_rad[0], config.rudder_limits_rad[1]),
        };
        let thrust_n = config.thrust_n(control.throttle);
        let wind_model_mps = rotation.inverse_transform_vector(&flip_yz(config.wind_world_mps));
        // Deliberate reference quirk: wind is ADDED to velocity, not subtracted.
        let air_velocity_model_mps = state.velocity_model_mps + wind_model_mps;
        let airframe = Airframe {
            density_kgpm3: config.density_kgpm3,
            span_m: config.span_m,
            area_m2: config.area_m2,
            chord_m: config.chord_m,
            efficiency: config.efficiency,
        };
        let mut loads = config.aerodynamics.loads(
            &airframe,
            air_velocity_model_mps,
            state.rates_model_radps,
            control,
        );
        let weight_model_n = rotation.inverse_transform_vector(&Vec3::new(
            0.0,
            0.0,
            config.mass_kg * config.gravity_mps2,
        ));
        loads.force_model_n = weight_model_n + Vec3::new(thrust_n, 0.0, 0.0) + loads.force_model_n;

        if config.use_ground_model && state.position_world_m.z < 0.0 {
            // Inherited ground model: damping is only 40 N*s/m, not mass-scaled.
            // This is not a validated landing model for the default eight-tonne aircraft.
            let ground_force_n = (-400.0 * config.mass_kg * state.position_world_m.z
                - 40.0 * state.velocity_world_mps.z)
                .max(0.0);
            loads.force_model_n +=
                rotation.inverse_transform_vector(&Vec3::new(0.0, 0.0, -ground_force_n));
        }
        if !matches!(self.launch, LaunchState::Flying) {
            loads.force_model_n = Vec3::zeros();
            loads.moment_model_nm = Vec3::zeros();
        }
        loads.force_model_n.x += catapult_force_n;
        let acceleration_model_mps2 = loads.force_model_n / config.mass_kg;
        let rates = state.rates_model_radps;
        let angular_acceleration_model_radps2 = config.inverse_inertia
            * (loads.moment_model_nm - rates.cross(&(config.inertia_kgm2 * rates)));

        // C++ evaluates loads from x_ (the start of the step), not RK4 stage x.
        // Freeze loads here; only kinematics and attitude vary inside the stages.
        let mut coordinates = state.coordinates();
        math::rk4(&mut coordinates, dt_s, |coordinates| {
            derivative(
                AircraftState::from_coordinates(*coordinates),
                acceleration_model_mps2,
                angular_acceleration_model_radps2,
            )
        });
        ensure!(
            coordinates.iter().all(|value| value.is_finite()),
            "FixedWing6DOF integration produced nonfinite state"
        );
        AircraftState::from_coordinates(coordinates).write_truth(truth);
        Ok(())
    }

    fn advance_launcher(&mut self, time_s: f64) -> f64 {
        match self.launch {
            LaunchState::Waiting => {
                if time_s > self.config.launch_time_s {
                    self.launch = LaunchState::Launching { start_s: time_s };
                }
                0.0
            }
            LaunchState::Launching { start_s } => {
                if self.config.launch_speed_mps / self.config.launch_accel_mps2 < time_s - start_s {
                    self.launch = LaunchState::Flying;
                }
                // Reference applies this force even on the transition-to-flight tick.
                self.config.launch_accel_mps2 * self.config.mass_kg
            }
            LaunchState::Flying => 0.0,
        }
    }
}

impl FixedWingConfig {
    fn thrust_n(&self, throttle: f64) -> f64 {
        // C++ Utilities::scale clamps again to the configured input range.
        // Defaults map zero throttle to -36,000 N (reverse thrust), not idle.
        // A non-reversing engine needs different limits; don't compensate in the PID.
        let throttle = throttle
            .clamp(-1.0, 1.0)
            .clamp(self.throttle_min, self.throttle_max);
        let scale =
            (self.thrust_max_n - self.thrust_min_n) / (self.throttle_max - self.throttle_min);
        (throttle - self.throttle_min) * scale + self.thrust_min_n
    }
}

fn surface_limits(
    params: &PluginParams<'_>,
    min_key: &str,
    max_key: &str,
    magnitude_rad: f64,
) -> Result<[f64; 2]> {
    let min_rad = params.number(min_key, -magnitude_rad)?;
    let max_rad = params.number(max_key, magnitude_rad)?;
    ensure!(
        min_rad <= max_rad,
        "FixedWing6DOF requires {min_key} <= {max_key}"
    );
    Ok([min_rad, max_rad])
}

fn parse_inertia(params: &PluginParams<'_>) -> Result<Matrix3<f64>> {
    // Legacy precedence: bundled slug defaults hide a mission's SI override.
    // Change the bundled defaults to SI when correcting this model locally.
    let (text, factor) = if let Some(text) = params.text("inertia_matrix_slug_ft_sq") {
        (text, 1.35581795)
    } else if let Some(text) = params.text("inertia_matrix") {
        (text, 1.0)
    } else {
        return Ok(Matrix3::identity());
    };
    let text = text.replace(['[', ']'], " ");
    let mut values = Vec::new();
    for value in text.split_whitespace() {
        values.push(
            value
                .parse::<f64>()
                .context("invalid FixedWing6DOF inertia value")?
                * factor,
        );
    }
    ensure!(
        values.len() == 9 && values.iter().all(|value| value.is_finite()),
        "FixedWing6DOF inertia requires nine finite values"
    );
    let inertia = Matrix3::from_row_slice(&values);
    ensure!(
        (inertia - inertia.transpose()).norm() < 1e-9 && inertia.cholesky().is_some(),
        "FixedWing6DOF inertia must be symmetric positive definite"
    );
    Ok(inertia)
}

#[cfg(test)]
mod tests {
    use super::FixedWing6Dof;
    use crate::{
        Params,
        plugin::{Plugin, PluginParams},
    };

    #[test]
    fn throttle_scaling_clamps_like_cpp_utilities_scale() -> anyhow::Result<()> {
        let params = Params::from([
            ("thrust_min".into(), "-36000".into()),
            ("thrust_max".into(), "36000".into()),
        ]);
        let config = FixedWing6Dof::configure(&PluginParams(&params))?;
        for (input, expected_n) in [
            (-2.0, -36000.0),
            (-1.0, -36000.0),
            (0.0, -36000.0),
            (0.5, 0.0),
            (1.0, 36000.0),
            (2.0, 36000.0),
        ] {
            assert!((config.thrust_n(input) - expected_n).abs() < 1e-9);
        }
        Ok(())
    }
}
