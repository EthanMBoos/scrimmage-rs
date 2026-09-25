//! Noisy own-state feedback and a typed LocalNetwork observation.
//!
//! C++ counterpart: src/plugins/sensor/NoisyState/NoisyState.cpp (LGPL-3.0-or-later).
//! Sensor phase: sample truth, perturb position/velocity/attitude, publish and update belief.
//! Uses Rust's per-instance RNG, not C++'s shared random sequence.

use anyhow::{Result, ensure};
use nalgebra::{Matrix3, UnitQuaternion};

use crate::math::{KinematicState, Quaternion, Vec3};
use crate::plugin::{Plugin, PluginParams, PluginRandom, Sensor, SensorContext, Update};

pub const STATE_TOPIC: &str = "StateWithCovariance";

/// Legacy `mean standard_deviation` noise for one axis.
#[derive(Clone, Copy)]
pub(super) struct AxisNoise {
    pub(super) mean: f64,
    pub(super) stddev: f64,
}

/// The C++ state-noise equations, shared by NoisyState and NoisyContacts so both
/// keep identical parameters, draw order, and attitude perturbation.
#[derive(Clone, Copy)]
pub(super) struct StateNoise {
    pub(super) position_m: [AxisNoise; 3],
    pub(super) velocity_mps: [AxisNoise; 3],
    pub(super) orientation_rad: [AxisNoise; 3],
}

pub struct NoisyStateConfig {
    noise: StateNoise,
}

/// Legacy state payload; covariance is a placeholder, NOT configured noise variance.
/// NoisyState uses identity and zero angular velocity; NoisyContacts uses 5I
/// and preserves the target's angular velocity, matching their C++ constructors.
#[derive(Clone, Debug)]
pub struct StateWithCovariance {
    pub state: KinematicState,
    pub covariance: Matrix3<f64>,
}

pub struct NoisyState {
    noise: StateNoise,
}

impl Plugin for NoisyState {
    type Config = NoisyStateConfig;

    fn configure(params: &PluginParams<'_>) -> Result<NoisyStateConfig> {
        Ok(NoisyStateConfig {
            noise: StateNoise::parse(params)?,
        })
    }

    fn new(config: &NoisyStateConfig) -> Self {
        Self {
            noise: config.noise,
        }
    }
}

impl Sensor for NoisyState {
    fn initialize(&mut self, context: &mut SensorContext<'_>) -> Result<()> {
        // Detach before the first motion update, as the C++ sensor does.
        context.set_belief(context.truth.clone());
        Ok(())
    }

    fn step(&mut self, context: &mut SensorContext<'_>) -> Result<Update> {
        let observation = self.measure(context.truth, context.random)?;
        context.set_belief(observation.state.clone());
        context
            .messages
            .publish("LocalNetwork", STATE_TOPIC, observation)?;
        Ok(Update::Applied)
    }
}

impl NoisyState {
    fn measure(
        &self,
        truth: &KinematicState,
        random: &mut PluginRandom,
    ) -> Result<StateWithCovariance> {
        let mut measured = self.noise.apply(truth, random)?;
        // Matches C++ for now, though we don't consider it correct: the C++ message
        // starts from a default state, so a rotating vehicle reports zero angular velocity.
        measured.angular_velocity_world_radps = Vec3::zeros();
        // Matches C++ for now: identity is a placeholder, not the configured noise variance.
        Ok(StateWithCovariance {
            state: measured,
            covariance: Matrix3::identity(),
        })
    }
}

impl StateNoise {
    /// Reads `pos_noise_0..2`, `vel_noise_0..2`, and `orient_noise_0..2`.
    /// A missing axis defaults to mean 0 and standard deviation 1, as in C++.
    pub(super) fn parse(params: &PluginParams<'_>) -> Result<Self> {
        Ok(Self {
            position_m: parse_axes(params, "pos_noise")?,
            velocity_mps: parse_axes(params, "vel_noise")?,
            orientation_rad: parse_axes(params, "orient_noise")?,
        })
    }

    /// Returns truth with noisy position, velocity, and attitude.
    /// Angular velocity is copied unchanged; callers apply their C++ constructor's rule.
    pub(super) fn apply(
        &self,
        truth: &KinematicState,
        random: &mut PluginRandom,
    ) -> Result<KinematicState> {
        let mut measured = truth.clone();
        // C++ interleaves position and velocity samples per axis, including zero noise.
        for axis in 0..3 {
            let position_noise = self.position_m[axis];
            let velocity_noise = self.velocity_mps[axis];
            measured.position_world_m[axis] = truth.position_world_m[axis]
                + random.normal(position_noise.mean, position_noise.stddev)?;
            measured.velocity_world_mps[axis] = truth.velocity_world_mps[axis]
                + random.normal(velocity_noise.mean, velocity_noise.stddev)?;
        }

        let roll_noise_rad =
            random.normal(self.orientation_rad[0].mean, self.orientation_rad[0].stddev)?;
        let pitch_noise_rad =
            random.normal(self.orientation_rad[1].mean, self.orientation_rad[1].stddev)?;
        let yaw_noise_rad =
            random.normal(self.orientation_rad[2].mean, self.orientation_rad[2].stddev)?;
        let roll_error =
            UnitQuaternion::from_axis_angle(&Vec3::x_axis(), roll_noise_rad).into_inner();
        let pitch_error =
            UnitQuaternion::from_axis_angle(&Vec3::y_axis(), pitch_noise_rad).into_inner();
        let yaw_error =
            UnitQuaternion::from_axis_angle(&Vec3::z_axis(), yaw_noise_rad).into_inner();
        let attitude = truth.orientation_world_from_body;
        let truth_orientation =
            nalgebra::Quaternion::new(attitude.w, attitude.x, attitude.y, attitude.z);
        let orientation = truth_orientation * roll_error * pitch_error * yaw_error;
        // Right multiplication perturbs body axes in roll, pitch, yaw order.
        // Do not replace this with additive Euler angles or normalize the result.
        measured.orientation_world_from_body = Quaternion {
            w: orientation.w,
            x: orientation.i,
            y: orientation.j,
            z: orientation.k,
        };
        Ok(measured)
    }
}

fn parse_axes(params: &PluginParams<'_>, prefix: &str) -> Result<[AxisNoise; 3]> {
    let mut axes = [AxisNoise {
        mean: 0.0,
        stddev: 1.0,
    }; 3];
    for (axis, noise) in axes.iter_mut().enumerate() {
        let key = format!("{prefix}_{axis}");
        let [mean, stddev] = params.vector(&key, [0.0, 1.0])?;
        ensure!(
            stddev >= 0.0,
            "{key} standard deviation must be nonnegative"
        );
        *noise = AxisNoise { mean, stddev };
    }
    Ok(axes)
}

#[cfg(test)]
mod tests {
    use super::{AxisNoise, NoisyState, NoisyStateConfig, StateNoise};
    use crate::Params;
    use crate::math::{EulerAngles, KinematicState, Quaternion, Vec3};
    use crate::plugin::{Plugin, PluginParams, PluginRandom};

    fn sensor(stddev: f64) -> NoisyState {
        NoisyState::new(&NoisyStateConfig {
            noise: StateNoise {
                position_m: [AxisNoise { mean: 2.0, stddev }; 3],
                velocity_mps: [AxisNoise { mean: -1.0, stddev }; 3],
                orientation_rad: [AxisNoise {
                    mean: 0.0,
                    stddev: 0.0,
                }; 3],
            },
        })
    }

    #[test]
    fn zero_noise_preserves_legacy_bias_covariance_and_angular_velocity() -> anyhow::Result<()> {
        let truth = KinematicState {
            position_world_m: Vec3::new(10.0, 20.0, 30.0),
            velocity_world_mps: Vec3::new(3.0, 4.0, 5.0),
            angular_velocity_world_radps: Vec3::repeat(7.0),
            ..KinematicState::default()
        };
        let sample = sensor(0.0).measure(&truth, &mut PluginRandom::new(123, 1, "NoisyState:0"))?;
        assert_eq!(sample.state.position_world_m, Vec3::new(12.0, 22.0, 32.0));
        assert_eq!(sample.state.velocity_world_mps, Vec3::new(2.0, 3.0, 4.0));
        assert_eq!(sample.state.angular_velocity_world_radps, Vec3::zeros());
        assert_eq!(sample.covariance, nalgebra::Matrix3::identity());
        assert_eq!(truth.position_world_m, Vec3::new(10.0, 20.0, 30.0));
        Ok(())
    }

    #[test]
    fn attitude_noise_rotates_body_axes_in_roll_pitch_yaw_order() -> anyhow::Result<()> {
        let mut sensor = sensor(0.0);
        sensor.noise.orientation_rad[0].mean = std::f64::consts::FRAC_PI_2;
        sensor.noise.orientation_rad[1].mean = std::f64::consts::FRAC_PI_2;
        let truth = KinematicState {
            orientation_world_from_body: Quaternion::from_euler(EulerAngles {
                yaw_world_from_body_rad: std::f64::consts::FRAC_PI_2,
                ..EulerAngles::default()
            }),
            ..KinematicState::default()
        };
        let sample = sensor.measure(&truth, &mut PluginRandom::new(123, 1, "NoisyState:0"))?;
        let nose = sample
            .state
            .orientation_world_from_body
            .rotate_body_to_world(Vec3::x());
        assert!((nose - Vec3::new(-1.0, 0.0, 0.0)).norm() < 1e-12);
        Ok(())
    }

    #[test]
    fn seeded_samples_follow_interleaved_axis_order() -> anyhow::Result<()> {
        let mut random = PluginRandom::new(123, 1, "NoisyState:0");
        let mut expected = PluginRandom::new(123, 1, "NoisyState:0");
        let sample = sensor(0.5).measure(&KinematicState::default(), &mut random)?;
        for axis in 0..3 {
            assert_eq!(
                sample.state.position_world_m[axis],
                expected.normal(2.0, 0.5)?
            );
            assert_eq!(
                sample.state.velocity_world_mps[axis],
                expected.normal(-1.0, 0.5)?
            );
        }
        for _ in 0..3 {
            expected.normal(0.0, 0.0)?;
        }
        assert_eq!(random.normal(0.0, 1.0)?, expected.normal(0.0, 1.0)?);
        Ok(())
    }

    #[test]
    fn position_noise_has_the_configured_mean_and_variance() -> anyhow::Result<()> {
        let sensor = sensor(0.5);
        let mut random = PluginRandom::new(123, 1, "NoisyState:0");
        let mut sum = 0.0;
        let mut sum_squared = 0.0;
        for _ in 0..10000 {
            let x = sensor
                .measure(&KinematicState::default(), &mut random)?
                .state
                .position_world_m
                .x;
            sum += x;
            sum_squared += x * x;
        }
        let mean = sum / 10000.0;
        let variance = sum_squared / 10000.0 - mean * mean;
        assert!((mean - 2.0).abs() < 0.03);
        assert!((variance - 0.25).abs() < 0.02);
        Ok(())
    }

    #[test]
    fn invalid_noise_parameters_are_rejected() {
        for value in ["0 -1", "NaN 1", "0 inf", "0 1 2"] {
            let params = Params::from([("pos_noise_0".into(), value.into())]);
            assert!(NoisyState::configure(&PluginParams(&params)).is_err());
        }
    }
}
