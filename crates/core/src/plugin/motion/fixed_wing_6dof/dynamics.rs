//! Named integration state and quaternion equations; no configuration or plugin I/O.
//! Model rates: forward/right/down body. Public truth velocities: ENU world.
//! Public attitude rotates forward/left/up body axes into the ENU world frame.

use std::f64::consts::PI;

use nalgebra::{Quaternion as ModelQuaternion, UnitQuaternion};

use crate::math::{KinematicState, Quaternion, Vec3};

#[derive(Clone, Copy)]
pub(super) struct AircraftState {
    pub velocity_model_mps: Vec3,
    pub rates_model_radps: Vec3,
    pub velocity_world_mps: Vec3,
    pub position_world_m: Vec3,
    pub attitude_model: ModelQuaternion<f64>,
}

pub(super) fn flip_yz(vector: Vec3) -> Vec3 {
    Vec3::new(vector.x, -vector.y, -vector.z)
}

/// Match the reference's two 180-degree X rotations, including quaternion sign.
fn switch_attitude_frame(attitude: ModelQuaternion<f64>) -> ModelQuaternion<f64> {
    let half_turn = ModelQuaternion::new((PI / 2.0).cos(), (PI / 2.0).sin(), 0.0, 0.0);
    (half_turn * (attitude * half_turn)).normalize()
}

impl AircraftState {
    pub fn from_truth(truth: &KinematicState) -> Self {
        let attitude = truth.orientation_world_from_body;
        let attitude_model = switch_attitude_frame(ModelQuaternion::new(
            attitude.w, attitude.x, attitude.y, attitude.z,
        ));
        let rotation = UnitQuaternion::new_unchecked(attitude_model);
        Self {
            velocity_model_mps: rotation
                .inverse_transform_vector(&flip_yz(truth.velocity_world_mps)),
            rates_model_radps: rotation
                .inverse_transform_vector(&flip_yz(truth.angular_velocity_world_radps)),
            velocity_world_mps: truth.velocity_world_mps,
            position_world_m: truth.position_world_m,
            attitude_model,
        }
    }

    pub fn write_truth(self, truth: &mut KinematicState) {
        let rotation = UnitQuaternion::new_normalize(self.attitude_model);
        let attitude = switch_attitude_frame(*rotation.quaternion());
        truth.orientation_world_from_body = Quaternion {
            w: attitude.w,
            x: attitude.i,
            y: attitude.j,
            z: attitude.k,
        };
        truth.position_world_m = self.position_world_m;
        truth.velocity_world_mps = self.velocity_world_mps;
        truth.angular_velocity_world_radps =
            flip_yz(rotation.transform_vector(&self.rates_model_radps));
    }

    // Flat coordinates exist only at the RK4 boundary; equations use named fields.
    pub fn coordinates(self) -> [f64; 16] {
        let velocity = self.velocity_model_mps;
        let rates = self.rates_model_radps;
        let world_velocity = self.velocity_world_mps;
        let position = self.position_world_m;
        let attitude = self.attitude_model;
        [
            velocity.x,
            velocity.y,
            velocity.z,
            rates.x,
            rates.y,
            rates.z,
            world_velocity.x,
            world_velocity.y,
            world_velocity.z,
            position.x,
            position.y,
            position.z,
            attitude.w,
            attitude.i,
            attitude.j,
            attitude.k,
        ]
    }

    pub fn from_coordinates(coordinates: [f64; 16]) -> Self {
        let [u, v, w, p, q, r, vx, vy, vz, x, y, z, qw, qx, qy, qz] = coordinates;
        Self {
            velocity_model_mps: Vec3::new(u, v, w),
            rates_model_radps: Vec3::new(p, q, r),
            velocity_world_mps: Vec3::new(vx, vy, vz),
            position_world_m: Vec3::new(x, y, z),
            attitude_model: ModelQuaternion::new(qw, qx, qy, qz),
        }
    }
}

pub(super) fn derivative(
    state: AircraftState,
    acceleration_model_mps2: Vec3,
    angular_acceleration_model_radps2: Vec3,
) -> [f64; 16] {
    let velocity = state.velocity_model_mps;
    let rates = state.rates_model_radps;
    let attitude = state.attitude_model;
    let qw = attitude.w;
    let qx = attitude.i;
    let qy = attitude.j;
    let qz = attitude.k;
    let lambda = 1.0 - (qw * qw + qx * qx + qy * qy + qz * qz);
    let attitude_derivative = ModelQuaternion::new(
        -0.5 * (qx * rates.x + qy * rates.y + qz * rates.z) + lambda * qw,
        0.5 * (qw * rates.x + qy * rates.z - qz * rates.y) + lambda * qx,
        0.5 * (qw * rates.y + qz * rates.x - qx * rates.z) + lambda * qy,
        0.5 * (qw * rates.z + qx * rates.y - qy * rates.x) + lambda * qz,
    );
    let rotation = UnitQuaternion::new_normalize(attitude);
    let derivative = AircraftState {
        velocity_model_mps: velocity.cross(&rates) + acceleration_model_mps2,
        rates_model_radps: angular_acceleration_model_radps2,
        velocity_world_mps: flip_yz(rotation.transform_vector(&acceleration_model_mps2)),
        position_world_m: flip_yz(rotation.transform_vector(&velocity)),
        attitude_model: attitude_derivative,
    };
    derivative.coordinates()
}

#[cfg(test)]
mod tests {
    use std::f64::consts::FRAC_PI_2;

    use super::AircraftState;
    use crate::math::{EulerAngles, KinematicState, Quaternion, Vec3};

    #[test]
    fn model_rates_become_world_rates_at_the_truth_boundary() {
        for (yaw_rad, expected_world_radps) in [
            (0.0, Vec3::new(1.0, -2.0, -3.0)),
            (FRAC_PI_2, Vec3::new(2.0, 1.0, -3.0)),
        ] {
            let mut truth = KinematicState {
                orientation_world_from_body: Quaternion::from_euler(EulerAngles {
                    yaw_world_from_body_rad: yaw_rad,
                    ..EulerAngles::default()
                }),
                ..KinematicState::default()
            };
            let mut aircraft = AircraftState::from_truth(&truth);
            aircraft.rates_model_radps = Vec3::new(1.0, 2.0, 3.0);
            aircraft.write_truth(&mut truth);
            assert!((truth.angular_velocity_world_radps - expected_world_radps).norm() < 1e-12);
        }
    }

    #[test]
    fn world_rates_become_model_rates_when_reading_truth() {
        let truth = KinematicState {
            orientation_world_from_body: Quaternion::from_euler(EulerAngles {
                yaw_world_from_body_rad: FRAC_PI_2,
                ..EulerAngles::default()
            }),
            angular_velocity_world_radps: Vec3::new(2.0, 1.0, -3.0),
            ..KinematicState::default()
        };
        let aircraft = AircraftState::from_truth(&truth);
        assert!((aircraft.rates_model_radps - Vec3::new(1.0, 2.0, 3.0)).norm() < 1e-12);
    }

    #[test]
    fn tilted_aircraft_round_trips_world_state() {
        let original = KinematicState {
            position_world_m: Vec3::new(100.0, -20.0, 500.0),
            velocity_world_mps: Vec3::new(80.0, 30.0, -5.0),
            angular_velocity_world_radps: Vec3::new(0.3, -0.2, 0.1),
            orientation_world_from_body: Quaternion::from_euler(EulerAngles {
                roll_world_from_body_rad: 0.3,
                pitch_world_from_body_rad: -0.2,
                yaw_world_from_body_rad: 1.1,
            }),
        };
        let mut restored = KinematicState::default();
        AircraftState::from_truth(&original).write_truth(&mut restored);
        assert!(
            (restored.angular_velocity_world_radps - original.angular_velocity_world_radps).norm()
                < 1e-12
        );
        assert_eq!(restored.position_world_m, original.position_world_m);
        assert_eq!(restored.velocity_world_mps, original.velocity_world_mps);
        // Compare rotations, not quaternion signs: q and -q describe the same attitude.
        for axis in [Vec3::x(), Vec3::y(), Vec3::z()] {
            let expected = original
                .orientation_world_from_body
                .rotate_body_to_world(axis);
            let actual = restored
                .orientation_world_from_body
                .rotate_body_to_world(axis);
            assert!((actual - expected).norm() < 1e-12);
        }
    }
}
