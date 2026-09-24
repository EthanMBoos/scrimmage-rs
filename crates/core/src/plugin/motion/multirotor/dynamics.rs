//! Multirotor integration coordinates: forward/left/up body, ENU world.
//! Unlike FixedWing6DOF, this model needs no body-axis flips.

use nalgebra::{Quaternion as BodyQuaternion, UnitQuaternion};

use crate::math::{KinematicState, Quaternion, Vec3};

#[derive(Clone, Copy)]
pub(super) struct RotorcraftState {
    pub velocity_body_mps: Vec3,
    pub rates_body_radps: Vec3,
    pub velocity_world_mps: Vec3,
    pub position_world_m: Vec3,
    pub attitude: BodyQuaternion<f64>,
}

impl Default for RotorcraftState {
    fn default() -> Self {
        Self {
            velocity_body_mps: Vec3::zeros(),
            rates_body_radps: Vec3::zeros(),
            velocity_world_mps: Vec3::zeros(),
            position_world_m: Vec3::zeros(),
            attitude: BodyQuaternion::identity(),
        }
    }
}

impl RotorcraftState {
    pub fn read_truth(&mut self, truth: &KinematicState) {
        // Matches C++ for now, though we don't consider it correct: body velocity and rates
        // stay private (initially zero). Position integrates body velocity while reported
        // world velocity integrates separately, so a nonzero initial or interaction-set
        // world velocity is reported but does not move the vehicle.
        let attitude = truth.orientation_world_from_body;
        self.attitude = BodyQuaternion::new(attitude.w, attitude.x, attitude.y, attitude.z);
        self.position_world_m = truth.position_world_m;
        self.velocity_world_mps = truth.velocity_world_mps;
    }

    pub fn write_truth(self, truth: &mut KinematicState) {
        let rotation = UnitQuaternion::new_normalize(self.attitude);
        let attitude = rotation.quaternion();
        truth.orientation_world_from_body = Quaternion {
            w: attitude.w,
            x: attitude.i,
            y: attitude.j,
            z: attitude.k,
        };
        truth.position_world_m = self.position_world_m;
        truth.velocity_world_mps = self.velocity_world_mps;
        truth.angular_velocity_world_radps = rotation.transform_vector(&self.rates_body_radps);
    }

    // Flatten only at the RK4 boundary; physical equations use the named fields.
    pub fn coordinates(self) -> [f64; 16] {
        let velocity = self.velocity_body_mps;
        let rates = self.rates_body_radps;
        let world_velocity = self.velocity_world_mps;
        let position = self.position_world_m;
        let attitude = self.attitude;
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
            velocity_body_mps: Vec3::new(u, v, w),
            rates_body_radps: Vec3::new(p, q, r),
            velocity_world_mps: Vec3::new(vx, vy, vz),
            position_world_m: Vec3::new(x, y, z),
            attitude: BodyQuaternion::new(qw, qx, qy, qz),
        }
    }
}

pub(super) fn derivative(
    state: RotorcraftState,
    acceleration_body_mps2: Vec3,
    angular_acceleration_body_radps2: Vec3,
) -> [f64; 16] {
    let rates = state.rates_body_radps;
    let attitude = state.attitude;
    let qw = attitude.w;
    let qx = attitude.i;
    let qy = attitude.j;
    let qz = attitude.k;
    let lambda = 1.0 - (qw * qw + qx * qx + qy * qy + qz * qz);
    let attitude_derivative = BodyQuaternion::new(
        -0.5 * (qx * rates.x + qy * rates.y + qz * rates.z) + lambda * qw,
        0.5 * (qw * rates.x + qy * rates.z - qz * rates.y) + lambda * qx,
        0.5 * (qw * rates.y + qz * rates.x - qx * rates.z) + lambda * qy,
        0.5 * (qw * rates.z + qx * rates.y - qy * rates.x) + lambda * qz,
    );
    let rotation = UnitQuaternion::new_normalize(attitude);
    RotorcraftState {
        velocity_body_mps: state.velocity_body_mps.cross(&rates) + acceleration_body_mps2,
        rates_body_radps: angular_acceleration_body_radps2,
        velocity_world_mps: rotation.transform_vector(&acceleration_body_mps2),
        position_world_m: rotation.transform_vector(&state.velocity_body_mps),
        attitude: attitude_derivative,
    }
    .coordinates()
}

#[cfg(test)]
mod tests {
    use super::RotorcraftState;
    use crate::math::{EulerAngles, KinematicState, Quaternion, Vec3};

    #[test]
    fn private_body_rates_are_rotated_when_writing_world_truth() {
        let orientation = Quaternion::from_euler(EulerAngles {
            roll_world_from_body_rad: 0.2,
            pitch_world_from_body_rad: -0.3,
            yaw_world_from_body_rad: 1.1,
        });
        let mut truth = KinematicState {
            orientation_world_from_body: orientation,
            ..KinematicState::default()
        };
        let mut state = RotorcraftState::default();
        state.read_truth(&truth);
        state.rates_body_radps = Vec3::new(1.0, 2.0, 3.0);
        state.write_truth(&mut truth);
        let expected = orientation.rotate_body_to_world(state.rates_body_radps);
        assert!((truth.angular_velocity_world_radps - expected).norm() < 1e-12);
    }
}
