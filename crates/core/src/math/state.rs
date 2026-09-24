use super::{Quaternion, Vec3};
use serde::{Deserialize, Serialize};

/// Model-specific state remains private to the motion plugin.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct KinematicState {
    pub position_world_m: Vec3,
    pub orientation_world_from_body: Quaternion,
    pub velocity_world_mps: Vec3,
    pub angular_velocity_body_radps: Vec3,
}
