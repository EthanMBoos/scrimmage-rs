//! Autonomy implementations selected by mission XML.
#[path = "autonomy/straight/straight.rs"]
mod straight;
pub use straight::Straight;
#[path = "autonomy/waypoint_follower/waypoint_follower.rs"]
mod waypoint_follower;
pub use waypoint_follower::WaypointFollower;
