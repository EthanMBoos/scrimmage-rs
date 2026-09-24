//! Drive until the position sensor reports that the goal has been reached.
use super::sensor::PositionReading;
use anyhow::Result;
use scrimmage_core::plugin::*;

pub struct DriveToGoal {
    goal_world_m: f64,
}
impl Plugin for DriveToGoal {
    type Config = f64;
    fn configure(params: &PluginParams<'_>) -> Result<f64> {
        params.number("goal_world_m", 1.0)
    }
    fn new(goal_world_m: &f64) -> Self {
        Self {
            goal_world_m: *goal_world_m,
        }
    }
    fn ports(_: &f64) -> Ports {
        Ports::default().output(Port::new("speed", Unit::MetersPerSecond, Frame::World))
    }
}
impl Autonomy for DriveToGoal {
    fn step(&mut self, context: &mut AgentContext<'_>, io: &mut PluginIo) -> Result<Update> {
        let position = context.observations.get::<PositionReading>("position")?;
        let reached_goal =
            position.is_some_and(|reading| reading.value.x_world_m >= self.goal_world_m);
        io.write("speed", if reached_goal { 0.0 } else { 2.0 })?;
        Ok(Update::Applied)
    }
}
