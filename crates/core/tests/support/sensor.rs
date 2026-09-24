//! Measure east position. A bias lets the mission distinguish belief from truth.
use anyhow::Result;
use scrimmage_core::plugin::*;

pub struct PositionReading {
    pub x_world_m: f64,
}
pub struct PositionSensor {
    bias_m: f64,
}
impl Plugin for PositionSensor {
    type Config = f64;
    fn configure(params: &PluginParams<'_>) -> Result<f64> {
        params.number("bias_m", 0.0)
    }
    fn new(bias_m: &f64) -> Self {
        Self { bias_m: *bias_m }
    }
}
impl Sensor for PositionSensor {
    fn step(&mut self, context: &mut SensorContext<'_>) -> Result<Update> {
        let measured_x_world_m = context.truth.position_world_m.x + self.bias_m;
        context.publish_local(
            "position",
            PositionReading {
                x_world_m: measured_x_world_m,
            },
        )?;
        Ok(Update::Applied)
    }
}
