//! A minimal controller demonstrating composable named input/output channels.
use anyhow::Result;
use scrimmage_core::plugin::*;

pub struct ScaleSpeed {
    gain: f64,
}
impl Plugin for ScaleSpeed {
    type Config = f64;
    fn configure(params: &PluginParams<'_>) -> Result<f64> {
        params.number("gain", 1.0)
    }
    fn new(gain: &f64) -> Self {
        Self { gain: *gain }
    }
    fn ports(_: &f64) -> Ports {
        let speed = Port::new("speed", Unit::MetersPerSecond, Frame::World);
        Ports::default().input(speed.clone()).output(speed)
    }
}
impl Controller for ScaleSpeed {
    fn step(&mut self, _: &mut AgentContext<'_>, io: &mut PluginIo) -> Result<Update> {
        let speed_world_mps = io.read("speed")?;
        io.write("speed", self.gain * speed_world_mps)?;
        Ok(Update::Applied)
    }
}
