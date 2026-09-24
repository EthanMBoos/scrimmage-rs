//! Pass ENU velocity commands to SingleIntegrator, matching the C++ controller.
//! Controller phase; inputs and outputs are separate even though their names match.

use anyhow::Result;

use crate::plugin::{
    AgentContext, Controller, Frame, Plugin, PluginIo, PluginParams, Port, Ports, Unit, Update,
};

pub struct SingleIntegratorControllerSimple;

impl Plugin for SingleIntegratorControllerSimple {
    type Config = ();
    fn configure(_: &PluginParams<'_>) -> Result<()> {
        Ok(())
    }
    fn new(_: &()) -> Self {
        Self
    }
    fn ports(_: &()) -> Ports {
        let mut ports = Ports::default();
        for name in ["velocity_x", "velocity_y", "velocity_z"] {
            let port = Port::new(name, Unit::MetersPerSecond, Frame::World);
            ports = ports.input(port).output(port);
        }
        ports
    }
}

impl Controller for SingleIntegratorControllerSimple {
    fn step(&mut self, _: &mut AgentContext<'_>, io: &mut PluginIo) -> Result<Update> {
        for name in ["velocity_x", "velocity_y", "velocity_z"] {
            io.write(name, io.read(name)?)?;
        }
        Ok(Update::Applied)
    }
}
