//! Rust-only open-loop motor test driver, not a flight controller.
//! Controller phase: write configured constant shaft speeds to motor_0, motor_1, ...
//! The reference fixture supplies the same commands to the upstream C++ motion model.

use anyhow::{Context, Result, ensure};

use crate::plugin::{
    AgentContext, Controller, Frame, Plugin, PluginIo, PluginParams, Port, Ports, Unit, Update,
};

#[derive(Clone)]
pub struct MotorSpeedsConfig {
    commands: Vec<MotorCommand>,
}

#[derive(Clone)]
struct MotorCommand {
    port: String,
    speed_radps: f64,
}

pub struct MotorSpeeds {
    config: MotorSpeedsConfig,
}

impl Plugin for MotorSpeeds {
    type Config = MotorSpeedsConfig;

    fn configure(params: &PluginParams<'_>) -> Result<Self::Config> {
        let text = params
            .text("speeds")
            .context("MotorSpeeds requires speeds in rad/s")?;
        let mut commands = Vec::new();
        for (index, value) in text.split_whitespace().enumerate() {
            let speed_radps: f64 = value.parse().context("invalid motor speed")?;
            ensure!(speed_radps.is_finite(), "motor speed must be finite");
            commands.push(MotorCommand {
                port: format!("motor_{index}"),
                speed_radps,
            });
        }
        ensure!(
            !commands.is_empty(),
            "MotorSpeeds requires at least one speed"
        );
        Ok(MotorSpeedsConfig { commands })
    }

    fn new(config: &Self::Config) -> Self {
        Self {
            config: config.clone(),
        }
    }

    fn ports(config: &Self::Config) -> Ports {
        let mut ports = Ports::default();
        for command in &config.commands {
            ports = ports.output(Port::new(
                &command.port,
                Unit::RadiansPerSecond,
                Frame::None,
            ));
        }
        ports
    }
}

impl Controller for MotorSpeeds {
    fn step(&mut self, _: &mut AgentContext<'_>, io: &mut PluginIo) -> Result<Update> {
        for command in &self.config.commands {
            io.write(&command.port, command.speed_radps)?;
        }
        Ok(Update::Applied)
    }
}
