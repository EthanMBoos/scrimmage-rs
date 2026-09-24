//! Nested heading/altitude and roll/pitch PID loops for FixedWing6DOF.
//!
//! C++ counterpart: controller/AircraftPIDController (GTRI, LGPL-3.0-or-later).
//! Controller phase: read desired commands and belief, output throttle and surfaces.

use anyhow::{Result, ensure};

use crate::common::{Pid, PidGains};
use crate::plugin::{
    AgentContext, Controller, Frame, Plugin, PluginIo, PluginParams, Port, Ports, Unit, Update,
};

pub struct AircraftPidConfig {
    heading: PidGains,
    altitude: PidGains,
    speed: PidGains,
    pitch: PidGains,
    roll: PidGains,
    max_pitch_rad: f64,
    max_roll_rad: f64,
    use_roll_control: bool,
}

pub struct AircraftPidController {
    heading: Pid,
    altitude: Pid,
    speed: Pid,
    pitch: Pid,
    roll: Pid,
    max_pitch_rad: f64,
    max_roll_rad: f64,
    use_roll_control: bool,
}

impl Plugin for AircraftPidController {
    type Config = AircraftPidConfig;

    fn configure(params: &PluginParams<'_>) -> Result<AircraftPidConfig> {
        let config = AircraftPidConfig {
            heading: gains(params, "heading_pid", [1.0, 0.2, 1.0, 45.0])?,
            altitude: gains(params, "altitude_pid", [0.1, 0.0001, 0.01, 10.0])?,
            speed: gains(params, "speed_pid", [1.0, 0.1, 0.03, 10.0])?,
            pitch: gains(params, "pitch_pid", [5.0, 0.01, 0.01, 9.0])?,
            roll: gains(params, "roll_pid", [0.3, 0.1, 0.001, 9.0])?,
            max_pitch_rad: params.number("max_pitch", 45.0)?.to_radians(),
            max_roll_rad: params.number("max_roll", 45.0)?.to_radians(),
            use_roll_control: params.boolean("use_roll_control", false)?,
        };
        ensure!(
            config.max_pitch_rad >= 0.0 && config.max_roll_rad >= 0.0,
            "AircraftPIDController attitude limits must be nonnegative"
        );
        Ok(config)
    }

    fn new(config: &AircraftPidConfig) -> Self {
        Self {
            heading: Pid::angular(config.heading),
            altitude: Pid::linear(config.altitude),
            speed: Pid::linear(config.speed),
            pitch: Pid::angular(config.pitch),
            roll: Pid::angular(config.roll),
            max_pitch_rad: config.max_pitch_rad,
            max_roll_rad: config.max_roll_rad,
            use_roll_control: config.use_roll_control,
        }
    }

    fn ports(config: &AircraftPidConfig) -> Ports {
        let steering = if config.use_roll_control {
            "desired_roll"
        } else {
            "desired_heading"
        };
        Ports::default()
            .input(Port::new(steering, Unit::Radians, Frame::World))
            .input(Port::new("desired_altitude", Unit::Meters, Frame::World))
            .input(Port::new(
                "desired_speed",
                Unit::MetersPerSecond,
                Frame::None,
            ))
            .output(Port::new("throttle", Unit::Dimensionless, Frame::None))
            .output(Port::new("elevator", Unit::Radians, Frame::Model))
            .output(Port::new("aileron", Unit::Radians, Frame::Model))
            .output(Port::new("rudder", Unit::Radians, Frame::Model))
    }
}

impl Controller for AircraftPidController {
    fn step(&mut self, context: &mut AgentContext<'_>, io: &mut PluginIo) -> Result<Update> {
        let state = context.state;
        let attitude = state.orientation_world_from_body;
        // Intentional C++ difference: use the supplied substep interval, not
        // time_->dt() (the full mission tick). Otherwise substeps overcount
        // integral time and undercount the derivative response.
        let dt_s = context.time.dt_s;
        let desired_pitch_rad =
            -self
                .altitude
                .step(io.read("desired_altitude")?, state.position_world_m.z, dt_s);
        let desired_pitch_rad = desired_pitch_rad.clamp(-self.max_pitch_rad, self.max_pitch_rad);
        let elevator_rad = self.pitch.step(
            desired_pitch_rad,
            attitude.pitch_world_from_body_rad(),
            dt_s,
        );

        let desired_roll_rad = if self.use_roll_control {
            io.read("desired_roll")?
        } else {
            -self.heading.step(
                io.read("desired_heading")?,
                attitude.yaw_world_from_body_rad(),
                dt_s,
            )
        };
        let desired_roll_rad = desired_roll_rad.clamp(-self.max_roll_rad, self.max_roll_rad);
        let aileron_rad =
            self.roll
                .step(desired_roll_rad, attitude.roll_world_from_body_rad(), dt_s);
        let throttle = self.speed.step(
            io.read("desired_speed")?,
            state.velocity_world_mps.norm(),
            dt_s,
        );

        io.write("elevator", elevator_rad.clamp(-1.0, 1.0))?;
        io.write("aileron", aileron_rad.clamp(-1.0, 1.0))?;
        io.write("throttle", throttle.clamp(-1.0, 1.0))?;
        io.write("rudder", 0.0)?;
        Ok(Update::Applied)
    }
}

fn gains(params: &PluginParams<'_>, key: &str, defaults: [f64; 4]) -> Result<PidGains> {
    let [proportional, integral, derivative, integral_band] = params.vector(key, defaults)?;
    Ok(PidGains {
        proportional,
        integral,
        derivative,
        integral_band,
    })
}

#[cfg(test)]
mod tests {
    use super::AircraftPidController;
    use crate::plugin::{
        AgentContext, Controller, EntityInfo, Messages, Observations, Plugin, PluginIo,
        PluginParams, StepTime,
    };
    use crate::{KinematicState, Params};

    #[test]
    fn pid_integral_and_derivative_use_the_supplied_substep_interval() -> anyhow::Result<()> {
        // Five 0.02-second updates cover one 0.1-second mission tick.
        // With constant speed error 1, I grows by 0.02 each update; D is
        // 0.01 / 0.02 = 0.5 on the first update and zero thereafter.
        let params = Params::from([("speed_pid".into(), "0,1,0.01,10".into())]);
        let config = AircraftPidController::configure(&PluginParams(&params))?;
        let mut controller = AircraftPidController::new(&config);
        let mut io = PluginIo::new(&AircraftPidController::ports(&config));
        io.receive(
            &[
                ("desired_heading", 0.0),
                ("desired_altitude", 0.0),
                ("desired_speed", 1.0),
            ]
            .into(),
        )?;
        let state = KinematicState::default();
        let observations = Observations::default();
        let mut messages = Messages::default();
        for (index, expected_throttle) in [0.52, 0.04, 0.06, 0.08, 0.10].into_iter().enumerate() {
            controller.step(
                &mut AgentContext {
                    entity: EntityInfo {
                        id: 1,
                        team_id: 1,
                        sub_swarm_id: 0,
                    },
                    time: StepTime {
                        time_s: index as f64 * 0.02,
                        dt_s: 0.02,
                    },
                    state: &state,
                    observations: &observations,
                    contacts_truth: &[],
                    messages: &mut messages,
                },
                &mut io,
            )?;
            assert!((io.outputs["throttle"] - expected_throttle).abs() < 1e-12);
        }
        Ok(())
    }
}
