//! FixedWing6DOF's coefficient equations in the forward/right/down model frame.
//! GTRI SCRIMMAGE reference: FixedWing6DOF.cpp (LGPL-3.0-or-later).

use std::f64::consts::PI;

use anyhow::Result;

use crate::{math::Vec3, plugin::PluginParams};

use super::Control;

/// Coefficient names deliberately match the aircraft equations and XML keys.
#[derive(Clone)]
pub(super) struct Aerodynamics {
    c_d0: f64,
    c_d_alpha: f64,
    c_d_elevator: f64,
    c_l0: f64,
    c_l_alpha: f64,
    c_lq: f64,
    c_l_elevator: f64,
    c_y_beta: f64,
    c_y_rudder: f64,
    c_roll_beta: f64,
    c_roll_p: f64,
    c_roll_r: f64,
    c_roll_aileron: f64,
    c_roll_rudder: f64,
    c_m0: f64,
    c_m_alpha: f64,
    c_mq: f64,
    c_m_elevator: f64,
    c_n_beta: f64,
    c_np: f64,
    c_nr: f64,
    c_n_aileron: f64,
    c_n_rudder: f64,
}

pub(super) struct Airframe {
    pub density_kgpm3: f64,
    pub span_m: f64,
    pub area_m2: f64,
    pub chord_m: f64,
    pub efficiency: f64,
}

pub(super) struct Loads {
    pub force_model_n: Vec3,
    pub moment_model_nm: Vec3,
}

impl Aerodynamics {
    pub fn parse(params: &PluginParams<'_>) -> Result<Self> {
        // Alpha-dot is always zero in the reference. Parse the values for basic
        // input checking, but do not invent a new unsteady aerodynamic model.
        params.number("C_L_alpha_dot", 0.72)?;
        params.number("C_M_alpha_dot", -1.1)?;
        Ok(Self {
            c_d0: params.number("C_D0", 0.03)?,
            c_d_alpha: params.number("C_D_alpha", 0.3)?,
            // Preserve the source's trailing-underscore key (not the XML typo).
            c_d_elevator: params.number("C_D_delta_elevator_", 0.01)?,
            c_l0: params.number("C_L0", 0.28)?,
            c_l_alpha: params.number("C_L_alpha", 3.45)?,
            c_lq: params.number("C_LQ", 0.0)?,
            c_l_elevator: params.number("C_L_delta_elevator", 0.36)?,
            c_y_beta: params.number("C_Y_beta", -0.98)?,
            c_y_rudder: params.number("C_Y_delta_rudder", 0.17)?,
            c_roll_beta: params.number("C_L_beta", -0.12)?,
            c_roll_p: params.number("C_LP", -0.26)?,
            c_roll_r: params.number("C_LR", 0.14)?,
            c_roll_aileron: params.number("C_L_delta_aileron", 0.08)?,
            c_roll_rudder: params.number("C_L_delta_rudder", -0.105)?,
            c_m0: params.number("C_M0", 0.0)?,
            c_m_alpha: params.number("C_M_alpha", -0.38)?,
            c_mq: params.number("C_MQ", -3.6)?,
            c_m_elevator: params.number("C_M_delta_elevator", -0.5)?,
            c_n_beta: params.number("C_N_beta", 0.25)?,
            c_np: params.number("C_NP", 0.022)?,
            c_nr: params.number("C_NR", -0.35)?,
            c_n_aileron: params.number("C_N_delta_aileron", 0.06)?,
            c_n_rudder: params.number("C_N_delta_rudder", 0.032)?,
        })
    }

    pub fn loads(
        &self,
        airframe: &Airframe,
        air_velocity_model_mps: Vec3,
        rates_model_radps: Vec3,
        control: Control,
    ) -> Loads {
        let mut airspeed_mps = air_velocity_model_mps.norm();
        if airspeed_mps.abs() < f64::EPSILON {
            airspeed_mps = 0.00001;
        }
        let alpha_rad = air_velocity_model_mps.z.atan2(air_velocity_model_mps.x);
        let beta_rad = (air_velocity_model_mps.y / airspeed_mps)
            .clamp(-1.0, 1.0)
            .asin();
        let twice_speed_mps = 2.0 * airspeed_mps;
        let span_m = airframe.span_m;
        let chord_m = airframe.chord_m;
        let aspect_ratio = span_m / chord_m;
        let pressure_area_n =
            airframe.density_kgpm3 * airspeed_mps.powi(2) * airframe.area_m2 / 2.0;
        let p_radps = rates_model_radps.x;
        let q_radps = rates_model_radps.y;
        let r_radps = rates_model_radps.z;

        let lift_coefficient = self.c_l0
            + self.c_l_alpha * alpha_rad
            + self.c_lq * q_radps * chord_m / twice_speed_mps
            + self.c_l_elevator * control.elevator_rad;
        let lift_n = lift_coefficient * pressure_area_n;
        let drag_n = (self.c_d0
            + self.c_d_alpha * alpha_rad.abs()
            + lift_coefficient * lift_coefficient / (PI * aspect_ratio * airframe.efficiency)
            + self.c_d_elevator * control.elevator_rad.abs())
            * pressure_area_n;
        let side_n =
            (self.c_y_beta * beta_rad + self.c_y_rudder * control.rudder_rad) * pressure_area_n;
        let force_model_n = Vec3::new(
            lift_n * alpha_rad.sin() - drag_n * alpha_rad.cos() - side_n * beta_rad.sin(),
            side_n * beta_rad.cos(),
            -lift_n * alpha_rad.cos() - drag_n * alpha_rad.sin(),
        );

        let roll_nm = (self.c_roll_beta * beta_rad
            + self.c_roll_p * p_radps * span_m / twice_speed_mps
            + self.c_roll_r * r_radps * span_m / twice_speed_mps
            + self.c_roll_aileron * control.aileron_rad
            + self.c_roll_rudder * control.rudder_rad)
            * pressure_area_n
            * span_m;
        let pitch_nm = (self.c_m0
            + self.c_m_alpha * alpha_rad
            + self.c_mq * q_radps * chord_m / twice_speed_mps
            + self.c_m_elevator * control.elevator_rad)
            * pressure_area_n
            * chord_m;
        let yaw_nm = (self.c_n_beta * beta_rad
            + self.c_np * p_radps * span_m / twice_speed_mps
            + self.c_nr * r_radps * span_m / twice_speed_mps
            + self.c_n_aileron * control.aileron_rad
            + self.c_n_rudder * control.rudder_rad)
            * pressure_area_n
            * span_m;
        Loads {
            force_model_n,
            moment_model_nm: Vec3::new(roll_nm, pitch_nm, yaw_nm),
        }
    }
}
