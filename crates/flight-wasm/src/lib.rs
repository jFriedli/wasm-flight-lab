use flight_core::{
    Simulator, beginner_quad,
    control::{AxisTuning, ControlConfig, DEG_TO_RAD, FlightMode},
};
use wasm_bindgen::prelude::*;
#[wasm_bindgen]
pub struct FlightSimulator {
    inner: Simulator,
}
#[wasm_bindgen]
impl FlightSimulator {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self {
            inner: Simulator::new(beginner_quad()),
        }
    }
    pub fn step(&mut self, dt: f64) {
        self.inner.step(dt);
    }
    pub fn set_control(&mut self, roll: f64, pitch: f64, yaw: f64, throttle: f64) {
        self.inner
            .set_control(glam::DVec3::new(roll, pitch, yaw), throttle);
    }
    pub fn set_mode(&mut self, angle: bool) {
        self.inner.controller.mode = if angle {
            FlightMode::Angle
        } else {
            FlightMode::Acro
        };
        self.inner.controller.reset();
    }
    pub fn set_pid(&mut self, axis: usize, kp: f64, ki: f64, kd: f64) {
        let mut config = self.inner.controller.config;
        let tuning = AxisTuning { kp, ki, kd };
        match axis {
            0 => config.roll = tuning,
            1 => config.pitch = tuning,
            2 => config.yaw = tuning,
            _ => return,
        }
        self.inner.controller.set_config(config);
    }
    pub fn set_rates(&mut self, roll_dps: f64, pitch_dps: f64, yaw_dps: f64, max_angle_deg: f64) {
        let mut config = self.inner.controller.config;
        config.max_rate_rps = glam::DVec3::new(roll_dps, pitch_dps, yaw_dps) * DEG_TO_RAD;
        config.max_angle_rad = max_angle_deg * DEG_TO_RAD;
        self.inner.controller.set_config(config);
    }
    pub fn reset_defaults(&mut self) {
        self.inner.controller.set_config(ControlConfig::default());
        self.inner.controller.reset();
    }
    pub fn set_motor_position(&mut self, index: usize, x: f64, y: f64, z: f64) {
        if let Some(m) = self.inner.vehicle.motors.get_mut(index)
            && [x, y, z].iter().all(|v| v.is_finite() && v.abs() <= 10.0)
        {
            m.position = glam::DVec3::new(x, y, z);
        }
    }
    pub fn reset(&mut self) {
        self.inner = Simulator::new(beginner_quad());
    }
    pub fn spawn(&mut self, airborne: bool) {
        self.reset();
        self.inner.state.position_ned_m.z = if airborne { -1.5 } else { 0.0 };
    }
    pub fn state_json(&self) -> String {
        let t = self.inner.controller.telemetry;
        let (yaw, pitch, roll) = self
            .inner
            .state
            .attitude_body_to_ned
            .to_euler(glam::EulerRot::ZYX);
        serde_json::json!({"time":self.inner.state.sim_time_s,"position":self.inner.state.position_ned_m.to_array(),"velocity":self.inner.state.velocity_ned_mps.to_array(),"attitude":self.inner.state.attitude_body_to_ned.to_array(),"euler":[roll,pitch,yaw],"rates":self.inner.state.angular_rate_body_rps.to_array(),"forces":{"thrust":self.inner.forces.thrust.to_array(),"lift":self.inner.forces.lift.to_array(),"drag":self.inner.forces.drag.to_array()},"control":{"target":t.target_rate_rps.to_array(),"actual":t.actual_rate_rps.to_array(),"error":t.error_rps.to_array(),"output":t.output.to_array(),"throttle":t.throttle,"motors":t.motors},"mass":self.inner.vehicle.mass(),"cg":self.inner.vehicle.center_of_mass().to_array()}).to_string()
    }
}
impl Default for FlightSimulator {
    fn default() -> Self {
        Self::new()
    }
}
