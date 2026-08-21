use flight_core::{Simulator, beginner_quad};
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
    pub fn set_motor(&mut self, index: usize, command: f64) {
        if let Some(m) = self.inner.vehicle.motors.get_mut(index) {
            m.command = command.clamp(0.0, 1.0);
        }
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
    pub fn state_json(&self) -> String {
        serde_json::json!({"time":self.inner.state.sim_time_s,"position":self.inner.state.position_ned_m.to_array(),"velocity":self.inner.state.velocity_ned_mps.to_array(),"attitude":self.inner.state.attitude_body_to_ned.to_array(),"forces":{"thrust":self.inner.forces.thrust.to_array(),"lift":self.inner.forces.lift.to_array(),"drag":self.inner.forces.drag.to_array()},"mass":self.inner.vehicle.mass(),"cg":self.inner.vehicle.center_of_mass().to_array()}).to_string()
    }
}
impl Default for FlightSimulator {
    fn default() -> Self {
        Self::new()
    }
}
