use flight_core::{
    Simulator,
    control::{AxisTuning, ControlConfig, DEG_TO_RAD, FlightMode},
    vehicle::VehicleDefinition,
};
use wasm_bindgen::prelude::*;
#[wasm_bindgen]
pub struct FlightSimulator {
    inner: Simulator,
    definition: VehicleDefinition,
}
#[wasm_bindgen]
impl FlightSimulator {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        let definition = VehicleDefinition::beginner();
        Self {
            inner: Simulator::new(definition.to_vehicle().expect("built-in preset")),
            definition,
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
        self.inner = Simulator::new(
            self.definition
                .to_vehicle()
                .expect("active definition was validated"),
        );
    }
    pub fn spawn(&mut self, airborne: bool) {
        self.reset();
        self.inner.state.position_ned_m.z = if airborne { -1.5 } else { 0.0 };
    }
    pub fn set_attitude_degrees(&mut self, roll: f64, pitch: f64, yaw: f64) {
        if [roll, pitch, yaw]
            .iter()
            .all(|value| value.is_finite() && value.abs() <= 180.0)
        {
            self.inner.state.attitude_body_to_ned = glam::DQuat::from_euler(
                glam::EulerRot::ZYX,
                yaw.to_radians(),
                pitch.to_radians(),
                roll.to_radians(),
            );
            self.inner.state.angular_rate_body_rps = glam::DVec3::ZERO;
            self.inner.controller.reset();
        }
    }
    pub fn preset_json(name: &str) -> String {
        let definition = if name == "freestyle" {
            VehicleDefinition::freestyle()
        } else {
            VehicleDefinition::beginner()
        };
        serde_json::to_string(&definition).expect("preset serializes")
    }
    pub fn vehicle_definition_json(&self) -> String {
        serde_json::to_string(&self.definition).expect("definition serializes")
    }
    pub fn configure_vehicle_json(&mut self, json: &str) -> Result<(), JsError> {
        if json.len() > 256_000 {
            return Err(JsError::new("vehicle file exceeds 256 KB"));
        }
        let definition: VehicleDefinition =
            serde_json::from_str(json).map_err(|_| JsError::new("invalid vehicle JSON"))?;
        definition.validate().map_err(|message| JsError::new(&message))?;
        let mode = self.inner.controller.mode;
        self.inner = Simulator::new(
            definition
                .to_vehicle()
                .map_err(|message| JsError::new(&message))?,
        );
        self.inner.controller.mode = mode;
        self.definition = definition;
        Ok(())
    }
    pub fn engineering_metrics_json(&self) -> String {
        serde_json::to_string(&self.definition.metrics().expect("active definition is validated"))
            .expect("metrics serialize")
    }
    pub fn state_json(&self) -> String {
        let t = self.inner.controller.telemetry;
        let (yaw, pitch, roll) = self
            .inner
            .state
            .attitude_body_to_ned
            .to_euler(glam::EulerRot::ZYX);
        let motors = self.inner.vehicle.motors.iter().enumerate().map(|(index,motor)| serde_json::json!({"positionBody":motor.position.to_array(),"forceNed":self.inner.forces.motor_thrust_ned.get(index).copied().unwrap_or_default().to_array()})).collect::<Vec<_>>();
        serde_json::json!({"time":self.inner.state.sim_time_s,"position":self.inner.state.position_ned_m.to_array(),"velocity":self.inner.state.velocity_ned_mps.to_array(),"attitude":self.inner.state.attitude_body_to_ned.to_array(),"euler":[roll,pitch,yaw],"rates":self.inner.state.angular_rate_body_rps.to_array(),"forces":{"gravity":self.inner.forces.gravity.to_array(),"thrust":self.inner.forces.thrust.to_array(),"lift":self.inner.forces.lift.to_array(),"drag":self.inner.forces.drag.to_array(),"motors":motors},"control":{"target":t.target_rate_rps.to_array(),"actual":t.actual_rate_rps.to_array(),"error":t.error_rps.to_array(),"output":t.output.to_array(),"throttle":t.throttle,"motors":t.motors,"actualMotors":self.inner.vehicle.motors.iter().map(|motor|motor.actual_output).collect::<Vec<_>>()},"battery":{"remainingMah":self.inner.battery_state.remaining_mah,"consumedWh":self.inner.battery_state.consumed_wh,"voltage":self.inner.battery_state.voltage_v,"current":self.inner.battery_state.current_a},"mass":self.inner.vehicle.mass(),"cg":self.inner.vehicle.center_of_mass().to_array(),"inertia":self.inner.vehicle.inertia_kg_m2.to_array()}).to_string()
    }
}
impl Default for FlightSimulator {
    fn default() -> Self {
        Self::new()
    }
}
