use flight_core::{
    PropulsionRole, Simulator,
    atmosphere::{WeatherPreset, WindField},
    control::{AxisTuning, ControlConfig, DEG_TO_RAD, FlightMode},
    terrain::{DEFAULT_TERRAIN_SEED, TerrainDefinition, TerrainKind},
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
    pub fn set_transition(&mut self, transition: f64) {
        self.inner.set_transition(transition);
    }
    pub fn set_environment(&mut self, alpine: bool, seed: u32) {
        self.inner.environment.terrain = TerrainDefinition {
            kind: if alpine {
                TerrainKind::AlpineRange
            } else {
                TerrainKind::TestRange
            },
            seed,
        };
    }
    pub fn set_weather(&mut self, preset: &str) {
        let preset = match preset {
            "breeze" => WeatherPreset::Breeze,
            "alpine" => WeatherPreset::Alpine,
            "soaring" => WeatherPreset::Soaring,
            "strong" => WeatherPreset::StrongWind,
            _ => WeatherPreset::Calm,
        };
        self.inner.environment.wind = WindField::preset(preset);
    }
    pub fn set_custom_wind(
        &mut self,
        speed_mps: f64,
        direction_from_deg: f64,
        gust_strength_mps: f64,
        turbulence_strength_mps: f64,
    ) {
        let mut wind = WindField::preset(WeatherPreset::Custom);
        wind.speed_mps = finite(speed_mps, 0.0, 30.0);
        wind.direction_from_deg = finite(direction_from_deg, 0.0, 360.0);
        wind.gust_strength_mps = finite(gust_strength_mps, 0.0, 12.0);
        wind.turbulence_strength_mps = finite(turbulence_strength_mps, 0.0, 6.0);
        self.inner.environment.wind = wind;
    }
    pub fn terrain_height(&self, north_m: f64, east_m: f64) -> f64 {
        if !north_m.is_finite() || !east_m.is_finite() {
            return 0.0;
        }
        self.inner.environment.terrain.elevation_m(north_m, east_m)
    }
    pub fn terrain_normal_json(&self, north_m: f64, east_m: f64) -> String {
        let normal = self.inner.environment.terrain.normal_up_ned(north_m, east_m);
        serde_json::to_string(&normal.to_array()).expect("normal serializes")
    }
    pub fn default_terrain_seed() -> u32 {
        DEFAULT_TERRAIN_SEED
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
        let environment = self.inner.environment.clone();
        self.inner = Simulator::new(
            self.definition
                .to_vehicle()
                .expect("active definition was validated"),
        );
        self.inner.environment = environment;
    }
    pub fn spawn(&mut self, airborne: bool) {
        self.reset();
        self.inner.state.position_ned_m.z = if airborne {
            if self.inner.vehicle.class == flight_core::VehicleClass::Multicopter {
                -1.5
            } else {
                -25.0
            }
        } else {
            0.0
        };
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
        let definition = match name {
            "freestyle" => VehicleDefinition::freestyle(),
            "trainer" => VehicleDefinition::fixed_wing_trainer(),
            "quadplane" => VehicleDefinition::quadplane(),
            "tiltrotor" => VehicleDefinition::tiltrotor(),
            _ => VehicleDefinition::beginner(),
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
        let motors = self
            .inner
            .vehicle
            .motors
            .iter()
            .enumerate()
            .map(|(index, motor)| {
                serde_json::json!({
                    "positionBody": motor.position.to_array(),
                    "forceNed": self.inner.forces.motor_thrust_ned.get(index).copied().unwrap_or_default().to_array()
                })
            })
            .collect::<Vec<_>>();
        let surfaces = self
            .inner
            .forces
            .surface_forces
            .iter()
            .map(|surface| {
                serde_json::json!({
                    "name": surface.name,
                    "positionBody": surface.position_body.to_array(),
                    "liftNed": surface.lift_ned.to_array(),
                    "dragNed": surface.drag_ned.to_array(),
                    "angle": surface.angle_rad,
                    "cl": surface.cl,
                    "cd": surface.cd,
                    "stalled": surface.stalled,
                    "commandedDeflection": surface.commanded_deflection_rad,
                    "actualDeflection": surface.actual_deflection_rad
                })
            })
            .collect::<Vec<_>>();
        let terrain_height = self.inner.environment.terrain.elevation_m(
            self.inner.state.position_ned_m.x,
            self.inner.state.position_ned_m.y,
        );
        let (transition_command, transition_actual) = self.inner.transition();
        let tilt_angle = self
            .inner
            .vehicle
            .motors
            .iter()
            .find(|motor| motor.role == PropulsionRole::Tilt)
            .map(|motor| motor.actual_tilt_rad)
            .unwrap_or(0.0);
        let vertical_propulsion = self
            .inner
            .vehicle
            .motors
            .iter()
            .map(|motor| motor.max_thrust_n * motor.actual_output * (-motor.direction.z).max(0.0))
            .sum::<f64>();
        let forward_propulsion = self
            .inner
            .vehicle
            .motors
            .iter()
            .map(|motor| motor.max_thrust_n * motor.actual_output * motor.direction.x.max(0.0))
            .sum::<f64>();
        let airspeed = self.inner.forces.air_velocity_ned.length();
        let weight = self.inner.vehicle.mass() * self.inner.environment.gravity_mps2;
        let wing_support_fraction = (-self.inner.forces.lift.z / weight).clamp(0.0, 2.0);
        let vertical_reserve = vertical_propulsion - (weight + self.inner.forces.lift.z).max(0.0);
        let regime = if transition_command < transition_actual - 0.05 {
            "BACK TRANSITION"
        } else if transition_actual < 0.2 && airspeed < 5.0 {
            "HOVER"
        } else if transition_actual < 0.2 && airspeed < 10.0 {
            "ACCELERATING"
        } else if transition_actual < 0.35 && airspeed >= 10.0 {
            "TRANSITION READY"
        } else if transition_actual > 0.8 && airspeed > 10.0 {
            "WING BORNE"
        } else {
            "TRANSITION"
        };
        serde_json::json!({
            "time": self.inner.state.sim_time_s,
            "vehicleClass": format!("{:?}", self.inner.vehicle.class),
            "position": self.inner.state.position_ned_m.to_array(),
            "velocity": self.inner.state.velocity_ned_mps.to_array(),
            "attitude": self.inner.state.attitude_body_to_ned.to_array(),
            "euler": [roll, pitch, yaw],
            "rates": self.inner.state.angular_rate_body_rps.to_array(),
            "airVelocity": self.inner.forces.air_velocity_ned.to_array(),
            "wind": {"combined": self.inner.forces.wind.combined_ned.to_array(), "base": self.inner.forces.wind.base_ned.to_array(), "gust": self.inner.forces.wind.gust_ned.to_array(), "turbulence": self.inner.forces.wind.turbulence_ned.to_array(), "terrain": self.inner.forces.wind.terrain_ned.to_array(), "thermal": self.inner.forces.wind.thermal_ned.to_array()},
            "angleOfAttack": self.inner.forces.angle_of_attack_rad,
            "stalled": self.inner.forces.stalled,
            "terrainHeight": terrain_height,
            "transition": {"command": transition_command, "actual": transition_actual, "tiltAngle": tilt_angle, "regime": regime, "verticalThrust": vertical_propulsion, "forwardThrust": forward_propulsion, "wingSupportFraction": wing_support_fraction, "verticalThrustReserve": vertical_reserve},
            "forces": {"gravity": self.inner.forces.gravity.to_array(), "thrust": self.inner.forces.thrust.to_array(), "lift": self.inner.forces.lift.to_array(), "drag": self.inner.forces.drag.to_array(), "propulsionTorqueBody": self.inner.forces.propulsion_torque_body.to_array(), "aerodynamicTorqueBody": self.inner.forces.aerodynamic_torque_body.to_array(), "totalTorqueBody": self.inner.forces.torque_body.to_array(), "motors": motors, "surfaces": surfaces},
            "control": {"target": t.target_rate_rps.to_array(), "actual": t.actual_rate_rps.to_array(), "error": t.error_rps.to_array(), "output": t.output.to_array(), "throttle": self.inner.throttle(), "sticks": self.inner.control_sticks().to_array(), "motors": t.motors, "actualMotors": self.inner.vehicle.motors.iter().map(|motor| motor.actual_output).collect::<Vec<_>>()},
            "battery": {"remainingMah": self.inner.battery_state.remaining_mah, "consumedWh": self.inner.battery_state.consumed_wh, "voltage": self.inner.battery_state.voltage_v, "current": self.inner.battery_state.current_a},
            "mass": self.inner.vehicle.mass(),
            "cg": self.inner.vehicle.center_of_mass().to_array(),
            "inertia": self.inner.vehicle.inertia_kg_m2.to_array()
        })
        .to_string()
    }
}

fn finite(value: f64, min: f64, max: f64) -> f64 {
    if value.is_finite() {
        value.clamp(min, max)
    } else {
        min
    }
}
impl Default for FlightSimulator {
    fn default() -> Self {
        Self::new()
    }
}
