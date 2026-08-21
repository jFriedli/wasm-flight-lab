//! Native-testable flight dynamics. SI units, right-handed NED world frame:
//! +X north, +Y east, +Z down. Body: +X forward, +Y right, +Z down.

use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};

pub mod control;
pub mod vehicle;
use control::FlightController;
use vehicle::{BatteryDefinition, VehicleDefinition};

pub const GRAVITY: f64 = 9.806_65;
pub const SEA_LEVEL_DENSITY: f64 = 1.225;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Motor {
    pub position: DVec3,
    pub direction: DVec3,
    pub max_thrust_n: f64,
    pub reaction_torque_nm: f64,
    pub spin: f64,
    pub command: f64,
    pub actual_output: f64,
    pub spin_up_time_s: f64,
    pub spin_down_time_s: f64,
    pub max_power_w: f64,
    pub effectiveness: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Wing {
    pub area_m2: f64,
    pub span_m: f64,
    pub incidence_rad: f64,
    pub position: DVec3,
    pub lift_slope: f64,
    pub stall_angle_rad: f64,
    pub cd0: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ComponentMass {
    pub mass_kg: f64,
    pub position: DVec3,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Vehicle {
    pub name: String,
    pub masses: Vec<ComponentMass>,
    pub motors: Vec<Motor>,
    pub wings: Vec<Wing>,
    pub inertia_kg_m2: DVec3,
    pub battery: BatteryDefinition,
}

impl Vehicle {
    pub fn mass(&self) -> f64 {
        self.masses.iter().map(|m| m.mass_kg).sum()
    }
    pub fn center_of_mass(&self) -> DVec3 {
        let mass = self.mass();
        if mass <= 0.0 {
            return DVec3::ZERO;
        }
        self.masses.iter().map(|m| m.position * m.mass_kg).sum::<DVec3>() / mass
    }
    pub fn validate(&self) -> Result<(), String> {
        if self.name.trim().is_empty() || self.name.len() > 80 {
            return Err("invalid vehicle name".into());
        }
        if self.masses.is_empty()
            || self.masses.len() > 128
            || self.motors.len() > 32
            || self.wings.len() > 16
        {
            return Err("invalid component count".into());
        }
        if !(0.05..=5000.0).contains(&self.mass()) {
            return Err("mass outside safe range".into());
        }
        let finite = |v: DVec3| v.is_finite() && v.abs().max_element() <= 1000.0;
        if self
            .masses
            .iter()
            .any(|m| !m.mass_kg.is_finite() || m.mass_kg <= 0.0 || !finite(m.position))
        {
            return Err("invalid mass component".into());
        }
        if self.motors.iter().any(|m| {
            !finite(m.position) || !finite(m.direction) || !(0.0..=100_000.0).contains(&m.max_thrust_n)
        }) {
            return Err("invalid motor".into());
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Environment {
    pub density_kg_m3: f64,
    pub wind_ned_mps: DVec3,
    pub gravity_mps2: f64,
}
impl Default for Environment {
    fn default() -> Self {
        Self {
            density_kg_m3: SEA_LEVEL_DENSITY,
            wind_ned_mps: DVec3::ZERO,
            gravity_mps2: GRAVITY,
        }
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct ForceBreakdown {
    pub gravity: DVec3,
    pub thrust: DVec3,
    pub lift: DVec3,
    pub drag: DVec3,
    pub torque_body: DVec3,
    pub motor_thrust_ned: Vec<DVec3>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct State {
    pub position_ned_m: DVec3,
    pub velocity_ned_mps: DVec3,
    pub attitude_body_to_ned: DQuat,
    pub angular_rate_body_rps: DVec3,
    pub sim_time_s: f64,
}
impl Default for State {
    fn default() -> Self {
        Self {
            position_ned_m: DVec3::new(0.0, 0.0, -1.5),
            velocity_ned_mps: DVec3::ZERO,
            attitude_body_to_ned: DQuat::IDENTITY,
            angular_rate_body_rps: DVec3::ZERO,
            sim_time_s: 0.0,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Pid {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub integral_limit: f64,
    pub output_limit: f64,
    integral: f64,
    previous_measurement: f64,
    filtered_derivative: f64,
}
impl Pid {
    pub fn new(kp: f64, ki: f64, kd: f64, integral_limit: f64, output_limit: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral_limit,
            output_limit,
            integral: 0.0,
            previous_measurement: 0.0,
            filtered_derivative: 0.0,
        }
    }
    pub fn set_gains(&mut self, kp: f64, ki: f64, kd: f64, output_limit: f64) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
        self.output_limit = output_limit;
    }
    pub fn update(&mut self, target: f64, actual: f64, dt: f64) -> f64 {
        if !target.is_finite() || !actual.is_finite() || !dt.is_finite() || dt <= 0.0 {
            return 0.0;
        }
        let e = target - actual;
        let derivative = -(actual - self.previous_measurement) / dt;
        let alpha = dt / (0.02 + dt);
        self.filtered_derivative += alpha * (derivative - self.filtered_derivative);
        self.previous_measurement = actual;
        let candidate = (self.integral + e * dt).clamp(-self.integral_limit, self.integral_limit);
        let unsaturated = self.kp * e + self.ki * candidate + self.kd * self.filtered_derivative;
        let saturated = unsaturated.clamp(-self.output_limit, self.output_limit);
        if unsaturated == saturated || e.signum() != unsaturated.signum() {
            self.integral = candidate;
        }
        saturated
    }
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_measurement = 0.0;
        self.filtered_derivative = 0.0;
    }
}

pub struct Simulator {
    pub vehicle: Vehicle,
    pub state: State,
    pub environment: Environment,
    pub forces: ForceBreakdown,
    pub controller: FlightController,
    pub battery_state: BatteryState,
    control_sticks: DVec3,
    throttle: f64,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct BatteryState {
    pub remaining_mah: f64,
    pub consumed_wh: f64,
    pub voltage_v: f64,
    pub current_a: f64,
}
impl Simulator {
    pub fn new(vehicle: Vehicle) -> Self {
        let battery_state = BatteryState {
            remaining_mah: vehicle.battery.capacity_mah,
            consumed_wh: 0.0,
            voltage_v: vehicle.battery.nominal_voltage(),
            current_a: 0.0,
        };
        Self {
            vehicle,
            state: State::default(),
            environment: Environment::default(),
            forces: ForceBreakdown::default(),
            controller: FlightController::default(),
            battery_state,
            control_sticks: DVec3::ZERO,
            throttle: 0.0,
        }
    }
    pub fn set_control(&mut self, sticks: DVec3, throttle: f64) {
        self.control_sticks = if sticks.is_finite() {
            sticks.clamp(DVec3::splat(-1.0), DVec3::ONE)
        } else {
            DVec3::ZERO
        };
        self.throttle = if throttle.is_finite() {
            throttle.clamp(0.0, 1.0)
        } else {
            0.0
        };
    }
    pub fn step(&mut self, dt: f64) {
        let dt = dt.clamp(0.0001, 0.02);
        let commands = self
            .controller
            .update(self.control_sticks, self.throttle, &self.state, dt);
        for (motor, command) in self.vehicle.motors.iter_mut().zip(commands) {
            motor.command = command;
            let tau = if command > motor.actual_output {
                motor.spin_up_time_s
            } else {
                motor.spin_down_time_s
            };
            let alpha = 1.0 - (-dt / tau.max(0.005)).exp();
            motor.actual_output += (command - motor.actual_output) * alpha;
        }
        let power_w = self
            .vehicle
            .motors
            .iter()
            .map(|motor| motor.max_power_w * motor.actual_output.powf(1.5))
            .sum::<f64>();
        let soc = (self.battery_state.remaining_mah / self.vehicle.battery.capacity_mah).clamp(0.0, 1.0);
        let open_voltage = f64::from(self.vehicle.battery.cells) * (3.3 + 0.9 * soc);
        self.battery_state.current_a = power_w / open_voltage.max(1.0);
        self.battery_state.voltage_v = (open_voltage
            - self.battery_state.current_a * self.vehicle.battery.internal_resistance_ohm)
            .max(f64::from(self.vehicle.battery.cells) * 2.8);
        self.battery_state.remaining_mah =
            (self.battery_state.remaining_mah - self.battery_state.current_a * dt / 3.6).max(0.0);
        self.battery_state.consumed_wh += power_w * dt / 3600.0;
        let mass = self.vehicle.mass();
        let q = self.state.attitude_body_to_ned;
        let com = self.vehicle.center_of_mass();
        let mut f = ForceBreakdown {
            gravity: DVec3::Z * mass * self.environment.gravity_mps2,
            ..Default::default()
        };
        for m in &self.vehicle.motors {
            let voltage_factor =
                (self.battery_state.voltage_v / self.vehicle.battery.nominal_voltage()).clamp(0.4, 1.1);
            let thrust = m.max_thrust_n
                * m.actual_output.clamp(0.0, 1.0)
                * m.effectiveness.clamp(0.0, 1.0)
                * (self.environment.density_kg_m3 / SEA_LEVEL_DENSITY)
                * voltage_factor;
            let fb = m.direction.normalize_or_zero() * thrust;
            f.thrust += q * fb;
            f.motor_thrust_ned.push(q * fb);
            f.torque_body += (m.position - com).cross(fb)
                + m.direction.normalize_or_zero() * m.reaction_torque_nm * m.spin * m.actual_output;
        }
        let air_ned = self.state.velocity_ned_mps - self.environment.wind_ned_mps;
        let air_body = q.conjugate() * air_ned;
        let speed = air_body.length();
        if speed > 0.01 {
            for w in &self.vehicle.wings {
                let alpha = (-air_body.z).atan2(air_body.x) + w.incidence_rad;
                let stall = (1.0 - (alpha.abs() / w.stall_angle_rad).powi(2)).clamp(0.08, 1.0);
                let cl = (w.lift_slope * alpha * stall).clamp(-1.4, 1.4);
                let dynp = 0.5 * self.environment.density_kg_m3 * speed * speed;
                let lift_body = DVec3::new(0.0, 0.0, -dynp * w.area_m2 * cl);
                let drag_body = -air_body.normalize() * dynp * w.area_m2 * (w.cd0 + 0.06 * cl * cl);
                f.lift += q * lift_body;
                f.drag += q * drag_body;
                f.torque_body += (w.position - com).cross(lift_body + drag_body);
            }
        }
        f.drag += -air_ned * air_ned.length() * 0.08 * self.environment.density_kg_m3;
        let accel = (f.gravity + f.thrust + f.lift + f.drag) / mass;
        self.state.velocity_ned_mps += accel * dt;
        self.state.position_ned_m += self.state.velocity_ned_mps * dt;
        let angular_accel = f.torque_body / self.vehicle.inertia_kg_m2.max(DVec3::splat(0.001));
        self.state.angular_rate_body_rps += angular_accel * dt;
        let dq = DQuat::from_scaled_axis(self.state.angular_rate_body_rps * dt);
        self.state.attitude_body_to_ned = (self.state.attitude_body_to_ned * dq).normalize();
        if self.state.position_ned_m.z > 0.0 {
            self.state.position_ned_m.z = 0.0;
            if self.state.velocity_ned_mps.z > 0.0 {
                self.state.velocity_ned_mps.z *= -0.15;
            }
            self.state.velocity_ned_mps.x *= 0.96;
            self.state.velocity_ned_mps.y *= 0.96;
        }
        self.state.sim_time_s += dt;
        self.forces = f;
    }
}

pub fn beginner_quad() -> Vehicle {
    VehicleDefinition::beginner()
        .to_vehicle()
        .expect("built-in preset must be valid")
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn center_of_mass_is_physical() {
        let mut v = beginner_quad();
        let old_mass = v.mass();
        let old_moment = v.center_of_mass().x * old_mass;
        v.masses.push(ComponentMass {
            mass_kg: 1.0,
            position: DVec3::X,
        });
        assert!((v.center_of_mass().x - (old_moment + 1.0) / (old_mass + 1.0)).abs() < 1e-9);
    }
    #[test]
    fn gravity_accelerates_down() {
        let mut v = beginner_quad();
        for m in &mut v.motors {
            m.command = 0.0;
        }
        let mut s = Simulator::new(v);
        s.step(0.01);
        assert!(s.state.velocity_ned_mps.z > 0.09);
    }
    #[test]
    fn symmetric_quad_has_near_zero_torque() {
        let mut vehicle = beginner_quad();
        vehicle.masses[1].position = DVec3::ZERO;
        vehicle.masses.last_mut().unwrap().position = DVec3::ZERO;
        let mut s = Simulator::new(vehicle);
        for motor in &mut s.vehicle.motors {
            motor.actual_output = 0.4;
        }
        s.set_control(DVec3::ZERO, 0.4);
        s.step(0.004);
        assert!(s.forces.torque_body.length() < 1e-10);
    }
    #[test]
    fn motor_failure_causes_torque() {
        let mut v = beginner_quad();
        v.motors[0].effectiveness = 0.0;
        let mut s = Simulator::new(v);
        for motor in &mut s.vehicle.motors {
            motor.actual_output = 0.4;
        }
        s.set_control(DVec3::ZERO, 0.4);
        s.step(0.004);
        assert!(s.forces.torque_body.length() > 0.1);
    }
    #[test]
    fn density_reduces_thrust() {
        let mut a = Simulator::new(beginner_quad());
        a.set_control(DVec3::ZERO, 0.4);
        a.step(0.004);
        let sea = a.forces.thrust.length();
        let mut b = Simulator::new(beginner_quad());
        b.set_control(DVec3::ZERO, 0.4);
        b.environment.density_kg_m3 = 0.8;
        b.step(0.004);
        assert!(b.forces.thrust.length() < sea);
    }
    #[test]
    fn pid_integral_is_bounded() {
        let mut p = Pid::new(0., 1., 0., 0.5, 0.5);
        for _ in 0..100 {
            p.update(10., 0., 0.1);
        }
        assert!(p.update(10., 0., 0.1) <= 0.5);
    }
    #[test]
    fn rejects_non_finite_import() {
        let mut v = beginner_quad();
        v.masses[0].mass_kg = f64::NAN;
        assert!(v.validate().is_err());
    }
    fn thrust_at(attitude: DQuat) -> DVec3 {
        let mut sim = Simulator::new(beginner_quad());
        sim.state.attitude_body_to_ned = attitude;
        for motor in &mut sim.vehicle.motors {
            motor.actual_output = 0.5;
        }
        sim.set_control(DVec3::ZERO, 0.5);
        sim.step(0.004);
        sim.forces.thrust
    }
    #[test]
    fn level_thrust_points_up_in_ned() {
        let thrust = thrust_at(DQuat::IDENTITY);
        assert!(thrust.x.abs() < 1e-12 && thrust.y.abs() < 1e-12);
        assert!(thrust.z < -15.0);
    }
    #[test]
    fn positive_roll_rotates_thrust_toward_positive_east() {
        let angle = 30_f64.to_radians();
        let thrust = thrust_at(DQuat::from_rotation_x(angle));
        let magnitude = thrust.length();
        assert!(thrust.x.abs() < 1e-12);
        assert!((thrust.y - magnitude * angle.sin()).abs() < 1e-10);
        assert!((thrust.z + magnitude * angle.cos()).abs() < 1e-10);
    }
    #[test]
    fn positive_pitch_rotates_thrust_toward_south() {
        let angle = 30_f64.to_radians();
        let thrust = thrust_at(DQuat::from_rotation_y(angle));
        let magnitude = thrust.length();
        assert!((thrust.x + magnitude * angle.sin()).abs() < 1e-10);
        assert!(thrust.y.abs() < 1e-12);
        assert!((thrust.z + magnitude * angle.cos()).abs() < 1e-10);
    }
    #[test]
    fn yaw_rotates_tilted_horizontal_thrust_in_ned() {
        let roll = 30_f64.to_radians();
        let yaw = 90_f64.to_radians();
        let attitude = DQuat::from_rotation_z(yaw) * DQuat::from_rotation_x(roll);
        let thrust = thrust_at(attitude);
        let magnitude = thrust.length();
        assert!((thrust.x + magnitude * roll.sin()).abs() < 1e-10);
        assert!(thrust.y.abs() < 1e-10);
        assert!((thrust.z + magnitude * roll.cos()).abs() < 1e-10);
    }
    #[test]
    fn arbitrary_attitude_matches_glam_body_to_world_rotation() {
        let attitude = DQuat::from_euler(glam::EulerRot::ZYX, 0.73, -0.31, 0.44);
        let actual = thrust_at(attitude);
        let expected = attitude * DVec3::new(0.0, 0.0, -actual.length());
        assert!((actual - expected).length() < 1e-10);
    }
    #[test]
    fn motor_output_has_first_order_lag() {
        let mut sim = Simulator::new(beginner_quad());
        sim.set_control(DVec3::ZERO, 1.0);
        sim.step(0.004);
        let first = sim.vehicle.motors[0].actual_output;
        assert!(first > 0.0 && first < 0.2);
        for _ in 0..300 {
            sim.step(0.004);
        }
        assert!(sim.vehicle.motors[0].actual_output > 0.9);
    }
    #[test]
    fn sustained_power_drains_battery() {
        let mut sim = Simulator::new(beginner_quad());
        let initial = sim.battery_state.remaining_mah;
        sim.set_control(DVec3::ZERO, 0.7);
        for _ in 0..500 {
            sim.step(0.004);
        }
        assert!(sim.battery_state.remaining_mah < initial);
        assert!(sim.battery_state.consumed_wh > 0.0);
        assert!(sim.battery_state.current_a > 0.0);
    }
    #[test]
    fn higher_motor_power_drains_energy_faster() {
        let run = |throttle| {
            let mut sim = Simulator::new(beginner_quad());
            sim.set_control(DVec3::ZERO, throttle);
            for _ in 0..500 {
                sim.step(0.004);
            }
            sim.battery_state.consumed_wh
        };
        assert!(run(0.8) > run(0.4) * 2.0);
    }
}
