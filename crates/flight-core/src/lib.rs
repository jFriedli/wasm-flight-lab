//! Native-testable flight dynamics. SI units, right-handed NED world frame:
//! +X north, +Y east, +Z down. Body: +X forward, +Y right, +Z down.

use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};

pub mod atmosphere;
pub mod control;
pub mod terrain;
pub mod vehicle;
use atmosphere::{WindField, WindSample};
use control::FlightController;
use terrain::TerrainDefinition;
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
    pub role: PropulsionRole,
    pub hover_direction: DVec3,
    pub actual_tilt_rad: f64,
    pub max_tilt_rad: f64,
    pub tilt_rate_rad_s: f64,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum PropulsionRole {
    Lift,
    Forward,
    Tilt,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum VehicleClass {
    Multicopter,
    FixedWing,
    QuadPlane,
    Tiltrotor,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub enum SurfacePlane {
    Horizontal,
    Vertical,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub enum ControlAxis {
    None,
    Roll,
    Pitch,
    Yaw,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AeroSurface {
    pub name: String,
    pub area_m2: f64,
    pub span_m: f64,
    pub chord_m: f64,
    pub incidence_rad: f64,
    pub position: DVec3,
    pub plane: SurfacePlane,
    pub lift_slope: f64,
    pub stall_angle_rad: f64,
    pub cd0: f64,
    pub induced_drag_k: f64,
    pub control_axis: ControlAxis,
    pub control_sign: f64,
    pub max_deflection_rad: f64,
    pub control_effectiveness: f64,
    pub trim_deflection_rad: f64,
    pub servo_rate_rad_s: f64,
    pub commanded_deflection_rad: f64,
    pub actual_deflection_rad: f64,
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
    pub class: VehicleClass,
    pub aero_surfaces: Vec<AeroSurface>,
    pub inertia_kg_m2: DVec3,
    pub battery: BatteryDefinition,
    /// Full body dimensions in body X/Y/Z, used for projected-area drag.
    pub body_dimensions_m: DVec3,
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
            || self.aero_surfaces.len() > 16
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
    pub wind: WindField,
    pub gravity_mps2: f64,
    pub terrain: TerrainDefinition,
}
impl Default for Environment {
    fn default() -> Self {
        Self {
            density_kg_m3: SEA_LEVEL_DENSITY,
            wind: WindField::default(),
            gravity_mps2: GRAVITY,
            terrain: TerrainDefinition::default(),
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
    pub propulsion_torque_body: DVec3,
    pub aerodynamic_torque_body: DVec3,
    pub motor_thrust_ned: Vec<DVec3>,
    pub surface_forces: Vec<SurfaceForce>,
    pub air_velocity_ned: DVec3,
    pub wind: WindSample,
    pub angle_of_attack_rad: f64,
    pub stalled: bool,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct SurfaceForce {
    pub name: String,
    pub position_body: DVec3,
    pub lift_ned: DVec3,
    pub drag_ned: DVec3,
    pub angle_rad: f64,
    pub cl: f64,
    pub cd: f64,
    pub stalled: bool,
    pub commanded_deflection_rad: f64,
    pub actual_deflection_rad: f64,
}

/// Velocity of a point fixed to the rigid body, relative to the air and
/// expressed in body axes. The offset is measured from the current CG.
pub fn local_air_velocity_body(
    cg_air_velocity_body: DVec3,
    angular_rate_body: DVec3,
    offset_body: DVec3,
) -> DVec3 {
    cg_air_velocity_body + angular_rate_body.cross(offset_body)
}

/// Bluff-body drag from the area projected normal to each body axis.
pub fn body_drag_force_ned(
    air_velocity_ned: DVec3,
    attitude_body_to_ned: DQuat,
    dimensions_m: DVec3,
    density_kg_m3: f64,
    drag_coefficient: f64,
) -> DVec3 {
    let air_body = attitude_body_to_ned.conjugate() * air_velocity_ned;
    let area = DVec3::new(
        dimensions_m.y * dimensions_m.z,
        dimensions_m.x * dimensions_m.z,
        dimensions_m.x * dimensions_m.y,
    );
    let drag_body = -0.5 * density_kg_m3 * drag_coefficient * area * air_body.abs() * air_body;
    attitude_body_to_ned * drag_body
}

pub fn aerodynamic_force(
    surface: &AeroSurface,
    air_body: DVec3,
    density: f64,
    deflection_rad: f64,
) -> (DVec3, DVec3, f64, f64, f64, bool) {
    let planar_speed = match surface.plane {
        SurfacePlane::Horizontal => DVec3::new(air_body.x, 0.0, air_body.z).length(),
        SurfacePlane::Vertical => DVec3::new(air_body.x, air_body.y, 0.0).length(),
    };
    if planar_speed < 1e-6 || !planar_speed.is_finite() {
        return (DVec3::ZERO, DVec3::ZERO, 0.0, 0.0, surface.cd0, false);
    }
    let flow_angle = match surface.plane {
        SurfacePlane::Horizontal => air_body.z.atan2(air_body.x),
        SurfacePlane::Vertical => air_body.y.atan2(air_body.x),
    };
    let effective_deflection = deflection_rad.clamp(-surface.max_deflection_rad, surface.max_deflection_rad)
        * surface.control_effectiveness;
    let alpha = flow_angle + surface.incidence_rad + effective_deflection;
    let critical = surface.stall_angle_rad.max(0.01);
    let linear_cl = surface.lift_slope * alpha;
    let stalled = alpha.abs() > critical;
    let cl = if stalled {
        let peak = surface.lift_slope * critical * alpha.signum();
        peak * (critical / alpha.abs()).powf(1.2)
    } else {
        linear_cl
    };
    let excess = ((alpha.abs() - critical) / critical).max(0.0);
    let cd = surface.cd0 + surface.induced_drag_k * cl * cl + 1.2 * excess * excess;
    let q = 0.5 * density.max(0.0) * planar_speed * planar_speed;
    let lift_direction = match surface.plane {
        SurfacePlane::Horizontal => DVec3::new(air_body.z, 0.0, -air_body.x).normalize_or_zero(),
        SurfacePlane::Vertical => DVec3::new(air_body.y, -air_body.x, 0.0).normalize_or_zero(),
    };
    let lift = lift_direction * q * surface.area_m2 * cl;
    let drag = -air_body.normalize_or_zero() * q * surface.area_m2 * cd;
    (lift, drag, alpha, cl, cd, stalled)
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
    transition_command: f64,
    transition_actual: f64,
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
            transition_command: 0.0,
            transition_actual: 0.0,
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
    pub fn control_sticks(&self) -> DVec3 {
        self.control_sticks
    }
    pub fn throttle(&self) -> f64 {
        self.throttle
    }
    pub fn set_transition(&mut self, transition: f64) {
        self.transition_command = if transition.is_finite() {
            transition.clamp(0.0, 1.0)
        } else {
            0.0
        };
    }
    pub fn transition(&self) -> (f64, f64) {
        (self.transition_command, self.transition_actual)
    }
    pub fn step(&mut self, dt: f64) {
        let dt = dt.clamp(0.0001, 0.02);
        let transition_rate = if self.vehicle.class == VehicleClass::Tiltrotor {
            self.vehicle
                .motors
                .iter()
                .filter(|motor| motor.role == PropulsionRole::Tilt)
                .map(|motor| motor.tilt_rate_rad_s / motor.max_tilt_rad.max(0.01))
                .fold(f64::INFINITY, f64::min)
                .min(0.5)
        } else {
            0.22
        };
        self.transition_actual += (self.transition_command - self.transition_actual)
            .clamp(-transition_rate * dt, transition_rate * dt);
        for motor in &mut self.vehicle.motors {
            if motor.role == PropulsionRole::Tilt {
                motor.actual_tilt_rad = self.transition_actual * motor.max_tilt_rad;
                motor.direction = DVec3::new(motor.actual_tilt_rad.sin(), 0.0, -motor.actual_tilt_rad.cos());
            }
        }
        let commands = match self.vehicle.class {
            VehicleClass::Multicopter => self
                .controller
                .update(self.control_sticks, self.throttle, &self.state, dt)
                .to_vec(),
            VehicleClass::FixedWing => vec![self.throttle; self.vehicle.motors.len()],
            VehicleClass::QuadPlane => {
                let airspeed_support = ((self.forces.air_velocity_ned.length() - 5.0) / 10.0).clamp(0.0, 1.0);
                let lift_collective =
                    self.throttle * (1.0 - 0.85 * self.transition_actual * airspeed_support);
                let lift = self
                    .controller
                    .update(self.control_sticks, lift_collective, &self.state, dt);
                let mut lift_index = 0;
                self.vehicle
                    .motors
                    .iter()
                    .map(|motor| match motor.role {
                        PropulsionRole::Forward => self.throttle * self.transition_actual,
                        _ => {
                            let command = lift.get(lift_index).copied().unwrap_or(lift_collective);
                            lift_index += 1;
                            command
                        }
                    })
                    .collect()
            }
            VehicleClass::Tiltrotor => {
                let mixed = self
                    .controller
                    .update(self.control_sticks, self.throttle, &self.state, dt);
                mixed
                    .into_iter()
                    .map(|command| self.throttle + (command - self.throttle) * (1.0 - self.transition_actual))
                    .collect()
            }
        };
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
            let motor_torque = (m.position - com).cross(fb)
                + m.direction.normalize_or_zero() * m.reaction_torque_nm * m.spin * m.actual_output;
            f.torque_body += motor_torque;
            f.propulsion_torque_body += motor_torque;
        }
        f.wind = self.environment.wind.sample(
            self.state.position_ned_m,
            self.state.sim_time_s,
            &self.environment.terrain,
        );
        let air_ned = self.state.velocity_ned_mps - f.wind.combined_ned;
        f.air_velocity_ned = air_ned;
        let air_body = q.conjugate() * air_ned;
        let aerodynamic_controls = if matches!(
            self.vehicle.class,
            VehicleClass::QuadPlane | VehicleClass::Tiltrotor
        ) {
            // The same rate/attitude controller effort that allocates rotor
            // differential is handed to aerodynamic surfaces. Dynamic
            // pressure naturally fades this authority in and out.
            self.controller.telemetry.output / self.controller.config.output_limit.max(1e-6)
        } else {
            self.control_sticks
        };
        for surface in &mut self.vehicle.aero_surfaces {
            let control = match surface.control_axis {
                ControlAxis::None => 0.0,
                ControlAxis::Roll => aerodynamic_controls.x,
                ControlAxis::Pitch => aerodynamic_controls.y,
                ControlAxis::Yaw => aerodynamic_controls.z,
            };
            let trim_scale = match self.vehicle.class {
                VehicleClass::FixedWing => 1.0,
                VehicleClass::QuadPlane | VehicleClass::Tiltrotor => self.transition_actual,
                VehicleClass::Multicopter => 0.0,
            };
            surface.commanded_deflection_rad = surface.trim_deflection_rad * trim_scale
                + control.clamp(-1.0, 1.0) * surface.control_sign * surface.max_deflection_rad;
            surface.commanded_deflection_rad = surface
                .commanded_deflection_rad
                .clamp(-surface.max_deflection_rad, surface.max_deflection_rad);
            let max_servo_step = surface.servo_rate_rad_s * dt;
            surface.actual_deflection_rad += (surface.commanded_deflection_rad
                - surface.actual_deflection_rad)
                .clamp(-max_servo_step, max_servo_step);
            let local_air_body =
                local_air_velocity_body(air_body, self.state.angular_rate_body_rps, surface.position - com)
                    .clamp_length_max(150.0);
            let (lift_body, drag_body, angle, cl, cd, stalled) = aerodynamic_force(
                surface,
                local_air_body,
                self.environment.density_kg_m3,
                surface.actual_deflection_rad,
            );
            let lift_ned = q * lift_body;
            let drag_ned = q * drag_body;
            f.lift += lift_ned;
            f.drag += drag_ned;
            let aerodynamic_torque = (surface.position - com).cross(lift_body + drag_body);
            f.torque_body += aerodynamic_torque;
            f.aerodynamic_torque_body += aerodynamic_torque;
            if matches!(surface.plane, SurfacePlane::Horizontal) && surface.area_m2 > 0.15 {
                f.angle_of_attack_rad = angle;
                f.stalled |= stalled;
            }
            f.surface_forces.push(SurfaceForce {
                name: surface.name.clone(),
                position_body: surface.position,
                lift_ned,
                drag_ned,
                angle_rad: angle,
                cl,
                cd,
                stalled,
                commanded_deflection_rad: surface.commanded_deflection_rad,
                actual_deflection_rad: surface.actual_deflection_rad,
            });
        }
        let body_cd = if self.vehicle.class == VehicleClass::Multicopter {
            0.9
        } else {
            0.35
        };
        f.drag += body_drag_force_ned(
            air_ned,
            q,
            self.vehicle.body_dimensions_m,
            self.environment.density_kg_m3,
            body_cd,
        );
        let accel = (f.gravity + f.thrust + f.lift + f.drag) / mass;
        self.state.velocity_ned_mps += accel * dt;
        self.state.position_ned_m += self.state.velocity_ned_mps * dt;
        let angular_accel = f.torque_body / self.vehicle.inertia_kg_m2.max(DVec3::splat(0.001));
        self.state.angular_rate_body_rps += angular_accel * dt;
        self.state.angular_rate_body_rps = self.state.angular_rate_body_rps.clamp_length_max(50.0);
        self.state.velocity_ned_mps = self.state.velocity_ned_mps.clamp_length_max(250.0);
        let dq = DQuat::from_scaled_axis(self.state.angular_rate_body_rps * dt);
        self.state.attitude_body_to_ned = (self.state.attitude_body_to_ned * dq).normalize();
        let ground_down = self
            .environment
            .terrain
            .ground_down_m(self.state.position_ned_m.x, self.state.position_ned_m.y);
        if self.state.position_ned_m.z > ground_down {
            self.state.position_ned_m.z = ground_down;
            let normal = self
                .environment
                .terrain
                .normal_up_ned(self.state.position_ned_m.x, self.state.position_ned_m.y);
            let normal_speed = self.state.velocity_ned_mps.dot(normal);
            if normal_speed < 0.0 {
                self.state.velocity_ned_mps -= normal * normal_speed * 1.15;
            }
            let ground_damping = if self.vehicle.class == VehicleClass::FixedWing {
                0.9995
            } else {
                0.96
            };
            self.state.velocity_ned_mps.x *= ground_damping;
            self.state.velocity_ned_mps.y *= ground_damping;
            if self.vehicle.class == VehicleClass::FixedWing && normal.z < -0.98 {
                let (_, pitch, _) = self.state.attitude_body_to_ned.to_euler(glam::EulerRot::ZYX);
                self.state.attitude_body_to_ned = DQuat::from_rotation_y(pitch.clamp(-0.08, 0.22));
                self.state.angular_rate_body_rps.x = 0.0;
                self.state.angular_rate_body_rps.y *= 0.9;
                self.state.angular_rate_body_rps.z = 0.0;
            }
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
    fn aircraft_collides_with_authoritative_mountain_height() {
        let mut sim = Simulator::new(beginner_quad());
        let north = 1_850.0;
        let east = 1_650.0;
        let ground = sim.environment.terrain.ground_down_m(north, east);
        assert!(ground < -500.0);
        sim.state.position_ned_m = DVec3::new(north, east, ground + 10.0);
        sim.state.velocity_ned_mps = DVec3::new(0.0, 0.0, 5.0);
        sim.step(0.004);
        assert!((sim.state.position_ned_m.z - ground).abs() < 1e-9);
        assert!(sim.state.velocity_ned_mps.is_finite());
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
    fn trainer_surface(name: &str) -> AeroSurface {
        VehicleDefinition::fixed_wing_trainer()
            .to_vehicle()
            .unwrap()
            .aero_surfaces
            .into_iter()
            .find(|s| s.name == name)
            .unwrap()
    }
    #[test]
    fn wing_zero_speed_has_zero_force() {
        let (lift, drag, ..) =
            aerodynamic_force(&trainer_surface("Left Wing"), DVec3::ZERO, SEA_LEVEL_DENSITY, 0.0);
        assert!(lift.length() < 1e-12 && drag.length() < 1e-12);
    }
    #[test]
    fn wing_force_scales_with_speed_squared() {
        let wing = trainer_surface("Left Wing");
        let at = |speed| {
            aerodynamic_force(
                &wing,
                DVec3::new(speed, 0.0, speed * 0.08),
                SEA_LEVEL_DENSITY,
                0.0,
            )
            .0
            .length()
        };
        let ratio = at(20.0) / at(10.0);
        assert!((ratio - 4.0).abs() < 1e-9);
    }
    #[test]
    fn angle_of_attack_sets_lift_sign() {
        let wing = trainer_surface("Left Wing");
        let positive = aerodynamic_force(&wing, DVec3::new(15.0, 0.0, 1.0), SEA_LEVEL_DENSITY, 0.0).0;
        let negative = aerodynamic_force(&wing, DVec3::new(15.0, 0.0, -1.5), SEA_LEVEL_DENSITY, 0.0).0;
        assert!(positive.z < 0.0 && negative.z > 0.0);
    }
    #[test]
    fn stall_reduces_cl_and_increases_drag() {
        let wing = trainer_surface("Left Wing");
        let normal = aerodynamic_force(&wing, DVec3::new(15.0, 0.0, 3.0), SEA_LEVEL_DENSITY, 0.0);
        let stalled = aerodynamic_force(&wing, DVec3::new(15.0, 0.0, 8.0), SEA_LEVEL_DENSITY, 0.0);
        assert!(stalled.5 && stalled.3.abs() < normal.3.abs());
        assert!(stalled.4 > normal.4);
    }
    #[test]
    fn stalled_trainer_loses_vertical_support() {
        let run = |vertical_air_speed| {
            let mut sim = Simulator::new(VehicleDefinition::fixed_wing_trainer().to_vehicle().unwrap());
            sim.state.position_ned_m.z = -50.0;
            sim.state.velocity_ned_mps = DVec3::new(15.0, 0.0, vertical_air_speed);
            sim.step(0.004);
            (sim.forces.lift.z, sim.forces.drag.length(), sim.forces.stalled)
        };
        let normal = run(3.0);
        let stalled = run(8.0);
        assert!(!normal.2 && stalled.2);
        assert!(stalled.0 > normal.0, "stall must provide less upward NED force");
        assert!(stalled.1 > normal.1, "stall must add drag");
    }
    #[test]
    fn aerodynamic_drag_opposes_air_velocity() {
        let air = DVec3::new(12.0, 1.0, -0.5);
        let drag = aerodynamic_force(&trainer_surface("Left Wing"), air, SEA_LEVEL_DENSITY, 0.0).1;
        assert!(drag.dot(air) < 0.0);
    }
    fn trainer_moment(sticks: DVec3, speed: f64) -> DVec3 {
        let mut sim = Simulator::new(VehicleDefinition::fixed_wing_trainer().to_vehicle().unwrap());
        sim.state.position_ned_m.z = -20.0;
        sim.state.velocity_ned_mps.x = speed;
        sim.set_control(sticks, 0.0);
        for surface in &mut sim.vehicle.aero_surfaces {
            let input = match surface.control_axis {
                ControlAxis::None => 0.0,
                ControlAxis::Roll => sticks.x,
                ControlAxis::Pitch => sticks.y,
                ControlAxis::Yaw => sticks.z,
            };
            surface.actual_deflection_rad =
                surface.trim_deflection_rad + input * surface.control_sign * surface.max_deflection_rad;
        }
        sim.step(0.004);
        sim.forces.torque_body
    }
    #[test]
    fn control_surfaces_create_expected_moments() {
        assert!(trainer_moment(DVec3::X, 18.0).x > 0.0);
        assert!(trainer_moment(DVec3::Y, 18.0).y > 0.0);
        assert!(trainer_moment(DVec3::Z, 18.0).z > 0.0);
    }
    #[test]
    fn off_cg_surface_force_creates_the_expected_moment() {
        let surface = trainer_surface("Elevator");
        let (lift, drag, ..) = aerodynamic_force(
            &surface,
            DVec3::new(18.0, 0.0, 1.0),
            SEA_LEVEL_DENSITY,
            -surface.max_deflection_rad,
        );
        let moment = surface.position.cross(lift + drag);
        assert!(moment.y > 0.0);
        assert!(moment.x.abs() < 1e-9 && moment.z.abs() < 1e-9);
    }
    #[test]
    fn local_surface_velocity_includes_full_rigid_body_rotation() {
        let cg_air = DVec3::new(20.0, 1.0, -0.5);
        let omega = DVec3::new(0.2, -0.3, 0.4);
        let offset = DVec3::new(-0.6, 0.7, 0.1);
        assert_eq!(
            local_air_velocity_body(cg_air, omega, offset),
            cg_air + omega.cross(offset)
        );
    }
    fn trainer_rate_moment(rate: DVec3, speed: f64) -> DVec3 {
        let mut sim = Simulator::new(VehicleDefinition::fixed_wing_trainer().to_vehicle().unwrap());
        sim.state.position_ned_m.z = -20.0;
        sim.state.velocity_ned_mps.x = speed;
        sim.state.angular_rate_body_rps = rate;
        sim.step(0.004);
        sim.forces.torque_body
    }
    #[test]
    fn trainer_surfaces_damp_roll_pitch_and_yaw_rates() {
        for axis in [DVec3::X, DVec3::Y, DVec3::Z] {
            let rate = axis * 0.5;
            let moment = trainer_rate_moment(rate, 16.0);
            assert!(
                moment.dot(rate) < 0.0,
                "rate {rate:?} was not opposed by moment {moment:?}"
            );
        }
    }
    #[test]
    fn trainer_servo_is_rate_limited() {
        let mut sim = Simulator::new(VehicleDefinition::fixed_wing_trainer().to_vehicle().unwrap());
        sim.state.position_ned_m.z = -20.0;
        sim.state.velocity_ned_mps.x = 16.0;
        sim.set_control(DVec3::Y, 0.0);
        sim.step(0.004);
        let elevator = sim
            .vehicle
            .aero_surfaces
            .iter()
            .find(|surface| surface.name == "Elevator")
            .unwrap();
        assert!(elevator.actual_deflection_rad.abs() < elevator.commanded_deflection_rad.abs());
        assert!(
            (elevator.actual_deflection_rad - elevator.trim_deflection_rad).abs()
                <= elevator.servo_rate_rad_s * 0.004 + 1e-12
        );
    }
    #[test]
    fn trainer_trim_has_small_angular_acceleration() {
        let vehicle = VehicleDefinition::fixed_wing_trainer().to_vehicle().unwrap();
        let inertia = vehicle.inertia_kg_m2;
        let moment = trainer_moment(DVec3::ZERO, 16.0);
        let angular_acceleration = moment / inertia;
        assert!(
            angular_acceleration.length() < 0.8,
            "trim acceleration {angular_acceleration:?} from moment {moment:?}"
        );
    }
    #[test]
    fn moderate_trainer_inputs_do_not_create_pathological_rates() {
        for axis in [DVec3::X, DVec3::Y, DVec3::Z] {
            let mut sim = Simulator::new(VehicleDefinition::fixed_wing_trainer().to_vehicle().unwrap());
            sim.state.position_ned_m.z = -100.0;
            sim.state.velocity_ned_mps.x = 16.0;
            let mut peak_rate: f64 = 0.0;
            for _ in 0..125 {
                sim.set_control(axis * 0.25, 0.45);
                sim.step(0.004);
                peak_rate = peak_rate.max(sim.state.angular_rate_body_rps.length());
            }
            for _ in 0..375 {
                sim.set_control(DVec3::ZERO, 0.45);
                sim.step(0.004);
                peak_rate = peak_rate.max(sim.state.angular_rate_body_rps.length());
            }
            assert!(peak_rate < 2.5, "axis {axis:?} reached {peak_rate} rad/s");
            assert!(sim.state.angular_rate_body_rps.length() < 0.8);
        }
    }
    #[test]
    fn trainer_remains_finite_during_gentle_thirty_second_cruise() {
        let mut sim = Simulator::new(VehicleDefinition::fixed_wing_trainer().to_vehicle().unwrap());
        sim.state.position_ned_m.z = -1_000.0;
        sim.state.velocity_ned_mps.x = 16.0;
        let mut peak_rate: f64 = 0.0;
        for step in 0..7_500 {
            let phase = step % 2_500;
            let sticks = if phase < 250 {
                DVec3::new(0.12, 0.0, 0.0)
            } else if (800..1_050).contains(&phase) {
                DVec3::new(-0.12, 0.0, 0.0)
            } else if (1_500..1_700).contains(&phase) {
                DVec3::new(0.0, 0.07, 0.05)
            } else {
                DVec3::ZERO
            };
            sim.set_control(sticks, 0.42);
            sim.step(0.004);
            peak_rate = peak_rate.max(sim.state.angular_rate_body_rps.length());
        }
        assert!(sim.state.position_ned_m.is_finite());
        assert!(sim.state.attitude_body_to_ned.is_finite());
        assert!(peak_rate < 2.5, "peak cruise rate {peak_rate} rad/s");
    }
    #[test]
    fn surface_authority_grows_with_airspeed() {
        assert!(trainer_moment(DVec3::Y, 20.0).y.abs() > trainer_moment(DVec3::Y, 10.0).y.abs() * 3.5);
    }
    #[test]
    fn lower_density_reduces_aerodynamic_force() {
        let wing = trainer_surface("Left Wing");
        let air = DVec3::new(15.0, 0.0, 1.0);
        let sea = aerodynamic_force(&wing, air, SEA_LEVEL_DENSITY, 0.0).0.length();
        let high = aerodynamic_force(&wing, air, 0.8, 0.0).0.length();
        assert!(high < sea);
    }
    #[test]
    fn trainer_can_accelerate_and_take_off() {
        let mut sim = Simulator::new(VehicleDefinition::fixed_wing_trainer().to_vehicle().unwrap());
        sim.state.position_ned_m = DVec3::ZERO;
        let mut maximum_altitude: f64 = 0.0;
        let mut maximum_speed: f64 = 0.0;
        for step in 0..5000 {
            let pitch = if step > 1400 && sim.state.position_ned_m.z > -1.5 {
                0.30
            } else {
                0.0
            };
            sim.set_control(DVec3::new(0.0, pitch, 0.0), 1.0);
            sim.step(0.004);
            maximum_altitude = maximum_altitude.max(-sim.state.position_ned_m.z);
            maximum_speed = maximum_speed.max(sim.state.velocity_ned_mps.x);
        }
        assert!(maximum_speed > 8.0);
        assert!(maximum_altitude > 0.3, "maximum altitude {maximum_altitude}");
    }
    #[test]
    fn extreme_valid_aerodynamics_remain_finite() {
        let mut sim = Simulator::new(VehicleDefinition::fixed_wing_trainer().to_vehicle().unwrap());
        sim.state.position_ned_m.z = -100.0;
        sim.state.velocity_ned_mps = DVec3::new(80.0, 20.0, -30.0);
        sim.set_control(DVec3::ONE, 1.0);
        for _ in 0..100 {
            sim.step(0.004);
        }
        assert!(sim.state.position_ned_m.is_finite() && sim.state.attitude_body_to_ned.is_finite());
    }
    #[test]
    fn quadplane_lift_motors_can_support_hover_weight() {
        let definition = VehicleDefinition::quadplane();
        let metrics = definition.metrics().unwrap();
        assert!(metrics.hover_throttle < 0.75);
        let mut sim = Simulator::new(definition.to_vehicle().unwrap());
        sim.state.position_ned_m.z = -20.0;
        sim.set_control(DVec3::ZERO, metrics.hover_throttle);
        for _ in 0..750 {
            sim.step(0.004);
        }
        assert!(
            (-sim.state.position_ned_m.z - 20.0).abs() < 5.0,
            "hover altitude {}",
            -sim.state.position_ned_m.z
        );
        assert!(sim.forces.lift.length() < sim.forces.thrust.length() * 0.1);
    }
    #[test]
    fn quadplane_wing_carries_weight_at_cruise_speed() {
        let mut sim = Simulator::new(VehicleDefinition::quadplane().to_vehicle().unwrap());
        sim.state.position_ned_m.z = -100.0;
        sim.state.velocity_ned_mps = DVec3::new(17.0, 0.0, 0.8);
        sim.transition_command = 1.0;
        sim.transition_actual = 1.0;
        sim.set_control(DVec3::ZERO, 0.5);
        sim.step(0.004);
        assert!(sim.forces.lift.z < -sim.vehicle.mass() * GRAVITY * 0.5);
        assert!(sim.forces.thrust.x > 0.0);
    }
    #[test]
    fn tiltrotor_force_direction_is_continuous_and_actuator_is_rate_limited() {
        let run = |transition: f64| {
            let mut sim = Simulator::new(VehicleDefinition::tiltrotor().to_vehicle().unwrap());
            sim.state.position_ned_m.z = -20.0;
            sim.transition_command = transition;
            sim.transition_actual = transition;
            sim.set_control(DVec3::ZERO, 0.6);
            for motor in &mut sim.vehicle.motors {
                motor.actual_output = 0.6;
            }
            sim.step(0.004);
            sim.forces.thrust
        };
        let hover = run(0.0);
        let halfway = run(0.5);
        let cruise = run(1.0);
        assert!(hover.z < -1.0 && hover.x.abs() < 1e-6);
        assert!(halfway.x > 1.0 && halfway.z < -1.0);
        assert!(cruise.x > 1.0 && cruise.z.abs() < 0.1);
        let expected = hover.length() / 2.0_f64.sqrt();
        assert!((halfway.x - expected).abs() < 0.1 && (halfway.z + expected).abs() < 0.1);

        let mut actuator = Simulator::new(VehicleDefinition::tiltrotor().to_vehicle().unwrap());
        actuator.set_transition(1.0);
        actuator.step(0.004);
        assert!(actuator.transition_actual > 0.0 && actuator.transition_actual <= 0.0021);
    }
    #[test]
    fn vtol_forces_remain_continuous_around_mid_transition() {
        let force = |transition: f64| {
            let mut sim = Simulator::new(VehicleDefinition::tiltrotor().to_vehicle().unwrap());
            sim.state.position_ned_m.z = -20.0;
            sim.transition_command = transition;
            sim.transition_actual = transition;
            sim.set_control(DVec3::ZERO, 0.5);
            for motor in &mut sim.vehicle.motors {
                motor.actual_output = 0.5;
            }
            sim.step(0.004);
            sim.forces.thrust
        };
        let before = force(0.499);
        let after = force(0.501);
        assert!((before - after).length() < before.length() * 0.01);
    }
    #[test]
    fn quadplane_transition_builds_airspeed_and_wing_lift() {
        let definition = VehicleDefinition::quadplane();
        let hover = definition.metrics().unwrap().hover_throttle;
        let mut sim = Simulator::new(definition.to_vehicle().unwrap());
        sim.state.position_ned_m.z = -100.0;
        sim.set_transition(1.0);
        let mut maximum_airspeed: f64 = 0.0;
        let mut maximum_lift: f64 = 0.0;
        for _ in 0..3_500 {
            sim.set_control(DVec3::ZERO, hover.max(0.58));
            sim.step(0.004);
            maximum_airspeed = maximum_airspeed.max(sim.forces.air_velocity_ned.length());
            maximum_lift = maximum_lift.max(-sim.forces.lift.z);
        }
        assert!(maximum_airspeed > 9.0, "airspeed {maximum_airspeed}");
        assert!(maximum_lift > sim.vehicle.mass() * GRAVITY * 0.35);
        assert!(sim.state.position_ned_m.is_finite());
    }
    #[test]
    fn tiltrotor_forward_and_back_transition_are_physical() {
        let definition = VehicleDefinition::tiltrotor();
        let mut sim = Simulator::new(definition.to_vehicle().unwrap());
        sim.state.position_ned_m.z = -150.0;
        sim.set_transition(1.0);
        let mut maximum_airspeed: f64 = 0.0;
        let mut maximum_lift: f64 = 0.0;
        for _ in 0..3_000 {
            sim.set_control(DVec3::ZERO, 0.55);
            sim.step(0.004);
            maximum_airspeed = maximum_airspeed.max(sim.forces.air_velocity_ned.length());
            maximum_lift = maximum_lift.max(-sim.forces.lift.z);
        }
        assert!(sim.transition_actual > 0.95);
        assert!(maximum_airspeed > 10.0);
        assert!(maximum_lift > sim.vehicle.mass() * GRAVITY * 0.3);
        sim.set_transition(0.0);
        for _ in 0..1_000 {
            sim.set_control(DVec3::ZERO, 0.5);
            sim.step(0.004);
        }
        assert!(sim.transition_actual < 0.05);
        assert!(sim.vehicle.motors[0].direction.z < -0.99);
        assert!(sim.state.position_ned_m.is_finite());
    }

    fn equal_hover_pitch_moment(definition: VehicleDefinition) -> f64 {
        let vehicle = definition.to_vehicle().unwrap();
        let cg = vehicle.center_of_mass();
        vehicle
            .motors
            .iter()
            .filter(|motor| motor.role != PropulsionRole::Forward)
            .map(|motor| {
                (motor.position - cg)
                    .cross(motor.hover_direction * motor.max_thrust_n)
                    .y
            })
            .sum()
    }

    #[test]
    fn neutral_hover_presets_have_no_pitch_bias() {
        for definition in [
            VehicleDefinition::beginner(),
            VehicleDefinition::freestyle(),
            VehicleDefinition::quadplane(),
            VehicleDefinition::tiltrotor(),
        ] {
            let moment = equal_hover_pitch_moment(definition.clone());
            assert!(moment.abs() < 1e-7, "{} pitch moment {moment}", definition.name);
        }
    }

    #[test]
    fn fixed_wing_trim_does_not_leak_into_vtol_hover() {
        for definition in [VehicleDefinition::quadplane(), VehicleDefinition::tiltrotor()] {
            let mut sim = Simulator::new(definition.to_vehicle().unwrap());
            sim.state.position_ned_m.z = -20.0;
            sim.step(0.004);
            let elevator = sim
                .vehicle
                .aero_surfaces
                .iter()
                .find(|surface| surface.name == "Elevator")
                .unwrap();
            assert_eq!(elevator.commanded_deflection_rad, 0.0);
            assert_eq!(elevator.actual_deflection_rad, 0.0);
        }
    }

    #[test]
    fn projected_quad_drag_is_directional_and_physically_sized() {
        let vehicle = VehicleDefinition::beginner().to_vehicle().unwrap();
        let forward = body_drag_force_ned(
            DVec3::X * 20.0,
            DQuat::IDENTITY,
            vehicle.body_dimensions_m,
            SEA_LEVEL_DENSITY,
            0.9,
        );
        let lateral = body_drag_force_ned(
            DVec3::Y * 20.0,
            DQuat::IDENTITY,
            vehicle.body_dimensions_m,
            SEA_LEVEL_DENSITY,
            0.9,
        );
        assert!(forward.x < 0.0 && lateral.y < 0.0);
        assert!((forward.x + 4.851).abs() < 0.02, "forward drag {forward:?}");
        assert!(lateral.length() > forward.length());
        assert!(forward.length() < vehicle.motors.iter().map(|m| m.max_thrust_n).sum());
    }

    #[test]
    fn freestyle_has_more_propulsive_performance_than_beginner() {
        let beginner = VehicleDefinition::beginner();
        let freestyle = VehicleDefinition::freestyle();
        let beginner_metrics = beginner.metrics().unwrap();
        let freestyle_metrics = freestyle.metrics().unwrap();
        assert!(
            freestyle_metrics.thrust_to_weight > beginner_metrics.thrust_to_weight * 1.7,
            "beginner {} freestyle {}",
            beginner_metrics.thrust_to_weight,
            freestyle_metrics.thrust_to_weight
        );
        assert!(
            freestyle.motors[0].spin_up_time_s < beginner.motors[0].spin_up_time_s,
            "freestyle motor must spool faster"
        );
    }

    fn level_translation_benchmark(definition: &VehicleDefinition, tilt_deg: f64) -> (f64, f64) {
        let vehicle = definition.to_vehicle().unwrap();
        let horizontal_thrust = vehicle.mass() * GRAVITY * tilt_deg.to_radians().tan();
        let drag_k =
            0.5 * SEA_LEVEL_DENSITY * 0.9 * vehicle.body_dimensions_m.y * vehicle.body_dimensions_m.z;
        let terminal = (horizontal_thrust / drag_k).sqrt();
        let mut speed = 0.0;
        let mut time_to_ten = f64::INFINITY;
        for step in 0..20_000 {
            speed += ((horizontal_thrust - drag_k * speed * speed) / vehicle.mass()) * 0.004;
            if speed >= 10.0 && !time_to_ten.is_finite() {
                time_to_ten = f64::from(step) * 0.004;
            }
        }
        (terminal, time_to_ten)
    }

    #[test]
    fn multicopter_translation_benchmarks_are_useful_and_distinct() {
        let beginner = level_translation_benchmark(&VehicleDefinition::beginner(), 30.0);
        let freestyle = level_translation_benchmark(&VehicleDefinition::freestyle(), 45.0);
        assert!(beginner.0 > 20.0 && beginner.1 < 3.0, "beginner {beginner:?}");
        assert!(
            freestyle.0 > 35.0 && freestyle.1 < beginner.1,
            "freestyle {freestyle:?}"
        );
        assert!(freestyle.0 > beginner.0 * 1.5);
    }

    #[test]
    fn trainer_reaches_a_useful_bank_without_runaway_roll() {
        let mut sim = Simulator::new(VehicleDefinition::fixed_wing_trainer().to_vehicle().unwrap());
        sim.state.position_ned_m.z = -100.0;
        sim.state.velocity_ned_mps = DVec3::X * 16.0;
        sim.vehicle.motors[0].actual_output = 0.45;
        let mut peak_rate: f64 = 0.0;
        for _ in 0..375 {
            sim.set_control(DVec3::new(0.5, 0.0, 0.0), 0.45);
            sim.step(0.004);
            peak_rate = peak_rate.max(sim.state.angular_rate_body_rps.x.abs());
        }
        let (_, _, roll) = sim.state.attitude_body_to_ned.to_euler(glam::EulerRot::ZYX);
        assert!(roll.abs().to_degrees() > 20.0, "bank {} deg", roll.to_degrees());
        assert!(
            peak_rate.to_degrees() < 120.0,
            "roll rate {} deg/s",
            peak_rate.to_degrees()
        );
        for _ in 0..375 {
            sim.set_control(DVec3::ZERO, 0.45);
            sim.step(0.004);
        }
        assert!(sim.state.angular_rate_body_rps.x.abs() < peak_rate);
    }

    #[test]
    fn headwind_and_tailwind_change_authoritative_airspeed() {
        use atmosphere::{WeatherPreset, WindField};
        let run = |from_deg: f64| {
            let mut sim = Simulator::new(VehicleDefinition::fixed_wing_trainer().to_vehicle().unwrap());
            sim.state.position_ned_m.z = -100.0;
            sim.state.velocity_ned_mps = DVec3::X * 15.0;
            let mut wind = WindField::preset(WeatherPreset::Custom);
            wind.speed_mps = 5.0;
            wind.direction_from_deg = from_deg;
            sim.environment.wind = wind;
            sim.step(0.004);
            sim.forces.air_velocity_ned.length()
        };
        assert!((run(0.0) - 20.0).abs() < 0.1);
        assert!((run(180.0) - 10.0).abs() < 0.1);
    }

    #[test]
    fn multicopter_wind_response_comes_from_relative_air_drag() {
        use atmosphere::{WeatherPreset, WindField};
        let mut sim = Simulator::new(beginner_quad());
        sim.state.position_ned_m.z = -20.0;
        let mut wind = WindField::preset(WeatherPreset::Custom);
        wind.speed_mps = 10.0;
        wind.direction_from_deg = 270.0;
        sim.environment.wind = wind;
        sim.step(0.004);
        assert!(sim.forces.air_velocity_ned.y < -9.9);
        assert!(sim.forces.drag.y > 0.0, "drag carries the quad downwind");
    }

    #[test]
    fn thermal_airmass_increases_trainer_vertical_support() {
        use atmosphere::{WeatherPreset, WindField};
        let run = |north: f64, east: f64| {
            let mut sim = Simulator::new(VehicleDefinition::fixed_wing_trainer().to_vehicle().unwrap());
            let ground = sim.environment.terrain.elevation_m(north, east);
            sim.state.position_ned_m = DVec3::new(north, east, -ground - 450.0);
            sim.state.velocity_ned_mps = DVec3::X * 16.0;
            sim.environment.wind = WindField::preset(WeatherPreset::Soaring);
            sim.step(0.004);
            -sim.forces.lift.z
        };
        let thermal = run(1_050.0, 850.0);
        let outside = run(0.0, 0.0);
        assert!(thermal > outside + 1.0, "thermal {thermal} outside {outside}");
    }

    #[test]
    fn ridge_airmass_increases_trainer_vertical_support() {
        use atmosphere::{WeatherPreset, WindField};
        let north = 1_200.0;
        let east = 1_000.0;
        let terrain = terrain::TerrainDefinition::default();
        let normal = terrain.normal_up_ned(north, east);
        let gradient = glam::DVec2::new(normal.x / normal.z, normal.y / normal.z).normalize();
        let to_deg = gradient.y.atan2(gradient.x).to_degrees();
        let run = |terrain_flow: bool| {
            let mut sim = Simulator::new(VehicleDefinition::fixed_wing_trainer().to_vehicle().unwrap());
            let ground = sim.environment.terrain.elevation_m(north, east);
            sim.state.position_ned_m = DVec3::new(north, east, -ground - 35.0);
            sim.state.velocity_ned_mps = DVec3::X * 16.0;
            let mut wind = WindField::preset(WeatherPreset::Alpine);
            wind.direction_from_deg = (to_deg + 180.0).rem_euclid(360.0);
            wind.terrain_flow = terrain_flow;
            sim.environment.wind = wind;
            sim.step(0.004);
            -sim.forces.lift.z
        };
        let ridge = run(true);
        let flat_air = run(false);
        assert!(ridge > flat_air + 0.2, "ridge {ridge} flat {flat_air}");
    }

    fn exercise_three_vtol_transitions(definition: VehicleDefinition) -> (f64, f64) {
        let hover = definition.metrics().unwrap().hover_throttle;
        let mut sim = Simulator::new(definition.to_vehicle().unwrap());
        sim.state.position_ned_m.z = -180.0;
        let mut peak_speed: f64 = 0.0;
        let mut peak_wing_fraction: f64 = 0.0;
        for _cycle in 0..3 {
            sim.set_transition(1.0);
            for _ in 0..3_500 {
                sim.set_control(DVec3::ZERO, hover.max(0.56));
                sim.step(0.004);
                peak_speed = peak_speed.max(sim.forces.air_velocity_ned.length());
                peak_wing_fraction =
                    peak_wing_fraction.max((-sim.forces.lift.z / (sim.vehicle.mass() * GRAVITY)).max(0.0));
            }
            sim.set_transition(0.0);
            for _ in 0..1_500 {
                sim.set_control(DVec3::ZERO, hover);
                sim.step(0.004);
            }
            assert!(sim.transition_actual < 0.05);
            assert!(sim.state.position_ned_m.is_finite());
        }
        (peak_speed, peak_wing_fraction)
    }

    #[test]
    fn quadplane_repeated_transition_cycle_remains_physical() {
        let (speed, wing_fraction) = exercise_three_vtol_transitions(VehicleDefinition::quadplane());
        assert!(speed > 9.0 && wing_fraction > 0.3);
    }

    #[test]
    fn tiltrotor_repeated_transition_cycle_remains_physical() {
        let (speed, wing_fraction) = exercise_three_vtol_transitions(VehicleDefinition::tiltrotor());
        assert!(speed > 10.0 && wing_fraction > 0.3);
    }
}
