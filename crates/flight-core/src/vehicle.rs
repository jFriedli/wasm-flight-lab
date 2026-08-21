use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::{ComponentMass, GRAVITY, Motor, Vehicle};

pub const VEHICLE_SCHEMA_VERSION: u32 = 1;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
#[serde(rename_all = "camelCase")]
pub struct FrameDefinition {
    pub arm_length_m: f64,
    pub body_mass_kg: f64,
    pub body_dimensions_m: DVec3,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
#[serde(rename_all = "camelCase")]
pub struct PropellerDefinition {
    pub diameter_m: f64,
    pub pitch_m: f64,
    pub blade_count: u8,
    pub efficiency: f64,
    pub mass_kg: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
#[serde(rename_all = "camelCase")]
pub struct MotorDefinition {
    pub position_m: DVec3,
    pub direction_body: DVec3,
    pub base_max_thrust_n: f64,
    pub max_power_w: f64,
    pub mass_kg: f64,
    pub spin: f64,
    pub reaction_torque_nm: f64,
    pub spin_up_time_s: f64,
    pub spin_down_time_s: f64,
    pub propeller: PropellerDefinition,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
#[serde(rename_all = "camelCase")]
pub struct BatteryDefinition {
    pub cells: u8,
    pub capacity_mah: f64,
    pub mass_kg: f64,
    pub position_m: DVec3,
    pub internal_resistance_ohm: f64,
    pub max_discharge_c: f64,
}

impl BatteryDefinition {
    pub fn nominal_voltage(&self) -> f64 {
        f64::from(self.cells) * 3.7
    }
    pub fn energy_wh(&self) -> f64 {
        self.nominal_voltage() * self.capacity_mah / 1000.0
    }
    pub fn max_current_a(&self) -> f64 {
        self.capacity_mah / 1000.0 * self.max_discharge_c
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
#[serde(rename_all = "camelCase")]
pub struct PayloadDefinition {
    pub name: String,
    pub mass_kg: f64,
    pub position_m: DVec3,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
#[serde(rename_all = "camelCase")]
pub struct VehicleDefinition {
    pub schema_version: u32,
    pub name: String,
    pub preset: String,
    pub frame: FrameDefinition,
    pub motors: Vec<MotorDefinition>,
    pub battery: BatteryDefinition,
    pub payloads: Vec<PayloadDefinition>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
#[serde(rename_all = "camelCase")]
pub struct EngineeringMetrics {
    pub total_mass_kg: f64,
    pub center_of_mass_m: DVec3,
    pub inertia_kg_m2: DVec3,
    pub max_thrust_n: f64,
    pub thrust_to_weight: f64,
    pub hover_throttle: f64,
    pub max_power_w: f64,
    pub battery_energy_wh: f64,
    pub hover_current_a: f64,
    pub hover_flight_time_min: f64,
    pub hover_motor_outputs: [f64; 4],
    pub warnings: Vec<String>,
}

impl VehicleDefinition {
    pub fn beginner() -> Self {
        Self::quad_preset(false)
    }
    pub fn freestyle() -> Self {
        Self::quad_preset(true)
    }

    fn quad_preset(freestyle: bool) -> Self {
        let arm = if freestyle { 0.14 } else { 0.18 };
        let motor = |x, y, spin| MotorDefinition {
            position_m: DVec3::new(x, y, 0.0),
            direction_body: -DVec3::Z,
            base_max_thrust_n: if freestyle { 12.0 } else { 8.0 },
            max_power_w: if freestyle { 420.0 } else { 220.0 },
            mass_kg: if freestyle { 0.035 } else { 0.045 },
            spin,
            reaction_torque_nm: if freestyle { 0.16 } else { 0.12 },
            spin_up_time_s: if freestyle { 0.035 } else { 0.075 },
            spin_down_time_s: if freestyle { 0.055 } else { 0.11 },
            propeller: PropellerDefinition {
                diameter_m: if freestyle { 0.127 } else { 0.254 },
                pitch_m: 0.114,
                blade_count: 2,
                efficiency: 0.72,
                mass_kg: 0.012,
            },
        };
        Self {
            schema_version: VEHICLE_SCHEMA_VERSION,
            name: if freestyle {
                "Freestyle Quad"
            } else {
                "Beginner Quad"
            }
            .into(),
            preset: if freestyle { "freestyle" } else { "beginner" }.into(),
            frame: FrameDefinition {
                arm_length_m: arm,
                body_mass_kg: if freestyle { 0.28 } else { 0.40 },
                body_dimensions_m: DVec3::new(0.30, 0.22, 0.10),
            },
            motors: vec![
                motor(arm, arm, 1.0),
                motor(arm, -arm, -1.0),
                motor(-arm, -arm, 1.0),
                motor(-arm, arm, -1.0),
            ],
            battery: BatteryDefinition {
                cells: if freestyle { 6 } else { 4 },
                capacity_mah: if freestyle { 1300.0 } else { 3000.0 },
                mass_kg: if freestyle { 0.22 } else { 0.30 },
                position_m: DVec3::new(-0.03, 0.0, 0.02),
                internal_resistance_ohm: if freestyle { 0.035 } else { 0.045 },
                max_discharge_c: if freestyle { 75.0 } else { 30.0 },
            },
            payloads: vec![PayloadDefinition {
                name: "Camera".into(),
                mass_kg: if freestyle { 0.04 } else { 0.08 },
                position_m: DVec3::new(0.08, 0.0, -0.02),
            }],
        }
    }

    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != VEHICLE_SCHEMA_VERSION {
            return Err("unsupported schemaVersion".into());
        }
        if self.name.trim().is_empty() || self.name.len() > 80 {
            return Err("name must contain 1-80 characters".into());
        }
        if self.motors.len() != 4 || self.payloads.len() > 32 {
            return Err("quad requires four motors and at most 32 payloads".into());
        }
        let finite_range = |v: f64, low: f64, high: f64| v.is_finite() && (low..=high).contains(&v);
        if !finite_range(self.frame.arm_length_m, 0.03, 2.0)
            || !finite_range(self.frame.body_mass_kg, 0.02, 100.0)
            || !self.frame.body_dimensions_m.is_finite()
        {
            return Err("invalid frame".into());
        }
        if !finite_range(self.battery.capacity_mah, 100.0, 100_000.0)
            || !(1..=16).contains(&self.battery.cells)
            || !finite_range(self.battery.mass_kg, 0.02, 100.0)
            || !self.battery.position_m.is_finite()
            || self.battery.position_m.abs().max_element() > 10.0
            || !finite_range(self.battery.internal_resistance_ohm, 0.0, 2.0)
            || !finite_range(self.battery.max_discharge_c, 1.0, 200.0)
        {
            return Err("invalid battery".into());
        }
        for motor in &self.motors {
            if !motor.position_m.is_finite()
                || motor.position_m.abs().max_element() > 10.0
                || !motor.direction_body.is_finite()
                || motor.direction_body.length() < 0.5
                || !finite_range(motor.base_max_thrust_n, 0.0, 10_000.0)
                || !finite_range(motor.mass_kg, 0.001, 10.0)
                || !finite_range(motor.max_power_w, 1.0, 100_000.0)
                || !finite_range(motor.spin_up_time_s, 0.005, 5.0)
                || !finite_range(motor.spin_down_time_s, 0.005, 5.0)
                || !finite_range(motor.reaction_torque_nm, 0.0, 100.0)
                || ![-1.0, 1.0].contains(&motor.spin)
                || !finite_range(motor.propeller.diameter_m, 0.02, 2.0)
                || !finite_range(motor.propeller.pitch_m, 0.01, 1.0)
                || !(1..=8).contains(&motor.propeller.blade_count)
                || !finite_range(motor.propeller.efficiency, 0.1, 1.0)
                || !finite_range(motor.propeller.mass_kg, 0.001, 5.0)
            {
                return Err("invalid motor".into());
            }
        }
        for payload in &self.payloads {
            if payload.name.trim().is_empty()
                || payload.name.len() > 60
                || !finite_range(payload.mass_kg, 0.001, 1000.0)
                || !payload.position_m.is_finite()
                || payload.position_m.abs().max_element() > 10.0
            {
                return Err("invalid payload".into());
            }
        }
        Ok(())
    }

    pub fn to_vehicle(&self) -> Result<Vehicle, String> {
        self.validate()?;
        let mut masses = vec![
            ComponentMass {
                mass_kg: self.frame.body_mass_kg,
                position: DVec3::ZERO,
            },
            ComponentMass {
                mass_kg: self.battery.mass_kg,
                position: self.battery.position_m,
            },
        ];
        for motor in &self.motors {
            masses.push(ComponentMass {
                mass_kg: motor.mass_kg + motor.propeller.mass_kg,
                position: motor.position_m,
            });
        }
        masses.extend(self.payloads.iter().map(|p| ComponentMass {
            mass_kg: p.mass_kg,
            position: p.position_m,
        }));
        let center = weighted_center(&masses);
        let dims = self.frame.body_dimensions_m;
        let body_i = self.frame.body_mass_kg / 12.0
            * DVec3::new(
                dims.y * dims.y + dims.z * dims.z,
                dims.x * dims.x + dims.z * dims.z,
                dims.x * dims.x + dims.y * dims.y,
            );
        let inertia = masses.iter().fold(body_i, |sum, m| {
            let r = m.position - center;
            sum + DVec3::new(
                m.mass_kg * (r.y * r.y + r.z * r.z),
                m.mass_kg * (r.x * r.x + r.z * r.z),
                m.mass_kg * (r.x * r.x + r.y * r.y),
            )
        });
        let motors = self
            .motors
            .iter()
            .map(|m| Motor {
                position: m.position_m,
                direction: m.direction_body.normalize(),
                max_thrust_n: m.base_max_thrust_n * propeller_factor(&m.propeller),
                reaction_torque_nm: m.reaction_torque_nm,
                spin: m.spin,
                command: 0.0,
                actual_output: 0.0,
                spin_up_time_s: m.spin_up_time_s,
                spin_down_time_s: m.spin_down_time_s,
                max_power_w: m.max_power_w,
                effectiveness: 1.0,
            })
            .collect();
        Ok(Vehicle {
            name: self.name.clone(),
            masses,
            motors,
            wings: vec![],
            inertia_kg_m2: inertia,
            battery: self.battery.clone(),
        })
    }

    pub fn metrics(&self) -> Result<EngineeringMetrics, String> {
        let vehicle = self.to_vehicle()?;
        let mass = vehicle.mass();
        let max_thrust = vehicle.motors.iter().map(|m| m.max_thrust_n).sum::<f64>();
        let hover = mass * GRAVITY / max_thrust.max(1e-9);
        let max_power = self.motors.iter().map(|m| m.max_power_w).sum::<f64>();
        let hover_power = max_power * hover.powf(1.5);
        let voltage = self.battery.nominal_voltage();
        let current = hover_power / voltage;
        let minutes = self.battery.energy_wh() / hover_power * 60.0 * 0.8;
        let trim = hover_trim(&vehicle);
        let mut warnings = vec![];
        if max_thrust <= mass * GRAVITY {
            warnings.push("CRITICAL: total thrust cannot support weight".into())
        } else if max_thrust / (mass * GRAVITY) < 1.2 {
            warnings.push("WARNING: thrust-to-weight is below 1.2".into())
        }
        if hover > 0.8 {
            warnings.push("WARNING: estimated hover throttle is very high".into())
        }
        if current > self.battery.max_current_a() {
            warnings.push("WARNING: estimated battery discharge limit exceeded".into())
        }
        if trim.iter().any(|v| *v < 0.0 || *v > 1.0) {
            warnings.push("CRITICAL: level hover trim saturates a motor".into())
        }
        if vehicle.center_of_mass().truncate().length() > self.frame.arm_length_m * 0.35 {
            warnings.push("WARNING: centre of mass is strongly offset".into())
        }
        Ok(EngineeringMetrics {
            total_mass_kg: mass,
            center_of_mass_m: vehicle.center_of_mass(),
            inertia_kg_m2: vehicle.inertia_kg_m2,
            max_thrust_n: max_thrust,
            thrust_to_weight: max_thrust / (mass * GRAVITY),
            hover_throttle: hover,
            max_power_w: max_power,
            battery_energy_wh: self.battery.energy_wh(),
            hover_current_a: current,
            hover_flight_time_min: minutes,
            hover_motor_outputs: trim,
            warnings,
        })
    }
}

pub fn propeller_factor(p: &PropellerDefinition) -> f64 {
    ((p.diameter_m / 0.254).powi(2)
        * (p.pitch_m / 0.114).powf(0.3)
        * (f64::from(p.blade_count) / 2.0).powf(0.2)
        * (p.efficiency / 0.72))
        .clamp(0.1, 4.0)
}
fn weighted_center(masses: &[ComponentMass]) -> DVec3 {
    let total = masses.iter().map(|m| m.mass_kg).sum::<f64>();
    masses.iter().map(|m| m.position * m.mass_kg).sum::<DVec3>() / total
}

fn hover_trim(vehicle: &Vehicle) -> [f64; 4] {
    let com = vehicle.center_of_mass();
    let mut a = [[0.0; 5]; 4];
    for (column, m) in vehicle.motors.iter().enumerate() {
        let force = m.direction.normalize() * m.max_thrust_n;
        let torque =
            (m.position - com).cross(force) + m.direction.normalize() * m.reaction_torque_nm * m.spin;
        a[0][column] = -force.z;
        a[1][column] = torque.x;
        a[2][column] = torque.y;
        a[3][column] = torque.z;
    }
    a[0][4] = vehicle.mass() * GRAVITY;
    for pivot in 0..4 {
        let row = (pivot..4)
            .max_by(|a_row, b_row| a[*a_row][pivot].abs().total_cmp(&a[*b_row][pivot].abs()))
            .unwrap();
        a.swap(pivot, row);
        if a[pivot][pivot].abs() < 1e-9 {
            return [f64::NAN; 4];
        }
        let divisor = a[pivot][pivot];
        for value in a[pivot].iter_mut().skip(pivot) {
            *value /= divisor
        }
        let pivot_row = a[pivot];
        for (i, row) in a.iter_mut().enumerate() {
            if i != pivot {
                let factor = row[pivot];
                for (j, value) in row.iter_mut().enumerate().skip(pivot) {
                    *value -= factor * pivot_row[j]
                }
            }
        }
    }
    [a[0][4], a[1][4], a[2][4], a[3][4]]
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn payload_moves_cg_and_inertia() {
        let mut d = VehicleDefinition::beginner();
        let base = d.metrics().unwrap();
        d.payloads[0].position_m = DVec3::new(0.5, 0.0, 0.0);
        let moved = d.metrics().unwrap();
        assert!(moved.center_of_mass_m.x > base.center_of_mass_m.x);
        assert!(moved.inertia_kg_m2.y > base.inertia_kg_m2.y);
    }
    #[test]
    fn longer_arms_increase_torque_and_inertia() {
        let mut d = VehicleDefinition::beginner();
        let near = d.to_vehicle().unwrap();
        for motor in &mut d.motors {
            motor.position_m *= 2.0
        }
        let far = d.to_vehicle().unwrap();
        let near_t = (near.motors[0].position - near.center_of_mass())
            .cross(-DVec3::Z * near.motors[0].max_thrust_n)
            .x
            .abs();
        let far_t = (far.motors[0].position - far.center_of_mass())
            .cross(-DVec3::Z * far.motors[0].max_thrust_n)
            .x
            .abs();
        assert!(far_t > near_t);
        assert!(far.inertia_kg_m2.x > near.inertia_kg_m2.x);
    }
    #[test]
    fn weaker_propulsion_raises_hover_and_can_fail() {
        let mut d = VehicleDefinition::beginner();
        let base = d.metrics().unwrap().hover_throttle;
        for m in &mut d.motors {
            m.base_max_thrust_n = 1.0
        }
        let weak = d.metrics().unwrap();
        assert!(weak.hover_throttle > base);
        assert!(weak.warnings.iter().any(|w| w.contains("CRITICAL")));
    }
    #[test]
    fn added_battery_mass_reduces_acceleration_for_the_same_thrust() {
        let mut heavy = VehicleDefinition::beginner();
        let light = heavy.to_vehicle().unwrap();
        heavy.battery.mass_kg *= 2.0;
        let heavy = heavy.to_vehicle().unwrap();
        let thrust = 10.0;
        assert!(thrust / heavy.mass() < thrust / light.mass());
    }
    #[test]
    fn trim_reacts_to_offset_payload() {
        let mut d = VehicleDefinition::beginner();
        d.payloads[0].position_m.y = 0.3;
        let trim = d.metrics().unwrap().hover_motor_outputs;
        assert!((trim[0] - trim[1]).abs() > 0.01);
    }
    #[test]
    fn validation_rejects_non_finite_and_component_bombs() {
        let mut d = VehicleDefinition::beginner();
        d.motors[0].position_m.x = f64::NAN;
        assert!(d.validate().is_err());
        d = VehicleDefinition::beginner();
        d.payloads = vec![d.payloads[0].clone(); 33];
        assert!(d.validate().is_err());
    }
}
