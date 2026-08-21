use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::{
    AeroSurface, ComponentMass, ControlAxis, GRAVITY, Motor, SEA_LEVEL_DENSITY, SurfacePlane, Vehicle,
    VehicleClass,
};

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

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum VehicleClassDefinition {
    #[default]
    Multicopter,
    FixedWing,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum SurfacePlaneDefinition {
    Horizontal,
    Vertical,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum ControlAxisDefinition {
    None,
    Roll,
    Pitch,
    Yaw,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields, rename_all = "camelCase")]
pub struct AeroSurfaceDefinition {
    pub name: String,
    pub position_m: DVec3,
    pub plane: SurfacePlaneDefinition,
    pub area_m2: f64,
    pub span_m: f64,
    pub chord_m: f64,
    pub incidence_deg: f64,
    pub lift_slope_per_rad: f64,
    pub stall_angle_deg: f64,
    pub cd0: f64,
    pub induced_drag_k: f64,
    pub control_axis: ControlAxisDefinition,
    pub control_sign: f64,
    pub max_deflection_deg: f64,
    pub control_effectiveness: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
#[serde(rename_all = "camelCase")]
pub struct VehicleDefinition {
    pub schema_version: u32,
    pub name: String,
    pub preset: String,
    #[serde(default)]
    pub vehicle_class: VehicleClassDefinition,
    pub frame: FrameDefinition,
    pub motors: Vec<MotorDefinition>,
    pub battery: BatteryDefinition,
    pub payloads: Vec<PayloadDefinition>,
    #[serde(default)]
    pub aero_surfaces: Vec<AeroSurfaceDefinition>,
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
    pub wing_area_m2: f64,
    pub aspect_ratio: f64,
    pub wing_loading_kg_m2: f64,
    pub estimated_stall_speed_mps: f64,
    pub power_to_weight_w_kg: f64,
    pub cg_mac_fraction: f64,
}

impl VehicleDefinition {
    pub fn beginner() -> Self {
        Self::quad_preset(false)
    }
    pub fn freestyle() -> Self {
        Self::quad_preset(true)
    }
    pub fn fixed_wing_trainer() -> Self {
        let propeller = PropellerDefinition {
            diameter_m: 0.28,
            pitch_m: 0.18,
            blade_count: 2,
            efficiency: 0.74,
            mass_kg: 0.025,
        };
        let surface = |name: &str, position, plane, area, span, chord, axis, sign| AeroSurfaceDefinition {
            name: name.into(),
            position_m: position,
            plane,
            area_m2: area,
            span_m: span,
            chord_m: chord,
            incidence_deg: if name.contains("Wing") { 2.0 } else { 0.0 },
            lift_slope_per_rad: 4.8,
            stall_angle_deg: 15.0,
            cd0: 0.025,
            induced_drag_k: 0.065,
            control_axis: axis,
            control_sign: sign,
            max_deflection_deg: 22.0,
            control_effectiveness: 0.55,
        };
        Self {
            schema_version: VEHICLE_SCHEMA_VERSION,
            name: "Fixed Wing Trainer".into(),
            preset: "trainer".into(),
            vehicle_class: VehicleClassDefinition::FixedWing,
            frame: FrameDefinition {
                arm_length_m: 0.3,
                body_mass_kg: 0.72,
                body_dimensions_m: DVec3::new(1.05, 0.16, 0.18),
            },
            motors: vec![MotorDefinition {
                position_m: DVec3::new(0.52, 0.0, 0.0),
                direction_body: DVec3::X,
                base_max_thrust_n: 15.0,
                max_power_w: 520.0,
                mass_kg: 0.09,
                spin: 1.0,
                reaction_torque_nm: 0.025,
                spin_up_time_s: 0.09,
                spin_down_time_s: 0.14,
                propeller,
            }],
            battery: BatteryDefinition {
                cells: 4,
                capacity_mah: 3300.0,
                mass_kg: 0.34,
                position_m: DVec3::new(0.04, 0.0, 0.02),
                internal_resistance_ohm: 0.04,
                max_discharge_c: 35.0,
            },
            payloads: vec![PayloadDefinition {
                name: "FPV Camera".into(),
                mass_kg: 0.06,
                position_m: DVec3::new(0.34, 0.0, -0.05),
            }],
            aero_surfaces: vec![
                surface(
                    "Left Wing",
                    DVec3::new(0.02, -0.55, 0.0),
                    SurfacePlaneDefinition::Horizontal,
                    0.24,
                    0.85,
                    0.28,
                    ControlAxisDefinition::Roll,
                    1.0,
                ),
                surface(
                    "Right Wing",
                    DVec3::new(0.02, 0.55, 0.0),
                    SurfacePlaneDefinition::Horizontal,
                    0.24,
                    0.85,
                    0.28,
                    ControlAxisDefinition::Roll,
                    -1.0,
                ),
                surface(
                    "Elevator",
                    DVec3::new(-0.43, 0.0, 0.0),
                    SurfacePlaneDefinition::Horizontal,
                    0.10,
                    0.55,
                    0.18,
                    ControlAxisDefinition::Pitch,
                    -1.0,
                ),
                surface(
                    "Rudder",
                    DVec3::new(-0.43, 0.0, -0.08),
                    SurfacePlaneDefinition::Vertical,
                    0.065,
                    0.25,
                    0.22,
                    ControlAxisDefinition::Yaw,
                    1.0,
                ),
            ],
        }
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
            vehicle_class: VehicleClassDefinition::Multicopter,
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
            aero_surfaces: vec![],
        }
    }

    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != VEHICLE_SCHEMA_VERSION {
            return Err("unsupported schemaVersion".into());
        }
        if self.name.trim().is_empty() || self.name.len() > 80 {
            return Err("name must contain 1-80 characters".into());
        }
        let motor_count_valid = match self.vehicle_class {
            VehicleClassDefinition::Multicopter => self.motors.len() == 4,
            VehicleClassDefinition::FixedWing => self.motors.len() == 1,
        };
        if !motor_count_valid || self.payloads.len() > 32 || self.aero_surfaces.len() > 16 {
            return Err("invalid component count for vehicle class".into());
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
        for surface in &self.aero_surfaces {
            if surface.name.is_empty()
                || surface.name.len() > 60
                || !surface.position_m.is_finite()
                || !finite_range(surface.area_m2, 0.005, 20.0)
                || !finite_range(surface.span_m, 0.05, 20.0)
                || !finite_range(surface.chord_m, 0.02, 5.0)
                || !finite_range(surface.incidence_deg, -30.0, 30.0)
                || !finite_range(surface.lift_slope_per_rad, 0.1, 12.0)
                || !finite_range(surface.stall_angle_deg, 5.0, 45.0)
                || !finite_range(surface.cd0, 0.001, 2.0)
                || !finite_range(surface.induced_drag_k, 0.0, 2.0)
                || !finite_range(surface.control_sign, -1.0, 1.0)
                || !finite_range(surface.max_deflection_deg, 0.0, 60.0)
                || !finite_range(surface.control_effectiveness, 0.0, 2.0)
            {
                return Err("invalid aerodynamic surface".into());
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
            class: match self.vehicle_class {
                VehicleClassDefinition::Multicopter => VehicleClass::Multicopter,
                VehicleClassDefinition::FixedWing => VehicleClass::FixedWing,
            },
            aero_surfaces: self
                .aero_surfaces
                .iter()
                .map(|s| AeroSurface {
                    name: s.name.clone(),
                    position: s.position_m,
                    plane: match s.plane {
                        SurfacePlaneDefinition::Horizontal => SurfacePlane::Horizontal,
                        SurfacePlaneDefinition::Vertical => SurfacePlane::Vertical,
                    },
                    area_m2: s.area_m2,
                    span_m: s.span_m,
                    chord_m: s.chord_m,
                    incidence_rad: s.incidence_deg.to_radians(),
                    lift_slope: s.lift_slope_per_rad,
                    stall_angle_rad: s.stall_angle_deg.to_radians(),
                    cd0: s.cd0,
                    induced_drag_k: s.induced_drag_k,
                    control_axis: match s.control_axis {
                        ControlAxisDefinition::None => ControlAxis::None,
                        ControlAxisDefinition::Roll => ControlAxis::Roll,
                        ControlAxisDefinition::Pitch => ControlAxis::Pitch,
                        ControlAxisDefinition::Yaw => ControlAxis::Yaw,
                    },
                    control_sign: s.control_sign,
                    max_deflection_rad: s.max_deflection_deg.to_radians(),
                    control_effectiveness: s.control_effectiveness,
                })
                .collect(),
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
        let trim = if self.vehicle_class == VehicleClassDefinition::Multicopter {
            hover_trim(&vehicle)
        } else {
            [0.0; 4]
        };
        let mut warnings = vec![];
        if self.vehicle_class == VehicleClassDefinition::Multicopter {
            if max_thrust <= mass * GRAVITY {
                warnings.push("CRITICAL: total thrust cannot support weight".into())
            } else if max_thrust / (mass * GRAVITY) < 1.2 {
                warnings.push("WARNING: thrust-to-weight is below 1.2".into())
            }
            if hover > 0.8 {
                warnings.push("WARNING: estimated hover throttle is very high".into())
            }
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
        let wing_area = self
            .aero_surfaces
            .iter()
            .filter(|s| {
                matches!(s.plane, SurfacePlaneDefinition::Horizontal)
                    && matches!(s.control_axis, ControlAxisDefinition::Roll)
            })
            .map(|s| s.area_m2)
            .sum::<f64>();
        let wing_span = self
            .aero_surfaces
            .iter()
            .filter(|s| matches!(s.control_axis, ControlAxisDefinition::Roll))
            .map(|s| s.span_m)
            .sum::<f64>();
        let aspect_ratio = wing_span * wing_span / wing_area.max(1e-9);
        let stall_speed = (2.0 * mass * GRAVITY / (SEA_LEVEL_DENSITY * wing_area.max(1e-9) * 1.15)).sqrt();
        let cg_mac_fraction = if self.vehicle_class == VehicleClassDefinition::FixedWing {
            (0.16 - vehicle.center_of_mass().x) / 0.28
        } else {
            0.0
        };
        if self.vehicle_class == VehicleClassDefinition::FixedWing
            && !(0.20..=0.38).contains(&cg_mac_fraction)
        {
            warnings.push("WARNING: CG outside educational 20-38% MAC guidance".into());
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
            wing_area_m2: wing_area,
            aspect_ratio,
            wing_loading_kg_m2: mass / wing_area.max(1e-9),
            estimated_stall_speed_mps: stall_speed,
            power_to_weight_w_kg: max_power / mass,
            cg_mac_fraction,
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
    #[test]
    fn trainer_metrics_respond_to_wing_area_and_mass() {
        let mut larger = VehicleDefinition::fixed_wing_trainer();
        let base = larger.metrics().unwrap();
        for surface in &mut larger.aero_surfaces {
            if matches!(surface.control_axis, ControlAxisDefinition::Roll) {
                surface.area_m2 *= 1.5;
            }
        }
        let large = larger.metrics().unwrap();
        assert!(large.wing_loading_kg_m2 < base.wing_loading_kg_m2);
        assert!(large.estimated_stall_speed_mps < base.estimated_stall_speed_mps);
        larger.frame.body_mass_kg *= 1.5;
        assert!(larger.metrics().unwrap().estimated_stall_speed_mps > large.estimated_stall_speed_mps);
    }
    #[test]
    fn aft_cg_changes_properties_and_warns() {
        let mut trainer = VehicleDefinition::fixed_wing_trainer();
        let base = trainer.metrics().unwrap();
        trainer.battery.position_m.x = -0.5;
        let aft = trainer.metrics().unwrap();
        assert!(aft.center_of_mass_m.x < base.center_of_mass_m.x);
        assert!(aft.inertia_kg_m2.y > base.inertia_kg_m2.y);
        assert!(aft.warnings.iter().any(|w| w.contains("CG outside")));
    }
    #[test]
    fn configured_stall_angle_changes_physical_stall_point() {
        let mut definition = VehicleDefinition::fixed_wing_trainer();
        let air = DVec3::new(15.0, 0.0, 5.0);
        let low = definition.to_vehicle().unwrap().aero_surfaces[0].clone();
        definition.aero_surfaces[0].stall_angle_deg = 30.0;
        let high = definition.to_vehicle().unwrap().aero_surfaces[0].clone();
        assert!(crate::aerodynamic_force(&low, air, SEA_LEVEL_DENSITY, 0.0).5);
        assert!(!crate::aerodynamic_force(&high, air, SEA_LEVEL_DENSITY, 0.0).5);
    }
}
