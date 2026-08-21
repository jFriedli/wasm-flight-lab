use glam::{DVec3, EulerRot};
use serde::{Deserialize, Serialize};

use crate::{Pid, State};

pub const DEG_TO_RAD: f64 = std::f64::consts::PI / 180.0;

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub enum FlightMode {
    #[default]
    Acro,
    Angle,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct AxisTuning {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

impl AxisTuning {
    fn validated(self) -> Self {
        Self {
            kp: finite_clamp(self.kp, 0.0, 2.0, 0.12),
            ki: finite_clamp(self.ki, 0.0, 1.0, 0.08),
            kd: finite_clamp(self.kd, 0.0, 0.2, 0.003),
        }
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct ControlConfig {
    pub roll: AxisTuning,
    pub pitch: AxisTuning,
    pub yaw: AxisTuning,
    pub max_rate_rps: DVec3,
    pub max_angle_rad: f64,
    pub attitude_gain: f64,
    pub output_limit: f64,
}

impl Default for ControlConfig {
    fn default() -> Self {
        Self {
            roll: AxisTuning {
                kp: 0.025,
                ki: 0.008,
                kd: 0.002,
            },
            pitch: AxisTuning {
                kp: 0.025,
                ki: 0.008,
                kd: 0.002,
            },
            yaw: AxisTuning {
                kp: 0.08,
                ki: 0.015,
                kd: 0.003,
            },
            max_rate_rps: DVec3::new(130.0, 130.0, 100.0) * DEG_TO_RAD,
            max_angle_rad: 30.0 * DEG_TO_RAD,
            attitude_gain: 4.5,
            output_limit: 0.10,
        }
    }
}

impl ControlConfig {
    pub fn validated(self) -> Self {
        let defaults = Self::default();
        Self {
            roll: self.roll.validated(),
            pitch: self.pitch.validated(),
            yaw: self.yaw.validated(),
            max_rate_rps: DVec3::new(
                finite_clamp(
                    self.max_rate_rps.x,
                    20.0 * DEG_TO_RAD,
                    720.0 * DEG_TO_RAD,
                    defaults.max_rate_rps.x,
                ),
                finite_clamp(
                    self.max_rate_rps.y,
                    20.0 * DEG_TO_RAD,
                    720.0 * DEG_TO_RAD,
                    defaults.max_rate_rps.y,
                ),
                finite_clamp(
                    self.max_rate_rps.z,
                    20.0 * DEG_TO_RAD,
                    540.0 * DEG_TO_RAD,
                    defaults.max_rate_rps.z,
                ),
            ),
            max_angle_rad: finite_clamp(
                self.max_angle_rad,
                5.0 * DEG_TO_RAD,
                80.0 * DEG_TO_RAD,
                defaults.max_angle_rad,
            ),
            attitude_gain: finite_clamp(self.attitude_gain, 0.1, 20.0, defaults.attitude_gain),
            output_limit: finite_clamp(self.output_limit, 0.02, 0.5, defaults.output_limit),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct ControlTelemetry {
    pub target_rate_rps: DVec3,
    pub actual_rate_rps: DVec3,
    pub error_rps: DVec3,
    pub output: DVec3,
    pub throttle: f64,
    pub motors: [f64; 4],
}

pub struct FlightController {
    pub config: ControlConfig,
    pub mode: FlightMode,
    pub telemetry: ControlTelemetry,
    roll: Pid,
    pitch: Pid,
    yaw: Pid,
}

impl Default for FlightController {
    fn default() -> Self {
        Self::new(ControlConfig::default())
    }
}

impl FlightController {
    pub fn new(config: ControlConfig) -> Self {
        let config = config.validated();
        Self {
            roll: pid(config.roll, config.output_limit),
            pitch: pid(config.pitch, config.output_limit),
            yaw: pid(config.yaw, config.output_limit),
            config,
            mode: FlightMode::Acro,
            telemetry: ControlTelemetry::default(),
        }
    }

    pub fn set_config(&mut self, config: ControlConfig) {
        self.config = config.validated();
        self.roll.set_gains(
            self.config.roll.kp,
            self.config.roll.ki,
            self.config.roll.kd,
            self.config.output_limit,
        );
        self.pitch.set_gains(
            self.config.pitch.kp,
            self.config.pitch.ki,
            self.config.pitch.kd,
            self.config.output_limit,
        );
        self.yaw.set_gains(
            self.config.yaw.kp,
            self.config.yaw.ki,
            self.config.yaw.kd,
            self.config.output_limit,
        );
    }

    pub fn reset(&mut self) {
        self.roll.reset();
        self.pitch.reset();
        self.yaw.reset();
        self.telemetry = ControlTelemetry::default();
    }

    pub fn update(&mut self, sticks: DVec3, throttle: f64, state: &State, dt: f64) -> [f64; 4] {
        let sticks = sanitize_sticks(sticks);
        let throttle = finite_clamp(throttle, 0.0, 1.0, 0.0);
        let mut target = sticks * self.config.max_rate_rps;
        if self.mode == FlightMode::Angle {
            let (_, pitch, roll) = state.attitude_body_to_ned.to_euler(EulerRot::ZYX);
            let desired_roll = sticks.x * self.config.max_angle_rad;
            let desired_pitch = sticks.y * self.config.max_angle_rad;
            target.x = ((desired_roll - roll) * self.config.attitude_gain)
                .clamp(-self.config.max_rate_rps.x, self.config.max_rate_rps.x);
            target.y = ((desired_pitch - pitch) * self.config.attitude_gain)
                .clamp(-self.config.max_rate_rps.y, self.config.max_rate_rps.y);
        }
        let actual = state.angular_rate_body_rps;
        let output = DVec3::new(
            self.roll.update(target.x, actual.x, dt),
            self.pitch.update(target.y, actual.y, dt),
            self.yaw.update(target.z, actual.z, dt),
        );
        let motors = mix_x_quad(throttle, output);
        self.telemetry = ControlTelemetry {
            target_rate_rps: target,
            actual_rate_rps: actual,
            error_rps: target - actual,
            output,
            throttle,
            motors,
        };
        motors
    }
}

fn pid(tuning: AxisTuning, output_limit: f64) -> Pid {
    Pid::new(tuning.kp, tuning.ki, tuning.kd, 0.35, output_limit)
}

pub fn shape_stick(raw: f64, deadzone: f64, expo: f64) -> f64 {
    let raw = finite_clamp(raw, -1.0, 1.0, 0.0);
    let deadzone = finite_clamp(deadzone, 0.0, 0.4, 0.0);
    let expo = finite_clamp(expo, 0.0, 1.0, 0.0);
    if raw.abs() <= deadzone {
        return 0.0;
    }
    let normalized = raw.signum() * ((raw.abs() - deadzone) / (1.0 - deadzone));
    (1.0 - expo) * normalized + expo * normalized.powi(3)
}

pub fn slew(current: f64, target: f64, attack_per_s: f64, release_per_s: f64, dt: f64) -> f64 {
    let target = finite_clamp(target, -1.0, 1.0, 0.0);
    let rate = if target.abs() > current.abs() {
        attack_per_s
    } else {
        release_per_s
    };
    let max_change = finite_clamp(rate, 0.0, 100.0, 0.0) * finite_clamp(dt, 0.0, 0.1, 0.0);
    current + (target - current).clamp(-max_change, max_change)
}

pub fn mix_x_quad(collective: f64, effort: DVec3) -> [f64; 4] {
    let c = finite_clamp(collective, 0.0, 1.0, 0.0);
    let e = if effort.is_finite() { effort } else { DVec3::ZERO };
    let differential = [
        -e.x + e.y - e.z,
        e.x + e.y + e.z,
        e.x - e.y - e.z,
        -e.x - e.y + e.z,
    ];
    let min_d = differential.iter().copied().fold(f64::INFINITY, f64::min);
    let max_d = differential.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let scale = if max_d - min_d > 1.0 {
        1.0 / (max_d - min_d)
    } else {
        1.0
    };
    let scaled = differential.map(|v| v * scale);
    let low = scaled.iter().copied().fold(f64::INFINITY, f64::min);
    let high = scaled.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let shifted_collective = c.clamp(-low, 1.0 - high);
    scaled.map(|v| (shifted_collective + v).clamp(0.0, 1.0))
}

fn sanitize_sticks(sticks: DVec3) -> DVec3 {
    if sticks.is_finite() {
        sticks.clamp(DVec3::splat(-1.0), DVec3::ONE)
    } else {
        DVec3::ZERO
    }
}

fn finite_clamp(value: f64, min: f64, max: f64, fallback: f64) -> f64 {
    if value.is_finite() {
        value.clamp(min, max)
    } else {
        fallback
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Simulator, beginner_quad};
    use glam::DQuat;

    #[test]
    fn expo_endpoints_and_monotonicity() {
        for expo in [0.0, 0.45, 1.0] {
            assert_eq!(shape_stick(-1.0, 0.05, expo), -1.0);
            assert_eq!(shape_stick(0.0, 0.05, expo), 0.0);
            assert_eq!(shape_stick(1.0, 0.05, expo), 1.0);
            let values = (-100..=100)
                .map(|x| shape_stick(x as f64 / 100.0, 0.05, expo))
                .collect::<Vec<_>>();
            assert!(values.windows(2).all(|w| w[0] <= w[1]));
        }
    }
    #[test]
    fn slew_is_time_based() {
        let one = slew(0.0, 1.0, 2.0, 3.0, 0.04);
        let mut many = 0.0;
        for _ in 0..10 {
            many = slew(many, 1.0, 2.0, 3.0, 0.004);
        }
        assert!((one - many).abs() < 1e-12);
    }
    #[test]
    fn mixer_collective_is_equal() {
        assert_eq!(mix_x_quad(0.4, DVec3::ZERO), [0.4; 4]);
    }
    #[test]
    fn mixer_axes_have_documented_signs() {
        let r = mix_x_quad(0.5, DVec3::new(0.1, 0.0, 0.0));
        assert!(r[1] > r[0] && r[2] > r[3]);
        let p = mix_x_quad(0.5, DVec3::new(0.0, 0.1, 0.0));
        assert!(p[0] > p[2] && p[1] > p[3]);
        let y = mix_x_quad(0.5, DVec3::new(0.0, 0.0, 0.1));
        assert!(y[1] > y[0] && y[3] > y[2]);
    }
    #[test]
    fn mixer_is_finite_and_bounded() {
        for e in [DVec3::splat(10.0), DVec3::splat(f64::NAN)] {
            assert!(
                mix_x_quad(2.0, e)
                    .iter()
                    .all(|v| v.is_finite() && (0.0..=1.0).contains(v))
            );
        }
    }
    #[test]
    fn rate_target_drives_requested_direction() {
        let mut sim = Simulator::new(beginner_quad());
        sim.set_control(DVec3::new(0.25, 0.0, 0.0), 0.31);
        for _ in 0..150 {
            sim.step(0.004);
        }
        assert!(
            sim.state.angular_rate_body_rps.x > 0.05,
            "rate {:?}, telemetry {:?}",
            sim.state.angular_rate_body_rps,
            sim.controller.telemetry
        );
        assert!(sim.controller.telemetry.error_rps.x.abs() < 0.7);
    }
    #[test]
    fn rate_controller_brakes_after_stick_release() {
        let mut sim = Simulator::new(beginner_quad());
        sim.set_control(DVec3::new(0.3, 0.0, 0.0), 0.31);
        for _ in 0..100 {
            sim.step(0.004);
        }
        let driven_rate = sim.state.angular_rate_body_rps.x.abs();
        sim.set_control(DVec3::ZERO, 0.31);
        for _ in 0..150 {
            sim.step(0.004);
        }
        assert!(driven_rate > 0.1);
        assert!(sim.state.angular_rate_body_rps.x.abs() < driven_rate * 0.3);
    }
    #[test]
    fn angle_mode_levels() {
        let mut sim = Simulator::new(beginner_quad());
        sim.state.attitude_body_to_ned = DQuat::from_rotation_x(0.25);
        sim.controller.mode = FlightMode::Angle;
        sim.set_control(DVec3::ZERO, 0.31);
        for _ in 0..300 {
            sim.step(0.004);
        }
        let (_, _, roll) = sim.state.attitude_body_to_ned.to_euler(EulerRot::ZYX);
        assert!(
            roll.abs() < 0.25,
            "roll {roll}, rate {:?}, telemetry {:?}",
            sim.state.angular_rate_body_rps,
            sim.controller.telemetry
        );
    }
    #[test]
    fn stick_limits_map_to_configured_rates() {
        let mut controller = FlightController::default();
        let state = State::default();
        controller.update(DVec3::ZERO, 0.3, &state, 0.004);
        assert_eq!(controller.telemetry.target_rate_rps, DVec3::ZERO);
        controller.update(DVec3::ONE, 0.3, &state, 0.004);
        assert_eq!(
            controller.telemetry.target_rate_rps,
            controller.config.max_rate_rps
        );
        assert!((130.0 * DEG_TO_RAD * 180.0 / std::f64::consts::PI - 130.0).abs() < 1e-12);
    }
    #[test]
    fn reset_clears_controller_transients() {
        let mut controller = FlightController::default();
        controller.update(DVec3::ONE, 0.5, &State::default(), 0.004);
        controller.reset();
        assert_eq!(controller.telemetry.target_rate_rps, DVec3::ZERO);
        assert_eq!(controller.telemetry.output, DVec3::ZERO);
    }
    #[test]
    fn render_cadence_does_not_change_fixed_step_evolution() {
        fn run(render_hz: u32) -> State {
            let mut sim = Simulator::new(beginner_quad());
            let mut accumulator = 0.0;
            let mut steps = 0;
            while steps < 500 {
                accumulator += 1.0 / f64::from(render_hz);
                while accumulator + 1e-12 >= 0.004 && steps < 500 {
                    let command = if steps < 250 { 0.18 } else { -0.1 };
                    sim.set_control(DVec3::new(command, 0.05, 0.02), 0.31);
                    sim.step(0.004);
                    accumulator -= 0.004;
                    steps += 1;
                }
            }
            sim.state
        }
        let a = run(30);
        for hz in [60, 144] {
            let b = run(hz);
            assert!((a.position_ned_m - b.position_ned_m).length() < 1e-12);
            assert!((a.angular_rate_body_rps - b.angular_rate_body_rps).length() < 1e-12);
        }
    }
}
