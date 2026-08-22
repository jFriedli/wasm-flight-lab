//! Deterministic educational wind field. Wind vectors are NED air-mass
//! velocities; aviation direction is the direction the wind comes FROM.

use glam::{DVec2, DVec3};
use serde::{Deserialize, Serialize};

use crate::terrain::TerrainDefinition;

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum WeatherPreset {
    Calm,
    Breeze,
    Alpine,
    Soaring,
    StrongWind,
    Custom,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WindField {
    pub preset: WeatherPreset,
    pub speed_mps: f64,
    /// Aviation direction: degrees FROM north, clockwise.
    pub direction_from_deg: f64,
    pub gust_strength_mps: f64,
    pub turbulence_strength_mps: f64,
    pub terrain_flow: bool,
    pub thermals: bool,
    pub seed: u32,
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct WindSample {
    pub base_ned: DVec3,
    pub gust_ned: DVec3,
    pub turbulence_ned: DVec3,
    pub terrain_ned: DVec3,
    pub thermal_ned: DVec3,
    pub combined_ned: DVec3,
}

impl Default for WindField {
    fn default() -> Self {
        Self::preset(WeatherPreset::Calm)
    }
}

impl WindField {
    pub fn preset(preset: WeatherPreset) -> Self {
        let (speed, direction, gust, turbulence, terrain_flow, thermals) = match preset {
            WeatherPreset::Calm => (0.0, 270.0, 0.0, 0.0, false, false),
            WeatherPreset::Breeze => (5.0, 270.0, 1.2, 0.35, false, false),
            WeatherPreset::Alpine => (8.0, 270.0, 2.0, 0.8, true, false),
            WeatherPreset::Soaring => (7.0, 270.0, 1.0, 0.4, true, true),
            WeatherPreset::StrongWind => (14.0, 250.0, 4.0, 1.8, true, false),
            WeatherPreset::Custom => (0.0, 270.0, 0.0, 0.0, false, false),
        };
        Self {
            preset,
            speed_mps: speed,
            direction_from_deg: direction,
            gust_strength_mps: gust,
            turbulence_strength_mps: turbulence,
            terrain_flow,
            thermals,
            seed: 2_026,
        }
    }

    pub fn base_velocity_ned(&self) -> DVec3 {
        let from = self.direction_from_deg.to_radians();
        DVec3::new(-from.cos(), -from.sin(), 0.0) * self.speed_mps.max(0.0)
    }

    pub fn sample(&self, position_ned: DVec3, time_s: f64, terrain: &TerrainDefinition) -> WindSample {
        let base = self.base_velocity_ned();
        let base_direction = base.normalize_or_zero();
        let gust_scalar = self.gust_strength_mps.max(0.0)
            * (0.55 * (time_s * 0.37 + f64::from(self.seed) * 0.01).sin()
                + 0.30 * (time_s * 0.73 + 1.7).sin()
                + 0.15 * (time_s * 1.31 + 4.2).sin());
        let gust = base_direction * gust_scalar;
        let p = position_ned;
        let turbulence_scale = self.turbulence_strength_mps.max(0.0);
        let turbulence = DVec3::new(
            (p.y * 0.0031 + time_s * 0.81 + 0.3).sin(),
            (p.x * 0.0027 - time_s * 0.67 + 2.1).sin(),
            0.55 * ((p.x + p.y) * 0.0023 + time_s * 0.91 + 4.0).sin(),
        ) * turbulence_scale;

        let needs_terrain = self.terrain_flow || self.thermals;
        let ground = if needs_terrain {
            terrain.elevation_m(p.x, p.y)
        } else {
            0.0
        };
        let agl = if needs_terrain {
            (-p.z - ground).max(0.0)
        } else {
            0.0
        };
        let normal = if self.terrain_flow {
            terrain.normal_up_ned(p.x, p.y)
        } else {
            DVec3::NEG_Z
        };
        let gradient = if normal.z.abs() > 1e-6 {
            DVec2::new(normal.x / normal.z, normal.y / normal.z)
        } else {
            DVec2::ZERO
        };
        let horizontal = DVec2::new(base.x, base.y);
        let slope_rate = horizontal.dot(gradient);
        let terrain_decay = (-agl / 260.0).exp();
        let terrain_vertical = if self.terrain_flow {
            if slope_rate >= 0.0 {
                -(slope_rate * 0.38).clamp(0.0, 7.0) * terrain_decay
            } else {
                (-slope_rate * 0.18).clamp(0.0, 3.5) * terrain_decay
            }
        } else {
            0.0
        };
        let lee_strength = if self.terrain_flow {
            (-slope_rate).clamp(0.0, 12.0) * terrain_decay
        } else {
            0.0
        };
        let lee_rotor = (p.x * 0.012 + p.y * 0.009 + time_s * 0.7).sin() * lee_strength * 0.12;
        let terrain_wind = DVec3::new(
            -base_direction.y * lee_rotor,
            base_direction.x * lee_rotor,
            terrain_vertical,
        );

        let thermal_up = if self.thermals {
            thermal_updraft_mps(p.x, p.y, agl, self.seed)
        } else {
            0.0
        };
        let thermal = DVec3::new(0.0, 0.0, -thermal_up);
        let combined = base + gust + turbulence + terrain_wind + thermal;
        WindSample {
            base_ned: base,
            gust_ned: gust,
            turbulence_ned: turbulence,
            terrain_ned: terrain_wind,
            thermal_ned: thermal,
            combined_ned: if combined.is_finite() {
                combined
            } else {
                DVec3::ZERO
            },
        }
    }
}

fn thermal_updraft_mps(north: f64, east: f64, agl: f64, seed: u32) -> f64 {
    const CENTERS: [(f64, f64, f64, f64); 5] = [
        (1_050.0, 850.0, 230.0, 3.2),
        (-1_250.0, 1_150.0, 280.0, 2.8),
        (1_700.0, -900.0, 250.0, 3.6),
        (-1_650.0, -1_350.0, 300.0, 3.0),
        (400.0, 1_850.0, 220.0, 2.5),
    ];
    let seed_scale = 0.9 + f64::from(seed % 101) / 500.0;
    CENTERS
        .iter()
        .map(|(cn, ce, radius, strength)| {
            let radial = ((north - cn).hypot(east - ce) / radius).min(3.0);
            let horizontal = (-2.2 * radial * radial).exp();
            let vertical = (-(agl - 450.0).powi(2) / (2.0 * 650.0_f64.powi(2))).exp();
            strength * seed_scale * horizontal * vertical
        })
        .sum::<f64>()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::terrain::TerrainDefinition;

    #[test]
    fn aviation_wind_direction_has_correct_signs() {
        let mut wind = WindField::preset(WeatherPreset::Custom);
        wind.speed_mps = 10.0;
        wind.direction_from_deg = 270.0;
        let v = wind.base_velocity_ned();
        assert!(v.y > 9.99 && v.x.abs() < 1e-9, "west wind travels east: {v:?}");
        wind.direction_from_deg = 0.0;
        assert!(wind.base_velocity_ned().x < -9.99, "north wind travels south");
    }

    #[test]
    fn wind_is_deterministic_smooth_and_finite() {
        let wind = WindField::preset(WeatherPreset::Alpine);
        let terrain = TerrainDefinition::default();
        let position = DVec3::new(900.0, 700.0, -300.0);
        let a = wind.sample(position, 12.0, &terrain);
        let b = wind.sample(position, 12.0, &terrain);
        let nearby = wind.sample(position, 12.01, &terrain);
        assert_eq!(a.combined_ned, b.combined_ned);
        assert!((nearby.combined_ned - a.combined_ned).length() < 0.1);
        assert!(a.combined_ned.is_finite() && a.combined_ned.length() < 40.0);
    }

    #[test]
    fn ridge_effect_decays_with_height() {
        let terrain = TerrainDefinition::default();
        let mut wind = WindField::preset(WeatherPreset::Alpine);
        // Find a sampled slope and direct the base wind uphill.
        let point = (1_200.0, 1_000.0);
        let normal = terrain.normal_up_ned(point.0, point.1);
        let gradient = DVec2::new(normal.x / normal.z, normal.y / normal.z);
        let uphill_to = gradient.normalize();
        let to_deg = uphill_to.y.atan2(uphill_to.x).to_degrees();
        wind.direction_from_deg = (to_deg + 180.0).rem_euclid(360.0);
        let ground = terrain.elevation_m(point.0, point.1);
        let low = wind.sample(DVec3::new(point.0, point.1, -ground - 30.0), 0.0, &terrain);
        let high = wind.sample(DVec3::new(point.0, point.1, -ground - 900.0), 0.0, &terrain);
        assert!(low.terrain_ned.z < 0.0);
        assert!(low.terrain_ned.z.abs() > high.terrain_ned.z.abs());

        wind.direction_from_deg = to_deg.rem_euclid(360.0);
        let lee = wind.sample(DVec3::new(point.0, point.1, -ground - 30.0), 0.0, &terrain);
        assert!(lee.terrain_ned.z > 0.0, "downslope flow produces bounded sink");
    }

    #[test]
    fn thermal_is_local_and_upward() {
        let terrain = TerrainDefinition::default();
        let wind = WindField::preset(WeatherPreset::Soaring);
        let center = wind.sample(DVec3::new(1_050.0, 850.0, -450.0), 0.0, &terrain);
        let outside = wind.sample(DVec3::new(0.0, 0.0, -450.0), 0.0, &terrain);
        assert!(center.thermal_ned.z < -1.0);
        assert!(outside.thermal_ned.z.abs() < 0.05);
    }
}
