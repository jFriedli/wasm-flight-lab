use glam::DVec3;
use serde::{Deserialize, Serialize};

pub const DEFAULT_TERRAIN_SEED: u32 = 1337;
pub const WORLD_HALF_SIZE_M: f64 = 3_200.0;
pub const LAKE_CENTER_NED_M: (f64, f64) = (900.0, -520.0);
pub const LAKE_LEVEL_M: f64 = 5.0;

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum TerrainKind {
    TestRange,
    AlpineRange,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct TerrainDefinition {
    pub kind: TerrainKind,
    pub seed: u32,
}

impl Default for TerrainDefinition {
    fn default() -> Self {
        Self {
            kind: TerrainKind::AlpineRange,
            seed: DEFAULT_TERRAIN_SEED,
        }
    }
}

impl TerrainDefinition {
    /// Terrain elevation above the airfield datum in metres.
    pub fn elevation_m(&self, north_m: f64, east_m: f64) -> f64 {
        if self.kind == TerrainKind::TestRange {
            return 0.0;
        }
        alpine_elevation(self.seed, north_m, east_m)
    }

    /// Ground coordinate in NED, where positive Z points down.
    pub fn ground_down_m(&self, north_m: f64, east_m: f64) -> f64 {
        -self.elevation_m(north_m, east_m)
    }

    pub fn normal_up_ned(&self, north_m: f64, east_m: f64) -> DVec3 {
        let spacing = 4.0;
        let dn = (self.elevation_m(north_m + spacing, east_m) - self.elevation_m(north_m - spacing, east_m))
            / (2.0 * spacing);
        let de = (self.elevation_m(north_m, east_m + spacing) - self.elevation_m(north_m, east_m - spacing))
            / (2.0 * spacing);
        DVec3::new(-dn, -de, -1.0).normalize_or_zero()
    }
}

fn smoothstep(edge0: f64, edge1: f64, value: f64) -> f64 {
    let t = ((value - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

fn fade(t: f64) -> f64 {
    t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
}

fn hash(seed: u32, x: i32, y: i32) -> f64 {
    let mut value = seed ^ (x as u32).wrapping_mul(0x9e37_79b9) ^ (y as u32).wrapping_mul(0x85eb_ca6b);
    value ^= value >> 16;
    value = value.wrapping_mul(0x7feb_352d);
    value ^= value >> 15;
    value = value.wrapping_mul(0x846c_a68b);
    value ^= value >> 16;
    f64::from(value) / f64::from(u32::MAX) * 2.0 - 1.0
}

fn value_noise(seed: u32, x: f64, y: f64) -> f64 {
    let x0 = x.floor() as i32;
    let y0 = y.floor() as i32;
    let tx = fade(x - f64::from(x0));
    let ty = fade(y - f64::from(y0));
    let a = hash(seed, x0, y0);
    let b = hash(seed, x0 + 1, y0);
    let c = hash(seed, x0, y0 + 1);
    let d = hash(seed, x0 + 1, y0 + 1);
    let ab = a + (b - a) * tx;
    let cd = c + (d - c) * tx;
    ab + (cd - ab) * ty
}

fn fbm(seed: u32, x: f64, y: f64, octaves: usize) -> f64 {
    let mut sum = 0.0;
    let mut amplitude = 0.5;
    let mut frequency = 1.0;
    for octave in 0..octaves {
        sum += value_noise(
            seed.wrapping_add(octave as u32 * 1013),
            x * frequency,
            y * frequency,
        ) * amplitude;
        frequency *= 2.03;
        amplitude *= 0.5;
    }
    sum
}

fn gaussian(value: f64, scale: f64) -> f64 {
    (-(value / scale).powi(2)).exp()
}

fn alpine_elevation(seed: u32, north_m: f64, east_m: f64) -> f64 {
    let valley_wall = smoothstep(260.0, 1_750.0, east_m.abs()).powf(1.35);
    let pass_carve = [(-1_550.0, 310.0), (250.0, 380.0), (1_650.0, 330.0)]
        .iter()
        .map(|(north, width)| gaussian(north_m - north, *width))
        .fold(0.0_f64, f64::max);
    let major = 650.0 * valley_wall * (1.0 - 0.55 * pass_carve);
    let broad = 105.0 * fbm(seed, north_m / 1_250.0, east_m / 1_050.0, 5);
    let ridge_noise = 1.0 - value_noise(seed.wrapping_add(71), north_m / 720.0, east_m / 620.0).abs();
    let ridges = 190.0 * ridge_noise.powi(3) * (0.2 + 0.8 * valley_wall);
    let detail = 32.0 * fbm(seed.wrapping_add(911), north_m / 260.0, east_m / 260.0, 4);
    let peaks = 360.0 * gaussian((north_m - 1_850.0).hypot(east_m - 1_650.0), 520.0)
        + 290.0 * gaussian((north_m + 1_650.0).hypot(east_m + 1_500.0), 480.0)
        + 240.0 * gaussian((north_m - 350.0).hypot(east_m + 2_050.0), 430.0);
    let mut elevation = 18.0 + major + broad + ridges + detail + peaks;

    let runway_blend =
        1.0 - smoothstep(500.0, 700.0, north_m.abs()).max(smoothstep(90.0, 240.0, east_m.abs()));
    elevation *= 1.0 - runway_blend;

    let (lake_north, lake_east) = LAKE_CENTER_NED_M;
    let lake_distance = (north_m - lake_north).hypot(east_m - lake_east);
    let basin = 1.0 - smoothstep(270.0, 390.0, lake_distance);
    elevation = elevation * (1.0 - basin) + (-12.0) * basin;
    elevation.clamp(-20.0, 1_250.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn terrain_is_deterministic_and_seeded() {
        let terrain = TerrainDefinition::default();
        let first = terrain.elevation_m(1_234.5, -987.0);
        assert_eq!(first, terrain.elevation_m(1_234.5, -987.0));
        let other = TerrainDefinition { seed: 42, ..terrain };
        assert!((first - other.elevation_m(1_234.5, -987.0)).abs() > 1e-3);
    }

    #[test]
    fn runway_is_flat_and_lake_is_below_water() {
        let terrain = TerrainDefinition::default();
        for north in [-450.0, 0.0, 450.0] {
            for east in [-75.0, 0.0, 75.0] {
                assert!(terrain.elevation_m(north, east).abs() < 1e-9);
            }
        }
        assert!(terrain.elevation_m(LAKE_CENTER_NED_M.0, LAKE_CENTER_NED_M.1) < LAKE_LEVEL_M);
    }

    #[test]
    fn terrain_samples_and_normals_are_bounded() {
        let terrain = TerrainDefinition::default();
        for north in (-3..=3).map(|value| f64::from(value) * 900.0) {
            for east in (-3..=3).map(|value| f64::from(value) * 900.0) {
                let elevation = terrain.elevation_m(north, east);
                let normal = terrain.normal_up_ned(north, east);
                assert!(elevation.is_finite() && (-20.0..=1_250.0).contains(&elevation));
                assert!(normal.is_finite() && (normal.length() - 1.0).abs() < 1e-9);
                assert!(normal.z < 0.0);
            }
        }
    }
}
