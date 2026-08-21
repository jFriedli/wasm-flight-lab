# Project status

Updated: 2026-08-21

## Complete

- M0: public repository, Rust workspace, real WASM call, Vite/Three.js static application, CI and Pages workflow, core documentation.
- M1 foundation: deterministic 250 Hz stepping; 6-DoF state; gravity; individual thrust, reaction and moment-arm torque; density scaling; relative-air drag; simple collision; quad preset; thrust vector display.
- Test foundations: native numeric physics tests, frontend unit tests, and Playwright WASM-start smoke test.

## Partial

- M2: direct keyboard collective/roll/pitch, reset and chase camera exist. Rust PID primitive is tested but not connected as a complete rate controller. Gamepad/calibration and FPV/free cameras remain.
- M3/M4 architecture: physical masses, motors and wings exist; center of mass is calculated and a simplified stall-capable wing force is implemented. The complete editor, presets and control surfaces remain.

## Next

Complete M2 before widening scope: rate PID mixer, stabilized mode, controller tuning/telemetry, Gamepad calibration persistence, then camera modes. Do not mark later milestones complete until their acceptance behavior has deterministic tests.

