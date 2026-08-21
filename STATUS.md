# Project status

Updated: 2026-08-21

## Complete

- M0: public repository, Rust/WASM/Vite toolchain, static Pages deployment and project documentation.
- M1: deterministic 250 Hz 6-DoF state, component mass/CG, gravity, individual thrust/reaction/moment-arm torque, density and relative-air effects, collision foundation and physics vectors.
- M2: Rust-owned Acro rate and Angle flight control; three filtered PIDs with anti-windup; tested X mixer/desaturation; Beginner/Sport/Custom response profiles; fixed-time keyboard expo/slew and gradual throttle; generic Gamepad mapping, calibration, local persistence and disconnect fail-safe; chase/FPV/free cameras; reset/spawn and pause; compact controller HUD, PID editor and bounded live graph.
- Automated coverage: 18 native tests, 7 frontend unit tests and 3 browser interaction tests. Fixed-step evolution is explicitly compared across 30/60/144 Hz render schedules.

## Runtime validation

- Keyboard throttle and rate command ramping, rate braking after release, Acro/Angle switching, Angle recovery, all camera selectors, live PID edits, reset and pause are exercised locally in Chromium without console errors.
- Browser Gamepad mapping/calibration is covered with deterministic mocked-axis unit tests. No physical Gamepad/USB radio was available in the execution environment, so hardware-specific axis ordering remains user-calibrated by design.

## Known limitations

- Motor response is instantaneous and battery/power effects are not yet modelled, so controller gains are educational preset values rather than hardware recommendations.
- FPV uses a fixed mount; FREE is orbit-style rather than a six-axis fly camera.
- Ground collision remains a plane response. Airspeed currently equals ground-relative speed because wind configuration is not exposed in M2.

## Next

M3 Vehicle Workshop: editable physical components, calculated inertia, engineering warnings and safe versioned local save/import/export. Do not begin fixed-wing or VTOL UI work before the component model is editable and validated.
