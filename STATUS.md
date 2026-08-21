# Project status

Updated: 2026-08-21

## Complete

- M0: public repository, Rust/WASM/Vite toolchain, static Pages deployment and project documentation.
- M1: deterministic 250 Hz 6-DoF state, component mass/CG, gravity, individual thrust/reaction/moment-arm torque, density and relative-air effects, collision foundation and physics vectors.
- M2: Rust-owned Acro rate and Angle flight control; three filtered PIDs with anti-windup; tested X mixer/desaturation; Beginner/Sport/Custom response profiles; fixed-time keyboard expo/slew and gradual throttle; generic Gamepad mapping, calibration, local persistence and disconnect fail-safe; chase/FPV/free cameras; reset/spawn and pause; compact controller HUD, PID editor and bounded live graph.
- P0 frame audit: retained correct Rust body-to-NED force rotation; replaced the reflected render basis and quaternion sign shuffle with centralized proper basis transforms; total/per-motor authoritative force vectors and known-attitude diagnostics; native and frontend direction regressions.
- M3: live BUILD workshop; Beginner/Freestyle definitions; editable frame, motors, propellers, positioned battery and payloads; component-derived mass/CG/diagonal inertia; static hover allocation; engineering metrics/warnings; first-order motor response; battery consumption/sag; visual component/CG preview; exact-definition test flight; bounded local save/duplicate/delete and secure JSON import/export.
- Automated coverage: 32 native tests, 15 frontend unit tests and 5 browser interaction tests. Fixed-step evolution is explicitly compared across 30/60/144 Hz render schedules.

## Runtime validation

- Keyboard throttle and rate command ramping, rate braking after release, Acro/Angle switching, Angle recovery, all camera selectors, live PID edits, reset and pause are exercised locally in Chromium without console errors.
- Browser Gamepad mapping/calibration is covered with deterministic mocked-axis unit tests. No physical Gamepad/USB radio was available in the execution environment, so hardware-specific axis ordering remains user-calibrated by design.
- Known roll/pitch attitudes are checked against rendered model thrust, authoritative thrust and lateral velocity. Workshop browser coverage proves a motor-position edit reaches preview, persistence and the runtime definition used for flight.

## Known limitations

- Propeller and battery models are deliberately empirical estimates; no blade-element or electrochemical/thermal model exists. Controller gains are educational rather than hardware recommendations.
- FPV uses a fixed mount; FREE is orbit-style rather than a six-axis fly camera.
- Ground collision remains a plane response. Airspeed currently equals ground-relative speed because wind configuration is not exposed in M2.

## Next

M4 Fixed Wing: wing/control-surface components, real airspeed-dependent takeoff, stall and runway landing. Do not begin VTOL before the fixed-wing force/control path is validated.
