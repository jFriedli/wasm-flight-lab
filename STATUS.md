# Project status

Updated: 2026-08-21

## Complete

- M0: public repository, Rust/WASM/Vite toolchain, static Pages deployment and project documentation.
- M1: deterministic 250 Hz 6-DoF state, component mass/CG, gravity, individual thrust/reaction/moment-arm torque, density and relative-air effects, collision foundation and physics vectors.
- M2: Rust-owned Acro rate and Angle flight control; three filtered PIDs with anti-windup; tested X mixer/desaturation; Beginner/Sport/Custom response profiles; fixed-time keyboard expo/slew and gradual throttle; generic Gamepad mapping, calibration, local persistence and disconnect fail-safe; chase/FPV/free cameras; reset/spawn and pause; compact controller HUD, PID editor and bounded live graph.
- P0 frame audit: retained correct Rust body-to-NED force rotation; replaced the reflected render basis and quaternion sign shuffle with centralized proper basis transforms; total/per-motor authoritative force vectors and known-attitude diagnostics; native and frontend direction regressions.
- M3: live BUILD workshop; Beginner/Freestyle definitions; editable frame, motors, propellers, positioned battery and payloads; component-derived mass/CG/diagonal inertia; static hover allocation; engineering metrics/warnings; first-order motor response; battery consumption/sag; visual component/CG preview; exact-definition test flight; bounded local save/duplicate/delete and secure JSON import/export.
- Desktop RC controls: persistent W/S throttle, A/D yaw, bounded pointer-lock mouse roll/pitch, configurable sensitivity/spring, visible virtual stick, Arrow fallback, explicit source selection and unchanged generic Gamepad support.
- M4 vertical slice: component-based 1.7 m Fixed Wing Trainer; one shared forward propulsor; separate left/right wing, elevator and rudder force points; air-relative AoA, dynamic-pressure lift, profile/induced/post-stall drag, continuous lift loss, airspeed-dependent surface authority and aerodynamic rate damping; simplified rolling contact and physical runway takeoff; fixed-wing HUD, stall warning, surface vectors, scaled model/animated surfaces, BUILD editors, wing loading/stall speed/CG guidance.
- Automated coverage: 47 native tests, 17 frontend unit tests and 7 browser interaction tests. Fixed-step evolution is explicitly compared across 30/60/144 Hz render schedules.

## Runtime validation

- Keyboard throttle and rate command ramping, rate braking after release, Acro/Angle switching, Angle recovery, all camera selectors, live PID edits, reset and pause are exercised locally in Chromium without console errors.
- Browser Gamepad mapping/calibration is covered with deterministic mocked-axis unit tests. No physical Gamepad/USB radio was available in the execution environment, so hardware-specific axis ordering remains user-calibrated by design.
- Known roll/pitch attitudes are checked against rendered model thrust, authoritative thrust and lateral velocity. Workshop browser coverage proves a motor-position edit reaches preview, persistence and the runtime definition used for flight.
- Native integration proves runway acceleration and aerodynamic takeoff. Browser coverage selects/edits the trainer, reaches flying speed, commands elevator and observes it become airborne. Pointer capture, bounded displacement, spring return and release are browser-tested.

## Known limitations

- Propeller and battery models are deliberately empirical estimates; no blade-element or electrochemical/thermal model exists. Controller gains are educational rather than hardware recommendations.
- FPV uses a fixed mount; FREE is orbit-style rather than a six-axis fly camera.
- Ground collision remains a plane response. With the current default zero wind, airspeed and ground speed coincide numerically; the authoritative relative-air architecture is ready for M6 wind inputs.
- Fixed-wing ground handling is deliberately simple; there are no wheel/suspension or detailed crash models. Manual circuits and landings require pilot finesse and the model is not tuned as a high-fidelity trainer.

## Next

M5 QuadPlane and Tiltrotor: compose vertical/forward propulsion with the now-shared aerodynamic surfaces, preserving continuous force-vector transition.
