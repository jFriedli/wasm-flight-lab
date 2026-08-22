# Project status

Updated: 2026-08-22

## Complete

- M0: public repository, Rust/WASM/Vite toolchain, static Pages deployment and project documentation.
- M1: deterministic 250 Hz 6-DoF state, component mass/CG, gravity, individual thrust/reaction/moment-arm torque, density and relative-air effects, collision foundation and physics vectors.
- M2: Rust-owned Acro rate and Angle flight control; three filtered PIDs with anti-windup; tested X mixer/desaturation; Beginner/Sport/Custom response profiles; fixed-time keyboard expo/slew and gradual throttle; generic Gamepad mapping, calibration, local persistence and disconnect fail-safe; chase/FPV/free cameras; reset/spawn and pause; compact controller HUD, PID editor and bounded live graph.
- P0 frame audit: retained correct Rust body-to-NED force rotation; replaced the reflected render basis and quaternion sign shuffle with centralized proper basis transforms; total/per-motor authoritative force vectors and known-attitude diagnostics; native and frontend direction regressions.
- M3: live BUILD workshop; Beginner/Freestyle definitions; editable frame, motors, propellers, positioned battery and payloads; component-derived mass/CG/diagonal inertia; static hover allocation; engineering metrics/warnings; first-order motor response; battery consumption/sag; visual component/CG preview; exact-definition test flight; bounded local save/duplicate/delete and secure JSON import/export.
- Desktop RC controls: persistent W/S throttle, A/D yaw, bounded pointer-lock mouse roll/pitch, configurable sensitivity/spring, visible virtual stick, Arrow fallback, explicit source selection and unchanged generic Gamepad support.
- M4 vertical slice: component-based 1.7 m Fixed Wing Trainer; one shared forward propulsor; separate left/right wing, elevator and rudder force points; air-relative AoA, dynamic-pressure lift, profile/induced/post-stall drag, continuous lift loss, airspeed-dependent surface authority and aerodynamic rate damping; simplified rolling contact and physical runway takeoff; fixed-wing HUD, stall warning, surface vectors, scaled model/animated surfaces, BUILD editors, wing loading/stall speed/CG guidance.
- Fixed-wing stability correction: surface mass/intrinsic inertia, full `ω×r` local airflow, emergent roll/pitch/yaw damping, bounded surface effectiveness, rate-limited servos, cruise trim, static-margin guidance, trainer-specific input scaling and true rate telemetry.
- Alpine Range: 6.4 km deterministic seed-1337 multi-octave/ridged heightfield, three passes/peaks, flat 700 m airfield, lake, bridge, huts, hangar, antenna, 550 instanced trees, authoritative height/normal collision, minimap, compass, N/E coordinates and home bearing/distance. TEST RANGE remains selectable.
- M5 experimental slice: QuadPlane Explorer and Tiltrotor Research VTOL presets; shared rigid body/surfaces/battery/propulsion; physical hover/cruise forces, airspeed-aware QuadPlane allocation, continuous 30°/s nacelle tilt, manual T/G/slider transition, BUILD fields, telemetry and force vectors.
- Flight-quality correction: exact neutral input diagnostics; balanced quad/VTOL hover thrust lines; VTOL-only transition-scaled wing trim; projected-area multicopter drag; corrected 5-inch Freestyle propulsion; distinct Beginner/Sport selection; and increased but bounded trainer aileron effectiveness.
- M6 atmosphere vertical slice: Rust-owned aviation-direction base wind, coherent gusts, deterministic turbulence, terrain-gradient ridge lift, lee sink/rotor, five thermals, six weather choices, authoritative windsock and sparse component airflow vectors.
- Automated coverage: 82 native tests, 20 frontend unit tests and 10 browser interaction tests. Fixed-step evolution is explicitly compared across 30/60/144 Hz render schedules.

## Runtime validation

- Keyboard throttle and rate command ramping, rate braking after release, Acro/Angle switching, Angle recovery, all camera selectors, live PID edits, reset and pause are exercised locally in Chromium without console errors.
- Browser Gamepad mapping/calibration is covered with deterministic mocked-axis unit tests. No physical Gamepad/USB radio was available in the execution environment, so hardware-specific axis ordering remains user-calibrated by design.
- Known roll/pitch attitudes are checked against rendered model thrust, authoritative thrust and lateral velocity. Workshop browser coverage proves a motor-position edit reaches preview, persistence and the runtime definition used for flight.
- Native integration proves runway acceleration and aerodynamic takeoff. Browser coverage selects/edits the trainer, reaches flying speed, commands elevator and observes it become airborne. Pointer capture, bounded displacement, spring return and release are browser-tested.
- A 25-pixel local mouse correction after takeoff produced a measured peak body rate of 22.5°/s without console errors. Native gentle-control cruise runs 30 simulated seconds and bounds rate below 2.5 rad/s.
- Local Chromium generated Alpine Range in about 2.7 seconds without errors. QuadPlane held 1.5 m AGL at its calculated 49% hover command; tiltrotor held near hover at 29%. Both exposed continuous transition telemetry and rotating force components.
- Neutral local browser sampling reported raw/normalized pitch and pitch target exactly zero for both quads and both VTOL presets. Quad/VTOL equal-hover propulsion pitch moments are below test tolerance; wing trim is zero in hover.
- The Beginner analytical 30° level-translation estimate is 21.7 m/s (10 m/s in about 1.9 s); Freestyle at 45° is 39.3 m/s (10 m/s in about 1.0 s, 20 m/s in about 2.25 s). These are still-air model benchmarks, not claimed flight envelopes.
- Local browser runs reached 12.3 m/s after six seconds at Beginner's 30° Angle limit and 28.1 m/s after six seconds at Freestyle's 45° limit; both retained altitude with pilot-selected collective.
- The trainer reached 30° bank in about 1.3 s during local browser validation at 24.7 m/s; sampled roll rate was about 26.5°/s and remained finite. Native repeated-transition tests exercise three forward/back cycles for each VTOL.
- With aerodynamic control allocation enabled, local Tiltrotor transition reached 35.6 m/s wing-borne flight at approximately 9° pitch; rotor/aerodynamic pitch moments balanced at about -0.64/+0.64 Nm. QuadPlane reached wing-borne status at 10.8 m/s with approximately 8° pitch. Both returned finite hover-oriented states in transition testing.
- SOARING weather produced a 7.6 m/s local field in Chromium; stationary/slow ground motion showed distinct airspeed, the windsock and sparse arrows agreed with the authoritative vector, and no console errors occurred.

## Known limitations

- Propeller and battery models are deliberately empirical estimates; no blade-element or electrochemical/thermal model exists. Controller gains are educational rather than hardware recommendations.
- FPV uses a fixed mount; FREE is orbit-style rather than a six-axis fly camera.
- Terrain contact uses the authoritative environment height and normal beneath the vehicle. CALM remains the default; weather presets make airspeed and ground speed diverge through authoritative relative airflow.
- Fixed-wing ground handling is deliberately simple; there are no wheel/suspension or detailed crash models. Manual circuits and landings require pilot finesse and the model is not tuned as a high-fidelity trainer.
- Terrain uses one 128×128 height mesh and point-height contact rather than streaming LOD or detailed landing-gear collision. Trees and landmarks are visual scale cues without individual collision geometry.
- M5 remains pilot-workload intensive: there is no transition/altitude assist, the tiltrotor uses four rotors for full hover control authority, and repeated native transitions do not replace broader human landing/circuit testing with physical controllers.
- Gust/turbulence/orographic flow are deterministic educational approximations rather than certified spectra or CFD. Density lapse rate, clouds and precipitation are not modelled.

## Next

M7 sensors, navigation, LIDAR and failure injection; continue M5 pilot-assist and hardware-controller flight testing alongside it.
