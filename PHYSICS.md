# Physics model

## Coordinate frames and units

All core values use SI units.

- NED world is right handed: +X North, +Y East, +Z Down.
- Aircraft body is right handed: +X Forward, +Y Right, +Z Down.
- Three.js world is right handed: +X East, +Y Up, +Z South. Thus `(N,E,D) → (E,-D,-N)`; the basis determinant is +1.
- The procedural Three model uses local +X Right, +Y Up and -Z Forward. Its fixed model-to-body basis is applied exactly once.

`attitude_body_to_ned` is a glam quaternion that actively rotates body vectors into NED. `motor.direction` and motor positions are body-frame values. Therefore a motor force enters translation as `force_ned = attitude_body_to_ned * force_body`, and its body torque is `(motor_position - CG) × force_body`. Rendering composes proper rotation matrices as `world_from_ned × body_to_ned × body_from_model`, then converts that result to a Three.js quaternion. Positions, velocity, wind and every debug force share the same NED conversion helpers. Reflections cannot be represented by quaternions; the previous `(N,E,D) → (E,-D,N)` mapping had determinant -1 and was removed.

## Modelled

- Six-degree-of-freedom translational/rotational state with semi-implicit Euler integration at 250 Hz.
- Gravity, quadratic body drag, per-motor thrust, moment arm torque `τ = (r - CG) × F`, and rotor reaction torque.
- Component-derived mass, center of mass and diagonal inertia. The central body uses a box approximation; motor, propeller, battery and payload point masses use the parallel-axis theorem about the computed CG.
- Air-relative velocity (`vehicle velocity - wind`), density-scaled propulsion, and simplified finite wing forces.
- Multiple wing/tail forces, airflow-dependent control surfaces and continuous stall attenuation as described below.
- A simple inelastic ground plane collision response.
- Acro body-rate control and an Angle attitude outer loop feeding three independent body-rate PIDs.
- An X-quad mixer whose signs follow +X forward, +Y right and +Z down.

## Flight controller

Beginner maximum rates are 130°/s roll and pitch and 100°/s yaw, stored internally as radians/second. Beginner Angle mode is limited to ±30°. Roll/pitch default gains are P 0.025, I 0.008 and D 0.002; yaw uses P 0.08, I 0.015 and D 0.003. Normalized axis outputs are limited to ±0.10.

PID derivative is taken on measurement to avoid derivative kick and low-pass filtered with a 20 ms time constant. Integration is bounded and conditionally stopped when saturation would deepen windup. Reset and mode changes clear controller transients.

Positive roll increases the left motor pair, positive pitch increases the front pair, and positive yaw increases motors whose reaction torque acts toward +Z. If requested mixer differentials exceed the available range, the mixer scales them together to preserve direction. Otherwise it shifts collective into available headroom before final numerical clamping.

Input expo uses `(1-e)x + ex³` after deadzone rescaling. It is monotonic and preserves -1, 0 and +1. Beginner keyboard commands slew at 1.6 normalized units/s on attack and 3.2 units/s on release; throttle changes at 0.16/s. These updates use fixed simulation time.

## Vehicle and energy approximations

Each motor command passes through a first-order response with separate configurable spin-up/down time constants. Actual output—not commanded output—sets thrust and power. Propeller diameter, pitch, blade count and an efficiency estimate scale a declared reference static thrust using a bounded empirical relationship. This is intentionally not blade-element analysis or CFD.

Power is estimated as `Pmax × output^1.5`. Battery state integrates current consumption; open-circuit cell voltage varies linearly with state of charge and load sag is `I × internal resistance`, with a conservative voltage floor. Hover time reserves 20% nominal energy. These outputs are educational estimates.

Static hover allocation solves four linear equations for vertical force and three body moments. It exposes the command imbalance caused by an offset CG and warns when the solution saturates.

### Flight-quality balance and multicopter drag

Neutral mouse and calibrated Gamepad centres now resolve to exact zero before Rust receives the normalized stick. Acro zero pitch requests zero body pitch rate; Angle zero pitch requests a level attitude. Fixed-wing elevator trim is applied fully only to the Fixed Wing Trainer and is blended with actual transition for winged VTOLs, so it cannot bias hover.

The earlier quad presets placed the battery/camera combined CG slightly aft of their symmetric rotor thrust centroid. That small but real moment, plus inherited wing trim on VTOLs, caused the common nose-up observation. Default lift-motor layouts are now centered on the component-derived non-hover mass balance point, and the quad battery positions balance their camera payload. Equal hover thrust consequently has negligible roll/pitch moment without controller compensation.

Multicopter body drag is evaluated in body axes from projected frame areas. For each axis `i`, `D_i = -½ ρ Cd A_i |V_i|V_i`, with `Cd = 0.9` and `A = (YZ, XZ, XY)`, then transformed to NED. The old isotropic coefficient produced about 39 N at 20 m/s—more than the Beginner Quad's entire static thrust. The corrected Beginner forward drag is about 4.85 N at 20 m/s. Analytical level-translation estimates are about 21.7 m/s at 30° for Beginner and 39.3 m/s at 45° for the physically smaller Freestyle frame; these are educational still-air estimates, not guaranteed flight envelopes.

## Fixed-wing aerodynamics

The Fixed Wing Trainer is an educational 1.7 m-span, 0.48 m²-wing aircraft of approximately 1.3 kg. Its 4-cell/3300 mAh battery drives one 15 N, 520 W forward propulsor. The conventional tail has a 0.10 m² elevator and 0.065 m² rudder. It is not based on a commercial airframe.

Authoritative air velocity is `v_air,NED = v_vehicle,NED - v_wind,NED`. The body-to-NED quaternion inverse transforms it into body coordinates. A surface at offset `r` from the CG samples the full rigid-body local velocity `v_surface,body = v_CG,body + ω_body × r_body`. A pitching tail, yawing fin, and rolling left/right wing therefore see different airflow and generate opposing aerodynamic moments. No frame-dependent angular-velocity multiplier or global damping torque is used.

For horizontal surfaces, angle of attack is `α = atan2(Vz, Vx) + incidence + control_effect`. Because body +Z is down, a nose above its flight path produces positive body `Vz` and positive α. Vertical surfaces use sideslip `β = atan2(Vy, Vx)`. AoA is derived from airflow, never vehicle pitch alone.

Dynamic pressure is `q = ½ρV²`. Before stall, `CL = CLα α`. At the configured critical magnitude `αs`, CL is continuous; beyond it, `CL = CLα αs sign(α)(αs/|α|)^1.2`, so lift physically falls instead of increasing indefinitely. Lift magnitude is `q S CL` and its direction is perpendicular to the local planar airflow. Near-zero airflow produces near-zero force.

Drag is opposite local relative velocity with `CD = CD0 + k CL² + 1.2 e²`, where `e = max(0,(|α|-αs)/αs)`. The final term supplies continuous post-stall pressure drag. Generic fuselage drag is reduced for fixed wings to avoid double-counting surface profile drag.

Every left/right wing, elevator and rudder has its own body position. Rust applies `τ = (r-CG) × (L+D)`. Ailerons use opposite incidence changes; elevator and rudder change their tail surface incidence. Consequently authority scales with dynamic pressure and is negligible while stationary. Control commands never directly inject attitude, velocity or arbitrary torque.

The trainer's surface masses contribute point-mass and rectangular-plate intrinsic inertia. Its current diagonal approximation is approximately `Ix/Iy/Iz = 0.077/0.105/0.177 kg·m²`, consistent in order of magnitude with a 1.7 m, roughly 1.4 kg airframe. Earlier massless wings left `Ix` near `0.004 kg·m²`; combined with whole-wing control effectiveness this caused the excessive rotation corrected in the current model.

Pilot commands target conservative 15° aileron, 18° elevator and 22° rudder limits. Their effective aerodynamic incidence factors are 0.09, 0.35 and 0.30 respectively. The aileron factor was increased from 0.06 after measured response showed excess latency; throws, inertia, full local airflow and natural rate damping remain unchanged. Degrees cross the schema/UI boundary once and become radians in Rust. Servos move actual deflection toward commanded deflection at a default 180°/s rather than teleporting. The trainer has 0.75° elevator trim at the documented 16 m/s reference condition. At roughly 16–25 m/s the current browser profile reaches a 30° bank in about 1.3 seconds under a sustained keyboard command while remaining rate-bounded.

An educational neutral-point estimate area/lift-slope weights the main wing and a 75%-effective tail. Static margin is `(x_CG - x_neutral)/MAC` in body +X-forward coordinates. BUILD labels 5–30% as stable guidance; this is not a certification calculation.

The simplified runway contact clamps penetration, damps forward/lateral motion lightly, and constrains roll/yaw while rolling. Once aerodynamic force creates upward velocity, the aircraft leaves the plane naturally. Low-impact contact settles; detailed wheels, suspension, tire steering and terrain collision shapes are not modelled.

## Procedural terrain

The default Alpine Range is a deterministic 6.4 km square heightfield with seed `1337`. Rust combines five-octave value-noise fBm, a ridged-noise term, broad valley shaping, three Gaussian pass cuts, and three landmark peaks. The runway footprint is analytically blended to the zero-metre datum; a lake basin is held below the 5 m water level. Elevation is bounded to -20…1250 m.

Rust owns `TerrainDefinition::elevation_m(North, East)`. Physics queries its NED ground coordinate `Down = -elevation`; Three.js samples the same exported WASM function for every indexed mesh vertex. Terrain normals use centred height gradients and contact removes velocity into the local upward normal. This is an efficient heightfield/contact-point approximation, not triangle-mesh landing-gear collision.

## VTOL composition

QuadPlane Explorer combines four fixed upward lift units, one forward unit and the same aerodynamic surfaces. All rotor, wing, drag and gravity forces are evaluated continuously. Transition command and actual transition are rate-limited. Forward propulsion grows with transition; vertical support is reduced only as both transition and measured airspeed indicate that wing support is available.

Tiltrotor Research VTOL uses four continuously rotating propulsion units. `0` is body `-Z` hover thrust and `1` is body `+X` cruise thrust. At tilt angle `θ`, direction is `(sin θ, 0, -cos θ)`; nacelles slew at 30°/s. Multicopter differentials fade as tilt approaches cruise while aerodynamic control surfaces remain active. This is a research/education transition model, not a validated tiltrotor control law.

VTOL lift units are longitudinally centered on the CG of the non-hover components, eliminating equal-thrust hover pitch moment. Fixed-wing trim fades in with actual transition. The common rate/attitude controller effort drives rotor differential and aerodynamic surfaces; rotor authority fades continuously for the tiltrotor while surface authority emerges naturally with dynamic pressure. Telemetry separates propulsion and aerodynamic pitch moments, wing support fraction, vertical thrust reserve and actual transition state. QuadPlane lift support decreases only when actual airspeed indicates growing wing effectiveness; no slider threshold changes the force model.

## Wind and Alpine airflow

`WindField::sample(position_ned, simulation_time, terrain)` is authoritative for every body and aerodynamic-surface relative-air calculation. Aviation direction is FROM: a 270° wind travels toward +East, while a 0° wind travels toward South (`-North`). Air velocity remains `vehicle velocity - local wind`, so headwind/tailwind airspeed differences emerge directly.

Gusts combine three bounded low-frequency sinusoids along the base-wind direction. Turbulence combines deterministic spatial/temporal waves on all three axes. These fields are continuous and reproducible, not independent frame noise or a certified Dryden/von Kármán spectrum.

Alpine terrain flow uses the authoritative height gradient. Horizontal wind dotted with the gradient produces bounded windward updraft, decaying exponentially with height above terrain; downslope flow produces weaker sink and a deterministic crosswind rotor component. The SOARING preset adds five deterministic Gaussian thermals with smooth radial and vertical falloff, all away from the runway. Ridge/lee flow and thermals alter the air-mass velocity—not aircraft lift directly—so all effects pass through the same AoA/lift/drag model. This is an educational orographic approximation, not CFD.

## Approximated

Motor/propeller performance is a bounded static estimate scaled by density and battery voltage. Controller tuning is specific to this educational model. Body drag uses a lumped coefficient. Each surface uses one force point with no spanwise flow, downwash, propwash, ground effect or wake interaction. Inertia omits component intrinsic tensors except the central body. Stall is deliberately smooth and educational, not an airfoil lookup, blade-element analysis, CFD, or a flight-test-derived model.

## Not yet modelled

Blade-element propellers, thermal/electrochemical battery behaviour, ground effect, gyroscopic rotor effects, detailed fuselage aerodynamics, compressibility, wake interaction, control-surface hinge dynamics, collision shapes/raycasting, damage/fracture, and atmospheric lapse layers. Results are qualitative engineering estimates, not design certification.
