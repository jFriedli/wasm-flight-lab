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

## Fixed-wing aerodynamics

The Fixed Wing Trainer is an educational 1.7 m-span, 0.48 m²-wing aircraft of approximately 1.3 kg. Its 4-cell/3300 mAh battery drives one 15 N, 520 W forward propulsor. The conventional tail has a 0.10 m² elevator and 0.065 m² rudder. It is not based on a commercial airframe.

Authoritative air velocity is `v_air,NED = v_vehicle,NED - v_wind,NED`. The body-to-NED quaternion inverse transforms it into body coordinates. Each surface also samples a bounded rotational contribution `0.1(ω × r)`; this and a speed-scaled angular damping approximation represent basic aerodynamic rate damping without a full distributed panel model.

For horizontal surfaces, angle of attack is `α = atan2(Vz, Vx) + incidence + control_effect`. Because body +Z is down, a nose above its flight path produces positive body `Vz` and positive α. Vertical surfaces use sideslip `β = atan2(Vy, Vx)`. AoA is derived from airflow, never vehicle pitch alone.

Dynamic pressure is `q = ½ρV²`. Before stall, `CL = CLα α`. At the configured critical magnitude `αs`, CL is continuous; beyond it, `CL = CLα αs sign(α)(αs/|α|)^1.2`, so lift physically falls instead of increasing indefinitely. Lift magnitude is `q S CL` and its direction is perpendicular to the local planar airflow. Near-zero airflow produces near-zero force.

Drag is opposite local relative velocity with `CD = CD0 + k CL² + 1.2 e²`, where `e = max(0,(|α|-αs)/αs)`. The final term supplies continuous post-stall pressure drag. Generic fuselage drag is reduced for fixed wings to avoid double-counting surface profile drag.

Every left/right wing, elevator and rudder has its own body position. Rust applies `τ = (r-CG) × (L+D)`. Ailerons use opposite incidence changes; elevator and rudder change their tail surface incidence. Consequently authority scales with dynamic pressure and is negligible while stationary. Control commands never directly inject attitude, velocity or arbitrary torque.

The simplified runway contact clamps penetration, damps forward/lateral motion lightly, and constrains roll/yaw while rolling. Once aerodynamic force creates upward velocity, the aircraft leaves the plane naturally. Low-impact contact settles; detailed wheels, suspension, tire steering and terrain collision shapes are not modelled.

## Approximated

Motor/propeller performance is a bounded static estimate scaled by density and battery voltage. Controller tuning is specific to this educational model. Body drag uses a lumped coefficient. Each surface uses one force point with no spanwise flow, downwash, propwash, ground effect or wake interaction. Inertia omits component intrinsic tensors except the central body. Stall is deliberately smooth and educational, not an airfoil lookup, blade-element analysis, CFD, or a flight-test-derived model.

## Not yet modelled

Blade-element propellers, thermal/electrochemical battery behaviour, ground effect, gyroscopic rotor effects, detailed fuselage aerodynamics, compressibility, wake interaction, control-surface hinge dynamics, collision shapes/raycasting, damage/fracture, and atmospheric lapse layers. Results are qualitative engineering estimates, not design certification.
