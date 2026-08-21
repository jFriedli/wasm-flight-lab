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
- Wing angle of attack `atan2(-Vz, Vx) + incidence`, dynamic pressure `q = ½ρV²`, linear lift slope with a smooth post-stall attenuation, parasite plus induced drag.
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

## Approximated

Motor/propeller performance is a bounded static estimate scaled by density and battery voltage. Controller tuning is specific to this educational model. Body drag uses a lumped coefficient. Wings use one force point and no spanwise flow. Inertia omits component intrinsic tensors except the central body. Stall is deliberately smooth and educational, not an airfoil lookup or CFD solution.

## Not yet modelled

Blade-element propellers, thermal/electrochemical battery behaviour, ground effect, gyroscopic rotor effects, detailed fuselage aerodynamics, control surfaces, compressibility, wake interaction, collision shapes/raycasting, damage/fracture, and atmospheric lapse layers. Results are qualitative engineering estimates, not design certification.
