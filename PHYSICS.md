# Physics model

## Frames and units

All core values use SI units. The inertial frame is right-handed NED: +X north, +Y east, +Z down. The body frame is +X forward, +Y right, +Z down. Quaternions rotate body vectors into NED.

## Modelled

- Six-degree-of-freedom translational/rotational state with semi-implicit Euler integration at 250 Hz.
- Gravity, quadratic body drag, per-motor thrust, moment arm torque `τ = (r - CG) × F`, and rotor reaction torque.
- Component-derived mass and center of mass; diagonal inertia is currently supplied by each preset.
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

## Approximated

Motors respond instantly to command; thrust is a configured static maximum scaled by command, effectiveness, and density ratio. Controller tuning is therefore specific to this educational motor model. Body drag uses a lumped coefficient. Wings use one force point and no spanwise flow. Inertia is diagonal. Stall is deliberately smooth and educational, not an airfoil lookup or CFD solution.

## Not yet modelled

Motor spool dynamics, blade-element propellers, battery sag, ground effect, gyroscopic rotor effects, detailed fuselage aerodynamics, control surfaces, compressibility, wake interaction, collision shapes/raycasting, damage/fracture, and atmospheric lapse layers. Results are qualitative engineering estimates, not design certification.
