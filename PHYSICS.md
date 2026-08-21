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

## Approximated

Motors respond instantly to command; thrust is a configured static maximum scaled by command, effectiveness, and density ratio. Body drag uses a lumped coefficient. Wings use one force point and no spanwise flow. Inertia is diagonal. Stall is deliberately smooth and educational, not an airfoil lookup or CFD solution.

## Not yet modelled

Motor spool dynamics, blade-element propellers, battery sag, ground effect, gyroscopic rotor effects, detailed fuselage aerodynamics, control surfaces, compressibility, wake interaction, collision shapes/raycasting, damage/fracture, and atmospheric lapse layers. Results are qualitative engineering estimates, not design certification.

