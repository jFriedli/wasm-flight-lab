# WASM Flight Lab

**Design it. Fly it. Break it. Understand it.**

An open-source aircraft-design and flight-dynamics lab whose standard simulator runs entirely in the browser. Rust/WebAssembly owns authoritative physics; TypeScript and Three.js render it. There is no API, database, simulation server, or runtime container.

## Current vertical slice

The current build stabilizes the physical Fixed Wing Trainer, adds a deterministic Alpine flight world, and introduces experimental QuadPlane and tiltrotor composition. Surface-local rigid-body airflow now supplies natural rate damping; seeded Rust terrain drives both rendering and collision. Frame, propulsion, battery, payload, aerodynamic, servo and VTOL parameters remain physical runtime inputs—not display-only values. See [STATUS.md](STATUS.md) for exact scope.

## Develop

Prerequisites: stable Rust, `wasm-pack`, Node 22+ and npm.

```sh
cd web
npm install
npm run dev
```

Run all local checks:

```sh
cargo fmt --check
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo test --workspace
cd web && npm run lint && npm run typecheck && npm test && npm run build
npx playwright install chromium
npm run test:e2e
```

Default desktop control follows a Mode-2 transmitter: W/S gradually changes persistent throttle, A/D commands yaw/rudder, and the pointer-locked virtual right stick controls roll and pitch. Mouse up is stick forward/nose down; mouse down is nose up. Click **ENABLE MOUSE FLIGHT** explicitly and press Escape to release. Arrow keys remain the keyboard-only roll/pitch fallback. Sensitivity and spring return are adjustable; all sources feed the same normalized, shaped controller input. ACRO and ANGLE apply to multicopters; the trainer uses manual aerodynamic surfaces.

The Gamepad panel supports ordinary pads and USB RC/FPV radios exposed by the browser Gamepad API. Map axes, capture centres and travel, then set inversion, deadzone and expo. Validated values stay in local browser storage. Disconnect zeros attitude commands and gently reduces throttle.

CHASE, aircraft-mounted FPV and mouse-orbit FREE cameras are available from the viewport toolbar. Controller P/I/D values take effect live; the bounded graph compares target rate, measured rate and output.

BUILD opens the workshop without reloading. Select a quad, Fixed Wing Trainer, QuadPlane Explorer, or Tiltrotor Research VTOL; edit bounded SI values, inspect the model and magenta CG marker, then choose TEST FLIGHT. The trainer exposes surface mass, trim, servo rate and aerodynamic parameters. VTOL motors expose role, placement and tilt rate. Exported `.flightlab.json` files remain human-readable, bounded and validated again in Rust.

ALPINE RANGE is the default environment; TEST RANGE remains available for debugging. The 6.4 km seeded world includes a flat 700 m runway, valley, ridges, three passes, lake, bridge, huts, hangar, peak antenna and sparse instanced trees. The map, NED heading, coordinates and home bearing are generated locally. T/G or the transition slider commands VTOL hover/cruise progression.

## Static deployment and base paths

`npm run build` emits only static files in `web/dist`, including the `.wasm`. Vite's base is controlled by `FLIGHT_LAB_BASE`:

```sh
FLIGHT_LAB_BASE=/wasm-flight-lab/ npm run build  # standalone Pages
FLIGHT_LAB_BASE=/labs/flight/ npm run build      # future portfolio path
```

The Pages workflow deploys the first form automatically. No root-relative production asset URL is emitted.

For future `jFriedli.github.io` integration, the preferred approach is to download this repository's versioned Pages/build artifact and copy its contents into `labs/flight/` during the website build. This keeps histories and toolchains independent. A git submodule is an alternative: it pins source precisely, but adds clone/update friction and couples the parent build to this repository. Neither requires merging this source tree, and this task does not modify the website repository.

Educational model only: this is not certified engineering or flight-safety software. MIT licensed; current visuals are procedural and have no third-party asset licenses.
