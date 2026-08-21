# WASM Flight Lab

**Design it. Fly it. Break it. Understand it.**

An open-source aircraft-design and flight-dynamics lab whose standard simulator runs entirely in the browser. Rust/WebAssembly owns authoritative physics; TypeScript and Three.js render it. There is no API, database, simulation server, or runtime container.

## Current vertical slice

The current M2 build includes a controllable component-based quad, 250 Hz fixed-step rigid-body integration, per-motor forces and torques, density-aware propulsion, a Rust rate/angle flight controller, saturation-aware X mixer, calibrated Gamepad input, three cameras, controller telemetry and live PID tuning. See [STATUS.md](STATUS.md) for exact scope.

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

Keyboard: W/S gradually changes collective, A/D commands roll, arrow up/down commands pitch, Q/E commands yaw, Space pauses and R resets. Keyboard attitude commands use simulation-clock slew and expo. ACRO provides direct rate control; ANGLE self-levels roll and pitch.

The Gamepad panel supports ordinary pads and USB RC/FPV radios exposed by the browser Gamepad API. Map axes, capture centres and travel, then set inversion, deadzone and expo. Validated values stay in local browser storage. Disconnect zeros attitude commands and gently reduces throttle.

CHASE, aircraft-mounted FPV and mouse-orbit FREE cameras are available from the viewport toolbar. Controller P/I/D values take effect live; the bounded graph compares target rate, measured rate and output.

## Static deployment and base paths

`npm run build` emits only static files in `web/dist`, including the `.wasm`. Vite's base is controlled by `FLIGHT_LAB_BASE`:

```sh
FLIGHT_LAB_BASE=/wasm-flight-lab/ npm run build  # standalone Pages
FLIGHT_LAB_BASE=/labs/flight/ npm run build      # future portfolio path
```

The Pages workflow deploys the first form automatically. No root-relative production asset URL is emitted.

For future `jFriedli.github.io` integration, the preferred approach is to download this repository's versioned Pages/build artifact and copy its contents into `labs/flight/` during the website build. This keeps histories and toolchains independent. A git submodule is an alternative: it pins source precisely, but adds clone/update friction and couples the parent build to this repository. Neither requires merging this source tree, and this task does not modify the website repository.

Educational model only: this is not certified engineering or flight-safety software. MIT licensed; current visuals are procedural and have no third-party asset licenses.
