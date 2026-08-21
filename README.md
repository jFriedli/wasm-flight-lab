# WASM Flight Lab

**Design it. Fly it. Break it. Understand it.**

An open-source aircraft-design and flight-dynamics lab whose standard simulator runs entirely in the browser. Rust/WebAssembly owns authoritative physics; TypeScript and Three.js render it. There is no API, database, simulation server, or runtime container.

## Current vertical slice

The v0.1 foundation includes a flyable component-based quad, 250 Hz fixed-step rigid-body integration, per-motor forces and torques, density-aware propulsion, drag, ground impact, a Rust PID primitive, a procedural test center, force visualization, keyboard control, and live engineering values. See [STATUS.md](STATUS.md) for exact scope.

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
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
cd web && npm run lint && npm run typecheck && npm test && npm run build
npx playwright install chromium
npm run test:e2e
```

Keyboard: W/S collective, A/D roll, arrow up/down pitch, R reset. Every control also has an accessible UI equivalent where applicable; Gamepad calibration is planned.

## Static deployment and base paths

`npm run build` emits only static files in `web/dist`, including the `.wasm`. Vite's base is controlled by `FLIGHT_LAB_BASE`:

```sh
FLIGHT_LAB_BASE=/wasm-flight-lab/ npm run build  # standalone Pages
FLIGHT_LAB_BASE=/labs/flight/ npm run build      # future portfolio path
```

The Pages workflow deploys the first form automatically. No root-relative production asset URL is emitted.

For future `jFriedli.github.io` integration, the preferred approach is to download this repository's versioned Pages/build artifact and copy its contents into `labs/flight/` during the website build. This keeps histories and toolchains independent. A git submodule is an alternative: it pins source precisely, but adds clone/update friction and couples the parent build to this repository. Neither requires merging this source tree, and this task does not modify the website repository.

Educational model only: this is not certified engineering or flight-safety software. MIT licensed; current visuals are procedural and have no third-party asset licenses.

