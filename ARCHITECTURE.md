# Architecture

```text
keyboard/gamepad + UI             Three.js renderer (~display Hz)
        │                                  ▲
        ▼                                  │ compact state snapshot
TypeScript fixed-clock shaping ─► WASM boundary
                                  │ 250 Hz fixed steps
                                  ▼
                       attitude/rate targets → PID → X mixer
                                  ↓
                       components → forces → rigid body
```

`flight-core` is browser-independent and tested natively. `flight-wasm` is deliberately thin. Render time never determines physics time: the frontend accumulates elapsed time, caps pathological gaps, and executes 4 ms steps. A dedicated Worker and packed numeric state buffer are planned once sensors and analysis increase bridge traffic; the current small state snapshot keeps the initial implementation inspectable and requires neither SharedArrayBuffer nor cross-origin isolation.

The core owns the versioned `VehicleDefinition`, runtime vehicle, environment, forces, battery state, Acro/Angle mode, PID state and motor mixing. Serialized definitions cross the WASM boundary only when a design changes. Compact runtime state crosses once per rendered frame. The browser supplies finite normalized commands, never motor values or imposed motion. Keyboard slew runs once per fixed simulation step, so render cadence cannot change commands. Gamepad mappings are validated before use and persistence.

Angle mode creates bounded roll/pitch targets and obtains current orientation from the authoritative quaternion before its attitude outer loop requests body rates. Yaw remains rate-commanded. Acro bypasses the outer loop and never auto-levels. One compact controller snapshot crosses into JavaScript per rendered frame; the tuning graph retains 240 samples rather than unbounded history.

`frames.ts` is the single render boundary for NED vectors, NED positions, body vectors and body-to-NED attitudes. The world and model basis matrices are proper rotations, so orientation uses matrix composition rather than quaternion component shuffling. All force arrows use authoritative NED vectors; motor arrow origins use authoritative body positions.

BUILD keeps a mutable, serializable definition separate from transient `Simulator` state. Each valid edit is revalidated and configured in Rust, while preview transforms update locally. TEST FLIGHT resets runtime state from that exact definition. Returning to BUILD retains it. Designs use bounded, versioned JSON in localStorage; imports are size/range/count checked in TypeScript and parsed again by Rust with unknown fields denied. Imported text is data only.

Chase, fixed-mount FPV and free-orbit cameras consume render state and never feed physics. The FPV transform can later come from a BUILD camera component. Future sensors have independent rate schedulers.
