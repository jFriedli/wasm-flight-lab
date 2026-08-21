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

The core owns vehicle components, environment, forces, state, Acro/Angle mode, PID state and motor mixing. The browser supplies finite normalized commands, never motor values or imposed motion. Keyboard slew runs once per fixed simulation step, so render cadence cannot change commands. Gamepad mappings are validated before use and persistence.

Angle mode creates bounded roll/pitch targets and obtains current orientation from the authoritative quaternion before its attitude outer loop requests body rates. Yaw remains rate-commanded. Acro bypasses the outer loop and never auto-levels. One compact controller snapshot crosses into JavaScript per rendered frame; the tuning graph retains 240 samples rather than unbounded history.

Chase, fixed-mount FPV and free-orbit cameras consume render state and never feed physics. The FPV transform can later come from a BUILD camera component. Future sensors have independent rate schedulers. Designs will use a versioned, range-validated JSON schema in IndexedDB; imported text is data only.
