# Architecture

```text
browser input + UI                 Three.js renderer (~display Hz)
        │                                  ▲
        ▼                                  │ compact state snapshot
TypeScript accumulator ───────► WASM boundary
                                  │ 250 Hz fixed steps
                                  ▼
                       flight-core (authoritative state)
                       components → forces → rigid body
```

`flight-core` is browser-independent and tested natively. `flight-wasm` is deliberately thin. Render time never determines physics time: the frontend accumulates elapsed time, caps pathological gaps, and executes 4 ms steps. A dedicated Worker and packed numeric state buffer are planned once sensors and analysis increase bridge traffic; the current small state snapshot keeps the initial implementation inspectable and requires neither SharedArrayBuffer nor cross-origin isolation.

The core owns vehicle components, environment, forces, state and controllers. Rendering is a projection of core state and cannot authoritatively move the aircraft. Future sensors have independent rate schedulers; telemetry uses bounded ring buffers. Designs will use a versioned, range-validated JSON schema in IndexedDB. Imported text is data only: no dynamic evaluation or remote URL fetching.

