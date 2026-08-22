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

The core owns the versioned `VehicleDefinition`, vehicle class, runtime components, environment, wind field, forces, battery state and control allocation. Multicopters route normalized commands through Acro/Angle loops, rate PID and an X mixer. Fixed wings route the same normalized commands to aileron/elevator/rudder deflection and collective throttle to shared propulsion units; they never use the quad mixer. Serialized definitions cross WASM only when a design changes. Compact runtime/aerodynamic/atmospheric telemetry crosses once per rendered frame.

Keyboard+mouse and Gamepad are explicit input sources producing one `{throttle, roll, pitch, yaw}` structure. Pointer motion changes a bounded virtual stick; fixed-clock spring, expo and slew processing occur before the WASM call. Sources are selected rather than summed, preventing ambiguous mixed commands.

Angle mode creates bounded roll/pitch targets and obtains current orientation from the authoritative quaternion before its attitude outer loop requests body rates. Yaw remains rate-commanded. Acro bypasses the outer loop and never auto-levels. One compact controller snapshot crosses into JavaScript per rendered frame; the tuning graph retains 240 samples rather than unbounded history.

`frames.ts` is the single render boundary for NED vectors, NED positions, body vectors and body-to-NED attitudes. The world and model basis matrices are proper rotations, so orientation uses matrix composition rather than quaternion component shuffling. All force arrows use authoritative NED vectors; motor arrow origins use authoritative body positions.

BUILD keeps a mutable, serializable definition separate from transient `Simulator` state. Each valid edit is revalidated and configured in Rust, while preview transforms update locally. TEST FLIGHT resets runtime state from that exact definition. Returning to BUILD retains it. Designs use bounded, versioned JSON in localStorage; imports are size/range/count checked in TypeScript and parsed again by Rust with unknown fields denied. Imported text is data only.

Aerodynamic surfaces and propulsion are shared components. QuadPlane composes horizontal surfaces, one forward propulsor and four vertical propulsors. Tiltrotor uses the same force loop with rate-limited propulsion orientation. There is no hover/cruise physics switch: transition changes allocation and force direction while all forces remain active.

`TerrainDefinition` is authoritative Rust state. Its seeded height/normal queries feed 250 Hz contact and are exported through the thin WASM boundary. The renderer samples that exact function once to construct a 128×128 indexed mesh; landmarks and instanced trees are deterministic presentation layers. The minimap reuses the height sampler and compact per-frame NED position/yaw.

`WindField` samples deterministic base wind, coherent gust, turbulence, terrain-gradient ridge/lee flow and optional thermals from NED position and simulation time. The one combined vector feeds body drag and every surface's local `vCG + ω×r` airflow. The windsock and sparse debug arrows consume the same snapshot; neither owns a visual-only wind model.

Chase, fixed-mount FPV and free-orbit cameras consume render state and never feed physics. The FPV transform can later come from a BUILD camera component. Future sensors have independent rate schedulers.
