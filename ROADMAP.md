# Roadmap

- M2 complete: rate/angle PID loops, tuning telemetry, generic persisted Gamepad calibration and chase/FPV/free cameras.
- M3 complete: workshop, battery/propeller/payload editors, calculated inertia, local versioned save and validated import/export.
- M4 complete vertical slice: physical Fixed Wing Trainer, per-surface aerodynamics/control, stall, runway takeoff, fixed-wing BUILD metrics and Mode-2 mouse controls.
- M5: QuadPlane and continuously tilting tiltrotor using shared propulsion/aerodynamic components.
- M6: Alpine valley, deterministic gust/turbulence fields, ridge/valley wind zones and atmospheric presets.
- M7–M8: independently clocked IMU/GPS/barometer/airspeed, EKF lab, failures, richer raycast LIDAR, bounded graphs and scenario reports.
- Later: MAVLink file inspector; optional local user-run WebSocket bridge to ArduPilot SITL. Investigate ArduPilot-in-WASM separately.
- Optional ROS 2 mode: browser WebSocket to the user's own rosbridge for sensor publication and command input; never a standard-site dependency.
- A*/RRT/RRT* planning, obstacle avoidance, tailsitter, swarm simulation, advanced aerodynamic tables, urban/island scenes and moving-ship landings.
