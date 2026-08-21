export type Vec3 = [number, number, number];
export interface SimState {
  time: number;
  position: Vec3;
  velocity: Vec3;
  attitude: [number, number, number, number];
  euler: Vec3;
  rates: Vec3;
  forces: {
    gravity: Vec3;
    thrust: Vec3;
    lift: Vec3;
    drag: Vec3;
    motors: Array<{ positionBody: Vec3; forceNed: Vec3 }>;
  };
  control: {
    target: Vec3;
    actual: Vec3;
    error: Vec3;
    output: Vec3;
    throttle: number;
    motors: [number, number, number, number];
    actualMotors: [number, number, number, number];
  };
  battery: { remainingMah: number; consumedWh: number; voltage: number; current: number };
  mass: number;
  cg: Vec3;
  inertia: Vec3;
}
export const clamp = (value: number, min: number, max: number) =>
  Math.min(max, Math.max(min, Number.isFinite(value) ? value : min));
export function metrics(mass: number, maxThrust = 32) {
  return {
    weight: mass * 9.80665,
    thrustToWeight: maxThrust / (mass * 9.80665),
    hoverThrottle: (mass * 9.80665) / maxThrust,
  };
}
