export type Vec3 = [number, number, number];
export interface SimState {
  time: number;
  position: Vec3;
  velocity: Vec3;
  attitude: [number, number, number, number];
  euler: Vec3;
  rates: Vec3;
  forces: { thrust: Vec3; lift: Vec3; drag: Vec3 };
  control: {
    target: Vec3;
    actual: Vec3;
    error: Vec3;
    output: Vec3;
    throttle: number;
    motors: [number, number, number, number];
  };
  mass: number;
  cg: Vec3;
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
