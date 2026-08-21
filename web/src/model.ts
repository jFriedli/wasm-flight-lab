export type Vec3 = [number, number, number];
export interface SimState {
  time: number;
  vehicleClass: "Multicopter" | "FixedWing";
  position: Vec3;
  velocity: Vec3;
  airVelocity: Vec3;
  angleOfAttack: number;
  stalled: boolean;
  attitude: [number, number, number, number];
  euler: Vec3;
  rates: Vec3;
  forces: {
    gravity: Vec3;
    thrust: Vec3;
    lift: Vec3;
    drag: Vec3;
    motors: Array<{ positionBody: Vec3; forceNed: Vec3 }>;
    surfaces: Array<{
      name: string;
      positionBody: Vec3;
      liftNed: Vec3;
      dragNed: Vec3;
      angle: number;
      cl: number;
      cd: number;
      stalled: boolean;
    }>;
  };
  control: {
    target: Vec3;
    actual: Vec3;
    error: Vec3;
    output: Vec3;
    throttle: number;
    motors: number[];
    actualMotors: number[];
    sticks: Vec3;
  };
  battery: {
    remainingMah: number;
    consumedWh: number;
    voltage: number;
    current: number;
  };
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
