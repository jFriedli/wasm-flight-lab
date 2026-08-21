import { clamp } from "./model";

export type AxisName = "roll" | "pitch" | "yaw" | "throttle";
export interface NormalizedControlInput {
  throttle: number;
  roll: number;
  pitch: number;
  yaw: number;
}
export interface VirtualStick {
  roll: number;
  pitch: number;
}
export interface AxisCalibration {
  source: number;
  min: number;
  center: number;
  max: number;
  invert: boolean;
  deadzone: number;
  expo: number;
}
export interface GamepadCalibration {
  version: 1;
  axes: Record<AxisName, AxisCalibration>;
}
export interface ControlProfile {
  name: "BEGINNER" | "SPORT" | "CUSTOM";
  rates: [number, number, number];
  maxAngle: number;
  expo: number;
  attack: number;
  release: number;
  throttleSlew: number;
}

export const PROFILES: Record<"BEGINNER" | "SPORT", ControlProfile> = {
  BEGINNER: {
    name: "BEGINNER",
    rates: [130, 130, 100],
    maxAngle: 30,
    expo: 0.55,
    attack: 1.6,
    release: 3.2,
    throttleSlew: 0.16,
  },
  SPORT: {
    name: "SPORT",
    rates: [280, 280, 200],
    maxAngle: 45,
    expo: 0.25,
    attack: 4.5,
    release: 6,
    throttleSlew: 0.28,
  },
};
export const DEFAULT_CALIBRATION: GamepadCalibration = {
  version: 1,
  axes: {
    roll: {
      source: 0,
      min: -1,
      center: 0,
      max: 1,
      invert: false,
      deadzone: 0.06,
      expo: 0.35,
    },
    pitch: {
      source: 1,
      min: -1,
      center: 0,
      max: 1,
      invert: true,
      deadzone: 0.06,
      expo: 0.35,
    },
    yaw: {
      source: 2,
      min: -1,
      center: 0,
      max: 1,
      invert: false,
      deadzone: 0.06,
      expo: 0.3,
    },
    throttle: {
      source: 3,
      min: -1,
      center: 0,
      max: 1,
      invert: true,
      deadzone: 0.03,
      expo: 0,
    },
  },
};

export function shape(value: number, deadzone: number, expo: number) {
  const raw = clamp(value, -1, 1);
  const dz = clamp(deadzone, 0, 0.4);
  if (Math.abs(raw) <= dz) return 0;
  const normalized = (Math.sign(raw) * (Math.abs(raw) - dz)) / (1 - dz);
  const e = clamp(expo, 0, 1);
  return (1 - e) * normalized + e * normalized ** 3;
}
export function slew(
  current: number,
  target: number,
  attack: number,
  release: number,
  dt: number,
) {
  const rate = Math.abs(target) > Math.abs(current) ? attack : release;
  const delta = Math.max(0, rate) * clamp(dt, 0, 0.1);
  return current + clamp(target - current, -delta, delta);
}
export function springVirtualStick(
  stick: VirtualStick,
  returnSpeed: number,
  dt: number,
): VirtualStick {
  return {
    roll: slew(stick.roll, 0, returnSpeed, returnSpeed, dt),
    pitch: slew(stick.pitch, 0, returnSpeed, returnSpeed, dt),
  };
}
export function applyMouseDelta(
  stick: VirtualStick,
  movementX: number,
  movementY: number,
  sensitivity: number,
): VirtualStick {
  const safe = Number.isFinite(sensitivity)
    ? clamp(sensitivity, 0.0001, 0.02)
    : 0.0025;
  return {
    roll: clamp(
      stick.roll + (Number.isFinite(movementX) ? movementX : 0) * safe,
      -1,
      1,
    ),
    pitch: clamp(
      stick.pitch + (Number.isFinite(movementY) ? movementY : 0) * safe,
      -1,
      1,
    ),
  };
}
export function normalizeAxis(
  raw: number,
  c: AxisCalibration,
  throttle = false,
) {
  if (!Number.isFinite(raw)) return 0;
  const safe = validateAxis(c);
  let value: number;
  if (raw >= safe.center)
    value = (raw - safe.center) / Math.max(0.001, safe.max - safe.center);
  else value = (raw - safe.center) / Math.max(0.001, safe.center - safe.min);
  if (safe.invert) value = -value;
  const shaped = shape(value, safe.deadzone, safe.expo);
  return throttle ? clamp((shaped + 1) / 2, 0, 1) : shaped;
}
export function validateAxis(value: Partial<AxisCalibration>): AxisCalibration {
  let min = clamp(Number(value.min), -2, 0);
  let max = clamp(Number(value.max), 0, 2);
  if (max - min < 0.1) {
    min = -1;
    max = 1;
  }
  return {
    source: Math.round(clamp(Number(value.source), 0, 31)),
    min,
    center: clamp(Number(value.center), min + 0.001, max - 0.001),
    max,
    invert: Boolean(value.invert),
    deadzone: clamp(Number(value.deadzone), 0, 0.3),
    expo: clamp(Number(value.expo), 0, 1),
  };
}
export function loadCalibration(
  storage: Pick<Storage, "getItem">,
): GamepadCalibration {
  try {
    const parsed = JSON.parse(
      storage.getItem("flightlab.gamepad.v1") ?? "null",
    ) as Partial<GamepadCalibration> | null;
    if (!parsed || parsed.version !== 1 || !parsed.axes)
      return structuredClone(DEFAULT_CALIBRATION);
    return {
      version: 1,
      axes: {
        roll: validateAxis(parsed.axes.roll),
        pitch: validateAxis(parsed.axes.pitch),
        yaw: validateAxis(parsed.axes.yaw),
        throttle: validateAxis(parsed.axes.throttle),
      },
    };
  } catch {
    return structuredClone(DEFAULT_CALIBRATION);
  }
}
export function saveCalibration(
  storage: Pick<Storage, "setItem">,
  value: GamepadCalibration,
) {
  const safe: GamepadCalibration = {
    version: 1,
    axes: {
      roll: validateAxis(value.axes.roll),
      pitch: validateAxis(value.axes.pitch),
      yaw: validateAxis(value.axes.yaw),
      throttle: validateAxis(value.axes.throttle),
    },
  };
  storage.setItem("flightlab.gamepad.v1", JSON.stringify(safe));
}
export function gamepadCommands(
  gamepad: Pick<Gamepad, "axes">,
  cal: GamepadCalibration,
) {
  const raw = (axis: AxisCalibration) =>
    gamepad.axes[axis.source] ?? Number.NaN;
  return {
    roll: normalizeAxis(raw(cal.axes.roll), cal.axes.roll),
    pitch: normalizeAxis(raw(cal.axes.pitch), cal.axes.pitch),
    yaw: normalizeAxis(raw(cal.axes.yaw), cal.axes.yaw),
    throttle: normalizeAxis(raw(cal.axes.throttle), cal.axes.throttle, true),
  };
}
