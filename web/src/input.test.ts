import { describe, expect, it } from "vitest";
import {
  applyMouseDelta,
  DEFAULT_CALIBRATION,
  gamepadCommands,
  loadCalibration,
  normalizeAxis,
  shape,
  slew,
  springVirtualStick,
} from "./input";
describe("input pipeline", () => {
  it("expo preserves endpoints and is monotonic", () => {
    expect(shape(-1, 0.05, 0.5)).toBe(-1);
    expect(shape(0, 0.05, 0.5)).toBe(0);
    expect(shape(1, 0.05, 0.5)).toBe(1);
    const x = Array.from({ length: 201 }, (_, i) =>
      shape(i / 100 - 1, 0.05, 0.5),
    );
    expect(x.every((v, i) => i === 0 || v >= x[i - 1])).toBe(true);
  });
  it("slew depends on elapsed time", () => {
    const once = slew(0, 1, 2, 4, 0.04);
    let many = 0;
    for (let i = 0; i < 10; i++) many = slew(many, 1, 2, 4, 0.004);
    expect(many).toBeCloseTo(once, 12);
  });
  it("normalizes asymmetric calibrated axes", () =>
    expect(
      normalizeAxis(0.6, {
        source: 0,
        min: -0.8,
        center: 0.1,
        max: 1,
        invert: false,
        deadzone: 0,
        expo: 0,
      }),
    ).toBeCloseTo(0.5556, 3));
  it("maps every calibrated center to exact neutral", () => {
    for (const axis of ["roll", "pitch", "yaw"] as const)
      expect(
        normalizeAxis(
          DEFAULT_CALIBRATION.axes[axis].center,
          DEFAULT_CALIBRATION.axes[axis],
        ),
      ).toBe(0);
  });
  it("fails safe on invalid gamepad values", () => {
    const c = gamepadCommands({ axes: [Number.NaN] }, DEFAULT_CALIBRATION);
    expect(c.roll).toBe(0);
    expect(c.throttle).toBe(0);
  });
  it("maps mouse as a bounded Mode-2 right stick", () => {
    expect(applyMouseDelta({ roll: 0, pitch: 0 }, 100, -100, 0.003)).toEqual({
      roll: 0.3,
      pitch: -0.3,
    });
    expect(
      applyMouseDelta({ roll: 0.9, pitch: -0.9 }, 100, -100, 0.01),
    ).toEqual({ roll: 1, pitch: -1 });
  });
  it("springs virtual stick toward neutral with fixed time", () => {
    expect(springVirtualStick({ roll: 0.5, pitch: -0.5 }, 2, 0.1)).toEqual({
      roll: 0.3,
      pitch: -0.3,
    });
    expect(springVirtualStick({ roll: 0.5, pitch: -0.5 }, 2, 0.05)).toEqual({
      roll: 0.4,
      pitch: -0.4,
    });
  });
  it("recenters nose-up and nose-down displacement symmetrically to exact zero", () => {
    for (const pitch of [-1, 1]) {
      let stick = { roll: 0, pitch };
      for (let i = 0; i < 250; i++)
        stick = springVirtualStick(stick, 1.4, 0.004);
      expect(Object.is(stick.pitch, -0)).toBe(false);
      expect(stick.pitch).toBe(0);
    }
  });
  it("rejects malformed persisted data", () => {
    const c = loadCalibration({ getItem: () => "{broken" });
    expect(c.version).toBe(1);
    expect(c.axes.roll.source).toBe(0);
  });
});
