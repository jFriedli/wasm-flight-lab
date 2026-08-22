import { describe, expect, it } from "vitest";
import { clamp, metrics } from "./model";
describe("engineering helpers", () => {
  it("clamps untrusted values", () => expect(clamp(Number.NaN, 0, 1)).toBe(0));
  it("computes hover metrics", () =>
    expect(metrics(1).hoverThrottle).toBeCloseTo(0.306));
});
