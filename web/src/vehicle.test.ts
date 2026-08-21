import { describe, expect, it } from "vitest";
import {
  loadDesigns,
  parseVehicle,
  safeFilename,
  validateVehicle,
  type VehicleDefinition,
} from "./vehicle";
const valid = {
  schemaVersion: 1,
  name: "Test",
  preset: "custom",
  frame: { armLengthM: 0.2, bodyMassKg: 0.4, bodyDimensionsM: [0.3, 0.2, 0.1] },
  motors: Array.from({ length: 4 }, (_, i) => ({
    positionM: [i < 2 ? 0.2 : -0.2, i % 3 ? -0.2 : 0.2, 0],
    directionBody: [0, 0, -1],
    baseMaxThrustN: 8,
    maxPowerW: 200,
    massKg: 0.04,
    spin: i % 2 ? 1 : -1,
    reactionTorqueNm: 0.1,
    spinUpTimeS: 0.08,
    spinDownTimeS: 0.1,
    propeller: {
      diameterM: 0.25,
      pitchM: 0.11,
      bladeCount: 2,
      efficiency: 0.7,
      massKg: 0.01,
    },
  })),
  battery: {
    cells: 4,
    capacityMah: 3000,
    massKg: 0.3,
    positionM: [0, 0, 0],
    internalResistanceOhm: 0.04,
    maxDischargeC: 30,
  },
  payloads: [],
} as VehicleDefinition;
describe("vehicle files", () => {
  it("accepts bounded versioned definitions", () =>
    expect(validateVehicle(valid)).toBe(true));
  it("rejects NaN and component bombs", () => {
    const broken = structuredClone(valid);
    broken.motors[0].massKg = Number.NaN;
    expect(validateVehicle(broken)).toBe(false);
    const bomb = structuredClone(valid);
    bomb.payloads = Array.from({ length: 33 }, () => ({
      name: "x",
      massKg: 1,
      positionM: [0, 0, 0],
    }));
    expect(validateVehicle(bomb)).toBe(false);
  });
  it("enforces input size", () =>
    expect(() => parseVehicle(" ".repeat(256001))).toThrow(/256 KB/));
  it("filters corrupt persistence", () =>
    expect(
      loadDesigns({ getItem: () => JSON.stringify([valid, { bad: true }]) }),
    ).toHaveLength(1));
  it("sanitizes export names", () =>
    expect(safeFilename("../ My <Quad> ")).toBe("My-Quad.flightlab.json"));
});
