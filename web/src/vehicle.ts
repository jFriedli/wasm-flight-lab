import type { Vec3 } from "./model";

export interface PropellerDefinition {
  diameterM: number;
  pitchM: number;
  bladeCount: number;
  efficiency: number;
  massKg: number;
}
export interface MotorDefinition {
  positionM: Vec3;
  directionBody: Vec3;
  baseMaxThrustN: number;
  maxPowerW: number;
  massKg: number;
  spin: number;
  reactionTorqueNm: number;
  spinUpTimeS: number;
  spinDownTimeS: number;
  propeller: PropellerDefinition;
}
export interface BatteryDefinition {
  cells: number;
  capacityMah: number;
  massKg: number;
  positionM: Vec3;
  internalResistanceOhm: number;
  maxDischargeC: number;
}
export interface PayloadDefinition {
  name: string;
  massKg: number;
  positionM: Vec3;
}
export interface VehicleDefinition {
  schemaVersion: 1;
  name: string;
  preset: string;
  frame: { armLengthM: number; bodyMassKg: number; bodyDimensionsM: Vec3 };
  motors: MotorDefinition[];
  battery: BatteryDefinition;
  payloads: PayloadDefinition[];
}
export interface EngineeringMetrics {
  totalMassKg: number;
  centerOfMassM: Vec3;
  inertiaKgM2: Vec3;
  maxThrustN: number;
  thrustToWeight: number;
  hoverThrottle: number;
  maxPowerW: number;
  batteryEnergyWh: number;
  hoverCurrentA: number;
  hoverFlightTimeMin: number;
  hoverMotorOutputs: [number, number, number, number];
  warnings: string[];
}
export const STORAGE_KEY = "flightlab.vehicles.v1",
  MAX_FILE_BYTES = 256_000;

const finite = (value: unknown, min: number, max: number) =>
  typeof value === "number" &&
  Number.isFinite(value) &&
  value >= min &&
  value <= max;
const vec = (value: unknown, max = 10): value is Vec3 =>
  Array.isArray(value) &&
  value.length === 3 &&
  value.every((v) => finite(v, -max, max));
export function validateVehicle(value: unknown): value is VehicleDefinition {
  if (!value || typeof value !== "object") return false;
  const v = value as VehicleDefinition;
  if (
    v.schemaVersion !== 1 ||
    typeof v.name !== "string" ||
    v.name.trim().length < 1 ||
    v.name.length > 80 ||
    !v.frame ||
    !v.battery
  )
    return false;
  if (
    !finite(v.frame.armLengthM, 0.03, 2) ||
    !finite(v.frame.bodyMassKg, 0.02, 100) ||
    !vec(v.frame.bodyDimensionsM, 10)
  )
    return false;
  if (
    !Array.isArray(v.motors) ||
    v.motors.length !== 4 ||
    !Array.isArray(v.payloads) ||
    v.payloads.length > 32
  )
    return false;
  if (
    !finite(v.battery.cells, 1, 16) ||
    !finite(v.battery.capacityMah, 100, 100000) ||
    !finite(v.battery.massKg, 0.02, 100) ||
    !vec(v.battery.positionM) ||
    !finite(v.battery.internalResistanceOhm, 0, 2) ||
    !finite(v.battery.maxDischargeC, 1, 200)
  )
    return false;
  return (
    v.motors.every(
      (m) =>
        m &&
        vec(m.positionM) &&
        vec(m.directionBody) &&
        finite(m.baseMaxThrustN, 0, 10000) &&
        finite(m.maxPowerW, 1, 100000) &&
        finite(m.massKg, 0.001, 10) &&
        finite(m.reactionTorqueNm, 0, 100) &&
        [-1, 1].includes(m.spin) &&
        finite(m.spinUpTimeS, 0.005, 5) &&
        finite(m.spinDownTimeS, 0.005, 5) &&
        m.propeller &&
        finite(m.propeller.diameterM, 0.02, 2) &&
        finite(m.propeller.pitchM, 0.01, 1) &&
        finite(m.propeller.bladeCount, 1, 8) &&
        Number.isInteger(m.propeller.bladeCount) &&
        finite(m.propeller.efficiency, 0.1, 1) &&
        finite(m.propeller.massKg, 0.001, 5),
    ) &&
    v.payloads.every(
      (p) =>
        p &&
        typeof p.name === "string" &&
        p.name.trim().length > 0 &&
        p.name.length <= 60 &&
        finite(p.massKg, 0.001, 1000) &&
        vec(p.positionM),
    )
  );
}
export function parseVehicle(text: string) {
  if (new TextEncoder().encode(text).length > MAX_FILE_BYTES)
    throw new Error("File exceeds 256 KB");
  let value: unknown;
  try {
    value = JSON.parse(text);
  } catch {
    throw new Error("Invalid JSON");
  }
  if (!validateVehicle(value))
    throw new Error("Vehicle schema or values are invalid");
  return value;
}
export function loadDesigns(storage: Pick<Storage, "getItem">) {
  try {
    const value = JSON.parse(storage.getItem(STORAGE_KEY) ?? "[]") as unknown;
    return Array.isArray(value)
      ? value.filter(validateVehicle).slice(0, 32)
      : [];
  } catch {
    return [];
  }
}
export function saveDesigns(
  storage: Pick<Storage, "setItem">,
  designs: VehicleDefinition[],
) {
  storage.setItem(
    STORAGE_KEY,
    JSON.stringify(designs.filter(validateVehicle).slice(0, 32)),
  );
}
export function safeFilename(name: string) {
  const clean = name
    .normalize("NFKD")
    .replace(/[^a-zA-Z0-9_-]+/g, "-")
    .replace(/^-+|-+$/g, "")
    .slice(0, 60);
  return `${clean || "aircraft"}.flightlab.json`;
}
