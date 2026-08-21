import { describe, expect, it } from "vitest";
import * as THREE from "three";
import {
  BODY_FROM_MODEL,
  WORLD_FROM_NED,
  attitudeBodyToNedToThree,
  bodyVectorToModel,
  determinant3,
  nedVectorToThree,
} from "./frames";

const tuple = (v: THREE.Vector3) =>
  v.toArray().map((x) => Math.round(x * 1e12) / 1e12);

describe("coordinate frames", () => {
  it("maps NED basis vectors into the documented Three basis", () => {
    expect(tuple(nedVectorToThree([1, 0, 0]))).toEqual([0, 0, -1]);
    expect(tuple(nedVectorToThree([0, 1, 0]))).toEqual([1, 0, 0]);
    expect(tuple(nedVectorToThree([0, 0, 1]))).toEqual([0, -1, 0]);
    expect(tuple(nedVectorToThree([0, 0, -1]))).toEqual([0, 1, 0]);
  });

  it("uses only right-handed basis rotations", () => {
    expect(determinant3(WORLD_FROM_NED)).toBeCloseTo(1, 12);
    expect(determinant3(BODY_FROM_MODEL)).toBeCloseTo(1, 12);
  });

  it("keeps rendered local thrust aligned with transformed NED thrust", () => {
    const bodyToNed = new THREE.Quaternion().setFromEuler(
      new THREE.Euler(
        THREE.MathUtils.degToRad(30),
        0,
        THREE.MathUtils.degToRad(25),
        "ZYX",
      ),
    );
    const bodyThrust = new THREE.Vector3(0, 0, -1);
    const nedThrust = bodyThrust.clone().applyQuaternion(bodyToNed);
    const expected = nedVectorToThree(
      nedThrust.toArray() as [number, number, number],
    ).normalize();
    const rendered = bodyVectorToModel([0, 0, -1])
      .applyQuaternion(attitudeBodyToNedToThree(bodyToNed.toArray()))
      .normalize();
    expect(rendered.angleTo(expected)).toBeLessThan(1e-10);
  });
});
