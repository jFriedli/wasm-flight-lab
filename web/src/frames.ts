import * as THREE from "three";
import type { Vec3 } from "./model";

/**
 * Proper rotation from NED world coordinates to Three world coordinates:
 * North -> -Z, East -> +X, Down -> -Y. Determinant is +1.
 */
export const WORLD_FROM_NED = new THREE.Matrix4().set(
  0,
  1,
  0,
  0,
  0,
  0,
  -1,
  0,
  -1,
  0,
  0,
  0,
  0,
  0,
  0,
  1,
);

/**
 * Proper rotation from Three aircraft-model local coordinates to body FRD.
 * Model +X is right, +Y is up, and -Z is forward.
 */
export const BODY_FROM_MODEL = new THREE.Matrix4().set(
  0,
  0,
  -1,
  0,
  1,
  0,
  0,
  0,
  0,
  -1,
  0,
  0,
  0,
  0,
  0,
  1,
);

export function nedVectorToThree(value: Vec3, target = new THREE.Vector3()) {
  return target.set(value[0], value[1], value[2]).applyMatrix4(WORLD_FROM_NED);
}

export const nedPositionToThree = nedVectorToThree;

export function bodyVectorToModel(value: Vec3, target = new THREE.Vector3()) {
  return target
    .set(value[0], value[1], value[2])
    .applyMatrix4(BODY_FROM_MODEL.clone().invert());
}

export function attitudeBodyToNedToThree(
  value: [number, number, number, number],
  target = new THREE.Quaternion(),
) {
  const bodyToNed = new THREE.Matrix4().makeRotationFromQuaternion(
    new THREE.Quaternion(value[0], value[1], value[2], value[3]),
  );
  // model -> body -> NED -> Three world; all three matrices are proper rotations.
  const modelToThree = WORLD_FROM_NED.clone()
    .multiply(bodyToNed)
    .multiply(BODY_FROM_MODEL);
  return target.setFromRotationMatrix(modelToThree).normalize();
}

export function determinant3(matrix: THREE.Matrix4) {
  return new THREE.Matrix3().setFromMatrix4(matrix).determinant();
}
