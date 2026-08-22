import { describe, expect, it } from "vitest";
import * as THREE from "three";
import { createTerrain } from "./terrain";

describe("terrain render bridge", () => {
  it("uses the authoritative sampler for every rendered mesh vertex", () => {
    const sampler = {
      terrain_height: (north: number, east: number) =>
        north * 0.01 + east * 0.02,
    };
    const group = createTerrain(sampler, false, 4);
    const mesh = group.children[0] as THREE.Mesh<THREE.BufferGeometry>;
    const positions = mesh.geometry.getAttribute("position");
    for (let index = 0; index < positions.count; index++) {
      const east = positions.getX(index);
      const north = -positions.getZ(index);
      expect(positions.getY(index)).toBeCloseTo(
        sampler.terrain_height(north, east),
        5,
      );
    }
    expect(mesh.userData.terrainResolution).toBe(4);
  });
});
