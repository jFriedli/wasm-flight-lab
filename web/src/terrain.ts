import * as THREE from "three";
import type { Vec3 } from "./model";

export const TERRAIN_SEED = 1337;
export const WORLD_SIZE_M = 6400;
export const LAKE = { north: 900, east: -520, level: 5 };

export interface TerrainSampler {
  terrain_height(northM: number, eastM: number): number;
}

export function createTerrain(
  sampler: TerrainSampler,
  alpine: boolean,
  resolution = 128,
) {
  const group = new THREE.Group();
  group.name = alpine ? "Alpine Range" : "Test Range";
  const vertices: number[] = [];
  const colors: number[] = [];
  const indices: number[] = [];
  const color = new THREE.Color();
  for (let northIndex = 0; northIndex <= resolution; northIndex++) {
    const north = -WORLD_SIZE_M / 2 + (northIndex / resolution) * WORLD_SIZE_M;
    for (let eastIndex = 0; eastIndex <= resolution; eastIndex++) {
      const east = -WORLD_SIZE_M / 2 + (eastIndex / resolution) * WORLD_SIZE_M;
      const elevation = sampler.terrain_height(north, east);
      vertices.push(east, elevation, -north);
      if (!alpine) color.set(0x567847);
      else if (elevation < 80) color.set(0x557943);
      else if (elevation < 420) color.set(0x3f633d);
      else if (elevation < 820) color.set(0x6d6b61);
      else color.set(0xd9e2e5);
      const variation =
        0.92 + (((northIndex * 37 + eastIndex * 17) % 13) / 13) * 0.13;
      colors.push(
        color.r * variation,
        color.g * variation,
        color.b * variation,
      );
    }
  }
  const row = resolution + 1;
  for (let north = 0; north < resolution; north++) {
    for (let east = 0; east < resolution; east++) {
      const a = north * row + east;
      indices.push(a, a + 1, a + row, a + 1, a + row + 1, a + row);
    }
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute(
    "position",
    new THREE.Float32BufferAttribute(vertices, 3),
  );
  geometry.setAttribute("color", new THREE.Float32BufferAttribute(colors, 3));
  geometry.setIndex(indices);
  geometry.computeVertexNormals();
  const mesh = new THREE.Mesh(
    geometry,
    new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.95 }),
  );
  mesh.receiveShadow = true;
  mesh.userData.terrainResolution = resolution;
  group.add(mesh);

  if (alpine) {
    const water = new THREE.Mesh(
      new THREE.CircleGeometry(340, 48),
      new THREE.MeshStandardMaterial({
        color: 0x438eaa,
        roughness: 0.25,
        metalness: 0.15,
        transparent: true,
        opacity: 0.86,
      }),
    );
    water.rotation.x = -Math.PI / 2;
    water.position.set(LAKE.east, LAKE.level, -LAKE.north);
    group.add(water);
    addLandmarks(group, sampler);
    addTrees(group, sampler);
  }
  return group;
}

function addLandmarks(group: THREE.Group, sampler: TerrainSampler) {
  const buildingMaterial = new THREE.MeshStandardMaterial({ color: 0xb8b0a0 });
  const roofMaterial = new THREE.MeshStandardMaterial({ color: 0x633c32 });
  const hangar = new THREE.Mesh(
    new THREE.BoxGeometry(45, 14, 28),
    buildingMaterial,
  );
  hangar.position.set(42, 7, 190);
  group.add(hangar);
  for (const [north, east] of [
    [-360, 85],
    [-390, 115],
    [-420, 72],
    [700, 120],
    [735, 145],
  ]) {
    const hut = new THREE.Mesh(
      new THREE.BoxGeometry(16, 9, 13),
      buildingMaterial,
    );
    hut.position.set(east, sampler.terrain_height(north, east) + 4.5, -north);
    const roof = new THREE.Mesh(new THREE.ConeGeometry(12, 5, 4), roofMaterial);
    roof.rotation.y = Math.PI / 4;
    roof.position.copy(hut.position).add(new THREE.Vector3(0, 7, 0));
    group.add(hut, roof);
  }
  const towerNorth = 1850,
    towerEast = 1650,
    towerBase = sampler.terrain_height(towerNorth, towerEast);
  const tower = new THREE.Mesh(
    new THREE.CylinderGeometry(1.5, 4, 75, 8),
    new THREE.MeshStandardMaterial({ color: 0xe15a45 }),
  );
  tower.position.set(towerEast, towerBase + 37.5, -towerNorth);
  group.add(tower);
  const bridge = new THREE.Mesh(
    new THREE.BoxGeometry(90, 4, 12),
    new THREE.MeshStandardMaterial({ color: 0x765b45 }),
  );
  bridge.position.set(LAKE.east + 250, LAKE.level + 9, -LAKE.north);
  group.add(bridge);
}

function addTrees(group: THREE.Group, sampler: TerrainSampler) {
  const geometry = new THREE.ConeGeometry(4, 18, 7);
  const material = new THREE.MeshStandardMaterial({ color: 0x244f35 });
  const positions: Array<[number, number, number]> = [];
  let random = TERRAIN_SEED;
  const next = () => {
    random = (random * 1664525 + 1013904223) >>> 0;
    return random / 0xffffffff;
  };
  for (let attempt = 0; attempt < 2400 && positions.length < 550; attempt++) {
    const north = (next() - 0.5) * WORLD_SIZE_M;
    const east = (next() - 0.5) * WORLD_SIZE_M;
    const elevation = sampler.terrain_height(north, east);
    const lakeDistance = Math.hypot(north - LAKE.north, east - LAKE.east);
    if (
      elevation < 15 ||
      elevation > 650 ||
      (Math.abs(north) < 720 && Math.abs(east) < 260) ||
      lakeDistance < 410
    )
      continue;
    positions.push([east, elevation + 9, -north]);
  }
  const trees = new THREE.InstancedMesh(geometry, material, positions.length);
  const transform = new THREE.Matrix4();
  positions.forEach((position, index) => {
    transform.makeTranslation(...position);
    trees.setMatrixAt(index, transform);
  });
  trees.instanceMatrix.needsUpdate = true;
  group.add(trees);
}

export function drawNavigationMap(
  canvas: HTMLCanvasElement,
  background: ImageData,
  positionNed: Vec3,
  headingRad: number,
) {
  const context = canvas.getContext("2d")!;
  context.putImageData(background, 0, 0);
  const scale = canvas.width / WORLD_SIZE_M;
  const x = canvas.width / 2 + positionNed[1] * scale;
  const y = canvas.height / 2 - positionNed[0] * scale;
  context.strokeStyle = "#ffffff";
  context.lineWidth = 2;
  context.beginPath();
  context.moveTo(x, y);
  context.lineTo(x + Math.sin(headingRad) * 12, y - Math.cos(headingRad) * 12);
  context.stroke();
  context.fillStyle = "#ffcf4a";
  context.beginPath();
  context.arc(x, y, 3.5, 0, Math.PI * 2);
  context.fill();
  context.fillStyle = "#f2aa3b";
  context.fillRect(canvas.width / 2 - 3, canvas.height / 2 - 3, 6, 6);
}

export function createMapBackground(
  canvas: HTMLCanvasElement,
  sampler: TerrainSampler,
) {
  const context = canvas.getContext("2d")!;
  const image = context.createImageData(canvas.width, canvas.height);
  for (let y = 0; y < canvas.height; y++) {
    for (let x = 0; x < canvas.width; x++) {
      const north = (0.5 - y / canvas.height) * WORLD_SIZE_M;
      const east = (x / canvas.width - 0.5) * WORLD_SIZE_M;
      const height = sampler.terrain_height(north, east);
      const shade = Math.max(0, Math.min(255, 55 + height * 0.16));
      const index = (y * canvas.width + x) * 4;
      image.data[index] = height < 8 ? 50 : shade * 0.7;
      image.data[index + 1] = height < 8 ? 115 : shade;
      image.data[index + 2] = height < 8 ? 155 : shade * 0.72;
      image.data[index + 3] = 235;
    }
  }
  return image;
}
