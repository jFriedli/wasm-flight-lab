import "./style.css";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import init, { FlightSimulator } from "./wasm/flight_wasm";
import { metrics, type SimState } from "./model";
import { setupWorkshop } from "./workshop";
import {
  attitudeBodyToNedToThree,
  bodyVectorToModel,
  nedPositionToThree,
  nedVectorToThree,
} from "./frames";
import {
  PROFILES,
  gamepadCommands,
  applyMouseDelta,
  loadCalibration,
  saveCalibration,
  shape,
  slew,
  springVirtualStick,
  type AxisName,
  type ControlProfile,
} from "./input";
import {
  createMapBackground,
  createTerrain,
  drawNavigationMap,
  TERRAIN_SEED,
  WORLD_SIZE_M,
} from "./terrain";
const DEG = 180 / Math.PI,
  DT = 0.004,
  AXES: AxisName[] = ["roll", "pitch", "yaw", "throttle"];
const root = document.querySelector<HTMLDivElement>("#app")!;
root.innerHTML = `<header><div><strong>WASM FLIGHT LAB</strong><small>Design it. Fly it. Break it. Understand it.</small></div><nav><button class="active">FLY</button><button disabled>BUILD</button><button disabled>MISSION</button><button disabled>ANALYZE</button></nav></header><main><section id="viewport"><div class="badge">LIVE · WASM 250 Hz</div><div class="toolbar"><span>MODE</span><button data-mode="ACRO" class="active">ACRO</button><button data-mode="ANGLE">ANGLE</button><span>CAMERA</span><button data-camera="CHASE" class="active">CHASE</button><button data-camera="FPV">FPV</button><button data-camera="FREE">FREE</button></div><div class="help">W/S throttle · A/D roll · ↑/↓ pitch · Q/E yaw · Space pause · R reset</div><div class="attitude"><i id="horizon"></i><span id="attitudeText">LEVEL</span></div></section><aside><section><h2>FLIGHT DATA</h2><dl><dt>Aircraft</dt><dd>Beginner Quad</dd><dt>Flight mode</dt><dd id="modeReadout">ACRO</dd><dt>Altitude</dt><dd id="alt">—</dd><dt>Ground speed</dt><dd id="speed">—</dd><dt>Airspeed</dt><dd id="airspeed">—</dd><dt>Vertical speed</dt><dd id="vertical">—</dd><dt>Attitude R / P / Y</dt><dd id="angles">—</dd><dt>Simulation</dt><dd id="simState">RUNNING</dd></dl><label class="range-label">Throttle <output id="throttleOut">31%</output><input id="throttle" type="range" min="0" max="100" value="31" list="throttleMarks"><datalist id="throttleMarks"><option value="31" label="hover"></option></datalist><small>Estimated hover 31%</small></label></section><details open><summary>CONTROLLER</summary><div class="segmented"><button data-profile="BEGINNER" class="active">BEGINNER</button><button data-profile="SPORT">SPORT</button><button data-profile="CUSTOM">CUSTOM</button></div><div class="response-grid"><label>Roll rate °/s<input data-response="roll" type="number" min="20" max="720" value="130"></label><label>Pitch rate °/s<input data-response="pitch" type="number" min="20" max="720" value="130"></label><label>Yaw rate °/s<input data-response="yaw" type="number" min="20" max="540" value="100"></label><label>Angle limit °<input data-response="angle" type="number" min="5" max="80" value="30"></label><label>Expo<input data-response="expo" type="number" min="0" max="1" step=".05" value=".55"></label><label>Attack /s<input data-response="attack" type="number" min=".2" max="10" step=".1" value="1.6"></label><label>Release /s<input data-response="release" type="number" min=".2" max="10" step=".1" value="3.2"></label></div><div id="axisTelemetry" class="axis-telemetry"></div><canvas id="graph" width="260" height="92"></canvas><label>Graph axis <select id="graphAxis"><option value="0">Roll</option><option value="1">Pitch</option><option value="2">Yaw</option></select></label><div id="pidGrid" class="pid-grid"></div><button id="pidReset">Reset safe defaults</button></details><details><summary>GAMEPAD & CALIBRATION</summary><p id="gamepadState" class="note">No controller connected</p><div id="gamepadAxes"></div><button id="captureCenter">Capture centers</button><button id="captureRange">Reset travel capture</button><p class="note">Centre, then move every stick through full travel. Calibration stays in this browser.</p></details><details><summary>VIEW & PHYSICS</summary><label><input id="vectors" type="checkbox" checked> Total forces</label><label><input id="motorVectors" type="checkbox"> Individual motor thrust</label><label>Known attitude <select id="knownAttitude"><option value="0,0">LEVEL</option><option value="30,0">ROLL +30°</option><option value="-30,0">ROLL -30°</option><option value="0,30">PITCH +30°</option><option value="0,-30">PITCH -30°</option></select></label><label>Chase distance <input id="chaseDistance" type="range" min="1.2" max="5" step=".1" value="2.4"></label><label>FOV <input id="fov" type="range" min="45" max="95" value="62"></label></details></aside></main><footer><span id="status">Initializing physics core…</span><div><button id="pause">Pause</button><select id="spawn"><option value="air">AIRBORNE</option><option value="ground">GROUND</option></select><button id="reset">Reset aircraft</button></div></footer>`;
const [flyNav, buildNav] = Array.from(
  document.querySelectorAll<HTMLButtonElement>("nav button"),
);
document
  .querySelector("aside section dl")!
  .insertAdjacentHTML(
    "beforeend",
    '<dt>Battery</dt><dd id="battery">—</dd><dt>Remaining</dt><dd id="batteryRemaining">—</dd><dt>Input source</dt><dd id="inputReadout">Keyboard + Mouse</dd><dt id="aoaLabel" hidden>Angle of attack</dt><dd id="aoa" hidden>—</dd>',
  );
document
  .querySelector("aside section dl")!
  .insertAdjacentHTML(
    "beforeend",
    '<dt id="vtolRegimeLabel" hidden>Flight regime</dt><dd id="vtolRegime" hidden>HOVER</dd><dt id="vtolForcesLabel" hidden>VTOL thrust</dt><dd id="vtolForces" hidden>—</dd>',
  );
buildNav.disabled = false;
const flyView = document.querySelector<HTMLElement>("main")!;
flyView.id = "flyView";
await init();
const sim = new FlightSimulator();
let state: SimState = JSON.parse(sim.state_json());
let activeBatteryCapacityMah = (
  JSON.parse(sim.vehicle_definition_json()) as {
    battery: { capacityMah: number };
  }
).battery.capacityMah;
let activeHoverThrottle = 0.31;
const view = document.querySelector<HTMLElement>("#viewport")!,
  scene = new THREE.Scene();
document.querySelector(".help")!.textContent =
  "W/S throttle · A/D yaw · Mouse pitch/roll · Arrows fallback · R reset";
view.insertAdjacentHTML(
  "beforeend",
  `<div id="mouseControls" class="mouse-controls"><button id="mouseFlight">ENABLE MOUSE FLIGHT</button><span id="mouseStatus">Click to capture · Esc releases</span><label>Source <select id="inputSource"><option value="mouse">KEYBOARD + MOUSE</option><option value="gamepad">GAMEPAD</option></select></label><label>Sensitivity <input id="mouseSensitivity" type="range" min="0.0005" max="0.008" step="0.0005" value="0.0025"></label><label>Return <input id="mouseReturn" type="range" min="0.3" max="4" step="0.1" value="1.4"></label><div id="virtualStick"><i></i><b>↑ PITCH FORWARD</b></div></div>`,
);
view.insertAdjacentHTML(
  "beforeend",
  '<div id="stallWarning" class="stall-warning" hidden>STALL · AoA HIGH</div>',
);
view.insertAdjacentHTML(
  "beforeend",
  '<div id="navigation"><button id="mapToggle">MAP</button><canvas id="map" width="180" height="180"></canvas><strong id="heading">N 000°</strong><span id="home">HOME 0.00 km · 000°</span><small id="coordinates">N 0 · E 0</small></div>',
);
view.insertAdjacentHTML(
  "beforeend",
  '<label id="transitionControl" hidden>VTOL TRANSITION <output>0%</output><input id="transition" type="range" min="0" max="100" value="0"><small>G hover · T cruise</small></label>',
);
document
  .querySelector(".toolbar")!
  .insertAdjacentHTML(
    "beforeend",
    '<span>ENV</span><select id="environment"><option value="alpine">ALPINE RANGE</option><option value="test">TEST RANGE</option></select>',
  );
scene.background = new THREE.Color(0x8fb9cf);
scene.fog = new THREE.Fog(0x8fb9cf, 900, 5_500);
const camera = new THREE.PerspectiveCamera(62, 1, 0.05, 10_000),
  renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
view.prepend(renderer.domElement);
const orbit = new OrbitControls(camera, renderer.domElement);
orbit.enabled = false;
orbit.enableDamping = true;
orbit.maxDistance = 4_000;
orbit.minDistance = 0.5;
scene.add(new THREE.HemisphereLight(0xd9f2ff, 0x37502b, 2));
const sun = new THREE.DirectionalLight(0xffffff, 2.4);
sun.position.set(8, 16, 7);
sun.castShadow = true;
scene.add(sun);
let alpineEnvironment = true;
sim.set_environment(true, TERRAIN_SEED);
let terrainGroup = createTerrain(sim, true);
scene.add(terrainGroup);
const runway = new THREE.Mesh(
  new THREE.PlaneGeometry(18, 700),
  new THREE.MeshStandardMaterial({ color: 0x30363a }),
);
runway.rotation.x = -Math.PI / 2;
runway.position.y = 0.012;
scene.add(runway);
for (let i = -300; i <= 300; i += 30) {
  const line = new THREE.Mesh(
    new THREE.PlaneGeometry(0.35, 12),
    new THREE.MeshBasicMaterial({ color: 0xffffff }),
  );
  line.rotation.x = -Math.PI / 2;
  line.position.set(0, 0.025, i);
  scene.add(line);
}
const mapCanvas = document.querySelector<HTMLCanvasElement>("#map")!;
let mapBackground = createMapBackground(mapCanvas, sim);
document.querySelector("#mapToggle")!.addEventListener("click", () => {
  mapCanvas.hidden = !mapCanvas.hidden;
});
document.querySelector<HTMLSelectElement>("#environment")!.onchange = (
  event,
) => {
  alpineEnvironment = (event.target as HTMLSelectElement).value === "alpine";
  sim.set_environment(alpineEnvironment, TERRAIN_SEED);
  view.dataset.environment = alpineEnvironment ? "alpine" : "test";
  scene.remove(terrainGroup);
  terrainGroup.traverse((object) => {
    if (object instanceof THREE.Mesh) object.geometry.dispose();
  });
  terrainGroup = createTerrain(sim, alpineEnvironment);
  scene.add(terrainGroup);
  mapBackground = createMapBackground(mapCanvas, sim);
  reset();
};
const craft = new THREE.Group(),
  body = new THREE.Mesh(
    new THREE.BoxGeometry(0.38, 0.13, 0.28),
    new THREE.MeshStandardMaterial({
      color: 0xf2aa3b,
      metalness: 0.25,
      roughness: 0.4,
    }),
  );
body.castShadow = true;
craft.add(body);
const flightMotorVisuals: THREE.Object3D[] = [];
const quadVisuals: THREE.Object3D[] = [body];
for (const [x, z] of [
  [0.18, 0.18],
  [0.18, -0.18],
  [-0.18, 0.18],
  [-0.18, -0.18],
]) {
  const arm = new THREE.Mesh(
    new THREE.CylinderGeometry(0.018, 0.018, Math.hypot(x, z) * 2, 8),
    new THREE.MeshStandardMaterial({ color: 0x14212a }),
  );
  arm.rotation.x = Math.PI / 2;
  arm.rotation.z = Math.atan2(x, z);
  craft.add(arm);
  quadVisuals.push(arm);
  const rotor = new THREE.Mesh(
    new THREE.CylinderGeometry(0.115, 0.115, 0.012, 24),
    new THREE.MeshStandardMaterial({
      color: 0x15242c,
      transparent: true,
      opacity: 0.6,
    }),
  );
  rotor.position.set(x, 0.045, z);
  craft.add(rotor);
  flightMotorVisuals.push(rotor);
  quadVisuals.push(rotor);
}
const fixedWingVisual = new THREE.Group();
const trainerMaterial = new THREE.MeshStandardMaterial({
  color: 0xe9edf0,
  roughness: 0.55,
});
const trainerAccent = new THREE.MeshStandardMaterial({
  color: 0xf2aa3b,
  roughness: 0.5,
});
const fuselage = new THREE.Mesh(
  new THREE.BoxGeometry(0.16, 0.18, 1.05),
  trainerMaterial,
);
const mainWing = new THREE.Mesh(
  new THREE.BoxGeometry(1.7, 0.025, 0.28),
  trainerMaterial,
);
mainWing.position.z = -0.02;
const horizontalTail = new THREE.Mesh(
  new THREE.BoxGeometry(0.55, 0.018, 0.18),
  trainerAccent,
);
horizontalTail.position.z = 0.43;
const verticalTail = new THREE.Mesh(
  new THREE.BoxGeometry(0.025, 0.25, 0.22),
  trainerAccent,
);
verticalTail.position.set(0, 0.1, 0.43);
const trainerPropeller = new THREE.Mesh(
  new THREE.CylinderGeometry(0.14, 0.14, 0.012, 28),
  new THREE.MeshStandardMaterial({
    color: 0x263b47,
    transparent: true,
    opacity: 0.55,
  }),
);
trainerPropeller.rotation.x = Math.PI / 2;
trainerPropeller.position.z = -0.53;
const leftAileron = new THREE.Mesh(
  new THREE.BoxGeometry(0.62, 0.014, 0.07),
  trainerAccent,
);
leftAileron.position.set(-0.5, -0.005, 0.11);
const rightAileron = new THREE.Mesh(
  new THREE.BoxGeometry(0.62, 0.014, 0.07),
  trainerAccent,
);
rightAileron.position.set(0.5, -0.005, 0.11);
fixedWingVisual.add(
  fuselage,
  mainWing,
  horizontalTail,
  verticalTail,
  trainerPropeller,
  leftAileron,
  rightAileron,
);
fixedWingVisual.visible = false;
craft.add(fixedWingVisual);
scene.add(craft);
const thrustArrow = new THREE.ArrowHelper(
    new THREE.Vector3(0, 1, 0),
    new THREE.Vector3(),
    1,
    0x3de3ff,
  ),
  velocityArrow = new THREE.ArrowHelper(
    new THREE.Vector3(0, 0, 1),
    new THREE.Vector3(),
    1,
    0xffcf4a,
  ),
  gravityArrow = new THREE.ArrowHelper(
    new THREE.Vector3(0, -1, 0),
    new THREE.Vector3(),
    1,
    0xff6f91,
  ),
  liftArrow = new THREE.ArrowHelper(
    new THREE.Vector3(0, 1, 0),
    new THREE.Vector3(),
    1,
    0x79f2a6,
  ),
  dragArrow = new THREE.ArrowHelper(
    new THREE.Vector3(0, 0, 1),
    new THREE.Vector3(),
    1,
    0xc28cff,
  );
const motorArrows = Array.from(
  { length: 8 },
  () =>
    new THREE.ArrowHelper(
      new THREE.Vector3(0, 1, 0),
      new THREE.Vector3(),
      0.2,
      0xffd166,
    ),
);
const surfaceArrows = Array.from(
  { length: 16 },
  () =>
    new THREE.ArrowHelper(
      new THREE.Vector3(0, 1, 0),
      new THREE.Vector3(),
      0.1,
      0x79f2a6,
    ),
);
scene.add(
  thrustArrow,
  velocityArrow,
  gravityArrow,
  liftArrow,
  dragArrow,
  ...motorArrows,
  ...surfaceArrows,
);
let profile: ControlProfile = {
    ...PROFILES.BEGINNER,
    rates: [...PROFILES.BEGINNER.rates],
  },
  mode: "ACRO" | "ANGLE" = "ACRO",
  cameraMode: "CHASE" | "FPV" | "FREE" = "CHASE",
  paused = false,
  collective = 0.31,
  accumulator = 0,
  last = performance.now(),
  gamepadWasActive = false,
  rangeCapture = false;
let inputSource: "mouse" | "gamepad" = "mouse";
let virtualStick = { roll: 0, pitch: 0 };
const commands = { roll: 0, pitch: 0, yaw: 0 },
  keys = new Set<string>();
const calibration = loadCalibration(localStorage);
const throttle = document.querySelector<HTMLInputElement>("#throttle")!,
  throttleOut = document.querySelector<HTMLOutputElement>("#throttleOut")!;
const inputSourceSelect =
    document.querySelector<HTMLSelectElement>("#inputSource")!,
  mouseSensitivity =
    document.querySelector<HTMLInputElement>("#mouseSensitivity")!,
  mouseReturn = document.querySelector<HTMLInputElement>("#mouseReturn")!,
  stickDot = document.querySelector<HTMLElement>("#virtualStick i")!;
const transitionInput =
  document.querySelector<HTMLInputElement>("#transition")!;
let transitionCommand = 0;
transitionInput.oninput = () => {
  transitionCommand = Number(transitionInput.value) / 100;
};
inputSourceSelect.onchange = () => {
  inputSource = inputSourceSelect.value as typeof inputSource;
  virtualStick = { roll: 0, pitch: 0 };
};
document
  .querySelector("#mouseFlight")!
  .addEventListener("click", () => view.requestPointerLock());
document.addEventListener("pointerlockchange", () => {
  const active = document.pointerLockElement === view;
  virtualStick = { roll: 0, pitch: 0 };
  document.querySelector("#mouseStatus")!.textContent = active
    ? "MOUSE FLIGHT ACTIVE · ESC to release"
    : "Click to capture · Esc releases";
  document.querySelector("#mouseFlight")!.textContent = active
    ? "MOUSE ACTIVE"
    : "ENABLE MOUSE FLIGHT";
});
document.addEventListener("mousemove", (event) => {
  if (document.pointerLockElement === view && inputSource === "mouse")
    virtualStick = applyMouseDelta(
      virtualStick,
      event.movementX,
      event.movementY,
      Number(mouseSensitivity.value),
    );
});
addEventListener("keydown", (e) => {
  if (["ArrowUp", "ArrowDown", "Space"].includes(e.code)) e.preventDefault();
  keys.add(e.code);
  if (e.code === "KeyR") reset();
  if (e.code === "Space") togglePause();
});
addEventListener("keyup", (e) => keys.delete(e.code));
addEventListener("blur", () => keys.clear());
throttle.oninput = () => {
  collective = Number(throttle.value) / 100;
};
function applyProfile(name: "BEGINNER" | "SPORT") {
  profile = { ...PROFILES[name], rates: [...PROFILES[name].rates] };
  sim.set_rates(...profile.rates, profile.maxAngle);
  const values: Record<string, number> = {
    roll: profile.rates[0],
    pitch: profile.rates[1],
    yaw: profile.rates[2],
    angle: profile.maxAngle,
    expo: profile.expo,
    attack: profile.attack,
    release: profile.release,
  };
  document
    .querySelectorAll<HTMLInputElement>("[data-response]")
    .forEach((input) => {
      input.value = String(values[input.dataset.response!]);
    });
  document
    .querySelectorAll("[data-profile]")
    .forEach((el) =>
      el.classList.toggle(
        "active",
        (el as HTMLElement).dataset.profile === name,
      ),
    );
}
document.querySelectorAll<HTMLButtonElement>("[data-profile]").forEach(
  (b) =>
    (b.onclick = () => {
      if (b.dataset.profile === "BEGINNER" || b.dataset.profile === "SPORT")
        applyProfile(b.dataset.profile);
      else {
        profile = { ...profile, name: "CUSTOM" };
        document
          .querySelectorAll("[data-profile]")
          .forEach((el) => el.classList.toggle("active", el === b));
      }
    }),
);
document
  .querySelectorAll<HTMLInputElement>("[data-response]")
  .forEach((input) => {
    input.onchange = () => {
      const key = input.dataset.response!;
      const min = Number(input.min),
        max = Number(input.max);
      const value = Math.min(max, Math.max(min, Number(input.value) || min));
      input.value = String(value);
      if (key === "roll") profile.rates[0] = value;
      else if (key === "pitch") profile.rates[1] = value;
      else if (key === "yaw") profile.rates[2] = value;
      else if (key === "angle") profile.maxAngle = value;
      else if (key === "expo") profile.expo = value;
      else if (key === "attack") profile.attack = value;
      else if (key === "release") profile.release = value;
      profile.name = "CUSTOM";
      sim.set_rates(...profile.rates, profile.maxAngle);
      document
        .querySelectorAll("[data-profile]")
        .forEach((el) =>
          el.classList.toggle(
            "active",
            (el as HTMLElement).dataset.profile === "CUSTOM",
          ),
        );
    };
  });
document.querySelectorAll<HTMLButtonElement>("[data-mode]").forEach(
  (b) =>
    (b.onclick = () => {
      mode = b.dataset.mode as typeof mode;
      sim.set_mode(mode === "ANGLE");
      document
        .querySelectorAll("[data-mode]")
        .forEach((el) => el.classList.toggle("active", el === b));
      document.querySelector("#modeReadout")!.textContent = mode;
    }),
);
document.querySelectorAll<HTMLButtonElement>("[data-camera]").forEach(
  (b) =>
    (b.onclick = () => {
      cameraMode = b.dataset.camera as typeof cameraMode;
      orbit.enabled = cameraMode === "FREE";
      if (orbit.enabled) {
        orbit.target.copy(craft.position);
        camera.position.copy(craft.position).add(new THREE.Vector3(3, 2, 3));
      }
      document
        .querySelectorAll("[data-camera]")
        .forEach((el) => el.classList.toggle("active", el === b));
    }),
);
document.querySelector("#pause")!.addEventListener("click", togglePause);
function togglePause() {
  paused = !paused;
  accumulator = 0;
  last = performance.now();
  document.querySelector("#pause")!.textContent = paused ? "Resume" : "Pause";
  document.querySelector("#simState")!.textContent = paused
    ? "PAUSED"
    : "RUNNING";
}
document.querySelector("#reset")!.addEventListener("click", reset);
function reset() {
  sim.spawn(
    document.querySelector<HTMLSelectElement>("#spawn")!.value === "air",
  );
  sim.set_mode(mode === "ANGLE");
  applyProfile(profile.name === "SPORT" ? "SPORT" : "BEGINNER");
  commands.roll = commands.pitch = commands.yaw = 0;
  collective =
    (JSON.parse(sim.state_json()) as SimState).vehicleClass === "FixedWing"
      ? 0
      : activeHoverThrottle;
  transitionCommand = 0;
  transitionInput.value = "0";
  sim.set_transition(0);
  accumulator = 0;
  history.length = 0;
}
const workshop = setupWorkshop(root, sim, (definition) => {
  workshop.hide();
  flyView.hidden = false;
  flyNav.classList.add("active");
  buildNav.classList.remove("active");
  activeBatteryCapacityMah = definition.battery.capacityMah;
  const fixedWing = definition.vehicleClass === "fixedWing";
  const vtol =
    definition.vehicleClass === "quadPlane" ||
    definition.vehicleClass === "tiltrotor";
  const winged = fixedWing || vtol;
  fixedWingVisual.visible = winged;
  quadVisuals.forEach(
    (object, index) => (object.visible = !winged || (vtol && index > 0)),
  );
  trainerPropeller.visible = definition.vehicleClass !== "tiltrotor";
  document.querySelector<HTMLElement>("#transitionControl")!.hidden = !vtol;
  body.scale.set(
    definition.frame.bodyDimensionsM[1] / 0.38,
    definition.frame.bodyDimensionsM[2] / 0.13,
    definition.frame.bodyDimensionsM[0] / 0.28,
  );
  definition.motors.forEach((motor, i) =>
    flightMotorVisuals[i]?.position.copy(bodyVectorToModel(motor.positionM)),
  );
  document.querySelector("aside dl dd")!.textContent = definition.name;
  const engineering = JSON.parse(sim.engineering_metrics_json()) as {
    hoverThrottle: number;
    estimatedStallSpeedMps: number;
  };
  activeHoverThrottle = Math.min(1, engineering.hoverThrottle);
  reset();
  collective = fixedWing ? 0 : activeHoverThrottle;
  document.querySelector("#modeReadout")!.textContent = fixedWing
    ? "TRAINER MANUAL"
    : vtol
      ? "HOVER"
      : mode;
  document.querySelector<HTMLButtonElement>(
    "[data-profile='BEGINNER']",
  )!.textContent = fixedWing ? "TRAINER" : "BEGINNER";
  mouseSensitivity.value = fixedWing ? "0.0015" : "0.0025";
  document
    .querySelectorAll<HTMLButtonElement>("[data-mode]")
    .forEach((button) => (button.disabled = fixedWing));
  document.querySelector<HTMLInputElement>("#chaseDistance")!.value = winged
    ? "3.4"
    : "2.4";
  document.querySelector(".range-label small")!.textContent = fixedWing
    ? "Forward propulsion · W/S persists"
    : `Estimated hover ${(engineering.hoverThrottle * 100).toFixed(0)}%`;
  throttle.value = String(Math.round(collective * 100));
  throttleOut.value = `${Math.round(collective * 100)}%`;
  view.dataset.vehicleDefinition = sim.vehicle_definition_json();
});
flyNav.onclick = () => {
  workshop.hide();
  flyView.hidden = false;
  flyNav.classList.add("active");
  buildNav.classList.remove("active");
};
buildNav.onclick = () => {
  flyView.hidden = true;
  workshop.show();
  flyNav.classList.remove("active");
  buildNav.classList.add("active");
};
document.querySelector<HTMLInputElement>("#fov")!.oninput = (e) => {
  camera.fov = Number((e.target as HTMLInputElement).value);
  camera.updateProjectionMatrix();
};
document.querySelector<HTMLSelectElement>("#knownAttitude")!.onchange = (
  event,
) => {
  const [roll, pitch] = (event.target as HTMLSelectElement).value
    .split(",")
    .map(Number);
  sim.set_attitude_degrees(roll, pitch, 0);
};
const tuning: Record<"roll" | "pitch" | "yaw", [number, number, number]> = {
    roll: [0.025, 0.008, 0.002],
    pitch: [0.025, 0.008, 0.002],
    yaw: [0.08, 0.015, 0.003],
  },
  pidGrid = document.querySelector("#pidGrid")!;
for (const [axis, index] of (["roll", "pitch", "yaw"] as const).map(
  (a, i) => [a, i] as const,
)) {
  pidGrid.insertAdjacentHTML(
    "beforeend",
    `<strong>${axis.toUpperCase()}</strong>${["P", "I", "D"].map((term, j) => `<label>${term}<input data-pid="${axis}-${j}" type="number" min="0" max="${j === 2 ? 0.2 : j === 1 ? 1 : 2}" step="${j === 2 ? 0.001 : 0.01}" value="${tuning[axis][j]}"></label>`).join("")}`,
  );
  document.querySelectorAll<HTMLInputElement>(`[data-pid^="${axis}-"]`).forEach(
    (input) =>
      (input.onchange = () => {
        const j = Number(input.dataset.pid!.split("-")[1]),
          max = j === 2 ? 0.2 : j === 1 ? 1 : 2;
        input.value = String(
          Math.min(max, Math.max(0, Number(input.value) || 0)),
        );
        tuning[axis][j] = Number(input.value);
        sim.set_pid(index, ...tuning[axis]);
        profile = { ...profile, name: "CUSTOM" };
        document
          .querySelectorAll("[data-profile]")
          .forEach((el) =>
            el.classList.toggle(
              "active",
              (el as HTMLElement).dataset.profile === "CUSTOM",
            ),
          );
      }),
  );
}
document.querySelector("#pidReset")!.addEventListener("click", () => {
  Object.assign(tuning, {
    roll: [0.025, 0.008, 0.002],
    pitch: [0.025, 0.008, 0.002],
    yaw: [0.08, 0.015, 0.003],
  });
  for (const [axis, index] of (["roll", "pitch", "yaw"] as const).map(
    (a, i) => [a, i] as const,
  )) {
    document
      .querySelectorAll<HTMLInputElement>(`[data-pid^="${axis}-"]`)
      .forEach((input, j) => (input.value = String(tuning[axis][j])));
    sim.set_pid(index, ...tuning[axis]);
  }
  applyProfile("BEGINNER");
});
function renderGamepadSettings() {
  const container = document.querySelector("#gamepadAxes")!;
  container.innerHTML = AXES.map((axis) => {
    const c = calibration.axes[axis];
    return `<fieldset><legend>${axis}</legend><label>Axis <input data-cal="${axis}-source" type="number" min="0" max="31" value="${c.source}"></label><label>Raw <output id="raw-${axis}">—</output></label><label>Normalized <output id="norm-${axis}">—</output></label><label>Min <input data-cal="${axis}-min" type="number" min="-2" max="0" step=".01" value="${c.min}"></label><label>Center <input data-cal="${axis}-center" type="number" min="-.8" max=".8" step=".01" value="${c.center}"></label><label>Max <input data-cal="${axis}-max" type="number" min="0" max="2" step=".01" value="${c.max}"></label><label>Deadzone <input data-cal="${axis}-deadzone" type="number" min="0" max=".3" step=".01" value="${c.deadzone}"></label><label>Expo <input data-cal="${axis}-expo" type="number" min="0" max="1" step=".05" value="${c.expo}"></label><label><input data-cal="${axis}-invert" type="checkbox" ${c.invert ? "checked" : ""}> Invert</label></fieldset>`;
  }).join("");
  container.querySelectorAll<HTMLInputElement>("[data-cal]").forEach(
    (input) =>
      (input.onchange = () => {
        const [axis, key] = input.dataset.cal!.split("-") as [
          AxisName,
          "source" | "min" | "center" | "max" | "deadzone" | "expo" | "invert",
        ];
        if (key === "invert") calibration.axes[axis].invert = input.checked;
        else calibration.axes[axis][key] = Number(input.value);
        saveCalibration(localStorage, calibration);
      }),
  );
}
renderGamepadSettings();
document.querySelector("#captureCenter")!.addEventListener("click", () => {
  const pad = navigator.getGamepads().find(Boolean);
  if (!pad) return;
  for (const axis of AXES) {
    const raw = pad.axes[calibration.axes[axis].source];
    if (Number.isFinite(raw)) calibration.axes[axis].center = raw;
  }
  saveCalibration(localStorage, calibration);
  renderGamepadSettings();
});
document.querySelector("#captureRange")!.addEventListener("click", () => {
  rangeCapture = true;
  for (const axis of AXES) {
    calibration.axes[axis].min = 0;
    calibration.axes[axis].max = 0;
  }
  renderGamepadSettings();
});
interface Sample {
  target: number;
  actual: number;
  output: number;
}
const history: Sample[] = [];
const graph = document.querySelector<HTMLCanvasElement>("#graph")!,
  ctx = graph.getContext("2d")!;
function drawGraph() {
  ctx.clearRect(0, 0, graph.width, graph.height);
  ctx.strokeStyle = "#263b47";
  ctx.beginPath();
  ctx.moveTo(0, graph.height / 2);
  ctx.lineTo(graph.width, graph.height / 2);
  ctx.stroke();
  for (const [key, color, scale] of [
    ["target", "#3de3ff", 0.35],
    ["actual", "#ffcf4a", 0.35],
    ["output", "#ff6f91", 2],
  ] as const) {
    ctx.strokeStyle = color;
    ctx.beginPath();
    history.forEach((s, i) => {
      const x = (i / Math.max(1, history.length - 1)) * graph.width,
        y = graph.height / 2 - (s[key] * scale * graph.height) / 2;
      if (i) ctx.lineTo(x, y);
      else ctx.moveTo(x, y);
    });
    ctx.stroke();
  }
}
function updateInput(dt: number) {
  const pad = navigator.getGamepads().find(Boolean);
  virtualStick = springVirtualStick(
    virtualStick,
    Number(mouseReturn.value),
    dt,
  );
  view.dataset.virtualStick = `${virtualStick.roll},${virtualStick.pitch}`;
  stickDot.style.transform = `translate(${virtualStick.roll * 35}px, ${virtualStick.pitch * 35}px)`;
  let targets = {
    roll: keys.has("ArrowRight")
      ? 1
      : keys.has("ArrowLeft")
        ? -1
        : virtualStick.roll,
    pitch: keys.has("ArrowUp")
      ? -1
      : keys.has("ArrowDown")
        ? 1
        : virtualStick.pitch,
    yaw: keys.has("KeyD") ? 1 : keys.has("KeyA") ? -1 : 0,
  };
  if (pad && inputSource === "gamepad") {
    gamepadWasActive = true;
    const p = gamepadCommands(pad, calibration);
    targets = { roll: p.roll, pitch: p.pitch, yaw: p.yaw };
    collective = p.throttle;
    document.querySelector("#gamepadState")!.textContent =
      `Connected: ${pad.id} · ${pad.axes.length} axes · ${pad.buttons.length} buttons`;
    for (const axis of AXES) {
      const c = calibration.axes[axis],
        raw = pad.axes[c.source] ?? Number.NaN;
      if (rangeCapture && Number.isFinite(raw)) {
        c.min = Math.min(c.min, raw);
        c.max = Math.max(c.max, raw);
        saveCalibration(localStorage, calibration);
      }
      document.querySelector(`#raw-${axis}`)!.textContent = Number.isFinite(raw)
        ? raw.toFixed(3)
        : "invalid";
      document.querySelector(`#norm-${axis}`)!.textContent = (
        axis === "throttle" ? p.throttle : p[axis]
      ).toFixed(3);
    }
  } else {
    document.querySelector("#gamepadState")!.textContent = gamepadWasActive
      ? "Controller disconnected · fail-safe active"
      : "No controller connected";
    if (gamepadWasActive && inputSource === "gamepad") {
      targets = { roll: 0, pitch: 0, yaw: 0 };
      collective = Math.max(0, collective - dt * 0.08);
    } else {
      if (keys.has("KeyW"))
        collective = Math.min(1, collective + profile.throttleSlew * dt);
      if (keys.has("KeyS"))
        collective = Math.max(0, collective - profile.throttleSlew * dt);
    }
  }
  for (const axis of ["roll", "pitch", "yaw"] as const)
    commands[axis] = slew(
      commands[axis],
      shape(
        targets[axis] * (state.vehicleClass === "FixedWing" ? 0.7 : 1),
        0,
        profile.expo,
      ),
      profile.attack,
      profile.release,
      dt,
    );
  sim.set_control(commands.roll, commands.pitch, commands.yaw, collective);
  if (keys.has("KeyT"))
    transitionCommand = Math.min(1, transitionCommand + dt * 0.2);
  if (keys.has("KeyG"))
    transitionCommand = Math.max(0, transitionCommand - dt * 0.25);
  transitionInput.value = String(Math.round(transitionCommand * 100));
  sim.set_transition(transitionCommand);
}
function resize() {
  const w = view.clientWidth,
    h = view.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(view);
resize();
const chasePosition = new THREE.Vector3();
function updateCamera() {
  const distance = Number(
    document.querySelector<HTMLInputElement>("#chaseDistance")!.value,
  );
  if (cameraMode === "CHASE") {
    const behind = new THREE.Vector3(0, 1, distance).applyQuaternion(
      craft.quaternion,
    );
    chasePosition.copy(craft.position).add(behind);
    chasePosition.y = Math.max(0.45, chasePosition.y);
    camera.position.lerp(chasePosition, 0.09);
    camera.lookAt(craft.position.clone().add(new THREE.Vector3(0, 0.12, 0)));
  } else if (cameraMode === "FPV") {
    const mount = new THREE.Vector3(0, 0.06, -0.16).applyQuaternion(
      craft.quaternion,
    );
    camera.position.copy(craft.position).add(mount);
    camera.quaternion.copy(craft.quaternion);
  } else {
    orbit.target.lerp(craft.position, 0.03);
    orbit.update();
  }
  const cameraTerrain = sim.terrain_height(
    -camera.position.z,
    camera.position.x,
  );
  camera.position.y = Math.max(camera.position.y, cameraTerrain + 1.5);
}
let lastMapUpdate = 0;
function updateHud() {
  const speed = Math.hypot(...state.velocity),
    angles = state.euler.map((v) => v * DEG);
  throttle.value = String(Math.round(collective * 100));
  throttleOut.value = `${throttle.value}%`;
  document.querySelector("#alt")!.textContent =
    `${(-state.position[2] - state.terrainHeight).toFixed(1)} m AGL`;
  document.querySelector("#speed")!.textContent = `${speed.toFixed(1)} m/s`;
  document.querySelector("#airspeed")!.textContent = `${speed.toFixed(1)} m/s`;
  const airspeed = Math.hypot(...state.airVelocity);
  document.querySelector("#airspeed")!.textContent =
    `${airspeed.toFixed(1)} m/s`;
  document.querySelector("#vertical")!.textContent =
    `${(-state.velocity[2]).toFixed(1)} m/s`;
  document.querySelector("#battery")!.textContent =
    `${state.battery.voltage.toFixed(1)} V · ${state.battery.current.toFixed(1)} A`;
  document.querySelector("#batteryRemaining")!.textContent =
    `${Math.max(0, (state.battery.remainingMah / activeBatteryCapacityMah) * 100).toFixed(0)}%`;
  document.querySelector("#inputReadout")!.textContent =
    inputSource === "gamepad" ? "Gamepad" : "Keyboard + Mouse";
  const fixedWing = state.vehicleClass === "FixedWing";
  const vtol =
    state.vehicleClass === "QuadPlane" || state.vehicleClass === "Tiltrotor";
  const winged = fixedWing || vtol;
  (document.querySelector("#aoaLabel") as HTMLElement).hidden = !winged;
  (document.querySelector("#aoa") as HTMLElement).hidden = !winged;
  document.querySelector("#aoa")!.textContent =
    `${(state.angleOfAttack * DEG).toFixed(1)}°`;
  (document.querySelector("#stallWarning") as HTMLElement).hidden = !(
    winged && state.stalled
  );
  for (const id of [
    "vtolRegimeLabel",
    "vtolRegime",
    "vtolForcesLabel",
    "vtolForces",
  ])
    (document.querySelector(`#${id}`) as HTMLElement).hidden = !vtol;
  document.querySelector("#vtolRegime")!.textContent = vtol
    ? `${state.transition.regime} · ${(state.transition.actual * 100).toFixed(0)}% · tilt ${(state.transition.tiltAngle * DEG).toFixed(0)}°`
    : "—";
  document.querySelector("#vtolForces")!.textContent = vtol
    ? `vertical ${state.transition.verticalThrust.toFixed(1)} N · forward ${state.transition.forwardThrust.toFixed(1)} N`
    : "—";
  document.querySelector("#transitionControl output")!.textContent =
    `${Math.round(state.transition.actual * 100)}%`;
  if (vtol)
    document.querySelector("#modeReadout")!.textContent =
      state.transition.regime;
  document.querySelector("#angles")!.textContent = angles
    .map((v) => `${v.toFixed(0)}°`)
    .join(" / ");
  document.querySelector("#attitudeText")!.textContent =
    `R ${angles[0].toFixed(0)}° · P ${angles[1].toFixed(0)}°`;
  const headingDegrees = ((angles[2] % 360) + 360) % 360;
  const cardinal = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"][
    Math.round(headingDegrees / 45) % 8
  ];
  document.querySelector("#heading")!.textContent =
    `${cardinal} ${headingDegrees.toFixed(0).padStart(3, "0")}°`;
  const homeDistance = Math.hypot(state.position[0], state.position[1]);
  const homeBearing =
    (((Math.atan2(-state.position[1], -state.position[0]) * DEG) % 360) + 360) %
    360;
  document.querySelector("#home")!.textContent =
    `HOME ${(homeDistance / 1000).toFixed(2)} km · ${homeBearing.toFixed(0).padStart(3, "0")}°`;
  document.querySelector("#coordinates")!.textContent =
    `N ${state.position[0].toFixed(0)} · E ${state.position[1].toFixed(0)}`;
  if (performance.now() - lastMapUpdate > 100) {
    drawNavigationMap(mapCanvas, mapBackground, state.position, state.euler[2]);
    lastMapUpdate = performance.now();
  }
  document.querySelector<HTMLElement>("#horizon")!.style.transform =
    `translateY(${Math.max(-24, Math.min(24, angles[1]))}px) rotate(${-angles[0]}deg)`;
  const fixedSurfaces = ["Left Wing", "Elevator", "Rudder"].map((name) =>
    state.forces.surfaces.find((surface) => surface.name === name),
  );
  document.querySelector("#axisTelemetry")!.innerHTML =
    ["Roll", "Pitch", "Yaw"]
      .map(
        (n, i) =>
          `<div><strong>${n}</strong><span>${fixedWing ? `Rate ${(state.rates[i] * DEG).toFixed(0)}°/s` : `T ${Math.round(state.control.target[i] * DEG)}°/s`}</span><span>${fixedWing ? `Cmd ${((fixedSurfaces[i]?.commandedDeflection ?? 0) * DEG).toFixed(0)}°` : `A ${Math.round(state.control.actual[i] * DEG)}°/s`}</span><span>${fixedWing ? `Act ${((fixedSurfaces[i]?.actualDeflection ?? 0) * DEG).toFixed(0)}°` : `E ${Math.round(state.control.error[i] * DEG)}°/s`}</span><span>${fixedWing ? `Stick ${state.control.sticks[i].toFixed(2)}` : `O ${state.control.output[i].toFixed(3)}`}</span></div>`,
      )
      .join("") +
    `<div class="motors"><strong>Motors</strong>${state.control.motors.map((v, i) => `<span>M${i + 1} ${Math.round(v * 100)}%</span>`).join("")}</div>`;
  const axis = Number(
    document.querySelector<HTMLSelectElement>("#graphAxis")!.value,
  );
  history.push({
    target: state.control.target[axis],
    actual: state.control.actual[axis],
    output: state.control.output[axis],
  });
  if (history.length > 240) history.shift();
  drawGraph();
  const boundsWarning =
    homeDistance > WORLD_SIZE_M * 0.43 ? " · RETURN TOWARD HOME" : "";
  document.querySelector("#status")!.textContent = winged
    ? `Simulation running locally · ${state.time.toFixed(1)} s · AoA ${(state.angleOfAttack * DEG).toFixed(1)}°${state.stalled ? " · STALL" : ""}${boundsWarning}`
    : `Simulation running locally · ${state.time.toFixed(1)} s · hover ${(metrics(state.mass).hoverThrottle * 100).toFixed(0)}%${boundsWarning}`;
}
function frame(now: number) {
  const elapsed = Math.min((now - last) / 1000, 0.05);
  last = now;
  if (!paused) accumulator += elapsed;
  else accumulator = 0;
  while (accumulator >= DT) {
    updateInput(DT);
    sim.step(DT);
    accumulator -= DT;
  }
  state = JSON.parse(sim.state_json()) as SimState;
  view.dataset.terrainHeight = String(state.terrainHeight);
  view.dataset.environment = alpineEnvironment ? "alpine" : "test";
  view.dataset.rates = state.rates.join(",");
  nedPositionToThree(state.position, craft.position);
  attitudeBodyToNedToThree(state.attitude, craft.quaternion);
  const setArrow = (
    arrow: THREE.ArrowHelper,
    vector: [number, number, number],
    scale: number,
  ) => {
    const converted = nedVectorToThree(vector);
    const magnitude = converted.length();
    if (magnitude > 1e-9) arrow.setDirection(converted.normalize());
    arrow.setLength(Math.max(0.01, magnitude * scale));
    arrow.position.copy(craft.position);
  };
  setArrow(thrustArrow, state.forces.thrust, 1 / 12);
  setArrow(gravityArrow, state.forces.gravity, 1 / 12);
  setArrow(liftArrow, state.forces.lift, 1 / 12);
  setArrow(dragArrow, state.forces.drag, 1 / 3);
  setArrow(velocityArrow, state.velocity, 0.25);
  const thrustDirection = nedVectorToThree(state.forces.thrust).normalize();
  const modelThrustDirection = new THREE.Vector3(
    0,
    state.vehicleClass === "FixedWing" ? 0 : 1,
    state.vehicleClass === "FixedWing" ? -1 : 0,
  )
    .applyQuaternion(craft.quaternion)
    .normalize();
  view.dataset.thrustDirection = thrustDirection.toArray().join(",");
  view.dataset.modelThrustDirection = modelThrustDirection.toArray().join(",");
  view.dataset.velocityDirection = nedVectorToThree(state.velocity)
    .normalize()
    .toArray()
    .join(",");
  state.forces.motors.forEach((motor, index) => {
    const arrow = motorArrows[index];
    const origin = bodyVectorToModel(motor.positionBody)
      .applyQuaternion(craft.quaternion)
      .add(craft.position);
    const force = nedVectorToThree(motor.forceNed);
    arrow.position.copy(origin);
    if (force.lengthSq() > 1e-12) arrow.setDirection(force.normalize());
    arrow.setLength(Math.max(0.01, Math.hypot(...motor.forceNed) / 8));
  });
  state.forces.surfaces.forEach((surface, index) => {
    const arrow = surfaceArrows[index];
    if (!arrow) return;
    const origin = bodyVectorToModel(surface.positionBody)
      .applyQuaternion(craft.quaternion)
      .add(craft.position);
    const force = nedVectorToThree(surface.liftNed).add(
      nedVectorToThree(surface.dragNed),
    );
    arrow.position.copy(origin);
    if (force.lengthSq() > 1e-12) arrow.setDirection(force.normalize());
    arrow.setLength(Math.max(0.01, force.length() / 8));
  });
  if (state.vehicleClass !== "Multicopter") {
    const deflection = (name: string) =>
      state.forces.surfaces.find((surface) => surface.name === name)
        ?.actualDeflection ?? 0;
    leftAileron.rotation.x = -deflection("Left Wing");
    rightAileron.rotation.x = -deflection("Right Wing");
    horizontalTail.rotation.x = -deflection("Elevator");
    verticalTail.rotation.y = deflection("Rudder");
  }
  if (state.vehicleClass === "Tiltrotor") {
    flightMotorVisuals.forEach(
      (rotor) => (rotor.rotation.x = state.transition.tiltAngle),
    );
  } else {
    flightMotorVisuals.forEach((rotor) => (rotor.rotation.x = 0));
  }
  const vectors = document.querySelector<HTMLInputElement>("#vectors")!.checked;
  thrustArrow.visible = vectors;
  velocityArrow.visible = vectors;
  gravityArrow.visible = vectors;
  liftArrow.visible = vectors && Math.hypot(...state.forces.lift) > 0.01;
  dragArrow.visible = vectors && Math.hypot(...state.forces.drag) > 0.01;
  const showMotors =
    document.querySelector<HTMLInputElement>("#motorVectors")!.checked;
  surfaceArrows.forEach(
    (arrow, index) =>
      (arrow.visible = showMotors && index < state.forces.surfaces.length),
  );
  motorArrows.forEach((arrow) => (arrow.visible = showMotors));
  updateCamera();
  updateHud();
  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}
applyProfile("BEGINNER");
sim.set_mode(false);
document.querySelector("#status")!.textContent =
  "Simulation running locally · No server";
requestAnimationFrame(frame);
