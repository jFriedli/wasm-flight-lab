import "./style.css";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import init, { FlightSimulator } from "./wasm/flight_wasm";
import { metrics, type SimState } from "./model";
import {
  PROFILES,
  gamepadCommands,
  loadCalibration,
  saveCalibration,
  shape,
  slew,
  type AxisName,
  type ControlProfile,
} from "./input";
const DEG = 180 / Math.PI,
  DT = 0.004,
  AXES: AxisName[] = ["roll", "pitch", "yaw", "throttle"];
const root = document.querySelector<HTMLDivElement>("#app")!;
root.innerHTML = `<header><div><strong>WASM FLIGHT LAB</strong><small>Design it. Fly it. Break it. Understand it.</small></div><nav><button class="active">FLY</button><button disabled>BUILD</button><button disabled>MISSION</button><button disabled>ANALYZE</button></nav></header><main><section id="viewport"><div class="badge">LIVE · WASM 250 Hz</div><div class="toolbar"><span>MODE</span><button data-mode="ACRO" class="active">ACRO</button><button data-mode="ANGLE">ANGLE</button><span>CAMERA</span><button data-camera="CHASE" class="active">CHASE</button><button data-camera="FPV">FPV</button><button data-camera="FREE">FREE</button></div><div class="help">W/S throttle · A/D roll · ↑/↓ pitch · Q/E yaw · Space pause · R reset</div><div class="attitude"><i id="horizon"></i><span id="attitudeText">LEVEL</span></div></section><aside><section><h2>FLIGHT DATA</h2><dl><dt>Aircraft</dt><dd>Beginner Quad</dd><dt>Flight mode</dt><dd id="modeReadout">ACRO</dd><dt>Altitude</dt><dd id="alt">—</dd><dt>Ground speed</dt><dd id="speed">—</dd><dt>Airspeed</dt><dd id="airspeed">—</dd><dt>Vertical speed</dt><dd id="vertical">—</dd><dt>Attitude R / P / Y</dt><dd id="angles">—</dd><dt>Simulation</dt><dd id="simState">RUNNING</dd></dl><label class="range-label">Throttle <output id="throttleOut">31%</output><input id="throttle" type="range" min="0" max="100" value="31" list="throttleMarks"><datalist id="throttleMarks"><option value="31" label="hover"></option></datalist><small>Estimated hover 31%</small></label></section><details open><summary>CONTROLLER</summary><div class="segmented"><button data-profile="BEGINNER" class="active">BEGINNER</button><button data-profile="SPORT">SPORT</button><button data-profile="CUSTOM">CUSTOM</button></div><div class="response-grid"><label>Roll rate °/s<input data-response="roll" type="number" min="20" max="720" value="130"></label><label>Pitch rate °/s<input data-response="pitch" type="number" min="20" max="720" value="130"></label><label>Yaw rate °/s<input data-response="yaw" type="number" min="20" max="540" value="100"></label><label>Angle limit °<input data-response="angle" type="number" min="5" max="80" value="30"></label><label>Expo<input data-response="expo" type="number" min="0" max="1" step=".05" value=".55"></label><label>Attack /s<input data-response="attack" type="number" min=".2" max="10" step=".1" value="1.6"></label><label>Release /s<input data-response="release" type="number" min=".2" max="10" step=".1" value="3.2"></label></div><div id="axisTelemetry" class="axis-telemetry"></div><canvas id="graph" width="260" height="92"></canvas><label>Graph axis <select id="graphAxis"><option value="0">Roll</option><option value="1">Pitch</option><option value="2">Yaw</option></select></label><div id="pidGrid" class="pid-grid"></div><button id="pidReset">Reset safe defaults</button></details><details><summary>GAMEPAD & CALIBRATION</summary><p id="gamepadState" class="note">No controller connected</p><div id="gamepadAxes"></div><button id="captureCenter">Capture centers</button><button id="captureRange">Reset travel capture</button><p class="note">Centre, then move every stick through full travel. Calibration stays in this browser.</p></details><details><summary>VIEW & PHYSICS</summary><label><input id="vectors" type="checkbox" checked> Force vectors</label><label>Chase distance <input id="chaseDistance" type="range" min="1.2" max="5" step=".1" value="2.4"></label><label>FOV <input id="fov" type="range" min="45" max="95" value="62"></label></details></aside></main><footer><span id="status">Initializing physics core…</span><div><button id="pause">Pause</button><select id="spawn"><option value="air">AIRBORNE</option><option value="ground">GROUND</option></select><button id="reset">Reset aircraft</button></div></footer>`;
await init();
const sim = new FlightSimulator();
let state: SimState = JSON.parse(sim.state_json());
const view = document.querySelector<HTMLElement>("#viewport")!,
  scene = new THREE.Scene();
scene.background = new THREE.Color(0x8fb9cf);
scene.fog = new THREE.Fog(0x8fb9cf, 35, 120);
const camera = new THREE.PerspectiveCamera(62, 1, 0.05, 500),
  renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
view.prepend(renderer.domElement);
const orbit = new OrbitControls(camera, renderer.domElement);
orbit.enabled = false;
orbit.enableDamping = true;
orbit.maxDistance = 40;
orbit.minDistance = 0.5;
scene.add(new THREE.HemisphereLight(0xd9f2ff, 0x37502b, 2));
const sun = new THREE.DirectionalLight(0xffffff, 2.4);
sun.position.set(8, 16, 7);
sun.castShadow = true;
scene.add(sun);
const ground = new THREE.Mesh(
  new THREE.PlaneGeometry(180, 180),
  new THREE.MeshStandardMaterial({ color: 0x567847 }),
);
ground.rotation.x = -Math.PI / 2;
ground.receiveShadow = true;
scene.add(ground);
const runway = new THREE.Mesh(
  new THREE.PlaneGeometry(8, 70),
  new THREE.MeshStandardMaterial({ color: 0x30363a }),
);
runway.rotation.x = -Math.PI / 2;
runway.position.y = 0.012;
scene.add(runway);
for (let i = -30; i <= 30; i += 6) {
  const line = new THREE.Mesh(
    new THREE.PlaneGeometry(0.18, 3),
    new THREE.MeshBasicMaterial({ color: 0xffffff }),
  );
  line.rotation.x = -Math.PI / 2;
  line.position.set(0, 0.025, i);
  scene.add(line);
}
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
}
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
  );
scene.add(thrustArrow, velocityArrow);
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
const commands = { roll: 0, pitch: 0, yaw: 0 },
  keys = new Set<string>();
const calibration = loadCalibration(localStorage);
const throttle = document.querySelector<HTMLInputElement>("#throttle")!,
  throttleOut = document.querySelector<HTMLOutputElement>("#throttleOut")!;
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
  collective = 0.31;
  accumulator = 0;
  history.length = 0;
}
document.querySelector<HTMLInputElement>("#fov")!.oninput = (e) => {
  camera.fov = Number((e.target as HTMLInputElement).value);
  camera.updateProjectionMatrix();
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
  let targets = {
    roll: keys.has("KeyD") ? 1 : keys.has("KeyA") ? -1 : 0,
    pitch: keys.has("ArrowUp") ? 1 : keys.has("ArrowDown") ? -1 : 0,
    yaw: keys.has("KeyE") ? 1 : keys.has("KeyQ") ? -1 : 0,
  };
  let gamepadActive = false;
  if (pad) {
    gamepadActive = true;
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
    if (gamepadWasActive) {
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
      shape(targets[axis], 0, profile.expo),
      profile.attack,
      profile.release,
      dt,
    );
  const shaped = gamepadActive
    ? commands
    : {
        roll: shape(commands.roll, 0, profile.expo),
        pitch: shape(commands.pitch, 0, profile.expo),
        yaw: shape(commands.yaw, 0, profile.expo),
      };
  sim.set_control(shaped.roll, shaped.pitch, shaped.yaw, collective);
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
    const behind = new THREE.Vector3(0, 1, -distance).applyQuaternion(
      craft.quaternion,
    );
    chasePosition.copy(craft.position).add(behind);
    chasePosition.y = Math.max(0.45, chasePosition.y);
    camera.position.lerp(chasePosition, 0.09);
    camera.lookAt(craft.position.clone().add(new THREE.Vector3(0, 0.12, 0)));
  } else if (cameraMode === "FPV") {
    const mount = new THREE.Vector3(0, 0.06, 0.16).applyQuaternion(
      craft.quaternion,
    );
    camera.position.copy(craft.position).add(mount);
    camera.quaternion.copy(craft.quaternion);
    camera.rotateY(Math.PI);
  } else {
    orbit.target.lerp(craft.position, 0.03);
    orbit.update();
  }
}
function updateHud() {
  const m = metrics(state.mass),
    speed = Math.hypot(...state.velocity),
    angles = state.euler.map((v) => v * DEG);
  throttle.value = String(Math.round(collective * 100));
  throttleOut.value = `${throttle.value}%`;
  document.querySelector("#alt")!.textContent =
    `${(-state.position[2]).toFixed(1)} m`;
  document.querySelector("#speed")!.textContent = `${speed.toFixed(1)} m/s`;
  document.querySelector("#airspeed")!.textContent = `${speed.toFixed(1)} m/s`;
  document.querySelector("#vertical")!.textContent =
    `${(-state.velocity[2]).toFixed(1)} m/s`;
  document.querySelector("#angles")!.textContent = angles
    .map((v) => `${v.toFixed(0)}°`)
    .join(" / ");
  document.querySelector("#attitudeText")!.textContent =
    `R ${angles[0].toFixed(0)}° · P ${angles[1].toFixed(0)}°`;
  document.querySelector<HTMLElement>("#horizon")!.style.transform =
    `translateY(${Math.max(-24, Math.min(24, angles[1]))}px) rotate(${-angles[0]}deg)`;
  document.querySelector("#axisTelemetry")!.innerHTML =
    ["Roll", "Pitch", "Yaw"]
      .map(
        (n, i) =>
          `<div><strong>${n}</strong><span>T ${Math.round(state.control.target[i] * DEG)}°/s</span><span>A ${Math.round(state.control.actual[i] * DEG)}°/s</span><span>E ${Math.round(state.control.error[i] * DEG)}°/s</span><span>O ${state.control.output[i].toFixed(3)}</span></div>`,
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
  document.querySelector("#status")!.textContent =
    `Simulation running locally · ${state.time.toFixed(1)} s · hover ${(m.hoverThrottle * 100).toFixed(0)}%`;
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
  craft.position.set(state.position[1], -state.position[2], state.position[0]);
  craft.quaternion.set(
    state.attitude[1],
    -state.attitude[2],
    -state.attitude[0],
    state.attitude[3],
  );
  thrustArrow.position.copy(craft.position);
  thrustArrow.setLength(
    Math.max(0.01, Math.hypot(...state.forces.thrust) / 12),
  );
  velocityArrow.position.copy(craft.position);
  const velocity = new THREE.Vector3(
    state.velocity[1],
    -state.velocity[2],
    state.velocity[0],
  );
  if (velocity.lengthSq() > 0.001) {
    velocityArrow.setDirection(velocity.normalize());
    velocityArrow.setLength(Math.min(4, Math.hypot(...state.velocity) * 0.25));
  }
  const vectors = document.querySelector<HTMLInputElement>("#vectors")!.checked;
  thrustArrow.visible = vectors;
  velocityArrow.visible = vectors;
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
