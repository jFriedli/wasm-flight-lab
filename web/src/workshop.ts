import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { FlightSimulator } from "./wasm/flight_wasm";
import { bodyVectorToModel } from "./frames";
import {
  loadDesigns,
  parseVehicle,
  safeFilename,
  saveDesigns,
  type EngineeringMetrics,
  type VehicleDefinition,
} from "./vehicle";

export function setupWorkshop(
  root: HTMLElement,
  sim: FlightSimulator,
  onTestFlight: (definition: VehicleDefinition) => void,
) {
  const panel = document.createElement("section");
  panel.id = "buildView";
  panel.hidden = true;
  panel.innerHTML = `<div class="build-components"><h2>COMPONENTS</h2><label>Preset <select id="vehiclePreset"><option value="beginner">Beginner Quad</option><option value="freestyle">Freestyle Quad</option><option value="trainer">Fixed Wing Trainer</option><option value="quadplane">QuadPlane Explorer</option><option value="tiltrotor">Tiltrotor Research VTOL</option></select></label><div id="componentList"></div><button id="addPayload">+ Payload</button><hr><label>Saved designs <select id="savedDesigns"><option value="">—</option></select></label><div class="button-row"><button id="saveDesign">Save</button><button id="duplicateDesign">Duplicate</button><button id="deleteDesign">Delete</button></div><div class="button-row"><button id="exportDesign">Export</button><button id="importDesign">Import</button><input id="importFile" type="file" accept=".json,.flightlab.json,application/json" hidden></div></div><div class="build-preview"><div id="buildCanvas"></div><div id="componentEditor"></div></div><div class="build-metrics"><h2>ENGINEERING METRICS</h2><div id="metrics"></div><h2>DESIGN GUIDANCE</h2><div id="designWarnings"></div><button id="testFlight" class="primary">TEST FLIGHT</button><p id="buildStatus" class="note"></p></div>`;
  root.append(panel);
  let definition = JSON.parse(
      sim.vehicle_definition_json(),
    ) as VehicleDefinition,
    selected = "frame";
  const saved = loadDesigns(localStorage);
  const canvas = panel.querySelector<HTMLElement>("#buildCanvas")!,
    scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0b1a25);
  const camera = new THREE.PerspectiveCamera(48, 1, 0.01, 50);
  camera.position.set(1.1, 0.8, 1.1);
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  canvas.append(renderer.domElement);
  const orbit = new OrbitControls(camera, renderer.domElement);
  orbit.enableDamping = true;
  scene.add(new THREE.HemisphereLight(0xd9f2ff, 0x223322, 2.2));
  const grid = new THREE.GridHelper(4, 20, 0x385463, 0x20333d);
  scene.add(grid);
  const vehicleGroup = new THREE.Group();
  scene.add(vehicleGroup);
  const cgMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.025, 16, 12),
    new THREE.MeshBasicMaterial({ color: 0xff4f80 }),
  );
  scene.add(cgMarker);
  new ResizeObserver(() => {
    renderer.setSize(canvas.clientWidth, canvas.clientHeight, false);
    camera.aspect = canvas.clientWidth / canvas.clientHeight;
    camera.updateProjectionMatrix();
  }).observe(canvas);
  function animate() {
    if (!panel.hidden) {
      orbit.update();
      renderer.render(scene, camera);
    }
    requestAnimationFrame(animate);
  }
  animate();
  const numberInput = (
    label: string,
    path: string,
    value: number,
    min: number,
    max: number,
    step = 0.01,
  ) =>
    `<label>${label}<input data-path="${path}" type="number" min="${min}" max="${max}" step="${step}" value="${value}"></label>`;
  function componentList() {
    const list = panel.querySelector("#componentList")!;
    list.innerHTML = `<button data-component="frame">Frame</button>${definition.motors.map((_, i) => `<button data-component="motor-${i}">Motor ${i + 1}</button>`).join("")}${(definition.aeroSurfaces ?? []).map((s, i) => `<button data-component="surface-${i}">${escapeText(s.name)}</button>`).join("")}<button data-component="battery">Battery</button>${definition.payloads.map((p, i) => `<button data-component="payload-${i}">${escapeText(p.name)}</button>`).join("")}`;
    list
      .querySelectorAll<HTMLButtonElement>("[data-component]")
      .forEach((button) => {
        button.classList.toggle(
          "active",
          button.dataset.component === selected,
        );
        button.onclick = () => {
          selected = button.dataset.component!;
          componentList();
          editor();
        };
      });
  }
  function editor() {
    const e = panel.querySelector("#componentEditor")!;
    if (selected === "frame") {
      const f = definition.frame;
      e.innerHTML = `<h2>FRAME</h2><label>Name<input data-path="name" value="${escapeText(definition.name)}" maxlength="80"></label>${numberInput("Arm length m", "frame.armLengthM", f.armLengthM, 0.03, 2)}${numberInput("Body mass kg", "frame.bodyMassKg", f.bodyMassKg, 0.02, 100)}<div class="xyz">${f.bodyDimensionsM.map((v, i) => numberInput(`Body ${"XYZ"[i]} m`, `frame.bodyDimensionsM.${i}`, v, 0.02, 5)).join("")}</div>`;
    } else if (selected.startsWith("motor-")) {
      const i = Number(selected.split("-")[1]),
        m = definition.motors[i];
      e.innerHTML = `<h2>MOTOR ${i + 1} · ${(m.role ?? "lift").toUpperCase()}</h2><div class="xyz">${m.positionM.map((v, j) => numberInput(`Position ${"XYZ"[j]} m`, `motors.${i}.positionM.${j}`, v, -10, 10)).join("")}</div><div class="xyz">${m.directionBody.map((v, j) => numberInput(`Direction ${"XYZ"[j]}`, `motors.${i}.directionBody.${j}`, v, -1, 1, 0.1)).join("")}</div>${numberInput("Tilt rate °/s", `motors.${i}.tiltRateDegS`, m.tiltRateDegS ?? 30, 5, 180, 5)}${numberInput("Spin direction (-1/+1)", `motors.${i}.spin`, m.spin, -1, 1, 2)}${numberInput("Maximum thrust N", `motors.${i}.baseMaxThrustN`, m.baseMaxThrustN, 0, 10000)}${numberInput("Maximum power W", `motors.${i}.maxPowerW`, m.maxPowerW, 1, 100000, 1)}${numberInput("Motor mass kg", `motors.${i}.massKg`, m.massKg, 0.001, 10, 0.001)}${numberInput("Reaction torque Nm", `motors.${i}.reactionTorqueNm`, m.reactionTorqueNm, 0, 100)}${numberInput("Spin-up time s", `motors.${i}.spinUpTimeS`, m.spinUpTimeS, 0.005, 5, 0.005)}${numberInput("Spin-down time s", `motors.${i}.spinDownTimeS`, m.spinDownTimeS, 0.005, 5, 0.005)}<h2>PROPELLER</h2>${numberInput("Diameter m", `motors.${i}.propeller.diameterM`, m.propeller.diameterM, 0.02, 2)}${numberInput("Pitch m", `motors.${i}.propeller.pitchM`, m.propeller.pitchM, 0.01, 1)}${numberInput("Blades", `motors.${i}.propeller.bladeCount`, m.propeller.bladeCount, 1, 8, 1)}${numberInput("Propeller mass kg", `motors.${i}.propeller.massKg`, m.propeller.massKg, 0.001, 5, 0.001)}${numberInput("Efficiency estimate", `motors.${i}.propeller.efficiency`, m.propeller.efficiency, 0.1, 1, 0.01)}`;
    } else if (selected.startsWith("surface-")) {
      const i = Number(selected.split("-")[1]),
        s = definition.aeroSurfaces![i];
      e.innerHTML = `<h2>${escapeText(s.name).toUpperCase()}</h2><div class="xyz">${s.positionM.map((v, j) => numberInput(`Position ${"XYZ"[j]} m`, `aeroSurfaces.${i}.positionM.${j}`, v, -10, 10)).join("")}</div>${numberInput("Mass kg", `aeroSurfaces.${i}.massKg`, s.massKg ?? 0.05, 0.001, 50, 0.005)}${numberInput("Span m", `aeroSurfaces.${i}.spanM`, s.spanM, 0.05, 20)}${numberInput("Chord m", `aeroSurfaces.${i}.chordM`, s.chordM, 0.02, 5)}${numberInput("Area m²", `aeroSurfaces.${i}.areaM2`, s.areaM2, 0.005, 20)}${numberInput("Incidence °", `aeroSurfaces.${i}.incidenceDeg`, s.incidenceDeg, -30, 30, 0.5)}${numberInput("Lift slope /rad", `aeroSurfaces.${i}.liftSlopePerRad`, s.liftSlopePerRad, 0.1, 12, 0.1)}${numberInput("Stall angle °", `aeroSurfaces.${i}.stallAngleDeg`, s.stallAngleDeg, 5, 45, 0.5)}${numberInput("CD0", `aeroSurfaces.${i}.cd0`, s.cd0, 0.001, 2, 0.005)}${numberInput("Induced drag k", `aeroSurfaces.${i}.inducedDragK`, s.inducedDragK, 0, 2, 0.005)}${numberInput("Max deflection °", `aeroSurfaces.${i}.maxDeflectionDeg`, s.maxDeflectionDeg, 0, 60, 0.5)}${numberInput("Control effectiveness", `aeroSurfaces.${i}.controlEffectiveness`, s.controlEffectiveness, 0, 2, 0.01)}${numberInput("Trim °", `aeroSurfaces.${i}.trimDeflectionDeg`, s.trimDeflectionDeg ?? 0, -20, 20, 0.25)}${numberInput("Servo rate °/s", `aeroSurfaces.${i}.servoRateDegS`, s.servoRateDegS ?? 180, 10, 1000, 10)}`;
    } else if (selected === "battery") {
      const b = definition.battery;
      e.innerHTML = `<h2>BATTERY</h2>${numberInput("Cells", "battery.cells", b.cells, 1, 16, 1)}${numberInput("Capacity mAh", "battery.capacityMah", b.capacityMah, 100, 100000, 50)}${numberInput("Mass kg", "battery.massKg", b.massKg, 0.02, 100, 0.01)}<div class="xyz">${b.positionM.map((v, j) => numberInput(`Position ${"XYZ"[j]} m`, `battery.positionM.${j}`, v, -10, 10)).join("")}</div>${numberInput("Internal resistance Ω", "battery.internalResistanceOhm", b.internalResistanceOhm, 0, 2, 0.005)}${numberInput("Maximum discharge C", "battery.maxDischargeC", b.maxDischargeC, 1, 200, 1)}`;
    } else {
      const i = Number(selected.split("-")[1]),
        p = definition.payloads[i];
      if (!p) {
        selected = "frame";
        editor();
        return;
      }
      e.innerHTML = `<h2>PAYLOAD</h2><label>Name<input data-path="payloads.${i}.name" value="${escapeText(p.name)}" maxlength="60"></label>${numberInput("Mass kg", `payloads.${i}.massKg`, p.massKg, 0.001, 1000, 0.01)}<div class="xyz">${p.positionM.map((v, j) => numberInput(`Position ${"XYZ"[j]} m`, `payloads.${i}.positionM.${j}`, v, -10, 10)).join("")}</div><button id="removePayload">Remove payload</button>`;
      e.querySelector<HTMLButtonElement>("#removePayload")!.onclick = () => {
        definition.payloads.splice(i, 1);
        selected = "frame";
        apply();
      };
    }
    e.querySelectorAll<HTMLInputElement>("[data-path]").forEach(
      (input) =>
        (input.onchange = () => {
          setPath(
            definition,
            input.dataset.path!,
            input.type === "number" ? Number(input.value) : input.value,
          );
          if (
            input.dataset.path === "frame.armLengthM" &&
            (definition.vehicleClass ?? "multicopter") === "multicopter"
          ) {
            for (const motor of definition.motors) {
              motor.positionM[0] =
                Math.sign(motor.positionM[0]) * Number(input.value);
              motor.positionM[1] =
                Math.sign(motor.positionM[1]) * Number(input.value);
            }
          }
          apply(false);
        }),
    );
  }
  function preview(metrics: EngineeringMetrics) {
    while (vehicleGroup.children.length)
      vehicleGroup.remove(vehicleGroup.children[0]);
    const body = new THREE.Mesh(
      new THREE.BoxGeometry(
        definition.frame.bodyDimensionsM[1],
        definition.frame.bodyDimensionsM[2],
        definition.frame.bodyDimensionsM[0],
      ),
      new THREE.MeshStandardMaterial({ color: 0xe2a33c }),
    );
    body.userData.component = "frame";
    vehicleGroup.add(body);
    definition.motors.forEach((motor, i) => {
      const position = bodyVectorToModel(motor.positionM),
        mesh = new THREE.Mesh(
          new THREE.CylinderGeometry(
            motor.propeller.diameterM / 2,
            motor.propeller.diameterM / 2,
            0.012,
            24,
          ),
          new THREE.MeshStandardMaterial({
            color: selected === `motor-${i}` ? 0x3de3ff : 0x243b46,
            transparent: true,
            opacity: 0.75,
          }),
        );
      mesh.position.copy(position);
      mesh.quaternion.setFromUnitVectors(
        new THREE.Vector3(0, 1, 0),
        bodyVectorToModel(motor.directionBody).normalize(),
      );
      mesh.userData.component = `motor-${i}`;
      vehicleGroup.add(mesh);
      const arm = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints([
          new THREE.Vector3(),
          position,
        ]),
        new THREE.LineBasicMaterial({ color: 0x7895a2 }),
      );
      vehicleGroup.add(arm);
    });
    (definition.aeroSurfaces ?? []).forEach((surface, i) => {
      const geometry =
        surface.plane === "horizontal"
          ? new THREE.BoxGeometry(surface.spanM, 0.012, surface.chordM)
          : new THREE.BoxGeometry(0.012, surface.spanM, surface.chordM);
      const mesh = new THREE.Mesh(
        geometry,
        new THREE.MeshStandardMaterial({
          color: selected === `surface-${i}` ? 0x3de3ff : 0xd6e5ea,
        }),
      );
      mesh.position.copy(bodyVectorToModel(surface.positionM));
      vehicleGroup.add(mesh);
    });
    definition.payloads.forEach((payload, i) => {
      const mesh = new THREE.Mesh(
        new THREE.BoxGeometry(0.07, 0.05, 0.06),
        new THREE.MeshStandardMaterial({
          color: selected === `payload-${i}` ? 0x3de3ff : 0x8e6fc2,
        }),
      );
      mesh.position.copy(bodyVectorToModel(payload.positionM));
      vehicleGroup.add(mesh);
    });
    const battery = new THREE.Mesh(
      new THREE.BoxGeometry(0.14, 0.045, 0.07),
      new THREE.MeshStandardMaterial({
        color: selected === "battery" ? 0x3de3ff : 0x4ba872,
      }),
    );
    battery.position.copy(bodyVectorToModel(definition.battery.positionM));
    vehicleGroup.add(battery);
    cgMarker.position.copy(bodyVectorToModel(metrics.centerOfMassM));
    canvas.dataset.motor0 = definition.motors[0].positionM.join(",");
  }
  function metricsView(m: EngineeringMetrics) {
    const winged = (definition.vehicleClass ?? "multicopter") !== "multicopter";
    const vtol =
      definition.vehicleClass === "quadPlane" ||
      definition.vehicleClass === "tiltrotor";
    const wingMetrics = `<dt>Wing area</dt><dd>${m.wingAreaM2.toFixed(3)} m²</dd><dt>Aspect ratio</dt><dd>${m.aspectRatio.toFixed(2)}</dd><dt>Wing loading</dt><dd>${m.wingLoadingKgM2.toFixed(2)} kg/m²</dd><dt>Stall speed</dt><dd>${m.estimatedStallSpeedMps.toFixed(1)} m/s est.</dd><dt>Power / weight</dt><dd>${m.powerToWeightWKg.toFixed(0)} W/kg</dd><dt>CG / MAC</dt><dd>${(m.cgMacFraction * 100).toFixed(0)}%</dd><dt>Static margin</dt><dd>${(m.estimatedStaticMargin * 100).toFixed(0)}% est. · ${m.estimatedStaticMargin < 0.05 ? "AFT" : m.estimatedStaticMargin > 0.3 ? "FORWARD" : "STABLE"}</dd>`;
    const hoverMetrics = `<dt>Thrust / weight</dt><dd>${m.thrustToWeight.toFixed(2)}</dd><dt>Hover throttle</dt><dd>${(m.hoverThrottle * 100).toFixed(1)}%</dd><dt>Hover current</dt><dd>${m.hoverCurrentA.toFixed(1)} A</dd><dt>Hover endurance</dt><dd>${m.hoverFlightTimeMin.toFixed(1)} min est.</dd><dt>Level-hover motors</dt><dd>${m.hoverMotorOutputs.map((v) => `${(v * 100).toFixed(0)}%`).join(" / ")}</dd>`;
    const flightMetrics = `${winged ? wingMetrics : ""}${!winged || vtol ? hoverMetrics : ""}`;
    panel.querySelector("#metrics")!.innerHTML =
      `<dl><dt>Total mass</dt><dd>${m.totalMassKg.toFixed(3)} kg</dd><dt>CG X/Y/Z</dt><dd>${m.centerOfMassM.map((v) => v.toFixed(3)).join(" / ")} m</dd><dt>Inertia X/Y/Z</dt><dd>${m.inertiaKgM2.map((v) => v.toFixed(4)).join(" / ")}</dd><dt>Maximum thrust</dt><dd>${m.maxThrustN.toFixed(1)} N</dd>${flightMetrics}<dt>Maximum power</dt><dd>${m.maxPowerW.toFixed(0)} W</dd><dt>Battery energy</dt><dd>${m.batteryEnergyWh.toFixed(1)} Wh</dd></dl>`;
    panel.querySelector("#designWarnings")!.innerHTML = m.warnings.length
      ? m.warnings
          .map(
            (w) =>
              `<p class="${w.startsWith("CRITICAL") ? "critical" : "warning"}">${escapeText(w)}</p>`,
          )
          .join("")
      : `<p class="ok">No configuration warnings</p>`;
    preview(m);
  }
  function apply(refreshEditor = true) {
    try {
      sim.configure_vehicle_json(JSON.stringify(definition));
      const m = JSON.parse(
        sim.engineering_metrics_json(),
      ) as EngineeringMetrics;
      metricsView(m);
      componentList();
      if (refreshEditor) editor();
      panel.querySelector("#buildStatus")!.textContent =
        "Configuration applied to Rust physics";
    } catch (error) {
      panel.querySelector("#buildStatus")!.textContent =
        error instanceof Error ? error.message : String(error);
    }
  }
  function savedOptions() {
    panel.querySelector("#savedDesigns")!.innerHTML =
      `<option value="">—</option>${saved.map((d, i) => `<option value="${i}">${escapeText(d.name)}</option>`).join("")}`;
  }
  panel.querySelector<HTMLSelectElement>("#vehiclePreset")!.onchange = (e) => {
    definition = JSON.parse(
      FlightSimulator.preset_json((e.target as HTMLSelectElement).value),
    );
    selected = "frame";
    apply();
  };
  panel.querySelector("#addPayload")!.addEventListener("click", () => {
    definition.payloads.push({
      name: "Package",
      massKg: 0.1,
      positionM: [0, 0, 0.03],
    });
    selected = `payload-${definition.payloads.length - 1}`;
    apply();
  });
  panel.querySelector("#testFlight")!.addEventListener("click", () => {
    sim.configure_vehicle_json(JSON.stringify(definition));
    onTestFlight(structuredClone(definition));
  });
  panel.querySelector("#saveDesign")!.addEventListener("click", () => {
    const index = saved.findIndex((d) => d.name === definition.name);
    if (index >= 0) saved[index] = structuredClone(definition);
    else saved.push(structuredClone(definition));
    saveDesigns(localStorage, saved);
    savedOptions();
  });
  panel.querySelector("#duplicateDesign")!.addEventListener("click", () => {
    definition = structuredClone(definition);
    definition.name = `${definition.name} Copy`.slice(0, 80);
    apply();
  });
  panel.querySelector("#deleteDesign")!.addEventListener("click", () => {
    const select = panel.querySelector<HTMLSelectElement>("#savedDesigns")!,
      index = Number(select.value);
    if (select.value !== "" && saved[index]) {
      saved.splice(index, 1);
      saveDesigns(localStorage, saved);
      savedOptions();
    }
  });
  panel.querySelector<HTMLSelectElement>("#savedDesigns")!.onchange = (e) => {
    const d = saved[Number((e.target as HTMLSelectElement).value)];
    if (d) {
      definition = structuredClone(d);
      apply();
    }
  };
  panel.querySelector("#exportDesign")!.addEventListener("click", () => {
    const blob = new Blob([JSON.stringify(definition, null, 2)], {
        type: "application/json",
      }),
      url = URL.createObjectURL(blob),
      link = document.createElement("a");
    link.href = url;
    link.download = safeFilename(definition.name);
    link.click();
    URL.revokeObjectURL(url);
  });
  panel
    .querySelector("#importDesign")!
    .addEventListener("click", () =>
      panel.querySelector<HTMLInputElement>("#importFile")!.click(),
    );
  panel.querySelector<HTMLInputElement>("#importFile")!.onchange = async (
    e,
  ) => {
    const file = (e.target as HTMLInputElement).files?.[0];
    if (!file) return;
    try {
      if (file.size > 256000) throw new Error("File exceeds 256 KB");
      definition = parseVehicle(await file.text());
      apply();
    } catch (error) {
      panel.querySelector("#buildStatus")!.textContent =
        error instanceof Error ? error.message : String(error);
    }
  };
  savedOptions();
  apply();
  return {
    show() {
      panel.hidden = false;
      apply(false);
    },
    hide() {
      panel.hidden = true;
    },
    getDefinition() {
      return structuredClone(definition);
    },
  };
}
function setPath(target: object, path: string, value: string | number) {
  const parts = path.split(".");
  let cursor: Record<string, unknown> = target as Record<string, unknown>;
  for (const part of parts.slice(0, -1))
    cursor = cursor[part] as Record<string, unknown>;
  cursor[parts.at(-1)!] = value;
}
function escapeText(value: string) {
  const node = document.createElement("span");
  node.textContent = value;
  return node.innerHTML;
}
