import { expect, test } from "@playwright/test";

test("WASM simulation and controller start", async ({ page }) => {
  await page.goto("/");
  await expect(page.getByText("WASM FLIGHT LAB")).toBeVisible();
  await expect(page.getByText(/Simulation running locally/)).toBeVisible();
  await expect(page.locator("#alt")).not.toHaveText("—");
  await expect(page.locator("#axisTelemetry")).toContainText("Roll");
  await expect(page.locator("#axisTelemetry")).toContainText("M4");
});

test("Beginner Quad performs a controlled vertical takeoff and landing", async ({
  page,
}) => {
  await page.goto("/");
  await page.locator("#spawn").selectOption("ground");
  await page.getByRole("button", { name: "Reset aircraft" }).click();
  await page.getByRole("button", { name: "ANGLE", exact: true }).click();
  const throttle = page.locator("#throttle");
  await throttle.evaluate((element: HTMLInputElement) => {
    element.value = "50";
    element.dispatchEvent(new Event("input", { bubbles: true }));
  });
  await page.waitForTimeout(1600);
  expect(
    Number.parseFloat(await page.locator("#alt").innerText()),
  ).toBeGreaterThan(0.2);
  await throttle.evaluate((element: HTMLInputElement) => {
    element.value = "0";
    element.dispatchEvent(new Event("input", { bubbles: true }));
  });
  await expect
    .poll(
      async () => Number.parseFloat(await page.locator("#alt").innerText()),
      { timeout: 10000 },
    )
    .toBeLessThan(0.15);
  await expect(page.locator("#angles")).toContainText("0° / 0°");
});

test("flight controls, modes, cameras, PID and pause are interactive", async ({
  page,
}) => {
  await page.goto("/");
  const throttle = page.locator("#throttle");
  await page.dispatchEvent("body", "keydown", { code: "KeyW" });
  await page.waitForTimeout(350);
  await page.dispatchEvent("body", "keyup", { code: "KeyW" });
  const heldThrottle = Number(await throttle.inputValue());
  expect(heldThrottle).toBeGreaterThan(31);
  await page.waitForTimeout(250);
  expect(Number(await throttle.inputValue())).toBe(heldThrottle);

  await page.dispatchEvent("body", "keydown", { code: "KeyD" });
  await page.waitForTimeout(350);
  await expect(page.locator("#axisTelemetry > div").nth(2)).not.toContainText(
    "T 0°/s",
  );
  await page.dispatchEvent("body", "keyup", { code: "KeyD" });

  await page.getByRole("button", { name: "ANGLE" }).click();
  await expect(page.locator("#modeReadout")).toHaveText("ANGLE");
  for (const name of ["FPV", "FREE", "CHASE"]) {
    await page.getByRole("button", { name, exact: true }).click();
    await expect(page.getByRole("button", { name, exact: true })).toHaveClass(
      /active/,
    );
  }

  const rollP = page.locator('[data-pid="roll-0"]');
  await rollP.fill("0.04");
  await rollP.blur();
  await expect(rollP).toHaveValue("0.04");

  await page.getByRole("button", { name: "Pause", exact: true }).click();
  await expect(page.locator("#simState")).toHaveText("PAUSED");
  await page.getByRole("button", { name: "Resume", exact: true }).click();
  await expect(page.locator("#simState")).toHaveText("RUNNING");
});

test("Mode-2 mouse stick is bounded, centered, and explicitly captured", async ({
  page,
}) => {
  await page.goto("/");
  await page.getByRole("button", { name: "ENABLE MOUSE FLIGHT" }).click();
  await expect(page.locator("#mouseStatus")).toContainText(
    "MOUSE FLIGHT ACTIVE",
  );
  await page.waitForTimeout(100);
  await page.locator("#mouseReturn").fill("0.3");
  await page.evaluate(() =>
    document.dispatchEvent(
      new MouseEvent("mousemove", { movementX: 60, movementY: -50 }),
    ),
  );
  await page.waitForTimeout(50);
  const stick = (await page
    .locator("#viewport")
    .getAttribute("data-virtual-stick"))!
    .split(",")
    .map(Number);
  expect(stick[0]).toBeGreaterThan(0);
  expect(stick[1]).toBeLessThan(0);
  const displaced = await page.locator("#virtualStick i").getAttribute("style");
  expect(displaced).not.toContain("translate(0px, 0px)");
  await page.locator("#mouseReturn").fill("4");
  await page.waitForTimeout(400);
  const centered = (await page
    .locator("#viewport")
    .getAttribute("data-virtual-stick"))!
    .split(",")
    .map(Number);
  expect(Math.abs(centered[0])).toBeLessThan(0.03);
  expect(Math.abs(centered[1])).toBeLessThan(0.03);
  await page.evaluate(() => document.exitPointerLock());
  await expect(page.locator("#mouseStatus")).toContainText("Click to capture");
});

test("rendered tilt, thrust arrow and lateral motion share one frame", async ({
  page,
}) => {
  await page.goto("/");
  await page.getByText("VIEW & PHYSICS").click();
  await page.getByRole("button", { name: "Reset aircraft" }).click();
  await page.locator("#knownAttitude").selectOption("30,0");
  await page.waitForTimeout(100);
  const attitude = (await page.locator("#angles").innerText())
    .match(/-?\d+/g)!
    .map(Number);
  expect(attitude[0]).toBeGreaterThan(25);
  expect(attitude[0]).toBeLessThan(35);
  expect(Math.abs(attitude[1])).toBeLessThan(8);
  const vectors = await page.locator("#viewport").evaluate((element) => ({
    thrust: element.dataset.thrustDirection!.split(",").map(Number),
    model: element.dataset.modelThrustDirection!.split(",").map(Number),
    velocity: element.dataset.velocityDirection!.split(",").map(Number),
  }));
  expect(vectors.thrust[0]).toBeGreaterThan(0.25);
  expect(vectors.model[0]).toBeGreaterThan(0.25);
  expect(vectors.velocity[0]).toBeGreaterThan(0.1);
  const alignment = vectors.thrust.reduce(
    (sum, value, index) => sum + value * vectors.model[index],
    0,
  );
  expect(alignment).toBeGreaterThan(0.999);
});

test("Alpine terrain and navigation instruments initialize", async ({
  page,
}) => {
  const errors: string[] = [];
  page.on("pageerror", (error) => errors.push(error.message));
  page.on("console", (message) => {
    if (message.type() === "error") errors.push(message.text());
  });
  await page.goto("/");
  await expect(page.locator("#environment")).toHaveValue("alpine");
  await expect(page.locator("#viewport")).toHaveAttribute(
    "data-environment",
    "alpine",
  );
  await expect(page.locator("#map")).toBeVisible();
  await expect(page.locator("#heading")).toContainText(/°/);
  await expect(page.locator("#home")).toContainText("HOME");
  await page.locator("#environment").selectOption("test");
  await expect(page.locator("#viewport")).toHaveAttribute(
    "data-environment",
    "test",
  );
  await page.locator("#environment").selectOption("alpine");
  expect(errors).toEqual([]);
});

test("authoritative weather changes local airflow and remains finite", async ({
  page,
}) => {
  const errors: string[] = [];
  page.on("pageerror", (error) => errors.push(error.message));
  page.on("console", (message) => {
    if (message.type() === "error") errors.push(message.text());
  });
  await page.goto("/");
  await page.getByText("ATMOSPHERE", { exact: true }).click();
  await page.locator("#weather").selectOption("breeze");
  await expect(page.locator("#windReadout")).toContainText("BREEZE");
  await expect
    .poll(async () => {
      const wind = (await page.locator("#viewport").getAttribute("data-wind"))!
        .split(",")
        .map(Number);
      return Math.hypot(...wind);
    })
    .toBeGreaterThan(3);
  const ground = Number.parseFloat(await page.locator("#speed").innerText());
  const air = Number.parseFloat(await page.locator("#airspeed").innerText());
  expect(air).toBeGreaterThan(ground + 2);
  await page.locator("#airflow").check();
  expect(errors).toEqual([]);
});

test("workshop edit reaches preview, persistence, and authoritative flight definition", async ({
  page,
}) => {
  await page.goto("/");
  await page.getByRole("button", { name: "BUILD", exact: true }).click();
  await expect(page.locator("#buildView")).toBeVisible();
  await page.getByRole("button", { name: "Motor 1" }).click();
  const positionX = page.locator('[data-path="motors.0.positionM.0"]');
  await positionX.fill("0.32");
  await positionX.blur();
  await expect
    .poll(() => page.locator("#buildCanvas").getAttribute("data-motor0"))
    .toContain("0.32");
  await page.getByRole("button", { name: "Save", exact: true }).click();
  expect(
    await page.evaluate(() => localStorage.getItem("flightlab.vehicles.v1")),
  ).toContain("0.32");
  await page.getByRole("button", { name: "TEST FLIGHT" }).click();
  await expect(page.locator("#viewport")).toBeVisible();
  const runtime = await page
    .locator("#viewport")
    .getAttribute("data-vehicle-definition");
  expect(JSON.parse(runtime!).motors[0].positionM[0]).toBe(0.32);
  await page.getByRole("button", { name: "BUILD", exact: true }).click();
  await expect(positionX).toHaveValue("0.32");
});

test("Fixed Wing Trainer builds, accelerates, and takes off from the runway", async ({
  page,
}) => {
  const errors: string[] = [];
  page.on("pageerror", (error) => errors.push(error.message));
  page.on("console", (message) => {
    if (message.type() === "error") errors.push(message.text());
  });
  await page.goto("/");
  await page.getByRole("button", { name: "BUILD", exact: true }).click();
  await page.locator("#vehiclePreset").selectOption("trainer");
  await expect(page.getByRole("button", { name: "Left Wing" })).toBeVisible();
  await expect(page.locator("#metrics")).toContainText("Wing loading");
  await page.getByRole("button", { name: "Left Wing" }).click();
  await expect(
    page.locator('[data-path="aeroSurfaces.0.stallAngleDeg"]'),
  ).toHaveValue("15");
  await page.getByRole("button", { name: "TEST FLIGHT" }).click();
  await expect(page.locator("#modeReadout")).toHaveText("TRAINER MANUAL");
  await page.locator("#spawn").selectOption("ground");
  await page.getByRole("button", { name: "Reset aircraft" }).click();
  await page.locator("#throttle").evaluate((element: HTMLInputElement) => {
    element.value = "100";
    element.dispatchEvent(new Event("input", { bubbles: true }));
  });
  await expect
    .poll(
      async () => Number.parseFloat(await page.locator("#speed").innerText()),
      { timeout: 10000 },
    )
    .toBeGreaterThan(6);
  await page.dispatchEvent("body", "keydown", { code: "ArrowDown" });
  await expect
    .poll(
      async () => Number.parseFloat(await page.locator("#alt").innerText()),
      { timeout: 10000 },
    )
    .toBeGreaterThan(0.1);
  await page.dispatchEvent("body", "keyup", { code: "ArrowDown" });
  await expect(page.locator("#aoa")).toBeVisible();
  await page.waitForTimeout(500);
  await page.getByRole("button", { name: "ENABLE MOUSE FLIGHT" }).click();
  await page.evaluate(() =>
    document.dispatchEvent(
      new MouseEvent("mousemove", { movementX: 25, movementY: 0 }),
    ),
  );
  await page.waitForTimeout(250);
  const rates = (await page.locator("#viewport").getAttribute("data-rates"))!
    .split(",")
    .map(Number);
  expect(rates.every(Number.isFinite)).toBe(true);
  expect(Math.hypot(...rates)).toBeLessThan(3);
  await page.evaluate(() => document.exitPointerLock());
  expect(errors).toEqual([]);
});

test("QuadPlane and tiltrotor expose physical transition telemetry", async ({
  page,
}) => {
  const errors: string[] = [];
  page.on("pageerror", (error) => errors.push(error.message));
  page.on("console", (message) => {
    if (message.type() === "error") errors.push(message.text());
  });
  await page.goto("/");
  for (const preset of ["quadplane", "tiltrotor"]) {
    await page.getByRole("button", { name: "BUILD", exact: true }).click();
    await page.locator("#vehiclePreset").selectOption(preset);
    await expect(page.locator("#metrics")).toContainText("Hover throttle");
    await page.getByRole("button", { name: "TEST FLIGHT" }).click();
    await page.locator("#spawn").selectOption("air");
    await page.getByRole("button", { name: "Reset aircraft" }).click();
    await expect(page.locator("#vtolRegime")).toContainText("HOVER");
    await page.locator("#transition").fill("50");
    await page.locator("#transition").dispatchEvent("input");
    await expect
      .poll(async () => await page.locator("#vtolRegime").innerText(), {
        timeout: 5000,
      })
      .toContain("TRANSITION");
    await expect(page.locator("#vtolForces")).toContainText("vertical");
  }
  expect(errors).toEqual([]);
});
