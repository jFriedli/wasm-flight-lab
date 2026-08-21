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
  expect(Number(await throttle.inputValue())).toBeGreaterThan(31);

  await page.dispatchEvent("body", "keydown", { code: "KeyD" });
  await page.waitForTimeout(350);
  await expect(page.locator("#axisTelemetry > div").first()).not.toContainText(
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

test("rendered tilt, thrust arrow and lateral motion share one frame", async ({
  page,
}) => {
  await page.goto("/");
  await page.getByText("VIEW & PHYSICS").click();
  await page.locator("#knownAttitude").selectOption("30,0");
  await page.waitForTimeout(160);
  await expect(page.locator("#angles")).toContainText("30° / 0°");
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
