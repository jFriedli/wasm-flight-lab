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
  const throttle = page.locator("#throttle");
  await throttle.evaluate((element: HTMLInputElement) => {
    element.value = "42";
    element.dispatchEvent(new Event("input", { bubbles: true }));
  });
  await page.waitForTimeout(1100);
  expect(
    Number.parseFloat(await page.locator("#alt").innerText()),
  ).toBeGreaterThan(0.2);
  await throttle.evaluate((element: HTMLInputElement) => {
    element.value = "12";
    element.dispatchEvent(new Event("input", { bubbles: true }));
  });
  await expect
    .poll(
      async () => Number.parseFloat(await page.locator("#alt").innerText()),
      { timeout: 6000 },
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
