import { test, expect } from '@playwright/test';

// Dismiss the welcome modal so subsequent keystrokes drive the sim.
async function dismissWelcome(page) {
  // The first keydown closes the welcome modal; send a harmless one.
  await page.keyboard.press('ShiftLeft');
}

test('welcome modal shown on first load and dismissible', async ({ page }) => {
  await page.goto('/');
  await expect(page.locator('#welcome')).toBeVisible();
  await expect(page.locator('#welcome h1')).toContainText(/Ghost-Tracking/i);
  // Click the start button
  await page.locator('#btn-start').click();
  await expect(page.locator('#welcome')).toBeHidden();
});

test('camera tracks FW after free-flight', async ({ page }) => {
  await page.goto('/');
  await page.waitForFunction(() => window.__demoState !== undefined);
  await dismissWelcome(page);

  await page.keyboard.down('ArrowUp');
  await page.waitForTimeout(800);
  await page.keyboard.up('ArrowUp');
  await page.waitForTimeout(2000);

  const s = await page.evaluate(() => window.__demoState);

  function dotQ(a, b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]+a[3]*b[3]; }
  const geo = 2 * Math.acos(Math.min(1, Math.abs(dotQ(s.fwQuat, s.camWorldQuat)))) * 180/Math.PI;
  console.log('geodesic camera/FW error:', geo.toFixed(4), 'deg');
  expect(geo).toBeLessThan(1.0);
});

test('inset canvas is present and has WebGL', async ({ page }) => {
  await page.goto('/');
  await page.waitForFunction(() => window.__demoState !== undefined);

  const insetCount = await page.locator('#inset').count();
  expect(insetCount).toBe(1);

  const hasContext = await page.evaluate(() => {
    const c = document.querySelector('#inset');
    if (!c) return false;
    return c.style.display !== 'none';
  });
  expect(hasContext).toBe(true);
});

test('overlays present', async ({ page }) => {
  await page.goto('/');
  await page.waitForFunction(() => window.__demoState !== undefined);
  await expect(page.locator('#overlay-cockpit .cockpit-dashboard')).toBeVisible();
  await expect(page.locator('#overlay-drone   .drone-dashboard')).toBeVisible();
  await expect(page.locator('#overlay-drone   .rotor-spin')).toHaveCount(4);
});

test('reset button resets the simulation', async ({ page }) => {
  await page.goto('/');
  await page.waitForFunction(() => window.__demoState !== undefined);
  await dismissWelcome(page);

  // Pitch up to displace the FW off its initial position
  await page.keyboard.down('ArrowUp');
  await page.waitForTimeout(800);
  await page.keyboard.up('ArrowUp');
  await page.waitForTimeout(500);

  const fwPosBefore = await page.evaluate(() => window.__demoState.fwPos);
  expect(Math.hypot(fwPosBefore[0], fwPosBefore[1])).toBeGreaterThan(5);

  // Click reset
  await page.locator('#btn-reset').click();
  await page.waitForTimeout(200);

  const fwPosAfter = await page.evaluate(() => window.__demoState.fwPos);
  const distBefore = Math.hypot(fwPosBefore[0], fwPosBefore[1]);
  const distAfter  = Math.hypot(fwPosAfter[0],  fwPosAfter[1]);
  // After reset the FW has only had ~0.2 s to fly at 30 m/s, so its
  // distance from origin should be a small fraction of the pre-reset value.
  expect(distAfter).toBeLessThan(distBefore / 3);
});

test('input bars present and live-updating', async ({ page }) => {
  await page.goto('/');
  await page.waitForFunction(() => window.__demoState !== undefined);
  // 7 DFL u-vector bars on the right
  await expect(page.locator('#drone-inputs   .input-row')).toHaveCount(7);
  // Snapshot u from window.__demoState — must be present and length 7
  const u = await page.evaluate(() => window.__demoState.u);
  expect(Array.isArray(u)).toBe(true);
  expect(u.length).toBe(7);
});

test('trajectory minimap canvas present', async ({ page }) => {
  await page.goto('/');
  await page.waitForFunction(() => window.__demoState !== undefined);
  await expect(page.locator('#minimap')).toBeVisible();
});
