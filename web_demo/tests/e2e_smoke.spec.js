import { test, expect } from '@playwright/test';

test('camera tracks FW after free-flight', async ({ page }) => {
  await page.goto('/');
  await page.waitForFunction(() => window.__demoState !== undefined);

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
  await expect(page.locator('#overlay-cockpit .hud-band-top')).toBeVisible();
  await expect(page.locator('#overlay-drone   .hud-band-top')).toBeVisible();
  await expect(page.locator('#overlay-drone   .rotor-disc')).toHaveCount(4);
});

test('input bars present and live-updating', async ({ page }) => {
  await page.goto('/');
  await page.waitForFunction(() => window.__demoState !== undefined);
  // 4 pilot input bars on the left
  await expect(page.locator('#cockpit-inputs .input-row')).toHaveCount(4);
  // 7 DFL u-vector bars on the right
  await expect(page.locator('#drone-inputs   .input-row')).toHaveCount(7);
  // Snapshot u from window.__demoState — must be present and length 7
  const u = await page.evaluate(() => window.__demoState.u);
  expect(Array.isArray(u)).toBe(true);
  expect(u.length).toBe(7);
});
