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
  console.log(`geodesic camera/FW error: ${geo} deg`);
  expect(geo).toBeLessThan(1.0);
});
