// web_demo/vitest.config.js
// Keep vitest focused on *.test.js so Playwright's *.spec.js files are not picked up.
import { defineConfig } from 'vitest/config';

export default defineConfig({
  test: {
    include: ['tests/**/*.test.js'],
  },
});
