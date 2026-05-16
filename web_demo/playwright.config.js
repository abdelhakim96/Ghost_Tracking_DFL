// web_demo/playwright.config.js
import { defineConfig } from '@playwright/test';
export default defineConfig({
  testDir: './tests',
  testMatch: '*.spec.js',
  use: { baseURL: 'http://localhost:8000' },
  webServer: {
    command: 'python3 -m http.server 8000',
    port: 8000,
    reuseExistingServer: true,
  },
});
