import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';
import { dflController } from '../sim/dfl_controller.js';

const __dirname = dirname(fileURLToPath(import.meta.url));
const samples = JSON.parse(readFileSync(resolve(__dirname, 'data/dfl_controller_samples.json'), 'utf8'));

describe('dfl_controller', () => {
  it('matches MATLAB dfl_controller3 on 15 samples', () => {
    for (const s of samples) {
      const u = dflController({
        droneState: s.drone_state, fwState: s.fw_state,
        xd: s.xd, vd: s.vd, ad: s.ad, jd: s.jd, sd: s.sd,
        psid: s.psid, gains: s.gains,
      });
      for (let i = 0; i < 7; i++) {
        expect(u[i]).toBeCloseTo(s.u[i], 3);
      }
    }
  });
});
