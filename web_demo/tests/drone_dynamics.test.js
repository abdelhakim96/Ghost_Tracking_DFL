// web_demo/tests/drone_dynamics.test.js
import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';
import { droneDerivative } from '../sim/drone_dynamics.js';

const __dirname = dirname(fileURLToPath(import.meta.url));
const samples = JSON.parse(readFileSync(resolve(__dirname, 'data/drone_dynamics_samples.json'), 'utf8'));

describe('drone_dynamics', () => {
  it('matches MATLAB quadrotor_dynamics_realtime3 on 15 samples', () => {
    for (const s of samples) {
      const sdot = droneDerivative({
        droneState: s.drone_state, fwState: s.fw_state,
        xd: s.xd, vd: s.vd, ad: s.ad, jd: s.jd, sd: s.sd,
        gains: s.gains,
      });
      for (let i = 0; i < 18; i++) {
        expect(sdot[i]).toBeCloseTo(s.sdot[i], 3);
      }
    }
  });
});
