import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';
import { alphaGimbal, betaGimbal } from '../sim/alpha_beta.js';

const __dirname = dirname(fileURLToPath(import.meta.url));
const samples = JSON.parse(readFileSync(resolve(__dirname, 'data/alpha_beta_samples.json'), 'utf8'));

const PARAMS = {
  Ap: 0, Aq: 0, Ar: 0,
  Ag_p: 0.01, Ag_q: 0.01, Ag_r: 0.01,
  Ix: 0.0023, Iy: 0.0023, Iz: 0.0046,
  Ig_x: 0.001, Ig_y: 0.001, Ig_z: 0.001,
  m: 0.468,
};

describe('alpha_beta', () => {
  it('matches MATLAB alpha_gimbal_func3 on 30 states', () => {
    for (const s of samples) {
      const zeta = s.state[16], xi = s.state[17];
      const a = alphaGimbal(s.state, PARAMS, zeta, xi);
      for (let i = 0; i < 7; i++) {
        expect(a[i]).toBeCloseTo(s.alpha[i], 4);
      }
    }
  });

  it('matches MATLAB beta_gimbal_func3 on 30 states', () => {
    for (const s of samples) {
      const zeta = s.state[16], xi = s.state[17];
      const b = betaGimbal(s.state, PARAMS, zeta, xi);
      // b is 7x7 row-major; beta_flat is column-major (MATLAB reshape)
      for (let r = 0; r < 7; r++) {
        for (let c = 0; c < 7; c++) {
          expect(b[r*7 + c]).toBeCloseTo(s.beta_flat[c*7 + r], 4);
        }
      }
    }
  });
});
