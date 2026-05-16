// web_demo/tests/quat.test.js
import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';
import { quatMul, quatConj, quatToR } from '../sim/quat.js';

const __dirname = dirname(fileURLToPath(import.meta.url));
const samples = JSON.parse(readFileSync(resolve(__dirname, 'data/quat_samples.json'), 'utf8'));

describe('quat', () => {
  it('matches MATLAB quat_mul on 30 samples', () => {
    for (const s of samples) {
      const got = quatMul(s.a, s.b);
      for (let i = 0; i < 4; i++) {
        expect(got[i]).toBeCloseTo(s.a_mul_b[i], 10);
      }
    }
  });

  it('matches MATLAB quat_conj on 30 samples', () => {
    for (const s of samples) {
      const got = quatConj(s.a);
      for (let i = 0; i < 4; i++) {
        expect(got[i]).toBeCloseTo(s.a_conj[i], 10);
      }
    }
  });

  it('quat_to_R produces a proper rotation matrix (R R^T = I, det = +1)', () => {
    for (const s of samples) {
      const R = quatToR(s.a);
      // R R^T = I (orthogonality)
      for (let i = 0; i < 3; i++) {
        for (let j = 0; j < 3; j++) {
          const dot = R[3*i]*R[3*j] + R[3*i+1]*R[3*j+1] + R[3*i+2]*R[3*j+2];
          expect(dot).toBeCloseTo(i === j ? 1 : 0, 10);
        }
      }
      // det(R) = +1 (proper, not reflection)
      const det = R[0]*(R[4]*R[8] - R[5]*R[7])
                - R[1]*(R[3]*R[8] - R[5]*R[6])
                + R[2]*(R[3]*R[7] - R[4]*R[6]);
      expect(det).toBeCloseTo(1, 10);
    }
  });
});
