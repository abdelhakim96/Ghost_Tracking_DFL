import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';
import { fwDerivative, edge540Params } from '../sim/fw_dynamics.js';

const __dirname = dirname(fileURLToPath(import.meta.url));
const data = JSON.parse(readFileSync(resolve(__dirname, 'data/fw_dynamics_samples.json'), 'utf8'));

describe('fw_dynamics', () => {
  it('matches MATLAB fw_6dof_quat on 30 random states', () => {
    for (const s of data.samples) {
      const xdot = fwDerivative(s.state, s.thrust, s.elevator, s.aileron, s.rudder, data.params);
      for (let i = 0; i < 13; i++) {
        expect(xdot[i]).toBeCloseTo(s.xdot[i], 6);
      }
    }
  });

  it('edge540Params returns sane numbers', () => {
    const p = edge540Params();
    expect(p.m).toBe(290);
    expect(p.S).toBe(9.1);
    expect(p.CL_alpha).toBeCloseTo(5.7, 6);
  });
});
