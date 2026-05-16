// web_demo/tests/keyboard.test.js
import { describe, it, expect } from 'vitest';
import { Keyboard } from '../input/keyboard.js';

describe('keyboard ramps', () => {
  it('trim elevator + zero aileron/rudder at init', () => {
    const k = new Keyboard();
    const c = k.controls();
    expect(c.elevator).toBeCloseTo(-0.022, 6);   // elevator trim for ~level flight at 30 m/s
    expect(c.aileron).toBe(0);
  });

  it('ramps elevator to limit when ArrowUp held for >= ramp time', () => {
    const k = new Keyboard();
    k._down('ArrowUp');
    for (let i = 0; i < 10; i++) k.step(0.05);   // 0.5 s total
    expect(k.controls().elevator).toBeCloseTo(-0.4, 6);   // ArrowUp = nose-up = negative
  });

  it('decays back to zero after release', () => {
    const k = new Keyboard();
    k._down('ArrowUp');
    for (let i = 0; i < 10; i++) k.step(0.05);
    k._up('ArrowUp');
    for (let i = 0; i < 20; i++) k.step(0.05);
    expect(Math.abs(k.controls().elevator - (-0.022))).toBeLessThan(0.01);
  });
});
