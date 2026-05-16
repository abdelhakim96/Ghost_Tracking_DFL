// web_demo/input/keyboard.js
const RAMP_TIME = 0.3;        // seconds to reach full deflection
const LIMITS = { elevator: 0.4, aileron: 0.4, rudder: 0.2, thrust: 200 };

export class Keyboard {
  constructor() {
    this.keys = new Set();
    this.values = { elevator: 0, aileron: 0, rudder: 0, thrust: 100 };
    this.attached = false;
  }

  attach() {
    if (this.attached) return;
    window.addEventListener('keydown', (e) => this._down(e.code));
    window.addEventListener('keyup',   (e) => this._up(e.code));
    this.attached = true;
  }

  _down(code) { this.keys.add(code); }
  _up(code)   { this.keys.delete(code); }

  step(dt) {
    const ramp = (cur, target, limit) => {
      const max = limit;
      const speed = max / RAMP_TIME;
      const delta = speed * dt * Math.sign(target - cur);
      if (Math.abs(target - cur) < Math.abs(delta)) return target;
      return cur + delta;
    };

    let elev_target = 0, ail_target = 0, rud_target = 0;
    if (this.keys.has('ArrowUp'))    elev_target = -LIMITS.elevator;
    if (this.keys.has('ArrowDown'))  elev_target =  LIMITS.elevator;
    if (this.keys.has('ArrowLeft'))  ail_target = -LIMITS.aileron;
    if (this.keys.has('ArrowRight')) ail_target =  LIMITS.aileron;
    if (this.keys.has('KeyA'))       rud_target = -LIMITS.rudder;
    if (this.keys.has('KeyD'))       rud_target =  LIMITS.rudder;

    this.values.elevator = ramp(this.values.elevator, elev_target, LIMITS.elevator);
    this.values.aileron  = ramp(this.values.aileron,  ail_target,  LIMITS.aileron);
    this.values.rudder   = ramp(this.values.rudder,   rud_target,  LIMITS.rudder);

    if (this.keys.has('KeyW')) this.values.thrust = Math.min(LIMITS.thrust, this.values.thrust + 50*dt);
    if (this.keys.has('KeyS')) this.values.thrust = Math.max(0,            this.values.thrust - 50*dt);
  }

  controls() { return { ...this.values }; }
}
