// web_demo/render/overlay_cockpit.js
// HTML/CSS overlay rendered on top of the left viewport. Reads FW state via
// a caller-provided getter and updates every animation frame.
// Also exposes setControls() for the pilot-input bars panel.

const YOKE_SVG = `
  <svg class="hud-icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"
       stroke-width="1.5" stroke-linecap="round">
    <circle cx="12" cy="12" r="6"/><line x1="2" y1="12" x2="22" y2="12"/>
    <line x1="12" y1="6" x2="12" y2="18"/>
  </svg>`;

// Pilot input limits, must match input/keyboard.js
const LIMITS = { elevator: 0.4, aileron: 0.4, rudder: 0.2, thrust: 200 };

/**
 * Mount the cockpit-style overlay on the given element.
 * @param {HTMLElement} parentEl  container DOM element
 * @param {() => number[]} getFwState  returns the live 13-element FW state
 * @returns {{ setControls: (c: { elevator:number, aileron:number, rudder:number, thrust:number }) => void }}
 */
export function mountCockpitOverlay(parentEl, getFwState) {
  parentEl.innerHTML = `
    <div class="hud-rail-left"></div>
    <div class="hud-rail-right"></div>
    <div class="hud-band-top">
      <div class="hud-readout" id="cockpit-readout">
        <span data-k="airspeed">AS    -- m/s</span>
        <span data-k="alt">     ALT   -- m</span>
        <span data-k="hdg">     HDG    -- °</span>
        <span data-k="pitch">   PIT    -- °</span>
        <span data-k="roll">    ROL    -- °</span>
      </div>
    </div>
    <div class="input-panel" id="cockpit-inputs">
      <div class="input-panel-title">PILOT INPUTS &rarr; FIXED-WING</div>
      ${inputRow('ELEVATOR', 'elevator', 'rad')}
      ${inputRow('AILERON',  'aileron',  'rad')}
      ${inputRow('RUDDER',   'rudder',   'rad')}
      ${inputRow('THRUST',   'thrust',   'N')}
    </div>
    <div class="hud-band-bottom">
      <span class="hud-label">FW COCKPIT</span>
      <span>${YOKE_SVG}</span>
    </div>
  `;
  const readout = parentEl.querySelector('#cockpit-readout');
  const inputs  = parentEl.querySelector('#cockpit-inputs');

  function tick() {
    const s = getFwState();
    const speed = Math.hypot(s[3], s[4], s[5]);
    const alt   = -s[2];
    const [yaw, pitch, roll] = quatToEulerDeg(s.slice(6, 10));
    setSpan(readout, 'airspeed', `AS ${pad(speed, 4)} m/s`);
    setSpan(readout, 'alt',      `ALT ${pad(alt,  4)} m`);
    setSpan(readout, 'hdg',      `HDG ${padSigned(yaw,   4)} °`);
    setSpan(readout, 'pitch',    `PIT ${padSigned(pitch, 4)} °`);
    setSpan(readout, 'roll',     `ROL ${padSigned(roll,  4)} °`);
    requestAnimationFrame(tick);
  }
  requestAnimationFrame(tick);

  function setControls(c) {
    setBar(inputs, 'elevator', c.elevator, LIMITS.elevator, 3, 'rad', true);
    setBar(inputs, 'aileron',  c.aileron,  LIMITS.aileron,  3, 'rad', true);
    setBar(inputs, 'rudder',   c.rudder,   LIMITS.rudder,   3, 'rad', true);
    setBar(inputs, 'thrust',   c.thrust,   LIMITS.thrust,   0, 'N',   false);
  }
  return { setControls };
}

function inputRow(label, key, unit) {
  return `
    <div class="input-row">
      <span class="input-label">${label}</span>
      <div class="input-bar"><div class="input-bar-fill" data-bar="${key}"></div></div>
      <span class="input-value" data-val="${key}">-- ${unit}</span>
    </div>
  `;
}

function setBar(root, key, value, limit, digits, unit, signed) {
  const fill = root.querySelector(`[data-bar="${key}"]`);
  const val  = root.querySelector(`[data-val="${key}"]`);
  if (!fill || !val) return;
  const frac = Math.max(-1, Math.min(1, value / limit));
  if (signed) {
    if (frac >= 0) {
      fill.style.left  = '50%';
      fill.style.width = `${frac * 50}%`;
      fill.classList.remove('neg');
    } else {
      fill.style.left  = `${50 + frac * 50}%`;
      fill.style.width = `${-frac * 50}%`;
      fill.classList.add('neg');
    }
  } else {
    fill.style.left  = '0%';
    fill.style.width = `${Math.max(0, frac) * 100}%`;
    fill.classList.remove('neg');
  }
  val.textContent = `${value.toFixed(digits)} ${unit}`;
}

function setSpan(root, key, text) {
  const el = root.querySelector(`[data-k="${key}"]`);
  if (el) el.textContent = text;
}

function pad(n, w) {
  const s = Math.round(n).toString();
  return s.padStart(w);
}

function padSigned(n, w) {
  const s = Math.round(n).toString();
  if (s.startsWith('-')) return s.padStart(w + 1);
  return ('+' + s).padStart(w + 1);
}

function quatToEulerDeg(q) {
  const [w, x, y, z] = q;
  const roll  = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const sp    = Math.max(-1, Math.min(1, 2 * (w * y - z * x)));
  const pitch = Math.asin(sp);
  const yaw   = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return [yaw * 180 / Math.PI, pitch * 180 / Math.PI, roll * 180 / Math.PI];
}
