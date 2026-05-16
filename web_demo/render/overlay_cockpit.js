// web_demo/render/overlay_cockpit.js
// Big cockpit-style overlay: canopy frame top + dashboard with SVG instruments
// across the bottom 33% of the left viewport. Updates live from FW state +
// pilot controls.

const LIMITS = { elevator: 0.4, aileron: 0.4, rudder: 0.2, thrust: 200 };

/**
 * Mount the cockpit overlay.
 * @param {HTMLElement} parentEl
 * @param {() => number[]} getFwState
 * @returns {{ setControls: (c: { elevator:number, aileron:number, rudder:number, thrust:number }) => void }}
 */
export function mountCockpitOverlay(parentEl, getFwState) {
  parentEl.innerHTML = `
    <!-- Canopy frame: arched top, side posts -->
    <div class="cockpit-canopy"></div>

    <!-- Top label band -->
    <div class="cockpit-top-band">
      <span class="hud-label">FW COCKPIT &mdash; PILOT</span>
      <span class="hud-label" id="cockpit-readout-text">AS -- ALT -- HDG --</span>
    </div>

    <!-- Bottom dashboard ~33% height -->
    <div class="cockpit-dashboard">
      <div class="dash-row">
        ${svgArtificialHorizon()}
        ${svgAirspeed()}
        ${svgAltimeter()}
        ${svgCompass()}
        ${svgJoystick()}
        ${svgThrottle()}
        ${svgRudderBar()}
      </div>
    </div>
  `;

  const readoutText = parentEl.querySelector('#cockpit-readout-text');
  let lastFw = null;

  function tick() {
    const s = getFwState();
    lastFw = s;
    const speed = Math.hypot(s[3], s[4], s[5]);
    const alt   = -s[2];
    const [yaw, pitch, roll] = quatToEulerDeg(s.slice(6, 10));
    readoutText.textContent =
      `AS ${pad(speed, 4)} m/s   ALT ${pad(alt, 4)} m   HDG ${padSigned(yaw, 4)}°` +
      `   PIT ${padSigned(pitch, 4)}°   ROL ${padSigned(roll, 4)}°`;
    // Update instruments
    updateArtificialHorizon(parentEl, pitch, roll);
    updateAirspeed(parentEl, speed);
    updateAltimeter(parentEl, alt);
    updateCompass(parentEl, yaw);
    requestAnimationFrame(tick);
  }
  requestAnimationFrame(tick);

  function setControls(c) {
    updateJoystick(parentEl, c.elevator, c.aileron);
    updateThrottle(parentEl, c.thrust);
    updateRudderBar(parentEl, c.rudder);
  }
  return { setControls };
}

// ---------- SVG instrument HTML ----------------------------------------------

function svgArtificialHorizon() {
  return `<div class="instr instr-horizon">
    <svg viewBox="-50 -50 100 100">
      <defs>
        <clipPath id="hzClip"><circle cx="0" cy="0" r="42"/></clipPath>
      </defs>
      <circle cx="0" cy="0" r="46" fill="#0a0e16" stroke="#6cc" stroke-width="1.5"/>
      <g clip-path="url(#hzClip)" id="hzRoll">
        <g id="hzPitch">
          <rect x="-100" y="-100" width="200" height="100" fill="#4a8acc"/>
          <rect x="-100" y="0"    width="200" height="100" fill="#7a5a3a"/>
          <line x1="-50" y1="0" x2="50" y2="0" stroke="#fff" stroke-width="1"/>
          <g stroke="#fff" stroke-width="0.6" opacity="0.7">
            <line x1="-15" y1="-20" x2="15" y2="-20"/>
            <line x1="-15" y1="20"  x2="15" y2="20"/>
            <line x1="-10" y1="-40" x2="10" y2="-40"/>
            <line x1="-10" y1="40"  x2="10" y2="40"/>
          </g>
        </g>
      </g>
      <!-- Aircraft symbol (fixed) -->
      <g stroke="#ff0" stroke-width="2" fill="none">
        <line x1="-22" y1="0" x2="-8" y2="0"/>
        <line x1="8"   y1="0" x2="22" y2="0"/>
        <circle cx="0" cy="0" r="2.5" fill="#ff0"/>
      </g>
      <text x="0" y="38" text-anchor="middle" fill="#cfe" font-size="6">ATTITUDE</text>
    </svg>
  </div>`;
}

function svgAirspeed() {
  return `<div class="instr instr-airspeed">
    <svg viewBox="-50 -50 100 100">
      <circle cx="0" cy="0" r="46" fill="#0a0e16" stroke="#6cc" stroke-width="1.5"/>
      <g stroke="#cfe" stroke-width="1" fill="none">
        ${tickMarks(0, 360, 30, 36, 44)}
      </g>
      <g fill="#cfe" font-size="6" text-anchor="middle">
        ${labelMarks(0, 360, 60, 28, ['0','20','40','60','80','100'])}
      </g>
      <line id="asNeedle" x1="0" y1="0" x2="0" y2="-36" stroke="#ff0" stroke-width="2.4"/>
      <circle cx="0" cy="0" r="3" fill="#ff0"/>
      <text x="0" y="38" text-anchor="middle" fill="#cfe" font-size="6">AIRSPEED m/s</text>
    </svg>
  </div>`;
}

function svgAltimeter() {
  return `<div class="instr instr-alt">
    <svg viewBox="-50 -50 100 100">
      <circle cx="0" cy="0" r="46" fill="#0a0e16" stroke="#6cc" stroke-width="1.5"/>
      <text id="altText" x="0" y="2" text-anchor="middle" fill="#ff0" font-size="14" font-family="monospace" font-weight="bold">-- m</text>
      <text x="0" y="-22" text-anchor="middle" fill="#cfe" font-size="6">ALTITUDE</text>
      <line id="altNeedle" x1="0" y1="0" x2="0" y2="-32" stroke="#ff0" stroke-width="2"/>
      <text x="0" y="38" text-anchor="middle" fill="#cfe" font-size="6">m AGL</text>
    </svg>
  </div>`;
}

function svgCompass() {
  return `<div class="instr instr-compass">
    <svg viewBox="-50 -50 100 100">
      <circle cx="0" cy="0" r="46" fill="#0a0e16" stroke="#6cc" stroke-width="1.5"/>
      <g id="cmpRing">
        <g fill="#cfe" font-size="7" text-anchor="middle" font-weight="bold">
          <text x="0"   y="-32">N</text>
          <text x="32"  y="3">E</text>
          <text x="0"   y="38">S</text>
          <text x="-32" y="3">W</text>
        </g>
        <g stroke="#cfe" stroke-width="0.8" fill="none">
          ${tickMarks(0, 360, 30, 38, 44)}
        </g>
      </g>
      <polygon points="0,-44 -3,-36 3,-36" fill="#ff0"/>
      <circle cx="0" cy="0" r="2" fill="#ff0"/>
      <text x="0" y="38" text-anchor="middle" fill="#cfe" font-size="6">HEADING</text>
    </svg>
  </div>`;
}

function svgJoystick() {
  return `<div class="instr instr-stick">
    <svg viewBox="-50 -50 100 100">
      <rect x="-44" y="-44" width="88" height="88" rx="6" fill="#0a0e16" stroke="#6cc" stroke-width="1.5"/>
      <line x1="-44" y1="0" x2="44" y2="0" stroke="#3a4a5a"/>
      <line x1="0" y1="-44" x2="0" y2="44" stroke="#3a4a5a"/>
      <text x="0" y="-34" text-anchor="middle" fill="#cfe" font-size="6">PITCH/ROLL</text>
      <text x="0" y="40"  text-anchor="middle" fill="#cfe" font-size="5">JOYSTICK</text>
      <line id="stickHandle" x1="0" y1="0" x2="0" y2="0" stroke="#ff0" stroke-width="2.5"/>
      <circle id="stickDot" cx="0" cy="0" r="5" fill="#ff0"/>
    </svg>
  </div>`;
}

function svgThrottle() {
  return `<div class="instr instr-thr">
    <svg viewBox="-30 -50 60 100">
      <rect x="-26" y="-44" width="52" height="88" rx="6" fill="#0a0e16" stroke="#6cc" stroke-width="1.5"/>
      <rect x="-6" y="-40" width="12" height="80" rx="3" fill="#1a2230" stroke="#6cc" stroke-width="0.6"/>
      <g stroke="#cfe" stroke-width="0.6">
        <line x1="-16" y1="-40" x2="-9" y2="-40"/>
        <line x1="-16" y1="-20" x2="-9" y2="-20"/>
        <line x1="-16" y1="0"   x2="-9" y2="0"/>
        <line x1="-16" y1="20"  x2="-9" y2="20"/>
        <line x1="-16" y1="40"  x2="-9" y2="40"/>
      </g>
      <rect id="thrLever" x="-10" y="0" width="20" height="6" rx="2" fill="#ff0"/>
      <text x="0" y="-46" text-anchor="middle" fill="#cfe" font-size="6">THROTTLE</text>
      <text id="thrText" x="0" y="48" text-anchor="middle" fill="#cfe" font-size="6">-- N</text>
    </svg>
  </div>`;
}

function svgRudderBar() {
  return `<div class="instr instr-rud">
    <svg viewBox="-50 -30 100 60">
      <rect x="-44" y="-26" width="88" height="52" rx="6" fill="#0a0e16" stroke="#6cc" stroke-width="1.5"/>
      <line x1="0" y1="-20" x2="0" y2="20" stroke="#3a4a5a"/>
      <rect id="rudFill" x="0" y="-8" width="0" height="16" rx="2" fill="#ff0"/>
      <text x="0" y="-22" text-anchor="middle" fill="#cfe" font-size="6">RUDDER</text>
      <text id="rudText" x="0" y="24" text-anchor="middle" fill="#cfe" font-size="6">-- rad</text>
    </svg>
  </div>`;
}

// ---------- Instrument update functions --------------------------------------

function updateArtificialHorizon(root, pitchDeg, rollDeg) {
  const roll = root.querySelector('#hzRoll');
  const pitch = root.querySelector('#hzPitch');
  if (roll && pitch) {
    roll.setAttribute('transform', `rotate(${-rollDeg})`);
    // 100 px clip; ~1 px per 1.5 deg pitch
    const py = Math.max(-50, Math.min(50, pitchDeg / 1.5));
    pitch.setAttribute('transform', `translate(0 ${py})`);
  }
}

function updateAirspeed(root, speed) {
  const needle = root.querySelector('#asNeedle');
  if (!needle) return;
  const speed_clamped = Math.max(0, Math.min(100, speed));
  const angle = -150 + (speed_clamped / 100) * 300;     // -150 to +150 deg
  needle.setAttribute('transform', `rotate(${angle})`);
}

function updateAltimeter(root, alt) {
  const txt = root.querySelector('#altText');
  const needle = root.querySelector('#altNeedle');
  if (txt) txt.textContent = `${alt.toFixed(0)} m`;
  if (needle) {
    const a = (alt % 1000) / 1000;     // 1 needle rev per 1000 m
    needle.setAttribute('transform', `rotate(${a * 360})`);
  }
}

function updateCompass(root, yawDeg) {
  const ring = root.querySelector('#cmpRing');
  if (ring) ring.setAttribute('transform', `rotate(${-yawDeg})`);
}

function updateJoystick(root, elevator, aileron) {
  // ArrowUp = nose-up = negative elevator => dot moves UP (negative y in SVG)
  const dot = root.querySelector('#stickDot');
  const handle = root.querySelector('#stickHandle');
  if (!dot || !handle) return;
  const x = (aileron / LIMITS.aileron) * 40;
  const y = (elevator / LIMITS.elevator) * 40;
  dot.setAttribute('cx', x.toString());
  dot.setAttribute('cy', y.toString());
  handle.setAttribute('x2', x.toString());
  handle.setAttribute('y2', y.toString());
}

function updateThrottle(root, thrust) {
  const lever = root.querySelector('#thrLever');
  const txt   = root.querySelector('#thrText');
  if (!lever || !txt) return;
  const frac = Math.max(0, Math.min(1, thrust / LIMITS.thrust));
  // 0 -> bottom (+40); 1 -> top (-40)
  const y = 40 - frac * 80 - 3;
  lever.setAttribute('y', y.toString());
  txt.textContent = `${thrust.toFixed(0)} N`;
}

function updateRudderBar(root, rudder) {
  const fill = root.querySelector('#rudFill');
  const txt  = root.querySelector('#rudText');
  if (!fill || !txt) return;
  const frac = Math.max(-1, Math.min(1, rudder / LIMITS.rudder));
  const w = Math.abs(frac) * 40;
  if (frac >= 0) {
    fill.setAttribute('x', '0');
  } else {
    fill.setAttribute('x', (-w).toString());
  }
  fill.setAttribute('width', w.toString());
  txt.textContent = `${rudder.toFixed(3)} rad`;
}

// ---------- helpers ----------------------------------------------------------

function tickMarks(start, end, step, r0, r1) {
  let out = '';
  for (let a = start; a < end; a += step) {
    const rad = a * Math.PI / 180;
    const sx = r0 * Math.sin(rad), sy = -r0 * Math.cos(rad);
    const ex = r1 * Math.sin(rad), ey = -r1 * Math.cos(rad);
    out += `<line x1="${sx.toFixed(1)}" y1="${sy.toFixed(1)}" x2="${ex.toFixed(1)}" y2="${ey.toFixed(1)}"/>`;
  }
  return out;
}

function labelMarks(start, end, step, r, labels) {
  let out = '', i = 0;
  for (let a = start; a < end; a += step) {
    const rad = a * Math.PI / 180;
    const x = r * Math.sin(rad), y = -r * Math.cos(rad) + 2;
    out += `<text x="${x.toFixed(1)}" y="${y.toFixed(1)}">${labels[i] || ''}</text>`;
    i++;
  }
  return out;
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
