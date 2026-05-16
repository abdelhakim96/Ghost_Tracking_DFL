// web_demo/render/overlay_drone.js
// Big drone-style overlay: visible quad frame at the bottom 33% with rotor
// arms coming from corners + gimbal mount + DFL u-vector bars.

const U_LIMITS = {
  T_ddot:    50,
  tau_phi:    1,
  tau_theta:  1,
  tau_psi:    1,
  dphi_g:     3,
  dtheta_g:   3,
  dpsi_g:     3,
};

/**
 * Mount the drone overlay.
 * @param {HTMLElement} parentEl
 * @param {() => number[]} getDroneState
 * @returns {{ setU: (u: number[]) => void }}
 */
export function mountDroneOverlay(parentEl, getDroneState) {
  parentEl.innerHTML = `
    <!-- Top label band -->
    <div class="drone-top-band">
      <span class="hud-label">DRONE GIMBAL CAMERA &mdash; 3-AXIS</span>
      <span class="hud-label" id="drone-readout-text">PHI -- THE -- PSI --</span>
    </div>

    <!-- Foreground quad frame, rotor arms + gimbal bracket -->
    <svg class="drone-frame" viewBox="0 0 100 100" preserveAspectRatio="xMidYMid slice">
      <defs>
        <radialGradient id="rotorGlow">
          <stop offset="0%"  stop-color="rgba(200,220,240,0.7)"/>
          <stop offset="80%" stop-color="rgba(200,220,240,0)"/>
        </radialGradient>
      </defs>
      <!-- 4 arms angled from corners toward centre-bottom -->
      <g stroke="#888" stroke-width="0.7" stroke-linecap="round" fill="none">
        <line x1="0"   y1="100" x2="35" y2="78"/>
        <line x1="100" y1="100" x2="65" y2="78"/>
        <line x1="0"   y1="55"  x2="35" y2="78"/>
        <line x1="100" y1="55"  x2="65" y2="78"/>
      </g>
      <!-- Body -->
      <rect x="42" y="74" width="16" height="10" rx="2" fill="#222831" stroke="#888" stroke-width="0.5"/>
      <!-- Gimbal mount bracket (frame attached below body) -->
      <path d="M44,84 L44,89 Q44,92 47,92 L53,92 Q56,92 56,89 L56,84"
            fill="none" stroke="#aaa" stroke-width="0.6"/>
      <!-- 4 rotor discs (CSS animated via SMIL would be heavier; use static blur with disc and tick) -->
      <g>
        <circle cx="0"   cy="100" r="9" fill="url(#rotorGlow)"/>
        <circle cx="100" cy="100" r="9" fill="url(#rotorGlow)"/>
        <circle cx="0"   cy="55"  r="9" fill="url(#rotorGlow)"/>
        <circle cx="100" cy="55"  r="9" fill="url(#rotorGlow)"/>
        <circle cx="0"   cy="100" r="9" fill="none" stroke="#9ab" stroke-width="0.3" stroke-dasharray="1,1.5" class="rotor-spin"/>
        <circle cx="100" cy="100" r="9" fill="none" stroke="#9ab" stroke-width="0.3" stroke-dasharray="1,1.5" class="rotor-spin"/>
        <circle cx="0"   cy="55"  r="9" fill="none" stroke="#9ab" stroke-width="0.3" stroke-dasharray="1,1.5" class="rotor-spin"/>
        <circle cx="100" cy="55"  r="9" fill="none" stroke="#9ab" stroke-width="0.3" stroke-dasharray="1,1.5" class="rotor-spin"/>
      </g>
    </svg>

    <!-- Dashboard at bottom: gimbal indicators + u-vector bars -->
    <div class="drone-dashboard">
      <div class="dash-row">
        ${svgGimbalRing('phi_g',   'PHI_g')}
        ${svgGimbalRing('theta_g', 'THE_g')}
        ${svgGimbalRing('psi_g',   'PSI_g')}
        <div class="u-bars" id="drone-inputs">
          <div class="u-bars-title">DFL u &rarr; DRONE</div>
          ${uBarRow('T_ddot',  'T̈',     'N/s²')}
          ${uBarRow('tau_phi', 'τ_φ',    'N·m')}
          ${uBarRow('tau_theta','τ_θ',   'N·m')}
          ${uBarRow('tau_psi', 'τ_ψ',    'N·m')}
          ${uBarRow('dphi_g',  'φ̇_g',   'rad/s')}
          ${uBarRow('dtheta_g','θ̇_g',   'rad/s')}
          ${uBarRow('dpsi_g',  'ψ̇_g',   'rad/s')}
        </div>
      </div>
    </div>
  `;

  const readoutText = parentEl.querySelector('#drone-readout-text');
  const inputs = parentEl.querySelector('#drone-inputs');

  function tick() {
    const s = getDroneState();
    const phi   = s[13] * 180 / Math.PI;
    const theta = s[14] * 180 / Math.PI;
    const psi   = s[15] * 180 / Math.PI;
    readoutText.textContent =
      `PHI_g ${padSigned(phi, 4)}°   THE_g ${padSigned(theta, 4)}°   PSI_g ${padSigned(psi, 4)}°`;
    updateGimbalRing(parentEl, 'phi_g',   phi);
    updateGimbalRing(parentEl, 'theta_g', theta);
    updateGimbalRing(parentEl, 'psi_g',   psi);
    requestAnimationFrame(tick);
  }
  requestAnimationFrame(tick);

  function setU(u) {
    setUBar(inputs, 'T_ddot',    u[0], U_LIMITS.T_ddot,    2, 'N/s²');
    setUBar(inputs, 'tau_phi',   u[1], U_LIMITS.tau_phi,   3, 'N·m');
    setUBar(inputs, 'tau_theta', u[2], U_LIMITS.tau_theta, 3, 'N·m');
    setUBar(inputs, 'tau_psi',   u[3], U_LIMITS.tau_psi,   3, 'N·m');
    setUBar(inputs, 'dphi_g',    u[4], U_LIMITS.dphi_g,    3, 'rad/s');
    setUBar(inputs, 'dtheta_g',  u[5], U_LIMITS.dtheta_g,  3, 'rad/s');
    setUBar(inputs, 'dpsi_g',    u[6], U_LIMITS.dpsi_g,    3, 'rad/s');
  }
  return { setU };
}

function svgGimbalRing(key, label) {
  return `<div class="instr instr-gimbal">
    <svg viewBox="-50 -50 100 100">
      <circle cx="0" cy="0" r="42" fill="#0a0e16" stroke="#6cc" stroke-width="1.5"/>
      <g stroke="#cfe" stroke-width="0.6" fill="none">
        ${tickMarks(0, 360, 30, 36, 42)}
      </g>
      <polygon points="0,-40 -3,-30 3,-30" fill="#ff0" id="ring-${key}-mark"/>
      <text x="0" y="2" text-anchor="middle" fill="#ff0" font-size="12" font-family="monospace" font-weight="bold" id="ring-${key}-val">-- °</text>
      <text x="0" y="38" text-anchor="middle" fill="#cfe" font-size="6">${label}</text>
    </svg>
  </div>`;
}

function updateGimbalRing(root, key, deg) {
  const mark = root.querySelector(`#ring-${key}-mark`);
  const val  = root.querySelector(`#ring-${key}-val`);
  if (mark) mark.setAttribute('transform', `rotate(${deg})`);
  if (val)  val.textContent = `${deg.toFixed(0)}°`;
}

function uBarRow(key, label, unit) {
  return `
    <div class="input-row u-row">
      <span class="input-label">${label}</span>
      <div class="input-bar"><div class="input-bar-fill" data-bar="${key}"></div></div>
      <span class="input-value" data-val="${key}">-- ${unit}</span>
    </div>
  `;
}

function setUBar(root, key, value, limit, digits, unit) {
  const fill = root.querySelector(`[data-bar="${key}"]`);
  const val  = root.querySelector(`[data-val="${key}"]`);
  if (!fill || !val) return;
  const frac = Math.max(-1, Math.min(1, value / limit));
  if (frac >= 0) {
    fill.style.left  = '50%';
    fill.style.width = `${frac * 50}%`;
    fill.classList.remove('neg');
  } else {
    fill.style.left  = `${50 + frac * 50}%`;
    fill.style.width = `${-frac * 50}%`;
    fill.classList.add('neg');
  }
  val.textContent = `${value.toFixed(digits)} ${unit}`;
}

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

function padSigned(n, w) {
  const s = Math.round(n).toString();
  if (s.startsWith('-')) return s.padStart(w + 1);
  return ('+' + s).padStart(w + 1);
}
