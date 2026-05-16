// web_demo/render/overlay_drone.js
// HTML/CSS overlay rendered on top of the right viewport. Shows spinning
// rotor discs, live gimbal angles, and the DFL controller's u vector as bars.

// Reasonable bar limits for the DFL u vector. These are visualisation only;
// the underlying values are not clamped. Picked so typical magnitudes are
// visible without saturating: torques ~ +-1 N·m, T_ddot ~ +-50 N/s^2,
// gimbal rates ~ +-3 rad/s.
const U_LIMITS = {
  T_ddot:  50,
  tau_phi:   1,
  tau_theta: 1,
  tau_psi:   1,
  dphi_g:    3,
  dtheta_g:  3,
  dpsi_g:    3,
};

/**
 * Mount the drone-style overlay on the given element.
 * @param {HTMLElement} parentEl  container DOM element
 * @param {() => number[]} getDroneState  returns the live 18-element drone state
 * @returns {{ setU: (u: number[]) => void }}
 */
export function mountDroneOverlay(parentEl, getDroneState) {
  parentEl.innerHTML = `
    <div class="hud-rail-left"></div>
    <div class="hud-rail-right"></div>
    <div class="hud-band-top">
      <div class="hud-readout" id="drone-readout">
        <span data-k="phig">  PHI_g    -- °</span>
        <span data-k="thetag">THE_g    -- °</span>
        <span data-k="psig">  PSI_g    -- °</span>
      </div>
      <div class="rotor-row">
        <div class="rotor-disc"></div>
        <div class="rotor-disc"></div>
        <div class="rotor-disc"></div>
        <div class="rotor-disc"></div>
      </div>
    </div>
    <div class="input-panel" id="drone-inputs">
      <div class="input-panel-title">DFL CONTROLLER OUTPUT u &rarr; DRONE</div>
      ${inputRow('T_ddot',  'T_ddot',    'N/s²')}
      ${inputRow('TAU_phi', 'tau_phi',   'N·m')}
      ${inputRow('TAU_the', 'tau_theta', 'N·m')}
      ${inputRow('TAU_psi', 'tau_psi',   'N·m')}
      ${inputRow('dPHI_g',  'dphi_g',    'rad/s')}
      ${inputRow('dTHE_g',  'dtheta_g',  'rad/s')}
      ${inputRow('dPSI_g',  'dpsi_g',    'rad/s')}
    </div>
    <div class="hud-band-bottom">
      <span class="hud-label">DRONE + GIMBAL CAMERA</span>
      <span class="hud-label">3-AXIS</span>
    </div>
  `;
  const readout = parentEl.querySelector('#drone-readout');
  const inputs  = parentEl.querySelector('#drone-inputs');

  function tick() {
    const s = getDroneState();
    const phi   = s[13] * 180 / Math.PI;
    const theta = s[14] * 180 / Math.PI;
    const psi   = s[15] * 180 / Math.PI;
    setSpan(readout, 'phig',   `PHI_g ${padSigned(phi,   4)} °`);
    setSpan(readout, 'thetag', `THE_g ${padSigned(theta, 4)} °`);
    setSpan(readout, 'psig',   `PSI_g ${padSigned(psi,   4)} °`);
    requestAnimationFrame(tick);
  }
  requestAnimationFrame(tick);

  function setU(u) {
    setBar(inputs, 'T_ddot',    u[0], U_LIMITS.T_ddot,    2, 'N/s²');
    setBar(inputs, 'tau_phi',   u[1], U_LIMITS.tau_phi,   3, 'N·m');
    setBar(inputs, 'tau_theta', u[2], U_LIMITS.tau_theta, 3, 'N·m');
    setBar(inputs, 'tau_psi',   u[3], U_LIMITS.tau_psi,   3, 'N·m');
    setBar(inputs, 'dphi_g',    u[4], U_LIMITS.dphi_g,    3, 'rad/s');
    setBar(inputs, 'dtheta_g',  u[5], U_LIMITS.dtheta_g,  3, 'rad/s');
    setBar(inputs, 'dpsi_g',    u[6], U_LIMITS.dpsi_g,    3, 'rad/s');
  }
  return { setU };
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

function setBar(root, key, value, limit, digits, unit) {
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

function setSpan(root, key, text) {
  const el = root.querySelector(`[data-k="${key}"]`);
  if (el) el.textContent = text;
}

function padSigned(n, w) {
  const s = Math.round(n).toString();
  if (s.startsWith('-')) return s.padStart(w + 1);
  return ('+' + s).padStart(w + 1);
}
