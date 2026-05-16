// web_demo/render/overlay_drone.js
// HTML/CSS overlay rendered on top of the right viewport. Shows spinning
// rotor discs and live gimbal angles.

/**
 * Mount the drone-style overlay on the given element.
 * @param {HTMLElement} parentEl  container DOM element
 * @param {() => number[]} getDroneState  returns the live 18-element drone state
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
    <div class="hud-band-bottom">
      <span class="hud-label">DRONE + GIMBAL CAMERA</span>
      <span class="hud-label">3-AXIS</span>
    </div>
  `;
  const readout = parentEl.querySelector('#drone-readout');

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
