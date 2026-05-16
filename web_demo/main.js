// web_demo/main.js
import { buildScene } from './render/scene.js';
import { initRenderer, renderDual, initInsetRenderer, renderInset } from './render/viewport.js';
import { makeCamera, setCameraPose, gimbalQuat } from './render/cameras.js';
import { Keyboard } from './input/keyboard.js';
import { unifiedRK4Step, snapshotU, defaultGains, initialFwState, initialDroneState } from './sim/unified_step.js';
import { mountCockpitOverlay } from './render/overlay_cockpit.js';
import { mountDroneOverlay }   from './render/overlay_drone.js';
import { buildInsetScene, updateInset } from './render/inset_scene.js';
import { initMinimap, updateMinimap, resetMinimap } from './render/minimap.js';

const canvas      = document.getElementById('canvas');
const insetCanvas = document.getElementById('inset');
const renderer    = initRenderer(canvas);
const scene       = buildScene();

const camFw    = makeCamera(window.innerWidth / 2 / window.innerHeight);
const camDrone = makeCamera(window.innerWidth / 2 / window.innerHeight);

const kbd = new Keyboard();
kbd.attach();

let fwState    = initialFwState();
let droneState = initialDroneState();
const gains    = defaultGains();
let paused     = false;

function resetSim() {
  fwState = initialFwState();
  droneState = initialDroneState();
  resetMinimap();
}
function togglePause() {
  paused = !paused;
  const btn = document.getElementById('btn-pause');
  if (btn) { btn.textContent = paused ? 'RESUME' : 'PAUSE'; btn.classList.toggle('active', paused); }
}
function showWelcome() {
  const m = document.getElementById('welcome');
  if (m) m.classList.remove('hidden');
}
function hideWelcome() {
  const m = document.getElementById('welcome');
  if (m) m.classList.add('hidden');
}

window.addEventListener('keydown', (e) => {
  // Dismiss welcome modal on any key
  const welcome = document.getElementById('welcome');
  if (welcome && !welcome.classList.contains('hidden')) {
    hideWelcome();
    return;
  }
  if (e.code === 'KeyR') resetSim();
  if (e.code === 'KeyP') togglePause();
});
document.getElementById('btn-reset')?.addEventListener('click', resetSim);
document.getElementById('btn-pause')?.addEventListener('click', togglePause);
document.getElementById('btn-help')?.addEventListener('click', showWelcome);
document.getElementById('btn-start')?.addEventListener('click', hideWelcome);

const cockpitApi = mountCockpitOverlay(document.getElementById('overlay-cockpit'), () => fwState);
const droneApi   = mountDroneOverlay  (document.getElementById('overlay-drone'),   () => droneState);

const insetAspect = insetCanvas.clientWidth / insetCanvas.clientHeight;
const insetGl = initInsetRenderer(insetCanvas);
const ins = insetGl ? buildInsetScene(insetAspect) : null;

// Trajectory minimap (Canvas2D, bottom-left)
const minimapCanvas = document.getElementById('minimap');
if (minimapCanvas) initMinimap(minimapCanvas);

const PHYS_DT = 0.005;
let acc = 0;
let lastTime = performance.now();

function frame(now) {
  requestAnimationFrame(frame);
  const dt = Math.min(0.05, (now - lastTime) / 1000);
  lastTime = now;

  if (!paused) {
    acc += dt;
    while (acc >= PHYS_DT) {
      kbd.step(PHYS_DT);
      const controls = kbd.controls();
      ({ fwState, droneState } = unifiedRK4Step({
        fwState, droneState, controlInputs: controls, gains, dt: PHYS_DT
      }));
      acc -= PHYS_DT;
    }
  }

  const fwPos  = fwState.slice(0, 3);
  const fwQuat = fwState.slice(6, 10);
  setCameraPose(camFw, fwQuat, fwPos);

  const dronePos  = droneState.slice(0, 3);
  const droneQuat = droneState.slice(3, 7);
  const gQ        = gimbalQuat(droneState[13], droneState[14], droneState[15]);
  const camWorldQuat = quatMul(droneQuat, gQ);
  setCameraPose(camDrone, camWorldQuat, dronePos);

  renderDual(scene, camFw, camDrone);

  if (ins) {
    updateInset(ins, fwState, droneState, now);
    renderInset(ins.scene, ins.camera);
  }

  // Trajectory minimap
  updateMinimap(fwPos, dronePos);

  // Update input-bar panels (pilot inputs and DFL u)
  const controlsNow = kbd.controls();
  cockpitApi.setControls(controlsNow);
  const u = snapshotU({ fwState, droneState, controlInputs: controlsNow, gains });
  droneApi.setU(u);

  window.__demoState = { fwQuat, camWorldQuat, dronePos, fwPos, controlsNow, u };
}

function quatMul(a, b) {
  return [
    a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
    a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
    a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
    a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0],
  ];
}

requestAnimationFrame((t) => { lastTime = t; frame(t); });
