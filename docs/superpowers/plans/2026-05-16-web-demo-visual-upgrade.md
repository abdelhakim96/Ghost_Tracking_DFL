# Web Demo Visual Upgrade Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Upgrade the ghost-tracking web demo with low-poly mountains + sea environment, cockpit/drone HTML overlays around each POV, and a corner 3D inset showing both vehicles externally with the FW semi-transparent.

**Architecture:** Pure additive extension to the existing demo. Three new modules under `web_demo/render/`, two new DOM elements + CSS rules, one overhauled `scene.js`, and ~10 lines added to `main.js`. The "two POVs look identical" invariant is preserved by layering overlays as HTML/CSS *over* the WebGL canvas — the image inside the camera frame stays pixel-identical between the two halves.

**Tech Stack:** Vanilla JS + Three.js (CDN ESM), no build step. HTML/CSS for overlays. Second `THREE.WebGLRenderer` for the inset canvas.

**Spec:** `docs/superpowers/specs/2026-05-16-web-demo-visual-upgrade-design.md`

---

## File Map

```
web_demo/
├── render/
│   ├── scene.js                 # OVERHAULED — replace cubes with mountains + sea
│   ├── cameras.js               # unchanged
│   ├── viewport.js              # MODIFIED — add initInsetRenderer + renderInset
│   ├── overlay_cockpit.js       # NEW — cockpit canopy + instruments (HTML/CSS)
│   ├── overlay_drone.js         # NEW — rotor blur + drone bracket (HTML/CSS)
│   ├── inset_scene.js           # NEW — separate Three.js scene with FW + drone
│   └── models.js                # NEW — procedural FW + quadrotor meshes
├── main.js                      # MODIFIED — wire overlays + inset
├── index.html                   # MODIFIED — 2 overlay <div>s + 1 inset <canvas>
├── styles.css                   # MODIFIED — overlay + inset CSS
└── tests/
    └── e2e_smoke.spec.js        # MODIFIED — add inset-canvas existence check
```

---

## Phase 1: Environment overhaul

### Task 1: Replace scene.js with mountains + sea

**Files:**
- Modify: `web_demo/render/scene.js` (full rewrite)

- [ ] **Step 1: Replace `web_demo/render/scene.js` with this content**

```javascript
// web_demo/render/scene.js
import * as THREE from 'three';

const SEA_SIZE = 20000;
const MOUNT_COUNT = 60;
const MOUNT_RADIUS_KM = 5;
const PALETTE = [0x4a6a3a, 0x5a7a48, 0x6b6a4a, 0x7a6a3a]; // brown-greens

export function buildScene() {
  const scene = new THREE.Scene();
  scene.add(makeSky());
  scene.add(makeSea());
  for (const m of makeMountains()) scene.add(m);
  scene.add(makeSun());
  scene.add(new THREE.AmbientLight(0x6080aa, 0.6));
  return scene;
}

function makeSky() {
  const geom = new THREE.SphereGeometry(8000, 32, 16);
  const mat  = new THREE.ShaderMaterial({
    side: THREE.BackSide,
    uniforms: {
      topColor:    { value: new THREE.Color(0x3a78c8) },
      bottomColor: { value: new THREE.Color(0xa8c8ee) },
    },
    vertexShader: `varying vec3 vPos;
      void main(){ vPos = position;
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position,1.0); }`,
    fragmentShader: `varying vec3 vPos;
      uniform vec3 topColor; uniform vec3 bottomColor;
      void main(){ float t = clamp(0.5 + 0.5 * normalize(vPos).y, 0.0, 1.0);
        gl_FragColor = vec4(mix(bottomColor, topColor, t), 1.0); }`,
  });
  return new THREE.Mesh(geom, mat);
}

function makeSea() {
  const geom = new THREE.PlaneGeometry(SEA_SIZE, SEA_SIZE, 1, 1);
  const mat  = new THREE.MeshPhongMaterial({ color: 0x1a4f7a, shininess: 60 });
  const mesh = new THREE.Mesh(geom, mat);
  mesh.rotation.x = -Math.PI / 2;
  mesh.position.y = 0;
  return mesh;
}

function makeMountains() {
  const rng = mulberry32(123);
  const out = [];
  for (let i = 0; i < MOUNT_COUNT; i++) {
    const r = 200 + rng() * (MOUNT_RADIUS_KM * 1000 - 200);
    const theta = rng() * Math.PI * 2;
    const x = r * Math.cos(theta);
    const z = r * Math.sin(theta);
    const height = 100 + rng() * 400;
    const radius = 80 + rng() * 220;
    const color  = PALETTE[Math.floor(rng() * PALETTE.length)];
    const geom = new THREE.ConeGeometry(radius, height, 5);
    const mat  = new THREE.MeshLambertMaterial({ color, flatShading: true });
    const mesh = new THREE.Mesh(geom, mat);
    mesh.position.set(x, height / 2, z);
    out.push(mesh);
  }
  return out;
}

function makeSun() {
  const sun = new THREE.DirectionalLight(0xffffff, 1.0);
  sun.position.set(1, 1, 0.5);
  return sun;
}

function mulberry32(seed) {
  return function() {
    seed |= 0; seed = (seed + 0x6D2B79F5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
```

- [ ] **Step 2: Run existing tests to confirm no regression**

```bash
cd web_demo && npm test
```

Expected: 12/12 pass (unit tests don't touch scene.js, so this is purely a regression sanity check).

- [ ] **Step 3: Smoke-test in browser**

```bash
cd web_demo && python3 -m http.server 8000 &
SERVER_PID=$!
sleep 1
curl -fsS http://localhost:8000/render/scene.js | head -5    # confirms file is served
kill $SERVER_PID 2>/dev/null
```

Expected: first 5 lines of scene.js printed.

- [ ] **Step 4: Run e2e to confirm rendering still works**

```bash
cd web_demo && npx playwright test 2>&1 | tail -8
```

Expected: 1 test passes (geodesic < 1°). The Playwright test doesn't care about scene aesthetics, only that the world renders and the controller tracks.

- [ ] **Step 5: Commit**

```bash
git add web_demo/render/scene.js
git commit -m "feat(web_demo): replace cube field with low-poly mountains + sea"
```

---

## Phase 2: HTML/CSS overlays

### Task 2: Add overlay DOM elements + base CSS

**Files:**
- Modify: `web_demo/index.html`
- Modify: `web_demo/styles.css`

- [ ] **Step 1: Add overlay containers to `web_demo/index.html`**

Replace the `<body>` of `web_demo/index.html` with:

```html
<body>
  <canvas id="canvas"></canvas>
  <canvas id="inset"></canvas>
  <div id="overlay-cockpit"></div>
  <div id="overlay-drone"></div>
  <div id="overlay">
    <div id="cheatsheet">
      <strong>Pilot:</strong>
      &uarr;/&darr; pitch &nbsp; &larr;/&rarr; roll &nbsp; A/D rudder &nbsp; W/S thrust &nbsp; R reset &nbsp; P pause
    </div>
  </div>
  <script type="importmap">
    { "imports": { "three": "https://unpkg.com/three@0.160.0/build/three.module.js" } }
  </script>
  <script type="module" src="main.js"></script>
</body>
```

Note: the `#labels` div is removed — the new overlays carry their own labels.

- [ ] **Step 2: Replace `web_demo/styles.css` with this content**

```css
* { margin: 0; padding: 0; box-sizing: border-box; }
html, body { width: 100%; height: 100%; overflow: hidden; background: #111; color: #ddd; font-family: system-ui, sans-serif; }
#canvas { display: block; width: 100vw; height: 100vh; }

#overlay { position: fixed; inset: 0; pointer-events: none; }
#cheatsheet { position: fixed; bottom: 8px; left: 50%; transform: translateX(-50%);
  padding: 6px 12px; background: rgba(0,0,0,0.5); border-radius: 6px; font-size: 13px; pointer-events: none; }

#overlay-cockpit, #overlay-drone {
  position: fixed; top: 0; height: 100vh; width: 50vw; pointer-events: none;
}
#overlay-cockpit { left: 0; }
#overlay-drone   { left: 50vw; }

.hud-band-top {
  position: absolute; left: 0; right: 0; top: 0; height: 60px;
  background: linear-gradient(to bottom, rgba(20,22,30,0.92) 0%, rgba(20,22,30,0.85) 70%, rgba(20,22,30,0) 100%);
  color: #cfe; font: 12px/16px monospace; padding: 8px 12px;
  display: flex; justify-content: space-between; align-items: flex-start;
}
.hud-band-bottom {
  position: absolute; left: 0; right: 0; bottom: 0; height: 28px;
  background: rgba(20,22,30,0.85);
  color: #cfe; font: 11px/14px monospace; padding: 6px 12px;
  display: flex; justify-content: space-between; align-items: center;
}
.hud-rail-left, .hud-rail-right {
  position: absolute; top: 0; bottom: 0; width: 12px;
}
.hud-rail-left  { left: 0;  background: linear-gradient(to right, rgba(0,0,0,0.6), transparent); }
.hud-rail-right { right: 0; background: linear-gradient(to left,  rgba(0,0,0,0.6), transparent); }

.hud-readout span { display: inline-block; min-width: 5em; margin-right: 8px; }
.hud-label   { letter-spacing: 0.1em; opacity: 0.85; }
.hud-icon    { width: 20px; height: 20px; opacity: 0.85; }

/* Drone-overlay-specific rotor visualization */
.rotor-row { display: flex; gap: 6px; }
.rotor-disc {
  width: 22px; height: 22px; border-radius: 50%;
  background: radial-gradient(circle, rgba(140,150,170,0.85) 0%, rgba(140,150,170,0.0) 70%);
  border: 1px dashed rgba(180,190,210,0.6);
  animation: rotor-spin 0.08s linear infinite;
}
@keyframes rotor-spin { from { transform: rotate(0deg); } to { transform: rotate(360deg); } }

/* Inset canvas */
#inset {
  position: fixed; right: 12px; bottom: 50px; width: 280px; height: 200px;
  border: 2px solid rgba(255,255,255,0.4); border-radius: 8px;
  background: rgba(0,0,0,0.6);
  pointer-events: none;
}
```

- [ ] **Step 3: Smoke-test by opening the page**

```bash
cd web_demo && python3 -m http.server 8000 &
SERVER_PID=$!
sleep 1
curl -fsS http://localhost:8000/ | grep -E "overlay-cockpit|overlay-drone|inset" | head
kill $SERVER_PID 2>/dev/null
```

Expected: three lines printed showing the new DOM elements are present.

- [ ] **Step 4: Commit**

```bash
git add web_demo/index.html web_demo/styles.css
git commit -m "feat(web_demo): add overlay + inset DOM elements + base CSS"
```

---

### Task 3: Cockpit overlay module

**Files:**
- Create: `web_demo/render/overlay_cockpit.js`

- [ ] **Step 1: Write `web_demo/render/overlay_cockpit.js`**

```javascript
// web_demo/render/overlay_cockpit.js
// HTML/CSS overlay rendered on top of the left viewport. Reads FW state via
// a caller-provided getter and updates every animation frame.

const YOKE_SVG = `
  <svg class="hud-icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"
       stroke-width="1.5" stroke-linecap="round">
    <circle cx="12" cy="12" r="6"/><line x1="2" y1="12" x2="22" y2="12"/>
    <line x1="12" y1="6" x2="12" y2="18"/>
  </svg>`;

/**
 * Mount the cockpit-style overlay on the given element.
 * @param {HTMLElement} parentEl  container DOM element
 * @param {() => number[]} getFwState  returns the live 13-element FW state
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
    <div class="hud-band-bottom">
      <span class="hud-label">FW COCKPIT</span>
      <span>${YOKE_SVG}</span>
    </div>
  `;
  const readout = parentEl.querySelector('#cockpit-readout');

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
```

- [ ] **Step 2: Commit**

```bash
git add web_demo/render/overlay_cockpit.js
git commit -m "feat(web_demo): cockpit overlay with live FW telemetry"
```

---

### Task 4: Drone overlay module

**Files:**
- Create: `web_demo/render/overlay_drone.js`

- [ ] **Step 1: Write `web_demo/render/overlay_drone.js`**

```javascript
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
```

- [ ] **Step 2: Commit**

```bash
git add web_demo/render/overlay_drone.js
git commit -m "feat(web_demo): drone overlay with spinning rotors + live gimbal angles"
```

---

## Phase 3: 3D inset

### Task 5: Procedural FW + quadrotor models

**Files:**
- Create: `web_demo/render/models.js`

- [ ] **Step 1: Write `web_demo/render/models.js`**

```javascript
// web_demo/render/models.js
// Procedural low-poly meshes for the inset 3D plot.
import * as THREE from 'three';

/**
 * Returns a simple low-poly fixed-wing aircraft group.
 * Span ~12 m, length ~8 m. Material is shared on the group so opacity can
 * be tuned via group.traverse(...).
 * @returns {THREE.Group}
 */
export function makeFwModel() {
  const mat = new THREE.MeshLambertMaterial({
    color: 0xcccccc, transparent: true, opacity: 0.35,
    flatShading: true, side: THREE.DoubleSide,
  });
  const group = new THREE.Group();

  const fuselage = new THREE.Mesh(new THREE.BoxGeometry(8, 1.5, 1.5), mat);
  group.add(fuselage);

  const wings = new THREE.Mesh(new THREE.BoxGeometry(2, 0.3, 12), mat);
  wings.position.set(0, 0.1, 0);
  group.add(wings);

  const tailV = new THREE.Mesh(new THREE.BoxGeometry(1.5, 2.5, 0.2), mat);
  tailV.position.set(-3.5, 1.2, 0);
  group.add(tailV);

  const tailH = new THREE.Mesh(new THREE.BoxGeometry(2, 0.2, 4), mat);
  tailH.position.set(-3.5, 0.3, 0);
  group.add(tailH);

  // Nose tip for visual orientation
  const nose = new THREE.Mesh(new THREE.ConeGeometry(0.6, 1.5, 6), mat);
  nose.position.set(4.5, 0, 0);
  nose.rotation.z = -Math.PI / 2;
  group.add(nose);

  return group;
}

/**
 * Returns a quadrotor model and its rotor mesh array so the caller can spin them.
 * @returns {{ group: THREE.Group, rotorMeshes: THREE.Mesh[] }}
 */
export function makeDroneModel() {
  const bodyMat = new THREE.MeshLambertMaterial({ color: 0x222831, flatShading: true });
  const armMat  = new THREE.MeshLambertMaterial({ color: 0x393e46, flatShading: true });
  const rotorMat = new THREE.MeshBasicMaterial({
    color: 0x9aa1aa, transparent: true, opacity: 0.7, side: THREE.DoubleSide,
  });

  const group = new THREE.Group();

  const body = new THREE.Mesh(new THREE.BoxGeometry(0.6, 0.3, 0.6), bodyMat);
  group.add(body);

  const rotorMeshes = [];
  for (let i = 0; i < 4; i++) {
    const angle = i * Math.PI / 2 + Math.PI / 4; // X arrangement
    const arm = new THREE.Mesh(new THREE.BoxGeometry(1.6, 0.08, 0.08), armMat);
    arm.position.set(0, 0, 0);
    arm.rotation.y = -angle;
    group.add(arm);

    const rx = Math.cos(angle) * 0.8;
    const rz = Math.sin(angle) * 0.8;
    const rotor = new THREE.Mesh(new THREE.CircleGeometry(0.5, 16), rotorMat);
    rotor.position.set(rx, 0.1, rz);
    rotor.rotation.x = -Math.PI / 2;     // lay flat (face up)
    group.add(rotor);
    rotorMeshes.push(rotor);
  }

  return { group, rotorMeshes };
}
```

- [ ] **Step 2: Commit**

```bash
git add web_demo/render/models.js
git commit -m "feat(web_demo): procedural FW + quadrotor models for inset"
```

---

### Task 6: Inset scene (camera + light + ground reference)

**Files:**
- Create: `web_demo/render/inset_scene.js`

- [ ] **Step 1: Write `web_demo/render/inset_scene.js`**

```javascript
// web_demo/render/inset_scene.js
// A small standalone Three.js scene used by the corner inset. Renders the
// FW model (semi-transparent) and the drone model (opaque) from an orbiting
// camera so the viewer can see how well they coincide.
import * as THREE from 'three';
import { makeFwModel, makeDroneModel } from './models.js';

const ORBIT_RADIUS = 30;     // m, distance from FW
const ORBIT_HEIGHT = 10;     // m, above FW
const ORBIT_RATE   = 10;     // deg/s

export function buildInsetScene(aspect) {
  const scene = new THREE.Scene();
  scene.background = null;     // transparent so CSS bg shows through

  const fwModel = makeFwModel();
  scene.add(fwModel);

  const { group: droneModel, rotorMeshes } = makeDroneModel();
  scene.add(droneModel);

  // Ground reference disc
  const disc = new THREE.Mesh(
    new THREE.RingGeometry(2, 2.5, 32),
    new THREE.MeshBasicMaterial({ color: 0x666666, side: THREE.DoubleSide, transparent: true, opacity: 0.4 }),
  );
  disc.rotation.x = -Math.PI / 2;
  disc.position.y = -2.5;
  scene.add(disc);

  scene.add(new THREE.AmbientLight(0xffffff, 0.6));
  const sun = new THREE.DirectionalLight(0xffffff, 0.8);
  sun.position.set(5, 8, 4);
  scene.add(sun);

  const camera = new THREE.PerspectiveCamera(40, aspect, 0.5, 500);
  camera.position.set(ORBIT_RADIUS, ORBIT_HEIGHT, 0);
  camera.lookAt(0, 0, 0);

  return { scene, fwModel, droneModel, rotorMeshes, camera };
}

/**
 * Update inset poses and orbit camera angle.
 * @param {object} ins                 result of buildInsetScene
 * @param {number[]} fwStateBodyToNED   13-element FW state
 * @param {number[]} droneState         18-element drone state
 * @param {number} timeNowMs            performance.now()
 */
export function updateInset(ins, fwStateBodyToNED, droneState, timeNowMs) {
  // Centre the inset world on the FW position. Both models receive a relative
  // offset; camera orbits the origin.
  const fwPosNED    = [fwStateBodyToNED[0], fwStateBodyToNED[1], fwStateBodyToNED[2]];
  const dronePosNED = [droneState[0],       droneState[1],       droneState[2]];

  // Convert NED -> Three (x=N, y=-D, z=E) and centre on FW
  const fwPos3    = [0, 0, 0];
  const dronePos3 = [
    dronePosNED[0] - fwPosNED[0],
    -(dronePosNED[2] - fwPosNED[2]),
    dronePosNED[1] - fwPosNED[1],
  ];

  ins.fwModel.position.set(...fwPos3);
  ins.droneModel.position.set(...dronePos3);

  // Apply orientations
  const qFw = fwStateBodyToNED.slice(6, 10);
  setNEDQuaternion(ins.fwModel, qFw);

  const qDrone = droneState.slice(3, 7);
  setNEDQuaternion(ins.droneModel, qDrone);

  // Spin rotors visibly
  const dt = 1 / 60;
  for (const r of ins.rotorMeshes) r.rotation.z += dt * 60;

  // Orbit camera around the FW (now at world origin)
  const angle = (timeNowMs / 1000) * (ORBIT_RATE * Math.PI / 180);
  ins.camera.position.set(
    ORBIT_RADIUS * Math.cos(angle),
    ORBIT_HEIGHT,
    ORBIT_RADIUS * Math.sin(angle),
  );
  ins.camera.lookAt(0, 0, 0);
}

// q_ned_to_three * q_body_to_ned   — same logic as render/cameras.js but inlined
// to avoid pulling render/cameras.js's setCameraPose into the inset.
function setNEDQuaternion(obj3D, qBodyToNED) {
  const qNedToThree = [Math.SQRT1_2, Math.SQRT1_2, 0, 0];
  const q = quatMul(qNedToThree, qBodyToNED);
  obj3D.quaternion.set(q[1], q[2], q[3], q[0]);
}

function quatMul(a, b) {
  return [
    a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
    a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
    a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
    a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0],
  ];
}
```

- [ ] **Step 2: Commit**

```bash
git add web_demo/render/inset_scene.js
git commit -m "feat(web_demo): inset scene with orbiting camera + spinning rotors"
```

---

### Task 7: Extend viewport.js for inset rendering

**Files:**
- Modify: `web_demo/render/viewport.js`

- [ ] **Step 1: Replace `web_demo/render/viewport.js` with this content**

```javascript
// web_demo/render/viewport.js
import * as THREE from 'three';

let renderer;
let insetRenderer;

export function initRenderer(canvas) {
  renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setScissorTest(true);
  window.addEventListener('resize', () => renderer.setSize(window.innerWidth, window.innerHeight));
  return renderer;
}

export function renderDual(scene, camLeft, camRight) {
  const W = window.innerWidth, H = window.innerHeight;
  const halfW = Math.floor(W / 2);
  camLeft.aspect  = halfW / H;
  camRight.aspect = halfW / H;
  camLeft.updateProjectionMatrix();
  camRight.updateProjectionMatrix();

  renderer.setViewport(0, 0, halfW, H);
  renderer.setScissor (0, 0, halfW, H);
  renderer.render(scene, camLeft);

  renderer.setViewport(halfW, 0, W - halfW, H);
  renderer.setScissor (halfW, 0, W - halfW, H);
  renderer.render(scene, camRight);
}

/**
 * Initialise a SECOND WebGL renderer dedicated to the corner inset canvas.
 * Independent context so it doesn't share scissor / viewport state with
 * the main renderer.
 * @param {HTMLCanvasElement} canvas
 * @returns {THREE.WebGLRenderer | null}  null if context creation fails
 */
export function initInsetRenderer(canvas) {
  try {
    insetRenderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
    insetRenderer.setPixelRatio(window.devicePixelRatio);
    insetRenderer.setSize(canvas.clientWidth, canvas.clientHeight, false);
    insetRenderer.setClearColor(0x000000, 0);
    return insetRenderer;
  } catch (e) {
    console.warn('Inset renderer failed to init, hiding inset:', e);
    canvas.style.display = 'none';
    insetRenderer = null;
    return null;
  }
}

export function renderInset(scene, camera) {
  if (!insetRenderer) return;
  insetRenderer.render(scene, camera);
}
```

- [ ] **Step 2: Run existing tests to confirm no regression**

```bash
cd web_demo && npm test
```

Expected: 12/12 pass.

- [ ] **Step 3: Commit**

```bash
git add web_demo/render/viewport.js
git commit -m "feat(web_demo): viewport extension for inset renderer"
```

---

### Task 8: Wire everything in main.js

**Files:**
- Modify: `web_demo/main.js` (full replacement)

- [ ] **Step 1: Replace `web_demo/main.js` with this content**

```javascript
// web_demo/main.js
import { buildScene } from './render/scene.js';
import { initRenderer, renderDual, initInsetRenderer, renderInset } from './render/viewport.js';
import { makeCamera, setCameraPose, gimbalQuat } from './render/cameras.js';
import { Keyboard } from './input/keyboard.js';
import { unifiedRK4Step, defaultGains, initialFwState, initialDroneState } from './sim/unified_step.js';
import { mountCockpitOverlay } from './render/overlay_cockpit.js';
import { mountDroneOverlay }   from './render/overlay_drone.js';
import { buildInsetScene, updateInset } from './render/inset_scene.js';

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

window.addEventListener('keydown', (e) => {
  if (e.code === 'KeyR') { fwState = initialFwState(); droneState = initialDroneState(); }
  if (e.code === 'KeyP') { paused = !paused; }
});

// Mount overlays (read live state via closures)
mountCockpitOverlay(document.getElementById('overlay-cockpit'), () => fwState);
mountDroneOverlay  (document.getElementById('overlay-drone'),   () => droneState);

// Inset
const insetAspect = insetCanvas.clientWidth / insetCanvas.clientHeight;
const insetGl = initInsetRenderer(insetCanvas);
const ins = insetGl ? buildInsetScene(insetAspect) : null;

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

  window.__demoState = { fwQuat, camWorldQuat, dronePos, fwPos };
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
```

- [ ] **Step 2: Run e2e to verify the demo still tracks the FW**

```bash
cd web_demo && npx playwright test 2>&1 | tail -8
```

Expected: 1 test PASS, geodesic < 1°.

- [ ] **Step 3: Smoke-test rendering**

```bash
cd web_demo && python3 -m http.server 8000 &
SERVER_PID=$!
sleep 1
curl -fsS http://localhost:8000/ > /tmp/page.html
grep -E "overlay-cockpit|overlay-drone|inset|main.js" /tmp/page.html | head
kill $SERVER_PID 2>/dev/null
```

Expected: lines mentioning the new DOM elements and main.js.

- [ ] **Step 4: Commit**

```bash
git add web_demo/main.js
git commit -m "feat(web_demo): wire overlays + inset + new main loop"
```

---

## Phase 4: e2e test extension

### Task 9: Add inset canvas assertion to Playwright

**Files:**
- Modify: `web_demo/tests/e2e_smoke.spec.js`

- [ ] **Step 1: Replace `web_demo/tests/e2e_smoke.spec.js` with this content**

```javascript
import { test, expect } from '@playwright/test';

test('camera tracks FW after free-flight', async ({ page }) => {
  await page.goto('/');
  await page.waitForFunction(() => window.__demoState !== undefined);

  await page.keyboard.down('ArrowUp');
  await page.waitForTimeout(800);
  await page.keyboard.up('ArrowUp');
  await page.waitForTimeout(2000);

  const s = await page.evaluate(() => window.__demoState);

  function dotQ(a, b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]+a[3]*b[3]; }
  const geo = 2 * Math.acos(Math.min(1, Math.abs(dotQ(s.fwQuat, s.camWorldQuat)))) * 180/Math.PI;
  console.log('geodesic camera/FW error:', geo.toFixed(4), 'deg');
  expect(geo).toBeLessThan(1.0);
});

test('inset canvas is present and has WebGL', async ({ page }) => {
  await page.goto('/');
  await page.waitForFunction(() => window.__demoState !== undefined);

  const insetCount = await page.locator('#inset').count();
  expect(insetCount).toBe(1);

  const hasContext = await page.evaluate(() => {
    const c = document.querySelector('#inset');
    if (!c) return false;
    // We initialised with a 3D context; if it succeeded the element is visible.
    return c.style.display !== 'none';
  });
  expect(hasContext).toBe(true);
});

test('overlays present', async ({ page }) => {
  await page.goto('/');
  await page.waitForFunction(() => window.__demoState !== undefined);
  await expect(page.locator('#overlay-cockpit .hud-band-top')).toBeVisible();
  await expect(page.locator('#overlay-drone   .hud-band-top')).toBeVisible();
  await expect(page.locator('#overlay-drone   .rotor-disc')).toHaveCount(4);
});
```

- [ ] **Step 2: Run e2e to confirm all three tests pass**

```bash
cd web_demo && npx playwright test 2>&1 | tail -15
```

Expected: 3 tests PASS.

If the geodesic test fails after the upgrade, the camera math in main.js has drifted — check that `setCameraPose(camDrone, camWorldQuat, dronePos)` still uses `camWorldQuat = quatMul(droneQuat, gimbalQuat(...))` exactly as in the original main.js.

If the overlay tests fail, check that:
- `#overlay-cockpit` and `#overlay-drone` exist in `index.html` (Task 2)
- `mountCockpitOverlay` and `mountDroneOverlay` are imported and called in `main.js` (Task 8)
- The DOM structure inside each overlay matches the selectors in this test (Tasks 3, 4)

- [ ] **Step 3: Commit**

```bash
git add web_demo/tests/e2e_smoke.spec.js
git commit -m "test(web_demo): assert overlays + inset are present"
```

---

## Phase 5: Final acceptance

### Task 10: Manual visual smoke + push

- [ ] **Step 1: Run full test suite one more time**

```bash
cd web_demo && npm test && npx playwright test
```

Expected: 12 unit tests pass, 3 e2e tests pass.

- [ ] **Step 2: Open in a browser**

```bash
cd web_demo && python3 -m http.server 8000
# in another terminal, open http://localhost:8000
```

Manually verify:
- Sky gradient + sea + mountains visible (replacing the cube field)
- Left half has cockpit-style top band with live telemetry (AS, ALT, HDG, PIT, ROL)
- Right half has drone-style top band with PHI_g/THE_g/PSI_g and 4 spinning rotor discs
- Lower-right shows a 280×200 inset with an aircraft (transparent) and a drone (opaque)
- Inset camera auto-orbits around the FW
- Pressing arrow keys moves both POVs together; the inset reflects the new poses

Stop server with Ctrl+C.

- [ ] **Step 3: Final commit if any tweaks were needed**

```bash
git status
# if anything changed:
git add -A
git commit -m "chore(web_demo): visual upgrade tuning"
```

- [ ] **Step 4: Summary acceptance checklist**

| # | Criterion | Status |
|---|---|---|
| 1 | Both POVs render mountains + sea + sky | Manual (Step 2) |
| 2 | Cockpit overlay on left, drone overlay on right | Manual + e2e Task 9 |
| 3 | Inset shows FW (transparent) + drone (opaque) updating live | Manual + e2e Task 9 |
| 4 | Two POVs still pixel-similar in the camera image | e2e Task 9 geodesic check (< 1°) |
| 5 | Frame rate stays ≥ 30 FPS on integrated GPU | Manual (Step 2) |

---

## Final notes for the implementer

- **The "identical POVs" invariant is the whole point.** If you accidentally include vehicle geometry in the main `scene` (Task 1), each POV will see itself rendered, breaking identity. Keep models only in the inset scene.
- **State indexing reminders:**
  - `fwState`: 13 elements — `[x, y, z, u, v, w, q0, q1, q2, q3, p, q, r]`
  - `droneState`: 18 elements — `[x, y, z, q0, q1, q2, q3, u_w, v_w, w_w, p, q, r, phi_g, theta_g, psi_g, zeta, xi]`
- **NED ↔ Three.js conversion** is handled in `cameras.js` and (inlined) `inset_scene.js` via `q_NED_to_three = [√½, √½, 0, 0]` plus the position swap `(N, -D, E)`. Don't replicate that logic in a third place; if you need it again, factor it into a shared helper.
- **Don't push to origin** — leave that to the user. The plan only `commit`s locally.
