# Web Demo Visual Upgrade — Design Spec

**Date:** 2026-05-16
**Owner:** abdelhakim96
**Status:** Approved (pending user review of this written spec)
**Builds on:** `2026-05-16-ghost-tracking-game-design.md`

## 1. Purpose

Add visual richness to the web demo so it's clearer what's being shown and more enjoyable to pilot, without breaking the "two POVs look identical" demonstration.

Three upgrades:
1. **Environment**: replace grid + colored cubes with low-poly mountains + animated sea + sky.
2. **POV cues**: each viewport gets a vehicle-specific HUD overlay *outside* the camera image — left has a cockpit-style frame, right has propeller blur + drone bracket. The world rendered *inside* the camera image stays identical between the two halves.
3. **3D inset**: a small lower-right panel renders the FW (semi-transparent) and the drone (opaque) from an orbiting camera so the viewer can directly see how well they coincide.

## 2. Design tension and resolution

The core demo point is "if the controller works, the two POVs show pixel-identical world content." Adding distinct overlays *inside* the camera image would break that. Resolution: overlays are HTML/CSS layered *over* the WebGL canvas with `pointer-events: none`. The world image underneath is pixel-identical between the two halves; the cockpit/drone frames are decoration that signals "this POV is from the plane / this POV is from the drone."

## 3. Stack

Stays the same as the existing demo:
- Vanilla JS + Three.js (CDN ESM)
- No build step
- GitHub Pages hosting

New code is pure addition — existing modules unchanged except `scene.js` (overhauled), `viewport.js` (extended to call inset render), and `main.js` (wires up the new pieces).

## 4. File layout

```
web_demo/
├── render/
│   ├── scene.js                # OVERHAULED — mountains + sea + sky
│   ├── cameras.js              # unchanged
│   ├── viewport.js             # NEW: renderInset() helper
│   ├── overlay_cockpit.js      # NEW — CSS-driven canopy frame + instrument bar
│   ├── overlay_drone.js        # NEW — CSS-driven rotor disc blur + drone bracket
│   ├── inset_scene.js          # NEW — separate Three.js scene for the corner inset
│   └── models.js               # NEW — procedural FW + quadrotor meshes
├── main.js                     # update wiring (~10 lines added)
├── index.html                  # add overlay <div>s + inset <canvas>
├── styles.css                  # add overlay + inset styling
└── tests/
    └── e2e_smoke.spec.js       # add inset-canvas existence check
```

## 5. Component specs

### 5.1 `scene.js` (overhauled)

**Sky**: keep existing gradient ShaderMaterial sphere.

**Sea**: one `PlaneGeometry(20000, 20000)` rotated to face up, at altitude 0. `MeshPhongMaterial({ color: 0x1a4f7a, shininess: 60 })`. Subtle `time`-uniform animated UV offset via a small custom shader (~10 LOC) so the surface has a hint of motion.

**Mountains**: 60 procedural ridge cones in a 5 km radius around origin, generated from a seeded `mulberry32(123)` so layout is stable across reloads. Each cone:
- `ConeGeometry(radius, height, radialSegments=5)` — faceted, not smooth
- Height: random 100-500 m
- Radius: random 100-300 m
- Color picked from a fixed palette of 4 brown-green Lambert materials

**Lighting**: keep existing directional sun + ambient.

**Reference cubes/spheres**: removed (mountains replace them as visual reference geometry).

### 5.2 `overlay_cockpit.js`

Exports `mountCockpitOverlay(parentEl, getFwState)` which appends a `<div class="cockpit-overlay">` to `parentEl`, positioned over the left half of the screen via CSS. The element is `pointer-events: none`.

Visual:
- **Top instrument band** (60 px high, dark `rgba(20, 20, 30, 0.85)`): live readouts of airspeed (m/s), altitude (m), heading (deg), pitch (deg), roll (deg). Updated via internal `requestAnimationFrame` loop reading from `getFwState()`.
- **Side rails** (12 px wide, gradient from `rgba(0,0,0,0.6)` to transparent): suggest canopy posts.
- **Bottom strip** (28 px, dark): label "FW COCKPIT" with a small SVG control-yoke icon.

Frame stays inside the left viewport boundary. On resize, anchor is `position: fixed; left: 0; top: 0; width: 50vw; height: 100vh`.

### 5.3 `overlay_drone.js`

Same shape as cockpit overlay, but on the right half (`left: 50vw; width: 50vw`):
- **Top band**: two animated rotor-blur SVGs (CSS `@keyframes spin { from { transform: rotate(0deg); } to { transform: rotate(360deg); } }`, duration `0.08s linear infinite`) plus telemetry: gimbal angles (phi_g, theta_g, psi_g) live.
- **Side rails**: thin bracket on the left edge suggesting a gimbal mount.
- **Bottom strip**: label "DRONE + GIMBAL CAMERA".

### 5.4 `inset_scene.js`

Exports:
- `buildInsetScene()` — returns `{ scene, fwModel, droneModel, rotorMeshes, camera }`
- `updateInset(scene, fwState, droneState, fwModel, droneModel, rotorMeshes, camera, now)` — per-frame update

Behavior:
- Scene contains the FW and drone meshes (from `models.js`), one directional light, one ambient light, a thin gray reference disc on the ground plane below them
- FW model uses `MeshLambertMaterial({ color: 0xcccccc, transparent: true, opacity: 0.35 })` so the drone behind it is visible
- Drone model uses `MeshLambertMaterial({ color: 0x222222 })`, rotor discs use `MeshBasicMaterial({ color: 0x666666, opacity: 0.7, transparent: true, side: DoubleSide })`
- Camera position is computed each frame: `target = FW position; offset = (cos(θ)*30, 10, sin(θ)*30)` where `θ` auto-increments at 10 deg/s. Camera looks at FW position.
- Rotor meshes spin: `rotorMeshes.forEach(m => m.rotation.y += dt * 60)` (very fast — pure visual indicator)

### 5.5 `viewport.js` extension

Adds `initInsetRenderer(insetCanvas)` and `renderInset(insetScene, insetCamera)` for the small corner canvas. The inset uses its own `THREE.WebGLRenderer` so it doesn't share scissor state with the main viewports.

### 5.6 `models.js`

Two factory functions, both returning `THREE.Group`:

`makeFwModel()` — composite of:
- Fuselage: `BoxGeometry(8, 1.5, 1.5)` (long, narrow box)
- Main wings: `BoxGeometry(2, 0.3, 12)` mounted at fuselage midpoint
- Tail vertical: `BoxGeometry(1.5, 2.5, 0.2)` at tail
- Tail horizontal: `BoxGeometry(2, 0.2, 4)` at tail
- All children share the same material so transparency can be set on the group

`makeDroneModel()` — composite of:
- Central body: `BoxGeometry(0.6, 0.3, 0.6)`
- 4 arms: `BoxGeometry(2, 0.1, 0.1)` rotated to form an X
- 4 rotor discs: `CircleGeometry(0.5, 16)` at the end of each arm, oriented horizontally
- Returns `{ group, rotorMeshes }` so caller can spin the rotors

Model scales chosen so they fill the inset frame nicely when viewed from 30 m away. (FW span 12 m, drone span ~2 m — realistic ratio for an Edge 540 vs a 0.468 kg quad.)

### 5.7 `main.js` updates

Add ~10 lines:
- Mount cockpit overlay (passing a `getFwState` closure)
- Mount drone overlay (passing a `getDroneState` closure)
- Init inset renderer with the inset canvas
- Build inset scene once
- Each frame: call `updateInset(...)` then `renderInset(...)`

### 5.8 `index.html` updates

Add three elements inside `<body>`:
```html
<div id="overlay-cockpit"></div>
<div id="overlay-drone"></div>
<canvas id="inset"></canvas>
```

### 5.9 `styles.css` updates

- `#overlay-cockpit { position: fixed; left: 0; top: 0; width: 50vw; height: 100vh; pointer-events: none; }`
- `#overlay-drone { position: fixed; left: 50vw; top: 0; width: 50vw; height: 100vh; pointer-events: none; }`
- `#inset { position: fixed; right: 12px; bottom: 50px; width: 280px; height: 200px; border: 2px solid rgba(255,255,255,0.4); border-radius: 8px; background: rgba(0,0,0,0.6); }`
- Cockpit/drone internal styling (top band, side rails, bottom strip)

## 6. Testing

- **No new unit tests** — all changes are presentational.
- **Existing tests must pass**: 12 Vitest unit tests + 1 Playwright e2e (geodesic < 1°).
- **One new Playwright assertion**: `expect(await page.locator('#inset').count()).toBe(1);` after page load, plus `expect(await page.evaluate(() => document.querySelector('#inset').getContext('webgl') !== null)).toBe(true);` to confirm the inset Canvas got a WebGL context.

## 7. Acceptance

| # | Criterion | How verified |
|---|---|---|
| 1 | Both POVs render mountains + sea + sky | Manual screenshot |
| 2 | Cockpit overlay visible on left half, drone overlay on right | Manual + DOM check |
| 3 | Inset shows FW (transparent) + drone (opaque) updating live | Manual; e2e checks element existence |
| 4 | Two POVs still pixel-similar in the camera image | Existing e2e geodesic < 1° |
| 5 | Frame rate stays ≥ 30 FPS on integrated GPU | Manual |

## 8. Effort

| Day | Work |
|---|---|
| 1 morning | New scene.js (mountains + sea + animated water); confirm POVs render |
| 1 mid | Cockpit + drone CSS/HTML overlays; live telemetry binding |
| 1 afternoon | Inset canvas, scene, models, orbit camera, rotor animation |
| 1 end | Tune scales, retest, commit, push |

~1 day end-to-end.

## 9. Risks & mitigations

| Risk | Mitigation |
|---|---|
| 3 WebGL contexts dip FPS on integrated GPUs | Inset is tiny (280×200 ≈ 56k px vs main ~2M px); +3% pixel cost. Acceptable. |
| Mountain triangle count blows up | 60 cones × 10 tris = 600 tris. Trivial. |
| CSS overlays misalign after resize | Use `vw`/`vh` units + `position: fixed`. Resize handler already in place. |
| Sea shader performance | Use simple `MeshPhongMaterial` with UV-offset shader (~10 LOC) — not full reflections/refractions. |
| Inset Three.js context creation can fail on systems with WebGL context limit | Catch error in `initInsetRenderer`, fall back to hiding the inset DOM element. |

## 10. Explicit scope cuts

- ❌ Real flight instruments (artificial horizon, attitude indicator, vertical speed gauge)
- ❌ Trees, buildings, runways
- ❌ Birds, weather, time-of-day cycle
- ❌ Loadable GLTF aircraft models
- ❌ Water reflections, refractions, foam
- ❌ Mountain shadows on water
- ❌ Engine sound
- ❌ Mobile/touch overlay variants
- ❌ Overlay toggle key (could be a follow-up if useful)

## 11. References

- Existing demo spec: `docs/superpowers/specs/2026-05-16-ghost-tracking-game-design.md`
- Existing demo plan: `docs/superpowers/plans/2026-05-16-ghost-tracking-game.md`
- Existing code: `web_demo/`
