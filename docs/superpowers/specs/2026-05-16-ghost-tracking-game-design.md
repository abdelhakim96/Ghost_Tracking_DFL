# Ghost-Tracking Interactive Web Demo — Design Spec

**Date:** 2026-05-16
**Owner:** abdelhakim96
**Status:** Approved (pending user review of this written spec)

## 1. Purpose

Build a single-page web app that lets a visitor pilot a fixed-wing aircraft (pitch, roll, yaw, thrust) and see, side-by-side in real time:

- **Left**: the camera POV from the fixed-wing aircraft (FW) itself
- **Right**: the camera POV from the multicopter+gimbal stack tracking the FW

When the DFL controller works (which the offline analysis already proved to ~10⁻⁴ deg orientation error), the two views are pixel-similar. That visual identity *is* the demo.

Audience: reviewers, students, collaborators. Distributed as a public URL — no install, no MATLAB.

## 2. Stack & Hosting

| Concern | Choice | Reason |
| --- | --- | --- |
| Platform | Browser (vanilla JS + Three.js) | Shareable URL, no install |
| Build | None (plain HTML/JS) | Lower friction, easier to debug, simpler CI |
| 3D | Three.js (CDN-loaded) | De-facto WebGL standard, well-documented |
| Tests | Vitest (Node) + Playwright (browser smoke) | Lightweight, no jsx/ts complexity |
| Hosting | GitHub Pages, auto-deploy on `master` push | Free, public, zero config |
| URL | `https://abdelhakim96.github.io/Ghost_Tracking_DFL/` | Existing repo |

**Decided:** no TypeScript, no Vite, no React. The complexity bar for this is "one HTML page + a few JS modules." A build step would not pay back.

## 3. Architecture

```
web_demo/
├── index.html                      single page; two <canvas> + a small key-mapping cheatsheet
├── styles.css
├── main.js                         entry: wires sim + render loops
├── sim/
│   ├── fw_dynamics.js              port of fw_6dof_quat.m
│   ├── drone_dynamics.js           port of quadrotor_dynamics_realtime3.m
│   ├── dfl_controller.js           port of dfl_controller3.m
│   ├── alpha_beta.js               auto-translated alpha_gimbal_func3 / beta_gimbal_func3
│   ├── unified_step.js             one fixed-step RK4 advance of the whole state
│   └── quat.js                     quat_mul, quat_conj, vec_to_quat, quat_to_R
├── render/
│   ├── scene.js                    shared Three.js scene (sky, ground, ref cubes)
│   ├── cameras.js                  the two THREE.PerspectiveCameras, position+quat each frame
│   └── viewport.js                 scissor + render both halves into one canvas
├── input/
│   └── keyboard.js                 smooth-ramp control inputs
└── tests/
    ├── fw_dynamics.test.js
    ├── dfl_controller.test.js
    ├── alpha_beta.test.js
    ├── quat.test.js
    └── e2e_smoke.spec.js           Playwright; loads page, drives keys, checks no error
```

### Loop structure

- **Physics loop**: fixed 5 ms step (200 Hz), advance via RK4. Decoupled from display so transient maneuvers stay accurate at any frame rate.
- **Render loop**: requestAnimationFrame (typically 60 Hz). Pull latest state, write into camera transforms, render both viewports.
- **Input loop**: handled inside the physics step. Key state is sampled each step, smooth-ramped to a desired-deflection, fed into FW controls.

### Data flow (per physics step)

```
keyboard.update(dt)              -> control_inputs {thrust, elevator, aileron, rudder}
fw_dynamics.step(fw_state, ci, dt) -> new fw_state                (13 components)
dfl_controller.compute(drone_state, fw_state, fw_state_derivs)
                                 -> u (7 inputs)
drone_dynamics.step(drone_state, u, dt) -> new drone_state        (18 components)
```

### Data flow (per render frame)

```
cam_fw.position    = fw_state.pos
cam_fw.quaternion  = fw_state.quat (x) q_cam_mount        // q_cam_mount aligns the
                                                          // Three.js -Z forward axis
                                                          // with FW body +X
cam_drone.position    = drone_state.pos + R(q_M) * t_G    // t_G = 0 in our setup
cam_drone.quaternion  = drone_state.quat (x) gimbal_quat(drone_state)
                                                  (x) q_cam_mount

viewport.render_left (cam_fw)
viewport.render_right(cam_drone)
```

## 4. Scene

- **Sky**: linear gradient background (blue at zenith, lighter at horizon)
- **Ground**: large plane (e.g. 10 km × 10 km) with a regular grid texture (50 m squares, brown/green colors). Gives unambiguous yaw and pitch cues at altitude.
- **Reference geometry**: 20-30 colored cubes/spheres scattered at known XYZ positions over a 5 km radius, sizes chosen so they're recognizable from typical FW flight altitudes (~100-300 m AGL).
- **Sun light**: one directional + ambient.

No skybox, no terrain, no aircraft model. The two cameras render the *same* scene, so the visual comparison is pixel-identical when poses match.

## 5. Controls (keyboard)

| Key | Action | Mapping |
| --- | --- | --- |
| `↑` / `↓` | Elevator (pitch down/up) | held → ramp deflection to ±0.4 rad over 0.3 s; release → decay to 0 over 0.3 s |
| `←` / `→` | Aileron (roll left/right) | ramp to ±0.4 rad |
| `A` / `D` | Rudder | ramp to ±0.2 rad |
| `W` / `S` | Thrust up/down | ramp 0…200 N |
| `R` | Reset simulation to initial state | instant |
| `P` | Pause / resume | toggle |

Smooth-ramp gives keyboard input a stick-like feel. Coefficients chosen so the FW responds in 0.5-1 s, which matches the existing Edge 540 dynamics.

## 6. Testing

### Unit tests (Vitest, Node)

- `quat.test.js`: round-trip identities; `q ⊗ q⁻¹ = 1`; `R(q ⊗ p) = R(q) R(p)`.
- `fw_dynamics.test.js`: 30 random (state, input) pairs evaluated against MATLAB's `fw_6dof_quat`. Relative error per state derivative < 1e-5.
- `alpha_beta.test.js`: 30 sampled drone states; compare every element of α and β against MATLAB. Relative error < 1e-6 (auto-translated symbolic expressions should be exact mod floating-point).
- `dfl_controller.test.js`: full `u` output vs MATLAB at 30 (drone_state, fw_state) pairs.

### Browser integration (Playwright)

- `e2e_smoke.spec.js`: open page, send keystrokes ("↑" for 1 s, "↓" for 1 s), wait 3 s no-input. Read `window.__demoState` (exposed for tests), assert geodesic camera-orientation error < 1°.

### Acceptance criteria

1. Both viewports render the scene continuously at ≥ 30 FPS on a 2020-era laptop's integrated graphics.
2. Page loads in < 5 s on a typical broadband connection (~5 MB JS budget total).
3. Pilot freely for 30 s without simulation divergence (no NaN states).
4. Visual identity: with default tracking on, the two viewports are visually identical (subjective for live demo, < 1° geodesic for the test).

## 7. Deployment

- GitHub Actions workflow on push to `master`:
  - copy `web_demo/` to `gh-pages` branch root
  - GitHub Pages serves it
- Estimated 5 lines of YAML.
- README in `web_demo/` with screenshot, key-mapping table, link to live URL.

## 8. Effort estimate

| Day | Work |
| --- | --- |
| 1 | Port FW + DFL + drone dynamics to JS. Write unit tests against MATLAB samples. |
| 2 | Three.js scene + dual-camera render + keyboard input + minimal HUD. |
| 3 | GitHub Pages CI, README with screenshot, polish, Playwright smoke test. |

3 days for a polished MVP that meets all acceptance criteria.

## 9. Risks

| Risk | Likelihood | Mitigation |
| --- | --- | --- |
| Numerical drift between JS port and MATLAB (different op order) | Medium | Unit-test against MATLAB samples; fix per-function bias if found. |
| 60 FPS dip on integrated graphics with 30+ scene objects | Low | Use `scissor` instead of two render passes; reduce reference object count. |
| FW dynamics open-loop divergence under aggressive pilot input | Medium | Add the same `fw_attitude_hold` autopilot as a "training wheels" toggle. |
| Public hosting → someone hostlinks the assets | Low | GitHub Pages bandwidth limits are generous; not a concern at expected traffic. |

## 10. Explicitly out of scope

- Networked multiplayer / shared sessions
- Save / replay of sessions
- Multiple aircraft types — Edge 540 only
- Wind, turbulence, atmospheric effects
- Stall / over-G modeling
- Live controller-tuning UI
- Mobile / touchscreen support
- Audio
- Skybox / weather

## 11. References

- `dfl_analysis.md` — context for what the controller does and why
- `ECC_2026/root.tex` — the paper
- `models/fw_6dof_quat.m`, `DFL_controller/dfl_controller3.m` — the MATLAB ground truth being ported
- `models/fw_attitude_hold.m` — the "training wheels" autopilot from the Immelmann work, also portable
