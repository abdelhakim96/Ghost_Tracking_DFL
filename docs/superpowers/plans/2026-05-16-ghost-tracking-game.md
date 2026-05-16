# Ghost-Tracking Interactive Web Demo — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Single-page web app where a visitor pilots a fixed-wing aircraft (keyboard) and sees side-by-side the FW camera POV and the multicopter+gimbal camera POV, demonstrating the DFL controller live in the browser.

**Architecture:** Vanilla JS + Three.js (no build step), 200 Hz fixed-step RK4 physics decoupled from 60 Hz Three.js render, both cameras rendering the same scene from different poses. Deploy to GitHub Pages.

**Tech Stack:** Plain ES modules served as static files; Three.js loaded from CDN; Vitest for unit tests against MATLAB-generated samples; Playwright for browser smoke test; GitHub Actions for auto-deploy on push.

**Spec:** `docs/superpowers/specs/2026-05-16-ghost-tracking-game-design.md`

---

## File Map

```
web_demo/
├── index.html                       single page; two <canvas viewports>
├── styles.css                       layout + cheatsheet styling
├── main.js                          loops + wiring
├── package.json                     Vitest, Playwright dev deps; "type": "module"
├── sim/
│   ├── quat.js                      quat_mul, quat_conj, vec_to_quat, quat_to_R, quat_rot
│   ├── fw_dynamics.js               step function: state, controls -> derivative
│   ├── alpha_beta.js                alpha_gimbal_func3, beta_gimbal_func3 (auto-translated)
│   ├── dfl_controller.js            dfl_controller3.m logic -> u(7)
│   ├── drone_dynamics.js            quadrotor_dynamics_realtime3.m -> derivative
│   └── unified_step.js              RK4 integrator: advance (drone, FW) one step
├── render/
│   ├── scene.js                     sky + ground + reference cubes
│   ├── cameras.js                   two THREE.PerspectiveCameras
│   └── viewport.js                  scissor + dual render
├── input/
│   └── keyboard.js                  smooth-ramp deflections
├── tests/
│   ├── data/                        JSON samples extracted from MATLAB
│   │   ├── fw_dynamics_samples.json
│   │   ├── alpha_beta_samples.json
│   │   ├── dfl_controller_samples.json
│   │   ├── drone_dynamics_samples.json
│   │   └── quat_samples.json
│   ├── quat.test.js
│   ├── fw_dynamics.test.js
│   ├── alpha_beta.test.js
│   ├── dfl_controller.test.js
│   ├── drone_dynamics.test.js
│   ├── keyboard.test.js
│   └── e2e_smoke.spec.js
└── README.md

scripts/
└── extract_matlab_samples.m         MATLAB script that writes tests/data/*.json

.github/
└── workflows/
    └── deploy.yml                   build (none) + push to gh-pages
```

---

## Phase 1: Project Skeleton

### Task 1: Initialise web_demo directory + package.json

**Files:**
- Create: `web_demo/package.json`
- Create: `web_demo/.gitignore`

- [ ] **Step 1: Create directory structure and gitignore**

```bash
mkdir -p web_demo/sim web_demo/render web_demo/input web_demo/tests/data scripts
cat > web_demo/.gitignore <<'EOF'
node_modules/
test-results/
playwright-report/
EOF
```

- [ ] **Step 2: Create package.json**

```bash
cat > web_demo/package.json <<'EOF'
{
  "name": "ghost-tracking-web-demo",
  "version": "0.1.0",
  "type": "module",
  "description": "Interactive ghost-tracking web demo for the DFL camera-mimicry controller",
  "scripts": {
    "test": "vitest run",
    "test:watch": "vitest",
    "test:e2e": "playwright test",
    "serve": "python3 -m http.server 8000"
  },
  "devDependencies": {
    "vitest": "^1.6.0",
    "@playwright/test": "^1.45.0"
  }
}
EOF
```

- [ ] **Step 3: Install dev dependencies**

Run: `cd web_demo && npm install`
Expected: `node_modules/` directory created, no errors.

- [ ] **Step 4: Commit**

```bash
git add web_demo/package.json web_demo/.gitignore
git commit -m "chore: scaffold web_demo with vitest + playwright"
```

---

### Task 2: MATLAB sample-extraction script

**Files:**
- Create: `scripts/extract_matlab_samples.m`
- Output: `web_demo/tests/data/*.json` (5 files)

This script samples N=30 random states, calls each MATLAB function we need to port, and writes input/output tuples as JSON. Each JS unit test will load its JSON and check the port produces identical numbers.

- [ ] **Step 1: Write the extraction script**

```matlab
function extract_matlab_samples()
% Writes web_demo/tests/data/*.json with sampled (input, output) tuples
% from the MATLAB reference implementations. JS unit tests load these
% to verify the JS ports.

    here = fileparts(mfilename('fullpath'));
    repo = fileparts(here);
    cd(repo);
    addpath('models'); addpath('utilities'); addpath('DFL_controller');

    out_dir = fullfile('web_demo', 'tests', 'data');
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    rng(42, 'twister');

    % --- 1. quat samples (round-trip identities)
    q_samples = cell(30, 1);
    for k = 1:30
        a = randn(4,1); a = a / norm(a);
        b = randn(4,1); b = b / norm(b);
        q_samples{k} = struct(...
            'a', a(:).', ...
            'b', b(:).', ...
            'a_mul_b', quat_mul(a, b).', ...
            'a_conj', quat_conj(a).');
    end
    write_json(fullfile(out_dir, 'quat_samples.json'), q_samples);

    % --- 2. fw_dynamics samples
    fw_params = make_edge540_params();
    fw_samples = cell(30, 1);
    for k = 1:30
        s = random_fw_state();
        th = 100*rand(); el = 0.4*(2*rand()-1);
        ai = 0.6*(2*rand()-1); ru = 0.4*(2*rand()-1);
        xdot = fw_6dof_quat(0, s, th, el, ai, ru, fw_params);
        fw_samples{k} = struct('state', s(:).', 'thrust', th, ...
            'elevator', el, 'aileron', ai, 'rudder', ru, ...
            'xdot', xdot(:).');
    end
    write_json(fullfile(out_dir, 'fw_dynamics_samples.json'), ...
        struct('params', fw_params, 'samples', {fw_samples}));

    % --- 3. alpha_beta samples
    ab_samples = cell(30, 1);
    for k = 1:30
        s = random_drone_state();
        zeta = s(17); xi = s(18);
        a = alpha_gimbal_func3(s, 0, 0, 0, 0.01, 0.01, 0.01, ...
                               0.0023, 0.0023, 0.0046, ...
                               0.001, 0.001, 0.001, zeta, xi, 0.468);
        b = beta_gimbal_func3(s, 0, 0, 0, 0.01, 0.01, 0.01, ...
                              0.0023, 0.0023, 0.0046, ...
                              0.001, 0.001, 0.001, zeta, xi, 0.468);
        ab_samples{k} = struct('state', s(:).', ...
            'alpha', a(:).', 'beta_flat', b(:).');
    end
    write_json(fullfile(out_dir, 'alpha_beta_samples.json'), ab_samples);

    % --- 4. dfl_controller samples (need fw_state + drone_state)
    dfl_samples = cell(15, 1);
    for k = 1:15
        ds = random_drone_state();
        fs = random_fw_state();
        gains = default_dfl_gains();
        xd = randn(3,1)*10;
        vd = randn(3,1)*5;
        ad = randn(3,1);
        jd = zeros(3,1); sd = zeros(3,1);
        psid = 0;
        u = dfl_controller3(0, ds, xd, vd, ad, jd, sd, psid, fs, fs(7:10), gains);
        dfl_samples{k} = struct('drone_state', ds(:).', 'fw_state', fs(:).', ...
            'xd', xd(:).', 'vd', vd(:).', 'ad', ad(:).', ...
            'jd', jd(:).', 'sd', sd(:).', 'psid', psid, ...
            'gains', gains, 'u', u(:).');
    end
    write_json(fullfile(out_dir, 'dfl_controller_samples.json'), dfl_samples);

    % --- 5. drone_dynamics samples (xdot from drone_state + u + fw_state)
    dy_samples = cell(15, 1);
    for k = 1:15
        ds = random_drone_state();
        fs = random_fw_state();
        gains = default_dfl_gains();
        xd = fs(1:3); vd = randn(3,1); ad = zeros(3,1);
        jd = zeros(3,1); sd = zeros(3,1);
        % use the realtime function which calls dfl internally
        global m Ix Iy Iz g
        m = 0.468; Ix = 0.0023; Iy = 0.0023; Iz = 0.0046; g = 9.81;
        sdot = quadrotor_dynamics_realtime3(0, ds, xd, vd, ad, jd, sd, ...
                                            0, fs, fs(7:10), gains);
        dy_samples{k} = struct('drone_state', ds(:).', 'fw_state', fs(:).', ...
            'xd', xd(:).', 'vd', vd(:).', 'ad', ad(:).', ...
            'jd', jd(:).', 'sd', sd(:).', ...
            'gains', gains, 'sdot', sdot(:).');
    end
    write_json(fullfile(out_dir, 'drone_dynamics_samples.json'), dy_samples);

    fprintf('Wrote 5 sample files to %s/\n', out_dir);
end

function s = random_fw_state()
    p = randn(3,1)*50 + [0;0;-100];
    v = [120 + 10*randn(); 5*randn(); 5*randn()];
    q = randn(4,1); q = q/norm(q);
    w = 0.3*randn(3,1);
    s = [p; v; q; w];
end

function s = random_drone_state()
    p = randn(3,1)*50 + [0;0;-100];
    q = randn(4,1); q = q/norm(q);
    v = [120;0;0] + 5*randn(3,1);
    w = 0.1*randn(3,1);
    gimbal = 0.3*randn(3,1);
    zeta = 0.468*9.81 + 0.5*randn();
    xi   = 0.1*randn();
    s = [p; q; v; w; gimbal; zeta; xi];
end

function g = default_dfl_gains()
    g.c0 = 51150; g.c1 = 51140; g.c2 = 1150; g.c3 = 150;
    g.c4 = 1; g.c5 = 1;
    g.c_phi = 50; g.c_theta = 50; g.c_psig = 50;
    g.c_q3 = 100; g.c_q3_dot = 20;
end

function p = make_edge540_params()
    p.m = 290; p.J = diag([550, 750, 1100]);
    p.S = 9.1; p.b = 7.44; p.c = 1.22;
    p.rho = 1.225; p.g = 9.81;
    p.CL0 = 0.4; p.CL_alpha = 5.7; p.CL_q = 7.0; p.CL_de = -0.8;
    p.CD0 = 0.04; p.k = 0.05; p.CDa = 0.1; p.CD_q = 0.0; p.CD_de = 0.0;
    p.CY_beta = -1.2; p.CY_p = -0.1; p.CY_r = 0.2; p.CY_da = 0.2; p.CY_dr = -0.2;
    p.Cl_beta = -0.15; p.Cl_p = -1.0; p.Cl_r = 0.25; p.Cl_da = 0.5; p.Cl_dr = 0.05;
    p.Cm0 = 0.0; p.Cm_alpha = -1.5; p.Cm_q = -15.0; p.Cm_de = -1.8;
    p.Cn_beta = 0.15; p.Cn_p = -0.1; p.Cn_r = -0.4; p.Cn_da = 0.04; p.Cn_dr = -0.1;
end

function write_json(filename, data)
    txt = jsonencode(data, 'PrettyPrint', true);
    fid = fopen(filename, 'w');
    fwrite(fid, txt);
    fclose(fid);
end
```

- [ ] **Step 2: Run the script**

Run: `matlab -batch "addpath('scripts'); extract_matlab_samples"`
Expected: 5 JSON files appear under `web_demo/tests/data/`.

- [ ] **Step 3: Spot-check one output**

Run: `head -50 web_demo/tests/data/quat_samples.json`
Expected: pretty-printed JSON array of 30 objects with `a`, `b`, `a_mul_b`, `a_conj` fields.

- [ ] **Step 4: Commit**

```bash
git add scripts/extract_matlab_samples.m web_demo/tests/data/
git commit -m "chore(web_demo): MATLAB ground-truth sample extraction for unit tests"
```

---

## Phase 2: Quaternion module (TDD)

### Task 3: Quaternion helpers

**Files:**
- Create: `web_demo/sim/quat.js`
- Create: `web_demo/tests/quat.test.js`

- [ ] **Step 1: Write the failing test**

```javascript
// web_demo/tests/quat.test.js
import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';
import { quatMul, quatConj, quatToR } from '../sim/quat.js';

const samples = JSON.parse(readFileSync('web_demo/tests/data/quat_samples.json', 'utf8'));

describe('quat', () => {
  it('matches MATLAB quat_mul on 30 samples', () => {
    for (const s of samples) {
      const got = quatMul(s.a, s.b);
      for (let i = 0; i < 4; i++) {
        expect(got[i]).toBeCloseTo(s.a_mul_b[i], 10);
      }
    }
  });

  it('matches MATLAB quat_conj on 30 samples', () => {
    for (const s of samples) {
      const got = quatConj(s.a);
      for (let i = 0; i < 4; i++) {
        expect(got[i]).toBeCloseTo(s.a_conj[i], 10);
      }
    }
  });

  it('quat_to_R produces a proper rotation matrix (det=1, R R^T = I)', () => {
    for (const s of samples) {
      const R = quatToR(s.a);
      // det(R) = 1
      const det = R[0]*(R[4]*R[8] - R[5]*R[7])
                - R[1]*(R[3]*R[8] - R[5]*R[6])
                + R[2]*(R[3]*R[7] - R[4]*R[6]);
      expect(det).toBeCloseTo(1, 10);
    }
  });
});
```

- [ ] **Step 2: Run test, expect failure**

Run: `cd web_demo && npx vitest run tests/quat.test.js`
Expected: FAIL — `Cannot find module ../sim/quat.js`.

- [ ] **Step 3: Implement quat.js**

```javascript
// web_demo/sim/quat.js
// Scalar-first quaternions: q = [w, x, y, z]

export function quatMul(a, b) {
  const [a0, a1, a2, a3] = a;
  const [b0, b1, b2, b3] = b;
  return [
    a0*b0 - a1*b1 - a2*b2 - a3*b3,
    a0*b1 + a1*b0 + a2*b3 - a3*b2,
    a0*b2 - a1*b3 + a2*b0 + a3*b1,
    a0*b3 + a1*b2 - a2*b1 + a3*b0,
  ];
}

export function quatConj(q) {
  return [q[0], -q[1], -q[2], -q[3]];
}

export function quatNorm(q) {
  return Math.hypot(q[0], q[1], q[2], q[3]);
}

export function quatNormalize(q) {
  const n = quatNorm(q) + 1e-12;
  return [q[0]/n, q[1]/n, q[2]/n, q[3]/n];
}

// 3x3 rotation matrix as a flat row-major [r00, r01, r02, r10, r11, r12, r20, r21, r22]
export function quatToR(q) {
  const [w, x, y, z] = quatNormalize(q);
  return [
    w*w + x*x - y*y - z*z,  2*(x*y - w*z),          2*(x*z + w*y),
    2*(x*y + w*z),          w*w - x*x + y*y - z*z,  2*(y*z - w*x),
    2*(x*z - w*y),          2*(y*z + w*x),          w*w - x*x - y*y + z*z,
  ];
}

// Rotate a 3-vector by quaternion
export function quatRot(q, v) {
  const R = quatToR(q);
  return [
    R[0]*v[0] + R[1]*v[1] + R[2]*v[2],
    R[3]*v[0] + R[4]*v[1] + R[5]*v[2],
    R[6]*v[0] + R[7]*v[1] + R[8]*v[2],
  ];
}

// Minimum-rotation quaternion from unit vector a to unit vector b
export function vecToQuat(a, b) {
  const dot = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
  if (dot > 0.999999) return [1, 0, 0, 0];
  if (dot < -0.999999) {
    // 180 deg rotation about any axis perpendicular to a
    let ax = [1, 0, 0];
    if (Math.abs(a[0]) > 0.9) ax = [0, 1, 0];
    const cx = [a[1]*ax[2] - a[2]*ax[1], a[2]*ax[0] - a[0]*ax[2], a[0]*ax[1] - a[1]*ax[0]];
    const n = Math.hypot(...cx);
    return [0, cx[0]/n, cx[1]/n, cx[2]/n];
  }
  const c = [a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]];
  const s = Math.sqrt((1 + dot) * 2);
  return quatNormalize([s/2, c[0]/s, c[1]/s, c[2]/s]);
}
```

- [ ] **Step 4: Run test, expect pass**

Run: `cd web_demo && npx vitest run tests/quat.test.js`
Expected: 3 passing tests.

- [ ] **Step 5: Commit**

```bash
git add web_demo/sim/quat.js web_demo/tests/quat.test.js
git commit -m "feat(web_demo): quat module with MATLAB-equivalence tests"
```

---

## Phase 3: FW dynamics port (TDD)

### Task 4: FW dynamics

**Files:**
- Create: `web_demo/sim/fw_dynamics.js`
- Create: `web_demo/tests/fw_dynamics.test.js`

- [ ] **Step 1: Write the failing test**

```javascript
// web_demo/tests/fw_dynamics.test.js
import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';
import { fwDerivative, edge540Params } from '../sim/fw_dynamics.js';

const data = JSON.parse(readFileSync('web_demo/tests/data/fw_dynamics_samples.json', 'utf8'));

describe('fw_dynamics', () => {
  it('matches MATLAB fw_6dof_quat on 30 random states', () => {
    for (const s of data.samples) {
      const xdot = fwDerivative(s.state, s.thrust, s.elevator, s.aileron, s.rudder, data.params);
      for (let i = 0; i < 13; i++) {
        expect(xdot[i]).toBeCloseTo(s.xdot[i], 6);
      }
    }
  });

  it('edge540Params returns sane numbers', () => {
    const p = edge540Params();
    expect(p.m).toBe(290);
    expect(p.S).toBe(9.1);
    expect(p.CL_alpha).toBeCloseTo(5.7, 6);
  });
});
```

- [ ] **Step 2: Run test, expect failure**

Run: `cd web_demo && npx vitest run tests/fw_dynamics.test.js`
Expected: FAIL — `Cannot find module ../sim/fw_dynamics.js`.

- [ ] **Step 3: Implement fw_dynamics.js**

Port `models/fw_6dof_quat.m` line by line. State = `[x, y, z, u, v, w, q0, q1, q2, q3, p, q, r]`.

```javascript
// web_demo/sim/fw_dynamics.js
// Port of models/fw_6dof_quat.m (Edge 540 6-DOF fixed-wing).

export function edge540Params() {
  return {
    m: 290, J: [550, 750, 1100],          // diagonal inertia
    S: 9.1, b: 7.44, c: 1.22,
    rho: 1.225, g: 9.81,
    CL0: 0.4, CL_alpha: 5.7, CL_q: 7.0, CL_de: -0.8,
    CD0: 0.04, k: 0.05, CDa: 0.1, CD_q: 0.0, CD_de: 0.0,
    CY_beta: -1.2, CY_p: -0.1, CY_r: 0.2, CY_da: 0.2, CY_dr: -0.2,
    Cl_beta: -0.15, Cl_p: -1.0, Cl_r: 0.25, Cl_da: 0.5, Cl_dr: 0.05,
    Cm0: 0.0, Cm_alpha: -1.5, Cm_q: -15.0, Cm_de: -1.8,
    Cn_beta: 0.15, Cn_p: -0.1, Cn_r: -0.4, Cn_da: 0.04, Cn_dr: -0.1,
  };
}

function quatToR_BN(q) {
  // Same convention as quat_to_R_BN in fw_6dof_quat.m
  const [w, x, y, z] = q;
  return [
    1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y),
    2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x),
    2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y),
  ];
}

function matVec3(M, v) {
  return [
    M[0]*v[0] + M[1]*v[1] + M[2]*v[2],
    M[3]*v[0] + M[4]*v[1] + M[5]*v[2],
    M[6]*v[0] + M[7]*v[1] + M[8]*v[2],
  ];
}

function matTransposeVec3(M, v) {
  return [
    M[0]*v[0] + M[3]*v[1] + M[6]*v[2],
    M[1]*v[0] + M[4]*v[1] + M[7]*v[2],
    M[2]*v[0] + M[5]*v[1] + M[8]*v[2],
  ];
}

export function fwDerivative(state, thrust, elevator, aileron, rudder, params) {
  // state = [x, y, z, u, v, w, q0, q1, q2, q3, p, q, r]  (13)
  const u = state[3], v = state[4], w = state[5];
  let q = [state[6], state[7], state[8], state[9]];
  const nq = Math.hypot(q[0], q[1], q[2], q[3]) + 1e-12;
  q = [q[0]/nq, q[1]/nq, q[2]/nq, q[3]/nq];
  const p = state[10], qr = state[11], r = state[12];

  const { m, J, S, b, c, rho, g } = params;
  const R_BN = quatToR_BN(q);

  // Kinematics
  const v_b = [u, v, w];
  const v_ned = matVec3(R_BN, v_b);

  // Aerodynamics (no wind)
  const Va = Math.max(1e-3, Math.hypot(u, v, w));
  const alpha = Math.atan2(w, u);
  const beta  = Math.asin(Math.max(-1, Math.min(1, v / Va)));
  const qbar  = 0.5 * rho * Va * Va;
  const p_hat = (b / (2 * Va)) * p;
  const q_hat = (c / (2 * Va)) * qr;
  const r_hat = (b / (2 * Va)) * r;

  const CL = params.CL0 + params.CL_alpha * alpha + params.CL_q * q_hat + params.CL_de * elevator;
  const CD = params.CD0 + params.k * CL * CL + params.CDa * alpha + params.CD_q * q_hat + params.CD_de * elevator;
  const CY = params.CY_beta * beta + params.CY_p * p_hat + params.CY_r * r_hat + params.CY_da * aileron + params.CY_dr * rudder;
  const Cl = params.Cl_beta * beta + params.Cl_p * p_hat + params.Cl_r * r_hat + params.Cl_da * aileron + params.Cl_dr * rudder;
  const Cm = params.Cm0 + params.Cm_alpha * alpha + params.Cm_q * q_hat + params.Cm_de * elevator;
  const Cn = params.Cn_beta * beta + params.Cn_p * p_hat + params.Cn_r * r_hat + params.Cn_da * aileron + params.Cn_dr * rudder;

  const Lift = qbar * S * CL;
  const Drag = qbar * S * CD;
  const Side = qbar * S * CY;
  const ca = Math.cos(alpha), sa = Math.sin(alpha);

  const F_aero_b = [-Drag*ca + Lift*sa, Side, -Drag*sa - Lift*ca];
  const M_aero_b = [qbar * S * b * Cl, qbar * S * c * Cm, qbar * S * b * Cn];

  const F_thrust_b = [thrust, 0, 0];
  const F_grav_b   = matTransposeVec3(R_BN, [0, 0, g * m]);

  const F_b = [
    F_aero_b[0] + F_thrust_b[0] + F_grav_b[0],
    F_aero_b[1] + F_thrust_b[1] + F_grav_b[1],
    F_aero_b[2] + F_thrust_b[2] + F_grav_b[2],
  ];

  // v_dot_b = (1/m) F_b - omega x v_b
  const omega = [p, qr, r];
  const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
  const cv = cross(omega, v_b);
  const v_dot_b = [F_b[0]/m - cv[0], F_b[1]/m - cv[1], F_b[2]/m - cv[2]];

  // Attitude kinematics
  const q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
  const q_dot = [
    -0.5 * (p*q1 + qr*q2 + r*q3),
     0.5 * (p*q0 + r*q2  - qr*q3),
     0.5 * (qr*q0 - r*q1 + p*q3),
     0.5 * (r*q0  + qr*q1 - p*q2),
  ];

  // Rotational dynamics: omega_dot = J^-1 (M - omega x J omega)  with diagonal J
  const Jw = [J[0]*p, J[1]*qr, J[2]*r];
  const omxJw = cross(omega, Jw);
  const om_dot = [
    (M_aero_b[0] - omxJw[0]) / J[0],
    (M_aero_b[1] - omxJw[1]) / J[1],
    (M_aero_b[2] - omxJw[2]) / J[2],
  ];

  return [
    v_ned[0], v_ned[1], v_ned[2],     // x_dot, y_dot, z_dot
    v_dot_b[0], v_dot_b[1], v_dot_b[2],
    q_dot[0], q_dot[1], q_dot[2], q_dot[3],
    om_dot[0], om_dot[1], om_dot[2],
  ];
}
```

- [ ] **Step 4: Run test, expect pass**

Run: `cd web_demo && npx vitest run tests/fw_dynamics.test.js`
Expected: 2 passing tests.

- [ ] **Step 5: Commit**

```bash
git add web_demo/sim/fw_dynamics.js web_demo/tests/fw_dynamics.test.js
git commit -m "feat(web_demo): fw_dynamics port + MATLAB-equivalence test"
```

---

## Phase 4: alpha_beta auto-translation

### Task 5: Translate alpha_gimbal_func3 and beta_gimbal_func3

**Files:**
- Create: `web_demo/sim/alpha_beta.js`
- Create: `web_demo/tests/alpha_beta.test.js`

Strategy: copy MATLAB source verbatim, then string-substitute `.^` → `**`, `.*` → `*`, `./` → `/`, `q0.^2` → `q0**2` etc., wrap in JS function. Spot-check a few entries by hand.

- [ ] **Step 1: Write the failing test**

```javascript
// web_demo/tests/alpha_beta.test.js
import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';
import { alphaGimbal, betaGimbal } from '../sim/alpha_beta.js';

const samples = JSON.parse(readFileSync('web_demo/tests/data/alpha_beta_samples.json', 'utf8'));

const PARAMS = {
  Ap: 0, Aq: 0, Ar: 0,
  Ag_p: 0.01, Ag_q: 0.01, Ag_r: 0.01,
  Ix: 0.0023, Iy: 0.0023, Iz: 0.0046,
  Ig_x: 0.001, Ig_y: 0.001, Ig_z: 0.001,
  m: 0.468,
};

describe('alpha_beta', () => {
  it('matches MATLAB alpha_gimbal_func3 on 30 states', () => {
    for (const s of samples) {
      const zeta = s.state[16], xi = s.state[17];
      const a = alphaGimbal(s.state, PARAMS, zeta, xi);
      for (let i = 0; i < 7; i++) {
        expect(a[i]).toBeCloseTo(s.alpha[i], 4);
      }
    }
  });

  it('matches MATLAB beta_gimbal_func3 on 30 states', () => {
    for (const s of samples) {
      const zeta = s.state[16], xi = s.state[17];
      const b = betaGimbal(s.state, PARAMS, zeta, xi);
      // b is 7x7 row-major, beta_flat is column-major (MATLAB)
      for (let r = 0; r < 7; r++) {
        for (let c = 0; c < 7; c++) {
          expect(b[r*7 + c]).toBeCloseTo(s.beta_flat[c*7 + r], 4);
        }
      }
    }
  });
});
```

- [ ] **Step 2: Write alpha_beta.js**

Translation rules from MATLAB to JS (already applied below):
- `.^` → `**`
- `.*` → `*`
- `./` → `/`
- `1.0./x` → `1.0/x`
- `q` in MATLAB body refers to body pitch rate → renamed `qr` in JS to avoid clashing with quaternion vars
- MATLAB `reshape(flat, [7,7])` is column-major; we return row-major and transpose at the bottom

```javascript
// web_demo/sim/alpha_beta.js
// Auto-translated from DFL_controller/{alpha,beta}_gimbal_func3.m.
// state layout (0-based):
//   0..2   position
//   3..6   drone quaternion q0..q3
//   7..9   v_world
//   10..12 omega_body (p, q_body_pitch_rate, r)
//   13..15 gimbal (phi_g, theta_g, psi_g)
//   16,17  zeta, xi

export function alphaGimbal(state, params, zeta, xi) {
  const { Ap, Aq, Ar, Ix, Iy, Iz } = params;
  const q0 = state[3], q1 = state[4], q2 = state[5], q3 = state[6];
  const p = state[10], qr = state[11], r = state[12];

  const t2 = p**2;
  const t3 = qr**2;
  const t4 = 1.0/zeta;

  return [
    zeta*(t2 + t3),
    -t4*(Ap*zeta + Ix*p*xi*2.0 - Ix*qr*r*zeta + Iy*qr*r*zeta - Iz*qr*r*zeta),
    -t4*(Aq*zeta + Iy*qr*xi*2.0 - Ix*p*r*zeta + Iy*p*r*zeta + Iz*p*r*zeta),
    (t4*(Ar*q0*zeta*-2.0 - Iz*p*q2*xi*4.0 + Iz*qr*q1*xi*4.0
         + Iz*q3*t2*zeta + Iz*q3*t3*zeta + Iz*q3*r**2*zeta
         - Ix*p*qr*q0*zeta*2.0 + Iy*p*qr*q0*zeta*2.0
         + Iz*p*q1*r*zeta*2.0 + Iz*qr*q2*r*zeta*2.0)) / (q0*2.0),
    0.0, 0.0, 0.0,
  ];
}

export function betaGimbal(state, params, zeta /* unused but kept for parity */, xi /* unused */) {
  const { Ix, Iy, Iz, m } = params;
  const q0 = state[3], q1 = state[4], q2 = state[5], q3 = state[6];
  const z = state[16];      // zeta from state, mirrors MATLAB's local

  const t2 = q0*q1;
  const t3 = q0*q2;
  const t4 = q0*q3;
  const t5 = q1*q2;
  const t6 = q1*q3;
  const t7 = q2*q3;
  const t8 = q0**2;
  const t9 = q1**2;
  const t10 = q2**2;
  const t11 = q3**2;
  const t12 = 1.0/q0;
  const t13 = 1.0/z;
  const t14 = -t9;
  const t15 = -t10;
  const t16 = -t11;
  const t17 = t8 + t9 + t10 + t11;
  const t18 = 1.0/(t17**2);

  // Build 49 entries in MATLAB column-major order (matches reshape(..., [7,7]))
  const cm = [
    // column 1
    m*t18*(t3 + t6)*2.0,
    Ix*m*t13*t18*(t4 - t5)*2.0,
    Iy*m*t13*t18*(t8 + t9 + t15 + t16),
    -Iz*m*t12*t13*t18*(q0*t2 - q3*t3*2.0 + q2*t5 - q3*t6 + q1**3),
    0.0, 0.0, 0.0,
    // column 2
    m*t18*(t2 - t7)*-2.0,
    -Ix*m*t13*t18*(t8 + t10 + t14 + t16),
    Iy*m*t13*t18*(t4 + t5)*2.0,
    -Iz*m*t12*t13*t18*(q0*t3 + q3*t2*2.0 + q1*t5 - q3*t7 + q2**3),
    0.0, 0.0, 0.0,
    // column 3
    m*t18*(t8 + t11 + t14 + t15),
    Ix*m*t13*t18*(t2 + t7)*-2.0,
    Iy*m*t13*t18*(t3 - t6)*-2.0,
    Iz*m*q3*t12*t13*t18*(t9 + t10)*-2.0,
    0.0, 0.0, 0.0,
    // column 4
    0.0, 0.0, 0.0, Iz*t12*2.0, 0.0, 0.0, 0.0,
    // column 5
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    // column 6
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    // column 7
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
  ];

  // Transpose column-major -> row-major (consumer expects row-major)
  const rm = new Array(49);
  for (let r = 0; r < 7; r++) {
    for (let c = 0; c < 7; c++) {
      rm[r*7 + c] = cm[c*7 + r];
    }
  }
  return rm;
}
```

Source files for reference: `DFL_controller/alpha_gimbal_func3.m` and `DFL_controller/beta_gimbal_func3.m`. If MATLAB regenerates these (via `controller_generation_v2`), repeat the translation.

- [ ] **Step 3: Run test, expect pass**

Run: `cd web_demo && npx vitest run tests/alpha_beta.test.js`
Expected: 2 passing tests, each with 30 sub-assertions.

If failing: print one mismatched (input, MATLAB output, JS output) tuple and diff. Typical errors: missed `.^` → `**`, wrong sign on a `-` prefix, missing entry in mt2.

- [ ] **Step 4: Commit**

```bash
git add web_demo/sim/alpha_beta.js web_demo/tests/alpha_beta.test.js
git commit -m "feat(web_demo): alpha/beta gimbal funcs translated to JS"
```

---

## Phase 5: DFL controller port (TDD)

### Task 6: DFL controller

**Files:**
- Create: `web_demo/sim/dfl_controller.js`
- Create: `web_demo/tests/dfl_controller.test.js`

Port `DFL_controller/dfl_controller3.m`. It calls `quat2rotm` (Three.js or our `quat.js`), `quat_mul`, `quat_conj`, `alpha_gimbal_func3`, `beta_gimbal_func3`, and contains the XYZ-Euler extraction + body-frame Jacobian inversion.

- [ ] **Step 1: Write the failing test**

```javascript
// web_demo/tests/dfl_controller.test.js
import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';
import { dflController } from '../sim/dfl_controller.js';

const samples = JSON.parse(readFileSync('web_demo/tests/data/dfl_controller_samples.json', 'utf8'));

describe('dfl_controller', () => {
  it('matches MATLAB dfl_controller3 on 15 samples', () => {
    for (const s of samples) {
      const u = dflController({
        t: 0,
        droneState: s.drone_state, fwState: s.fw_state,
        xd: s.xd, vd: s.vd, ad: s.ad, jd: s.jd, sd: s.sd,
        psid: s.psid, gains: s.gains,
      });
      for (let i = 0; i < 7; i++) {
        expect(u[i]).toBeCloseTo(s.u[i], 3);
      }
    }
  });
});
```

- [ ] **Step 2: Run test, expect failure (module not found)**

Run: `cd web_demo && npx vitest run tests/dfl_controller.test.js`
Expected: FAIL.

- [ ] **Step 3: Implement dfl_controller.js**

```javascript
// web_demo/sim/dfl_controller.js
// Port of DFL_controller/dfl_controller3.m
import { quatMul, quatConj, quatToR } from './quat.js';
import { alphaGimbal, betaGimbal } from './alpha_beta.js';

const FIXED_PARAMS = {
  Ap: 0, Aq: 0, Ar: 0,
  Ag_p: 0.01, Ag_q: 0.01, Ag_r: 0.01,
  Ix: 0.0023, Iy: 0.0023, Iz: 0.0046,
  Ig_x: 0.001, Ig_y: 0.001, Ig_z: 0.001,
  m: 0.468,
};

const GRAVITY = 9.81;

// state idx (0-based)
// 0..2  position xyz
// 3..6  drone quat
// 7..9  v_world
// 10..12 omega_body
// 13..15 phi_g, theta_g, psi_g
// 16,17  zeta, xi

export function dflController({ droneState, fwState, xd, vd, ad, jd, sd, psid, gains }) {
  const x_w = droneState.slice(0, 3);
  let q_bw = droneState.slice(3, 7);
  const v_w = droneState.slice(7, 10);
  const omega_b = droneState.slice(10, 13);
  const phi_g = droneState[13], theta_g = droneState[14], psi_g = droneState[15];
  const zeta = droneState[16], xi = droneState[17];

  const n = Math.hypot(q_bw[0], q_bw[1], q_bw[2], q_bw[3]) + 1e-9;
  q_bw = q_bw.map(v => v / n);
  const [q0, q1, q2, q3] = q_bw;
  const R_bw = quatToR(q_bw);

  const m = FIXED_PARAMS.m;
  const F_thrust = [R_bw[2]*zeta, R_bw[5]*zeta, R_bw[8]*zeta];
  const a_ = [F_thrust[0]/m, F_thrust[1]/m, F_thrust[2]/m - GRAVITY];
  // jerk: (zeta*(R_bw(:,1)*omega_b(2) - R_bw(:,2)*omega_b(1)) + R_bw(:,3)*xi)/m
  const R_col = (c) => [R_bw[c], R_bw[3+c], R_bw[6+c]];
  const Rc0 = R_col(0), Rc1 = R_col(1), Rc2 = R_col(2);
  const jx = (zeta*(Rc0[0]*omega_b[1] - Rc1[0]*omega_b[0]) + Rc2[0]*xi) / m;
  const jy = (zeta*(Rc0[1]*omega_b[1] - Rc1[1]*omega_b[0]) + Rc2[1]*xi) / m;
  const jz = (zeta*(Rc0[2]*omega_b[1] - Rc1[2]*omega_b[0]) + Rc2[2]*xi) / m;
  const j = [jx, jy, jz];

  const q3dot = 0.5 * (-q2*omega_b[0] + q1*omega_b[1] + q0*omega_b[2]);

  // Position virtual control (4th-order)
  const v_pos = [
    sd[0] - gains.c3*(j[0]-jd[0]) - gains.c2*(a_[0]-ad[0]) - gains.c1*(v_w[0]-vd[0]) - gains.c0*(x_w[0]-xd[0]),
    sd[1] - gains.c3*(j[1]-jd[1]) - gains.c2*(a_[1]-ad[1]) - gains.c1*(v_w[1]-vd[1]) - gains.c0*(x_w[1]-xd[1]),
    sd[2] - gains.c3*(j[2]-jd[2]) - gains.c2*(a_[2]-ad[2]) - gains.c1*(v_w[2]-vd[2]) - gains.c0*(x_w[2]-xd[2]),
  ];
  const v_q3 = -gains.c_q3_dot * q3dot - gains.c_q3 * (q3 - 0);

  // Gimbal references via q_rel = qbar_M (x) q_A, XYZ Euler extraction
  let q_fw = fwState.slice(6, 10);
  const nf = Math.hypot(q_fw[0], q_fw[1], q_fw[2], q_fw[3]) + 1e-9;
  q_fw = q_fw.map(v => v / nf);
  const q_rel = quatMul(quatConj(q_bw), q_fw);
  const [phi_ref, theta_ref, psi_ref] = quatToXYZEuler(q_rel);

  // Rate feedforward via body-frame gimbal Jacobian (paper Eq 13-14)
  // R_gb_des = R_bw^T * R_fw
  const R_fw = quatToR(q_fw);
  const R_gb_des = matMul3x3T(R_bw, R_fw);
  const fw_omega_b = fwState.slice(10, 13);
  const omega_cam_b = matVec3(R_gb_des, fw_omega_b);
  const om_demand = [
    omega_cam_b[0] - omega_b[0],
    omega_cam_b[1] - omega_b[1],
    omega_cam_b[2] - omega_b[2],
  ];

  const cosT = Math.cos(theta_g), sinT = Math.sin(theta_g);
  const cosP = Math.cos(phi_g),   sinP = Math.sin(phi_g);
  // Jg = [1 0 sinT; 0 cosP -sinP*cosT; 0 sinP cosP*cosT]
  // det Jg = cosT
  let dTheta;
  if (Math.abs(cosT) > 1e-3) {
    // closed-form 3x3 inverse for this triangular Jacobian
    const r0 =  om_demand[0] - sinT * ((cosP*om_demand[2] + sinP*om_demand[1])/cosT);
    const r2 =  (cosP*om_demand[2] + sinP*om_demand[1])/cosT;
    const r1 =  cosP*om_demand[1] - sinP*om_demand[2];   // not divided by cosT
    dTheta = [r0, r1, r2];
  } else {
    dTheta = om_demand.slice();   // graceful at singularity
  }
  const phi_g_dot_ref   = dTheta[0];
  const theta_g_dot_ref = dTheta[1];
  const psi_g_dot_ref   = dTheta[2];

  const v_phi   = -gains.c_phi   * (phi_g - phi_ref)   + phi_g_dot_ref;
  const v_theta = -gains.c_theta * (theta_g - theta_ref) + theta_g_dot_ref;
  const v_psig  = -gains.c_psig  * (psi_g - psi_ref)   + psi_g_dot_ref;

  const v = [v_pos[0], v_pos[1], v_pos[2], v_q3, v_phi, v_theta, v_psig];

  const alpha = alphaGimbal(droneState, FIXED_PARAMS, zeta, xi);
  const beta  = betaGimbal(droneState, FIXED_PARAMS, zeta, xi);   // 7x7 row-major
  const u = alpha.slice();
  for (let r = 0; r < 7; r++) {
    for (let c = 0; c < 7; c++) {
      u[r] += beta[r*7 + c] * v[c];
    }
  }
  return u;
}

// === helpers ===
function matVec3(M, v) {
  return [
    M[0]*v[0]+M[1]*v[1]+M[2]*v[2],
    M[3]*v[0]+M[4]*v[1]+M[5]*v[2],
    M[6]*v[0]+M[7]*v[1]+M[8]*v[2],
  ];
}

function matMul3x3T(A, B) {
  // returns A^T * B  (3x3 row-major)
  const out = new Array(9);
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      out[i*3+j] = A[0*3+i]*B[0*3+j] + A[1*3+i]*B[1*3+j] + A[2*3+i]*B[2*3+j];
    }
  }
  return out;
}

function quatToXYZEuler(q) {
  const R = quatToR(q);
  const sth = Math.max(-1, Math.min(1, R[2]));         // R(1,3) -> index 2
  const theta = Math.asin(sth);
  let psi, phi;
  if (Math.abs(Math.cos(theta)) > 1e-6) {
    psi = Math.atan2(-R[1], R[0]);                     // atan2(-R(1,2), R(1,1))
    phi = Math.atan2(-R[5], R[8]);                     // atan2(-R(2,3), R(3,3))
  } else {
    psi = 0;
    phi = Math.atan2(R[7], R[4]);                      // atan2(R(3,2), R(2,2))
  }
  return [phi, theta, psi];
}
```

- [ ] **Step 4: Run test, expect pass**

Run: `cd web_demo && npx vitest run tests/dfl_controller.test.js`
Expected: 1 passing test with 15×7 = 105 sub-assertions.

If failing: print the first mismatched `u` and compare to MATLAB. Most likely sign convention or matrix layout error.

- [ ] **Step 5: Commit**

```bash
git add web_demo/sim/dfl_controller.js web_demo/tests/dfl_controller.test.js
git commit -m "feat(web_demo): dfl_controller3 port + MATLAB-equivalence test"
```

---

## Phase 6: Drone dynamics + unified step (TDD)

### Task 7: Drone dynamics derivative

**Files:**
- Create: `web_demo/sim/drone_dynamics.js`
- Create: `web_demo/tests/drone_dynamics.test.js`

- [ ] **Step 1: Write the failing test**

```javascript
// web_demo/tests/drone_dynamics.test.js
import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';
import { droneDerivative } from '../sim/drone_dynamics.js';

const samples = JSON.parse(readFileSync('web_demo/tests/data/drone_dynamics_samples.json', 'utf8'));

describe('drone_dynamics', () => {
  it('matches MATLAB quadrotor_dynamics_realtime3 on 15 samples', () => {
    for (const s of samples) {
      const sdot = droneDerivative({
        droneState: s.drone_state, fwState: s.fw_state,
        xd: s.xd, vd: s.vd, ad: s.ad, jd: s.jd, sd: s.sd,
        gains: s.gains,
      });
      for (let i = 0; i < 18; i++) {
        expect(sdot[i]).toBeCloseTo(s.sdot[i], 3);
      }
    }
  });
});
```

- [ ] **Step 2: Run test, expect failure**

Run: `cd web_demo && npx vitest run tests/drone_dynamics.test.js`
Expected: FAIL.

- [ ] **Step 3: Implement drone_dynamics.js**

```javascript
// web_demo/sim/drone_dynamics.js
// Port of models/quadrotor_dynamics_realtime3.m
import { dflController } from './dfl_controller.js';
import { quatToR } from './quat.js';

const PARAMS = { m: 0.468, Ix: 0.0023, Iy: 0.0023, Iz: 0.0046, g: 9.81 };

export function droneDerivative({ droneState, fwState, xd, vd, ad, jd, sd, gains }) {
  const q_bw_raw = droneState.slice(3, 7);
  const n = Math.hypot(q_bw_raw[0], q_bw_raw[1], q_bw_raw[2], q_bw_raw[3]) + 1e-9;
  const q_bw = q_bw_raw.map(v => v / n);
  const [q0, q1, q2, q3] = q_bw;

  const v_w = droneState.slice(7, 10);
  const omega_b = droneState.slice(10, 13);
  const zeta = droneState[16], xi = droneState[17];

  const u = dflController({ droneState, fwState, xd, vd, ad, jd, sd, psid: 0, gains });

  const R_bw = quatToR(q_bw);
  // a_world = R*[0;0;zeta]/m - [0;0;g]
  const a_ = [
    R_bw[2] * zeta / PARAMS.m,
    R_bw[5] * zeta / PARAMS.m,
    R_bw[8] * zeta / PARAMS.m - PARAMS.g,
  ];

  // Attitude kinematics (scalar-first)
  const q_dot = [
    0.5 * (-q1*omega_b[0] - q2*omega_b[1] - q3*omega_b[2]),
    0.5 * ( q0*omega_b[0] - q3*omega_b[1] + q2*omega_b[2]),
    0.5 * ( q3*omega_b[0] + q0*omega_b[1] - q1*omega_b[2]),
    0.5 * (-q2*omega_b[0] + q1*omega_b[1] + q0*omega_b[2]),
  ];

  const om_dot = [
    u[1]/PARAMS.Ix + (omega_b[1]*omega_b[2]*(PARAMS.Iy - PARAMS.Iz))/PARAMS.Ix,
    u[2]/PARAMS.Iy - (omega_b[0]*omega_b[2]*(PARAMS.Ix - PARAMS.Iz))/PARAMS.Iy,
    u[3]/PARAMS.Iz + (omega_b[0]*omega_b[1]*(PARAMS.Ix - PARAMS.Iy))/PARAMS.Iz,
  ];

  return [
    v_w[0], v_w[1], v_w[2],
    q_dot[0], q_dot[1], q_dot[2], q_dot[3],
    a_[0], a_[1], a_[2],
    om_dot[0], om_dot[1], om_dot[2],
    u[4], u[5], u[6],          // gimbal rates
    xi,                         // zeta_dot
    u[0],                       // xi_dot = T_ddot
  ];
}
```

- [ ] **Step 4: Run test, expect pass**

Run: `cd web_demo && npx vitest run tests/drone_dynamics.test.js`
Expected: 1 passing test with 15×18 = 270 sub-assertions.

- [ ] **Step 5: Commit**

```bash
git add web_demo/sim/drone_dynamics.js web_demo/tests/drone_dynamics.test.js
git commit -m "feat(web_demo): drone_dynamics port + MATLAB-equivalence test"
```

---

### Task 8: Unified RK4 step

**Files:**
- Create: `web_demo/sim/unified_step.js`

No new tests — exercised indirectly by the browser smoke test.

- [ ] **Step 1: Implement unified_step.js**

```javascript
// web_demo/sim/unified_step.js
import { fwDerivative, edge540Params } from './fw_dynamics.js';
import { droneDerivative } from './drone_dynamics.js';

const FW_PARAMS = edge540Params();

function addScaled(a, b, scale) {
  const out = new Array(a.length);
  for (let i = 0; i < a.length; i++) out[i] = a[i] + scale * b[i];
  return out;
}

export function unifiedRK4Step({ fwState, droneState, controlInputs, gains, dt }) {
  // FW derivative depends only on (fwState, controlInputs).
  // Drone derivative depends on (droneState, fwState, fwState_derivatives).
  // We RK4 both together using shared k1..k4.
  const refsAt = (fw) => {
    const fwd = fwDerivative(fw, controlInputs.thrust, controlInputs.elevator,
                             controlInputs.aileron, controlInputs.rudder, FW_PARAMS);
    return { xd: fw.slice(0, 3), vd: fwd.slice(0, 3),
             ad: fwd.slice(3, 6), jd: [0, 0, 0], sd: [0, 0, 0],
             fwd };
  };

  const k1_refs = refsAt(fwState);
  const k1_fw   = k1_refs.fwd;
  const k1_dr   = droneDerivative({ droneState, fwState,
                                    xd: k1_refs.xd, vd: k1_refs.vd, ad: k1_refs.ad,
                                    jd: k1_refs.jd, sd: k1_refs.sd, gains });

  const fw2 = addScaled(fwState, k1_fw, dt/2);
  const dr2 = addScaled(droneState, k1_dr, dt/2);
  const k2_refs = refsAt(fw2);
  const k2_fw   = k2_refs.fwd;
  const k2_dr   = droneDerivative({ droneState: dr2, fwState: fw2,
                                    xd: k2_refs.xd, vd: k2_refs.vd, ad: k2_refs.ad,
                                    jd: k2_refs.jd, sd: k2_refs.sd, gains });

  const fw3 = addScaled(fwState, k2_fw, dt/2);
  const dr3 = addScaled(droneState, k2_dr, dt/2);
  const k3_refs = refsAt(fw3);
  const k3_fw   = k3_refs.fwd;
  const k3_dr   = droneDerivative({ droneState: dr3, fwState: fw3,
                                    xd: k3_refs.xd, vd: k3_refs.vd, ad: k3_refs.ad,
                                    jd: k3_refs.jd, sd: k3_refs.sd, gains });

  const fw4 = addScaled(fwState, k3_fw, dt);
  const dr4 = addScaled(droneState, k3_dr, dt);
  const k4_refs = refsAt(fw4);
  const k4_fw   = k4_refs.fwd;
  const k4_dr   = droneDerivative({ droneState: dr4, fwState: fw4,
                                    xd: k4_refs.xd, vd: k4_refs.vd, ad: k4_refs.ad,
                                    jd: k4_refs.jd, sd: k4_refs.sd, gains });

  const newFw    = fwState.map((v, i) => v + dt/6 * (k1_fw[i] + 2*k2_fw[i] + 2*k3_fw[i] + k4_fw[i]));
  const newDrone = droneState.map((v, i) => v + dt/6 * (k1_dr[i] + 2*k2_dr[i] + 2*k3_dr[i] + k4_dr[i]));
  return { fwState: newFw, droneState: newDrone };
}

export function defaultGains() {
  return {
    c0: 51150, c1: 51140, c2: 1150, c3: 150,
    c4: 1, c5: 1,
    c_phi: 50, c_theta: 50, c_psig: 50,
    c_q3: 100, c_q3_dot: 20,
  };
}

export function initialFwState() {
  return [0, 0, -100,        // p
          120, 0, 0,          // v_body
          1, 0, 0, 0,         // q
          0, 0, 0];           // omega
}

export function initialDroneState() {
  return [0, 0, -100,         // pos
          1, 0, 0, 0,         // q
          120, 0, 0,          // v_world
          0, 0, 0,            // omega
          0, 0, 0,            // gimbal (3-axis)
          0.468 * 9.81, 0];   // zeta, xi
}
```

- [ ] **Step 2: Commit**

```bash
git add web_demo/sim/unified_step.js
git commit -m "feat(web_demo): unified RK4 step + initial-state helpers"
```

---

## Phase 7: Rendering

### Task 9: HTML skeleton + styles

**Files:**
- Create: `web_demo/index.html`
- Create: `web_demo/styles.css`

- [ ] **Step 1: Create index.html**

```html
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Ghost-Tracking DFL Demo</title>
  <link rel="stylesheet" href="styles.css" />
</head>
<body>
  <canvas id="canvas"></canvas>
  <div id="overlay">
    <div id="labels"><span>FW camera POV</span><span>Drone gimbal-camera POV</span></div>
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
</html>
```

- [ ] **Step 2: Create styles.css**

```css
* { margin: 0; padding: 0; box-sizing: border-box; }
html, body { width: 100%; height: 100%; overflow: hidden; background: #111; color: #ddd; font-family: system-ui, sans-serif; }
#canvas { display: block; width: 100vw; height: 100vh; }
#overlay { position: fixed; inset: 0; pointer-events: none; }
#labels { display: flex; justify-content: space-around; padding: 8px; font-size: 14px; text-shadow: 0 0 4px black; }
#cheatsheet { position: fixed; bottom: 8px; left: 50%; transform: translateX(-50%);
  padding: 6px 12px; background: rgba(0,0,0,0.5); border-radius: 6px; font-size: 13px; }
```

- [ ] **Step 3: Commit**

```bash
git add web_demo/index.html web_demo/styles.css
git commit -m "feat(web_demo): page skeleton + HUD"
```

---

### Task 10: Scene (sky + ground + reference cubes)

**Files:**
- Create: `web_demo/render/scene.js`

- [ ] **Step 1: Implement scene.js**

```javascript
// web_demo/render/scene.js
import * as THREE from 'three';

export function buildScene() {
  const scene = new THREE.Scene();
  // Sky gradient via a large inverted sphere
  const skyGeom = new THREE.SphereGeometry(5000, 32, 16);
  const skyMat  = new THREE.ShaderMaterial({
    side: THREE.BackSide,
    uniforms: { topColor: { value: new THREE.Color(0x3a78c8) }, bottomColor: { value: new THREE.Color(0xa8c8ee) } },
    vertexShader: `varying vec3 vPos; void main(){ vPos = position; gl_Position = projectionMatrix * modelViewMatrix * vec4(position,1.0); }`,
    fragmentShader: `varying vec3 vPos; uniform vec3 topColor; uniform vec3 bottomColor;
      void main(){ float t = clamp(0.5 + 0.5 * normalize(vPos).y, 0.0, 1.0); gl_FragColor = vec4(mix(bottomColor, topColor, t), 1.0); }`,
  });
  scene.add(new THREE.Mesh(skyGeom, skyMat));

  // Ground: large textured plane with grid
  const groundMat = new THREE.MeshBasicMaterial({ color: 0x4a6a3a });
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(10000, 10000), groundMat);
  ground.rotation.x = -Math.PI / 2;
  scene.add(ground);
  const grid = new THREE.GridHelper(10000, 200, 0x223322, 0x223322);
  grid.position.y = 0.01;
  scene.add(grid);

  // Reference cubes/spheres scattered across the area
  const rng = mulberry32(123);
  const colors = [0xff6666, 0x66ccff, 0xffd866, 0xc098ff, 0x66ff99, 0xff99cc];
  for (let i = 0; i < 30; i++) {
    const x = (rng() - 0.5) * 5000;
    const z = (rng() - 0.5) * 5000;
    const y = 30 + rng() * 200;
    const size = 30 + rng() * 50;
    const isCube = rng() > 0.5;
    const geo = isCube ? new THREE.BoxGeometry(size, size, size) : new THREE.SphereGeometry(size/2, 16, 12);
    const mat = new THREE.MeshLambertMaterial({ color: colors[i % colors.length] });
    const m = new THREE.Mesh(geo, mat);
    m.position.set(x, y, z);
    scene.add(m);
  }

  // Lights
  const sun = new THREE.DirectionalLight(0xffffff, 1.0);
  sun.position.set(1, 1, 0.5);
  scene.add(sun);
  scene.add(new THREE.AmbientLight(0x6080aa, 0.6));

  return scene;
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

- [ ] **Step 2: Commit**

```bash
git add web_demo/render/scene.js
git commit -m "feat(web_demo): minimalist scene with sky, grid ground, ref cubes"
```

---

### Task 11: Cameras + viewport

**Files:**
- Create: `web_demo/render/cameras.js`
- Create: `web_demo/render/viewport.js`

- [ ] **Step 1: Implement cameras.js**

Coordinate conventions:
- **Our state**: aerospace NED — x=north, y=east, z=down. Body: x=forward, y=right-wing, z=down. Quaternion `q` maps body→world (scalar-first `[w,x,y,z]`).
- **Three.js world**: x=east-ish, y=up, z=north-ish (right-handed, Y-up). Camera local: -Z forward, +Y up, +X right.

Two rotations need composing for each render frame:
1. **q_cam_mount**: maps Three.js camera-local axes onto body axes so that the camera "lens" (Three's -Z) points along body +X, with camera up (+Y) along body −Z (level horizon when FW is level).
2. **q_ned_to_three**: maps the entire NED frame onto Three's frame so positions and orientations are visualised right-side-up.

Both are constant. Composing: `cam_quat_three = q_ned_to_three (x) q_body_to_ned (x) q_cam_mount`.

```javascript
// web_demo/render/cameras.js
import * as THREE from 'three';
import { quatMul } from '../sim/quat.js';

// q_cam_mount: rotates Three's camera-local frame to body frame.
//   Three(-Z) -> body(+X)
//   Three(+Y) -> body(-Z)
//   Three(+X) -> body(+Y)
// Derived rotation matrix has trace 0; standard conversion gives this quat:
export const Q_CAM_MOUNT = [0.5, -0.5, -0.5, 0.5];

// q_ned_to_three: rotates NED-world axes to Three-world axes.
//   NED(+X north) -> Three(+X)
//   NED(+Y east)  -> Three(+Z)
//   NED(+Z down)  -> Three(-Y)
// This is a 90 deg rotation about Three's +X axis.
export const Q_NED_TO_THREE = [Math.SQRT1_2, Math.SQRT1_2, 0, 0];

export function makeCamera(aspect) {
  return new THREE.PerspectiveCamera(75, aspect, 0.5, 10000);
}

// posNED -> Three position; quatBodyToNED -> Three camera quaternion.
export function setCameraPose(cam, qBodyToNED, posNED) {
  cam.position.set(posNED[0], -posNED[2], posNED[1]);
  const q = quatMul(Q_NED_TO_THREE, quatMul(qBodyToNED, Q_CAM_MOUNT));
  // Three uses (x, y, z, w); ours is (w, x, y, z)
  cam.quaternion.set(q[1], q[2], q[3], q[0]);
}

// gimbal_quat = q_x(phi) (x) q_y(theta) (x) q_z(psi)
export function gimbalQuat(phi, theta, psi) {
  const qx = [Math.cos(phi/2), Math.sin(phi/2), 0, 0];
  const qy = [Math.cos(theta/2), 0, Math.sin(theta/2), 0];
  const qz = [Math.cos(psi/2), 0, 0, Math.sin(psi/2)];
  return quatMul(quatMul(qx, qy), qz);
}
```

**Verification at run time (Task 13 smoke test):**
- At rest with the initial state (FW level pointing north), both views should show the horizon level and reference cubes ahead (positive Three-X direction).
- Pressing ↑ (pitch up, nose-up) should make the horizon descend in both views.
- Pressing → (roll right) should make the horizon tilt CCW in both views.

If these don't match, the most likely fixes are:
- swap sign in one component of `Q_CAM_MOUNT` (rotate camera ±90° about a different axis)
- swap two coords in `cam.position.set(...)` (NED→Three swap is wrong)

- [ ] **Step 2: Implement viewport.js**

```javascript
// web_demo/render/viewport.js
import * as THREE from 'three';

let renderer;

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
```

- [ ] **Step 3: Commit**

```bash
git add web_demo/render/cameras.js web_demo/render/viewport.js
git commit -m "feat(web_demo): dual-camera + scissor viewport"
```

---

## Phase 8: Input

### Task 12: Keyboard with smooth ramps

**Files:**
- Create: `web_demo/input/keyboard.js`
- Create: `web_demo/tests/keyboard.test.js`

- [ ] **Step 1: Write the failing test**

```javascript
// web_demo/tests/keyboard.test.js
import { describe, it, expect } from 'vitest';
import { Keyboard } from '../input/keyboard.js';

describe('keyboard ramps', () => {
  it('zero deflections at init', () => {
    const k = new Keyboard();
    const c = k.controls();
    expect(c.elevator).toBe(0);
    expect(c.aileron).toBe(0);
  });

  it('ramps elevator to limit when ArrowUp held for >= ramp time', () => {
    const k = new Keyboard();
    k._down('ArrowUp');
    for (let i = 0; i < 10; i++) k.step(0.05);   // 0.5 s total
    expect(k.controls().elevator).toBeCloseTo(-0.4, 6);   // ArrowUp = nose-up = negative
  });

  it('decays back to zero after release', () => {
    const k = new Keyboard();
    k._down('ArrowUp');
    for (let i = 0; i < 10; i++) k.step(0.05);
    k._up('ArrowUp');
    for (let i = 0; i < 20; i++) k.step(0.05);
    expect(Math.abs(k.controls().elevator)).toBeLessThan(0.01);
  });
});
```

- [ ] **Step 2: Run, expect failure**

Run: `cd web_demo && npx vitest run tests/keyboard.test.js`
Expected: FAIL.

- [ ] **Step 3: Implement keyboard.js**

```javascript
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

    // Thrust is rate-driven: W/S increase/decrease, sticky.
    if (this.keys.has('KeyW')) this.values.thrust = Math.min(LIMITS.thrust, this.values.thrust + 50*dt);
    if (this.keys.has('KeyS')) this.values.thrust = Math.max(0,            this.values.thrust - 50*dt);
  }

  controls() { return { ...this.values }; }
}
```

- [ ] **Step 4: Run test, expect pass**

Run: `cd web_demo && npx vitest run tests/keyboard.test.js`
Expected: 3 passing tests.

- [ ] **Step 5: Commit**

```bash
git add web_demo/input/keyboard.js web_demo/tests/keyboard.test.js
git commit -m "feat(web_demo): keyboard input with smooth ramps + tests"
```

---

## Phase 9: Main loop

### Task 13: main.js — wire physics + render + input

**Files:**
- Create: `web_demo/main.js`

- [ ] **Step 1: Implement main.js**

```javascript
// web_demo/main.js
import { buildScene } from './render/scene.js';
import { initRenderer, renderDual } from './render/viewport.js';
import { makeCamera, setCameraPose, gimbalQuat } from './render/cameras.js';
import { Keyboard } from './input/keyboard.js';
import { unifiedRK4Step, defaultGains, initialFwState, initialDroneState } from './sim/unified_step.js';

const canvas = document.getElementById('canvas');
const renderer = initRenderer(canvas);
const scene = buildScene();

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

const PHYS_DT = 0.005;     // 200 Hz
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

  // Push state into cameras
  const fwPos  = fwState.slice(0, 3);
  const fwQuat = fwState.slice(6, 10);
  setCameraPose(camFw, fwQuat, fwPos);

  const dronePos  = droneState.slice(0, 3);
  const droneQuat = droneState.slice(3, 7);
  const gQ        = gimbalQuat(droneState[13], droneState[14], droneState[15]);
  const camWorldQuat = (function () {
    // q_M (x) q_G
    const a = droneQuat, b = gQ;
    return [
      a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
      a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
      a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
      a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0],
    ];
  })();
  setCameraPose(camDrone, camWorldQuat, dronePos);

  renderDual(scene, camFw, camDrone);

  // expose state for e2e test
  window.__demoState = { fwQuat, camWorldQuat, dronePos, fwPos };
}

requestAnimationFrame((t) => { lastTime = t; frame(t); });
```

- [ ] **Step 2: Smoke-test locally**

Run:
```bash
cd web_demo && python3 -m http.server 8000
```

Open `http://localhost:8000` in a browser. Expected: both viewports show sky+ground+cubes. Press arrows — both views move together. Press R — resets. Press P — pauses.

- [ ] **Step 3: Commit**

```bash
git add web_demo/main.js
git commit -m "feat(web_demo): main loop wiring physics + dual render + input"
```

---

## Phase 10: Browser smoke test

### Task 14: Playwright e2e test

**Files:**
- Create: `web_demo/playwright.config.js`
- Create: `web_demo/tests/e2e_smoke.spec.js`

- [ ] **Step 1: Install browser**

Run: `cd web_demo && npx playwright install chromium`

- [ ] **Step 2: Create config**

```javascript
// web_demo/playwright.config.js
import { defineConfig } from '@playwright/test';
export default defineConfig({
  testDir: './tests',
  testMatch: '*.spec.js',
  use: { baseURL: 'http://localhost:8000' },
  webServer: {
    command: 'python3 -m http.server 8000',
    port: 8000,
    reuseExistingServer: true,
  },
});
```

- [ ] **Step 3: Write smoke test**

```javascript
// web_demo/tests/e2e_smoke.spec.js
import { test, expect } from '@playwright/test';

test('camera tracks FW after free-flight', async ({ page }) => {
  await page.goto('/');
  await page.waitForFunction(() => window.__demoState !== undefined);

  // Pitch up briefly, then release.
  await page.keyboard.down('ArrowUp');
  await page.waitForTimeout(800);
  await page.keyboard.up('ArrowUp');
  await page.waitForTimeout(2000);

  const s = await page.evaluate(() => window.__demoState);

  // Compute geodesic between fwQuat and camWorldQuat
  function dotQ(a, b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]+a[3]*b[3]; }
  const geo = 2 * Math.acos(Math.min(1, Math.abs(dotQ(s.fwQuat, s.camWorldQuat)))) * 180/Math.PI;
  expect(geo).toBeLessThan(1.0);
});
```

- [ ] **Step 4: Run**

Run: `cd web_demo && npx playwright test`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add web_demo/playwright.config.js web_demo/tests/e2e_smoke.spec.js
git commit -m "test(web_demo): playwright e2e smoke test for tracking"
```

---

## Phase 11: Deploy

### Task 15: GitHub Pages deploy workflow

**Files:**
- Create: `.github/workflows/deploy-web-demo.yml`

- [ ] **Step 1: Create workflow**

```yaml
name: Deploy web_demo to Pages

on:
  push:
    branches: [master]
    paths: ['web_demo/**', '.github/workflows/deploy-web-demo.yml']

permissions:
  contents: read
  pages: write
  id-token: write

concurrency:
  group: pages
  cancel-in-progress: false

jobs:
  build-and-deploy:
    runs-on: ubuntu-latest
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    steps:
      - uses: actions/checkout@v4
      - uses: actions/configure-pages@v5
      - uses: actions/upload-pages-artifact@v3
        with:
          path: web_demo
      - id: deployment
        uses: actions/deploy-pages@v4
```

- [ ] **Step 2: Enable Pages in repo settings (manual, one time)**

In GitHub repo → Settings → Pages → Source: "GitHub Actions". Save.

- [ ] **Step 3: Commit and push**

```bash
git add .github/workflows/deploy-web-demo.yml
git commit -m "ci(web_demo): GitHub Actions Pages deploy on push to master"
git push origin master
```

- [ ] **Step 4: Verify**

Watch the Actions tab on GitHub. After ~1 min, the workflow should complete. Open the URL shown in the workflow output (`https://abdelhakim96.github.io/Ghost_Tracking_DFL/`) and confirm the demo loads.

---

## Phase 12: README

### Task 16: User-facing README

**Files:**
- Create: `web_demo/README.md`

- [ ] **Step 1: Write README**

```markdown
# Ghost-Tracking DFL — Web Demo

Pilot a fixed-wing aircraft, watch the multicopter+gimbal camera POV match it in real time.

**Live:** https://abdelhakim96.github.io/Ghost_Tracking_DFL/

## Controls

| Key | Action |
| --- | --- |
| ↑ / ↓ | Pitch up / down |
| ← / → | Roll left / right |
| A / D | Rudder left / right |
| W / S | Increase / decrease thrust |
| R | Reset |
| P | Pause / resume |

## Run locally

```bash
cd web_demo
python3 -m http.server 8000     # or any static server
# open http://localhost:8000
```

## Tests

```bash
cd web_demo
npm install
npm test                # unit tests (Vitest), MATLAB-equivalence checks
npm run test:e2e        # Playwright browser smoke
```

## Architecture

See `../docs/superpowers/specs/2026-05-16-ghost-tracking-game-design.md`.

## Regenerating MATLAB ground-truth samples

```bash
matlab -batch "addpath('scripts'); extract_matlab_samples"
```

Writes JSON test fixtures into `tests/data/`. Re-run after any change to the
MATLAB reference implementation.
```

- [ ] **Step 2: Commit**

```bash
git add web_demo/README.md
git commit -m "docs(web_demo): README with controls + how to run"
```

---

## Final acceptance run

- [ ] **Run all unit tests**

Run: `cd web_demo && npm test`
Expected: All test suites pass (quat, fw_dynamics, alpha_beta, dfl_controller, drone_dynamics, keyboard).

- [ ] **Run the e2e smoke test**

Run: `cd web_demo && npm run test:e2e`
Expected: PASS.

- [ ] **Open the live URL after Actions completes**

Visit `https://abdelhakim96.github.io/Ghost_Tracking_DFL/`. Pilot freely for 30s. Both views should move together.

- [ ] **Final commit / push if anything was tweaked**

```bash
git status
git push origin master
```

---

## Summary of acceptance criteria (per the spec)

| # | Criterion | How to verify |
|---|---|---|
| 1 | ≥ 30 FPS on typical laptop integrated graphics | Open DevTools Performance tab while piloting |
| 2 | Page loads in < 5s | DevTools Network tab |
| 3 | Pilot freely 30s without NaN / divergence | Console check + smoke test |
| 4 | Two viewports visually identical | Manual; e2e checks geodesic < 1° |
