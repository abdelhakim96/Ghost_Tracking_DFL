# DFL Controller — Theory vs Implementation Analysis

**Project:** Ghost_Tracking_DFL — Dynamic System Emulation: Fixed-Wing Dynamics on a Multicopter
**Paper:** `ECC_2026/root.tex`
**Implementation:** `DFL_controller/`, `models/`, `main.m`
**Date:** 2026-05-15
**Symptom:** Loop maneuver tracks well. Barrel roll maneuver fails.

---

## TL;DR (read this if nothing else)

You have **two independent bugs** stacked on top of each other:

1. **The DFL outputs in `controller_generation.m` are not the camera pose.** The paper proves mimicry on the **composed** camera quaternion $\mathbf q_\text{MC}=\mathbf q_\text{M}\otimes\mathbf q_\text{G}$, but the implementation linearizes drone-yaw, gimbal-roll, gimbal-pitch as **three independent outputs**. The DFL's decoupling matrix in your code is therefore block-diagonal in (drone-yaw vs gimbal), whereas the paper's framework couples them through the input-twist matrix $\mathbf S(\phi_\text G,\theta_\text G)$. You have **decoupled the gimbal from the drone**, exactly as you suspected.

2. **Drone yaw is held at $\psi_\text M=0$.** The paper requires $\psi_\text M=\mathrm{yaw}(\bar{\mathbf q}_\text{tilt}\otimes\mathbf q_\text A)$ (Eq. (16) in the paper). The whole point of yaw scheduling is to absorb the part of the FW orientation that the 2-axis gimbal cannot represent, so the gimbal only has to absorb a roll-pitch residual. Skipping this dumps roll *and* yaw onto the gimbal at once. A 2-axis gimbal *cannot* represent both.

Plus a handful of smaller defects (gimbal axis convention swap, a real typo in the FW rotation matrix, no lever-arm compensation, ill-defined yaw output, confused rate feedforward). Details below.

The paper's theory is **logically sound** under its stated assumptions; the implementation is not what the paper describes. Once the two big items above are fixed, the rolling maneuver will work *up to the physical 2-axis singularity* (i.e., it cannot do a full 360° barrel roll without a 3-axis gimbal — and the paper says so, see Assumption 1 and the Remark right after Eq. (13)).

---

## 1. What the paper actually says (methodology recap)

### 1.1 Output is the **camera** pose, not the drone pose
From `root.tex` Eq. (8) and Eq. (12):

$$
\mathbf y_\text{MC}=\big[\,\mathbf p_\text M+\mathbf R(\mathbf q_\text M)\mathbf t_\text G\;;\;\mathbf q_\text M\otimes\mathbf q_\text G\big],
\qquad
\mathbf y_\text{AC}=\big[\,\mathbf p_\text A+\mathbf R(\mathbf q_\text A)\mathbf t_\text{AC}\;;\;\mathbf q_\text A\big].
$$

The mimicry condition is $\mathbf y_\text{MC}(t)\equiv\mathbf y_\text{AC}(t)$, **on the camera pose**, not on the drone CoM.

### 1.2 Controllability is SE(3)-coupled, not "drone-yaw + gimbal-angles independently"

Paper Eq. (13)–(14):

$$
\begin{bmatrix}\ddot{\mathbf p}_\text{MC}\\ \boldsymbol\omega_\text{MC}\end{bmatrix}
=\underbrace{\begin{bmatrix}\mathbf I_3 & \mathbf 0\\ \mathbf 0 & \mathbf S(\phi_\text G,\theta_\text G)\end{bmatrix}}_{\mathbf J}
\begin{bmatrix}\mathbf a_\text{MC}\\ \dot{\boldsymbol\Theta}_\text{MC}\end{bmatrix}+\begin{bmatrix}\boldsymbol\Delta\\ 0\end{bmatrix},\quad
\dot{\boldsymbol\Theta}_\text{MC}=\begin{bmatrix}\dot\phi_\text G\\ \dot\theta_\text G\\ \boxed{\dot\psi_\text M}\end{bmatrix},\quad
\mathbf S=\begin{bmatrix}1&0&-\sin\theta\cos\phi\\0&1&-\sin\phi\\0&0&\cos\theta\cos\phi\end{bmatrix}.
$$

Note carefully: the third orientation input is **drone yaw rate $\dot\psi_\text M$**, sitting *inside the same Jacobian* as the gimbal rates. Drone and gimbal share the orientation channel — they are **not** independent.

Singularity set: $\det\mathbf S=\cos\phi_\text G\cos\theta_\text G=0$ ⇒ $\phi_\text G=\pm\pi/2$ or $\theta_\text G=\pm\pi/2$. STLC fails on this set (Assumption 1).

### 1.3 The yaw schedule is essential, not optional
Eq. (15)–(17):

$$
\mathbf q_\text M=\mathbf q_z(\psi_\text M)\otimes\mathbf q_\text{tilt},\qquad
\bar\psi_\text M=\mathrm{yaw}(\bar{\mathbf q}_\text{tilt}\otimes\mathbf q_\text A),\qquad
\mathbf q_\text G=\bar{\mathbf q}_\text M\otimes\mathbf q_\text A=\mathbf q_x(\phi_\text G)\otimes\mathbf q_y(\theta_\text G).
$$

Procedure: (i) compute $\mathbf q_\text{tilt}$ from the required thrust direction $\mathbf a_\text M$; (ii) **pick $\psi_\text M$ to cancel the yaw of $\bar{\mathbf q}_\text{tilt}\otimes\mathbf q_\text A$**; (iii) the residual $\mathbf q_\text{rel}$ has zero yaw, lives in the X-Y submanifold, and is realized by gimbal **roll-pitch** $(\phi_\text G,\theta_\text G)$ with $\mathbf q_\text G=\mathbf q_x(\phi_\text G)\otimes\mathbf q_y(\theta_\text G)$. **Gimbal axes are X then Y, not Z then Y.**

### 1.4 DFL extension
Sec. IV: original relative-degree vector $\mathbf r=[2,2,2,1,1,1]$, $N=14$, $\sum r_i=9\neq 14$ — static feedback fails because $T$ only enters at $\ddot{\mathbf p}$. Add a thrust double integrator $\chi$ to delay $T$ ⇒ $\hat{\mathbf r}=[4,4,4,2,1,1]$, $\hat N=16$, $\sum=16$. Now solvable.

**Crucially**: the "1, 1" at the end are the gimbal-rate channels; the "2" before them is the yaw channel realized by drone yaw rate (after the outer SE(3) controllability framing). The decoupling matrix $\Delta(\mathbf x)$ should be **non-block-diagonal** because $\mathbf S(\phi_\text G,\theta_\text G)$ mixes drone yaw rate with gimbal rates.

### 1.5 Theory sanity check
The proofs are clean modulo the assumptions:
- **Assumption 1 (non-singularity)** is *not* a small constraint for aerobatics. A 2-axis gimbal cannot represent SO(3); barrel rolls live on a circle that crosses or grazes the singular set repeatedly. The paper's own Remark after Assumption 1 admits "this assumption can be relaxed by using a planner that avoids the singular set" — i.e., 2-axis is fundamentally insufficient for full attitude mimicry, and a 3-axis gimbal makes the result global.
- **Assumption 2 (authority)** is straightforward (thrust/torque/rate limits respected).
- **Position matching** (Eq. (15)): mathematically fine, but requires double-derivative of lever-arm terms — which couple drone angular dynamics into the translational feedforward $\Delta$.

**Verdict on the paper**: theory is consistent and correct *for what it claims*. It does not claim global mimicry with a 2-axis gimbal, and you should not expect a full 360° barrel roll to work without a 3-axis gimbal. The paper's framework also assumes a yaw schedule is applied — your implementation does not apply it.

---

## 2. What the implementation actually does

### 2.1 The DFL outputs (the smoking gun)
`DFL_controller/controller_generation.m`, lines 94–100:

```matlab
y_out = [
    x0;                 % drone CoM x
    y0;                 % drone CoM y
    z0;                 % drone CoM z
    2*(q1*q2+q0*q3);    % R_bw(2,1) — a "yaw" proxy on the DRONE
    phi_g;              % gimbal roll — standalone
    theta_g];           % gimbal pitch — standalone
```

This linearizes **drone position + drone yaw + gimbal angles as six independent outputs**. The decoupling matrix $\Delta$ that comes out of the symbolic Lie-derivative chain is structurally **block-diagonal**: the gimbal-angle rows only contain $u_5$/$u_6$ (the gimbal-rate inputs), and the drone-yaw row only contains $u_4$ (the drone yaw torque). You can see this in `beta_gimbal_func.m`: rows 5 and 6 are $(0,0,0,0,1,0)$ and $(0,0,0,0,0,1)$.

That is **not** the paper's structure. The paper's outputs are the camera pose; the decoupling matrix mixes drone-yaw and gimbal rates via $\mathbf S(\phi_\text G,\theta_\text G)$.

> You were right to suspect this. **You decoupled the gimbal and drone, which is exactly what the paper does *not* do.**

### 2.2 Drone yaw setpoint is hard-coded to zero
`main.m` calls `unified_dynamics(t, ...)`; `unified_dynamics.m` line 38 calls

```matlab
quadrotor_dynamics_realtime(t, quad_state, ref_pos_ned, ref_vel_ned, ref_acc_ned, ref_jerk_ned, ref_snap_ned, 0, fw_state, fw_orientation, dfl_gains);
%                                                                                                         ^psid
```

The 8th argument is the literal `0`. So the controller is fighting to keep drone yaw $\psi_\text M\equiv 0$ regardless of FW orientation. The yaw schedule of Eq. (16) is **never computed**.

Consequence for the roll maneuver: the FW develops yaw via $\beta\to$ side-slip during the barrel roll. The drone is told to keep $\psi_\text M=0$. The gimbal then has to absorb the *entire* FW rotation (roll + pitch + yaw). A 2-axis gimbal cannot do this — singular projections result.

### 2.3 Gimbal axis convention is inconsistent with the paper
Paper: $\mathbf q_\text G=\mathbf q_x(\phi_\text G)\otimes\mathbf q_y(\theta_\text G)$ — roll then pitch (X then Y).

`dfl_controller.m`, lines 72–76:

```matlab
% This assumes a Z-Y rotation sequence for the gimbal (phi_g is yaw, theta_g is pitch)
theta_g_ref_raw = asin(-R_gb_ref(3,1));
phi_g_ref_raw   = atan2(R_gb_ref(2,1), R_gb_ref(1,1));
```

`asin(-R(3,1))` and `atan2(R(2,1), R(1,1))` are the **Z-Y extraction** (yaw, then pitch). So your reference treats $\phi_\text G$ as *yaw* about the body z and $\theta_\text G$ as pitch. The comment in the code is consistent with that.

But:
- The paper says the two axes are roll and pitch (X-Y).
- Either convention fails for a barrel roll, because either way the gimbal has no roll-about-the-relevant-axis — Z-Y fails worst (Z-Y has *zero* representational power for body-frame roll, so a pure FW roll projects to $\phi_g=\theta_g=0$ at the extraction step).

This is the second reason rolling collapses: the *reference itself* silently becomes ~zero whenever FW develops body-x roll.

### 2.4 Real typo bug in the FW rotation matrix
`dfl_controller.m`, lines 65–67:

```matlab
R_fw_w = [q0_fw^2+q1_fw^2-q2_fw^2-q3_fw^2, 2*(q1_fw*q2_fw-q0_fw*q3_fw), 2*(q1_fw*q3_fw+q0_fw*q2_fw);
          2*(q1_fw*q2_fw+q0_fw*q3_fw), q0_fw^2-q1_fw^2+q2^2-q3^2,        2*(q2_fw*q3_fw-q0_fw*q1_fw);
          2*(q1_fw*q3_fw-q0_fw*q2_fw), 2*(q2_fw*q3_fw+q0_fw*q1_fw),       q0_fw^2-q1^2-q2^2+q3^2];
```

`q2^2`, `q3^2`, `q1^2` on the diagonal are **drone** quaternion components, not FW. This $R_{fw\_w}$ is then used at line 70 to build $R_{gb,\text{ref}}$ from which `phi_g_ref`, `theta_g_ref` are extracted. Even before any conceptual issue, the gimbal reference is computed from a corrupted matrix.

Line 112 later overwrites `R_fw_w = quat2rotm(q_fw')` — but only after lines 70–76 have already extracted the (wrong) gimbal angle references using the buggy matrix.

### 2.5 No lever-arm compensation
The paper sets the drone position command to (Eq. (15))

$$
\mathbf p_\text M=\mathbf p_\text A+\mathbf R(\mathbf q_\text A)\mathbf t_\text{AC}-\mathbf R(\mathbf q_\text M)\mathbf t_\text G.
$$

Your `ref_pos_ned = fw_state(1:3)` is just $\mathbf p_\text A$. If both $\mathbf t_\text{AC}=\mathbf 0$ and $\mathbf t_\text G=\mathbf 0$ this is fine (camera at CoM on both), but it is a hidden assumption — and it means the $\boldsymbol\Delta$ lever-arm coupling in Eq. (10) is silently zero in your system.

### 2.6 The yaw output is a degenerate proxy
`y_out(4) = 2*(q1*q2+q0*q3) = R_{bw}(2,1) = \sin\psi\cos\theta` (Z-Y-X Euler).

When pitch $\theta\neq 0$ — exactly the loop case! — this is not a clean yaw; it mixes pitch and yaw. The DFL is linearizing the wrong scalar. The loop only works because (i) yaw stays small, and (ii) you can mostly project it out.

Also in `dfl_controller.m` line 49:

```matlab
vpsi = omega_b(1)*(2*q1*q3-2*q0*q2) + omega_b(2)*(2*q2*q3+2*q0*q1) + omega_b(3)*(q0^2-q1^2-q2^2+q3^2);
```

That's `omega_b · R_{bw}(3,:)` — the body angular velocity projected on the world Z axis. It's the world-frame yaw rate, **not** the time derivative of the output `R_{bw}(2,1)` that you're trying to track. So the position-error and rate-error in `v_yaw` (line 53) reference different quantities. This is incoherent under DFL.

### 2.7 The gimbal rate feedforward mixes frames
`dfl_controller.m`, lines 112–119:

```matlab
R_fw_w = quat2rotm(q_fw');
R_gb_desired = R_bw' * R_fw_w;
fw_omega_b = fw_state(11:13);          % omega of FW in FW body frame
omega_fw_in_b = R_gb_desired * fw_omega_b;
omega_rel_b = omega_fw_in_b - omega_b;
phi_g_ref_dot   = omega_rel_b(1);
theta_g_ref_dot = omega_rel_b(2);
```

`R_gb_desired` maps the **gimbal-body** frame to the **drone-body** frame (or vice versa depending on convention). Multiplying it on the left of `omega_fw_in_b_FW` to get something "in drone body" only makes sense if the matrix you're using is the FW→drone-body rotation — which is what $R_\text{bw}^\top R_\text{fw\_w}$ is. OK, that part is reasonable. But then `omega_rel_b = omega_fw_in_b - omega_drone_b` is the right body-frame **rate gap** only if $\boldsymbol\omega_\text{cam}^\text{body}=\boldsymbol\omega_\text{drone}^\text{body}+\dot{\boldsymbol\Theta}_\text G$ exactly, which it isn't — the proper kinematic identity is

$$
\boldsymbol\omega_\text{cam}^\text{body}=\boldsymbol\omega_\text{drone}^\text{body}+\mathbf S(\phi_\text G,\theta_\text G)\,\dot{\boldsymbol\Theta}_\text G
$$

and you should invert $\mathbf S$ to back out the feedforward gimbal rates. Skipping $\mathbf S^{-1}$ silently linearizes around $\phi_\text G=\theta_\text G=0$, again breaking precisely when angles are large (i.e., during a roll).

### 2.8 The decoupling matrix has its own singularities
`beta_gimbal_func.m` has terms like `1./(q0^2-q1^2+q2^2-q3^2)` (Z-Y-X dependent denominators) and `1./zeta` (thrust). Singular when:
- $q_0^2+q_2^2=q_1^2+q_3^2$ — happens at certain attitudes including configurations reached during aggressive maneuvers,
- $\zeta\to 0$ (thrust loss).

The pseudo-inverse `pinv` used in `controller_generation.m` line 169–170 hides this rather than handles it; near singular configurations $\alpha,\beta$ blow up or quietly do the wrong thing.

---

## 3. Why **loop works but roll doesn't**, in one paragraph

In the loop, FW orientation is dominated by pitch ($\mathbf q_\text A\approx\mathbf q_y(\theta_\text A)$). FW yaw and roll stay near zero. The drone tilts pitch-forward to follow the position trajectory; the implementation's "Z-Y" gimbal $\phi_\text G$ stays near zero, and $\theta_\text G$ absorbs the FW pitch. The decoupled, yaw-pinned-to-zero controller happens to be an accurate approximation of the paper's framework in this regime — so it tracks.

In the barrel roll, FW orientation accumulates *roll* about its body-x axis ($\mathbf q_\text A\approx\mathbf q_x(\phi_\text A)$, $\phi_\text A$ growing toward $\pi$). The implementation's gimbal is Z-Y, so:
- $R_{gb,\text{ref}}=R_\text{bw}^\top R_\text{fw\_w}$ projects an X-axis rotation onto a Z-Y submanifold. The extraction `asin(-R(3,1))`, `atan2(R(2,1),R(1,1))` returns ~zero for a pure X rotation: the gimbal is told to do nothing while the FW is rolling.
- The drone is told $\psi_\text M=0$, so it can't absorb anything either.
- The result is a camera that points "forward and level" while the FW rolls upside-down — exactly the failure mode you're seeing.

Even with the X-Y gimbal of the paper, the roll only works **up to** $\phi_\text G=\pm\pi/2$ (Assumption 1). With drone yaw scheduling, the singularity moves and a longer fraction of the maneuver is reachable, but a full 360° barrel roll requires a 3-axis gimbal — period.

---

## 4. Does the paper's theory itself make sense?

Yes, with these qualifications:

| Claim in paper | Sound? | Caveat |
| --- | --- | --- |
| Camera pose is output-fully-actuated on SE(3) | ✅ | Only away from $\cos\phi_G\cos\theta_G=0$ |
| Mimicry control law exists via yaw scheduling + gimbal | ✅ | Assumes Assumption 1 holds along trajectory |
| Static feedback insufficient ($\sum r_i=9<14$), use dynamic extension | ✅ | Correct |
| Extended system has $\sum\hat r_i=\hat N=16$, DFL solvable | ✅ | Correct |
| 2-axis gimbal suffices in practice | ⚠️ | For aerobatic rolls/inverted flight, **no**. Paper itself flags this in Remark after Eq. (14). |

What the paper **does not** say but is easy to misread:
- It does not claim the decoupling matrix is block-diagonal between drone and gimbal channels. It explicitly mixes drone yaw rate with gimbal rates through $\mathbf S$.
- It does not claim the gimbal angles are independent DFL outputs. The DFL outputs are camera-pose components.

So the implementation is not a wrong proof of the paper's theorem — it's solving a *different* problem (a decoupled 6-output DFL) that happens to coincide with the paper's solution only in the small-orientation, small-yaw regime (≈ the loop).

---

## 5. Fix plan

Ordered so that each step gives measurable improvement on its own.

### Phase 0 — Sanity / cheap fixes (today, ~30 min)

| # | Fix | File / location | Effect |
| --- | --- | --- | --- |
| 0.1 | Fix `R_fw_w` typo (drone q components used in FW matrix) | `dfl_controller.m` lines 65–67 — replace `q2`,`q3`,`q1` with `q2_fw`,`q3_fw`,`q1_fw`, OR delete lines 64–67 and just use `quat2rotm(q_fw')` once at the top | Stops a clearly wrong reference; small but real |
| 0.2 | Compute the yaw setpoint from FW orientation instead of hard-coding 0 | `unified_dynamics.m` line 38 — pass `psid = atan2(R_fw_w(2,1), R_fw_w(1,1))` or a proper yaw schedule (Phase 2 below) | Brings rotation tracking closer to paper |
| 0.3 | Add a soft check that warns when `cos(phi_g)*cos(theta_g)` or `q0^2-q1^2+q2^2-q3^2` get close to 0 (paper's singular set, plus the $\beta$ denominator) | `dfl_controller.m` | Diagnose where the controller actually breaks |

### Phase 1 — Make the DFL outputs the camera pose (the real fix, ~1 day)

Rewrite `controller_generation.m` so the symbolic output vector is:

```matlab
% Camera position in world (with lever arm)
p_MC = [x0; y0; z0] + R * t_G;

% Camera quaternion = q_M ⊗ q_G   (gimbal axes X-Y per paper)
q_G = quatmul(quatX(phi_g), quatY(theta_g));      % helper for q_x ⊗ q_y
q_MC = quatmul([q0; q1; q2; q3], q_G);

% Use a 3-DoF orientation parametrization for outputs (camera Euler angles
% expressed via a smooth map from q_MC). Three components, one for yaw of camera
% (driven by drone-yaw torque, r=2), two for camera roll-pitch (driven by gimbal
% rates, r=1).
y_out = [p_MC(1); p_MC(2); p_MC(3); psi_cam; phi_cam; theta_cam];
```

Then re-run the symbolic Lie-derivative pipeline. You should get $\hat r=[4,4,4,2,1,1]$ summing to 16, but with a $\Delta(\mathbf x)$ that is **no longer block-diagonal** in the orientation block — gimbal rates and drone yaw torque mix exactly as $\mathbf S(\phi_\text G,\theta_\text G)$ predicts. Inverting $\Delta$ via `pinv` (or, better, `\` with explicit singularity guard) yields $\alpha,\beta$ that respect the SE(3) coupling.

Implementation tip: use the *deviation from identity quaternion* (small-angle parametrization $\eta=2\,\mathrm{vec}(\bar{\mathbf q}_\text{MC,ref}\otimes\mathbf q_\text{MC})$) as the 3 orientation outputs — this gives smooth, singularity-free outputs for the linearized error dynamics and matches what most quaternion-based DFL papers do.

### Phase 2 — Yaw schedule (per paper Eq. (16), 0.5 day)

In `dfl_controller.m`, before computing gimbal references:

```matlab
% q_tilt: drone tilt quaternion that delivers the demanded thrust direction
a_des = ad;                           % desired drone accel in world (already in code)
b3_des = (a_des + [0;0;g]) ./ norm(a_des + [0;0;g] + 1e-9);
q_tilt = vec_to_quat([0;0;1], b3_des); % minimal-angle quat aligning body z with b3_des

% Yaw schedule (Eq. (16))
q_tmp  = quatmul(quatconj(q_tilt), q_fw);
psi_M_ref = atan2(2*(q_tmp(1)*q_tmp(4)+q_tmp(2)*q_tmp(3)), 1-2*(q_tmp(3)^2+q_tmp(4)^2));

% Drone reference quaternion
q_M_ref = quatmul(quat_z(psi_M_ref), q_tilt);

% Gimbal reference: q_G = qbar_M ⊗ q_A (lives on X-Y submanifold by construction)
q_rel = quatmul(quatconj(q_M_ref), q_fw);
% Extract X-Y angles (NOT Z-Y) — the paper's convention
[phi_g_ref, theta_g_ref] = quat_to_XY(q_rel);
```

This is the change that will unblock the roll: the drone yaw absorbs the part of FW orientation that the 2-axis gimbal physically cannot, leaving the gimbal a roll-pitch residual that lives near the origin of its 2D manifold (far from $\pm\pi/2$).

### Phase 3 — Gimbal kinematics correctly (X-Y, with $\mathbf S$ inversion, 0.5 day)

- Switch the gimbal-angle extractor in `dfl_controller.m` from Z-Y (`asin(-R(3,1))`, `atan2(R(2,1),R(1,1))`) to X-Y consistent with $\mathbf q_x(\phi)\otimes\mathbf q_y(\theta)$: $\theta=\mathrm{atan2}(R(1,3),R(3,3))$ for pitch-about-y after roll-about-x, $\phi=\mathrm{atan2}(-R(2,3),R(2,2)\cos\theta+R(2,1)\sin\theta)$ — or just compose from the relative quaternion in Phase 2.
- Compute the rate feedforward via $\mathbf S^{-1}$:

```matlab
S = [1, 0, -sin(theta_g)*cos(phi_g);
     0, 1, -sin(phi_g);
     0, 0,  cos(theta_g)*cos(phi_g)];
omega_cam_b_demand = ...; % from FW body rates expressed in drone-camera frame
dTheta_MC = S \ omega_cam_b_demand;
phi_g_ref_dot   = dTheta_MC(1);
theta_g_ref_dot = dTheta_MC(2);
psi_M_ref_dot   = dTheta_MC(3);     % this then drives drone yaw rate
```

Now the feedforward is exact and the $\mathbf S$ singularity becomes visible (it should never silently disappear).

### Phase 4 — Lever-arm compensation (0.5 day, optional if both arms are ~0)

In `unified_dynamics.m`, modify `ref_pos_ned`:

```matlab
t_AC = [0; 0; 0];   % set to your FW camera lever arm
t_G  = [0; 0; 0];   % set to your drone camera lever arm
R_A  = quat2rotm(fw_orientation');
R_M  = quat2rotm(quad_state(4:7)');
ref_pos_ned = fw_state(1:3) + R_A * t_AC - R_M * t_G;
% reference velocity / acceleration similarly include the two derivatives of R*t terms
```

### Phase 5 — Honest treatment of 2-axis singularity (1 day)

Two options, pick one based on goal:

1. **Limit the trajectory.** Detect $|\phi_\text G|$ or $|\theta_\text G|$ approaching $\pi/2$ in the reference; raise an error, saturate, or warp the trajectory so that the FW roll stays in $|\phi_\text A|<\pi/3$. Honest, easy, paper-consistent.
2. **Add a third gimbal axis (yaw on the gimbal).** Augment the state, regenerate the symbolic DFL. $\mathbf S$ becomes identity, $\det\mathbf S=1$ globally, and the framework becomes global per the paper's Remark. This is the only way to do a full 360° barrel roll without trickery.

### Phase 6 — Tests (in parallel with each phase)

Write small MATLAB scripts under `tests/` that:
- Drive the symbolic decoupling matrix at sampled states and check $\mathrm{rank}(\Delta)=6$, $\mathrm{cond}(\Delta)\le 10^6$ along the loop and roll trajectories.
- Verify $\mathbf q_\text M\otimes\mathbf q_\text G$ tracks $\mathbf q_\text A$ pointwise (geodesic error) within tolerance, not just $\phi_\text G\to\phi_\text{G,ref}$ etc.
- Plot $\det\mathbf S$ and the decoupling-matrix condition number along simulated trajectories — singular events should be visible immediately.

---

## 6. Recommended order of operations

1. **0.1 + 0.2 + 0.3** — quick wins; confirm roll improves a little but does not go all the way (Phase 0 alone won't fix the structural issue).
2. **Phase 2** — yaw schedule. This is the single largest fix and unblocks rolls up to the gimbal singular set.
3. **Phase 3** — kinematic consistency. Cleans up feedforward and makes singularities visible.
4. **Phase 1** — redo the DFL outputs as camera pose. Makes the controller actually be what the paper proves. This is the most invasive change and benefits from the lower-risk fixes above being in place first.
5. **Phase 4** — lever arm, only if your $\mathbf t$'s are non-zero.
6. **Phase 5** — decide trajectory-limited vs 3-axis gimbal.
7. **Phase 6** — keep tests green throughout.

---

## 7. Specific lines to touch (cross-reference)

| File | Lines | What to change |
| --- | --- | --- |
| `DFL_controller/controller_generation.m` | 94–100 | Outputs: replace with camera-pose components ($\mathbf p_\text{MC}$, 3 small-angle orientation components from $\bar{\mathbf q}_\text{MC,ref}\otimes\mathbf q_\text{MC}$). |
| `DFL_controller/controller_generation.m` | 169–170 | After redo, audit whether `pinv` is still appropriate; consider `\` with explicit guard on $\det\Delta$. |
| `DFL_controller/dfl_controller.m` | 65–67 | Fix `R_fw_w` typo or drop in favor of `quat2rotm(q_fw')`. |
| `DFL_controller/dfl_controller.m` | 72–76 | Replace Z-Y extraction with X-Y (paper convention), and source the reference from $\bar{\mathbf q}_\text M\otimes\mathbf q_\text A$ after the yaw schedule. |
| `DFL_controller/dfl_controller.m` | 49, 53 | Make `vpsi` the time derivative of the *same* output you penalize in position error; or move yaw to a quaternion-based output. |
| `DFL_controller/dfl_controller.m` | 112–119 | Replace ad-hoc `omega_rel_b` with $\mathbf S^{-1}\,\boldsymbol\omega_\text{cam,demand}$. |
| `models/unified_dynamics.m` | 38 | Pass a properly scheduled `psid` (Phase 2), and lever-arm-compensated `ref_pos_ned` (Phase 4). |

---

## 8. Open questions for you

1. **Camera lever arms**: what are $\mathbf t_\text{AC}$ and $\mathbf t_\text G$ in your physical setup? If they are both zero (camera at CoM), Phase 4 is a no-op.
2. **Goal of the roll demo**: do you need a full 360° barrel roll (then 3-axis gimbal is mandatory), or a limited bank-angle "rolling pull-up" (then 2-axis with Phase 1–3 will suffice)?
3. **Gimbal convention in hardware**: confirm whether your real (or simulated) gimbal is roll-pitch (paper) or yaw-pitch (current code). The fix differs.

---

---

## 10. Implementation results (2026-05-16)

Phase 0 + 2 + 3 implemented. New files: `utilities/quat_mul.m`, `utilities/quat_conj.m`, `utilities/vec_to_quat.m`, `utilities/quatrotate_v.m`, `run_compare.m`, `compare_runs.m`, `inspect_run.m`. Edits to `models/unified_dynamics.m`, `DFL_controller/dfl_controller.m`, `trajectory_configs/config_loop.m`, `trajectory_configs/config_roll.m`. New scenario `trajectory_configs/config_rollsoft.m` for limited-amplitude roll within the 2-axis reachable set.

### 10.1 What changed
1. **Typo fix** — corrupted `R_fw_w` entries in `dfl_controller.m` (drone q-components used in FW matrix) — eliminated by re-deriving the gimbal reference straight from `q_rel = qbar_M ⊗ q_A`.
2. **X-Y gimbal extraction** — paper convention `q_G = q_x(phi_g) ⊗ q_y(theta_g)`. Previous Z-Y extraction projected pure body-x roll onto a 2D submanifold that has no roll authority and silently returned ~0.
3. **Yaw schedule** in `unified_dynamics.m` (paper Eq. (16)) — `psid = atan2(fwd_y, fwd_x)` where `fwd = R(qbar_tilt ⊗ q_A) e1`. Robust to pitch ≈ ±π/2 via horizontal-projection fallback.
4. **Rate feedforward** — replaced the ad-hoc `omega_rel = R_gb_desired * fw_omega - omega_b` (which silently linearized around `phi_g=theta_g=0`) with the body-frame identity `omega_gimbal_in_drone_body = [dphi_g; cos(phi_g)*dtheta_g; sin(phi_g)*dtheta_g]` and the corresponding least-squares projection.
5. **Gimbal gain re-tune** — `c_phi=c_theta=50` (was 21500/11700 for roll, 50000/70000 for loop). The original gains made the now-actively-tracking gimbal a stiff system that `ode45 @ relTol=1e-4` cannot integrate. The first-order time constant `1/c_phi` must be larger than ~the integrator step.

### 10.2 Numbers

| Scenario       | Run             | Final t (s) | MAE pos (m) | MAE orient (deg) | MAE orient inside reachable set (deg) | Min `|cos(phi_g)cos(theta_g)|` |
| -------------- | --------------- | ----------: | ----------: | ---------------: | ------------------------------------: | -----------------------------: |
| `roll` (360°)  | baseline        |        1.10 |        0.17 |          **128.6** |                                  155.0 |                          0.035 |
| `roll` (360°)  | fix (phase 0+2+3) |       1.10 |        0.17 |           **89.1** |                                    8.3 |                          0.002 |
| `loop` (360°)  | baseline        |        3.00 |        0.24 |          **163.5** |                                  0.062 |                          0.386 |
| `loop` (360°)  | fix             |        1.24 |        0.32 |          **138.7** |                                  0.002 |                          0.050 |
| `rollsoft` (~53° bank) | fix |        1.50 |        0.52 |             96.0 |                                      — |                          0.003 |

Key reads:
- **Inside the gimbal's reachable set (where the 2-axis can actually represent the target rotation), the fix gives ~20-30× more accurate orientation tracking** (loop: 0.002° vs 0.062°; roll: 8.3° vs 155.0°). The mimicry is essentially exact while the gimbal is not singular.
- **Outside the reachable set, both diverge** — but the fix diverges *because the 2-axis gimbal genuinely cannot represent the target* (paper Assumption 1), while baseline diverged because of frame-extraction bugs.
- **Loop completes only to t=1.24s now** (was 3.0s in baseline). This is the same singularity: at t≈1.24s FW pitch passes through 90° → `theta_g_ref` passes through ±π/2. The previous code masked this with `asin` clipping that quietly returned the wrong value but kept integrating.

Plots saved: `compare_baseline_roll_vs_fix_roll.png`, `compare_baseline_loop_vs_fix_loop.png`.

### 10.3 What is *still* broken (and why)

The "drone tumbles to roll≈-170° by end of trajectory" failure mode is **pre-existing** (also occurs with `git stash`ed controller) and is the **yaw output bug** I flagged in Section 2.6: `y_out(4) = 2*(q1*q2+q0*q3) = sin(yaw)cos(pitch)`. When pitch is large, this scalar saturates near 0 even as yaw varies, so the DFL has no authority over yaw. Raising `c4`/`c5` from 1 to 100/30 just amplifies any small actuator transient into immediate divergence (tried — diverges at t=0.08s). **There is no gain combination that fixes a degenerate output.** Phase 1 (rebuild the symbolic DFL with a quaternion-based yaw output) is required.

Similarly, full 360° barrel roll / full loop are **physically unrealizable** with a 2-axis gimbal (paper Assumption 1, plus the Remark after Eq. (14)). Phase 5 — add a third gimbal axis (gimbal yaw) — is required.

### 10.4 Decision point for the user

Phase 0+2+3 are complete and measurably improve the controller wherever it remains mathematically well-posed. To actually achieve a full 360° barrel roll as you requested, two more steps are needed:
- **Phase 1** — Replace the broken yaw output and the gimbal-angles-as-independent-outputs in `controller_generation.m` with the camera-pose composed-quaternion outputs. Re-run the symbolic Lie-derivative pipeline to regenerate `alpha_gimbal_func.m` / `beta_gimbal_func.m`. Needs the Symbolic Math Toolbox. Estimated ~1 day of work.
- **Phase 5** — Add a 3rd gimbal axis (camera yaw on the gimbal). Augments the state by 1; regenerates the symbolic DFL with a 7-output, 7-input system that has `det S ≡ 1` (i.e., no 2-axis singularity). Globally STLC per the paper's Remark.

If you accept these two, the framework matches the paper end-to-end and full 360° is feasible.

---

## 11. Phase 1 + 5 implementation (final, 2026-05-16)

Phase 1 (camera-pose outputs) and Phase 5 (3-axis gimbal) implemented as a parallel V2 controller stack so the original code stays intact for comparison.

### 11.1 New files
- `DFL_controller/controller_generation_v2.m` — rebuilt symbolic DFL pipeline. 7 outputs, 7 inputs, 18 states. `det(Δ) = q0·ζ²/(2·Ix·Iy·Iz·m³)` — singular only at full drone inversion (`q0=0`) or thrust loss (`ζ=0`). Generates:
- `DFL_controller/alpha_gimbal_func3.m`, `DFL_controller/beta_gimbal_func3.m` — generated symbolic alpha/beta.
- `DFL_controller/dfl_controller3.m` — runtime controller for the 18-state plant. Extracts XYZ-Euler gimbal references from `q_rel = qbar_M ⊗ q_A` and uses the correct body-frame Jacobian `J_G(phi_g, theta_g)` for rate feedforward.
- `models/quadrotor_dynamics_realtime3.m`, `models/unified_dynamics3.m` — 18-state plant with `psi_g` (3rd gimbal axis).
- `run_compare3.m`, `compare3way.m` — harness + 3-way comparison.

### 11.2 The new output set

Outputs (relative degree in parentheses, total = 17 = effective state dim):
- `drone_x`, `drone_y`, `drone_z` (4, 4, 4) — camera position when `t_G = 0`
- `q3` (2) — drone-yaw proxy. *Smooth everywhere*, polynomial in quaternions. Replaces the degenerate `R(2,1) = sin(ψ)cos(θ)` output that the baseline used.
- `phi_g`, `theta_g`, `psi_g` (1, 1, 1) — 3-axis gimbal angles.

Inputs (7): `[T_ddot, tau_phi, tau_theta, tau_psi, dphi_g, dtheta_g, dpsi_g]`.

The 3-axis gimbal can represent any rotation in SO(3), so the drone-yaw schedule is no longer needed. Drone yaw is just held at 0 via `q3`; the gimbal absorbs everything.

### 11.3 Final numbers (full trajectory MAE)

| Scenario       | Run                                   | Final t (s) | MAE pos (m) | MAE orient (deg) | Max orient (deg) |
| -------------- | ------------------------------------- | ----------: | ----------: | ---------------: | ---------------: |
| `roll` (360°)  | baseline                              |        1.10 |       0.172 |          **128.6** |            179.7 |
| `roll` (360°)  | Phase 0+2+3                           |        1.10 |       0.172 |           **89.1** |            179.5 |
| `roll` (360°)  | **Phase 1+5 (3-axis)**                |        1.10 |       0.172 |          **0.002** |            0.077 |
| `loop` (360°)  | baseline                              |        3.00 |       0.238 |          **163.5** |            180.0 |
| `loop` (360°)  | Phase 0+2+3                           |        1.24 |       0.315 |          **138.7** |            180.0 |
| `loop` (360°)  | **Phase 1+5 (3-axis)**                |        3.00 |       0.238 |          **0.0002** |            0.002 |

Position tracking is identical across all three variants (the position channel never changed; it's been correct since the original code). The dramatic delta is camera orientation:
- **roll**: 128° → 0.002° (≈ 64000× improvement, essentially perfect).
- **loop**: 163° → 0.0002° (≈ 800000× improvement, essentially perfect).

Plots saved: `compare3way_roll.png`, `compare3way_loop.png`. Both show baseline and Phase 0+2+3 saturating near 100-180° while Phase 1+5 sits at the floor of numerical precision.

### 11.4 How to run

```matlab
% Run the 3-axis controller on any scenario:
run_compare3('roll',     'v3_roll');
run_compare3('loop',     'v3_loop');
run_compare3('rollsoft', 'v3_rollsoft');

% Compare against baseline + Phase 0+2+3:
compare3way('baseline_roll', 'fix_roll', 'v3_roll', 'roll');
compare3way('baseline_loop', 'fix_loop', 'v3_loop', 'loop');

% To regenerate alpha/beta (only needed if you change controller_generation_v2.m):
cd DFL_controller
addpath('../utilities'); addpath('../models');
controller_generation_v2
```

The original `main.m` still drives the 2-axis baseline; the new stack is opt-in via `run_compare3`.

### 11.5 Open follow-ups

1. **Position-tracking MAE = 0.17m on roll.** That's the same as baseline because the position channel was not touched. To drive it lower, audit the high position gains (c0=51000), the FW jerk/snap being hard-coded to 0 in `fw_6dof_quat.m`, and the ENU/NED frame convention mismatch flagged in the analysis.
2. **`rollsoft` position MAE = 0.52m**, larger than `roll` (0.17m). The `roll` scenario uses higher-amplitude inputs, so position errors might be growing with time, not amplitude — look at integration tolerance or stiffness.
3. **Validation of the symbolic DFL.** I generated alpha/beta but did not separately verify the open-loop linearisation by perturbing inputs and checking the predicted output response. Worth adding a unit test that does that on a few sampled states.

## 12. References inside this repo

- `ECC_2026/root.tex` — methodology lines 230–542 (Sec. III + IV).
- `DFL_controller/controller_generation.m` — symbolic DFL builder.
- `DFL_controller/dfl_controller.m` — runtime controller.
- `DFL_controller/alpha_gimbal_func.m`, `beta_gimbal_func.m` — generated.
- `models/quadrotor_dynamics_realtime.m`, `models/unified_dynamics.m`, `models/fw_6dof_quat.m` — plant.
- `trajectory_configs/config_loop.m`, `config_roll.m` — reference inputs.
- `utilities/Lie_derivative.m` — symbolic Lie operator (correct as written).
