# Ghost Tracking via Dynamic Feedback Linearization

A MATLAB simulation framework for a multicopter with a 2-axis gimbal that mimics the camera pose of a fixed-wing aircraft performing aerobatic maneuvers (loop, roll, straight flight). The core idea: a drone + gimbal can reproduce the exact camera viewpoint of an airplane, even during aggressive maneuvers.

## Theory

### Problem Statement

Given a fixed-wing aircraft with a body-fixed camera, compute multicopter and gimbal inputs so that the multicopter's gimballed camera reproduces the same pose (position + orientation) in SE(3) at all times.

### System Models

**Fixed-wing aircraft** (eq. 4 in the paper): 13-state model with position, body-frame velocity, quaternion attitude, and body rates. Full aerodynamic force/moment model with lift, drag, side force, and control surfaces (thrust, aileron, elevator, rudder).

**Multicopter with 2-axis gimbal** (eq. 10): 17-state extended model:
- Position (3), quaternion (4), world-frame velocity (3), body rates (3) = 13 core states
- Gimbal angles: phi_g (roll), theta_g (pitch) = 2 states
- Thrust double integrator: zeta (thrust), xi (thrust rate) = 2 DFL extended states

The gimbal rotation is `R_gb = R_x(phi_g) * R_y(theta_g)` — a roll-pitch gimbal where:
- phi_g rotates the camera about the drone's body x-axis (forward)
- theta_g rotates about the y-axis (lateral)

### Camera-Pose Mimicry (Section III)

The multicopter camera pose `y_MC = [p_M + R(q_M)*t_G, q_M * q_G]` must equal the fixed-wing camera pose `y_AC = [p_A + R(q_A)*t_AC, q_A]`.

**Position matching**: The DFL controller tracks the fixed-wing position (with lever-arm compensation if applicable).

**Orientation matching**: `q_M * q_G = q_A`, so `q_G = q_M^{-1} * q_A`. The drone's attitude splits into:
- **Tilt** (set by thrust direction from position tracking)
- **Free yaw** (scheduled to keep gimbal angles feasible)

The gimbal then absorbs the remaining roll-pitch offset.

### Output Full Actuation on SE(3) (Section III-A)

The Jacobian from inputs to camera acceleration/twist is block-triangular (eq. 9):
```
J = [I_3,  0  ]
    [ 0,  S(phi,theta)]
```
where S is the input-twist matrix with `det(S) = cos(theta)*cos(phi)`. Since `rank(J) = 6` away from gimbal singularities, the camera pose is **output fully actuated** and **small-time locally controllable (STLC)** on SE(3).

### Dynamic Feedback Linearization (Section IV)

Standard static feedback linearization fails because the decoupling matrix is singular — thrust affects acceleration directly but not through the attitude torques. The solution: add a **double integrator on thrust** (eq. 16), making thrust an internal state driven by its second derivative.

**Extended system**: 15 effective states (17 total - 2 gimbal, which are controlled separately), 4 inputs `[ddot_T, tau_phi, tau_theta, tau_psi]`, 4 outputs `[x, y, z, R(2,1)]`.

**Relative degrees**: `r = [4, 4, 4, 2]`, sum = 14 = 15 - 1 (quaternion constraint), satisfying the exact linearization condition (eq. 17).

The DFL control law (eq. 19):
```
u_hat = alpha(x) + beta(x) * v
```
where:
- `alpha(x) = -Delta(x)^{-1} * b(x)` cancels nonlinear drift
- `beta(x) = Delta(x)^{-1}` decouples input-output channels
- `v = [v_pos; v_yaw]` is the virtual control with error feedback

**Position virtual control** (4th-order error dynamics):
```
v_pos = snap_ref - c3*(jerk - jerk_ref) - c2*(acc - acc_ref) - c1*(vel - vel_ref) - c0*(pos - pos_ref)
```

**Yaw virtual control** (2nd-order, using R(2,1) = sin(yaw)*cos(pitch)):
```
v_yaw = -c5*(R21_dot - R21_dot_ref) - c4*(R21 - R21_ref)
```

### Gimbal Control (Geometric SO(3))

The gimbal tracks the relative orientation `R_gb_desired = R_bw^T * R_fw_w` using:
1. **SO(3) error**: `e_R = 0.5 * vee(R_des^T * R - R^T * R_des)`
2. **Feedforward**: relative angular velocity between FW and drone, transformed to gimbal frame
3. **Kinematic inversion**: maps gimbal-frame angular velocity to joint rates `[phi_dot, theta_dot]`

## Project Structure

```
Ghost_Tracking_DFL/
├── main.m                          # Entry point — runs simulation
├── README.md                       # This file
├── DFL_controller/
│   ├── controller_generation.m     # Symbolic derivation of alpha/beta (Lie derivatives)
│   ├── alpha_func.m                # Auto-generated: drift cancellation term
│   ├── beta_func.m                 # Auto-generated: decoupling matrix inverse
│   ├── dfl_controller.m            # Real-time DFL control law
│   ├── geometric_gimbal_controller.m  # SO(3) geometric gimbal controller
│   ├── gimbal_controller.m         # Alternative kinematic gimbal controller
│   ├── alpha_gimbal_func.m         # Auto-generated gimbal alpha (legacy)
│   └── beta_gimbal_func.m          # Auto-generated gimbal beta (legacy)
├── models/
│   ├── unified_dynamics.m          # ODE function combining FW + quad dynamics
│   ├── fw_6dof_quat.m             # 6-DOF fixed-wing model with full aerodynamics
│   └── quadrotor_dynamics_realtime.m  # Quadrotor + gimbal dynamics with DFL
├── trajectory_configs/
│   ├── config_loop.m              # Loop maneuver configuration
│   ├── config_roll.m              # Roll maneuver configuration
│   └── config_straight.m          # Straight flight configuration
├── utilities/
│   ├── plot_results.m             # Comprehensive result visualization
│   ├── Lie_derivative.m           # Recursive Lie derivative computation
│   ├── wrapToPi.m                 # Angle wrapping to [-pi, pi]
│   ├── unwrapAngle.m              # Angle unwrapping for smooth plotting
│   ├── correctAngleJump.m         # Angle jump correction
│   ├── neglectAngleJump.m         # Angle jump rejection
│   └── stlread.m                  # STL file reader for CAD models
├── CAD/
│   ├── aero.stl                   # Fixed-wing aircraft 3D model
│   └── quad.stl                   # Quadrotor 3D model
└── results/                       # Generated result plots (PDF)
```

## How to Run

1. Open MATLAB and navigate to the repository root.
2. Open `main.m` and set `config_to_run` to the desired maneuver:
   - `'loop'` — vertical loop
   - `'roll'` — aileron roll
   - `'straight'` — level flight
3. Run `main.m`. The simulation uses `ode45` and generates 5 result plots:
   - 3D trajectory with STL vehicle models
   - Position and orientation tracking comparison
   - Control inputs (thrust, moments, gimbal rates)
   - Drone state evolution
   - Debug angle comparison plots

Results are saved as PDFs in `results/results_<maneuver>/`.

## Simulation Flow

```
main.m
 ├── Load config_<maneuver>.m (parameters, gains, initial conditions)
 ├── Initialize combined state [quad(17); fw(13)]
 └── ode45(@unified_dynamics)
      ├── fw_6dof_quat()        → FW state derivative + acc/jerk/snap reference
      └── quadrotor_dynamics_realtime()  → Quad state derivative
           ├── dfl_controller()          → [ddot_T, tau_phi, tau_theta, tau_psi]
           │    ├── alpha_func()         → nonlinearity cancellation
           │    └── beta_func()          → input-output decoupling
           └── geometric_gimbal_controller() → [phi_g_dot, theta_g_dot]
```

## Regenerating the DFL Controller

If you modify the system dynamics or outputs, regenerate `alpha_func.m` and `beta_func.m`:

1. Navigate to `DFL_controller/`
2. Run `controller_generation.m` (requires Symbolic Math Toolbox)
3. The script computes Lie derivatives symbolically, derives the decoupling matrix and drift terms, and generates optimized MATLAB functions.

Note: The gravity constant `g` does not appear in the generated functions because it is constant and vanishes after repeated differentiation (Lie derivatives eliminate constant terms at orders > 2).

## Key Design Decisions

1. **R(2,1) for yaw output**: Using the rotation matrix element `R(2,1) = sin(yaw)*cos(pitch)` instead of the Euler yaw angle avoids gimbal-lock singularities at pitch = ±90° and provides smooth quaternion-based control.

2. **Roll-pitch gimbal (R_x * R_y)**: The gimbal has roll (x-axis) and pitch (y-axis) DOFs. Combined with the drone's free body yaw, this provides full 3-DOF rotational control of the camera. Singularity occurs at `phi_g = ±90°`.

3. **Dynamic thrust extension**: A double integrator on thrust delays its appearance to higher-order derivatives, raising the relative degree from [2,2,2,2] (singular) to [4,4,4,2] (non-singular), enabling exact input-output decoupling.

4. **Jerk feedforward**: The fixed-wing model computes NED-frame jerk analytically as `R * (omega x F/m)`, providing feedforward for the DFL's 4th-order position channel. This significantly improves tracking during aggressive maneuvers.

## Gain Tuning Guide

### Position channel (c0-c3)
The 4th-order error dynamics have characteristic polynomial `s^4 + c3*s^3 + c2*s^2 + c1*s + c0`. For critically-damped response with bandwidth `w`:
- `c0 = w^4`, `c1 = 4*w^3`, `c2 = 6*w^2`, `c3 = 4*w`

### Yaw channel (c4-c5)
2nd-order: `s^2 + c5*s + c4`. For bandwidth `wn` and damping `zeta`:
- `c4 = wn^2`, `c5 = 2*zeta*wn`

### Gimbal (kp_R_gimbal, kp_omega_gimbal)
- `kp_R_gimbal`: proportional gain on SO(3) orientation error
- `kp_omega_gimbal`: derivative gain (damping) on angular velocity error
