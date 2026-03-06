function u = dfl_controller(t, state, xd, vd, ad, jd, sd, psid, fw_state, dfl_gains)
% DFL_CONTROLLER  Dynamic Feedback Linearization controller for the quadrotor.
%
% Implements the DFL control law from eq. (19) in the paper:
%   u_hat = alpha(x_hat) + beta(x_hat) * v
%
% The system has 4 outputs: [x, y, z, R(2,1)] with relative degrees
% [4, 4, 4, 2], totalling 14 = effective state dim (15 states - 1 quat constraint).
%
% Inputs:
%   t         - current time
%   state     - 15-element drone state: [pos(3); quat(4); vel_w(3); omega_b(3); zeta; xi]
%   xd, vd    - reference position and velocity (from fixed-wing)
%   ad, jd    - reference acceleration and jerk (from fixed-wing)
%   sd        - reference snap (4th derivative of position)
%   psid      - desired yaw (unused, yaw scheduled from R(2,1))
%   fw_state  - fixed-wing state [pos(3); vel_b(3); quat(4); omega_b(3)]
%   dfl_gains - struct with gains c0..c5
%
% Outputs:
%   u - 4-element control: [ddot_T; tau_phi; tau_theta; tau_psi]

% Define global variables
global m Ix Iy Iz g

% -----------------------------------------------------------------------
% Unpack the drone state vector (15 states)
% -----------------------------------------------------------------------
x_w = state(1:3);       % Position in World Frame [N, E, D]
q_bw = state(4:7);      % Quaternion from Body to World [q0, q1, q2, q3]
v_w = state(8:10);      % Velocity in World Frame
omega_b = state(11:13); % Angular velocity in Body Frame [p, q, r]
zeta = state(14);       % Total thrust (extended state from DFL compensator)
xi = state(15);         % Derivative of total thrust (extended state)

% -----------------------------------------------------------------------
% Controller gains
% -----------------------------------------------------------------------
% c0..c3: position channel gains (4th-order: snap = feedforward - c3*jerk_err
%         - c2*acc_err - c1*vel_err - c0*pos_err)
% c4, c5: yaw channel gains (2nd-order: R(2,1)_ddot = -c4*R21_err - c5*R21_dot_err)
c0 = dfl_gains.c0;  % Position error gain
c1 = dfl_gains.c1;  % Velocity error gain
c2 = dfl_gains.c2;  % Acceleration error gain
c3 = dfl_gains.c3;  % Jerk error gain
c4 = dfl_gains.c4;  % R(2,1) error gain
c5 = dfl_gains.c5;  % R(2,1) rate error gain

% -----------------------------------------------------------------------
% Normalize quaternion and compute rotation matrix
% -----------------------------------------------------------------------
q_bw = q_bw / (norm(q_bw) + 1e-9);
R_bw = quat2rotm(q_bw');   % Rotation matrix from Body to World

% -----------------------------------------------------------------------
% Compute current acceleration and jerk for position error feedback
% -----------------------------------------------------------------------
% Acceleration in world frame: a = R*[0;0;T]/m - [0;0;g]
F_thrust = R_bw * [0; 0; zeta];
a_ = (F_thrust/m) - [0; 0; g];

% Jerk in world frame: j = d/dt(a)
% Derived from: d/dt(R*[0;0;T]) = R*([omega]_x*[0;0;T]) + R*[0;0;T_dot]
% [omega]_x * [0;0;T] = [q*T; -p*T; 0], so:
%   j = (T*(R(:,1)*q - R(:,2)*p) + R(:,3)*xi) / m
j = (zeta*(R_bw(:,1)*omega_b(2) - R_bw(:,2)*omega_b(1)) + R_bw(:,3)*xi)/m;

% -----------------------------------------------------------------------
% Position virtual control (relative degree 4)
% -----------------------------------------------------------------------
% v_pos represents the desired snap (4th derivative of position).
% The DFL transforms this into the actual control inputs via alpha/beta.
v_pos = sd - c3*(j - jd) - c2*(a_ - ad) - c1*(v_w - vd) - c0*(x_w - xd);

% -----------------------------------------------------------------------
% Yaw virtual control using R(2,1) (relative degree 2)
% -----------------------------------------------------------------------
% The 4th DFL output is y4 = R_bw(2,1) = 2*(q1*q2 + q0*q3), which
% corresponds to sin(yaw)*cos(pitch). This is a smooth quaternion-based
% yaw proxy that avoids Euler angle singularities.

% Current drone R(2,1) and its time derivative
y4_drone = R_bw(2,1);

% Time derivative of R(2,1):
%   R_dot = R * [omega_b]_x
%   R_dot(2,1) = R(2,:) * [omega_b]_x(:,1)
%   where [omega_b]_x(:,1) = [0; r; -q]
%   So R_dot(2,1) = R(2,2)*r - R(2,3)*q
y4_dot_drone = R_bw(2,2)*omega_b(3) - R_bw(2,3)*omega_b(2);

% Extract fixed-wing orientation for yaw reference
fw_orientation = fw_state(7:10);
q_fw = fw_orientation / (norm(fw_orientation) + 1e-9);
R_fw_w = quat2rotm(q_fw');

% Reference R(2,1) from fixed-wing
y4_ref = R_fw_w(2,1);

% Reference R(2,1) time derivative using fixed-wing body rates
fw_omega_b = fw_state(11:13);  % [p_fw; q_fw; r_fw]
y4_dot_ref = R_fw_w(2,2)*fw_omega_b(3) - R_fw_w(2,3)*fw_omega_b(2);

% Virtual control for yaw channel: desired y4_ddot
% Second-order error dynamics: y4_ddot = v_yaw
%   v_yaw = y4_ref_ddot - c5*(y4_dot - y4_dot_ref) - c4*(y4 - y4_ref)
% We approximate y4_ref_ddot ≈ 0 (smooth reference assumption)
v_yaw = -c5 * (y4_dot_drone - y4_dot_ref) - c4 * (y4_drone - y4_ref);

% -----------------------------------------------------------------------
% Combine virtual control and apply DFL (eq. 19)
% -----------------------------------------------------------------------
v = [v_pos; v_yaw];

% alpha(x): drift cancellation term (L_f^r h)
% beta(x):  decoupling matrix inverse (L_g L_f^{r-1} h)^{-1}
% These are auto-generated by controller_generation.m using Lie derivatives.
% Aerodynamic moment parameters (Ap,Aq,Ar) set to 0 — no external moments modeled.
alpha_val = alpha_func(state, 0, 0, 0, Ix, Iy, Iz, zeta, xi, m);
beta_val = beta_func(state, 0, 0, 0, Ix, Iy, Iz, zeta, xi, m);
u = alpha_val + beta_val * v;

end
