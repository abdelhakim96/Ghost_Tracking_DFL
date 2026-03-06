function u = dfl_controller(t, state_full, xd, vd, ad, jd, sd, psid, fw_state, dfl_gains)
% DFL_CONTROLLER  Unified DFL controller for multicopter position + gimbal.
%
% Implements the paper's control architecture with 6 effective outputs:
%   [x, y, z]      - position tracking via DFL (relative degree 4)
%   R_bw(2,1)       - drone yaw via DFL (relative degree 2)
%   [phi_G, theta_G] - gimbal angles via direct rate control (relative degree 1)
%
% The decoupling matrix is block-diagonal: the position+yaw block uses the
% symbolically-generated alpha_func/beta_func, and the gimbal block is the
% identity (gimbal rates are direct inputs).
%
% Yaw scheduling (paper eq. 10): the drone yaw reference is extracted from
% the FW orientation so the gimbal only needs roll-pitch to match the camera.
%
% Gimbal control: feedforward from FW angular velocity + proportional
% feedback on gimbal angle error, with angle wrapping for 360-degree rolls.
%
% Inputs:
%   state_full - 17-element state: [pos(3); quat(4); vel_w(3); omega_b(3);
%                                    phi_g; theta_g; zeta; xi]
%   xd, vd     - reference position and velocity (from fixed-wing)
%   ad, jd, sd - reference acceleration, jerk, snap
%   psid       - (unused, yaw scheduled internally)
%   fw_state   - fixed-wing state [pos(3); vel_b(3); quat(4); omega_b(3)]
%   dfl_gains  - struct with gains c0..c5, kp_gimbal
%
% Outputs:
%   u - 6-element control: [ddot_T; tau_phi; tau_theta; tau_psi;
%                            phi_g_dot; theta_g_dot]

global m Ix Iy Iz g

% -----------------------------------------------------------------------
% Unpack state vector (17 states)
% -----------------------------------------------------------------------
x_w = state_full(1:3);       % Position in World Frame [N, E, D]
q_bw = state_full(4:7);      % Quaternion Body->World [q0, q1, q2, q3]
v_w = state_full(8:10);      % Velocity in World Frame
omega_b = state_full(11:13); % Angular velocity in Body Frame [p, q, r]
phi_g = state_full(14);      % Gimbal roll angle
theta_g = state_full(15);    % Gimbal pitch angle
zeta = state_full(16);       % Thrust (DFL extended state)
xi = state_full(17);         % Thrust rate (DFL extended state)

% Gains
c0 = dfl_gains.c0;  c1 = dfl_gains.c1;
c2 = dfl_gains.c2;  c3 = dfl_gains.c3;
c4 = dfl_gains.c4;  c5 = dfl_gains.c5;
kp_gimbal = dfl_gains.kp_gimbal;

% -----------------------------------------------------------------------
% Rotation matrices
% -----------------------------------------------------------------------
q_bw = q_bw / (norm(q_bw) + 1e-9);
R_bw = quat2rotm(q_bw');

% FW orientation
q_fw = fw_state(7:10) / (norm(fw_state(7:10)) + 1e-9);
R_fw = quat2rotm(q_fw');
fw_omega_body = fw_state(11:13);

% -----------------------------------------------------------------------
% Position virtual control (relative degree 4) — unchanged
% -----------------------------------------------------------------------
F_thrust = R_bw * [0; 0; zeta];
a_ = (F_thrust/m) - [0; 0; g];

% Jerk: j = (zeta*(R(:,1)*q - R(:,2)*p) + R(:,3)*xi) / m
j = (zeta*(R_bw(:,1)*omega_b(2) - R_bw(:,2)*omega_b(1)) + R_bw(:,3)*xi)/m;

v_pos = sd - c3*(j - jd) - c2*(a_ - ad) - c1*(v_w - vd) - c0*(x_w - xd);

% -----------------------------------------------------------------------
% Yaw channel: schedule yaw from FW orientation (paper eq. 10)
% -----------------------------------------------------------------------
% Extract yaw from FW world-frame orientation.
% During a roll, the FW's heading (yaw) stays approximately constant
% because it flies mostly straight, so this gives a smooth reference.
psi_ref = atan2(R_fw(2,1), R_fw(1,1));

% Desired R_bw(2,1) for the drone = sin(psi_ref)
% (valid when drone pitch is small, which it is since drone stays level)
y4_ref = sin(psi_ref);

% Current drone R(2,1)
y4_drone = R_bw(2,1);

% Time derivative: R_dot(2,1) = R(2,2)*r - R(2,3)*q
y4_dot_drone = R_bw(2,2)*omega_b(3) - R_bw(2,3)*omega_b(2);

% Reference rate: d/dt R_fw(2,1) is approximately 0 during a roll
% (FW heading barely changes). For loops, it changes slowly.
y4_dot_ref = 0;

% Virtual control for yaw channel
v_yaw = -c5 * (y4_dot_drone - y4_dot_ref) - c4 * (y4_drone - y4_ref);

% -----------------------------------------------------------------------
% DFL inversion for position + yaw (4 outputs, 4 controls)
% -----------------------------------------------------------------------
v_4 = [v_pos; v_yaw];

% Regularize zeta to prevent 1/zeta singularity
zeta_min = 0.1 * m * g;  % 10% of hover thrust
drone_state = [state_full(1:13); state_full(16:17)];  % 15-state for alpha/beta
state_safe = drone_state;
if abs(state_safe(14)) < zeta_min
    state_safe(14) = zeta_min * sign(state_safe(14) + 1e-10);
end

alpha_val = alpha_func(state_safe, 0, 0, 0, Ix, Iy, Iz, zeta, xi, m);
beta_val = beta_func(state_safe, 0, 0, 0, Ix, Iy, Iz, zeta, xi, m);
u_drone = alpha_val + beta_val * v_4;

% -----------------------------------------------------------------------
% Gimbal channel: track FW orientation via drone-relative angles
% -----------------------------------------------------------------------
% Desired gimbal rotation: R_G_desired = R_M^T * R_A
% This gives the rotation the gimbal must achieve so that
% R_camera = R_M * R_G = R_A (camera matches FW orientation).
R_G_desired = R_bw' * R_fw;

% Extract gimbal angles from R_G = R_x(phi) * R_y(theta):
%   R = [cos(theta),          0,       sin(theta) ]
%       [sin(phi)*sin(theta),  cos(phi), -sin(phi)*cos(theta)]
%       [-cos(phi)*sin(theta), sin(phi),  cos(phi)*cos(theta)]
%
% phi   = atan2(R(3,2), R(2,2))
% theta = atan2(R(1,3), R(1,1))
phi_G_ref = atan2(R_G_desired(3,2), R_G_desired(2,2));
theta_G_ref = atan2(R_G_desired(1,3), R_G_desired(1,1));

% Feedforward: relative angular velocity projected onto gimbal axes.
% The gimbal must track the FW's rotation rate minus the drone's rotation rate.
fw_omega_world = R_fw * fw_omega_body;
drone_omega_world = R_bw * omega_b;
rel_omega_world = fw_omega_world - drone_omega_world;
rel_omega_body = R_bw' * rel_omega_world;

% Project onto gimbal joint rates using pseudo-inverse of the Jacobian:
%   J = [1, 0; 0, cos(phi_g); 0, sin(phi_g)]
%   J^+ * omega_body = [omega(1); cos(phi)*omega(2) + sin(phi)*omega(3)]
c_phi = cos(phi_g); s_phi = sin(phi_g);
phi_G_dot_ff = rel_omega_body(1);
theta_G_dot_ff = c_phi * rel_omega_body(2) + s_phi * rel_omega_body(3);

% Angle error with wrapping (handles 360-degree rolls)
e_phi = wrapToPi(phi_G_ref - phi_g);
e_theta = wrapToPi(theta_G_ref - theta_g);

% Combined: feedforward + proportional feedback
phi_G_dot_cmd = phi_G_dot_ff + kp_gimbal * e_phi;
theta_G_dot_cmd = theta_G_dot_ff + kp_gimbal * e_theta;

% -----------------------------------------------------------------------
% Assemble full 6-element control vector
% -----------------------------------------------------------------------
u = [u_drone; phi_G_dot_cmd; theta_G_dot_cmd];

end
