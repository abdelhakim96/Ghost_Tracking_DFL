function u_gimbal = geometric_gimbal_controller(state, fw_state, dfl_gains)
% GEOMETRIC_GIMBAL_CONTROLLER  SO(3) geometric control for 2-DOF roll-pitch gimbal.
%
% Tracks the relative orientation R_gb_desired = R_bw^T * R_fw_w so that
% the camera's global orientation matches the FW: R_bw * R_gb = R_fw_w.
%
% Gimbal: R_gb = R_x(phi_g) * R_y(theta_g)
%
% Key feature: uses pseudo-inverse kinematic inversion (J^+ = J^T for
% this Jacobian) which is singularity-free at all gimbal angles.

% -----------------------------------------------------------------------
% Unpack states
% -----------------------------------------------------------------------
q_bw = state(4:7);
omega_b = state(11:13);
phi_g = state(14);
theta_g = state(15);

kp_R = dfl_gains.kp_R_gimbal;
kd   = dfl_gains.kp_omega_gimbal;

% -----------------------------------------------------------------------
% Rotation matrices
% -----------------------------------------------------------------------
q_bw = q_bw / (norm(q_bw) + 1e-9);
R_bw = quat2rotm(q_bw');

q_fw = fw_state(7:10) / (norm(fw_state(7:10)) + 1e-9);
fw_omega_fw_body = fw_state(11:13);
R_fw_w = quat2rotm(q_fw');

% -----------------------------------------------------------------------
% Desired vs current gimbal orientation
% -----------------------------------------------------------------------
R_gb_desired = R_bw' * R_fw_w;

c_phi = cos(phi_g);  s_phi = sin(phi_g);
c_theta = cos(theta_g);  s_theta = sin(theta_g);

R_gb = [c_theta,          0,       s_theta;
        s_phi*s_theta,  c_phi,  -s_phi*c_theta;
       -c_phi*s_theta,  s_phi,   c_phi*c_theta];

% -----------------------------------------------------------------------
% SO(3) orientation error (vee map)
% -----------------------------------------------------------------------
R_err = R_gb_desired' * R_gb - R_gb' * R_gb_desired;
e_R = 0.5 * [R_err(3,2); R_err(1,3); R_err(2,1)];

% -----------------------------------------------------------------------
% Feedforward: relative angular velocity in gimbal frame
% -----------------------------------------------------------------------
fw_omega_world = R_fw_w * fw_omega_fw_body;
fw_omega_drone_b = R_bw' * fw_omega_world;
omega_rel_b = fw_omega_drone_b - omega_b;
omega_ff = R_gb' * omega_rel_b;

% -----------------------------------------------------------------------
% Control law: PD on SO(3) error + feedforward
% -----------------------------------------------------------------------
omega_g_cmd = -(kp_R + kd) * e_R + omega_ff;

% -----------------------------------------------------------------------
% Kinematic inversion using pseudo-inverse (SINGULARITY-FREE)
% -----------------------------------------------------------------------
% The body-frame angular velocity from gimbal joint rates is:
%   omega_body = J * [phi_dot; theta_dot]
%   J = [1,      0       ]
%       [0,   cos(phi_g) ]
%       [0,   sin(phi_g) ]
%
% The pseudo-inverse is J^+ = (J^T J)^{-1} J^T = J^T (since J^T J = I).
% This gives:
%   phi_dot   = omega_body(1)
%   theta_dot = cos(phi_g)*omega_body(2) + sin(phi_g)*omega_body(3)
%
% This is equivalent to the standard inversion 1/cos(phi_g) when
% cos(phi_g) != 0, but remains well-defined at phi_g = ±90° where
% it smoothly uses omega_body(3) instead.
% -----------------------------------------------------------------------
omega_cmd_body = R_gb * omega_g_cmd;

phi_g_dot = omega_cmd_body(1);
theta_g_dot = c_phi * omega_cmd_body(2) + s_phi * omega_cmd_body(3);

u_gimbal = [phi_g_dot; theta_g_dot];

end
