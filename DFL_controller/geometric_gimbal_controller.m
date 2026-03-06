function u_gimbal = geometric_gimbal_controller(state, fw_state, dfl_gains)
% GEOMETRIC_GIMBAL_CONTROLLER  SO(3) geometric control for 2-DOF roll-pitch gimbal.
%
% Tracks the relative orientation between drone and fixed-wing so that
% the camera's global orientation matches the FW: R_bw * R_gb = R_fw_w.
%
% Gimbal convention: R_gb = R_x(phi_g) * R_y(theta_g)
%   phi_g:   roll (rotation about body x-axis)
%   theta_g: pitch (rotation about body y-axis)
%
% Control law: omega_gimbal = -kp * e_R + feedforward
% with kinematic inversion to get joint rates.

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
% Desired gimbal orientation: R_gb_des = R_bw^T * R_fw_w
% -----------------------------------------------------------------------
R_gb_desired = R_bw' * R_fw_w;

% -----------------------------------------------------------------------
% Current gimbal: R_gb = R_x(phi_g) * R_y(theta_g)
% -----------------------------------------------------------------------
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
% FW body rates -> world -> drone body -> gimbal frame
fw_omega_world = R_fw_w * fw_omega_fw_body;
fw_omega_drone_b = R_bw' * fw_omega_world;
omega_rel_b = fw_omega_drone_b - omega_b;
omega_ff = R_gb' * omega_rel_b;

% -----------------------------------------------------------------------
% Control law: proportional + feedforward (NO fake damping)
%
% Previously had: omega_cmd = desired - kd*(0 - omega_ff)
% This AMPLIFIED the feedforward by (1+kd) because omega_current=0
% is unknown, not zero. Removed to prevent oscillation/instability.
% -----------------------------------------------------------------------
omega_g_cmd = -kp_R * e_R + omega_ff;

% Optional: add light damping based on the error derivative estimate
% e_R_dot ≈ -omega_rel (in gimbal frame), so damping on omega_ff:
omega_g_cmd = omega_g_cmd - kd * e_R;

% -----------------------------------------------------------------------
% Kinematic inversion: gimbal omega -> joint rates
% -----------------------------------------------------------------------
% For R_x(phi)*R_y(theta), body-frame angular velocity from joints is:
%   omega_body = [phi_dot; cos(phi)*theta_dot; sin(phi)*theta_dot]
% Inversion: phi_dot = omega_body_x, theta_dot = omega_body_y / cos(phi)
omega_cmd_body = R_gb * omega_g_cmd;

if abs(c_phi) < 0.05
    phi_g_dot = omega_cmd_body(1);
    theta_g_dot = 0;
else
    phi_g_dot = omega_cmd_body(1);
    theta_g_dot = omega_cmd_body(2) / c_phi;
end

u_gimbal = [phi_g_dot; theta_g_dot];

end
