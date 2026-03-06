function u_gimbal = geometric_gimbal_controller(state, fw_state, dfl_gains)
% GEOMETRIC_GIMBAL_CONTROLLER  SO(3) geometric control for 2-DOF roll-pitch gimbal.
%
% Implements the gimbal orientation matching from Section III-C of the paper:
%   q_G = q_M^{-1} * q_A  (eq. 12)
%
% The gimbal tracks the relative orientation between the drone body and
% the fixed-wing aircraft, so that the camera's global orientation matches
% the fixed-wing's orientation: R_bw * R_gb = R_fw_w.
%
% The controller uses geometric error on SO(3) (vee map of skew-symmetric
% error) with feedforward from the fixed-wing's angular velocity.
%
% Gimbal convention: R_gb = R_x(phi_g) * R_y(theta_g)
%   - phi_g: roll (rotation about body x-axis)
%   - theta_g: pitch (rotation about body y-axis)
%   - The drone's free yaw provides the 3rd rotational DOF
%
% Inputs:
%   state     - 17-element quadrotor+gimbal state
%   fw_state  - 13-element fixed-wing state
%   dfl_gains - struct with kp_R_gimbal and kp_omega_gimbal
%
% Outputs:
%   u_gimbal  - [phi_g_dot; theta_g_dot] gimbal joint rate commands

% -----------------------------------------------------------------------
% Unpack states
% -----------------------------------------------------------------------
q_bw = state(4:7);      % Drone quaternion (body to world)
omega_b = state(11:13); % Drone angular velocity in body frame
phi_g = state(14);      % Gimbal roll angle
theta_g = state(15);    % Gimbal pitch angle

% Controller gains
kp_R_gimbal = dfl_gains.kp_R_gimbal;      % Orientation error gain
kp_omega_gimbal = dfl_gains.kp_omega_gimbal;  % Angular velocity damping gain

% -----------------------------------------------------------------------
% Compute rotation matrices
% -----------------------------------------------------------------------
q_bw = q_bw / (norm(q_bw) + 1e-9);
R_bw = quat2rotm(q_bw');  % Drone body to world

q_fw = fw_state(7:10) / (norm(fw_state(7:10)) + 1e-9);
fw_omega_fw_body = fw_state(11:13);  % FW angular velocity in FW body frame
R_fw_w = quat2rotm(q_fw');  % Fixed-wing body to world

% -----------------------------------------------------------------------
% Desired gimbal orientation (eq. 12: q_G = q_M^{-1} * q_A)
% -----------------------------------------------------------------------
% R_bw * R_gb_desired = R_fw_w  =>  R_gb_desired = R_bw^T * R_fw_w
R_gb_desired = R_bw' * R_fw_w;

% -----------------------------------------------------------------------
% Current gimbal orientation: R_gb = R_x(phi_g) * R_y(theta_g)
% -----------------------------------------------------------------------
c_phi = cos(phi_g);  s_phi = sin(phi_g);
c_theta = cos(theta_g);  s_theta = sin(theta_g);

R_gb = [c_theta,          0,       s_theta;
        s_phi*s_theta,  c_phi,  -s_phi*c_theta;
       -c_phi*s_theta,  s_phi,   c_phi*c_theta];

% -----------------------------------------------------------------------
% SO(3) orientation error (vee map of skew-symmetric error matrix)
% -----------------------------------------------------------------------
% e_R = 0.5 * vee(R_des^T * R - R^T * R_des)
% This is a well-known geometric error metric on SO(3) that vanishes
% when R = R_des and is proportional to the rotation angle for small errors.
R_error_mat = R_gb_desired' * R_gb - R_gb' * R_gb_desired;
e_R = 0.5 * [R_error_mat(3,2); R_error_mat(1,3); R_error_mat(2,1)];

% -----------------------------------------------------------------------
% Feedforward: desired angular velocity from fixed-wing motion
% -----------------------------------------------------------------------
% Transform FW angular velocity: FW body -> world -> drone body
fw_omega_world = R_fw_w * fw_omega_fw_body;
fw_omega_drone_b = R_bw' * fw_omega_world;

% Relative angular velocity = FW rate - drone rate (in drone body frame)
% This is the rate at which the gimbal needs to rotate to track the FW.
omega_rel_b = fw_omega_drone_b - omega_b;

% Transform to gimbal frame for feedforward
omega_d = R_gb' * omega_rel_b;

% -----------------------------------------------------------------------
% Geometric control law
% -----------------------------------------------------------------------
% Desired gimbal angular velocity = -kp * error + feedforward
omega_g_desired = -kp_R_gimbal * e_R + omega_d;

% Add velocity damping (gimbal has no angular velocity state — first-order)
omega_g_current = [0; 0; 0];
omega_g_cmd = omega_g_desired - kp_omega_gimbal * (omega_g_current - omega_d);

% -----------------------------------------------------------------------
% Kinematic inversion: gimbal-frame omega -> joint rates
% -----------------------------------------------------------------------
% For R_gb = R_x(phi_g) * R_y(theta_g), the body-frame angular velocity
% due to joint rates is:
%   omega_body = [phi_dot; cos(phi)*theta_dot; sin(phi)*theta_dot]
%
% Inversion:
%   phi_dot   = omega_x_body
%   theta_dot = omega_y_body / cos(phi)
%
% Singularity at phi_g = ±90° (gimbal lock for roll axis).
omega_cmd_body = R_gb * omega_g_cmd;

if abs(c_phi) < 0.05
    % Near gimbal lock — freeze pitch, only command roll
    phi_g_dot = omega_cmd_body(1);
    theta_g_dot = 0;
else
    phi_g_dot = omega_cmd_body(1);
    theta_g_dot = omega_cmd_body(2) / c_phi;
end

u_gimbal = [phi_g_dot; theta_g_dot];

end
