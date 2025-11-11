function u_gimbal = geometric_gimbal_controller(state, fw_state, dfl_gains)
% This function computes the control input for a 2-DOF gimbal (roll-pitch)
% using a geometric controller on SO(3) to track a fixed-wing aircraft's orientation.

% Unpack the state vector
q_bw = state(4:7);      % Quaternion from Body to World
omega_b = state(11:13); % Angular velocity in Body Frame
phi_g = state(14);      % Gimbal roll
theta_g = state(15);    % Gimbal pitch

% Controller gains
kp_R_gimbal = dfl_gains.kp_R_gimbal; 
kp_omega_gimbal = dfl_gains.kp_omega_gimbal; 

% Normalize the quaternion
q_bw = q_bw / (norm(q_bw) + 1e-9);

% Rotation matrix from Body to World
R_bw = quat2rotm(q_bw');

% Extract fixed-wing orientation and angular velocity
q_fw = fw_state(7:10) / (norm(fw_state(7:10)) + 1e-9);
fw_omega_fw_body = fw_state(11:13); % Angular velocity in fixed-wing body frame [p; q; r]
R_fw_w = quat2rotm(q_fw');

% === DESIRED GIMBAL ORIENTATION ===
% The gimbal (attached to drone body) should orient such that:
% R_gimbal_world = R_fw_world
% R_body_world * R_gimbal_body = R_fw_world
% Therefore: R_gimbal_body_desired = R_body_world^T * R_fw_world
R_gb_desired = R_bw' * R_fw_w;

% === CURRENT GIMBAL ORIENTATION ===
% Build the rotation matrix consistent with your dynamics
% This represents: Ry(theta_g) * Rx(phi_g) - pitch then roll
c_phi = cos(phi_g);
s_phi = sin(phi_g);
c_theta = cos(theta_g);
s_theta = sin(theta_g);

R_gb = [c_phi*c_theta,  -s_phi,  c_phi*s_theta;
        s_phi*c_theta,   c_phi,  s_phi*s_theta;
       -s_theta,         0,      c_theta];

% === ORIENTATION ERROR (SO(3) error using vee map) ===
R_error_mat = R_gb_desired' * R_gb - R_gb' * R_gb_desired;
e_R = 0.5 * [R_error_mat(3,2); R_error_mat(1,3); R_error_mat(2,1)]; % vee map

% === FEEDFORWARD: DESIRED ANGULAR VELOCITY ===
% fw_omega_fw_body is in fixed-wing body frame, transform to drone body frame
% Step 1: fw body -> world frame
fw_omega_world = R_fw_w * fw_omega_fw_body;
% Step 2: world -> drone body frame
fw_omega_drone_b = R_bw' * fw_omega_world;

% Relative angular velocity in drone body frame
omega_rel_b = fw_omega_drone_b - omega_b;

% Transform to gimbal frame
omega_d = R_gb' * omega_rel_b;

% === CONTROL LAW (GEOMETRIC CONTROL ON SO(3)) ===
% Desired angular velocity in gimbal frame
omega_g_desired = -kp_R_gimbal * e_R + omega_d;

% Add damping on angular velocity error
% Current gimbal angular velocity in gimbal frame
omega_g_current = [0; 0; 0]; % First-order gimbal has no measured angular velocity state

% Control output with damping
omega_g_cmd = omega_g_desired - kp_omega_gimbal * (omega_g_current - omega_d);

% === KINEMATIC INVERSION ===
% Map from gimbal frame angular velocity to gimbal joint rates
% For pitch-then-roll (Ry * Rx) gimbal:
% omega_gimbal = [p_g; q_g; r_g] relates to [phi_dot; theta_dot] as:
% p_g = phi_dot * cos(theta_g)
% q_g = theta_dot
% r_g = phi_dot * sin(theta_g)
%
% Inversion (avoiding singularity at theta_g = ±90°):
p_g = omega_g_cmd(1);
q_g = omega_g_cmd(2);
r_g = omega_g_cmd(3);

% Singularity check
if abs(cos(theta_g)) < 0.1
    % Near gimbal lock, use damped solution
    phi_g_dot = 0;
    theta_g_dot = q_g;
else
    % Standard inversion
    phi_g_dot = (p_g * cos(theta_g) - r_g * sin(theta_g)) / (cos(theta_g)^2 + sin(theta_g)^2);
    theta_g_dot = q_g;
end


u_gimbal = [phi_g_dot; theta_g_dot];

end