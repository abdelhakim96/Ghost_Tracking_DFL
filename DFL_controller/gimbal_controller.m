function u_gimbal = gimbal_controller(state, fw_state, dfl_gains)
% This function computes the control input for the gimbal using a standalone,
% singularity-free kinematic controller.

% Unpack the state vector (17 states for 1st order gimbal)
q_bw = state(4:7);      % Quaternion from Body to World [q0, q1, q2, q3]
omega_b = state(11:13); % Angular velocity in Body Frame [p, q, r]
phi_g = state(14);      % Gimbal roll
theta_g = state(15);    % Gimbal pitch

% Gains for the virtual controller of the first-order gimbal
c_phi = dfl_gains.c_phi;      % Proportional gain for gimbal roll
c_theta = dfl_gains.c_theta;    % Proportional gain for gimbal pitch
c_ff_phi = dfl_gains.c_ff_phi;    % Feedforward gain for gimbal roll
c_ff_theta = dfl_gains.c_ff_theta;% Feedforward gain for gimbal pitch

% Normalize the quaternion (scalar-first format [w, x, y, z])
q_bw = q_bw / (norm(q_bw) + 1e-9);

% Rotation matrix from Body to World using MATLAB's built-in function
R_bw = quat2rotm(q_bw');

% Extract fixed-wing orientation
fw_orientation = fw_state(7:10);
q_fw = fw_orientation / (norm(fw_orientation) + 1e-9);

% Extract fixed-wing angular velocity (body frame)
fw_omega_b = fw_state(11:13);  % [p_fw, q_fw, r_fw]

% Build rotation matrix for fixed-wing
R_fw_w = quat2rotm(q_fw');

% --- Robust Error Calculation using Rotation Matrices ---
% The desired gimbal rotation relative to the drone body is R_gb_desired = R_bw' * R_fw_w.
R_gb_desired = R_bw' * R_fw_w;

% --- Singularity-Free Kinematic Control Law ---
% Construct the current gimbal rotation matrix based on a pitch-roll sequence
R_gb = eul2rotm([0, theta_g, phi_g], 'ZYX'); % R_gb = Ry(theta_g) * Rx(phi_g)

% Calculate the error rotation matrix
R_error = R_gb' * R_gb_desired;

% Extract the axis-angle error vector (vector part of the error quaternion)
error_vec = 0.5 * [R_error(3,2) - R_error(2,3); 
                   R_error(1,3) - R_error(3,1); 
                   R_error(2,1) - R_error(1,2)];

% Transform the error vector from the gimbal frame to the body frame
error_vec_b = R_gb * error_vec;

% Project the error onto the gimbal's controllable axes (kinematic inversion)
p_corr = error_vec_b(1);
q_corr = error_vec_b(2);
r_corr = error_vec_b(3);
phi_g_error_term = p_corr * cos(theta_g) - r_corr * sin(theta_g);
theta_g_error_term = q_corr;

% --- Calculate Feedforward Rates from Relative Angular Velocity ---
% Transform fixed-wing angular velocity to drone body frame
omega_fw_in_b = R_bw' * (R_fw_w * fw_omega_b);

% Calculate relative angular velocity in the drone's body frame
omega_rel_b = omega_fw_in_b - omega_b;

% Project the relative angular velocity onto the gimbal's controllable axes
% This is the kinematic inversion for a pitch-then-roll gimbal
p_rel_b = omega_rel_b(1);
q_rel_b = omega_rel_b(2);
r_rel_b = omega_rel_b(3);
phi_g_ref_dot = p_rel_b * cos(theta_g) - r_rel_b * sin(theta_g);
theta_g_ref_dot = q_rel_b;

% Final virtual control law combining feedback and feedforward terms
v_phi = -c_phi * phi_g_error_term + c_ff_phi * phi_g_ref_dot;
v_theta = -c_theta * theta_g_error_term + c_ff_theta * theta_g_ref_dot;

u_gimbal = [v_phi; v_theta];

end
