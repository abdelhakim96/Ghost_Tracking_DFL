function u_geom = geometric_controller(state, fw_state, geom_gains)
% This function computes the control input for the gimbal roll and pitch
% rates to track the fixed-wing's orientation.

% Unpack drone state
q_bw = state(4:7);      % Quaternion from Body to World [q0, q1, q2, q3]
phi_g = state(14);      % Gimbal roll
theta_g = state(15);    % Gimbal pitch

% Unpack fixed-wing state
q_fw = fw_state(7:10); % Fixed-wing orientation quaternion

% Controller gains
k_g = geom_gains.k_g; % Proportional gain for gimbal angular velocity

% Normalize quaternions
q_bw = q_bw / (norm(q_bw) + 1e-9);
q_fw = q_fw / (norm(q_fw) + 1e-9);

% Current gimbal orientation in world frame
R_bw = quat2rotm(q_bw');
% Consistent Pitch-Roll (Y-X) gimbal model
Rx = [1, 0, 0; 0, cos(phi_g), -sin(phi_g); 0, sin(phi_g), cos(phi_g)];
Ry = [cos(theta_g), 0, sin(theta_g); 0, 1, 0; -sin(theta_g), 0, cos(theta_g)];
R_gb = Ry * Rx;
R_gw = R_bw * R_gb;
q_gw = rotm2quat(R_gw);

% Desired gimbal orientation is the fixed-wing's orientation
q_gw_d = q_fw;

% Ensure quaternions are row vectors for multiplication
if iscolumn(q_gw)
    q_gw = q_gw';
end
if iscolumn(q_gw_d)
    q_gw_d = q_gw_d';
end

% Quaternion error
q_e = quatmultiply(quatconj(q_gw), q_gw_d);

% Desired angular velocity for the gimbal in the gimbal frame
omega_g_d = 2 * k_g * [q_e(2); q_e(3); q_e(4)];
if q_e(1) < 0
    omega_g_d = -omega_g_d;
end

% The gimbal control inputs are the roll and pitch rates
% This accounts for the gimbal kinematics
v_phi = omega_g_d(1);
% Corresponding kinematic inversion for Pitch-Roll (Y-X) gimbal
v_theta = omega_g_d(2) * cos(phi_g) + omega_g_d(3) * sin(phi_g);

u_geom = [v_phi; v_theta];

end
