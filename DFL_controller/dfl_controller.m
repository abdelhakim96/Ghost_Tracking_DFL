function u = dfl_controller(t, state, xd, vd, ad, jd, sd, psid, fw_state, dfl_gains)
% This function computes the control input for the quadrotor using a DFL
% controller with a first-order gimbal model.

persistent last_phi_g_ref last_theta_g_ref last_t;

if t == 0 % Reset persistent variables at the start of the simulation
    last_phi_g_ref = [];
    last_theta_g_ref = [];
    last_t = 0;
end

% Define global variables
global m Ix Iy Iz g

% Unpack the state vector (17 states for 1st order gimbal)
x_w = state(1:3);       % Position in World Frame [N, E, D]
q_bw = state(4:7);      % Quaternion from Body to World [q0, q1, q2, q3]
v_w = state(8:10);      % Velocity in World Frame
omega_b = state(11:13); % Angular velocity in Body Frame [p, q, r]
phi_g = state(14);      % Gimbal roll
theta_g = state(15);    % Gimbal pitch
zeta = state(16);       % Total thrust
xi = state(17);         % Derivative of total thrust

% Controller gains for the main body
c0 = dfl_gains.c0;  % Position gain
c1 = dfl_gains.c1;  % Velocity gain
c2 = dfl_gains.c2;   % Acceleration gain
c3 = dfl_gains.c3;    % Jerk gain
c4 = dfl_gains.c4;   % Yaw gain
c5 = dfl_gains.c5;    % Yaw rate gain

% Gains for the virtual controller of the first-order gimbal
c_phi = dfl_gains.c_phi;      % Proportional gain for gimbal roll
c_theta = dfl_gains.c_theta;    % Proportional gain for gimbal pitch
c_ff_phi = dfl_gains.c_ff_phi;    % Feedforward gain for gimbal roll
c_ff_theta = dfl_gains.c_ff_theta;% Feedforward gain for gimbal pitch

% Normalize the quaternion (scalar-first format [w, x, y, z])
q_bw = q_bw / (norm(q_bw) + 1e-9);

% Rotation matrix from Body to World using MATLAB's built-in function
% Note: quat2rotm expects scalar-first format.
R_bw = quat2rotm(q_bw');

% Dynamics
F_thrust = R_bw * [0; 0; zeta];
a_ = (F_thrust/m) - [0; 0; g];
j = (zeta*(R_bw(:,1)*omega_b(2) - R_bw(:,2)*omega_b(1)) + R_bw(:,3)*xi)/m;

% --- Consistent Yaw Rate Calculation ---
% To get the world-frame yaw rate (vpsi), we need to transform the body-frame
% angular velocity omega_b into world-frame angular rates using the full
% kinematic transformation.
eul_drone = quat2eul(q_bw', 'ZYX');
phi = eul_drone(3);   % Roll
theta = eul_drone(2); % Pitch

% Full kinematic transformation from body rates to Euler rates
% This avoids singularities when pitch is not zero.
T_inv = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
         0, cos(phi),            -sin(phi);
         0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

% Add a safeguard for the singularity at pitch = +/- 90 degrees
if abs(cos(theta)) < 1e-6
    eul_drone_rates = [0; 0; 0]; % Avoid division by zero
else
    eul_drone_rates = T_inv * omega_b;
end
vpsi = eul_drone_rates(1); % Yaw rate in world frame

% Virtual control input
v_pos = sd - c3*(j - jd) - c2*(a_ - ad) - c1*(v_w - vd) - c0*(x_w - xd);

% Gimbal virtual control
% --- UPDATED: Drone follows FW yaw dynamics, Gimbal follows FW roll/pitch ---

% Extract fixed-wing orientation
fw_orientation = fw_state(7:10);
q_fw = fw_orientation / (norm(fw_orientation) + 1e-9);

% Convert fixed-wing quaternion to Euler angles using MATLAB's built-in function
% Using 'ZYX' convention for [yaw, pitch, roll] and scalar-first quaternion
eul_fw = quat2eul(q_fw', 'ZYX');
fw_yaw = eul_fw(1);
fw_pitch = eul_fw(2);
fw_roll = eul_fw(3);

% Extract fixed-wing angular velocity (body frame)
fw_omega_b = fw_state(11:13);  % [p_fw, q_fw, r_fw]

% Convert FW body rates to world frame yaw rate consistently
phi_fw = fw_roll;
theta_fw = fw_pitch;

% Full kinematic transformation from body rates to Euler rates
T_inv_fw = [1, sin(phi_fw)*tan(theta_fw), cos(phi_fw)*tan(theta_fw);
            0, cos(phi_fw),              -sin(phi_fw);
            0, sin(phi_fw)/cos(theta_fw), cos(phi_fw)/cos(theta_fw)];

% Add a safeguard for the singularity at pitch = +/- 90 degrees
if abs(cos(theta_fw)) < 1e-6
    eul_fw_rates = [0; 0; 0]; % Avoid division by zero
else
    eul_fw_rates = T_inv_fw * fw_omega_b;
end
fw_yaw_rate = eul_fw_rates(1);

% Update the desired yaw trajectory for the quadrotor
psid = fw_yaw;           % Desired yaw angle
psid_dot = fw_yaw_rate;  % Desired yaw rate

% Virtual control input with feedforward yaw rate
v_pos = sd - c3*(j - jd) - c2*(a_ - ad) - c1*(v_w - vd) - c0*(x_w - xd);

% Enhanced yaw control with feedforward
drone_yaw = eul_drone(1);
yaw_error = wrapToPi(drone_yaw - psid);
v_yaw = psid_dot - c5*(vpsi - psid_dot) - c4*yaw_error;

% For the gimbal: compute relative orientation accounting for yaw tracking
% Build rotation matrix for fixed-wing using MATLAB's built-in function
R_fw_w = quat2rotm(q_fw');

% --- Robust Error Calculation using Rotation Matrices ---
% The desired gimbal orientation should align the drone's body frame with the fixed-wing's frame.
% R_gw_desired = R_fw_w. Since R_gw = R_bw * R_gb, we have R_bw * R_gb_desired = R_fw_w.
% So, the desired gimbal rotation relative to the drone body is R_gb_desired = R_bw' * R_fw_w.
R_gb_desired = R_bw' * R_fw_w;

% Extract desired gimbal angles from the rotation matrix using the 'ZYX' convention.
eul_gb_desired = rotm2eul(R_gb_desired, 'ZYX');
% The output is [yaw, pitch, roll].
theta_g_ref_raw = eul_gb_desired(2); % Pitch
phi_g_ref_raw = eul_gb_desired(3);   % Roll

% --- Reference Angle Unwrapping and Singularity Avoidance ---
if isempty(last_phi_g_ref)
    last_phi_g_ref = phi_g_ref_raw;
    last_theta_g_ref = theta_g_ref_raw;
end

% Use the utility function to unwrap the gimbal roll reference angle
phi_g_ref = unwrapAngle(phi_g_ref_raw, last_phi_g_ref);
last_phi_g_ref = phi_g_ref; % Update for next iteration's unwrap

% Use the utility function to unwrap the gimbal pitch reference angle
theta_g_ref = unwrapAngle(theta_g_ref_raw, last_theta_g_ref);
last_theta_g_ref = theta_g_ref; % Update for next iteration's unwrap

% --- Calculate desired gimbal rates from relative angular velocity ---
% Transform fixed-wing angular velocity to drone body frame
omega_fw_in_b = R_gb_desired * fw_omega_b;

% Calculate relative angular velocity
omega_rel_b = omega_fw_in_b - omega_b;

% Use the first two components as desired gimbal rates (feedforward term)
phi_g_ref_dot = omega_rel_b(1);   % Desired roll rate
theta_g_ref_dot = omega_rel_b(2); % Desired pitch rate


% Virtual control for gimbal with feedforward
phi_g_error = wrapToPi(phi_g - phi_g_ref);
theta_g_error = wrapToPi(theta_g - theta_g_ref);
v_phi = -c_phi * phi_g_error + c_ff_phi * phi_g_ref_dot;
v_theta = -c_theta * theta_g_error + c_ff_theta * theta_g_ref_dot;


%v_phi = 0.0;
%v_theta = 0.0;

% Combined virtual control vector
v = [v_pos; v_yaw; v_phi; v_theta];

% Gimbal parameters (aerodynamics, inertia not used in 1st order model)
Ag_p = 0.01;
Ag_q = 0.01;
Ig_x = 0.001; % Kept for function signature compatibility
Ig_y = 0.001; % Kept for function signature compatibility

% Feedback linearization using the newly generated functions
alpha_val = alpha_gimbal_func(state, 0, 0, 0, Ag_p, Ag_q, Ix, Iy, Iz, Ig_x, Ig_y, zeta, xi, m);
beta_val = beta_gimbal_func(state, 0, 0, 0, Ag_p, Ag_q, Ix, Iy, Iz, Ig_x, Ig_y, zeta, xi, m);
u = alpha_val + beta_val * v;

end
