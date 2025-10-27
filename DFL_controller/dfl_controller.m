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
% angular velocity omega_b into world-frame angular rates.
eul_drone = quat2eul(q_bw', 'ZYX');
eul_drone_rates = [0, sin(eul_drone(3)), cos(eul_drone(3));
                   0, cos(eul_drone(3)), -sin(eul_drone(3));
                   1, 0, 0] * omega_b;
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
eul_fw_rates = [0, sin(fw_roll), cos(fw_roll);
                0, cos(fw_roll), -sin(fw_roll);
                1, 0, 0] * fw_omega_b;
fw_yaw_rate = eul_fw_rates(1);

% Update the desired yaw trajectory for the quadrotor
psid = fw_yaw;           % Desired yaw angle
psid_dot = fw_yaw_rate;  % Desired yaw rate

% Virtual control input with feedforward yaw rate
v_pos = sd - c3*(j - jd) - c2*(a_ - ad) - c1*(v_w - vd) - c0*(x_w - xd);

% Enhanced yaw control with feedforward
drone_yaw = eul_drone(1);
v_yaw = psid_dot - c5*(vpsi - psid_dot) - c4*(drone_yaw - psid);

% For the gimbal: compute relative orientation accounting for yaw tracking
% Build rotation matrix for fixed-wing using MATLAB's built-in function
R_fw_w = quat2rotm(q_fw');

% --- Robust Error Calculation using Rotation Matrices ---
% The desired gimbal orientation should align the drone's body frame with the fixed-wing's frame.
% R_gw_desired = R_fw_w. Since R_gw = R_bw * R_gb, we have R_bw * R_gb_desired = R_fw_w.
% So, the desired gimbal rotation relative to the drone body is R_gb_desired = R_bw' * R_fw_w.
R_gb_desired = R_bw' * R_fw_w;

% Extract desired gimbal angles (phi_g_ref, theta_g_ref) from R_gb_desired.
% The gimbal kinematics are a ZY rotation sequence (Ry(theta) then Rz(phi)).
% R = [c(phi)c(th), -s(phi), c(phi)s(th); 
%      s(phi)c(th),  c(phi), s(phi)s(th); 
%          -s(th),       0,       c(th)]
% From this, theta = asin(-R(3,1)) and phi = atan2(R(2,1), R(1,1)).
phi_g_ref_raw = atan2(R_gb_desired(2,1), R_gb_desired(1,1));
theta_g_ref_raw = asin(-R_gb_desired(3,1));

% --- Reference Angle Unwrapping and Singularity Avoidance ---
if isempty(last_phi_g_ref)
    last_phi_g_ref = phi_g_ref_raw;
    last_theta_g_ref = theta_g_ref_raw;
end

% Robust angle unwrapping with jump rejection for phi_g (roll)
delta_phi = phi_g_ref_raw - last_phi_g_ref;
delta_phi = mod(delta_phi + pi, 2*pi) - pi;
if abs(delta_phi) > (170 * pi / 180)
    phi_g_ref = last_phi_g_ref;
else
    phi_g_ref = last_phi_g_ref + delta_phi;
end

dt = t - last_t;
if dt > 1e-6
    phi_g_ref_dot = (phi_g_ref - last_phi_g_ref) / dt;
else
    phi_g_ref_dot = 0;
end
last_phi_g_ref = phi_g_ref;

% Robust angle unwrapping with jump rejection for theta_g (pitch)
delta_theta = theta_g_ref_raw - last_theta_g_ref;
delta_theta = mod(delta_theta + pi, 2*pi) - pi;
if abs(delta_theta) > (170 * pi / 180)
    theta_g_ref = last_theta_g_ref;
else
    theta_g_ref = last_theta_g_ref + delta_theta;
end

if dt > 1e-6
    theta_g_ref_dot = (theta_g_ref - last_theta_g_ref) / dt;
else
    theta_g_ref_dot = 0;
end
last_theta_g_ref = theta_g_ref;
last_t = t;

% Virtual control for gimbal with feedforward
v_phi = -c_phi * (phi_g - phi_g_ref) + c_ff_phi * phi_g_ref_dot;
v_theta = -c_theta * (theta_g - theta_g_ref) + c_ff_theta * theta_g_ref_dot;


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
