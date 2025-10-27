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

% Normalize the quaternion
q_bw = q_bw / (norm(q_bw) + 1e-9);
q0=q_bw(1); q1=q_bw(2); q2=q_bw(3); q3=q_bw(4);

% Rotation matrix from Body to World
R_bw = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
        2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
        2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];

% Dynamics
F_thrust = R_bw * [0; 0; zeta];
a_ = (F_thrust/m) - [0; 0; g];
j = (zeta*(R_bw(:,1)*omega_b(2) - R_bw(:,2)*omega_b(1)) + R_bw(:,3)*xi)/m;
vpsi = omega_b(1)*(2*q1*q3 - 2*q0*q2) + omega_b(2)*(2*q2*q3 + 2*q0*q1) + omega_b(3)*(q0^2 - q1^2 - q2^2 + q3^2);

% Virtual control input
v_pos = sd - c3*(j - jd) - c2*(a_ - ad) - c1*(v_w - vd) - c0*(x_w - xd);

% Gimbal virtual control
% --- UPDATED: Drone follows FW yaw dynamics, Gimbal follows FW roll/pitch ---

% Extract fixed-wing Euler angles (roll, pitch, yaw) from quaternion
fw_orientation = fw_state(7:10);
q_fw = fw_orientation / (norm(fw_orientation) + 1e-9);
q0_fw=q_fw(1); q1_fw=q_fw(2); q2_fw=q_fw(3); q3_fw=q_fw(4);

% Fixed-wing Euler angles (ZYX convention)
fw_roll = atan2(2*(q0_fw*q1_fw + q2_fw*q3_fw), 1 - 2*(q1_fw^2 + q2_fw^2));
fw_pitch = asin(2*(q0_fw*q2_fw - q3_fw*q1_fw));
fw_yaw = atan2(2*(q0_fw*q3_fw + q1_fw*q2_fw), 1 - 2*(q2_fw^2 + q3_fw^2));

% Extract fixed-wing angular velocity (body frame)
fw_omega_b = fw_state(11:13);  % [p_fw, q_fw, r_fw]

% Convert FW body rates to world frame yaw rate
% For ZYX Euler angles: yaw_dot = (r*cos(phi) - p*sin(phi))/cos(theta)
% But we can use quaternion kinematics directly for yaw rate in world frame:
fw_yaw_rate = fw_omega_b(1)*(2*q1_fw*q3_fw - 2*q0_fw*q2_fw) + ...
              fw_omega_b(2)*(2*q2_fw*q3_fw + 2*q0_fw*q1_fw) + ...
              fw_omega_b(3)*(q0_fw^2 - q1_fw^2 - q2_fw^2 + q3_fw^2);

% Update the desired yaw trajectory for the quadrotor
psid = fw_yaw;           % Desired yaw angle
psid_dot = fw_yaw_rate;  % Desired yaw rate

% Virtual control input with feedforward yaw rate
v_pos = sd - c3*(j - jd) - c2*(a_ - ad) - c1*(v_w - vd) - c0*(x_w - xd);

% Enhanced yaw control with feedforward
% v_yaw now includes feedforward of desired yaw rate
v_yaw = psid_dot - c5*(vpsi - psid_dot) - c4*(atan2(2*(q0*q3+q1*q2), 1-2*(q2^2+q3^2)) - psid);

% For the gimbal: compute relative orientation accounting for yaw tracking
% Build rotation matrix for fixed-wing
R_fw_w = [q0_fw^2+q1_fw^2-q2_fw^2-q3_fw^2, 2*(q1_fw*q2_fw-q0_fw*q3_fw), 2*(q1_fw*q3_fw+q0_fw*q2_fw);
          2*(q1_fw*q2_fw+q0_fw*q3_fw), q0_fw^2-q1_fw^2+q2_fw^2-q3_fw^2, 2*(q2_fw*q3_fw-q0_fw*q1_fw);
          2*(q1_fw*q3_fw-q0_fw*q2_fw), 2*(q2_fw*q3_fw+q0_fw*q1_fw), q0_fw^2-q1_fw^2-q2_fw^2+q3_fw^2];

% Calculate the desired gimbal orientation relative to the quadrotor body
R_gb_ref = R_bw' * R_fw_w;

% Extract gimbal roll and pitch from the desired rotation matrix
phi_g_ref_raw = atan2(R_gb_ref(3,2), R_gb_ref(3,3));
theta_g_ref_raw = asin(-R_gb_ref(3,1));

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
