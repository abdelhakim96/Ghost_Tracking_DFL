function u = drone_gimbal_controller(t, state, xd, vd, ad, jd, sd, psid, fw_state, dfl_gains)
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

% --- DFL Virtual Control ---
% Yaw control remains the same
eul_fw = quat2eul(fw_state(7:10)', 'ZYX');
fw_yaw = eul_fw(1);
fw_omega_b = fw_state(11:13);
phi_fw = eul_fw(3);
theta_fw = eul_fw(2);
T_inv_fw = [1, sin(phi_fw)*tan(theta_fw), cos(phi_fw)*tan(theta_fw);
            0, cos(phi_fw),              -sin(phi_fw);
            0, sin(phi_fw)/cos(theta_fw), cos(phi_fw)/cos(theta_fw)];
if abs(cos(theta_fw)) < 1e-6
    eul_fw_rates = [0; 0; 0];
else
    eul_fw_rates = T_inv_fw * fw_omega_b;
end
fw_yaw_rate = eul_fw_rates(1);
psid = fw_yaw;
psid_dot = fw_yaw_rate;
drone_yaw = eul_drone(1);
yaw_error = wrapToPi(drone_yaw - psid);
v_yaw = psid_dot - c5*(vpsi - psid_dot) - c4*yaw_error;

% Combined virtual control vector for DFL part
v = [v_pos; v_yaw];

% Gimbal parameters (aerodynamics, inertia not used in 1st order model)
Ag_p = 0.01;
Ag_q = 0.01;
Ig_x = 0.001; % Kept for function signature compatibility
Ig_y = 0.001; % Kept for function signature compatibility

% Feedback linearization for the drone dynamics
alpha_val = alpha_gimbal_func(state, 0, 0, 0, Ag_p, Ag_q, Ix, Iy, Iz, Ig_x, Ig_y, zeta, xi, m);
beta_val = beta_gimbal_func(state, 0, 0, 0, Ag_p, Ag_q, Ix, Iy, Iz, Ig_x, Ig_y, zeta, xi, m);

% The DFL part of the control
u_dfl = alpha_val + beta_val * v;

% The final control input vector
u = [u_dfl(1); u_dfl(2); u_dfl(3); u_dfl(4)];

end
