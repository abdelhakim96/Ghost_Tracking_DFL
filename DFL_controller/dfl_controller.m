function u = dfl_controller(t, state, xd, vd, ad, jd, sd, psid, fw_state, dfl_gains)
% This function computes the control input for the quadrotor using a DFL controller.

% Define global variables
global m Ix Iy Iz g

% Unpack the state vector (15 states for drone dynamics)
x_w = state(1:3);       % Position in World Frame [N, E, D]
q_bw = state(4:7);      % Quaternion from Body to World [q0, q1, q2, q3]
v_w = state(8:10);      % Velocity in World Frame
omega_b = state(11:13); % Angular velocity in Body Frame [p, q, r]
zeta = state(14);       % Total thrust
xi = state(15);         % Derivative of total thrust

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

% Extract fixed-wing orientation for yaw tracking
fw_orientation = fw_state(7:10);
q_fw = fw_orientation / (norm(fw_orientation) + 1e-9);
eul_fw = quat2eul(q_fw', 'ZYX');
fw_yaw = eul_fw(1);

% Extract fixed-wing angular velocity for yaw rate feedforward
fw_omega_b = fw_state(11:13);
R_fw_w = quat2rotm(q_fw');
omega_fw_w = R_fw_w * fw_omega_b;
fw_yaw_rate = omega_fw_w(3);

% Update the desired yaw trajectory for the quadrotor
psid_dot = fw_yaw_rate;

% Enhanced yaw control with feedforward
drone_yaw = eul_drone(1);
yaw_error = wrapToPi(drone_yaw - fw_yaw);
v_yaw = -c5 * (vpsi - psid_dot) - c4 * yaw_error;

% Combined virtual control vector
v = [v_pos; v_yaw];

% Feedback linearization using the newly generated functions
alpha_val = alpha_func(state, 0, 0, 0, Ix, Iy, Iz, zeta, xi, m);
beta_val = beta_func(state, 0, 0, 0, Ix, Iy, Iz, zeta, xi, m);
u = alpha_val + beta_val * v;

end
