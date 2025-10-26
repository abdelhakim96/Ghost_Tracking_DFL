function state_dot = quadrotor_dynamics_realtime(t, state, xd, vd, ad, jd, sd, psid, fw_orientation, dfl_gains)
% This function defines the dynamics of the quadrotor using a DFL controller
% with a first-order gimbal model.

persistent last_phi_g_ref last_theta_g_ref;
if t == 0 % Reset persistent variables at the start of the simulation
    last_phi_g_ref = [];
    last_theta_g_ref = [];
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
v_yaw = 0 - c5*(vpsi - 0) - c4*(atan2(2*(q0*q3+q1*q2), 1-2*(q2^2+q3^2)) - psid);

% --- Gimbal virtual control ---
% The objective is to match the camera's world-frame orientation to the fixed-wing's world-frame orientation.
% R_cam_w = R_bw * R_gb
% We want R_cam_w = R_fw_w
% So, R_bw * R_gb = R_fw_w
% Which means the desired gimbal orientation is R_gb = R_bw' * R_fw_w

% Get fixed-wing rotation matrix from its quaternion
q_fw = fw_orientation;
q0_fw=q_fw(1); q1_fw=q_fw(2); q2_fw=q_fw(3); q3_fw=q_fw(4);
R_fw_w = [q0_fw^2+q1_fw^2-q2_fw^2-q3_fw^2, 2*(q1_fw*q2_fw-q0_fw*q3_fw), 2*(q1_fw*q3_fw+q0_fw*q2_fw);
          2*(q1_fw*q2_fw+q0_fw*q3_fw), q0_fw^2-q1_fw^2+q2_fw^2-q3_fw^2, 2*(q2_fw*q3_fw-q0_fw*q1_fw);
          2*(q1_fw*q3_fw-q0_fw*q2_fw), 2*(q2_fw*q3_fw+q0_fw*q1_fw), q0_fw^2-q1_fw^2-q2_fw^2+q3_fw^2];

% Calculate the desired gimbal rotation matrix
R_gb_ref = R_bw' * R_fw_w;

% Extract Euler angles from the desired gimbal rotation matrix
% R_gb = [ c_p*c_t, -s_p, c_p*s_t;
%          s_p*c_t,  c_p, s_p*s_t;
%             -s_t,    0,     c_t ];
% From this, we can derive:
% theta_g = asin(-R_gb(3,1))
% phi_g = atan2(R_gb(2,1)/cos(theta_g), R_gb(1,1)/cos(theta_g)) -> atan2(R_gb(2,1), R_gb(1,1))
% Note: This assumes a specific Euler angle sequence (e.g., ZYX), which matches the gimbal model.

phi_g_ref_kinematic = atan2(R_gb_ref(2,1), R_gb_ref(1,1));
theta_g_ref_kinematic = asin(-R_gb_ref(3,1));

% --- Generate the reference for the controller ---
% Ramp the reference from 0 to the kinematic reference over the first 0.5s
ramp_time = 0.5;
if t < ramp_time
    ramp_factor = t / ramp_time;
    phi_g_ref = ramp_factor * phi_g_ref_kinematic;
    theta_g_ref = ramp_factor * theta_g_ref_kinematic;
else
    phi_g_ref = phi_g_ref_kinematic;
    theta_g_ref = theta_g_ref_kinematic;
end

% Virtual control for gimbal (first-order system)
v_phi = -c_phi * (phi_g - phi_g_ref);
v_theta = -c_theta * (theta_g - theta_g_ref);

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

% State Derivatives
x_dot = v_w;
q_dot = 0.5 * [-q1, -q2, -q3; q0, -q3, q2; q3, q0, -q1; -q2, q1, q0] * omega_b;
v_dot = a_;
omega_dot = [ (u(2)/Ix) + (omega_b(2)*omega_b(3)*(Iy - Iz))/Ix;
              (u(3)/Iy) - (omega_b(1)*omega_b(3)*(Ix - Iz))/Iy;
              (u(4)/Iz) + (omega_b(1)*omega_b(2)*(Ix - Iy))/Iz ];
zeta_dot = xi;
xi_dot = u(1);

% Gimbal dynamics (first-order)
phi_g_dot = u(5);
theta_g_dot = u(6);

% Assemble state derivative vector (17 states)
state_dot = zeros(17,1);
state_dot(1:3) = x_dot;
state_dot(4:7) = q_dot;
state_dot(8:10) = v_dot;
state_dot(11:13) = omega_dot;
state_dot(14) = phi_g_dot;
state_dot(15) = theta_g_dot;
state_dot(16) = zeta_dot;
state_dot(17) = xi_dot;

end
