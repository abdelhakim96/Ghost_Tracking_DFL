function u = dfl_controller(t, state, xd, vd, ad, jd, sd, psid,fw_state, fw_orientation, dfl_gains)
% This function computes the control input for the quadrotor using a DFL controller
% with a first-order gimbal model.

state = real(state);

% persistent last_phi_g_ref last_theta_g_ref last_gamma_g_ref;
% if t == 0 
%     last_phi_g_ref = [];
%     last_theta_g_ref = [];
%     last_gamma_g_ref = [];
% end

% Define global variables
global m Ix Iy Iz g

% Unpack the state vector (18 states for 3-axis gimbal)
x_w = state(1:3);       % Position in World Frame [N, E, D]
q_bw = state(4:7);      % Quaternion from Body to World [q0, q1, q2, q3]
v_w = state(8:10);      % Velocity in World Frame
omega_b = state(11:13); % Angular velocity in Body Frame [p, q, r]
phi_g = state(14);      % Gimbal yaw
theta_g = state(15);    % Gimbal pitch
gamma_g = state(16);    % Gimbal roll
zeta = state(17);       % Total thrust
xi = state(18);         % Derivative of total thrust

% Controller gains for the main body
c0 = dfl_gains.c0;  % Position gain
c1 = dfl_gains.c1;  % Velocity gain
c2 = dfl_gains.c2;   % Acceleration gain
c3 = dfl_gains.c3;    % Jerk gain
c4 = dfl_gains.c4;   % Yaw gain
c5 = dfl_gains.c5;    % Yaw rate gain

% Gains for the virtual controller of the first-order gimbal
c_phi = dfl_gains.c_phi;      % Proportional gain for gimbal yaw
c_theta = dfl_gains.c_theta;    % Proportional gain for gimbal pitch
c_gamma = dfl_gains.c_gamma;    % Proportional gain for gimbal roll

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
% Drone yaw now tracks the fixed-wing's yaw
q_fw_yaw = fw_orientation / (norm(fw_orientation) + 1e-9);
psi_fw = atan2(2*(q_fw_yaw(1)*q_fw_yaw(4)+q_fw_yaw(2)*q_fw_yaw(3)), 1-2*(q_fw_yaw(3)^2+q_fw_yaw(4)^2));
v_yaw = 0 - c5*(vpsi - 0) - c4*(atan2(2*(q0*q3+q1*q2), 1-2*(q2^2+q3^2)) - psi_fw);

% Gimbal virtual control
% --- NEW: Full Orientation Tracking ---
% The reference is the fixed-wing's full 3D orientation.
% We want the gimbal's world orientation to match the fixed-wing's world orientation.
% R_gimbal_w = R_bw * R_gb  should equal R_fw_w
% So, the desired gimbal orientation in the body frame is R_gb_ref = R_bw' * R_fw_w

% Convert fixed-wing quaternion to rotation matrix
q_fw = fw_orientation / (norm(fw_orientation) + 1e-9);
q0_fw=q_fw(1); q1_fw=q_fw(2); q2_fw=q_fw(3); q3_fw=q_fw(4);
R_fw_w = [q0_fw^2+q1_fw^2-q2_fw^2-q3_fw^2, 2*(q1_fw*q2_fw-q0_fw*q3_fw), 2*(q1_fw*q3_fw+q0_fw*q2_fw);
          2*(q1_fw*q2_fw+q0_fw*q3_fw), q0_fw^2-q1_fw^2+q2^2-q3^2, 2*(q2_fw*q3_fw-q0_fw*q1_fw);
          2*(q1_fw*q3_fw-q0_fw*q2_fw), 2*(q2_fw*q3_fw+q0_fw*q1_fw), q0_fw^2-q1^2-q2^2+q3^2];

% Calculate the desired gimbal orientation relative to the quadrotor body
R_gb_ref = R_bw' * R_fw_w;

% --- Reverted to Euler Angle Control Law ---
% Extract target Euler angles from the desired rotation matrix
% Using a ZYX rotation sequence for the gimbal (yaw, pitch, roll)
angles_ref = rotm2eul(R_gb_ref, 'ZYX');
phi_g_ref = angles_ref(1);   % Yaw
theta_g_ref = angles_ref(2); % Pitch
gamma_g_ref = angles_ref(3); % Roll

% Virtual control for gimbal (simple proportional control on angle error)
v_phi = -c_phi * (phi_g - phi_g_ref);
v_theta = -c_theta * (theta_g - theta_g_ref);
v_gamma = -c_gamma * (gamma_g - gamma_g_ref);

% Combined virtual control vector
v = [v_pos; v_yaw; v_phi; v_theta; v_gamma];

% Gimbal parameters (aerodynamics, inertia not used in 1st order model)
Ag_p = 0.01;
Ag_q = 0.01;
Ig_x = 0.001; % Kept for function signature compatibility
Ig_y = 0.001; % Kept for function signature compatibility

% Feedback linearization using the newly generated functions
% Note: Ag_r, Ig_z are placeholders as they are not used in the 1st order model
alpha_val = alpha_gimbal_func(state, 0, 0, 0, 0, 0, 0, Ix, Iy, Iz, 0, 0, 0, zeta, xi, m);
beta_val = beta_gimbal_func(state, 0, 0, 0, 0, 0, 0, Ix, Iy, Iz, 0, 0, 0, zeta, xi, m);
u = alpha_val + beta_val * v;

end
