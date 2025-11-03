function state_dot = quadrotor_dynamics_realtime(t, state, xd, vd, ad, jd, sd, psid,fw_state, fw_orientation, dfl_gains)
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
          2*(q1_fw*q2_fw+q0_fw*q3_fw), q0_fw^2-q1_fw^2+q2_fw^2-q3_fw^2, 2*(q2_fw*q3_fw-q0_fw*q1_fw);
          2*(q1_fw*q3_fw-q0_fw*q2_fw), 2*(q2_fw*q3_fw+q0_fw*q1_fw), q0_fw^2-q1_fw^2-q2_fw^2+q3_fw^2];

% Calculate the desired gimbal orientation relative to the quadrotor body
R_gb_ref = R_bw' * R_fw_w;

% Extract gimbal angles from the desired rotation matrix.
% This assumes a Z-Y rotation sequence for the gimbal (phi_g is yaw, theta_g is pitch)
% which matches the kinematics used to generate the pointing vector previously.
theta_g_ref_raw = asin(-R_gb_ref(3,1));
phi_g_ref_raw = atan2(R_gb_ref(2,1), R_gb_ref(1,1));

% --- Reference Angle Unwrapping and Singularity Avoidance ---
% Initialize persistent variables on first run
if isempty(last_phi_g_ref)
    last_phi_g_ref = phi_g_ref_raw;
    last_theta_g_ref = theta_g_ref_raw;
end

% Robust angle unwrapping with jump rejection for phi_g (yaw-like angle)
delta_phi = phi_g_ref_raw - last_phi_g_ref;
% Wrap to [-pi, pi]
delta_phi = mod(delta_phi + pi, 2*pi) - pi;
% Reject large jumps (e.g., near singularity)
if abs(delta_phi) > (170 * pi / 180)
    phi_g_ref = last_phi_g_ref;
else
    phi_g_ref = last_phi_g_ref + delta_phi;
end
last_phi_g_ref = phi_g_ref;

% Robust angle unwrapping with jump rejection for theta_g (pitch-like angle)
delta_theta = theta_g_ref_raw - last_theta_g_ref;
% Wrap to [-pi, pi]
delta_theta = mod(delta_theta + pi, 2*pi) - pi;
% Reject large jumps
if abs(delta_theta) > (170 * pi / 180)
    theta_g_ref = last_theta_g_ref;
else
    theta_g_ref = last_theta_g_ref + delta_theta;
end
last_theta_g_ref = theta_g_ref;

% Virtual control for gimbal (first-order system)
%

R_fw_w = quat2rotm(q_fw');
R_gb_desired = R_bw' * R_fw_w;

fw_omega_b = fw_state(11:13);  % [p_fw, q_fw, r_fw]
omega_fw_in_b = R_gb_desired * fw_omega_b;
omega_rel_b = omega_fw_in_b - omega_b;
phi_g_ref_dot = omega_rel_b(1);   % Desired roll rate
theta_g_ref_dot = omega_rel_b(2); % Desired pitch rate

%

v_phi = -c_phi * (phi_g - phi_g_ref) + 1.0 * phi_g_ref_dot;
v_theta = -c_theta * (theta_g - theta_g_ref) + 1.0 * theta_g_ref_dot;

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
