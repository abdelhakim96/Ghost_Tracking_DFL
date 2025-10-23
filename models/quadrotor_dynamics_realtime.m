function state_dot = quadrotor_dynamics_realtime(t, state, xd, vd, ad, jd, sd, psid, fw_orientation)
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
c0 = 29250.0;  % Position gain
c1 = 22400.0;  % Velocity gain
c2 = 350.0;   % Acceleration gain
c3 = 100.0;    % Jerk gain
c4 = 10.0;   % Yaw gain
c5 = 10.00;    % Yaw rate gain

% Gains for the virtual controller of the first-order gimbal
c_phi = 50000.0;      % Proportional gain for gimbal roll
c_theta = 70000.0;    % Proportional gain for gimbal pitch

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
% The reference is the fixed-wing's velocity vector in the world frame.
% However, the input `vd` to this function IS the fixed-wing's velocity vector.
% So, we can use it directly.
fw_forward_vec_w = vd / (norm(vd) + 1e-9); % Normalize to get a direction vector

% Transform the fixed-wing forward vector to the quadrotor's body frame
fw_forward_vec_b = R_bw' * fw_forward_vec_w;

% --- Reference Angle Calculation with Singularity Avoidance ---

% Calculate projection on body XY plane
xy_norm = sqrt(fw_forward_vec_b(1)^2 + fw_forward_vec_b(2)^2);

% Initialize persistent variables on first run
if isempty(last_phi_g_ref)
    if xy_norm < 1e-6
        last_phi_g_ref = 0; % Default to 0 if starting in singularity
    else
        last_phi_g_ref = atan2(fw_forward_vec_b(2), fw_forward_vec_b(1));
    end
    last_theta_g_ref = atan2(-fw_forward_vec_b(3), xy_norm + 1e-9);
end

% Check for singularity (gimbal lock)
if xy_norm < 1e-6
    % In singularity, phi is ill-defined. Hold the last known value.
    phi_g_ref = last_phi_g_ref;
else
    % Calculate raw angle
    phi_g_ref_raw = atan2(fw_forward_vec_b(2), fw_forward_vec_b(1));
    
    % Robust angle unwrapping with jump rejection
    delta_phi = phi_g_ref_raw - last_phi_g_ref;

    % Wrap to [-pi, pi]
    delta_phi = mod(delta_phi + pi, 2*pi) - pi;

    % Reject large jumps
    if abs(delta_phi) > (170 * pi / 180)
        phi_g_ref = last_phi_g_ref;
    else
        phi_g_ref = last_phi_g_ref + delta_phi;
    end
end

% Update persistent variable for next time step
last_phi_g_ref = phi_g_ref;

% Calculate theta reference (pitch) with robust unwrapping and jump rejection
theta_g_ref_raw = atan2(-fw_forward_vec_b(3), xy_norm + 1e-9);
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
