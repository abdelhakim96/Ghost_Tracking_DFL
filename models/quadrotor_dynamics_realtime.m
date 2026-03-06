function state_dot = quadrotor_dynamics_realtime(t, state, xd, vd, ad, jd, sd, psid, fw_state, dfl_gains)
% QUADROTOR_DYNAMICS_REALTIME  Multicopter + 2-axis gimbal dynamics with unified DFL control.
%
% The DFL controller produces all 6 controls:
%   [ddot_T; tau_phi; tau_theta; tau_psi; phi_g_dot; theta_g_dot]
%
% State vector (17 elements):
%   [pos(3); quat(4); vel_world(3); omega_body(3); phi_g; theta_g; zeta; xi]

persistent history;

% Special case: retrieve logged history for post-processing
if ischar(t) && strcmp(t, 'get_history')
    state_dot = history;
    return;
end

if t == 0 % Reset persistent variables at the start of the simulation
    history = [];
end

% Define global variables (shared with DFL controller)
global m Ix Iy Iz g

% -----------------------------------------------------------------------
% Unpack state vector (17 states)
% -----------------------------------------------------------------------
x_w = state(1:3);       % Position in World Frame [N, E, D]
q_bw = state(4:7);      % Quaternion from Body to World [q0, q1, q2, q3]
v_w = state(8:10);      % Velocity in World Frame
omega_b = state(11:13); % Angular velocity in Body Frame [p, q, r]
phi_g = state(14);      % Gimbal roll angle (rotation about body x-axis)
theta_g = state(15);    % Gimbal pitch angle (rotation about body y-axis)
zeta = state(16);       % Total thrust (DFL extended state)
xi = state(17);         % Thrust rate (DFL extended state)

% -----------------------------------------------------------------------
% Normalize quaternion and compute rotation matrix
% -----------------------------------------------------------------------
q_bw = q_bw / (norm(q_bw) + 1e-9);
q0=q_bw(1); q1=q_bw(2); q2=q_bw(3); q3=q_bw(4);

R_bw = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
        2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
        2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];

% -----------------------------------------------------------------------
% Euler angle logging (for debugging/history only, not used in control)
% -----------------------------------------------------------------------
eul = quat2eul([q0, q1, q2, q3], 'ZYX');
yaw = eul(1); pitch = eul(2); roll = eul(3);

if isempty(history)
    previous_roll = []; previous_pitch = []; previous_yaw = [];
else
    previous_roll = history(end, 12);
    previous_pitch = history(end, 13);
    previous_yaw = history(end, 14);
end
corrected_roll = correctAngleJump(roll, previous_roll);
corrected_pitch = correctAngleJump(pitch, previous_pitch);
corrected_yaw = correctAngleJump(yaw, previous_yaw);

% -----------------------------------------------------------------------
% Gimbal orientation in world frame (for debugging)
% -----------------------------------------------------------------------
R_gb = [cos(theta_g),                  0,             sin(theta_g);
        sin(phi_g)*sin(theta_g),  cos(phi_g),  -sin(phi_g)*cos(theta_g);
       -cos(phi_g)*sin(theta_g),  sin(phi_g),   cos(phi_g)*cos(theta_g)];
R_gimbal_w = R_bw * R_gb;
gimbal_global_roll = atan2(R_gimbal_w(3,2), R_gimbal_w(3,3));

% -----------------------------------------------------------------------
% Call unified DFL controller (produces all 6 controls)
% -----------------------------------------------------------------------
u_all = dfl_controller(t, state, xd, vd, ad, jd, sd, psid, fw_state, dfl_gains);

% Split into drone controls and gimbal controls
u_drone = u_all(1:4);    % [ddot_T; tau_phi; tau_theta; tau_psi]
u_gimbal = u_all(5:6);   % [phi_g_dot; theta_g_dot]

% -----------------------------------------------------------------------
% Dynamics: translational (world frame)
% -----------------------------------------------------------------------
F_thrust = R_bw * [0; 0; zeta];
a_ = (F_thrust/m) - [0; 0; g];

% -----------------------------------------------------------------------
% State derivatives
% -----------------------------------------------------------------------
x_dot = v_w;                    % Position kinematics
q_dot = 0.5 * [-q1, -q2, -q3;  % Quaternion kinematics: q_dot = 0.5*G(q)*omega
               q0, -q3, q2;
               q3, q0, -q1;
               -q2, q1, q0] * omega_b;
v_dot = a_;                     % Translational dynamics

% Rotational dynamics: omega_dot = J^{-1}*(-omega x J*omega + tau)
omega_dot = [ (u_drone(2)/Ix) + (omega_b(2)*omega_b(3)*(Iy - Iz))/Ix;
              (u_drone(3)/Iy) - (omega_b(1)*omega_b(3)*(Ix - Iz))/Iy;
              (u_drone(4)/Iz) + (omega_b(1)*omega_b(2)*(Ix - Iy))/Iz ];

% DFL extended states: double integrator on thrust
zeta_dot = xi;          % zeta_dot = xi (thrust rate)
xi_dot = u_drone(1);    % xi_dot = ddot_T (thrust acceleration, DFL input)

% Gimbal joint rates (from unified DFL controller)
phi_g_dot = u_gimbal(1);
theta_g_dot = u_gimbal(2);

% -----------------------------------------------------------------------
% Assemble state derivative vector (17 states)
% -----------------------------------------------------------------------
state_dot = zeros(17,1);
state_dot(1:3) = x_dot;
state_dot(4:7) = q_dot;
state_dot(8:10) = v_dot;
state_dot(11:13) = omega_dot;
state_dot(14) = phi_g_dot;
state_dot(15) = theta_g_dot;
state_dot(16) = zeta_dot;
state_dot(17) = xi_dot;

% -----------------------------------------------------------------------
% Log history for post-processing plots
% -----------------------------------------------------------------------
history(end+1, :) = [t, zeta, u_drone(2), u_drone(3), u_drone(4), ...
    omega_b', u_gimbal(1), u_gimbal(2), gimbal_global_roll, ...
    corrected_roll, corrected_pitch, corrected_yaw];

end
