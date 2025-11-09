function state_dot = quadrotor_dynamics_realtime(t, state, xd, vd, ad, jd, sd, psid, fw_state, gains, controller_type)
% This function defines the dynamics of the quadrotor.
% The controller is now in a separate file: DFL_controller/dfl_controller.m

persistent history;

% Special case to retrieve history
if ischar(t) && strcmp(t, 'get_history')
    state_dot = history;
    return;
end

if t == 0 % Reset persistent variables at the start of the simulation
    history = [];
end

% Define global variables
global m Ix Iy Iz g

% Unpack the state vector (15 states for direct thrust control)
x_w = state(1:3);       % Position in World Frame [N, E, D]
q_bw = state(4:7);      % Quaternion from Body to World [q0, q1, q2, q3]
v_w = state(8:10);      % Velocity in World Frame
omega_b = state(11:13); % Angular velocity in Body Frame [p, q, r]
phi_g = state(14);      % Gimbal roll
theta_g = state(15);    % Gimbal pitch

% Normalize the quaternion
q_bw = q_bw / (norm(q_bw) + 1e-9);
q0=q_bw(1); q1=q_bw(2); q2=q_bw(3); q3=q_bw(4);

% Rotation matrix from Body to World
R_bw = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
        2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
        2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];

% --- Angle Jump Correction ---
% Calculate Euler angles from quaternion
roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));
pitch = asin(2*(q0*q2 - q3*q1));
yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2));

% Get previous angles from history
if isempty(history)
    previous_roll = [];
    previous_pitch = [];
    previous_yaw = [];
else
    previous_roll = history(end, 12);
    previous_pitch = history(end, 13);
    previous_yaw = history(end, 14);
end

% Correct for angle jumps
corrected_roll = correctAngleJump(roll, previous_roll);
corrected_pitch = correctAngleJump(pitch, previous_pitch);
corrected_yaw = correctAngleJump(yaw, previous_yaw);
% --- End Angle Jump Correction ---

% Calculate gimbal's orientation in world frame for debugging
% Standard roll-pitch gimbal (Y-X rotation)
R_gb = [cos(theta_g), sin(theta_g)*sin(phi_g), sin(theta_g)*cos(phi_g);
        0, cos(phi_g), -sin(phi_g);
        -sin(theta_g), cos(theta_g)*sin(phi_g), cos(theta_g)*cos(phi_g)];
R_gimbal_w = R_bw * R_gb;
gimbal_global_roll = atan2(R_gimbal_w(3,2), R_gimbal_w(3,3));

% Call the controller to get the control input u
if strcmp(controller_type, 'DFL')
    u = dfl_controller(t, state, xd, vd, ad, jd, sd, psid, fw_state, gains);
elseif strcmp(controller_type, 'MPC')
    u = mpc_controller(t, state, xd, vd, ad, jd, sd, psid, fw_state, gains);
else
    error('Invalid controller type specified.');
end

% --- REALISTIC CONSTRAINTS ---
% Maximum and minimum thrust in Newtons
%max_thrust = 1500.0; % Corresponds to a thrust-to-weight ratio of ~3.3 for 0.468kg drone
%min_thrust = 0.0;

% Maximum roll, pitch, and yaw torques in N*m
%max_roll_torque = 10000.5;
%max_pitch_torque = 1000.5;
%max_yaw_torque = 1000.2;

% Saturate the control inputs
%u(1) = max(min(u(1), max_thrust), min_thrust); % Saturate total thrust
%u(2) = max(min(u(2), max_roll_torque), -max_roll_torque);   % Saturate roll torque
%u(3) = max(min(u(3), max_pitch_torque), -max_pitch_torque); % Saturate pitch torque
%u(4) = max(min(u(4), max_yaw_torque), -max_yaw_torque);     % Saturate yaw torque
% --- END REALISTIC CONSTRAINTS ---

% Dynamics
F_thrust = R_bw * [0; 0; u(1)]; % u(1) is now total thrust
a_ = (F_thrust/m) + [0; 0; g];

% State Derivatives
x_dot = v_w;
q_dot = 0.5 * [-q1, -q2, -q3; q0, -q3, q2; q3, q0, -q1; -q2, q1, q0] * omega_b;
v_dot = a_;
omega_dot = [ (u(2)/Ix) + (omega_b(2)*omega_b(3)*(Iy - Iz))/Ix;
              (u(3)/Iy) - (omega_b(1)*omega_b(3)*(Ix - Iz))/Iy;
              (u(4)/Iz) + (omega_b(1)*omega_b(2)*(Ix - Iy))/Iz ];

% Gimbal dynamics (first-order)
phi_g_dot = u(5);
theta_g_dot = u(6);

% Assemble state derivative vector (15 states)
state_dot = zeros(15,1);
state_dot(1:3) = x_dot;
state_dot(4:7) = q_dot;
state_dot(8:10) = v_dot;
state_dot(11:13) = omega_dot;
state_dot(14) = phi_g_dot;
state_dot(15) = theta_g_dot;

% Store history

%gimbal_global_roll = phi_g;

history(end+1, :) = [t, u(1), u(2), u(3), u(4), omega_b', u(5), u(6), gimbal_global_roll, corrected_roll, corrected_pitch, corrected_yaw];

fprintf('Gimbal Local Roll Angle (phi_g): %f, Gimbal Global Roll Angle: %f\n', phi_g, gimbal_global_roll);

end
