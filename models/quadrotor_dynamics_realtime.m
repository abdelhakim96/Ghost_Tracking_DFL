function state_dot = quadrotor_dynamics_realtime(t, state, xd, vd, ad, jd, sd, psid, fw_state, dfl_gains)
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

% Unpack the state vector (17 states for 1st order gimbal)
x_w = state(1:3);       % Position in World Frame [N, E, D]
q_bw = state(4:7);      % Quaternion from Body to World [q0, q1, q2, q3]
v_w = state(8:10);      % Velocity in World Frame
omega_b = state(11:13); % Angular velocity in Body Frame [p, q, r]
phi_g = state(14);      % Gimbal roll
theta_g = state(15);    % Gimbal pitch
zeta = state(16);       % Total thrust
xi = state(17);         % Derivative of total thrust

% Normalize the quaternion
q_bw = q_bw / (norm(q_bw) + 1e-9);
q0=q_bw(1); q1=q_bw(2); q2=q_bw(3); q3=q_bw(4);

% Rotation matrix from Body to World
R_bw = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
        2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
        2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];

% Calculate gimbal's orientation in world frame for debugging
R_gb = [cos(phi_g)*cos(theta_g), -sin(phi_g), cos(phi_g)*sin(theta_g);
        sin(phi_g)*cos(theta_g),  cos(phi_g), sin(phi_g)*sin(theta_g);
       -sin(theta_g),                 0,       cos(theta_g)];
R_gimbal_w = R_bw * R_gb;
%gimbal_global_roll = atan2(R_gimbal_w(3,2), R_gimbal_w(3,3));

% Call the controller to get the control input u
u = dfl_controller(t, state, xd, vd, ad, jd, sd, psid, fw_state, dfl_gains);

% Dynamics
F_thrust = R_bw * [0; 0; zeta];
a_ = (F_thrust/m) - [0; 0; g];

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

% Store history
gimbal_global_roll =0.0;
history(end+1, :) = [t, zeta, u(2), u(3), u(4), omega_b', u(5), u(6), gimbal_global_roll];


fprintf('Gimbal Local Roll Angle (phi_g): %f, Gimbal Global Roll Angle: %f\n', phi_g, gimbal_global_roll);

end
