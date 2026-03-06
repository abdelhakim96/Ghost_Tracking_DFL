function u = dfl_controller(t, state, xd, vd, ad, jd, sd, psid, fw_state, dfl_gains)
% DFL_CONTROLLER  Dynamic Feedback Linearization controller for the quadrotor.
%
% Implements the DFL control law from eq. (19) in the paper:
%   u_hat = alpha(x_hat) + beta(x_hat) * v
%
% The system has 4 outputs: [x, y, z, R(2,1)] with relative degrees
% [4, 4, 4, 2], totalling 14 = effective state dim (15 - 1 quat constraint).
%
% The yaw channel (R(2,1)) tracks the fixed-wing VELOCITY HEADING,
% NOT the FW's body orientation. This keeps the drone pointed forward
% (yaw stable) while the gimbal handles all camera orientation matching.
% This avoids the beta matrix singularity at R(2,2)=0 which occurs when
% the drone tries to match the FW's rapidly-changing attitude during rolls.
%
% Inputs:
%   state     - 15-element drone state: [pos(3); quat(4); vel_w(3); omega_b(3); zeta; xi]
%   xd, vd    - reference position and velocity (from fixed-wing)
%   ad, jd, sd - reference acceleration, jerk, snap
%   psid      - (unused)
%   fw_state  - fixed-wing state [pos(3); vel_b(3); quat(4); omega_b(3)]
%   dfl_gains - struct with gains c0..c5
%
% Outputs:
%   u - 4-element control: [ddot_T; tau_phi; tau_theta; tau_psi]

global m Ix Iy Iz g

% -----------------------------------------------------------------------
% Unpack the drone state vector (15 states)
% -----------------------------------------------------------------------
x_w = state(1:3);       % Position in World Frame [N, E, D]
q_bw = state(4:7);      % Quaternion Body->World [q0, q1, q2, q3]
v_w = state(8:10);      % Velocity in World Frame
omega_b = state(11:13); % Angular velocity in Body Frame [p, q, r]
zeta = state(14);       % Thrust (DFL extended state)
xi = state(15);         % Thrust rate (DFL extended state)

% Gains
c0 = dfl_gains.c0;  c1 = dfl_gains.c1;
c2 = dfl_gains.c2;  c3 = dfl_gains.c3;
c4 = dfl_gains.c4;  c5 = dfl_gains.c5;

% -----------------------------------------------------------------------
% Rotation matrix & current acceleration/jerk
% -----------------------------------------------------------------------
q_bw = q_bw / (norm(q_bw) + 1e-9);
R_bw = quat2rotm(q_bw');

F_thrust = R_bw * [0; 0; zeta];
a_ = (F_thrust/m) - [0; 0; g];

% Jerk: j = d/dt(R*[0;0;T]/m) = (T*(R(:,1)*q - R(:,2)*p) + R(:,3)*xi)/m
j = (zeta*(R_bw(:,1)*omega_b(2) - R_bw(:,2)*omega_b(1)) + R_bw(:,3)*xi)/m;

% -----------------------------------------------------------------------
% Position virtual control (relative degree 4)
% -----------------------------------------------------------------------
v_pos = sd - c3*(j - jd) - c2*(a_ - ad) - c1*(v_w - vd) - c0*(x_w - xd);

% -----------------------------------------------------------------------
% Yaw virtual control (relative degree 2 via R(2,1))
% -----------------------------------------------------------------------
% Strategy: track the VELOCITY HEADING of the fixed-wing, not its body
% orientation. During a roll, the FW's velocity direction barely changes
% (it flies mostly straight), so the heading reference is smooth and stable.
%
% For small drone pitch/yaw: R(2,1) ≈ sin(yaw) ≈ yaw
% So tracking R(2,1) = sin(heading_ref) provides stable yaw control.

% Compute FW velocity heading from the NED velocity reference
heading_ref = atan2(vd(2), vd(1));

% Desired R(2,1) for the drone = sin(heading_ref)
% (valid when drone pitch is small, which it should be since the drone
% stays roughly level while the gimbal handles orientation)
y4_ref = sin(heading_ref);

% Current drone R(2,1)
y4_drone = R_bw(2,1);

% Time derivative of R(2,1): R_dot(2,1) = R(2,2)*r - R(2,3)*q
y4_dot_drone = R_bw(2,2)*omega_b(3) - R_bw(2,3)*omega_b(2);

% Reference R(2,1) rate ≈ 0 for straight or nearly-straight flight
y4_dot_ref = 0;

% Virtual control for yaw channel
v_yaw = -c5 * (y4_dot_drone - y4_dot_ref) - c4 * (y4_drone - y4_ref);

% -----------------------------------------------------------------------
% Combine and apply DFL: u = alpha(x) + beta(x) * v
% -----------------------------------------------------------------------
v = [v_pos; v_yaw];

% Regularize zeta to prevent 1/zeta singularity in alpha_func.
% alpha_func reads zeta from state(14), so we clamp it in the state copy.
% This is a safety net — with proper initial conditions, zeta should
% never approach zero during normal operation.
zeta_min = 0.1 * m * g;  % 10% of hover thrust
state_safe = state;
if abs(state_safe(14)) < zeta_min
    state_safe(14) = zeta_min * sign(state_safe(14) + 1e-10);
end

alpha_val = alpha_func(state_safe, 0, 0, 0, Ix, Iy, Iz, zeta, xi, m);
beta_val = beta_func(state_safe, 0, 0, 0, Ix, Iy, Iz, zeta, xi, m);
u = alpha_val + beta_val * v;

end
