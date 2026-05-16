function u = dfl_controller(t, state, xd, vd, ad, jd, sd, psid,fw_state, fw_orientation, dfl_gains)
% This function computes the control input for the quadrotor using a DFL controller
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

% Gimbal virtual control --- paper-consistent X-Y extraction from q_rel
% Paper: q_M (x) q_G = q_A  with  q_G = q_x(phi_g) (x) q_y(theta_g).
% After the yaw schedule (applied upstream in unified_dynamics via psid),
% q_rel = qbar_M (x) q_A lies (approximately) on the X-Y submanifold of SO(3),
% so we can read off (phi_g, theta_g) directly.

q_fw = fw_orientation / (norm(fw_orientation) + 1e-9);
q_rel = quat_mul(quat_conj(q_bw(:)), q_fw(:));

% q_x(phi) (x) q_y(theta) -> rotation matrix:
%   R = [ cos(theta)            0           sin(theta);
%         sin(phi)*sin(theta)   cos(phi)   -sin(phi)*cos(theta);
%        -cos(phi)*sin(theta)   sin(phi)    cos(phi)*cos(theta) ];
% So:  sin(theta) =  R(1,3),  cos(theta) = sqrt(R(1,1)^2 + R(1,2)^2)
%      sin(phi)   = -R(2,3)/cos(theta)? — actually:
%      phi   = atan2( R(2,3) * (-1) , R(3,3) )   no — use:
% Re-derive from rotation: with R built as above,
%   theta = atan2(  R(1,3),  R(1,1) )       (since R(1,2)=0 in this submanifold)
%   phi   = atan2( -R(2,3),  R(3,3) )
% These are exact when q_rel is on the X-Y submanifold; off-manifold they are
% the projection.
qr = q_rel / (norm(q_rel) + 1e-9);
qr0=qr(1); qr1=qr(2); qr2=qr(3); qr3=qr(4);
R_rel = [qr0^2+qr1^2-qr2^2-qr3^2, 2*(qr1*qr2-qr0*qr3), 2*(qr1*qr3+qr0*qr2);
         2*(qr1*qr2+qr0*qr3), qr0^2-qr1^2+qr2^2-qr3^2, 2*(qr2*qr3-qr0*qr1);
         2*(qr1*qr3-qr0*qr2), 2*(qr2*qr3+qr0*qr1), qr0^2-qr1^2-qr2^2+qr3^2];

theta_g_ref_raw = atan2( R_rel(1,3),  R_rel(1,1));
phi_g_ref_raw   = atan2(-R_rel(2,3),  R_rel(3,3));

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

% Virtual control for gimbal (first-order system).
% Body-frame derivation: with q_G = q_x(phi_g) (x) q_y(theta_g), the gimbal
% contribution to the camera angular velocity, expressed in DRONE body frame, is
%   omega_gimbal_in_drone_body = [ dphi_g;  cos(phi_g)*dtheta_g;  sin(phi_g)*dtheta_g ].
% We want  omega_cam_in_drone_body = omega_drone_in_drone_body + omega_gimbal_in_drone_body
% with omega_cam = omega_FW (mimicry condition q_cam = q_A).
fw_omega_b = fw_state(11:13);
R_gb_desired = R_bw' * quat2rotm(q_fw');      % FW body -> drone body
omega_cam_b_demand = R_gb_desired * fw_omega_b - omega_b;

% Least-squares solve M * [dphi_g; dtheta_g] = omega_cam_b_demand
%   M = [1, 0; 0, cos(phi_g); 0, sin(phi_g)]
% Exact when the demand lies in range(M); residual along (sin(phi_g)e2 - cos(phi_g)e3)
% goes into the drone-yaw channel via the psid schedule (handled upstream).
phi_g_ref_dot   = omega_cam_b_demand(1);
theta_g_ref_dot = cos(phi_g)*omega_cam_b_demand(2) + sin(phi_g)*omega_cam_b_demand(3);

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

end
