function u = dfl_controller(t, state, xd, vd, ad, jd, sd, psid, fw_state, dfl_gains)
% This function computes the control input for the quadrotor using a
% cascaded DFL controller.

persistent last_phi_g_ref last_theta_g_ref;

if t == 0 % Reset persistent variables at the start of the simulation
    last_phi_g_ref = [];
    last_theta_g_ref = [];
end

% Define global variables
global m Ix Iy Iz g

% Unpack the state vector
x_w = state(1:3);
q_bw = state(4:7);
v_w = state(8:10);
omega_b = state(11:13);
phi_g = state(14);
theta_g = state(15);
zeta = state(16);
xi = state(17);

% Normalize the quaternion
q_bw = q_bw / (norm(q_bw) + 1e-9);

% Rotation matrix from Body to World using MATLAB's built-in function
R_bw = quat2rotm(q_bw');

% Dynamics
F_thrust = R_bw * [0; 0; zeta];
a_ = (F_thrust/m) - [0; 0; g];
j = (zeta*(R_bw(:,1)*omega_b(2) - R_bw(:,2)*omega_b(1)) + R_bw(:,3)*xi)/m;

% --- Consistent Yaw Rate Calculation ---
eul_drone = quat2eul(q_bw', 'ZYX');
phi = eul_drone(3);
theta = eul_drone(2);

T_inv = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
         0, cos(phi),            -sin(phi);
         0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

if abs(cos(theta)) < 1e-6
    eul_drone_rates = [0; 0; 0];
else
    eul_drone_rates = T_inv * omega_b;
end
vpsi = eul_drone_rates(3);

% --- UPDATED: Drone follows FW yaw dynamics, Gimbal follows FW roll/pitch ---
fw_orientation = fw_state(7:10);
q_fw = fw_orientation / (norm(fw_orientation) + 1e-9);
eul_fw = quat2eul(q_fw', 'ZYX');
fw_yaw = eul_fw(1);
fw_pitch = eul_fw(2);
fw_roll = eul_fw(3);

fw_omega_b = fw_state(11:13);
phi_fw = fw_roll;
theta_fw = fw_pitch;

T_inv_fw = [1, sin(phi_fw)*tan(theta_fw), cos(phi_fw)*tan(theta_fw);
            0, cos(phi_fw),              -sin(phi_fw);
            0, sin(phi_fw)/cos(theta_fw), cos(phi_fw)/cos(theta_fw)];

if abs(cos(theta_fw)) < 1e-6
    eul_fw_rates = [0; 0; 0];
else
    eul_fw_rates = T_inv_fw * fw_omega_b;
end
fw_yaw_rate = eul_fw_rates(3);

psid = 0.5; % Constant yaw reference
psid_dot = 0; % Derivative of constant is zero

drone_yaw = eul_drone(1);
yaw_error = wrapToPi(drone_yaw - psid);

% --- GIMBAL CONTROL IN WORLD FRAME ---
Rx_g = [1, 0, 0; 0, cos(phi_g), -sin(phi_g); 0, sin(phi_g), cos(phi_g)];
Ry_g = [cos(theta_g), 0, sin(theta_g); 0, 1, 0; -sin(theta_g), 0, cos(theta_g)];
R_gb = Ry_g * Rx_g;
R_wg = R_bw * R_gb;
eul_wg = rotm2eul(R_wg, 'ZYX');
phi_wg = eul_wg(3);
theta_wg = eul_wg(2);

phi_g_ref = fw_roll;
theta_g_ref = fw_pitch;

phi_g_ref_dot = -eul_fw_rates(1);
theta_g_ref_dot = eul_fw_rates(2);

if isempty(last_phi_g_ref)
    last_phi_g_ref = phi_g_ref;
    last_theta_g_ref = theta_g_ref;
end
phi_g_ref_unwrapped = unwrapAngle(phi_g_ref, last_phi_g_ref);
last_phi_g_ref = phi_g_ref_unwrapped;
theta_g_ref_unwrapped = unwrapAngle(theta_g_ref, last_theta_g_ref);
last_theta_g_ref = theta_g_ref_unwrapped;

phi_g_error = wrapToPi(phi_wg - phi_g_ref_unwrapped);
theta_g_error = wrapToPi(theta_wg - theta_g_ref_unwrapped);

% --- DFL1: Drone Position and Yaw Control ---
drone_state = [x_w; q_bw; v_w; omega_b; zeta; xi];
v_drone = sd - dfl_gains.c3*(j - jd) - dfl_gains.c2*(a_ - ad) - dfl_gains.c1*(v_w - vd) - dfl_gains.c0*(x_w - xd);
v_yaw = -dfl_gains.c5*(vpsi - psid_dot) - dfl_gains.c4*yaw_error;
v1 = [v_drone; v_yaw];

% Aerodynamic forces and moments are assumed to be zero for this controller
Ax = 0; Ay = 0; Az = 0;
Ap = 0; Aq = 0; Ar = 0;

alpha1 = alpha_drone_func(drone_state, Ax, Ay, Az, Ap, Aq, Ar);
beta1 = beta_drone_func(drone_state, Ax, Ay, Az, Ap, Aq, Ar);
u1 = alpha1 + beta1 * v1;

% --- DFL2: Gimbal Roll and Pitch Control ---
gimbal_state = [phi_g; theta_g; q_bw; omega_b]; % Use the extended state for world-frame controller
v_phi = -dfl_gains.c_phi * phi_g_error + dfl_gains.c_ff_phi * phi_g_ref_dot;
v_theta = -dfl_gains.c_theta * theta_g_error + dfl_gains.c_ff_theta * theta_g_ref_dot;
v2 = [v_phi; v_theta];

alpha2 = alpha_gimbal_world_func(gimbal_state);
beta2 = beta_gimbal_world_func(gimbal_state);
u2 = alpha2 + beta2 * v2;

% --- Combine Control Inputs ---
u = [u1; u2];

end
