function u = dfl_controller(t, state, xd, vd, ad, jd, sd, psid, fw_state, dfl_gains)
% FIXED: Proper gimbal kinematics for feedforward term
% Key fix: Account for kinematic coupling in gimbal rates

persistent last_phi_g_ref last_theta_g_ref last_t last_R_gb_desired;

if t == 0
    last_phi_g_ref = [];
    last_theta_g_ref = [];
    last_t = 0;
    last_R_gb_desired = [];
end

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

% Controller gains
c0 = dfl_gains.c0;
c1 = dfl_gains.c1;
c2 = dfl_gains.c2;
c3 = dfl_gains.c3;
c4 = dfl_gains.c4;
c5 = dfl_gains.c5;
c_phi = dfl_gains.c_phi;
c_theta = dfl_gains.c_theta;
c_ff_phi = dfl_gains.c_ff_phi;
c_ff_theta = dfl_gains.c_ff_theta;

% Normalize quaternion
q_bw = q_bw / (norm(q_bw) + 1e-9);
R_bw = quat2rotm(q_bw');

% Dynamics
F_thrust = R_bw * [0; 0; zeta];
a_ = (F_thrust/m) - [0; 0; g];
j = (zeta*(R_bw(:,1)*omega_b(2) - R_bw(:,2)*omega_b(1)) + R_bw(:,3)*xi)/m;

% Drone Euler angles and rates
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
vpsi = eul_drone_rates(1);

% Virtual control input
v_pos = sd - c3*(j - jd) - c2*(a_ - ad) - c1*(v_w - vd) - c0*(x_w - xd);

% Fixed-wing state
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
fw_yaw_rate = eul_fw_rates(1);

% Yaw control
psid = fw_yaw;
psid_dot = fw_yaw_rate;

drone_yaw = eul_drone(1);
yaw_error = wrapToPi(drone_yaw - psid);
v_yaw = psid_dot - c5*(vpsi - psid_dot) - c4*yaw_error;

% Gimbal control
R_fw_w = quat2rotm(q_fw');
R_gb_desired = R_bw' * R_fw_w;

% Extract gimbal angles (matching your dynamics rotation matrix)
theta_g_ref_raw = asin(-R_gb_desired(3,1));
phi_g_ref_raw = atan2(-R_gb_desired(1,2), R_gb_desired(2,2));

% Unwrap angles
if isempty(last_phi_g_ref)
    last_phi_g_ref = phi_g_ref_raw;
    last_theta_g_ref = theta_g_ref_raw;
    last_R_gb_desired = R_gb_desired;
end

phi_g_ref = unwrapAngle(phi_g_ref_raw, last_phi_g_ref);
theta_g_ref = unwrapAngle(theta_g_ref_raw, last_theta_g_ref);

% ========== CORRECTED GIMBAL RATE FEEDFORWARD ==========
% The key insight: We need d/dt(phi_g_ref) and d/dt(theta_g_ref)
% These are NOT simply body rate components due to kinematic coupling

% Method 1: Compute from rotation matrix time derivative
% d/dt(R_gb_desired) relates to angular velocities
% R_gb_desired = R_bw' * R_fw_w
% d/dt(R_gb_desired) = d/dt(R_bw') * R_fw_w + R_bw' * d/dt(R_fw_w)

% Angular velocity of gimbal frame relative to drone body in drone body frame
% omega_gimbal_wrt_drone_in_drone = R_bw' * (omega_fw_world - omega_drone_world)
omega_fw_world = R_fw_w * fw_omega_b;
omega_drone_world = R_bw * omega_b;
omega_gimbal_wrt_drone = R_bw' * (omega_fw_world - omega_drone_world);

% Now convert this angular velocity to gimbal angle rates
% For your gimbal rotation convention (theta around Y, then phi around new X):
% The kinematic relationship is:
% [omega_x]   [1,      0,        -sin(theta_g)  ] [phi_g_dot  ]
% [omega_y] = [0,  cos(phi_g),  sin(phi_g)*cos(theta_g)] [theta_g_dot]
% [omega_z]   [0, -sin(phi_g),  cos(phi_g)*cos(theta_g)] [psi_g_dot  ]
%
% For 2-axis gimbal (no yaw), we only care about first two components:
% omega_x = phi_g_dot - sin(theta_g)*psi_g_dot
% omega_y = cos(phi_g)*theta_g_dot + sin(phi_g)*cos(theta_g)*psi_g_dot
%
% Since psi_g = 0 (2-axis gimbal), we have:
% omega_x = phi_g_dot
% omega_y = cos(phi_g)*theta_g_dot
%
% But wait - this assumes gimbal angles measured from current drone body frame
% Actually, for your rotation matrix, the relationship is simpler:

% Build the Jacobian relating gimbal rates to angular velocity
% For R_gb with theta around Y first, then phi around X:
% d/dt(R_gb) = skew(omega) * R_gb
% where omega is expressed in gimbal body frame

% Actually, the most robust approach: use the current gimbal angles
% The gimbal Jacobian (from gimbal angles to angular velocity in drone body frame)
% For your specific rotation convention:
cp = cos(phi_g);
sp = sin(phi_g);
ct = cos(theta_g);
st = sin(theta_g);

% Jacobian: omega_body = J * [phi_g_dot; theta_g_dot]
J_gimbal = [1,  0;
            0,  cp;
            0, -sp];

% For reference angles:
cp_ref = cos(phi_g_ref);
sp_ref = sin(phi_g_ref);
ct_ref = cos(theta_g_ref);
st_ref = sin(theta_g_ref);

J_gimbal_ref = [1,    0;
                0,   cp_ref;
                0,  -sp_ref];

% Solve for gimbal rates (pseudo-inverse)
% We only use first two components of omega_gimbal_wrt_drone
if abs(cp_ref) > 0.1
    gimbal_rates = J_gimbal_ref(1:2,:) \ omega_gimbal_wrt_drone(1:2);
    phi_g_ref_dot = gimbal_rates(1);
    theta_g_ref_dot = gimbal_rates(2);
else
    % Near singularity, use simplified approach
    phi_g_ref_dot = omega_gimbal_wrt_drone(1);
    theta_g_ref_dot = omega_gimbal_wrt_drone(2) / max(0.1, abs(cp_ref));
end

% Alternative method: Numerical differentiation (more robust for debugging)
dt = t - last_t;
if dt > 1e-6 && dt < 0.1  % Reasonable time step
    phi_g_ref_dot_numerical = (phi_g_ref_raw - last_phi_g_ref) / dt;
    theta_g_ref_dot_numerical = (theta_g_ref_raw - last_theta_g_ref) / dt;
    
    % Use numerical differentiation with limiting
    phi_g_ref_dot = max(-5, min(5, phi_g_ref_dot_numerical));
    theta_g_ref_dot = max(-5, min(5, theta_g_ref_dot_numerical));
end

% Update persistent variables
last_phi_g_ref = phi_g_ref_raw;
last_theta_g_ref = theta_g_ref_raw;
last_t = t;
last_R_gb_desired = R_gb_desired;

% Virtual control for gimbal
phi_g_error = wrapToPi(phi_g - phi_g_ref);
theta_g_error = wrapToPi(theta_g - theta_g_ref);

v_phi = -c_phi * phi_g_error + c_ff_phi * phi_g_ref_dot;
v_theta = -c_theta * theta_g_error + c_ff_theta * theta_g_ref_dot;

% Combined virtual control vector
v = [v_pos; v_yaw; v_phi; v_theta];

% Gimbal parameters
Ag_p = 0.01;
Ag_q = 0.01;
Ig_x = 0.001;
Ig_y = 0.001;

% Feedback linearization
alpha_val = alpha_gimbal_func(state, 0, 0, 0, Ag_p, Ag_q, Ix, Iy, Iz, Ig_x, Ig_y, zeta, xi, m);
beta_val = beta_gimbal_func(state, 0, 0, 0, Ag_p, Ag_q, Ix, Iy, Iz, Ig_x, Ig_y, zeta, xi, m);
u = alpha_val + beta_val * v;

% Debug
if mod(round(t*100), 50) == 0
    fprintf('t=%.2f | FW[φ=%.1f θ=%.1f ψ=%.1f] Gim[φ=%.1f→%.1f θ=%.1f→%.1f] Rates[%.2f %.2f] Err[%.1f° %.1f°]\n', ...
        rad2deg(fw_roll), rad2deg(fw_pitch), rad2deg(fw_yaw), ...
        rad2deg(phi_g), rad2deg(phi_g_ref), rad2deg(theta_g), rad2deg(theta_g_ref), ...
        phi_g_ref_dot, theta_g_ref_dot, ...
        rad2deg(phi_g_error), rad2deg(theta_g_error));
end

end