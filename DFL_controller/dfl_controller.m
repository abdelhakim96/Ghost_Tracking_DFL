function u = dfl_controller(t, state, xd, vd, ad, jd, sd, psid, fw_state, dfl_gains, corrected_roll, corrected_pitch, corrected_yaw)
% DFL_CONTROLLER - Differential Flatness-based controller for quadrotor
% with a first-order gimbal model. The controller includes feedforward
% terms for both the drone and the gimbal.

persistent last_phi_g_ref last_theta_g_ref last_t

if t == 0 % Reset persistent variables at the start of the simulation
    last_phi_g_ref = [];
    last_theta_g_ref = [];
    last_t = 0;
end

% --- Global constants ---
global m Ix Iy Iz g

% --- Unpack state vector (17 states for 1st order gimbal) ---
x_w      = state(1:3);       % Position in World Frame [N, E, D]
q_bw     = state(4:7);       % Quaternion from Body to World [q0, q1, q2, q3]
v_w      = state(8:10);      % Velocity in World Frame
omega_b  = state(11:13);     % Angular velocity in Body Frame [p, q, r]
phi_g    = state(14);        % Gimbal roll
theta_g  = state(15);        % Gimbal pitch
zeta     = state(16);        % Total thrust
xi       = state(17);        % Derivative of total thrust

% --- Controller gains ---
c0 = dfl_gains.c0;  
c1 = dfl_gains.c1;  
c2 = dfl_gains.c2;  
c3 = dfl_gains.c3;  
c4 = dfl_gains.c4;  
c5 = dfl_gains.c5;  

c_phi     = dfl_gains.c_phi;
c_theta   = dfl_gains.c_theta;
c_ff_phi  = dfl_gains.c_ff_phi;
c_ff_theta= dfl_gains.c_ff_theta;

% --- Normalize quaternion (scalar-first) ---
q_bw = q_bw / (norm(q_bw) + 1e-9);

% --- Rotation matrix from Body to World ---
R_bw = quat2rotm(q_bw');

% --- Drone translational dynamics ---
F_thrust = R_bw * [0; 0; zeta];
a_ = (F_thrust / m) - [0; 0; g];
j = (zeta * (R_bw(:,1)*omega_b(2) - R_bw(:,2)*omega_b(1)) + R_bw(:,3)*xi) / m;

% --- Extract drone Euler angles (corrected) ---
phi   = corrected_roll;  
theta = corrected_pitch; 
psi   = corrected_yaw;

% --- Convert body rates to Euler rates [phi_dot; theta_dot; psi_dot] ---
if abs(cos(theta)) < 1e-6
    eul_drone_rates = [0; 0; 0]; % Avoid singularities
else
    T_body2eul = [1, sin(phi)*tan(theta),  cos(phi)*tan(theta);
                  0, cos(phi),            -sin(phi);
                  0, sin(phi)/cos(theta),  cos(phi)/cos(theta)];
    eul_drone_rates = T_body2eul * omega_b;
end

vpsi = eul_drone_rates(3); % ✅ Correct yaw rate (was previously wrong)

% --- Virtual position control (flatness-based) ---
v_pos = sd - c3*(j - jd) - c2*(a_ - ad) - c1*(v_w - vd) - c0*(x_w - xd);

% --- Fixed-wing state (for synchronization) ---
q_fw = fw_state(7:10) / (norm(fw_state(7:10)) + 1e-9);
fw_omega_b = fw_state(11:13);

% Convert FW quaternion to Euler (ZYX → [yaw, pitch, roll])
eul_fw = quat2eul(q_fw', 'ZYX');
fw_yaw   = eul_fw(1);
fw_pitch = eul_fw(2);
fw_roll  = eul_fw(3);

% --- Convert FW body rates to Euler rates ---
if abs(cos(fw_pitch)) < 1e-6
    eul_fw_rates = [0; 0; 0];
else
    T_fw = [1, sin(fw_roll)*tan(fw_pitch),  cos(fw_roll)*tan(fw_pitch);
            0, cos(fw_roll),               -sin(fw_roll);
            0, sin(fw_roll)/cos(fw_pitch),  cos(fw_roll)/cos(fw_pitch)];
    eul_fw_rates = T_fw * fw_omega_b;
end
fw_yaw_rate = eul_fw_rates(3);

% --- Desired yaw trajectory (drone follows fixed-wing yaw) ---
psid     = fw_yaw;
psid_dot = fw_yaw_rate;

% --- Yaw control (feedforward + PD structure) ---
yaw_error = wrapToPi(psi - psid);
v_yaw = psid_dot - c5*(vpsi - psid_dot) - c4*yaw_error;

% --- Gimbal control ---
R_fw_w = quat2rotm(q_fw');
R_gb_desired = R_bw' * R_fw_w;  % Desired gimbal orientation (relative)

% Convert to Euler (ZYX → [yaw, pitch, roll])
eul_gb_desired = rotm2eul(R_gb_desired, 'ZYX');
theta_g_ref_raw = eul_gb_desired(2); % Pitch
phi_g_ref_raw   = eul_gb_desired(3); % Roll

% Unwrap angles and compute desired gimbal rates
if isempty(last_phi_g_ref)
    phi_g_ref   = phi_g_ref_raw;
    theta_g_ref = theta_g_ref_raw;
    phi_g_ref_dot = 0;
    theta_g_ref_dot = 0;
else
    phi_g_ref   = unwrapAngle(phi_g_ref_raw, last_phi_g_ref);
    theta_g_ref = unwrapAngle(theta_g_ref_raw, last_theta_g_ref);
    
    delta_t = t - last_t;
    if delta_t > 1e-6 % Avoid division by zero
        phi_g_ref_dot = (phi_g_ref - last_phi_g_ref) / delta_t;
        theta_g_ref_dot = (theta_g_ref - last_theta_g_ref) / delta_t;
    else
        phi_g_ref_dot = 0;
        theta_g_ref_dot = 0;
    end
end

% Update persistent variables for next iteration
last_phi_g_ref   = phi_g_ref;
last_theta_g_ref = theta_g_ref;
last_t = t;

% --- Virtual control for gimbal (PD + feedforward) ---
phi_g_error   = phi_g - phi_g_ref;
theta_g_error = theta_g - theta_g_ref;
v_phi   = -c_phi   * phi_g_error   + c_ff_phi   * phi_g_ref_dot;
v_theta = -c_theta * theta_g_error + c_ff_theta * theta_g_ref_dot;

% --- Combined virtual control vector ---
v = [v_pos; v_yaw; v_phi; v_theta];

% --- Gimbal parameters (placeholders for 1st order model) ---
Ag_p = 0.01;
Ag_q = 0.01;
Ig_x = 0.001;
Ig_y = 0.001;

% --- Feedback linearization ---
alpha_val = alpha_gimbal_func(state, 0, 0, 0, Ag_p, Ag_q, Ix, Iy, Iz, Ig_x, Ig_y, zeta, xi, m);
beta_val  = beta_gimbal_func(state, 0, 0, 0, Ag_p, Ag_q, Ix, Iy, Iz, Ig_x, Ig_y, zeta, xi, m);

u = alpha_val + beta_val * v;

end
