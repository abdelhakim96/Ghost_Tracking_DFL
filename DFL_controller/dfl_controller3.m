function u = dfl_controller3(t, state, xd, vd, ad, jd, sd, psid, fw_state, fw_orientation, dfl_gains)
% Phase 1 + 5 DFL controller. 18-state plant, 7-input DFL.
%   Outputs being linearised: [drone x, y, z, q3_drone, phi_g, theta_g, psi_g].
%   Gimbal:  q_G = q_x(phi_g) (x) q_y(theta_g) (x) q_z(psi_g)  (XYZ Euler).
%   Lever arm t_G = 0 (camera at drone CoM).
%
% Inputs (DFL):
%   u(1) = T_ddot
%   u(2) = tau_phi, u(3) = tau_theta, u(4) = tau_psi
%   u(5) = phi_g_dot, u(6) = theta_g_dot, u(7) = psi_g_dot
%
% psid argument is accepted for interface compatibility but is now mostly
% unused: the 3-axis gimbal absorbs camera-orientation matching, so the
% drone's free yaw is just held at q3 = 0 (yaw = 0).

    global m Ix Iy Iz g

    x_w     = state(1:3);
    q_bw    = state(4:7);
    v_w     = state(8:10);
    omega_b = state(11:13);
    phi_g   = state(14);
    theta_g = state(15);
    psi_g   = state(16);
    zeta    = state(17);
    xi      = state(18);

    c0 = dfl_gains.c0;  c1 = dfl_gains.c1;
    c2 = dfl_gains.c2;  c3 = dfl_gains.c3;
    cq = dfl_gains.c_q3;     % gain on q3 error
    cqd = dfl_gains.c_q3_dot; % gain on q3 rate
    c_phi   = dfl_gains.c_phi;
    c_theta = dfl_gains.c_theta;
    c_psig  = dfl_gains.c_psig;

    q_bw = q_bw / (norm(q_bw) + 1e-9);
    q0=q_bw(1); q1=q_bw(2); q2=q_bw(3); q3=q_bw(4);

    R_bw = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
            2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
            2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];

    F_thrust = R_bw * [0; 0; zeta];
    a_ = (F_thrust/m) - [0; 0; g];
    j = (zeta*(R_bw(:,1)*omega_b(2) - R_bw(:,2)*omega_b(1)) + R_bw(:,3)*xi)/m;
    % q3_dot in body frame
    q3_dot = 0.5 * (-q2*omega_b(1) + q1*omega_b(2) + q0*omega_b(3));

    % Position virtual control (4th-order chain)
    v_pos = sd - c3*(j - jd) - c2*(a_ - ad) - c1*(v_w - vd) - c0*(x_w - xd);

    % Drone-yaw output: q3, tracked to 0 (drone yaw = 0)
    v_q3 = 0 - cqd*(q3_dot - 0) - cq*(q3 - 0);

    % ---- Gimbal references from q_rel = qbar_M (x) q_A ----------------------
    q_fw  = fw_orientation(:) / (norm(fw_orientation) + 1e-9);
    q_rel = quat_mul(quat_conj(q_bw(:)), q_fw);

    % Extract XYZ Euler (phi_g, theta_g, psi_g) from q_rel = q_x(phi)(x)q_y(theta)(x)q_z(psi)
    [phi_g_ref, theta_g_ref, psi_g_ref] = quat_to_XYZ_euler(q_rel);
    [phi_g_ref, theta_g_ref, psi_g_ref] = unwrap_refs(t, phi_g_ref, theta_g_ref, psi_g_ref);

    % Rate feedforward (body-frame identity): omega_cam_b = omega_drone_b + J_G * dTheta
    %   With q_G = q_x (x) q_y (x) q_z, the body-frame gimbal Jacobian is:
    %     J_G(phi,theta) = [ 1,          0,           sin(theta);
    %                        0,  cos(phi),  -sin(phi)*cos(theta);
    %                        0,  sin(phi),   cos(phi)*cos(theta) ]
    %   det J_G = cos(theta) ; well-conditioned away from theta = +-pi/2.
    fw_omega_b = fw_state(11:13);
    R_gb_des   = R_bw' * quat2rotm(q_fw');
    om_demand  = R_gb_des * fw_omega_b - omega_b;
    Jg = [1, 0,                  sin(theta_g);
          0, cos(phi_g),        -sin(phi_g)*cos(theta_g);
          0, sin(phi_g),         cos(phi_g)*cos(theta_g)];
    if abs(cos(theta_g)) > 1e-3
        dTheta_ref = Jg \ om_demand;
    else
        dTheta_ref = pinv(Jg) * om_demand;
    end
    phi_g_ref_dot   = dTheta_ref(1);
    theta_g_ref_dot = dTheta_ref(2);
    psi_g_ref_dot   = dTheta_ref(3);

    v_phi   = -c_phi  *(phi_g   - phi_g_ref)   + phi_g_ref_dot;
    v_theta = -c_theta*(theta_g - theta_g_ref) + theta_g_ref_dot;
    v_psig  = -c_psig *(psi_g   - psi_g_ref)   + psi_g_ref_dot;

    v = [v_pos; v_q3; v_phi; v_theta; v_psig];

    Ag_p = 0.01; Ag_q = 0.01; Ag_r = 0.01;
    Ig_x = 0.001; Ig_y = 0.001; Ig_z = 0.001;

    alpha_val = alpha_gimbal_func3(state, 0, 0, 0, Ag_p, Ag_q, Ag_r, Ix, Iy, Iz, Ig_x, Ig_y, Ig_z, zeta, xi, m);
    beta_val  = beta_gimbal_func3 (state, 0, 0, 0, Ag_p, Ag_q, Ag_r, Ix, Iy, Iz, Ig_x, Ig_y, Ig_z, zeta, xi, m);
    u = alpha_val + beta_val * v;
end

function [phi, theta, psi] = quat_to_XYZ_euler(q)
% Inverse of q = q_x(phi) (x) q_y(theta) (x) q_z(psi).
% Rotation matrix:
%   R = R_x(phi) * R_y(theta) * R_z(psi)
% =>  R(1,3) = sin(theta), R(1,1) = cos(theta) cos(psi), R(1,2) = -cos(theta) sin(psi)
%     R(2,3) = -sin(phi) cos(theta), R(3,3) = cos(phi) cos(theta)
    q = q(:); q = q/(norm(q)+1e-12);
    q0=q(1); q1=q(2); q2=q(3); q3=q(4);
    R = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
         2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
         2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];
    sth = max(-1, min(1, R(1,3)));
    theta = asin(sth);
    if abs(cos(theta)) > 1e-6
        psi = atan2(-R(1,2), R(1,1));
        phi = atan2(-R(2,3), R(3,3));
    else
        % Singular at theta = +-pi/2 — pick a consistent branch
        psi = 0;
        phi = atan2(R(3,2), R(2,2));
    end
end

function [phi, theta, psi] = unwrap_refs(t, phi, theta, psi)
    persistent last_phi last_theta last_psi
    if t == 0
        last_phi = []; last_theta = []; last_psi = [];
    end
    if isempty(last_phi)
        last_phi = phi; last_theta = theta; last_psi = psi;
        return;
    end
    phi   = last_phi   + wrap(phi   - last_phi);
    theta = last_theta + wrap(theta - last_theta);
    psi   = last_psi   + wrap(psi   - last_psi);
    last_phi = phi; last_theta = theta; last_psi = psi;
end

function d = wrap(d)
    d = mod(d + pi, 2*pi) - pi;
end
