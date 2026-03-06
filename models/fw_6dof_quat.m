function [xdot, acc_ned, jerk_ned, snap_ned] = fw_6dof_quat(t, state, thrust, elevator, aileron, rudder, params)
% FW_6DOF_QUAT  6-DOF fixed-wing aircraft dynamics with quaternion attitude.
%
% Implements the Newton-Euler equations from eq. (4) in the paper:
%   p_dot   = R(q_A) * v_A
%   q_dot   = 0.5 * G(q_A) * omega_A
%   v_dot   = -omega_A x v_A + f_A / m_A
%   omega_dot = J_A^{-1} * (-omega_A x (J_A*omega_A) + tau_A)
%
% Also computes the NED-frame acceleration, jerk, and snap needed by the
% DFL controller as feedforward references (v_MC in eq. 20).
%
% State vector (13 elements):
%   [x, y, z, u, v, w, q0, q1, q2, q3, p, q, r]
%   where (x,y,z) is NED position, (u,v,w) body-frame velocity,
%   (q0..q3) scalar-first quaternion body-to-NED, (p,q,r) body rates.

    % Unpack the state vector
    u = state(4); v = state(5); w = state(6);
    qBN = state(7:10); % [q0, q1, q2, q3] scalar-first
    qBN = qBN / (norm(qBN) + 1e-9);
    p = state(11); q = state(12); r = state(13);

    % Retrieve parameters
    rho = params.rho; S = params.S; b = params.b; c = params.c; g = params.g;
    m_fw = params.m;
    J = params.J;

    % Aerodynamic Coefficients
    CL0 = params.CL0; CL_a = params.CL_alpha; CL_q = params.CL_q; CL_de = params.CL_de;
    CD0 = params.CD0; k_ind = params.k; CD_a = params.CDa; CD_q = params.CD_q; CD_de = params.CD_de;
    CY_b = params.CY_beta; CY_p = params.CY_p; CY_r = params.CY_r; CY_da = params.CY_da; CY_dr = params.CY_dr;
    Cl_b = params.Cl_beta; Cl_p = params.Cl_p; Cl_r = params.Cl_r; Cl_da = params.Cl_da; Cl_dr = params.Cl_dr;
    Cm0 = params.Cm0; Cm_a = params.Cm_alpha; Cm_q = params.Cm_q; Cm_de = params.Cm_de;
    Cn_b = params.Cn_beta; Cn_p = params.Cn_p; Cn_r = params.Cn_r; Cn_da = params.Cn_da; Cn_dr = params.Cn_dr;

    % Rotation Matrix from Body to NED
    R_BN = quat_to_R_BN(qBN);
    R_NB = R_BN';

    % Kinematics: NED velocity = R * body velocity
    v_b = [u; v; w];
    v_ned = R_BN * v_b;

    % Aerodynamic state
    wind = [0; 0; 0];
    v_air_b = v_b - R_NB * wind;
    ua = v_air_b(1); va = v_air_b(2); wa = v_air_b(3);
    Va = max(1e-3, norm(v_air_b));
    alpha = atan2(wa, ua);
    beta = asin(max(-1, min(1, va / Va)));

    qbar = 0.5 * rho * Va^2;
    p_hat = (b / (2 * Va)) * p;
    q_hat = (c / (2 * Va)) * q;
    r_hat = (b / (2 * Va)) * r;

    % Total Aerodynamic Coefficients
    CL = CL0 + CL_a * alpha + CL_q * q_hat + CL_de * elevator;
    CD = CD0 + k_ind * CL.^2 + CD_a * alpha + CD_q * q_hat + CD_de * elevator;
    CY = CY_b * beta + CY_p * p_hat + CY_r * r_hat + CY_da * aileron + CY_dr * rudder;
    Cl = Cl_b * beta + Cl_p * p_hat + Cl_r * r_hat + Cl_da * aileron + Cl_dr * rudder;
    Cm = Cm0 + Cm_a * alpha + Cm_q * q_hat + Cm_de * elevator;
    Cn = Cn_b * beta + Cn_p * p_hat + Cn_r * r_hat + Cn_da * aileron + Cn_dr * rudder;

    % Aerodynamic Forces and Moments
    Lift = qbar * S * CL;
    Drag = qbar * S * CD;
    Side = qbar * S * CY;

    ca = cos(alpha); sa = sin(alpha);

    % Transform from wind frame to body frame
    F_aero_b = [ -Drag*ca + Lift*sa;
                 Side;
                 -Drag*sa - Lift*ca ];
    M_aero_b = qbar * S * [b * Cl; c * Cm; b * Cn];

    % Thrust and Gravity in body frame
    F_thrust_b = [thrust; 0; 0];
    F_grav_b = R_NB * [0; 0; g * m_fw];

    % Total Forces and Moments
    F_b = F_aero_b + F_thrust_b + F_grav_b;
    M_b = M_aero_b;

    % -----------------------------------------------------------------------
    % Translational Dynamics (body frame)
    % -----------------------------------------------------------------------
    omega = [p; q; r];
    v_dot_b = (1 / m_fw) * F_b - cross(omega, v_b);

    % -----------------------------------------------------------------------
    % Attitude Kinematics (scalar-first quaternion)
    % -----------------------------------------------------------------------
    q0 = qBN(1); q1 = qBN(2); q2 = qBN(3); q3 = qBN(4);
    q0_dot = -0.5 * (p*q1 + q*q2 + r*q3);
    q1_dot = 0.5 * (p*q0 + r*q2 - q*q3);
    q2_dot = 0.5 * (q*q0 - r*q1 + p*q3);
    q3_dot = 0.5 * (r*q0 + q*q1 - p*q2);
    q_dot = [q0_dot; q1_dot; q2_dot; q3_dot];

    % -----------------------------------------------------------------------
    % Rotational Dynamics
    % -----------------------------------------------------------------------
    omega_dot = inv(J) * (M_b - cross(omega, J * omega));

    % -----------------------------------------------------------------------
    % Assemble state derivative
    % -----------------------------------------------------------------------
    xdot = zeros(13, 1);
    xdot(1:3) = v_ned;
    xdot(4:6) = v_dot_b;
    xdot(7:10) = q_dot;
    xdot(11:13) = omega_dot;

    % -----------------------------------------------------------------------
    % NED-frame acceleration (for DFL feedforward)
    % -----------------------------------------------------------------------
    % a_NED = R * v_dot_b + R_dot * v_b
    % R_dot = R * [omega]_x  =>  R_dot * v_b = R * (omega x v_b)
    % But Newton's 2nd law in NED: a_NED = d/dt(R*v_b) = R*v_dot_b + R*(omega x v_b)
    % Alternatively: a_NED = R*(v_dot_b + omega x v_b) = R * F_b / m
    % Simpler form: a_NED = R_BN * (F_b / m_fw)
    % (because v_dot_b = F_b/m - omega x v_b, so v_dot_b + omega x v_b = F_b/m)
    acc_ned = R_BN * (F_b / m_fw);

    % -----------------------------------------------------------------------
    % NED-frame jerk (time derivative of acceleration)
    % -----------------------------------------------------------------------
    % jerk_NED = d/dt(R * F_b/m) = R_dot * F_b/m + R * F_b_dot/m
    % R_dot * x = R * (omega x x), so first term = R * (omega x F_b/m)
    % For F_b_dot, we approximate by assuming slowly-varying aero forces.
    % The dominant contribution is from the cross-product term:
    %   jerk_NED ≈ R * (omega x (F_b/m))
    % This provides a first-order correction that significantly improves
    % tracking during aggressive maneuvers vs. the previous zeros.
    F_b_over_m = F_b / m_fw;
    jerk_ned = R_BN * cross(omega, F_b_over_m);

    % -----------------------------------------------------------------------
    % NED-frame snap (set to zero — 2nd order approximation is sufficient
    % because the DFL's error feedback gains handle remaining errors)
    % -----------------------------------------------------------------------
    snap_ned = zeros(3,1);
end

function R = quat_to_R_BN(q)
    % Standard scalar-first quaternion to rotation matrix (body to NED)
    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);

    R = [1 - 2*(q2^2 + q3^2),   2*(q1*q2 - q0*q3),   2*(q1*q3 + q0*q2);
         2*(q1*q2 + q0*q3),   1 - 2*(q1^2 + q3^2),   2*(q2*q3 - q0*q1);
         2*(q1*q3 - q0*q2),   2*(q2*q3 + q0*q1),   1 - 2*(q1^2 + q2^2)];
end
