function [xdot, acc_ned, jerk_ned, snap_ned] = fw_6dof_quat(t, state, thrust, elevator, aileron, rudder, params)
    % Unpack the state vector
    x = state(1); y = state(2); z = state(3);
    u = state(4); v = state(5); w = state(6);
    qBN = state(7:10); % [q0, q1, q2, q3] scalar-first convention for MATLAB
    qBN = qBN / (norm(qBN) + 1e-9); % Normalize the quaternion
    p = state(11); q = state(12); r = state(13);

    % Retrieve parameters
    rho = params.rho; S = params.S; b = params.b; c = params.c; g = params.g;
    m_fw = params.m;
    J = params.J;

    % Aerodynamic Coefficients (simplified for clarity, assuming they are all in params)
    CL0 = params.CL0; CL_a = params.CL_alpha; CL_q = params.CL_q; CL_de = params.CL_de;
    CD0 = params.CD0; k_ind = params.k; CD_a = params.CDa; CD_q = params.CD_q; CD_de = params.CD_de;
    CY_b = params.CY_beta; CY_p = params.CY_p; CY_r = params.CY_r; CY_da = params.CY_da; CY_dr = params.CY_dr;
    Cl_b = params.Cl_beta; Cl_p = params.Cl_p; Cl_r = params.Cl_r; Cl_da = params.Cl_da; Cl_dr = params.Cl_dr;
    Cm0 = params.Cm0; Cm_a = params.Cm_alpha; Cm_q = params.Cm_q; Cm_de = params.Cm_de;
    Cn_b = params.Cn_beta; Cn_p = params.Cn_p; Cn_r = params.Cn_r; Cn_da = params.Cn_da; Cn_dr = params.Cn_dr;

    % Rotation Matrix from Body to NED
    R_BN = quat_to_R_BN(qBN);
    R_NB = R_BN';

    % Kinematics
    v_b = [u; v; w];
    v_ned = R_BN * v_b;
    x_dot = v_ned(1); y_dot = v_ned(2); z_dot = v_ned(3);

    % Aerodynamics
    wind = [0; 0; 0]; % Assuming no wind for simplicity
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

    ca = cos(alpha);
    sa = sin(alpha);

    % Standard transformation from wind frame to body frame
    F_aero_b = [ -Drag*ca + Lift*sa;
                 Side;
                 -Drag*sa - Lift*ca ];
    M_aero_b = qbar * S * [b * Cl; c * Cm; b * Cn];

    % Thrust Force and Moment
    F_thrust_b = [thrust; 0; 0];
    M_thrust_b = [0; 0; 0]; % Assuming thrust acts through CG

    % Gravitational Force
    F_grav_b = R_NB * [0; 0; g * m_fw];

    % Total Forces and Moments
    F_b = F_aero_b + F_thrust_b + F_grav_b;
    M_b = M_aero_b + M_thrust_b;

    % Dynamics
    omega = [p; q; r];
    v_dot_b = (1 / m_fw) * F_b - cross(omega, v_b);
    u_dot = v_dot_b(1); v_dot = v_dot_b(2); w_dot = v_dot_b(3);

    % Attitude Kinematics (scalar-first quaternion)
    q0 = qBN(1); q1 = qBN(2); q2 = qBN(3); q3 = qBN(4);
    q0_dot = -0.5 * (p*q1 + q*q2 + r*q3);
    q1_dot = 0.5 * (p*q0 + r*q2 - q*q3);
    q2_dot = 0.5 * (q*q0 - r*q1 + p*q3);
    q3_dot = 0.5 * (r*q0 + q*q1 - p*q2);
    q_dot = [q0_dot; q1_dot; q2_dot; q3_dot];
    
    % Rotational Dynamics
    omega_dot = inv(J) * (M_b - cross(omega, J * omega));
    p_dot = omega_dot(1); q_dotb = omega_dot(2); r_dot = omega_dot(3);

    % Assemble state derivative
    xdot = zeros(13, 1);
    xdot(1:3) = [x_dot; y_dot; z_dot];
    xdot(4:6) = [u_dot; v_dot; w_dot];
    xdot(7:10) = q_dot;
    xdot(11:13) = [p_dot; q_dotb; r_dot];

    % Calculate acceleration in NED frame for output
    acc_ned = R_BN * v_dot_b;

    % Simplified jerk and snap calculations (assuming constant control inputs)
    jerk_ned = zeros(3,1);
    snap_ned = zeros(3,1);
end

function R = quat_to_R_BN(q)
    % Standard scalar-first quaternion to rotation matrix
    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
    
    R = [1 - 2*(q2^2 + q3^2),   2*(q1*q2 - q0*q3),   2*(q1*q3 + q0*q2);
         2*(q1*q2 + q0*q3),   1 - 2*(q1^2 + q3^2),   2*(q2*q3 - q0*q1);
         2*(q1*q3 - q0*q2),   2*(q2*q3 + q0*q1),   1 - 2*(q1^2 + q2^2)];
end
