function state_dot = quadrotor_dynamics_realtime3(t, state, xd, vd, ad, jd, sd, psid, fw_state, fw_orientation, dfl_gains)
% 18-state quadrotor + 3-axis gimbal dynamics, driven by dfl_controller3.

    global m Ix Iy Iz g

    q_bw    = state(4:7);
    v_w     = state(8:10);
    omega_b = state(11:13);
    zeta    = state(17);
    xi      = state(18);

    q_bw = q_bw / (norm(q_bw) + 1e-9);
    q0=q_bw(1); q1=q_bw(2); q2=q_bw(3); q3=q_bw(4);

    R_bw = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
            2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
            2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];

    u = dfl_controller3(t, state, xd, vd, ad, jd, sd, psid, fw_state, fw_orientation, dfl_gains);

    F_thrust = R_bw * [0; 0; zeta];
    a_ = (F_thrust/m) - [0; 0; g];

    state_dot = zeros(18, 1);
    state_dot(1:3)   = v_w;
    state_dot(4:7)   = 0.5 * [-q1,-q2,-q3; q0,-q3,q2; q3,q0,-q1; -q2,q1,q0] * omega_b;
    state_dot(8:10)  = a_;
    state_dot(11)    = (u(2)/Ix) + (omega_b(2)*omega_b(3)*(Iy - Iz))/Ix;
    state_dot(12)    = (u(3)/Iy) - (omega_b(1)*omega_b(3)*(Ix - Iz))/Iy;
    state_dot(13)    = (u(4)/Iz) + (omega_b(1)*omega_b(2)*(Ix - Iy))/Iz;
    state_dot(14)    = u(5);                % dphi_g
    state_dot(15)    = u(6);                % dtheta_g
    state_dot(16)    = u(7);                % dpsi_g
    state_dot(17)    = xi;
    state_dot(18)    = u(1);                % T_ddot
end
