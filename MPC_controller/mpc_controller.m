function u = mpc_controller(t, state, xd, vd, ad, jd, sd, psid, fw_state, mpc_gains)
% This function computes the control input for the quadrotor using a NMPC
% controller with CasADi.

persistent opti sol x0 U X x_ref

import casadi.*

% MPC parameters
N = 10; % Prediction horizon
nx = 15; % Number of states
nu = 6;  % Number of inputs
dt = 0.1; % Discretization time step

% Unpack gains
Q = mpc_gains.Q;
R = mpc_gains.R;

% Define global variables
global m Ix Iy Iz g

if isempty(opti)
    % --- CasADi setup ---
    opti = casadi.Opti();

    % Decision variables
    X = opti.variable(nx, N+1); % State trajectory
    U = opti.variable(nu, N);   % Control trajectory

    % Parameters
    x0 = opti.parameter(nx, 1); % Initial state
    x_ref = opti.parameter(nx, N+1); % Reference trajectory

    % --- Dynamics function ---
    f = @(x,u) [x(8:10); ...
                0.5 * [-x(5), -x(6), -x(7); x(4), -x(7), x(6); x(7), x(4), -x(5); -x(6), x(5), x(4)] * x(11:13); ...
                (casadi_quat2rotm(x(4:7)) * [0; 0; u(1)])/m + [0; 0; g]; ...
                [ (u(2)/Ix) + (x(12)*x(13)*(Iy - Iz))/Ix; ...
                  (u(3)/Iy) - (x(11)*x(13)*(Ix - Iz))/Iy; ...
                  (u(4)/Iz) + (x(11)*x(12)*(Ix - Iy))/Iz ]; ...
                u(5:6)];

    % --- Dynamics constraints ---
    for k = 1:N
        % Forward Euler integration
        x_next = X(:,k) + dt * f(X(:,k), U(:,k));
        opti.subject_to(X(:,k+1) == x_next);
        
        % Quaternion normalization (relaxed)
        opti.subject_to(sum(X(4:7,k).^2) >= 0.99);
        opti.subject_to(sum(X(4:7,k).^2) <= 1.01);
    end

    % --- Cost function ---
    cost = 0;
    for k = 1:N
        % State and control at step k
        x_k = X(:, k);
        u_k = U(:, k);
        ref_k = x_ref(:, k);

        % Quaternion error
        q_err = quat_mul(quat_conj(ref_k(4:7)), x_k(4:7));
        
        % Cost for other states
        non_quat_states = [1:3, 8:15];
        state_err = x_k(non_quat_states) - ref_k(non_quat_states);
        
        % Correctly select the weights for the non-quaternion states from the Q matrix
        Q_non_quat = Q([1:3, 7:14], [1:3, 7:14]);

        cost = cost + state_err' * Q_non_quat * state_err + ...
               q_err(2:4)' * Q(4:6, 4:6) * q_err(2:4) + ...
               u_k' * R * u_k;
    end
    % Terminal cost
    x_N = X(:, N+1);
    ref_N = x_ref(:, N+1);
    q_err_N = quat_mul(quat_conj(ref_N(4:7)), x_N(4:7));
    state_err_N = x_N(non_quat_states) - ref_N(non_quat_states);
    
    % Use the same corrected weights for the terminal cost
    cost = cost + state_err_N' * Q_non_quat * state_err_N + ...
           q_err_N(2:4)' * Q(4:6, 4:6) * q_err_N(2:4);

    opti.minimize(cost);

    % --- Boundary constraints ---
    opti.subject_to(X(:, 1) == x0);

    % --- Bounds on control inputs ---
    max_thrust = 2*m*g;
    min_thrust = 0;
    max_torque = 10;
    max_gimbal_rate = pi;
    opti.subject_to(U(1, :) >= min_thrust);
    opti.subject_to(U(1, :) <= max_thrust);
    for i = 2:4
        opti.subject_to(U(i, :) >= -max_torque);
        opti.subject_to(U(i, :) <= max_torque);
    end
    for i = 5:6
        opti.subject_to(U(i, :) >= -max_gimbal_rate);
        opti.subject_to(U(i, :) <= max_gimbal_rate);
    end

    % --- Solve the NLP ---
    opts = struct('ipopt', struct('print_level', 0), 'print_time', 0);
    opti.solver('ipopt', opts);
end

% --- Set parameter values ---
opti.set_value(x0, state(1:15));

% Reference trajectory
q_fw = fw_state(7:10) / norm(fw_state(7:10));
eul_fw = quat2eul(q_fw', 'ZYX');
fw_roll = eul_fw(3);
fw_pitch = eul_fw(2);

% Fixed-wing angular velocity in body frame
omega_fw_b = fw_state(11:13);

% Transformation from fw body to quad body is assumed to be identity for now
omega_quad_b = omega_fw_b;

ref_traj = [xd; q_fw; vd; omega_quad_b; fw_roll; fw_pitch];
opti.set_value(x_ref, repmat(ref_traj, 1, N+1));

% --- Warm start ---
if ~isempty(sol)
    opti.set_initial(sol.value_variables());
end

% Solve
sol = opti.solve();
u = sol.value(U(:, 1));

end

function R = casadi_quat2rotm(q)
    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
    R = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
         2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
         2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];
end

function q_res = quat_mul(q1, q2)
    w1 = q1(1); x1 = q1(2); y1 = q1(3); z1 = q1(4);
    w2 = q2(1); x2 = q2(2); y2 = q2(3); z2 = q2(4);
    q_res = [w1*w2 - x1*x2 - y1*y2 - z1*z2;
             w1*x2 + x1*w2 + y1*z2 - z1*y2;
             w1*y2 - x1*z2 + y1*w2 + z1*x2;
             w1*z2 + x1*y2 - y1*x2 + z1*w2];
end

function q_conj = quat_conj(q)
    q_conj = [q(1); -q(2); -q(3); -q(4)];
end
