clear all
clc

addpath('../utilities');
addpath('../models');

%% Define symbolic variables

% Drone states
syms x0 y0 z0 real % absolute position of the UAV
syms q0 q1 q2 q3 real % attitude of the UAV in quaternions
syms u v w real % absolute velocity of the UAV
syms p q r real % angular velocity of drone body
syms m real % mass of the UAV
syms Ix Iy Iz real % moments of inertia of the UAV

% Gimbal states (First Order Model)
syms phi_g theta_g real % gimbal roll and pitch angles (body frame)

% Control and external inputs
syms Ax Ay Az real % aerodynamic forces
syms Ap Aq Ar real % aerodynamic moments
syms g real % gravity constant
syms zeta xi real % internal states for extended system (thrust)

%% Drone rotation matrix from quaternions
R = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
     2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
     2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];

%% Gimbal rotation matrix (Body to Gimbal)
Rx_g = [1, 0, 0;
        0, cos(phi_g), -sin(phi_g);
        0, sin(phi_g), cos(phi_g)];

Ry_g = [cos(theta_g), 0, sin(theta_g);
        0, 1, 0;
        -sin(theta_g), 0, cos(theta_g)];

R_g = Ry_g * Rx_g;

%% World to Gimbal rotation matrix
R_wg = R * R_g;

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DFL1: Drone Position and Yaw Controller
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('=======================================================\n');
fprintf('  DFL1: DRONE POSITION AND YAW CONTROLLER\n');
fprintf('=======================================================\n\n');

% State vector for DFL1
x_bar_drone = [x0; y0; z0; q0; q1; q2; q3; u; v; w; p; q; r; zeta; xi]; % 15 states

% Drone quaternion derivative
q_dot_vec_drone = 0.5 * [-q1, -q2, -q3; q0, -q3, q2; q3, q0, -q1; -q2, q1, q0] * [p; q; r];

% System dynamics for DFL1
fx_bar_drone = [
    u; v; w;                                    % Position derivatives
    q_dot_vec_drone;                            % Quaternion derivatives
    -Ax/m + R(1,3)*zeta/m;                     % u_dot
    -Ay/m + R(2,3)*zeta/m;                     % v_dot
    -Az/m + g + R(3,3)*zeta/m;                 % w_dot
    (Iy - Iz)/Ix*q*r + Ap/Ix;                  % p_dot
    (Iz - Ix)/Iy*p*r + Aq/Iy;                  % q_dot
    (Ix - Iy)/Iz*p*q + Ar/Iz;                  % r_dot
    xi;                                         % zeta_dot
    0];                                         % xi_dot

% Control input matrix for DFL1
% Controls: [u1, u2, u3, u4] = [thrust_accel, roll_moment, pitch_moment, yaw_moment]
gx_bar_drone = [
    zeros(10, 4);           % Position, quaternion, velocity (no direct control)
    0, 1/Ix,    0,    0;   % p_dot controlled by roll moment
    0,    0, 1/Iy,    0;   % q_dot controlled by pitch moment
    0,    0,    0, 1/Iz;   % r_dot controlled by yaw moment
    zeros(1, 4);            % zeta (no direct control)
    1,    0,    0,    0];  % xi_dot controlled by thrust acceleration

% Outputs for DFL1: Position (x, y, z) and Yaw angle
y_out_drone = [
    x0;                             % x position
    y0;                             % y position
    z0;                             % z position
    2*atan2(q3, q0)];              % Yaw angle (no singularities)

fprintf('Outputs: [x, y, z, yaw]\n');
fprintf('Controls: [thrust_accel, roll_moment, pitch_moment, yaw_moment]\n\n');

% Compute relative degrees and generate controller
[alphax_drone, betax_drone, ri_drone] = generate_dfl_controller(x_bar_drone, fx_bar_drone, gx_bar_drone, y_out_drone);

fprintf('\nRelative degrees: [%s]\n', num2str(ri_drone'));
fprintf('Sum of relative degrees: %d\n', sum(ri_drone));
fprintf('State dimension: %d (with quaternion constraint: %d effective)\n\n', length(x_bar_drone), length(x_bar_drone)-1);

% Substitute numeric parameters for drone
param_symbols_drone = [m, Ix, Iy, Iz, g];
param_values_drone = [0.468, 0.0023, 0.0023, 0.0046, 9.81];

alphax_drone_num = subs(alphax_drone, param_symbols_drone, param_values_drone);
betax_drone_num = subs(betax_drone, param_symbols_drone, param_values_drone);

alphax_drone_simplified = simplify(alphax_drone_num);
betax_drone_simplified = simplify(betax_drone_num);

% Generate MATLAB functions for DFL1
fprintf('Generating MATLAB functions for DFL1...\n');
matlabFunction(alphax_drone_simplified, 'File', 'alpha_drone_func', 'Vars', ...
    {x_bar_drone, Ax, Ay, Az, Ap, Aq, Ar}, 'Outputs', {'alpha_val'});
matlabFunction(betax_drone_simplified, 'File', 'beta_drone_func', 'Vars', ...
    {x_bar_drone, Ax, Ay, Az, Ap, Aq, Ar}, 'Outputs', {'beta_val'});
fprintf('✓ Generated: alpha_drone_func.m, beta_drone_func.m\n\n');

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DFL2: WORLD-FRAME GIMBAL CONTROLLER
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('=======================================================\n');
fprintf('  DFL2: WORLD-FRAME GIMBAL POINTING CONTROLLER\n');
fprintf('=======================================================\n\n');

fprintf('NOTE: This controller allows gimbal to point at fixed world targets\n');
fprintf('      while the drone body rotates independently.\n\n');

% State vector for DFL2 (includes drone attitude + gimbal angles)
x_bar_gimbal = [phi_g; theta_g; q0; q1; q2; q3; p; q; r]; % 9 states

% System dynamics for DFL2
% Gimbal angles controlled by rates (u5, u6)
% Drone attitude evolves but is not controlled here (passive dynamics)
q_dot_vec_gimbal = 0.5 * [-q1, -q2, -q3; q0, -q3, q2; q3, q0, -q1; -q2, q1, q0] * [p; q; r];

fx_bar_gimbal = [
    0;                  % phi_g_dot (controlled)
    0;                  % theta_g_dot (controlled)
    q_dot_vec_gimbal;  % Quaternion dynamics (passive - drone attitude changes)
    0; 0; 0];          % Angular rates (passive - drone rotation continues)

% Control input matrix for DFL2
% Controls: [u5, u6] = [gimbal_roll_rate, gimbal_pitch_rate]
gx_bar_gimbal = [
    1, 0;              % phi_g_dot = u5
    0, 1;              % theta_g_dot = u6
    zeros(7, 2)];      % Quaternion and angular rates (not controlled by gimbal)

% Outputs for DFL2: World-frame gimbal orientation
% Extract world-frame gimbal Euler angles from R_wg
phi_wg = atan2(R_wg(3,2), R_wg(3,3));      % World-frame roll
theta_wg = asin(-R_wg(3,1));                % World-frame pitch

y_out_gimbal = [
    phi_wg;            % World-frame gimbal roll
    theta_wg];         % World-frame gimbal pitch

fprintf('Outputs: [phi_wg, theta_wg] (world-frame gimbal angles)\n');
fprintf('Controls: [gimbal_roll_rate, gimbal_pitch_rate]\n\n');

fprintf('NOTE: These outputs are NONLINEAR functions of:\n');
fprintf('      - Drone attitude (q0, q1, q2, q3)\n');
fprintf('      - Gimbal angles (phi_g, theta_g)\n');
fprintf('This is why feedback linearization is necessary!\n\n');

% Compute relative degrees and generate controller
fprintf('WARNING: This computation may take several minutes due to complexity...\n');
[alphax_gimbal, betax_gimbal, ri_gimbal] = generate_dfl_controller(x_bar_gimbal, fx_bar_gimbal, gx_bar_gimbal, y_out_gimbal);

fprintf('\nRelative degrees: [%s]\n', num2str(ri_gimbal'));
fprintf('Sum of relative degrees: %d\n', sum(ri_gimbal));
fprintf('State dimension: %d\n\n', length(x_bar_gimbal));

if sum(ri_gimbal) < length(x_bar_gimbal)
    warning('Internal dynamics present! Sum(ri)=%d < dim(x)=%d', sum(ri_gimbal), length(x_bar_gimbal));
    fprintf('The uncontrolled states (drone angular rates p,q,r and quaternion dynamics)\n');
    fprintf('form the internal dynamics. These should remain bounded naturally.\n\n');
end

% Simplify (no numeric substitution needed - no parameters)
alphax_gimbal_simplified = simplify(alphax_gimbal);
betax_gimbal_simplified = simplify(betax_gimbal);

% Generate MATLAB functions for DFL2
fprintf('Generating MATLAB functions for DFL2...\n');
matlabFunction(alphax_gimbal_simplified, 'File', 'alpha_gimbal_world_func', 'Vars', ...
    {x_bar_gimbal}, 'Outputs', {'alpha_val'});
matlabFunction(betax_gimbal_simplified, 'File', 'beta_gimbal_world_func', 'Vars', ...
    {x_bar_gimbal}, 'Outputs', {'beta_val'});
fprintf('✓ Generated: alpha_gimbal_world_func.m, beta_gimbal_world_func.m\n\n');

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% USAGE INSTRUCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('=======================================================\n');
fprintf('  CASCADED CONTROLLER USAGE\n');
fprintf('=======================================================\n\n');

fprintf('CONTROL LOOP STRUCTURE:\n\n');
fprintf('%% ===== OUTER LOOP: Reference Tracking =====\n');
fprintf('%% Drone position and yaw errors\n');
fprintf('e_drone = [x0; y0; z0; yaw_current] - [x_ref; y_ref; z_ref; yaw_ref];\n');
fprintf('e_drone_dot = ... %% derivative of errors\n\n');

fprintf('%% Drone virtual control (PD law with PRIORITY on yaw)\n');
fprintf('K_p_drone = diag([50, 50, 50, 100]);  %% Higher yaw gain!\n');
fprintf('K_d_drone = diag([20, 20, 20, 40]);\n');
fprintf('v_drone = -K_p_drone * e_drone - K_d_drone * e_drone_dot;\n\n');

fprintf('%% Gimbal world-frame errors\n');
fprintf('phi_wg_current = atan2(R_wg(3,2), R_wg(3,3));\n');
fprintf('theta_wg_current = asin(-R_wg(3,1));\n');
fprintf('e_gimbal = [phi_wg_current; theta_wg_current] - [phi_wg_ref; theta_wg_ref];\n');
fprintf('e_gimbal_dot = ... %% derivative of errors\n\n');

fprintf('%% Gimbal virtual control (LOWER priority - smaller gains)\n');
fprintf('K_p_gimbal = diag([10, 10]);  %% Lower than drone!\n');
fprintf('K_d_gimbal = diag([5, 5]);\n');
fprintf('v_gimbal = -K_p_gimbal * e_gimbal - K_d_gimbal * e_gimbal_dot;\n\n');

fprintf('%% ===== INNER LOOP: Feedback Linearization =====\n');
fprintf('%% DFL1: Drone control\n');
fprintf('x_drone = [x0; y0; z0; q0; q1; q2; q3; u; v; w; p; q; r; zeta; xi];\n');
fprintf('alpha_drone = alpha_drone_func(x_drone, Ax, Ay, Az, Ap, Aq, Ar);\n');
fprintf('beta_drone = beta_drone_func(x_drone, Ax, Ay, Az, Ap, Aq, Ar);\n');
fprintf('u_drone = alpha_drone + beta_drone * v_drone;  %% [u1; u2; u3; u4]\n\n');

fprintf('%% DFL2: Gimbal control\n');
fprintf('x_gimbal = [phi_g; theta_g; q0; q1; q2; q3; p; q; r];\n');
fprintf('alpha_gimbal = alpha_gimbal_world_func(x_gimbal);\n');
fprintf('beta_gimbal = beta_gimbal_world_func(x_gimbal);\n');
fprintf('u_gimbal = alpha_gimbal + beta_gimbal * v_gimbal;  %% [u5; u6]\n\n');

fprintf('%% ===== CONTROL SATURATION =====\n');
fprintf('u_drone = saturate(u_drone, u_max_drone);\n');
fprintf('u_gimbal = saturate(u_gimbal, u_max_gimbal);\n\n');

fprintf('=======================================================\n');
fprintf('  TUNING RECOMMENDATIONS\n');
fprintf('=======================================================\n\n');

fprintf('PHASE 1: Tune drone controller first\n');
fprintf('  - Set gimbal references to current angles (no gimbal motion)\n');
fprintf('  - Tune drone gains until tracking is excellent\n');
fprintf('  - Focus especially on yaw tracking (increase K_p_yaw if needed)\n\n');

fprintf('PHASE 2: Add gimbal motion gradually\n');
fprintf('  - Start with slow gimbal reference changes\n');
fprintf('  - Use rate limiters on gimbal references\n');
fprintf('  - Gradually increase gimbal motion speed\n\n');

fprintf('PHASE 3: Balance priorities\n');
fprintf('  - If yaw degrades: INCREASE drone yaw gains OR DECREASE gimbal gains\n');
fprintf('  - If gimbal lags too much: slightly increase gimbal gains\n');
fprintf('  - Keep K_p_yaw >> K_p_gimbal (e.g., 100 vs 10)\n\n');

fprintf('TYPICAL GAIN VALUES:\n');
fprintf('  Drone position: K_p = 50, K_d = 20\n');
fprintf('  Drone yaw:      K_p = 100, K_d = 40  (2x position gains)\n');
fprintf('  Gimbal:         K_p = 10, K_d = 5    (5x lower than yaw)\n\n');

%% Save workspace
save('cascaded_controller_workspace.mat');
fprintf('Workspace saved to: cascaded_controller_workspace.mat\n\n');
fprintf('=======================================================\n');
fprintf('  GENERATION COMPLETE!\n');
fprintf('=======================================================\n');

%% Helper function to generate DFL controller
function [alphax, betax, ri] = generate_dfl_controller(x_bar, fx_bar, gx_bar, hx)
    fprintf('Computing relative degrees...\n');
    max_degree = 10;
    ri = zeros(length(hx), 1);

    for i = 1:length(hx)
        Lfh_i = hx(i);
        for j = 1:max_degree
            LgLfh = Lie_derivative(Lfh_i, x_bar, gx_bar, 1);
            if any(simplify(LgLfh) ~= 0)
                ri(i) = j;
                fprintf('  Output %d: relative degree = %d\n', i, ri(i));
                break;
            end
            Lfh_i = Lie_derivative(Lfh_i, x_bar, fx_bar, 1);
            if j == max_degree
                error('Could not determine relative degree for output %d.', i);
            end
        end
    end

    fprintf('Computing decoupling matrix (Lie derivatives)...\n');
    deltax_rows = sym(zeros(length(hx), size(gx_bar, 2)));
    for i = 1:length(hx)
        Lfh_i = hx(i);
        if ri(i) > 1
            Lfh_i = Lie_derivative(Lfh_i, x_bar, fx_bar, ri(i) - 1);
        end
        deltax_rows(i, :) = Lie_derivative(Lfh_i, x_bar, gx_bar, 1);
    end
    deltax = simplify(deltax_rows);

    fprintf('Computing drift terms...\n');
    bx_rows = sym(zeros(length(hx), 1));
    for i = 1:length(hx)
        bx_rows(i) = Lie_derivative(hx(i), x_bar, fx_bar, ri(i));
    end
    bx = simplify(bx_rows);

    fprintf('Checking decoupling matrix invertibility...\n');
    det_delta = simplify(det(deltax));
    if det_delta == 0
        error('Decoupling matrix is singular! Cannot invert.');
    end
    fprintf('  Determinant is non-zero ✓\n');

    fprintf('Computing control laws...\n');
    alphax = simplify(-deltax \ bx);
    betax = simplify(inv(deltax));
    
    fprintf('Control law computation complete!\n');
end