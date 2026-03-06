% CONTROLLER_GENERATION  Symbolic derivation of DFL alpha/beta functions.
%
% This script uses the Symbolic Math Toolbox to derive the feedback
% linearization control law for the multicopter with dynamic thrust
% extension (double integrator on thrust, eq. 16 in the paper).
%
% Extended system (eq. 16):
%   State: x_hat = [p_M; q_M; v_M; omega_M; zeta; xi]  (15 states)
%   Inputs: u_hat = [ddot_T; tau_phi; tau_theta; tau_psi] (4 inputs)
%   Outputs: y = [x; y; z; R(2,1)]                       (4 outputs)
%
% The relative degrees are r = [4, 4, 4, 2], sum = 14 = 15 - 1
% (one constraint from unit-norm quaternion).
%
% The script computes:
%   Delta(x) = decoupling matrix (eq. 14)
%   b(x)     = drift terms (eq. 15)
%   alpha(x) = -Delta^{-1} * b(x)
%   beta(x)  = Delta^{-1}
%
% and generates alpha_func.m and beta_func.m as optimized MATLAB functions.
%
% NOTE: Gravity constant g does not appear in the generated functions
% because it is constant and vanishes after repeated differentiation
% (the Lie derivatives eliminate constant terms at orders > 2).
% The generated alpha/beta are therefore valid regardless of the gravity
% sign convention used here.

clear all
clc

addpath('../utilities');
addpath('../models');

%% Define symbolic variables

% Drone states
syms x0 y0 z0 real % NED position of the multicopter
syms q0 q1 q2 q3 real % scalar-first quaternion (body to world)
syms u v w real % world-frame velocity of the multicopter
syms p q r real % body-frame angular velocity [roll rate, pitch rate, yaw rate]
syms m real % mass of the multicopter

% Inertia
syms Ix Iy Iz real % principal moments of inertia

% External forces/moments (passed as parameters for generality)
syms Ax Ay Az real % external force components (set to 0 in practice)
syms Ap Aq Ar real % external moment components (set to 0 in practice)

% Gravity and extended states
syms g real % gravitational acceleration constant
syms zeta xi real % DFL extended states: zeta = thrust, xi = thrust rate

%% Rotation matrix from body to world (quaternion)
% Standard scalar-first quaternion rotation matrix R(q) in SO(3).
% R maps vectors from body frame to world frame: v_world = R * v_body
R = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
     2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
     2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];

%% State vector (15 states)
% Corresponds to the extended system in eq. (16), where the thrust T
% has been replaced by a double integrator: zeta_dot = xi, xi_dot = ddot_T.
x_bar = [x0; y0; z0;                % Position in world frame (3)
         q0; q1; q2; q3;            % Quaternion body-to-world (4)
         u; v; w;                    % Velocity in world frame (3)
         p; q; r;                    % Angular velocity in body frame (3)
         zeta; xi];                  % Extended thrust states (2)
                                     % Total: 15 states

%% Quaternion kinematics: q_dot = 0.5 * G(q) * omega
% G(q) is the 4x3 quaternion kinematics matrix (scalar-first convention).
q_dot_vec = 0.5 * [-q1, -q2, -q3; q0, -q3, q2; q3, q0, -q1; -q2, q1, q0] * [p; q; r];
q0_dot = q_dot_vec(1);
q1_dot = q_dot_vec(2);
q2_dot = q_dot_vec(3);
q3_dot = q_dot_vec(4);

%% System drift dynamics f(x)
% This is f_M from eq. (16). The velocity derivatives are in the world
% frame: v_dot = R*[0;0;zeta]/m - [0;0;g] (matching the actual dynamics
% in quadrotor_dynamics_realtime.m).
%
% Note: The gravity sign convention is v_dot_z = R(3,3)*zeta/m - g,
% consistent with thrust along body +z and gravity subtracted.
% Since g is constant, it vanishes from the Lie derivatives at orders
% >= 3 and does not affect the generated alpha/beta functions.
fx_bar = [
    u;                                      % x_dot = v_x (world frame)
    v;                                      % y_dot = v_y
    w;                                      % z_dot = v_z
    q0_dot;                                 % quaternion kinematics
    q1_dot;
    q2_dot;
    q3_dot;
    -Ax/m + R(1,3)*zeta/m;                 % u_dot = F_x/m (world frame)
    -Ay/m + R(2,3)*zeta/m;                 % v_dot = F_y/m
    -Az/m - g + R(3,3)*zeta/m;             % w_dot = F_z/m - g
    (Iy - Iz)/Ix*q*r + Ap/Ix;              % p_dot (Euler equation)
    (Iz - Ix)/Iy*p*r + Aq/Iy;              % q_dot (Euler equation)
    (Ix - Iy)/Iz*p*q + Ar/Iz;              % r_dot (Euler equation)
    xi;                                     % zeta_dot = xi
    0];                                     % xi_dot = 0 (control input enters here)

%% Control input matrix g(x)
% The 4 control inputs are: [ddot_T, tau_phi, tau_theta, tau_psi]
% ddot_T enters the xi_dot equation (double integrator on thrust).
% tau_phi, tau_theta, tau_psi enter the angular acceleration equations.
gx_bar = [
    zeros(10, 4);                           % Position, quaternion, velocity unaffected
    0, 1/Ix,    0,    0;                    % p_dot += tau_phi / Ix
    0,    0, 1/Iy,    0;                    % q_dot += tau_theta / Iy
    0,    0,    0, 1/Iz;                    % r_dot += tau_psi / Iz
    zeros(1, 4);                            % zeta unaffected by inputs
    1,    0,    0,    0];                   % xi_dot = ddot_T (thrust 2nd deriv)

%% Full state-space dynamics
syms u1 u2 u3 u4 real
u_vec = [u1; u2; u3; u4];
x_dot_eqs = fx_bar + gx_bar * u_vec;
fprintf('System dynamics symbolically defined: x_dot = f(x) + g(x)*u\n');

%% Define outputs for feedback linearization
% Output y = [x, y, z, R(2,1)] — 4 outputs.
%
% Position outputs (x, y, z) have relative degree 4 because:
%   y -> v -> a -> j -> snap (4 differentiations until input ddot_T appears).
%
% The 4th output R(2,1) = 2*(q1*q2 + q0*q3) = sin(yaw)*cos(pitch).
% This is a smooth, singularity-free proxy for yaw control using
% quaternion-based rotation matrix elements. Its relative degree is 2
% because body-rate torques appear after 2 differentiations.
y_out = [
    x0;                 % x position (relative degree 4)
    y0;                 % y position (relative degree 4)
    z0;                 % z position (relative degree 4)
    2*(q1*q2+q0*q3)];  % R(2,1) = sin(yaw)*cos(pitch) (relative degree 2)

hx = y_out;

%% Automatically compute relative degrees
fprintf('Automatically computing relative degrees...\n');
max_degree = 10;
ri = zeros(length(hx), 1);

for i = 1:length(hx)
    Lfh_i = hx(i);
    for j = 1:max_degree
        % Check if any control input appears in the current Lie derivative
        LgLfh = Lie_derivative(Lfh_i, x_bar, gx_bar, 1);
        if any(simplify(LgLfh) ~= 0)
            ri(i) = j;
            fprintf('  Output %d: r_%d = %d\n', i, i, ri(i));
            break;
        end
        % Input doesn't appear yet — differentiate again along f
        Lfh_i = Lie_derivative(Lfh_i, x_bar, fx_bar, 1);
        if j == max_degree
            error('Could not determine relative degree for output %d within %d iterations.', i, max_degree);
        end
    end
end

% Verify the necessary condition for exact linearization (eq. 17):
% sum(r_i) = N (effective state dimension)
effective_dim = length(x_bar) - 1; % -1 for quaternion unit-norm constraint
fprintf('Sum of relative degrees: %d, Effective state dim: %d\n', sum(ri), effective_dim);
if sum(ri) > length(x_bar)
    error('Sum of relative degrees (%d) exceeds state dimension (%d).', sum(ri), length(x_bar));
elseif sum(ri) ~= effective_dim
    warning('Sum(r_i) = %d != %d. Internal dynamics present.', sum(ri), effective_dim);
end

%% Compute decoupling matrix Delta(x) and drift b(x)
fprintf('Computing decoupling matrix (this may take several minutes)...\n');

% Delta(x) = [L_g L_f^{r_i-1} h_i]  (eq. 14)
% Each row i: differentiate output h_i along f (r_i-1) times, then along g.
deltax_rows = sym(zeros(length(hx), size(gx_bar, 2)));
for i = 1:length(hx)
    Lfh_i = hx(i);
    if ri(i) > 1
        Lfh_i = Lie_derivative(Lfh_i, x_bar, fx_bar, ri(i) - 1);
    end
    deltax_rows(i, :) = Lie_derivative(Lfh_i, x_bar, gx_bar, 1);
end
deltax = simplify(deltax_rows);

fprintf('Computing drift terms b(x)...\n');

% b(x) = [L_f^{r_i} h_i]  (eq. 15)
% Each element: differentiate output h_i along f exactly r_i times.
bx_rows = sym(zeros(length(hx), 1));
for i = 1:length(hx)
    bx_rows(i) = Lie_derivative(hx(i), x_bar, fx_bar, ri(i));
end
bx = simplify(bx_rows);

% Check invertibility
fprintf('Checking determinant of Delta(x)...\n');
det_deltax = simplify(det(deltax));
fprintf('det(Delta) = '); disp(det_deltax);
fprintf('Singularities occur where this expression is zero.\n');

%% Compute control law: u = alpha(x) + beta(x)*v  (eq. 18)
fprintf('Computing alpha(x) and beta(x)...\n');

% alpha(x) = -Delta^{-1} * b(x)  — cancels nonlinear drift
% beta(x)  = Delta^{-1}          — decouples input-output channels
% Using pseudo-inverse for numerical robustness near singularities.
alphax = simplify(-pinv(deltax) * bx);
betax = simplify(pinv(deltax));

%% Generate optimized MATLAB functions
fprintf('Generating alpha_func.m and beta_func.m...\n');

matlabFunction(alphax, 'File', 'alpha_func', 'Vars', ...
    {x_bar, Ap, Aq, Ar, Ix, Iy, Iz, zeta, xi, m}, 'Outputs', {'alpha_val'});
matlabFunction(betax, 'File', 'beta_func', 'Vars', ...
    {x_bar, Ap, Aq, Ar, Ix, Iy, Iz, zeta, xi, m}, 'Outputs', {'beta_val'});

fprintf('\nDone! Generated alpha_func.m and beta_func.m\n');
fprintf('State vector:  %d states\n', length(x_bar));
fprintf('Control inputs: %d\n', size(gx_bar, 2));
fprintf('Outputs:        %d\n', length(hx));
fprintf('Relative degrees: [%s]\n', num2str(ri'));
fprintf('Sum(r_i) = %d\n', sum(ri));
