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

% Control and external inputs
syms Ax Ay Az real % aerodynamic forces
syms Ap Aq Ar real % aerodynamic moments
syms g real % gravity constant
syms zeta xi real % internal states for extended system (thrust)

%% Drone rotation matrix from quaternions
R = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
     2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
     2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];

%% State vector
x_bar = [x0; y0; z0;                % Position (3)
         q0; q1; q2; q3;            % Drone quaternion (4)
         u; v; w;                   % Velocity (3)
         p; q; r;                   % Drone angular velocity (3)
         zeta; xi];                 % Extended integrators (2)
                                    % Total: 15 states

%% Drone quaternion derivative
q_dot_vec = 0.5 * [-q1, -q2, -q3; q0, -q3, q2; q3, q0, -q1; -q2, q1, q0] * [p; q; r];
q0_dot = q_dot_vec(1);
q1_dot = q_dot_vec(2);
q2_dot = q_dot_vec(3);
q3_dot = q_dot_vec(4);

%% System dynamics
fx_bar = [
    u;                                      % x_dot
    v;                                      % y_dot
    w;                                      % z_dot
    q0_dot;                                 % quaternion
    q1_dot;
    q2_dot;
    q3_dot;
    -Ax/m + R(1,3)*zeta/m;                 % u_dot
    -Ay/m + R(2,3)*zeta/m;                 % v_dot
    -Az/m + g + R(3,3)*zeta/m;             % w_dot
    (Iy - Iz)/Ix*q*r + Ap/Ix;              % p_dot
    (Iz - Ix)/Iy*p*r + Aq/Iy;              % q_dot
    (Ix - Iy)/Iz*p*q + Ar/Iz;              % r_dot
    xi;                                     % zeta_dot
    0];                                     % xi_dot (control input)

%% Control input matrix
gx_bar = [
    zeros(10, 4);                           % Position, quaternion, velocity
    0, 1/Ix,    0,    0;        % p_dot (drone roll)
    0,    0, 1/Iy,    0;        % q_dot (drone pitch)
    0,    0,    0, 1/Iz;        % r_dot (drone yaw)
    zeros(1, 4);                            % zeta
    1,    0,    0,    0];       % xi_dot (thrust control)

%% Full state-space dynamics with control inputs
syms u1 u2 u3 u4 real % Define symbolic control inputs
u_vec = [u1; u2; u3; u4]; % Control vector

% x_dot = f(x) + g(x)*u
x_dot_eqs = fx_bar + gx_bar * u_vec;

fprintf('Full system dynamics (x_dot = f(x) + g(x)*u) have been symbolically defined in x_dot_eqs.\n');


%% Define outputs for feedback linearization
% Using R(2,1) for yaw control. R(2,1) = 2*(q1*q2+q0*q3), which is a
% smooth function of the quaternions related to sin(yaw).
y_out = [
    x0;                 % x position
    y0;                 % y position
    z0;                 % z position
    2*(q1*q2+q0*q3)];    % yaw control via rotation matrix element

hx = y_out;

%% Automatically compute relative degrees
fprintf('Automatically computing relative degrees...\n');
max_degree = 10; % Maximum degree to check to prevent infinite loops
ri = zeros(length(hx), 1);

for i = 1:length(hx)
    Lfh_i = hx(i);
    for j = 1:max_degree
        % Check if the input appears
        LgLfh = Lie_derivative(Lfh_i, x_bar, gx_bar, 1);
        if any(simplify(LgLfh) ~= 0)
            ri(i) = j;
            fprintf('Relative degree for output %d (%s) is %d.\n', i, char(hx(i)), ri(i));
            break;
        end
        % If input does not appear, compute the next Lie derivative w.r.t. f
        Lfh_i = Lie_derivative(Lfh_i, x_bar, fx_bar, 1);
        if j == max_degree
            error('Could not determine relative degree for output %d (%s) within %d iterations.', i, char(hx(i)), max_degree);
        end
    end
end

% Check if the sum of relative degrees equals the effective state dimension
effective_dim = length(x_bar) - 1; % Subtract 1 for the quaternion constraint
if sum(ri) > length(x_bar)
    error('Sum of relative degrees (%d) is greater than the state dimension (%d).', sum(ri), length(x_bar));
elseif sum(ri) ~= effective_dim
    warning('Sum of relative degrees (%d) does not match effective state dimension (%d). This implies the presence of internal dynamics.', sum(ri), effective_dim);
end

fprintf('Computing Lie derivatives for decoupling matrix...\n');
fprintf('This may take several minutes...\n');

% Decoupling matrix (Lg Lf^(ri-1) hx)
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

% Drift terms (Lf^ri hx)
bx_rows = sym(zeros(length(hx), 1));
for i = 1:length(hx)
    bx_rows(i) = Lie_derivative(hx(i), x_bar, fx_bar, ri(i));
end
bx = simplify(bx_rows);

% Check invertibility of the decoupling matrix
fprintf('Checking determinant of the decoupling matrix...\n');
det_deltax = simplify(det(deltax));
fprintf('Symbolic determinant of deltax:\n');
disp(det_deltax);
fprintf('Please inspect the determinant for potential singularities.\n');

fprintf('Computing control laws...\n');

% Linearizing control law: u = alpha(x) + beta(x)*v
% Using pseudo-inverse for robustness, as recommended.
alphax = simplify(-pinv(deltax) * bx);
betax = simplify(pinv(deltax));

%% Generate MATLAB functions
fprintf('Generating MATLAB functions...\n');

matlabFunction(alphax, 'File', 'alpha_func', 'Vars', ...
    {x_bar, Ap, Aq, Ar, Ix, Iy, Iz, zeta, xi, m}, 'Outputs', {'alpha_val'});
matlabFunction(betax, 'File', 'beta_func', 'Vars', ...
    {x_bar, Ap, Aq, Ar, Ix, Iy, Iz, zeta, xi, m}, 'Outputs', {'beta_val'});

fprintf('Done! Generated alpha_func.m and beta_func.m\n');
fprintf('\nState vector size: %d states\n', length(x_bar));
fprintf('Control vector size: %d inputs\n', size(gx_bar, 2));
fprintf('Output vector size: %d outputs\n', length(hx));
fprintf('Relative degrees: [%d, %d, %d, %d]\n', ri);
