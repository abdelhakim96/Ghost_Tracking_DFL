%% controller_generation_v2.m
%% Phase 1 + 5 rebuild of the symbolic DFL.
%%   - 3-axis gimbal: q_G = q_x(phi_g) (x) q_y(theta_g) (x) q_z(psi_g)
%%   - Yaw output replaced from R(2,1) (degenerate at large pitch) to q3
%%     (smooth everywhere; with the drone kept near level yaw it is monotone in yaw)
%%   - 7 outputs, 7 inputs, sum of relative degrees = 4+4+4+2+1+1+1 = 17 = N_eff
%%
%% This script writes alpha_gimbal_func3.m, beta_gimbal_func3.m.

clear all; clc;
addpath('../utilities');
addpath('../models');

%% Symbols
syms x0 y0 z0 real
syms q0 q1 q2 q3 real
syms u v w real
syms p q r real
syms m real
syms Ix Iy Iz real
syms phi_g theta_g psi_g real
syms Ig_x Ig_y Ig_z real          % kept for func signature; first-order gimbal so unused
syms Ax Ay Az real
syms Ap Aq Ar real
syms Ag_p Ag_q Ag_r real
syms g real
syms zeta xi real                 % thrust double-integrator extension

%% Drone rotation matrix
R = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
     2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
     2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];

%% State (18 + 0 = 18 elements -- effective dim = 17 due to quat constraint)
x_bar = [x0; y0; z0;            % 1-3  position
         q0; q1; q2; q3;        % 4-7  drone quaternion
         u; v; w;               % 8-10 world-frame velocity
         p; q; r;               % 11-13 body-frame ang vel
         phi_g; theta_g; psi_g; % 14-16 gimbal angles (3-axis)
         zeta; xi];             % 17-18 thrust double-integrator

q_dot_vec = 0.5 * [-q1,-q2,-q3; q0,-q3,q2; q3,q0,-q1; -q2,q1,q0] * [p; q; r];

fx_bar = [
    u;
    v;
    w;
    q_dot_vec(1);
    q_dot_vec(2);
    q_dot_vec(3);
    q_dot_vec(4);
    -Ax/m + R(1,3)*zeta/m;
    -Ay/m + R(2,3)*zeta/m;
    -Az/m + g + R(3,3)*zeta/m;
    (Iy - Iz)/Ix*q*r + Ap/Ix;
    (Iz - Ix)/Iy*p*r + Aq/Iy;
    (Ix - Iy)/Iz*p*q + Ar/Iz;
    0;       % phi_g_dot — input
    0;       % theta_g_dot — input
    0;       % psi_g_dot — input
    xi;
    0];      % T_ddot — input

%% Input matrix: 7 inputs [u1=T_ddot, u2=tau_phi, u3=tau_theta, u4=tau_psi, u5=dphi_g, u6=dtheta_g, u7=dpsi_g]
gx_bar = sym(zeros(18, 7));
gx_bar(11, 2) = 1/Ix;     % tau_phi    -> p_dot
gx_bar(12, 3) = 1/Iy;     % tau_theta  -> q_dot
gx_bar(13, 4) = 1/Iz;     % tau_psi    -> r_dot
gx_bar(14, 5) = 1;        % dphi_g
gx_bar(15, 6) = 1;        % dtheta_g
gx_bar(16, 7) = 1;        % dpsi_g
gx_bar(18, 1) = 1;        % T_ddot     -> xi_dot

%% Outputs (7)
%   1-3 drone position (= camera position when t_G = 0)
%   4   drone yaw proxy q3 -- smooth everywhere, polynomial in q
%   5-7 gimbal angles -- direct DFL inputs at rel degree 1
y_out = [
    x0;
    y0;
    z0;
    q3;
    phi_g;
    theta_g;
    psi_g];
hx = y_out;

%% Relative degrees
fprintf('Computing relative degrees...\n');
max_degree = 10;
ri = zeros(length(hx), 1);
for i = 1:length(hx)
    Lfh_i = hx(i);
    for j = 1:max_degree
        LgLfh = Lie_derivative(Lfh_i, x_bar, gx_bar, 1);
        if any(simplify(LgLfh) ~= 0)
            ri(i) = j;
            fprintf('  r_%d (%s) = %d\n', i, char(hx(i)), j);
            break;
        end
        Lfh_i = Lie_derivative(Lfh_i, x_bar, fx_bar, 1);
        if j == max_degree
            error('Could not determine relative degree for output %d', i);
        end
    end
end
fprintf('  sum r_i = %d ; effective state dim = %d\n', sum(ri), length(x_bar)-1);

%% Decoupling matrix Delta and drift b
fprintf('Computing decoupling matrix (this can take a few minutes)...\n');
deltax = sym(zeros(length(hx), size(gx_bar, 2)));
for i = 1:length(hx)
    Lfh_i = hx(i);
    if ri(i) > 1
        Lfh_i = Lie_derivative(Lfh_i, x_bar, fx_bar, ri(i)-1);
    end
    deltax(i, :) = Lie_derivative(Lfh_i, x_bar, gx_bar, 1);
end
deltax = simplify(deltax);

fprintf('Computing drift terms b(x)...\n');
bx = sym(zeros(length(hx), 1));
for i = 1:length(hx)
    bx(i) = simplify(Lie_derivative(hx(i), x_bar, fx_bar, ri(i)));
end

fprintf('det(Delta) =\n');
disp(simplify(det(deltax)));

%% Symbolic inversion -> alpha, beta
fprintf('Inverting decoupling matrix...\n');
alphax = simplify(-pinv(deltax) * bx);
betax  = simplify(pinv(deltax));

%% Generate MATLAB functions
fprintf('Writing alpha_gimbal_func3.m, beta_gimbal_func3.m ...\n');
matlabFunction(alphax, 'File', 'alpha_gimbal_func3', 'Vars', ...
    {x_bar, Ap, Aq, Ar, Ag_p, Ag_q, Ag_r, Ix, Iy, Iz, Ig_x, Ig_y, Ig_z, zeta, xi, m}, ...
    'Outputs', {'alpha_val'});
matlabFunction(betax, 'File', 'beta_gimbal_func3', 'Vars', ...
    {x_bar, Ap, Aq, Ar, Ag_p, Ag_q, Ag_r, Ix, Iy, Iz, Ig_x, Ig_y, Ig_z, zeta, xi, m}, ...
    'Outputs', {'beta_val'});

fprintf('Done.\n');
fprintf('  state dim   : %d\n', length(x_bar));
fprintf('  control dim : %d\n', size(gx_bar, 2));
fprintf('  output dim  : %d\n', length(hx));
fprintf('  rel degrees : [%s]\n', strjoin(string(ri), ', '));
