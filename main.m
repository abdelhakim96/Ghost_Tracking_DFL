%% 
clear all;
clc;

% Select the configuration file to use
% 'loop', 'roll', or 'straight'
config_to_run = 'roll';

run(['trajectory_configs/config_' config_to_run '.m']);

addpath('DFL_controller');
addpath('models');
addpath('utilities');
addpath('trajectory_configs');

%% Define global variables for quadrotor controller
global m Ix Iy Iz g Ax Ay Az Ap Aq Ar
m = quad_params.m;
Ix = quad_params.Ix;
Iy = quad_params.Iy;
Iz = quad_params.Iz;
g = quad_params.g;
Ax = quad_params.Ax;
Ay = quad_params.Ay;
Az = quad_params.Az;
Ap = quad_params.Ap;
Aq = quad_params.Aq;
Ar = quad_params.Ar;

%% Initial Conditions
% Fixed-wing
fw_x0 = fw_initial.x0;

% Quadrotor
x0_quad = quad_initial.pos(1); y0_quad = quad_initial.pos(2); z0_quad = quad_initial.pos(3);
u0_quad = quad_initial.vel(1); v0_quad = quad_initial.vel(2); w0_quad = quad_initial.vel(3);
p_quad = quad_initial.ang_vel(1); q_quad = quad_initial.ang_vel(2); r_quad = quad_initial.ang_vel(3);

% Convert initial Euler angles to quaternions
phi = quad_initial.angle(1);
theta = quad_initial.angle(2);
psi = quad_initial.angle(3);
q0_quad = cos(psi/2)*cos(theta/2)*cos(phi/2) + sin(psi/2)*sin(theta/2)*sin(phi/2);
q1_quad = cos(psi/2)*cos(theta/2)*sin(phi/2) - sin(psi/2)*sin(theta/2)*cos(phi/2);
q2_quad = cos(psi/2)*sin(theta/2)*cos(phi/2) + sin(psi/2)*cos(theta/2)*sin(phi/2);
q3_quad = sin(psi/2)*cos(theta/2)*cos(phi/2) - cos(psi/2)*sin(theta/2)*sin(phi/2);

zeta = quad_params.m*quad_params.g; xi = 0;
phi_g = quad_initial.relative_angle(1); theta_g = quad_initial.relative_angle(2);
% phi_g_dot and theta_g_dot are no longer states
quad_initial_state = [x0_quad; y0_quad; z0_quad; q0_quad; q1_quad; q2_quad; q3_quad; u0_quad; v0_quad; w0_quad; p_quad; q_quad; r_quad; phi_g; theta_g; zeta; xi];

% Combined state vector
initial_state = [quad_initial_state; fw_x0];

%% Simulation
clear unified_dynamics; % Clear persistent variables
clear quadrotor_dynamics_realtime; % Clear persistent variables
ode_options = odeset('RelTol', 1e-4, 'AbsTol', 1e-4);
[t, state] = ode45(@(t,s) unified_dynamics(t, s, fw_params, fw_controls, dfl_gains), t_sim, initial_state, ode_options);

%% Post-processing and Plotting
run('utilities/plot_results.m');
