%% Simulation parameters
t_end = 6.0;         % End time of the simulation (s)
delta_t = 0.01;     % Time step for the simulation (s)
t_sim = 0:delta_t:t_end; % Time vector for the simulation

%% Quadrotor parameters
quad_params.m = 0.468;      % Mass of the quadrotor (kg)
quad_params.Ix = 0.0023;    % Moment of inertia around x-axis (kg*m^2)
quad_params.Iy = 0.0023;    % Moment of inertia around y-axis (kg*m^2)
quad_params.Iz = 0.0046;    % Moment of inertia around z-axis (kg*m^2)
quad_params.Ax = 0.25;      % Aerodynamic drag force coefficient along x-axis
quad_params.Ay = 0.25;      % Aerodynamic drag force coefficient along y-axis
quad_params.Az = 0.25;      % Aerodynamic drag force coefficient along z-axis
quad_params.Ap = 0.022;     % Aerodynamic drag moment coefficient around x-axis
quad_params.Aq = 0.022;     % Aerodynamic drag moment coefficient around y-axis
quad_params.Ar = 0.022;     % Aerodynamic drag moment coefficient around z-axis
quad_params.g = 9.81;       % Acceleration due to gravity (m/s^2)
quad_params.l = 0.225;      % Distance from the center of mass to each rotor (m)
quad_params.d = 3e-6;       % Drag factor
quad_params.b = 1e-5;       % Thrust factor

% Gimbal parameters
% (No specific gimbal parameters were listed in the original file)

%% Fixed-wing model parameters for Red Bull Air Race Plane (Edge 540)
fw_params.m = 590; % mass, kg
fw_params.J = diag([550, 750, 1100]); % inertia matrix, kg*m^2 (estimated)
fw_params.S = 9.1; % wing area, m^2
fw_params.b = 7.44; % wingspan, m
fw_params.c = 1.22; % mean aerodynamic chord, m
fw_params.rho = 1.225; % air density, kg/m^3
fw_params.g = 9.81; % gravity, m/s^2

% Aerodynamic Coefficients (representative values for a highly agile aircraft)
fw_params.CL0 = 0.4; fw_params.CL_alpha = 5.7; fw_params.CL_q = 7.0; fw_params.CL_de = -0.8;
fw_params.CD0 = 0.04; fw_params.k = 0.05; fw_params.CDa = 0.1; fw_params.CD_q = 0.0; fw_params.CD_de = 0.0;
fw_params.CY_beta = -1.2; fw_params.CY_p = -0.1; fw_params.CY_r = 0.2; fw_params.CY_da = 0.2; fw_params.CY_dr = -0.2;
fw_params.Cl_beta = -0.15; fw_params.Cl_p = -1.2; fw_params.Cl_r = 0.25; fw_params.Cl_da = 0.5; fw_params.Cl_dr = 0.05;
fw_params.Cm0 = 0.0; fw_params.Cm_alpha = -1.5; fw_params.Cm_q = -15.0; fw_params.Cm_de = -1.8;
fw_params.Cn_beta = 0.15; fw_params.Cn_p = -0.1; fw_params.Cn_r = -0.4; fw_params.Cn_da = 0.04; fw_params.Cn_dr = -0.1;

%% Initial Conditions
% Fixed-wing
fw_initial.u0 = 70; 
fw_initial.v0 = 0; 
fw_initial.w0 = 0;
fw_initial.x0 = [0; 0; -100; fw_initial.u0; fw_initial.v0; fw_initial.w0; 1; 0; 0; 0; 0; 0; 0]; % x, y, z, u, v, w, q0, q1, q2, q3, p, q, r

%% Fixed-wing control inputs
fw_controls.thrust = 390;      % Constant thrust (N)
fw_controls.elevator = -0.5;   % Constant elevator deflection (rad)
fw_controls.aileron = 0.2;       % No roll input
fw_controls.rudder = 0;        % No yaw input

%% DFL Controller Gains
% Position and Yaw Gains
dfl_gains.c0 = 9250.0;  % Position gain
dfl_gains.c1 = 2400.0;  % Velocity gain
dfl_gains.c2 = 350.0;   % Acceleration gain
dfl_gains.c3 = 100.0;    % Jerk gain
dfl_gains.c4 = 10.0;   % Yaw gain
dfl_gains.c5 = 10.00;    % Yaw rate gain

% Gimbal Gains
dfl_gains.c_phi = 50000.0;      % Proportional gain for gimbal roll
dfl_gains.c_theta = 70000.0;    % Proportional gain for gimbal pitch
