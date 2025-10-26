%% Simulation parameters
t_end = 1.65;         % End time of the simulation (s)
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

%% Fixed-wing model parameters
fw_params.m = 13.5;         % mass, kg
fw_params.J = diag([0.8244, 1.135, 1.759]); % inertia matrix, kg*m^2
fw_params.S = 0.55;         % wing area, m^2
fw_params.b = 2.8956;       % wingspan, m
fw_params.c = 0.18994;      % mean aerodynamic chord, m
fw_params.rho = 1.225;      % air density, kg/m^3
fw_params.g = 9.81;         % gravity, m/s^2

% Aerodynamic Coefficients
fw_params.CL0 = 0.28; fw_params.CL_alpha = 3.45; fw_params.CL_q = 0.0; fw_params.CL_de = -0.36;
fw_params.CD0 = 0.03; fw_params.k = 0.04; fw_params.CDa = 0.0; fw_params.CD_q = 0.0; fw_params.CD_de = 0.0;
fw_params.CY_beta = -0.98; fw_params.CY_p = 0.0; fw_params.CY_r = 0.0; fw_params.CY_da = 0.0; fw_params.CY_dr = -0.17;
fw_params.Cl_beta = -0.12; fw_params.Cl_p = -0.1; fw_params.Cl_r = 0.14; fw_params.Cl_da = 0.2; fw_params.Cl_dr = 0.105;
fw_params.Cm0 = -0.023; fw_params.Cm_alpha = -0.38; fw_params.Cm_q = -3.6; fw_params.Cm_de = -0.5;
fw_params.Cn_beta = 0.25; fw_params.Cn_p = 0.022; fw_params.Cn_r = -0.35; fw_params.Cn_da = 0.0; fw_params.Cn_dr = -0.032;

%% Initial Conditions
% Fixed-wing
fw_initial.u0 = 100;
fw_initial.v0 = 0;
fw_initial.w0 = 0;
fw_initial.x0 = [0; 0; -100; fw_initial.u0; fw_initial.v0; fw_initial.w0; 1; 0; 0; 0; 0; 0; 0]; % x, y, z, u, v, w, q0, q1, q2, q3, p, q, r

%% Fixed-wing control inputs - Immelmann Turn Maneuver
fw_controls.t_sim = t_sim; % Pass time vector for interpolation

% Immelmann Turn maneuver timing
loop_start_time = 0.5;      % Start time for the half-loop (s)
loop_duration = 2.2;        % Duration of the half-loop (s)
loop_end_time = loop_start_time + loop_duration;

roll_start_time = loop_start_time + 0.8; % Start roll earlier, during the loop
roll_duration = 0.05;        % Very short duration for snap roll (s)
roll_end_time = roll_start_time + roll_duration;

% Thrust profile - increase significantly during the maneuver
thrust_input = 360 * ones(size(t_sim));
maneuver_indices = t_sim >= loop_start_time & t_sim <= roll_end_time;
thrust_input(maneuver_indices) = 175; % Higher thrust to maintain energy
% Maintain higher thrust for longer to prevent drop
sustain_indices = t_sim > roll_end_time & t_sim <= roll_end_time + 1.0;
thrust_input(sustain_indices) = 65;
fw_controls.thrust = thrust_input;

% Elevator input - Pull up for half-loop, reduce during roll, then IMMEDIATE pitch up after roll
elevator_amplitude = -0.35; % Negative for pull-up (rad)
elevator_input = zeros(size(t_sim));
loop_indices = t_sim >= loop_start_time & t_sim <= loop_end_time;
% Smooth pull-up
elevator_input(loop_indices) = elevator_amplitude;
% Reduce elevator during roll to allow rotation
during_roll = t_sim >= roll_start_time & t_sim <= roll_end_time;
elevator_input(during_roll) = -0.15; % Less elevator during roll
% IMMEDIATE strong pitch up right after roll completes
pitch_up_start = roll_end_time;
pitch_up_end = roll_end_time + 0.3; % 0.3 second strong pitch up
pitch_up_indices = t_sim >= pitch_up_start & t_sim <= pitch_up_end;
elevator_input(pitch_up_indices) = 0.2; % Strong pitch up immediately after roll
% Maintain some elevator after pitch up to prevent drop
recovery_indices = t_sim > pitch_up_end & t_sim <= pitch_up_end + 1.0;
elevator_input(recovery_indices) = -0.1; % Small up elevator for recovery
fw_controls.elevator = elevator_input;

% Aileron input - Very aggressive half roll
aileron_amplitude = 1.1; % High amplitude for snap roll (rad)
aileron_input = zeros(size(t_sim));
roll_indices = t_sim >= roll_start_time & t_sim <= roll_end_time;
% Smooth roll using sine pulse for 180 degrees
aileron_input(roll_indices) = aileron_amplitude * sin(pi * (t_sim(roll_indices) - roll_start_time) / roll_duration);
fw_controls.aileron = aileron_input;

% Rudder - minimal input for coordination
fw_controls.rudder = zeros(size(t_sim));

%% DFL Controller Gains
% Position and Yaw Gains
dfl_gains.c0 = 99250.0;  % Position gain
dfl_gains.c1 = 82400.0;  % Velocity gain
dfl_gains.c2 = 850.0;    % Acceleration gain
dfl_gains.c3 = 100.0;    % Jerk gain
dfl_gains.c4 = 1.0;     % Yaw gain
dfl_gains.c5 = 1.00;    % Yaw rate gain

% Gimbal Gains
dfl_gains.c_phi = 90000.0;   % Proportional gain for gimbal roll
dfl_gains.c_theta = 90000.0; % Proportional gain for gimbal pitch