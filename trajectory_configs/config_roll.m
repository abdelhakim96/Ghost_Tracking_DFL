%% Configuration: Roll Maneuver
% Fixed-wing performs a full aileron roll while the multicopter + gimbal
% tracks the camera pose using DFL position control + geometric gimbal control.
%
% The drone tracks position and keeps yaw stable (velocity heading).
% The gimbal handles all camera orientation matching during the roll.

%% Simulation parameters
t_end = 3.0;          % Long enough for full roll + settling
delta_t = 0.01;       % Time step (match loop config)
t_sim = 0:delta_t:t_end;

%% Quadrotor parameters (same as loop config — proven to work)
quad_params.m = 0.5;        % Mass (kg) — light, agile drone
quad_params.Ix = 0.0023;    % Moment of inertia x (kg*m^2)
quad_params.Iy = 0.0023;    % Moment of inertia y (kg*m^2)
quad_params.Iz = 0.0046;    % Moment of inertia z (kg*m^2)
quad_params.Ax = 0.25;
quad_params.Ay = 0.25;
quad_params.Az = 0.25;
quad_params.Ap = 0.022;
quad_params.Aq = 0.022;
quad_params.Ar = 0.022;
quad_params.g = 9.81;
quad_params.l = 0.225;
quad_params.d = 3e-6;
quad_params.b = 1e-5;

%% Fixed-wing model parameters (same as loop config)
fw_params.m = 290;
fw_params.J = diag([550, 750, 1100]);
fw_params.S = 9.1;
fw_params.b = 7.44;
fw_params.c = 1.22;
fw_params.rho = 1.225;
fw_params.g = 9.81;

% Aerodynamic Coefficients
fw_params.CL0 = 0.4; fw_params.CL_alpha = 5.7; fw_params.CL_q = 7.0; fw_params.CL_de = -0.8;
fw_params.CD0 = 0.04; fw_params.k = 0.05; fw_params.CDa = 0.1; fw_params.CD_q = 0.0; fw_params.CD_de = 0.0;
fw_params.CY_beta = -1.2; fw_params.CY_p = -0.1; fw_params.CY_r = 0.2; fw_params.CY_da = 0.2; fw_params.CY_dr = -0.2;
fw_params.Cl_beta = -0.15; fw_params.Cl_p = -1.0; fw_params.Cl_r = 0.25; fw_params.Cl_da = 0.5; fw_params.Cl_dr = 0.05;
fw_params.Cm0 = 0.0; fw_params.Cm_alpha = -1.5; fw_params.Cm_q = -15.0; fw_params.Cm_de = -1.8;
fw_params.Cn_beta = 0.15; fw_params.Cn_p = -0.1; fw_params.Cn_r = -0.4; fw_params.Cn_da = 0.04; fw_params.Cn_dr = -0.1;

%% Initial Conditions
% Fixed-wing: straight level flight at 80 m/s, altitude 100 m
fw_initial.u0 = 80;
fw_initial.v0 = 0;
fw_initial.w0 = 0;
fw_initial.x0 = [0; 0; -100; fw_initial.u0; fw_initial.v0; fw_initial.w0; 1; 0; 0; 0; 0; 0; 0];

% Drone: co-located with fixed-wing (small offset), matching velocity
quad_initial.pos = [-1.0; 0; -99.9];
quad_initial.vel = [fw_initial.u0; 0; 0];
quad_initial.angle = [0; 0; 0.0];
quad_initial.ang_vel = [0; 0; 0];
quad_initial.rpm = [0; 0; 0; 0];
quad_initial.relative_angle = [0; 0; 0];

%% Fixed-wing control inputs — Roll maneuver
fw_controls.t_sim = t_sim;

% Constant thrust to maintain speed
fw_controls.thrust = 340 * ones(size(t_sim));

% Roll timing
roll_start = 0.5;     % Allow 0.5s settling before roll
roll_duration = 1.5;  % 1.5s for a full roll (moderate rate ≈ 240 deg/s)
roll_end = roll_start + roll_duration;

% Aileron input: sine pulse for one full roll
aileron_amp = -0.5;
aileron_input = zeros(size(t_sim));
idx = t_sim >= roll_start & t_sim <= roll_end;
aileron_input(idx) = aileron_amp * sin(pi * (t_sim(idx) - roll_start) / roll_duration);
fw_controls.aileron = aileron_input;

% Slight elevator to maintain altitude
fw_controls.elevator = -0.25 * ones(size(t_sim));  % Same as loop config

% No rudder
fw_controls.rudder = zeros(size(t_sim));

%% DFL Controller Gains (matched to loop config structure)
% Position channel — same high gains as loop (proven stable)
dfl_gains.c0 = 53250.0;   % Position error
dfl_gains.c1 = 23400.0;   % Velocity error
dfl_gains.c2 = 550.0;     % Acceleration error
dfl_gains.c3 = 100.0;     % Jerk error

% Yaw channel — low gains, just stabilize heading
% (gimbal handles all FW orientation tracking)
dfl_gains.c4 = 1.0;       % R(2,1) error (same as loop)
dfl_gains.c5 = 0.0;       % R(2,1) rate (same as loop)

% Gimbal SO(3) controller gains (same as loop config)
dfl_gains.kp_R_gimbal = 5;
dfl_gains.kp_omega_gimbal = 1;
