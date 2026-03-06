%% Configuration: Roll Maneuver
% Fixed-wing performs a full aileron roll while the multicopter + gimbal
% tracks the camera pose using DFL position control + geometric gimbal control.

%% Simulation parameters
t_end = 2.5;         % End time — long enough for full roll + settling
delta_t = 0.005;     % Time step for the simulation (s)
t_sim = 0:delta_t:t_end;

%% Quadrotor parameters
quad_params.m = 2.0;        % Mass of the quadrotor (kg) — realistic for camera drone
quad_params.Ix = 0.015;     % Moment of inertia around x-axis (kg*m^2)
quad_params.Iy = 0.015;     % Moment of inertia around y-axis (kg*m^2)
quad_params.Iz = 0.025;     % Moment of inertia around z-axis (kg*m^2)
quad_params.Ax = 0.25;      % Aerodynamic drag coefficient x
quad_params.Ay = 0.25;      % Aerodynamic drag coefficient y
quad_params.Az = 0.25;      % Aerodynamic drag coefficient z
quad_params.Ap = 0.022;     % Aerodynamic drag moment coefficient x
quad_params.Aq = 0.022;     % Aerodynamic drag moment coefficient y
quad_params.Ar = 0.022;     % Aerodynamic drag moment coefficient z
quad_params.g = 9.81;       % Gravity (m/s^2)
quad_params.l = 0.225;      % Rotor arm length (m)
quad_params.d = 3e-6;       % Drag factor
quad_params.b = 1e-5;       % Thrust factor

%% Fixed-wing model parameters (Edge 540 — Red Bull Air Race)
fw_params.m = 290;    % mass, kg
fw_params.J = diag([550, 750, 1100]); % inertia matrix, kg*m^2
fw_params.S = 9.1;    % wing area, m^2
fw_params.b = 7.44;   % wingspan, m
fw_params.c = 1.22;   % mean aerodynamic chord, m
fw_params.rho = 1.225; % air density, kg/m^3
fw_params.g = 9.81;   % gravity, m/s^2

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

% Drone: co-located with fixed-wing, matching velocity
quad_initial.pos = [-0.5; 0; -100.0];
quad_initial.vel = [fw_initial.u0; 0; 0];
quad_initial.angle = [0; 0; 0.0];
quad_initial.ang_vel = [0; 0; 0];
quad_initial.rpm = [0; 0; 0; 0];
quad_initial.relative_angle = [0; 0; 0];

%% Fixed-wing control inputs — Roll maneuver
fw_controls.t_sim = t_sim;

% Constant thrust to maintain speed
fw_controls.thrust = 340 * ones(size(t_sim));

% Roll maneuver timing
roll_start_time = 0.3;   % Allow 0.3s for controller to settle
roll_duration = 1.5;     % 1.5s for a full roll (moderate roll rate)
roll_end_time = roll_start_time + roll_duration;

% Aileron input: sine pulse for one full roll
aileron_amplitude = -0.5;  % rad — moderate deflection
aileron_input = zeros(size(t_sim));
roll_indices = t_sim >= roll_start_time & t_sim <= roll_end_time;
aileron_input(roll_indices) = aileron_amplitude * sin(pi * (t_sim(roll_indices) - roll_start_time) / roll_duration);
fw_controls.aileron = aileron_input;

% Small elevator to maintain altitude during the roll
elevator_amplitude = -0.08;  % rad — slight nose-up
elevator_input = elevator_amplitude * ones(size(t_sim));
fw_controls.elevator = elevator_input;

% No rudder
fw_controls.rudder = zeros(size(t_sim));

%% DFL Controller Gains
% Position channel: 4th-order error dynamics
% Characteristic polynomial: s^4 + c3*s^3 + c2*s^2 + c1*s + c0
% For a well-damped response, place poles at -w*[1, 1, 1, 1]:
%   w ≈ 12 rad/s gives c0 = w^4 = 20736, c1 = 4w^3 = 6912,
%   c2 = 6w^2 = 864, c3 = 4w = 48
dfl_gains.c0 = 20000.0;   % Position error gain (≈ w^4)
dfl_gains.c1 = 7000.0;    % Velocity error gain (≈ 4w^3)
dfl_gains.c2 = 850.0;     % Acceleration error gain (≈ 6w^2)
dfl_gains.c3 = 50.0;      % Jerk error gain (≈ 4w)

% Yaw channel (R(2,1)): 2nd-order error dynamics
% s^2 + c5*s + c4
dfl_gains.c4 = 20.0;      % R(2,1) error gain
dfl_gains.c5 = 8.0;       % R(2,1) rate error gain

% Gimbal SO(3) controller gains
dfl_gains.kp_R_gimbal = 50.0;       % Orientation error gain
dfl_gains.kp_omega_gimbal = 10.0;   % Velocity damping gain
