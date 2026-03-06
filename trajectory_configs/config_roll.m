%% Configuration: Roll Maneuver
% Fixed-wing performs a full aileron roll while the multicopter + gimbal
% tracks the camera pose using DFL position control + geometric gimbal control.
%
% The drone tracks position and keeps yaw stable (velocity heading).
% The gimbal handles all camera orientation matching during the roll.
%
% CRITICAL: The FW must start in aerodynamic TRIM so that the initial
% NED acceleration is ~zero. Otherwise, the acceleration mismatch drives
% the DFL thrust state (zeta) through zero, triggering the 1/zeta
% singularity in alpha_func.m within ~50 ms.
%
% Trim conditions at V=40 m/s:
%   qbar = 0.5*1.225*40^2 = 980 Pa,  qbar*S = 8918 N
%   CL_trim = W/(qbar*S) = 2845/8918 = 0.319
%   alpha_trim = (CL_trim - CL0)/CL_alpha = -0.0127 rad
%   w0 = V*sin(alpha_trim) = -0.51 m/s  (initial body z-velocity for trim)
%   Cm_trim: de = -(Cm0 + Cm_alpha*alpha)/Cm_de = 0.012 rad
%   CD_trim = CD0 + k*CL^2 = 0.045,  Drag = 402 N  =>  Thrust ~ 400 N
%
% With w0=-0.51: residual z-acceleration ~ 0.04 m/s^2 (vs 2.25 without).
% Drone position and velocity EXACTLY match FW at t=0 (zero initial error).

%% Simulation parameters
t_end = 5.0;          % Long enough for full roll + settling
delta_t = 0.01;       % Time step
t_sim = 0:delta_t:t_end;

%% Quadrotor parameters (same as loop config)
quad_params.m = 0.5;        % Mass (kg)
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
% Fixed-wing: TRIMMED level flight at 40 m/s, altitude 100 m
% Set w0 = V*sin(alpha_trim) so that CL matches weight at t=0.
% This eliminates the initial acceleration mismatch that causes
% the DFL singularity.
fw_initial.u0 = 40;
fw_initial.v0 = 0;
fw_initial.w0 = -0.51;     % Trim alpha: atan2(-0.51, 40) = -0.0127 rad
fw_initial.x0 = [0; 0; -100; fw_initial.u0; fw_initial.v0; fw_initial.w0; 1; 0; 0; 0; 0; 0; 0];

% Drone: EXACTLY co-located with FW (zero initial error!)
% Any position/velocity offset creates enormous snap commands through
% the high DFL gains, driving zeta through zero.
quad_initial.pos = [0; 0; -100];
% FW NED velocity = R_BN * [u0; v0; w0]. With R=I: v_ned = [u0; 0; w0].
% Must match EXACTLY to avoid velocity error * c1 = 23400 transient.
quad_initial.vel = [fw_initial.u0; 0; fw_initial.w0];
quad_initial.angle = [0; 0; 0.0];
quad_initial.ang_vel = [0; 0; 0];
quad_initial.rpm = [0; 0; 0; 0];
quad_initial.relative_angle = [0; 0; 0];

%% Fixed-wing control inputs — Roll maneuver
fw_controls.t_sim = t_sim;

% Thrust to balance drag at trim (~400 N)
fw_controls.thrust = 400 * ones(size(t_sim));

% Roll timing
roll_start = 0.5;     % Allow 0.5s settling before roll
roll_duration = 2.0;  % 2.0s for a full roll (moderate rate ~180 deg/s)
roll_end = roll_start + roll_duration;

% Aileron input: sine pulse for one full roll
aileron_amp = -0.4;
aileron_input = zeros(size(t_sim));
idx = t_sim >= roll_start & t_sim <= roll_end;
aileron_input(idx) = aileron_amp * sin(pi * (t_sim(idx) - roll_start) / roll_duration);
fw_controls.aileron = aileron_input;

% Elevator at trim value (0.012 rad for level flight at 40 m/s)
fw_controls.elevator = 0.012 * ones(size(t_sim));

% No rudder
fw_controls.rudder = zeros(size(t_sim));

%% DFL Controller Gains
% Position channel — same gains as loop config
dfl_gains.c0 = 53250.0;   % Position error
dfl_gains.c1 = 23400.0;   % Velocity error
dfl_gains.c2 = 550.0;     % Acceleration error
dfl_gains.c3 = 100.0;     % Jerk error

% Yaw channel
dfl_gains.c4 = 1.0;       % R(2,1) error
dfl_gains.c5 = 0.0;       % R(2,1) rate

% Gimbal SO(3) controller gains
dfl_gains.kp_R_gimbal = 10;
dfl_gains.kp_omega_gimbal = 3;
