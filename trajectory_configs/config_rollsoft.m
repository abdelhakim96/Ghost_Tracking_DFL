%% Reduced-amplitude rolling maneuver (stays inside 2-axis reachable set)
%% Goal: validate the corrected controller end-to-end without provoking
%%       the gimbal singularity at +-pi/2.

t_end   = 1.5;
delta_t = 0.01;
t_sim   = 0:delta_t:t_end;

%% Quadrotor parameters (same as roll)
quad_params.m = 0.468;
quad_params.Ix = 0.0023; quad_params.Iy = 0.0023; quad_params.Iz = 0.0046;
quad_params.Ax = 0.25;   quad_params.Ay = 0.25;   quad_params.Az = 0.25;
quad_params.Ap = 0.022;  quad_params.Aq = 0.022;  quad_params.Ar = 0.022;
quad_params.g = 9.81;
quad_params.l = 0.225;   quad_params.d = 3e-6;    quad_params.b = 1e-5;

%% Fixed-wing model parameters (same as roll)
fw_params.m = 290;
fw_params.J = diag([550, 750, 1100]);
fw_params.S = 9.1;  fw_params.b = 7.44;  fw_params.c = 1.22;
fw_params.rho = 1.225;  fw_params.g = 9.81;
fw_params.CL0 = 0.4; fw_params.CL_alpha = 5.7; fw_params.CL_q = 7.0; fw_params.CL_de = -0.8;
fw_params.CD0 = 0.04; fw_params.k = 0.05; fw_params.CDa = 0.1; fw_params.CD_q = 0.0; fw_params.CD_de = 0.0;
fw_params.CY_beta = -1.2; fw_params.CY_p = -0.1; fw_params.CY_r = 0.2; fw_params.CY_da = 0.2; fw_params.CY_dr = -0.2;
fw_params.Cl_beta = -0.15; fw_params.Cl_p = -1.0; fw_params.Cl_r = 0.25; fw_params.Cl_da = 0.5; fw_params.Cl_dr = 0.05;
fw_params.Cm0 = 0.0; fw_params.Cm_alpha = -1.5; fw_params.Cm_q = -15.0; fw_params.Cm_de = -1.8;
fw_params.Cn_beta = 0.15; fw_params.Cn_p = -0.1; fw_params.Cn_r = -0.4; fw_params.Cn_da = 0.04; fw_params.Cn_dr = -0.1;

%% Initial conditions
fw_initial.u0 = 120; fw_initial.v0 = 0; fw_initial.w0 = 0;
fw_initial.x0 = [0; 0; -100; fw_initial.u0; fw_initial.v0; fw_initial.w0; 1; 0; 0; 0; 0; 0; 0];

%% Fixed-wing controls — reduced aileron amplitude so peak roll stays well below 90 deg
fw_controls.t_sim = t_sim;
fw_controls.thrust = 100 * ones(size(t_sim));

roll_start = 0.2; roll_dur = 0.6;
ail_amp = 0.15;          % was 0.6 in config_roll
ele_amp = 0.02;          % was 0.04

ail = zeros(size(t_sim));
ele = zeros(size(t_sim));
idx = t_sim >= roll_start & t_sim <= (roll_start + roll_dur);
ail(idx) = ail_amp * sin(pi * (t_sim(idx) - roll_start) / roll_dur);
ele(idx) = ele_amp * sin(pi * (t_sim(idx) - roll_start) / roll_dur);

fw_controls.aileron  = ail;
fw_controls.elevator = ele;
fw_controls.rudder   = zeros(size(t_sim));

%% DFL gains
dfl_gains.c0 = 51150.0;
dfl_gains.c1 = 51140.0;
dfl_gains.c2 = 1150.0;
dfl_gains.c3 = 150.0;
dfl_gains.c4 = 1.0;     % NOTE: yaw output 2*(q1*q2+q0*q3) is degenerate at large pitch
dfl_gains.c5 = 1.0;     % Higher gains destabilize. Phase 1 (proper yaw output) needed.
dfl_gains.c_phi   = 50.0;
dfl_gains.c_theta = 50.0;
