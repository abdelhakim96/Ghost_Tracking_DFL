%% config_barrelroll.m — proper helical barrel roll
%
% Approach:
%   1. Pre-roll pitch-up to load the wing (small elevator pull) so the
%      maneuver climbs slightly going in.
%   2. Aileron pulse drives the roll. Roll rate ~ Cl_da * delta_a * (V/b).
%   3. Hold elevator through the roll so the lift vector keeps tracing a
%      helix instead of just dropping the nose when inverted.
%   4. Wash out aileron and elevator at the end.
%
% All inputs are open-loop. Tune amplitudes/timings by re-running
% test_barrelroll.m and watching the metrics it prints.

%% Sim parameters
t_end   = 2.5;
delta_t = 0.005;
t_sim   = 0:delta_t:t_end;

%% Quadrotor parameters (needed if this config is also used through the full stack)
quad_params.m  = 0.468;
quad_params.Ix = 0.0023; quad_params.Iy = 0.0023; quad_params.Iz = 0.0046;
quad_params.Ax = 0.25;   quad_params.Ay = 0.25;   quad_params.Az = 0.25;
quad_params.Ap = 0.022;  quad_params.Aq = 0.022;  quad_params.Ar = 0.022;
quad_params.g  = 9.81;
quad_params.l  = 0.225;  quad_params.d  = 3e-6;   quad_params.b  = 1e-5;

%% FW parameters (Edge 540, same as config_roll)
fw_params.m = 290;
fw_params.J = diag([550, 750, 1100]);
fw_params.S = 9.1;  fw_params.b = 7.44;  fw_params.c = 1.22;
fw_params.rho = 1.225;  fw_params.g = 9.81;
fw_params.CL0 = 0.4; fw_params.CL_alpha = 5.7; fw_params.CL_q = 7.0;  fw_params.CL_de = -0.8;
fw_params.CD0 = 0.04; fw_params.k = 0.05; fw_params.CDa = 0.1; fw_params.CD_q = 0.0; fw_params.CD_de = 0.0;
fw_params.CY_beta = -1.2; fw_params.CY_p = -0.1; fw_params.CY_r = 0.2;  fw_params.CY_da = 0.2; fw_params.CY_dr = -0.2;
fw_params.Cl_beta = -0.15; fw_params.Cl_p = -1.0; fw_params.Cl_r = 0.25; fw_params.Cl_da = 0.5; fw_params.Cl_dr = 0.05;
fw_params.Cm0 = 0.0; fw_params.Cm_alpha = -1.5; fw_params.Cm_q = -15.0; fw_params.Cm_de = -1.8;
fw_params.Cn_beta = 0.15; fw_params.Cn_p = -0.1; fw_params.Cn_r = -0.4; fw_params.Cn_da = 0.04; fw_params.Cn_dr = -0.1;

%% IC: 120 m/s level forward, 100 m altitude
fw_initial.u0 = 120; fw_initial.v0 = 0; fw_initial.w0 = 0;
fw_initial.x0 = [0; 0; -100; fw_initial.u0; fw_initial.v0; fw_initial.w0; 1; 0; 0; 0; 0; 0; 0];

%% Inputs
fw_controls.t_sim = t_sim;

% Constant thrust to maintain energy through the roll
fw_controls.thrust = 100 * ones(size(t_sim));

% --- Aileron: one full roll
% Constant aileron deflection for the duration of the roll (open-loop spin
% rate); sine pulse is smoother than a step but here we use a flat segment
% with raised-cosine ramps so the roll rate stays roughly constant.
ail = zeros(size(t_sim));
roll_t0 = 0.30;   roll_t1 = 1.80;        % aileron-on window
ramp    = 0.10;                          % ramp in/out duration
ail_amp = 0.28;                          % rad — slightly higher to clear 360°
for k = 1:length(t_sim)
    tk = t_sim(k);
    if tk >= roll_t0 - ramp && tk < roll_t0
        ail(k) = ail_amp * 0.5 * (1 - cos(pi * (tk - (roll_t0 - ramp)) / ramp));
    elseif tk >= roll_t0 && tk <= roll_t1
        ail(k) = ail_amp;
    elseif tk > roll_t1 && tk <= roll_t1 + ramp
        ail(k) = ail_amp * 0.5 * (1 + cos(pi * (tk - roll_t1) / ramp));
    end
end
fw_controls.aileron = ail;

% --- Elevator: constant small pull-up during the roll, washed in/out.
% This is what makes it a *barrel* roll instead of an aileron roll — the
% sustained AoA keeps the lift vector tracing a helix.
el = zeros(size(t_sim));
ele_t0 = 0.10;  ele_t1 = 2.10;
% Positive elevator = nose-down. The Edge 540's trim lift is much larger
% than weight at 120 m/s, so the FW naturally climbs without any
% deflection. A small positive (push) elevator keeps the maneuver level.
ele_amp = +0.010;
ele_ramp = 0.10;
for k = 1:length(t_sim)
    tk = t_sim(k);
    if tk >= ele_t0 - ele_ramp && tk < ele_t0
        el(k) = ele_amp * 0.5 * (1 - cos(pi * (tk - (ele_t0 - ele_ramp)) / ele_ramp));
    elseif tk >= ele_t0 && tk <= ele_t1
        el(k) = ele_amp;
    elseif tk > ele_t1 && tk <= ele_t1 + ele_ramp
        el(k) = ele_amp * 0.5 * (1 + cos(pi * (tk - ele_t1) / ele_ramp));
    end
end
fw_controls.elevator = el;

% --- Rudder: none for the basic barrel roll
fw_controls.rudder = zeros(size(t_sim));

%% Quadrotor controller gains (loaded only if this config is used end-to-end)
dfl_gains.c0 = 51150.0; dfl_gains.c1 = 51140.0; dfl_gains.c2 = 1150.0; dfl_gains.c3 = 150.0;
dfl_gains.c4 = 1.0;     dfl_gains.c5 = 1.0;
dfl_gains.c_phi   = 50.0;
dfl_gains.c_theta = 50.0;
dfl_gains.c_psig  = 50.0;
dfl_gains.c_q3      = 100.0;
dfl_gains.c_q3_dot  = 20.0;
