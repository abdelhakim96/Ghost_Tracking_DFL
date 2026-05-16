%% config_immelmann.m — Immelmann turn (half-loop + half-roll)
%
% Sequence:
%   Phase A : sustained nose-up elevator -> half loop, ending inverted.
%   Phase B : aileron pulse -> half roll, ending right-side-up.
%   The phases are sequenced — A finishes before B starts.

%% Sim parameters
% Wider loop + recovery + level-out time so the heading-reversed leg is
% visible. Lower elevator amplitude makes the loop big and recognizable
% instead of a near-vertical sprint.
t_end   = 7.5;            % maneuver finishes ~t=4s, ~3.5s coast for autopilot to settle and show heading-reversed leg
delta_t = 0.005;
t_sim   = 0:delta_t:t_end;

%% Quadrotor and FW parameters (Edge 540, same family as the other configs)
quad_params.m  = 0.468;
quad_params.Ix = 0.0023; quad_params.Iy = 0.0023; quad_params.Iz = 0.0046;
quad_params.Ax = 0.25;   quad_params.Ay = 0.25;   quad_params.Az = 0.25;
quad_params.Ap = 0.022;  quad_params.Aq = 0.022;  quad_params.Ar = 0.022;
quad_params.g  = 9.81;
quad_params.l  = 0.225;  quad_params.d  = 3e-6;   quad_params.b  = 1e-5;

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

fw_initial.u0 = 120; fw_initial.v0 = 0; fw_initial.w0 = 0;
fw_initial.x0 = [0; 0; -100; fw_initial.u0; fw_initial.v0; fw_initial.w0; 1; 0; 0; 0; 0; 0; 0];

%% Inputs
fw_controls.t_sim = t_sim;
fw_controls.thrust = 100 * ones(size(t_sim));

% Phase A: pull up through 180° of pitch (half loop)
% Body-pitch rate ~ Cm_de * elevator * qbar*S*c / Iy. With elevator = -0.4,
% qbar = 8820, expect q ≈ -0.4 * -1.8 * 8820 * 9.1 * 1.22 / 750 ≈ 95 rad/s^2 (!)
% — way too aggressive. Reduce to -0.2 to get pitch rate ~ 1.5 rad/s
% Wider loop with elevator amp tuned so the integrated body-pitch lands
% right at 180 deg (a half-loop). Previously -0.06 for 2.35 s gave 209 deg
% of body-pitch -- too much overshoot. Shorter pulse compensates.
ele = zeros(size(t_sim));
loop_t0 = 0.30;
loop_t1 = 2.20;
ele_amp = -0.06;
ele_ramp = 0.15;
% Phase D: small positive (push-down) elevator after the half-roll, to
% counter the Edge 540's natural excess lift and keep the FW level on the
% heading-reversed leg. Without this the trajectory continues climbing
% after the maneuver and never visibly "flies back".
D_t0 = 3.80;
D_amp = +0.010;

for k = 1:length(t_sim)
    tk = t_sim(k);
    if tk >= loop_t0 - ele_ramp && tk < loop_t0
        ele(k) = ele_amp * 0.5*(1 - cos(pi*(tk - (loop_t0-ele_ramp))/ele_ramp));
    elseif tk >= loop_t0 && tk <= loop_t1
        ele(k) = ele_amp;
    elseif tk > loop_t1 && tk <= loop_t1 + ele_ramp
        ele(k) = ele_amp * 0.5*(1 + cos(pi*(tk - loop_t1)/ele_ramp));
    end
    % Phase D trim (additive; window doesn't overlap with Phase A)
    if tk >= D_t0 - ele_ramp && tk < D_t0
        ele(k) = ele(k) + D_amp * 0.5*(1 - cos(pi*(tk - (D_t0-ele_ramp))/ele_ramp));
    elseif tk >= D_t0
        ele(k) = ele(k) + D_amp;
    end
end
fw_controls.elevator = ele;

% Phase B: half roll (180°) starting after the half-loop completes
% Half-roll after the half-loop completes. Previous pulse (0.9 s at 0.30
% rad) only delivered ~162 deg of body-roll because the FW had decelerated
% during the loop and roll effectiveness (~V) is reduced. Bump duration to
% 1.05 s to reach a full 180 deg.
ail = zeros(size(t_sim));
roll_t0 = 2.45;
roll_t1 = 3.55;
ail_amp = 0.30;
ail_ramp = 0.10;
for k = 1:length(t_sim)
    tk = t_sim(k);
    if tk >= roll_t0 - ail_ramp && tk < roll_t0
        ail(k) = ail_amp * 0.5*(1 - cos(pi*(tk - (roll_t0-ail_ramp))/ail_ramp));
    elseif tk >= roll_t0 && tk <= roll_t1
        ail(k) = ail_amp;
    elseif tk > roll_t1 && tk <= roll_t1 + ail_ramp
        ail(k) = ail_amp * 0.5*(1 + cos(pi*(tk - roll_t1)/ail_ramp));
    end
end
fw_controls.aileron = ail;

fw_controls.rudder = zeros(size(t_sim));

% --- Attitude-hold autopilot (Option A: small PD on roll/pitch, rate damper
%     on yaw). Engages just after the scripted half-roll completes, holds
%     the FW level on the heading-reversed leg.
fw_controls.stabilize_after = 3.65;             % half-roll ends at ~3.55+ramp
fw_controls.stabilize_target.phi_ref   = 0;     % wings level
fw_controls.stabilize_target.theta_ref = 0;     % nose level (overridden by altitude hold)
fw_controls.stabilize_target.r_ref     = 0;     % no yaw rate
% Altitude hold target. After the half-loop dive the FW naturally arrives
% around +50-100 m above start altitude — target that, not the peak.
fw_controls.stabilize_alt_ref = 150;
% gains use defaults from fw_attitude_hold.m

%% Quadrotor controller gains
dfl_gains.c0 = 51150.0; dfl_gains.c1 = 51140.0; dfl_gains.c2 = 1150.0; dfl_gains.c3 = 150.0;
dfl_gains.c4 = 1.0;     dfl_gains.c5 = 1.0;
dfl_gains.c_phi   = 50.0;
dfl_gains.c_theta = 50.0;
dfl_gains.c_psig  = 50.0;
dfl_gains.c_q3      = 100.0;
dfl_gains.c_q3_dot  = 20.0;
