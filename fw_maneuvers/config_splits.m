%% config_splits.m — Split-S (half roll inverted + half-loop pull-through)
%
% Sequence:
%   Phase A : aileron pulse for ~180° of roll, ending inverted upright (canopy down)
%   Phase B : elevator pull to half-loop down to right-side-up

%% Sim parameters
t_end   = 3.0;
delta_t = 0.005;
t_sim   = 0:delta_t:t_end;

%% Quadrotor + FW parameters
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
% Start higher so the Split-S has room to descend
fw_initial.x0 = [0; 0; -300; fw_initial.u0; fw_initial.v0; fw_initial.w0; 1; 0; 0; 0; 0; 0; 0];

%% Inputs
fw_controls.t_sim = t_sim;
fw_controls.thrust = 100 * ones(size(t_sim));

% Phase A: half roll  (aileron pulse, same shape as Immelmann's roll)
ail = zeros(size(t_sim));
roll_t0 = 0.20;
roll_t1 = 0.82;
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

% Phase B: half-loop pull-through (starts after roll completes)
%
% Now inverted: elevator -0.18 (nose-up in body frame) pulls the inverted
% aircraft "up" relative to its canopy = "down" in world. After 180° of
% pitch, aircraft is upright again, heading reversed, at lower altitude.
ele = zeros(size(t_sim));
loop_t0 = 1.40;
loop_t1 = 2.30;
ele_amp = -0.18;
ele_ramp = 0.10;
for k = 1:length(t_sim)
    tk = t_sim(k);
    if tk >= loop_t0 - ele_ramp && tk < loop_t0
        ele(k) = ele_amp * 0.5*(1 - cos(pi*(tk - (loop_t0-ele_ramp))/ele_ramp));
    elseif tk >= loop_t0 && tk <= loop_t1
        ele(k) = ele_amp;
    elseif tk > loop_t1 && tk <= loop_t1 + ele_ramp
        ele(k) = ele_amp * 0.5*(1 + cos(pi*(tk - loop_t1)/ele_ramp));
    end
end
fw_controls.elevator = ele;

fw_controls.rudder = zeros(size(t_sim));

%% Quadrotor controller gains
dfl_gains.c0 = 51150.0; dfl_gains.c1 = 51140.0; dfl_gains.c2 = 1150.0; dfl_gains.c3 = 150.0;
dfl_gains.c4 = 1.0;     dfl_gains.c5 = 1.0;
dfl_gains.c_phi   = 50.0;
dfl_gains.c_theta = 50.0;
dfl_gains.c_psig  = 50.0;
dfl_gains.c_q3      = 100.0;
dfl_gains.c_q3_dot  = 20.0;
