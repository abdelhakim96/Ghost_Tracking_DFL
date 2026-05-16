%% config_cuban8.m — half Cuban-8 (loop + roll + recover)

%% Sim parameters
t_end   = 5.0;
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
fw_initial.x0 = [0; 0; -200; fw_initial.u0; fw_initial.v0; fw_initial.w0; 1; 0; 0; 0; 0; 0; 0];

%% Inputs
fw_controls.t_sim = t_sim;
fw_controls.thrust = 100 * ones(size(t_sim));

% Phase A: 5/8 loop  (~225 deg body-pitch).  Half loop was 180 deg in ~0.9s
% pulse (Immelmann), so 5/8 needs ~0.9 * 225/180 = 1.125s pulse.
ele = zeros(size(t_sim));
ele_amp = -0.18;
ele_ramp = 0.10;
A_t0 = 0.20;
A_t1 = 1.50;

% Phase C: 1/8 loop (~45 deg body-pitch).  Pulse ~0.30s (the FW slows during
% the 5/8 loop so elevator effectiveness drops, pulse needs to be a bit longer
% than the linear scaling suggests).
C_t0 = 3.20;
C_t1 = 3.55;

for k = 1:length(t_sim)
    tk = t_sim(k);
    % Phase A elevator
    if tk >= A_t0 - ele_ramp && tk < A_t0
        ele(k) = ele_amp * 0.5*(1 - cos(pi*(tk - (A_t0-ele_ramp))/ele_ramp));
    elseif tk >= A_t0 && tk <= A_t1
        ele(k) = ele_amp;
    elseif tk > A_t1 && tk <= A_t1 + ele_ramp
        ele(k) = ele_amp * 0.5*(1 + cos(pi*(tk - A_t1)/ele_ramp));
    end
    % Phase C elevator (additive; the windows don't overlap)
    if tk >= C_t0 - ele_ramp && tk < C_t0
        ele(k) = ele(k) + ele_amp * 0.5*(1 - cos(pi*(tk - (C_t0-ele_ramp))/ele_ramp));
    elseif tk >= C_t0 && tk <= C_t1
        ele(k) = ele(k) + ele_amp;
    elseif tk > C_t1 && tk <= C_t1 + ele_ramp
        ele(k) = ele(k) + ele_amp * 0.5*(1 + cos(pi*(tk - C_t1)/ele_ramp));
    end
end
fw_controls.elevator = ele;

% Phase B: half-roll between phases A and C.  Aileron pulse ~0.65s for 180 deg.
ail = zeros(size(t_sim));
B_t0 = 1.85;
B_t1 = 2.74;
ail_amp = 0.30;
ail_ramp = 0.10;
for k = 1:length(t_sim)
    tk = t_sim(k);
    if tk >= B_t0 - ail_ramp && tk < B_t0
        ail(k) = ail_amp * 0.5*(1 - cos(pi*(tk - (B_t0-ail_ramp))/ail_ramp));
    elseif tk >= B_t0 && tk <= B_t1
        ail(k) = ail_amp;
    elseif tk > B_t1 && tk <= B_t1 + ail_ramp
        ail(k) = ail_amp * 0.5*(1 + cos(pi*(tk - B_t1)/ail_ramp));
    end
end
fw_controls.aileron = ail;

fw_controls.rudder = zeros(size(t_sim));

%% Quadrotor controller gains
dfl_gains.c0 = 51150.0; dfl_gains.c1 = 51140.0; dfl_gains.c2 = 1150.0; dfl_gains.c3 = 150.0;
dfl_gains.c4 = 1.0;     dfl_gains.c5 = 1.0;
dfl_gains.c_phi   = 50.0;
dfl_gains.c_theta = 50.0;
dfl_gains.c_psig  = 50.0;
dfl_gains.c_q3      = 100.0;
dfl_gains.c_q3_dot  = 20.0;
