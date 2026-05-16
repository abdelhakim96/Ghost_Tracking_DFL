function out = run_compare3(config_to_run, tag)
% run_compare3  Runner for the 3-axis (Phase 1+5) controller.
    if nargin < 2, tag = config_to_run; end

    here = fileparts(mfilename('fullpath'));
    cd(here);
    addpath('DFL_controller'); addpath('models'); addpath('utilities'); addpath('trajectory_configs'); addpath('fw_maneuvers');

    cpath = fullfile('trajectory_configs', ['config_' config_to_run '.m']);
    if ~isfile(cpath)
        cpath = fullfile('fw_maneuvers', ['config_' config_to_run '.m']);
    end
    run(cpath);

    % Augment dfl_gains with the new q3/psi_g gains if missing
    if ~isfield(dfl_gains, 'c_q3'),     dfl_gains.c_q3     = 100; end
    if ~isfield(dfl_gains, 'c_q3_dot'), dfl_gains.c_q3_dot = 20;  end
    if ~isfield(dfl_gains, 'c_psig'),   dfl_gains.c_psig   = 50;  end

    global m Ix Iy Iz g Ax Ay Az Ap Aq Ar
    m = quad_params.m; Ix = quad_params.Ix; Iy = quad_params.Iy; Iz = quad_params.Iz;
    g = quad_params.g; Ax = quad_params.Ax; Ay = quad_params.Ay; Az = quad_params.Az;
    Ap = quad_params.Ap; Aq = quad_params.Aq; Ar = quad_params.Ar;

    fw_x0 = fw_initial.x0;
    x0_quad = fw_x0(1) - 0.01; y0_quad = fw_x0(2) - 0.001; z0_quad = fw_x0(3) - 0.001;
    q0_quad=1; q1_quad=0; q2_quad=0; q3_quad=0;
    u0_quad = fw_initial.u0; v0_quad = fw_initial.v0; w0_quad = fw_initial.w0;
    p_quad=0; q_quad=0; r_quad=0;
    zeta = quad_params.m*quad_params.g; xi = 0;
    phi_g=0; theta_g=0; psi_g=0;
    quad_initial_state = [x0_quad;y0_quad;z0_quad; q0_quad;q1_quad;q2_quad;q3_quad; ...
                          u0_quad;v0_quad;w0_quad; p_quad;q_quad;r_quad; ...
                          phi_g; theta_g; psi_g; zeta; xi];
    initial_state = [quad_initial_state; fw_x0];

    clear unified_dynamics3 quadrotor_dynamics_realtime3 dfl_controller3
    opts = odeset('RelTol',1e-4,'AbsTol',1e-4);
    try
        [t, state] = ode45(@(t,s) unified_dynamics3(t,s,fw_params,fw_controls,dfl_gains), ...
                           t_sim, initial_state, opts);
        diverged = false;
    catch ME
        warning('run_compare3:diverged','ode45 failed: %s', ME.message);
        t = []; state = []; diverged = true;
    end

    out.tag = tag; out.config = config_to_run; out.t = t; out.state = state; out.diverged = diverged;
    if ~diverged && ~isempty(state)
        out.metrics = compute_metrics3(t, state);
    else
        out.metrics = struct('mae_pos',NaN,'mae_orient_deg',NaN,'max_pos_err',NaN, ...
                             'max_orient_err_deg',NaN,'min_costheta_g',NaN,'final_time',NaN);
    end
    if ~exist('results', 'dir'), mkdir('results'); end
    save(fullfile('results', ['results_' tag '.mat']), 'out');
    fprintf('[run_compare3] %s done. final t=%.3fs, MAE pos=%.4fm, MAE orient=%.3f deg, min|cos(theta_g)|=%.4f\n', ...
        tag, out.metrics.final_time, out.metrics.mae_pos, out.metrics.mae_orient_deg, out.metrics.min_costheta_g);
end

function M = compute_metrics3(t, state)
    quad = state(:,1:18);
    fw   = state(:,19:31);
    pos_err = quad(:,1:3) - fw(:,1:3);
    M.mae_pos     = mean(vecnorm(pos_err,2,2));
    M.max_pos_err = max(vecnorm(pos_err,2,2));

    geo_err = zeros(size(quad,1),1);
    costh   = zeros(size(quad,1),1);
    for k = 1:size(quad,1)
        qM = quad(k,4:7).';  qM = qM/norm(qM+1e-12);
        phi_k=quad(k,14); theta_k=quad(k,15); psi_k=quad(k,16);
        qGx = [cos(phi_k/2); sin(phi_k/2); 0; 0];
        qGy = [cos(theta_k/2); 0; sin(theta_k/2); 0];
        qGz = [cos(psi_k/2); 0; 0; sin(psi_k/2)];
        qG  = quat_mul(quat_mul(qGx,qGy),qGz);
        qC  = quat_mul(qM,qG);
        qA  = fw(k,7:10).'; qA = qA/norm(qA+1e-12);
        qErr = quat_mul(quat_conj(qC), qA); qErr = qErr/norm(qErr+1e-12);
        geo_err(k) = 2*acos(min(1,abs(qErr(1))));
        costh(k) = abs(cos(theta_k));
    end
    M.mae_orient_deg = mean(geo_err)*180/pi;
    M.max_orient_err_deg = max(geo_err)*180/pi;
    M.min_costheta_g = min(costh);
    M.final_time = t(end);
    M.geo_err_deg = geo_err*180/pi;
    M.pos_err_mag = vecnorm(pos_err,2,2);
    M.costheta_g  = costh;
end
