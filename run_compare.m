function out = run_compare(config_to_run, tag)
% run_compare  Run a config headlessly, save state + metrics, return a struct.
%   config_to_run : 'loop' | 'roll' | 'straight'
%   tag           : string used in the saved .mat filename

    if nargin < 2, tag = config_to_run; end

    here = fileparts(mfilename('fullpath'));
    cd(here);
    addpath('DFL_controller'); addpath('models'); addpath('utilities'); addpath('trajectory_configs');

    run(fullfile('trajectory_configs', ['config_' config_to_run '.m']));

    global m Ix Iy Iz g Ax Ay Az Ap Aq Ar
    m = quad_params.m;  Ix = quad_params.Ix; Iy = quad_params.Iy; Iz = quad_params.Iz;
    g = quad_params.g;  Ax = quad_params.Ax; Ay = quad_params.Ay; Az = quad_params.Az;
    Ap = quad_params.Ap; Aq = quad_params.Aq; Ar = quad_params.Ar;

    fw_x0 = fw_initial.x0;
    x0_quad = fw_x0(1) - 0.01; y0_quad = fw_x0(2) - 0.001; z0_quad = fw_x0(3) - 0.001;
    q0_quad=1; q1_quad=0; q2_quad=0; q3_quad=0;
    u0_quad = fw_initial.u0; v0_quad = fw_initial.v0; w0_quad = fw_initial.w0;
    p_quad=0; q_quad=0; r_quad=0;
    zeta = quad_params.m*quad_params.g; xi = 0;
    phi_g=0; theta_g=0;
    quad_initial_state = [x0_quad;y0_quad;z0_quad;q0_quad;q1_quad;q2_quad;q3_quad; ...
                          u0_quad;v0_quad;w0_quad;p_quad;q_quad;r_quad; ...
                          phi_g;theta_g;zeta;xi];
    initial_state = [quad_initial_state; fw_x0];

    clear unified_dynamics quadrotor_dynamics_realtime dfl_controller
    opts = odeset('RelTol',1e-4,'AbsTol',1e-4);
    try
        [t, state] = ode45(@(t,s) unified_dynamics(t,s,fw_params,fw_controls,dfl_gains), ...
                           t_sim, initial_state, opts);
        diverged = false;
    catch ME
        warning('run_compare:diverged','ode45 failed: %s', ME.message);
        t = []; state = []; diverged = true;
    end

    out.tag = tag;
    out.config = config_to_run;
    out.t = t;
    out.state = state;
    out.diverged = diverged;

    if ~diverged && ~isempty(state)
        out.metrics = compute_metrics(t, state);
    else
        out.metrics = struct('mae_pos', NaN, 'mae_orient_deg', NaN, ...
                             'max_pos_err', NaN, 'max_orient_err_deg', NaN, ...
                             'min_cosphig_costhetag', NaN, 'final_time', NaN);
    end

    save(['results_' tag '.mat'], 'out');
    fprintf('[run_compare] %s done. Final t=%.3fs, MAE pos=%.4fm, MAE orient=%.3f deg, min |cos(phi_g)cos(theta_g)|=%.4f\n', ...
        tag, out.metrics.final_time, out.metrics.mae_pos, out.metrics.mae_orient_deg, out.metrics.min_cosphig_costhetag);
end

function M = compute_metrics(t, state)
    quad = state(:,1:17);
    fw   = state(:,18:30);

    % --- Position MAE between drone CoM and FW CoM (both are camera if lever arms are 0)
    pos_err = quad(:,1:3) - fw(:,1:3);
    M.mae_pos     = mean(vecnorm(pos_err,2,2));
    M.max_pos_err = max(vecnorm(pos_err,2,2));

    % --- Orientation: composed camera quaternion q_cam = q_M (x) q_G  vs  q_A (FW)
    geo_err = zeros(size(quad,1),1);
    cosc    = zeros(size(quad,1),1);
    for k = 1:size(quad,1)
        qM = quad(k,4:7).';  qM = qM/norm(qM+1e-12);
        phi_g_k = quad(k,14); theta_g_k = quad(k,15);
        % Paper convention: q_G = q_x(phi_g) (x) q_y(theta_g)
        qGx = [cos(phi_g_k/2); sin(phi_g_k/2); 0; 0];
        qGy = [cos(theta_g_k/2); 0; sin(theta_g_k/2); 0];
        qG  = quatmul(qGx,qGy);
        qC  = quatmul(qM,qG);
        qA  = fw(k,7:10).'; qA = qA/norm(qA+1e-12);
        qErr = quatmul(quatconj(qC), qA);
        qErr = qErr/norm(qErr+1e-12);
        geo_err(k) = 2*acos(min(1,abs(qErr(1))));   % geodesic angle on SO(3)
        cosc(k) = abs(cos(phi_g_k)*cos(theta_g_k));
    end
    M.mae_orient_deg     = mean(geo_err)*180/pi;
    M.max_orient_err_deg = max(geo_err)*180/pi;
    M.min_cosphig_costhetag = min(cosc);
    M.final_time = t(end);
    M.geo_err_deg = geo_err*180/pi;
    M.pos_err_mag = vecnorm(pos_err,2,2);
    M.cosphi_costheta = cosc;
end

function q = quatmul(a,b)
    a0=a(1);a1=a(2);a2=a(3);a3=a(4);
    b0=b(1);b1=b(2);b2=b(3);b3=b(4);
    q = [a0*b0 - a1*b1 - a2*b2 - a3*b3;
         a0*b1 + a1*b0 + a2*b3 - a3*b2;
         a0*b2 - a1*b3 + a2*b0 + a3*b1;
         a0*b3 + a1*b2 - a2*b1 + a3*b0];
end

function qc = quatconj(q)
    qc = [q(1); -q(2); -q(3); -q(4)];
end
