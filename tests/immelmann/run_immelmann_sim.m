function traj = run_immelmann_sim()
% run_immelmann_sim  Run the Immelmann config (FW only) and return a
% rich trajectory struct that the per-property sub-tests inspect.
%
% Cached on the file system as results/results_immelmann_fw.mat so that
% sub-tests rerun in seconds instead of recomputing ode45 every time.
% Pass force=true via env var to force recompute (handled implicitly by
% rebuilding when the config mtime is newer than the cache).

    here = fileparts(mfilename('fullpath'));
    repo = fileparts(fileparts(here));
    cd(repo);
    addpath('models'); addpath('utilities'); addpath('fw_maneuvers'); addpath('trajectory_configs');

    cache = fullfile('results', 'results_immelmann_fw.mat');
    cfg   = fullfile('fw_maneuvers', 'config_immelmann.m');
    if isfile(cache)
        ci = dir(cfg); ca = dir(cache);
        if ca.datenum > ci.datenum
            S = load(cache); traj = S.traj; return;
        end
    end

    run(cfg);
    fw_x0 = fw_initial.x0;
    opts = odeset('RelTol', 1e-5, 'AbsTol', 1e-5);
    [t, fw_state] = ode45(@(t,s) fw_step(t, s, fw_controls, fw_params), t_sim, fw_x0, opts);

    n = length(t);
    pos    = fw_state(:, 1:3);
    v_body = fw_state(:, 4:6);
    qs     = fw_state(:, 7:10);
    omega  = fw_state(:, 11:13);

    alt     = -pos(:,3);
    R33     = zeros(n, 1);
    fwd_w   = zeros(n, 3);
    v_world = zeros(n, 3);
    euler   = zeros(n, 3);
    for k = 1:n
        qa = qs(k,:).'; qa = qa/(norm(qa)+1e-12);
        R          = quat_to_R(qa);
        R33(k)     = R(3,3);
        fwd_w(k,:) = R(:,1).';
        v_world(k,:) = (R * v_body(k,:).').';
        euler(k,:)   = quat_to_zyx_deg(qa);
    end
    euler = unwrap(euler*pi/180, [], 1) * 180/pi;

    p_rate = omega(:,1); q_rate = omega(:,2); r_rate = omega(:,3);
    body_pitch_int = cumtrapz(t, q_rate) * 180/pi;       % deg
    body_roll_int  = cumtrapz(t, p_rate) * 180/pi;

    yaw_proj = atan2(fwd_w(:,2), fwd_w(:,1)) * 180/pi;
    yaw_proj = unwrap(yaw_proj*pi/180) * 180/pi;

    traj.t      = t;
    traj.pos    = pos;
    traj.alt    = alt;
    traj.qs     = qs;
    traj.v_body = v_body;
    traj.v_world = v_world;
    traj.omega  = omega;
    traj.euler  = euler;
    traj.R33    = R33;
    traj.fwd_w  = fwd_w;
    traj.yaw_proj_deg = yaw_proj;
    traj.body_pitch_int_deg = body_pitch_int;
    traj.body_roll_int_deg  = body_roll_int;

    if ~exist('results','dir'), mkdir('results'); end
    save(cache, 'traj');
end

% --------------- helpers ---------------------------------------------------
function R = quat_to_R(q)
    q = q(:)/max(norm(q),1e-12);
    q0=q(1); q1=q(2); q2=q(3); q3=q(4);
    R = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
         2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
         2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];
end

function rpy = quat_to_zyx_deg(q)
    q0=q(1); q1=q(2); q2=q(3); q3=q(4);
    roll  = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));
    sp    = 2*(q0*q2 - q3*q1);
    if abs(sp) >= 1, pitch = sign(sp)*pi/2; else, pitch = asin(sp); end
    yaw   = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2));
    rpy = [roll, pitch, yaw] * 180/pi;
end

function xdot = fw_step(t, s, fw_controls, fw_params)
    if isfield(fw_controls, 't_sim') && isvector(fw_controls.thrust)
        th = interp1(fw_controls.t_sim, fw_controls.thrust,   t, 'linear', 'extrap');
        el = interp1(fw_controls.t_sim, fw_controls.elevator, t, 'linear', 'extrap');
        ai = interp1(fw_controls.t_sim, fw_controls.aileron,  t, 'linear', 'extrap');
        ru = interp1(fw_controls.t_sim, fw_controls.rudder,   t, 'linear', 'extrap');
    else
        th = fw_controls.thrust;
        el = fw_controls.elevator;
        ai = fw_controls.aileron;
        ru = fw_controls.rudder;
    end
    if isfield(fw_controls, 'stabilize_after') && t >= fw_controls.stabilize_after
        target = struct();
        if isfield(fw_controls, 'stabilize_target'), target = fw_controls.stabilize_target; end
        gains = struct();
        if isfield(fw_controls, 'stabilize_gains'),  gains  = fw_controls.stabilize_gains;  end
        alt_ref = [];
        if isfield(fw_controls, 'stabilize_alt_ref'), alt_ref = fw_controls.stabilize_alt_ref; end
        [del, dai, dru] = fw_attitude_hold(s, target, gains, alt_ref);
        el = el + del; ai = ai + dai; ru = ru + dru;
    end
    xdot = fw_6dof_quat(t, s, th, el, ai, ru, fw_params);
end
