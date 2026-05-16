function [pass, summary, traj] = test_fw_maneuver(config_name, checks, save_prefix)
% test_fw_maneuver  Run the fixed-wing alone on a maneuver config and
%                   verify it does what the maneuver says it should do.
%
%   config_name : 'barrelroll' | 'immelmann' | 'splits' | 'cuban8' | ...
%                 (must resolve to trajectory_configs/config_<name>.m)
%   checks      : struct of expected criteria for the maneuver
%                  .label                : human-readable title
%                  .roll_change_deg      : [min,max] expected total roll change
%                  .pitch_change_deg     : [min,max] expected total pitch sweep
%                  .heading_change_deg   : [min,max] expected heading change
%                  .alt_change_m         : [min,max] (final-initial), negative=descent
%                  .alt_range_max_m      : max |alt-initial_alt| during run
%                  any field set to [] is skipped.
%   save_prefix : prefix for plots in plots/fw_maneuvers/
%
%   Outputs:
%     pass    : logical, true iff all assertions pass
%     summary : struct of measured numbers (for diagnostics)
%     traj    : struct with t, state, euler (Nx3 deg), pos (Nx3)
%
% Side effects:
%     plots/fw_maneuvers/<save_prefix>_3d.png
%     plots/fw_maneuvers/<save_prefix>_euler.png
%     plots/fw_maneuvers/<save_prefix>_position.png
%     plots/fw_maneuvers/<save_prefix>_inputs.png

    if nargin < 3, save_prefix = config_name; end

    here = fileparts(mfilename('fullpath'));
    repo = fileparts(here);
    cd(repo);
    addpath('models'); addpath('utilities'); addpath('trajectory_configs');
    addpath(fullfile(repo, 'fw_maneuvers'));

    % Load config (must define t_sim, fw_params, fw_controls, fw_initial)
    config_path = locate_config(config_name);
    run(config_path);

    fw_x0 = fw_initial.x0;
    opts = odeset('RelTol', 1e-5, 'AbsTol', 1e-5);
    [t, fw_state] = ode45(@(t,s) fw_step(t, s, fw_controls, fw_params), t_sim, fw_x0, opts);

    n = length(t);
    pos = fw_state(:, 1:3);
    euler = zeros(n, 3);
    R33   = zeros(n, 1);                 % world-Z projection of body-Z: 1=upright, -1=inverted
    fwd_w = zeros(n, 3);                 % body-x in world
    p_rate = fw_state(:, 11);
    q_rate = fw_state(:, 12);
    r_rate = fw_state(:, 13);
    for k = 1:n
        qa = fw_state(k, 7:10).'; qa = qa/(norm(qa)+1e-12);
        euler(k,:) = quat_to_zyx_euler_deg(qa);
        R          = quat_to_R(qa);
        R33(k)     = R(3,3);
        fwd_w(k,:) = R(:,1).';
    end
    euler = unwrap(euler*pi/180, [], 1) * 180/pi;

    % --- Heading via forward-axis projection: robust at any pitch
    yaw_proj = atan2(fwd_w(:,2), fwd_w(:,1)) * 180/pi;
    yaw_proj = unwrap(yaw_proj*pi/180) * 180/pi;

    % --- Measured numbers ----------------------------------------------
    summary = struct();
    summary.t_end_s         = t(end);
    summary.roll_change_deg     = euler(end,1) - euler(1,1);
    summary.pitch_change_deg    = euler(end,2) - euler(1,2);
    summary.heading_change_deg  = wrap180(euler(end,3) - euler(1,3));
    summary.heading_change_proj_deg = wrap180(yaw_proj(end) - yaw_proj(1));
    summary.pitch_min_deg       = min(euler(:,2));
    summary.pitch_max_deg       = max(euler(:,2));
    summary.pitch_sweep_deg     = summary.pitch_max_deg - summary.pitch_min_deg;
    summary.body_pitch_rotation_deg = abs(trapz(t, q_rate)) * 180/pi;   % integrated |q|
    summary.body_roll_rotation_deg  = abs(trapz(t, p_rate)) * 180/pi;
    summary.body_yaw_rotation_deg   = abs(trapz(t, r_rate)) * 180/pi;
    summary.min_R33             = min(R33);     % how inverted did it get? (-1 = fully inverted)
    summary.max_R33             = max(R33);
    summary.final_R33           = R33(end);     % +1 upright, -1 inverted at end
    summary.alt_initial_m       = -pos(1,3);
    summary.alt_final_m         = -pos(end,3);
    summary.alt_change_m        = summary.alt_final_m - summary.alt_initial_m;
    summary.alt_max_dev_m       = max(abs(-pos(:,3) - summary.alt_initial_m));
    summary.xy_distance_m       = norm(pos(end,1:2) - pos(1,1:2));

    % --- Assertions ----------------------------------------------------
    pass = true;
    fails = {};
    pass = check_range(pass, fails, 'roll_change_deg',    summary.roll_change_deg,   checks, 'roll_change_deg');
    pass = check_range(pass, fails, 'pitch_change_deg',   summary.pitch_change_deg,  checks, 'pitch_change_deg');
    pass = check_range(pass, fails, 'pitch_sweep_deg',    summary.pitch_sweep_deg,   checks, 'pitch_sweep_deg');
    pass = check_range(pass, fails, 'heading_change_deg', summary.heading_change_deg, checks, 'heading_change_deg');
    pass = check_range(pass, fails, 'alt_change_m',       summary.alt_change_m,      checks, 'alt_change_m');
    pass = check_upper(pass, fails, 'alt_max_dev_m',      summary.alt_max_dev_m,     checks, 'alt_range_max_m');
    pass = check_range(pass, fails, 'heading_change_proj_deg', summary.heading_change_proj_deg, checks, 'heading_change_proj_deg');
    % Magnitude check (for sign-agnostic heading reversal)
    if isfield(checks, 'abs_heading_change_proj_deg') && ~isempty(checks.abs_heading_change_proj_deg)
        lo = checks.abs_heading_change_proj_deg(1); hi = checks.abs_heading_change_proj_deg(2);
        a = abs(summary.heading_change_proj_deg);
        if a < lo || a > hi
            pass = false;
            fails{end+1} = sprintf('|heading_change_proj_deg| = %.3f, expected in [%.3f, %.3f]', a, lo, hi);
        end
    end
    pass = check_range(pass, fails, 'body_pitch_rotation_deg', summary.body_pitch_rotation_deg, checks, 'body_pitch_rotation_deg');
    pass = check_range(pass, fails, 'body_roll_rotation_deg',  summary.body_roll_rotation_deg,  checks, 'body_roll_rotation_deg');
    pass = check_range(pass, fails, 'final_R33',          summary.final_R33,         checks, 'final_R33');
    pass = check_range(pass, fails, 'min_R33',            summary.min_R33,           checks, 'min_R33');
    pass = check_range(pass, fails, 'max_R33',            summary.max_R33,           checks, 'max_R33');

    fprintf('\n=== test_fw_maneuver  [%s] ===\n', checks.label);
    flist = fieldnames(summary);
    for i = 1:numel(flist), fprintf('  %-22s : %10.3f\n', flist{i}, summary.(flist{i})); end
    if pass
        fprintf('  RESULT : PASS\n');
    else
        fprintf('  RESULT : FAIL\n');
        for i = 1:numel(fails), fprintf('    - %s\n', fails{i}); end
    end

    traj.t = t; traj.state = fw_state; traj.euler = euler; traj.pos = pos;

    % --- Plots ---------------------------------------------------------
    out_dir = fullfile(repo, 'plots', 'fw_maneuvers');
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    fig = figure('Name', ['FW ' config_name], 'NumberTitle','off', 'Position', [80 80 1100 700]);
    titles = {'Roll  \phi  (deg)', 'Pitch  \theta  (deg)', 'Yaw  \psi  (deg)'};
    for ax = 1:3
        subplot(3,1,ax); grid on;
        plot(t, euler(:,ax), 'r-', 'LineWidth', 1.5);
        ylabel(titles{ax});
        if ax == 1, title(sprintf('%s  -  FW Euler angles', checks.label)); end
        if ax == 3, xlabel('t (s)'); end
    end
    saveas(fig, fullfile(out_dir, [save_prefix '_euler.png']));

    fig2 = figure('Name', ['FW pos ' config_name], 'NumberTitle','off', 'Position', [100 100 1100 700]);
    pn = {'x  North  (m)', 'y  East  (m)', 'Altitude  -z  (m)'};
    pv = {pos(:,1), pos(:,2), -pos(:,3)};
    for ax = 1:3
        subplot(3,1,ax); grid on;
        plot(t, pv{ax}, 'r-', 'LineWidth', 1.5);
        ylabel(pn{ax});
        if ax == 1, title(sprintf('%s  -  FW position', checks.label)); end
        if ax == 3, xlabel('t (s)'); end
    end
    saveas(fig2, fullfile(out_dir, [save_prefix '_position.png']));

    fig3 = figure('Name', ['FW 3D ' config_name], 'NumberTitle','off', 'Position', [120 120 900 700]);
    plot3(pos(:,1), pos(:,2), -pos(:,3), 'r-', 'LineWidth', 2); hold on;
    grid on; axis equal; daspect([1 1 1]);
    xlabel('North (m)'); ylabel('East (m)'); zlabel('Altitude (m)');
    title(sprintf('%s  -  FW 3D trajectory', checks.label));
    view(40, 20);
    saveas(fig3, fullfile(out_dir, [save_prefix '_3d.png']));

    fig4 = figure('Name', ['FW inputs ' config_name], 'NumberTitle','off', 'Position', [140 140 900 600]);
    plot_inputs(t, fw_controls); title(sprintf('%s  -  FW control inputs', checks.label));
    saveas(fig4, fullfile(out_dir, [save_prefix '_inputs.png']));

    fprintf('  plots -> plots/fw_maneuvers/%s_*\n', save_prefix);
end

% =====================================================================
function p = locate_config(name)
    here = fileparts(mfilename('fullpath'));
    repo = fileparts(here);
    c1 = fullfile(repo, 'fw_maneuvers',       ['config_' name '.m']);
    c2 = fullfile(repo, 'trajectory_configs', ['config_' name '.m']);
    if isfile(c1), p = c1; elseif isfile(c2), p = c2; else
        error('test_fw_maneuver:config_not_found', 'No config_%s.m in fw_maneuvers/ or trajectory_configs/', name);
    end
end

function R = quat_to_R(q)
    q = q(:)/max(norm(q),1e-12);
    q0=q(1); q1=q(2); q2=q(3); q3=q(4);
    R = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
         2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
         2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];
end

function rpy = quat_to_zyx_euler_deg(q)
    q = q(:)/max(norm(q),1e-12);
    q0=q(1); q1=q(2); q2=q(3); q3=q(4);
    roll  = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));
    sp    = 2*(q0*q2 - q3*q1);
    if abs(sp) >= 1, pitch = sign(sp)*pi/2; else, pitch = asin(sp); end
    yaw   = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2));
    rpy = [roll, pitch, yaw] * 180/pi;
end

function w = wrap180(d)
    w = mod(d + 180, 360) - 180;
end

function pass = check_range(pass, fails, name, value, checks, key)
    if isfield(checks, key) && ~isempty(checks.(key))
        lo = checks.(key)(1); hi = checks.(key)(2);
        if value < lo || value > hi
            pass = false;
            fails{end+1} = sprintf('%s = %.3f, expected in [%.3f, %.3f]', name, value, lo, hi);
            assignin('caller','fails',fails);
        end
    end
end

function pass = check_upper(pass, fails, name, value, checks, key)
    if isfield(checks, key) && ~isempty(checks.(key))
        if abs(value) > checks.(key)
            pass = false;
            fails{end+1} = sprintf('%s = %.3f, expected |.| <= %.3f', name, value, checks.(key));
            assignin('caller','fails',fails);
        end
    end
end

function plot_inputs(t, fw_controls)
    function v = fetch(name, default)
        if isfield(fw_controls, name)
            x = fw_controls.(name);
            if isscalar(x), v = x * ones(size(t)); else, v = interp1(fw_controls.t_sim, x, t, 'linear', 'extrap'); end
        else
            v = default * ones(size(t));
        end
    end
    th = fetch('thrust', 0);
    el = fetch('elevator', 0);
    ai = fetch('aileron', 0);
    ru = fetch('rudder', 0);
    subplot(4,1,1); plot(t, th, 'b-', 'LineWidth', 1.4); grid on; ylabel('thrust (N)');
    subplot(4,1,2); plot(t, el, 'b-', 'LineWidth', 1.4); grid on; ylabel('elevator (rad)');
    subplot(4,1,3); plot(t, ai, 'b-', 'LineWidth', 1.4); grid on; ylabel('aileron (rad)');
    subplot(4,1,4); plot(t, ru, 'b-', 'LineWidth', 1.4); grid on; ylabel('rudder (rad)'); xlabel('t (s)');
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
    xdot = fw_6dof_quat(t, s, th, el, ai, ru, fw_params);
end
