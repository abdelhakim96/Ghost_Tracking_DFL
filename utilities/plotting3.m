function plotting3(t, state, config_to_run, save_prefix)
% plotting3  3D trajectory + camera/FW frame overlay for the 18-state
%            3-axis-gimbal stack (Phase 1+5).
%
%   t, state : output of run_compare3 / unified_dynamics3
%   config_to_run : 'roll' | 'loop' | 'rollsoft' (used only in titles)
%   save_prefix : prefix for PNG output, e.g. 'v3_roll'

    if nargin < 4, save_prefix = config_to_run; end

    here = fileparts(mfilename('fullpath'));
    repo = fileparts(here);
    addpath(here);

    quad = state(:, 1:18);
    fw   = state(:, 19:31);

    x_quad = quad(:, 1:3);
    x_fw   = fw  (:, 1:3);

    % ---- 3D trajectory with FW aircraft model + frames -----------------
    fig1 = figure('Name', ['3D Trajectory (' config_to_run ')'], 'NumberTitle','off', ...
                  'Position', [50 50 1200 800]);
    % Display flips z (NED z-down -> altitude up)
    p1 = plot3(x_quad(:,1), x_quad(:,2), -x_quad(:,3), 'b-', 'LineWidth', 1.6); hold on;
    p1.Color(4) = 0.5;
    p2 = plot3(x_fw(:,1), x_fw(:,2), -x_fw(:,3), 'r--', 'LineWidth', 1.6);
    p2.Color(4) = 0.6;
    grid on; axis equal;
    xlabel('North (m)'); ylabel('East (m)'); zlabel('Altitude (m)');
    title(sprintf('Camera POV tracking: 3-axis Phase 1+5  (%s)', config_to_run));
    view(45, 25);

    % Load STL aircraft (if present)
    stl_path = fullfile(repo, 'CAD', 'aero.stl');
    have_stl = exist(stl_path, 'file') == 2;
    if have_stl
        [F, V, ~] = stlread(stl_path);
        V = V - mean(V);
        V = V / 20 * 0.7;
        % corrective rotation to match aircraft body axes
        Ryaw = [cos(-pi) -sin(-pi) 0; sin(-pi) cos(-pi) 0; 0 0 1];
        Rpitch = [cos(20*pi/180) 0 sin(20*pi/180); 0 1 0; -sin(20*pi/180) 0 cos(20*pi/180)];
        V = (Rpitch * Ryaw * V')';
    end

    legend_done_fw    = false;
    legend_done_cam   = false;
    legend_done_drone = false;

    step = max(1, round(length(t) / 25));   % ~25 snapshots along the trajectory
    for i = 1:step:length(t)
        % ----- FW pose -----
        pos_fw = [x_fw(i,1), x_fw(i,2), -x_fw(i,3)];
        qA = fw(i, 7:10); qA = qA / norm(qA + 1e-12);
        R_fw = quat_to_R(qA);

        if have_stl
            V_w = (R_fw * V')' + pos_fw;
            if ~legend_done_fw
                patch('Faces', F, 'Vertices', V_w, 'FaceColor', [0.85 0.25 0.25], ...
                      'EdgeColor', 'none', 'FaceAlpha', 0.15, 'FaceLighting','gouraud', ...
                      'DisplayName', 'Ghost FW (q_A)');
                legend_done_fw = true;
            else
                patch('Faces', F, 'Vertices', V_w, 'FaceColor', [0.85 0.25 0.25], ...
                      'EdgeColor', 'none', 'FaceAlpha', 0.15, 'FaceLighting','gouraud', ...
                      'HandleVisibility','off');
            end
        end

        % FW body-frame triad (red / green / blue, dashed)
        L = 4;
        draw_triad(pos_fw, R_fw, L, [0.7 0 0], [0 0.7 0], [0 0 0.7], '-', 1.2, ...
                   'FW frame (q_A)', ~legend_done_fw);

        % ----- Drone + camera pose -----
        pos_dr = [x_quad(i,1), x_quad(i,2), -x_quad(i,3)];
        qM = quad(i, 4:7); qM = qM / norm(qM + 1e-12);
        phi_g_k = quad(i,14); theta_g_k = quad(i,15); psi_g_k = quad(i,16);
        qGx = [cos(phi_g_k/2); sin(phi_g_k/2); 0; 0];
        qGy = [cos(theta_g_k/2); 0; sin(theta_g_k/2); 0];
        qGz = [cos(psi_g_k/2); 0; 0; sin(psi_g_k/2)];
        qG  = quat_mul(quat_mul(qGx, qGy), qGz);
        qC  = quat_mul(qM(:), qG);
        R_cam = quat_to_R(qC);

        % Camera-frame triad in cyan / magenta / yellow (different from FW so
        % overlap is visually obvious when they coincide)
        if ~legend_done_cam
            draw_triad(pos_dr, R_cam, L*0.7, [0.9 0.3 0.0], [0.0 0.6 0.6], [0.6 0.0 0.6], ...
                       ':', 2.0, 'Camera frame (q_M (x) q_G)', true);
            legend_done_cam = true;
        else
            draw_triad(pos_dr, R_cam, L*0.7, [0.9 0.3 0.0], [0.0 0.6 0.6], [0.6 0.0 0.6], ...
                       ':', 2.0, '', false);
        end
    end

    % Lights
    light('Position',[1 1 1],'Style','infinite');
    light('Position',[-1 -1 -0.5],'Style','infinite');
    legend('Location', 'best');

    out_dir = fullfile(repo, 'plots', '3axis_phase1+5');
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end
    out1 = fullfile(out_dir, sprintf('%s_3d.png', save_prefix));
    saveas(fig1, out1);
    fprintf('Saved %s\n', out1);

    % ---- Side-by-side: FW frame vs camera frame triads (zoomed snapshot)
    fig2 = figure('Name', ['Frame overlap (' config_to_run ')'], ...
                  'NumberTitle','off', 'Position',[100 100 1100 450]);
    snap_idx = round(length(t) * 0.5);     % mid-trajectory snapshot
    pos_fw = [x_fw(snap_idx,1), x_fw(snap_idx,2), -x_fw(snap_idx,3)];
    pos_dr = [x_quad(snap_idx,1), x_quad(snap_idx,2), -x_quad(snap_idx,3)];
    qA = fw(snap_idx, 7:10); qA = qA / norm(qA + 1e-12);
    R_fw = quat_to_R(qA);
    qM = quad(snap_idx, 4:7); qM = qM / norm(qM + 1e-12);
    phi_g_k = quad(snap_idx,14); theta_g_k = quad(snap_idx,15); psi_g_k = quad(snap_idx,16);
    qGx = [cos(phi_g_k/2); sin(phi_g_k/2); 0; 0];
    qGy = [cos(theta_g_k/2); 0; sin(theta_g_k/2); 0];
    qGz = [cos(psi_g_k/2); 0; 0; sin(psi_g_k/2)];
    qG  = quat_mul(quat_mul(qGx, qGy), qGz);
    qC  = quat_mul(qM(:), qG);
    R_cam = quat_to_R(qC);

    subplot(1,2,1); hold on; grid on; axis equal; view(45,25);
    title(sprintf('FW frame (red/green/blue) at t=%.2fs', t(snap_idx)));
    draw_triad([0 0 0], R_fw, 1.0, [0.7 0 0], [0 0.7 0], [0 0 0.7], '-', 2.0, '', false);
    xlabel('x'); ylabel('y'); zlabel('z'); xlim([-1.2 1.2]); ylim([-1.2 1.2]); zlim([-1.2 1.2]);

    subplot(1,2,2); hold on; grid on; axis equal; view(45,25);
    title(sprintf('Camera frame (cyan/magenta/orange) at t=%.2fs', t(snap_idx)));
    draw_triad([0 0 0], R_cam, 1.0, [0.9 0.3 0.0], [0.0 0.6 0.6], [0.6 0.0 0.6], '-', 2.0, '', false);
    xlabel('x'); ylabel('y'); zlabel('z'); xlim([-1.2 1.2]); ylim([-1.2 1.2]); zlim([-1.2 1.2]);

    sgtitle(sprintf('q_M (x) q_G  vs  q_A  at midpoint  (%s)', config_to_run));
    out2 = fullfile(out_dir, sprintf('%s_frames.png', save_prefix));
    saveas(fig2, out2);
    fprintf('Saved %s\n', out2);

    % ---- Orientation error timeseries
    fig3 = figure('Name', ['Orient error (' config_to_run ')'], 'NumberTitle','off', ...
                  'Position', [120 120 900 360]);
    geo = zeros(length(t),1);
    for k = 1:length(t)
        qM = quad(k,4:7).'; qM = qM/norm(qM+1e-12);
        phi_k=quad(k,14); theta_k=quad(k,15); psi_k=quad(k,16);
        qGx = [cos(phi_k/2); sin(phi_k/2); 0; 0];
        qGy = [cos(theta_k/2); 0; sin(theta_k/2); 0];
        qGz = [cos(psi_k/2); 0; 0; sin(psi_k/2)];
        qG  = quat_mul(quat_mul(qGx,qGy),qGz);
        qC  = quat_mul(qM,qG);
        qA  = fw(k,7:10).'; qA = qA/norm(qA+1e-12);
        qErr = quat_mul(quat_conj(qC), qA); qErr = qErr/norm(qErr+1e-12);
        geo(k) = 2*acos(min(1,abs(qErr(1)))) * 180/pi;
    end
    semilogy(t, geo, 'g-', 'LineWidth', 1.6); grid on;
    xlabel('t (s)'); ylabel('|q_M (x) q_G  ⊖  q_A|  (deg)');
    title(sprintf('Camera vs FW orientation error  (%s, 3-axis Phase 1+5)', config_to_run));
    ylim([1e-5 200]);
    out3 = fullfile(out_dir, sprintf('%s_orient_err.png', save_prefix));
    saveas(fig3, out3);
    fprintf('Saved %s\n', out3);
end

function R = quat_to_R(q)
    q = q(:) / max(norm(q), 1e-12);
    q0=q(1); q1=q(2); q2=q(3); q3=q(4);
    R = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
         2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
         2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];
end

function draw_triad(pos, R, L, cx, cy, cz, style, lw, label, show_label)
    bx = R(:,1) * L;
    by = R(:,2) * L;
    bz = R(:,3) * L;
    % NED -> display: flip z component for arrow tail->head
    quiver3(pos(1), pos(2), pos(3), bx(1), bx(2), -bx(3), 0, ...
            'Color', cx, 'LineWidth', lw, 'LineStyle', style, 'HandleVisibility','off');
    quiver3(pos(1), pos(2), pos(3), by(1), by(2), -by(3), 0, ...
            'Color', cy, 'LineWidth', lw, 'LineStyle', style, 'HandleVisibility','off');
    h = quiver3(pos(1), pos(2), pos(3), bz(1), bz(2), -bz(3), 0, ...
            'Color', cz, 'LineWidth', lw, 'LineStyle', style);
    if show_label && ~isempty(label)
        set(h, 'DisplayName', label);
    else
        set(h, 'HandleVisibility','off');
    end
end
