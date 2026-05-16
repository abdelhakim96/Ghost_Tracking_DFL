function plotting3(t, state, config_to_run, save_prefix)
% plotting3  Per-scenario plots for the 18-state 3-axis stack.
%   Outputs (written to plots/3axis_phase1+5/):
%       <save_prefix>_3d.png            3D trajectory with FW STL + frames
%       <save_prefix>_euler.png         FW vs camera Euler angles  (roll, pitch, yaw)
%       <save_prefix>_position.png      FW vs camera position      (x, y, z)
%       <save_prefix>_orient_err.png    geodesic camera-orientation error (log)

    if nargin < 4, save_prefix = config_to_run; end

    here = fileparts(mfilename('fullpath'));
    repo = fileparts(here);
    addpath(here);

    out_dir = fullfile(repo, 'plots', '3axis_phase1+5');
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    quad = state(:, 1:18);
    fw   = state(:, 19:31);
    x_quad = quad(:, 1:3);
    x_fw   = fw  (:, 1:3);

    %% --- Compute camera quaternion at every sample ---------------------
    n = numel(t);
    qC_hist = zeros(n,4);
    qA_hist = zeros(n,4);
    for k = 1:n
        qM = quad(k,4:7).'; qM = qM/norm(qM+1e-12);
        phi_k = quad(k,14); theta_k = quad(k,15); psi_k = quad(k,16);
        qGx = [cos(phi_k/2); sin(phi_k/2); 0; 0];
        qGy = [cos(theta_k/2); 0; sin(theta_k/2); 0];
        qGz = [cos(psi_k/2); 0; 0; sin(psi_k/2)];
        qG  = quat_mul(quat_mul(qGx, qGy), qGz);
        qC_hist(k,:) = quat_mul(qM, qG).';
        qa = fw(k,7:10).'; qa = qa/norm(qa+1e-12);
        qA_hist(k,:) = qa.';
    end

    %% --- 1. 3D trajectory with STL ------------------------------------
    fig1 = figure('Name', ['3D trajectory (' config_to_run ')'], 'NumberTitle','off', ...
                  'Position', [50 50 1200 800]);
    p1 = plot3(x_quad(:,1), x_quad(:,2), -x_quad(:,3), 'b-', 'LineWidth', 1.6); hold on;
    p1.Color(4) = 0.5;
    p2 = plot3(x_fw(:,1), x_fw(:,2), -x_fw(:,3), 'r--', 'LineWidth', 1.6);
    p2.Color(4) = 0.6;
    grid on;
    axis equal;
    daspect([1 1 1]);                   % enforce equal aspect on all three axes
    xlabel('North (m)'); ylabel('East (m)'); zlabel('Altitude (m)');
    title(sprintf('Camera POV tracking, 3-axis Phase 1+5  (%s)', config_to_run));
    % Pick a viewing angle whose elevation works for both vertical and
    % horizontal maneuvers
    view(40, 20);

    stl_path = fullfile(repo, 'CAD', 'aero.stl');
    have_stl = exist(stl_path, 'file') == 2;
    if have_stl
        [F, V, ~] = stlread(stl_path);
        V = V - mean(V);  V = V / 20 * 0.7;
        Ryaw   = [cos(-pi) -sin(-pi) 0; sin(-pi) cos(-pi) 0; 0 0 1];
        Rpitch = [cos(20*pi/180) 0 sin(20*pi/180); 0 1 0; -sin(20*pi/180) 0 cos(20*pi/180)];
        V = (Rpitch * Ryaw * V')';
    end

    legend_done = false;
    step = max(1, round(n / 25));
    for i = 1:step:n
        pos_fw = [x_fw(i,1), x_fw(i,2), -x_fw(i,3)];
        R_fw = quat_to_R(qA_hist(i,:));
        if have_stl
            V_w = (R_fw * V')' + pos_fw;
            if ~legend_done
                patch('Faces', F, 'Vertices', V_w, 'FaceColor', [0.85 0.25 0.25], ...
                      'EdgeColor', 'none', 'FaceAlpha', 0.15, 'FaceLighting','gouraud', ...
                      'DisplayName', 'FW (q_A)');
            else
                patch('Faces', F, 'Vertices', V_w, 'FaceColor', [0.85 0.25 0.25], ...
                      'EdgeColor', 'none', 'FaceAlpha', 0.15, 'FaceLighting','gouraud', ...
                      'HandleVisibility','off');
            end
        end
        L = 5;
        draw_triad(pos_fw, R_fw, L, [0.7 0 0], [0 0.7 0], [0 0 0.7], '-', 1.3, ...
                   'FW body frame', ~legend_done);

        pos_dr = [x_quad(i,1), x_quad(i,2), -x_quad(i,3)];
        R_cam  = quat_to_R(qC_hist(i,:));
        draw_triad(pos_dr, R_cam, L*0.7, [0.9 0.3 0.0], [0.0 0.6 0.6], [0.6 0.0 0.6], ...
                   ':', 2.0, 'Camera frame  q_M (x) q_G', ~legend_done);
        legend_done = true;
    end
    light('Position',[1 1 1],'Style','infinite');
    light('Position',[-1 -1 -0.5],'Style','infinite');
    legend('Location', 'best');
    saveas(fig1, fullfile(out_dir, [save_prefix '_3d.png']));
    fprintf('Saved %s_3d.png\n', save_prefix);

    %% --- 2. Euler angles: FW vs camera  (roll/pitch/yaw)
    eA = zeros(n, 3);  eC = zeros(n, 3);
    for k = 1:n
        eA(k,:) = quat_to_zyx_euler(qA_hist(k,:));
        eC(k,:) = quat_to_zyx_euler(qC_hist(k,:));
    end
    % Unwrap the FW trajectory across time so it's continuous past +-pi.
    eA = unwrap(eA, [], 1);
    % Place camera Euler on the SAME branch as FW (ZYX gimbal lock at
    % pitch=+-pi/2 allows two valid Euler reps of the same quaternion — pick
    % the one closest to FW so the two traces overlap visually).
    for ax = 1:3
        d = eC(:,ax) - eA(:,ax);
        d = mod(d + pi, 2*pi) - pi;
        eC(:,ax) = eA(:,ax) + d;
    end

    fig2 = figure('Name', ['Euler (' config_to_run ')'], 'NumberTitle','off', ...
                  'Position', [80 80 1100 700]);
    titles = {'Roll  \phi  (deg)', 'Pitch  \theta  (deg)', 'Yaw  \psi  (deg)'};
    for ax = 1:3
        subplot(3,1,ax); hold on; grid on;
        plot(t, eA(:,ax)*180/pi, 'r-',  'LineWidth', 1.6, 'DisplayName', 'FW  (q_A)');
        plot(t, eC(:,ax)*180/pi, 'g--', 'LineWidth', 1.4, 'DisplayName', 'Camera  (q_M (x) q_G)');
        ylabel(titles{ax});
        if ax == 1
            title(sprintf('Euler angles  (ZYX, intrinsic)  —  %s, 3-axis Phase 1+5', config_to_run));
            legend('Location', 'best');
        end
        if ax == 3, xlabel('t (s)'); end
    end
    saveas(fig2, fullfile(out_dir, [save_prefix '_euler.png']));
    fprintf('Saved %s_euler.png\n', save_prefix);

    %% --- 3. Per-axis position: FW vs camera ----------------------------
    fig3 = figure('Name', ['Position (' config_to_run ')'], 'NumberTitle','off', ...
                  'Position', [110 110 1100 700]);
    axis_names = {'x  North  (m)', 'y  East  (m)', 'z  Down  (m)  [neg = altitude]'};
    for ax = 1:3
        subplot(3,1,ax); hold on; grid on;
        plot(t, x_fw(:,ax),   'r-',  'LineWidth', 1.6, 'DisplayName', 'FW  (p_A)');
        plot(t, x_quad(:,ax), 'g--', 'LineWidth', 1.4, 'DisplayName', 'Camera  (p_M + R t_G,  t_G=0)');
        ylabel(axis_names{ax});
        if ax == 1
            title(sprintf('Position  (FW vs camera)  —  %s, 3-axis Phase 1+5', config_to_run));
            legend('Location', 'best');
        end
        if ax == 3, xlabel('t (s)'); end
    end
    saveas(fig3, fullfile(out_dir, [save_prefix '_position.png']));
    fprintf('Saved %s_position.png\n', save_prefix);

    %% --- 4. Orientation geodesic error ---------------------------------
    geo = zeros(n,1);
    for k = 1:n
        qErr = quat_mul(quat_conj(qC_hist(k,:).'), qA_hist(k,:).');
        qErr = qErr / (norm(qErr) + 1e-12);
        geo(k) = 2*acos(min(1,abs(qErr(1)))) * 180/pi;
    end
    fig4 = figure('Name', ['Orient err (' config_to_run ')'], 'NumberTitle','off', ...
                  'Position', [140 140 900 360]);
    semilogy(t, geo, 'g-', 'LineWidth', 1.6); grid on;
    xlabel('t (s)'); ylabel('|q_M (x) q_G  ⊖  q_A|  (deg)');
    title(sprintf('Camera vs FW orientation error  (%s, 3-axis Phase 1+5)', config_to_run));
    ylim([1e-5 200]);
    saveas(fig4, fullfile(out_dir, [save_prefix '_orient_err.png']));
    fprintf('Saved %s_orient_err.png\n', save_prefix);
end

% =====================================================================
function R = quat_to_R(q)
    q = q(:) / max(norm(q), 1e-12);
    q0=q(1); q1=q(2); q2=q(3); q3=q(4);
    R = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
         2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
         2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];
end

function rpy = quat_to_zyx_euler(q)
% Returns [roll; pitch; yaw] for the standard aerospace ZYX intrinsic Euler convention.
    q = q(:) / max(norm(q), 1e-12);
    q0=q(1); q1=q(2); q2=q(3); q3=q(4);
    roll  = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));
    sinp  = 2*(q0*q2 - q3*q1);
    if abs(sinp) >= 1
        pitch = sign(sinp)*pi/2;
    else
        pitch = asin(sinp);
    end
    yaw   = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2));
    rpy = [roll, pitch, yaw];
end

function draw_triad(pos, R, L, cx, cy, cz, style, lw, label, show_label)
    bx = R(:,1) * L;  by = R(:,2) * L;  bz = R(:,3) * L;
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
