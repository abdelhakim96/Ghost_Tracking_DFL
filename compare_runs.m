function compare_runs(base_tag, fix_tag)
% compare_runs Side-by-side metrics + figure for two saved runs.

    base = load(['results_' base_tag '.mat']); base = base.out;
    fix  = load(['results_' fix_tag  '.mat']); fix  = fix.out;

    fprintf('\n=== %s vs %s ===\n', base_tag, fix_tag);
    fprintf('%-35s %12s %12s\n', 'metric', base_tag, fix_tag);
    fprintf('%-35s %12.4f %12.4f\n', 'final t (s)',            base.metrics.final_time, fix.metrics.final_time);
    fprintf('%-35s %12.4f %12.4f\n', 'MAE position (m)',       base.metrics.mae_pos,    fix.metrics.mae_pos);
    fprintf('%-35s %12.4f %12.4f\n', 'max position err (m)',   base.metrics.max_pos_err,fix.metrics.max_pos_err);
    fprintf('%-35s %12.4f %12.4f\n', 'MAE orientation (deg)',  base.metrics.mae_orient_deg, fix.metrics.mae_orient_deg);
    fprintf('%-35s %12.4f %12.4f\n', 'max orientation err (deg)', base.metrics.max_orient_err_deg, fix.metrics.max_orient_err_deg);
    fprintf('%-35s %12.4f %12.4f\n', 'min |cos(phi_g)cos(theta_g)|', base.metrics.min_cosphig_costhetag, fix.metrics.min_cosphig_costhetag);

    % Common window: from t=0 to min(end times)
    t_end_common = min(base.metrics.final_time, fix.metrics.final_time);
    fprintf('\nFair comparison over t ∈ [0, %.3f]s:\n', t_end_common);
    [mb_pos, mb_or] = window_metrics(base, t_end_common);
    [mf_pos, mf_or] = window_metrics(fix,  t_end_common);
    fprintf('%-35s %12.4f %12.4f   (%+.1f%%)\n', 'MAE position (m)', mb_pos, mf_pos, 100*(mf_pos-mb_pos)/max(mb_pos,1e-9));
    fprintf('%-35s %12.4f %12.4f   (%+.1f%%)\n', 'MAE orientation (deg)', mb_or,  mf_or,  100*(mf_or -mb_or )/max(mb_or, 1e-9));

    % Also report orientation error before singularity activates (cosc > 0.5)
    [mb_or_safe, mb_t_safe] = pre_singularity_orient(base, 0.5);
    [mf_or_safe, mf_t_safe] = pre_singularity_orient(fix , 0.5);
    fprintf('\nOrientation MAE on segment with |cos(phi_g)cos(theta_g)| > 0.5 (gimbal reachable):\n');
    fprintf('%-35s %12.4f deg over t<=%.3fs\n', base_tag, mb_or_safe, mb_t_safe);
    fprintf('%-35s %12.4f deg over t<=%.3fs\n', fix_tag,  mf_or_safe, mf_t_safe);

    % Plot
    fig = figure('Name', sprintf('%s vs %s', base_tag, fix_tag), 'NumberTitle','off', 'Position',[100 100 1100 800]);
    subplot(2,2,1); hold on; grid on;
    plot(base.t, base.metrics.pos_err_mag, 'r-', 'LineWidth', 1.4, 'DisplayName', base_tag);
    plot(fix.t,  fix.metrics.pos_err_mag,  'b-', 'LineWidth', 1.4, 'DisplayName', fix_tag);
    xlabel('t (s)'); ylabel('|p_M - p_A| (m)'); title('Position tracking error'); legend('Location','best');

    subplot(2,2,2); hold on; grid on;
    plot(base.t, base.metrics.geo_err_deg, 'r-', 'LineWidth', 1.4, 'DisplayName', base_tag);
    plot(fix.t,  fix.metrics.geo_err_deg,  'b-', 'LineWidth', 1.4, 'DisplayName', fix_tag);
    xlabel('t (s)'); ylabel('geodesic err (deg)'); title('Camera-orientation error  (q_M ⊗ q_G  vs  q_A)'); legend('Location','best');

    subplot(2,2,3); hold on; grid on;
    plot(base.t, base.state(:,14)*180/pi, 'r-', 'LineWidth', 1.2, 'DisplayName', [base_tag ' \phi_g']);
    plot(base.t, base.state(:,15)*180/pi, 'r--','LineWidth', 1.2, 'DisplayName', [base_tag ' \theta_g']);
    plot(fix.t,  fix.state(:,14)*180/pi,  'b-', 'LineWidth', 1.2, 'DisplayName', [fix_tag  ' \phi_g']);
    plot(fix.t,  fix.state(:,15)*180/pi,  'b--','LineWidth', 1.2, 'DisplayName', [fix_tag  ' \theta_g']);
    yline( 90,'k:'); yline(-90,'k:');
    xlabel('t (s)'); ylabel('gimbal angle (deg)'); title('Gimbal state (dashed lines = singular set)'); legend('Location','best','NumColumns',2);

    subplot(2,2,4); hold on; grid on;
    plot(base.t, base.metrics.cosphi_costheta, 'r-', 'LineWidth', 1.4, 'DisplayName', base_tag);
    plot(fix.t,  fix.metrics.cosphi_costheta,  'b-', 'LineWidth', 1.4, 'DisplayName', fix_tag);
    yline(0,'k:');
    xlabel('t (s)'); ylabel('|cos\phi_g cos\theta_g|'); title('2-axis gimbal authority (paper Assumption 1)'); legend('Location','best');

    saveas(fig, sprintf('compare_%s_vs_%s.png', base_tag, fix_tag));
    fprintf('\nSaved figure: compare_%s_vs_%s.png\n', base_tag, fix_tag);
end

function [pos, ori] = window_metrics(run, t_end_common)
    mask = run.t <= t_end_common;
    pos = mean(run.metrics.pos_err_mag(mask));
    ori = mean(run.metrics.geo_err_deg(mask));
end

function [ori, t_safe] = pre_singularity_orient(run, thr)
    csc = run.metrics.cosphi_costheta;
    bad = find(csc < thr, 1, 'first');
    if isempty(bad)
        idx = 1:numel(run.t);
    else
        idx = 1:bad-1;
        if isempty(idx), idx = 1; end
    end
    ori = mean(run.metrics.geo_err_deg(idx));
    t_safe = run.t(max(idx));
end
