function compare3way(base_tag, fix_tag, v3_tag, title_str)
% compare3way Side-by-side plot: baseline vs Phase 0+2+3 vs Phase 1+5.
    B = load(fullfile('results', ['results_' base_tag '.mat'])); B = B.out;
    F = load(fullfile('results', ['results_' fix_tag  '.mat'])); F = F.out;
    V = load(fullfile('results', ['results_' v3_tag   '.mat'])); V = V.out;

    fprintf('\n=== %s ===\n', title_str);
    fprintf('%-30s %15s %15s %15s\n', 'metric', base_tag, fix_tag, v3_tag);
    fprintf('%-30s %15.3f %15.3f %15.3f\n', 'final t (s)',         B.metrics.final_time, F.metrics.final_time, V.metrics.final_time);
    fprintf('%-30s %15.4f %15.4f %15.4f\n', 'MAE pos (m)',         B.metrics.mae_pos,    F.metrics.mae_pos,    V.metrics.mae_pos);
    fprintf('%-30s %15.4f %15.4f %15.4f\n', 'MAE orient (deg)',    B.metrics.mae_orient_deg, F.metrics.mae_orient_deg, V.metrics.mae_orient_deg);
    fprintf('%-30s %15.4f %15.4f %15.4f\n', 'max orient (deg)',    B.metrics.max_orient_err_deg, F.metrics.max_orient_err_deg, V.metrics.max_orient_err_deg);

    fig = figure('Name', title_str, 'NumberTitle','off', 'Position',[100 100 1100 600]);
    subplot(1,2,1); hold on; grid on;
    plot(B.t, B.metrics.pos_err_mag, 'r-',  'LineWidth', 1.4, 'DisplayName','baseline');
    plot(F.t, F.metrics.pos_err_mag, 'b--', 'LineWidth', 1.4, 'DisplayName','Phase 0+2+3');
    plot(V.t, V.metrics.pos_err_mag, 'g-',  'LineWidth', 1.6, 'DisplayName','Phase 1+5 (3-axis)');
    xlabel('t (s)'); ylabel('|p_M - p_A| (m)'); title('Position tracking error');
    legend('Location','best');

    subplot(1,2,2); hold on; grid on;
    plot(B.t, B.metrics.geo_err_deg, 'r-',  'LineWidth', 1.4, 'DisplayName','baseline');
    plot(F.t, F.metrics.geo_err_deg, 'b--', 'LineWidth', 1.4, 'DisplayName','Phase 0+2+3');
    plot(V.t, V.metrics.geo_err_deg, 'g-',  'LineWidth', 1.6, 'DisplayName','Phase 1+5 (3-axis)');
    xlabel('t (s)'); ylabel('camera orientation error (deg)'); title('Camera orientation: q_M (x) q_G  vs  q_A');
    set(gca, 'YScale', 'log');
    ylim([1e-4, 200]);
    legend('Location','best');

    sgtitle(title_str);
    out_dir = fullfile('plots', '3way_comparison');
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end
    fname = fullfile(out_dir, sprintf('%s_3way.png', strrep(title_str, ' ', '_')));
    saveas(fig, fname);
    fprintf('Saved figure: %s\n', fname);
end
