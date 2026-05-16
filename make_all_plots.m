function make_all_plots()
% make_all_plots  One-shot regeneration of every comparison .mat + plot.
%
% Layout produced:
%   results/results_<tag>.mat   (gitignored)
%   plots/2axis_phase0+2+3/<scenario>_baseline_vs_fix.png
%   plots/3way_comparison/<scenario>_3way.png
%   plots/3axis_phase1+5/<scenario>_{3d,frames,orient_err}.png
%
% Scenarios driven: roll (360 deg barrel roll), loop (360 deg loop), rollsoft
% (limited bank, demo for 2-axis controller).

    here = fileparts(mfilename('fullpath'));
    cd(here);
    addpath('DFL_controller'); addpath('models'); addpath('utilities'); addpath('trajectory_configs');

    scenarios = {'roll', 'loop', 'rollsoft'};
    fprintf('=== running 2-axis baseline (original code, no fixes) ===\n');
    fprintf('(skipping — needs git revert; baseline_*.mat already present)\n');
    fprintf('=== running 2-axis Phase 0+2+3 (yaw schedule + X-Y + S inversion) ===\n');
    for k = 1:numel(scenarios)
        run_compare(scenarios{k}, ['fix_' scenarios{k}]);
    end
    fprintf('\n=== running 3-axis Phase 1+5 (camera-pose outputs) ===\n');
    for k = 1:numel(scenarios)
        run_compare3(scenarios{k}, ['v3_' scenarios{k}]);
    end

    fprintf('\n=== rendering 2-axis comparison plots ===\n');
    for k = 1:numel(scenarios)
        if isfile(fullfile('results', ['results_baseline_' scenarios{k} '.mat']))
            compare_runs(['baseline_' scenarios{k}], ['fix_' scenarios{k}]);
        else
            fprintf('  (skip %s — no baseline run available)\n', scenarios{k});
        end
    end

    fprintf('\n=== rendering 3-way comparison plots ===\n');
    for k = 1:numel(scenarios)
        b = fullfile('results', ['results_baseline_' scenarios{k} '.mat']);
        f = fullfile('results', ['results_fix_'      scenarios{k} '.mat']);
        v = fullfile('results', ['results_v3_'       scenarios{k} '.mat']);
        if isfile(b) && isfile(f) && isfile(v)
            compare3way(['baseline_' scenarios{k}], ['fix_' scenarios{k}], ['v3_' scenarios{k}], scenarios{k});
        else
            fprintf('  (skip %s — missing one of baseline/fix/v3)\n', scenarios{k});
        end
    end

    fprintf('\n=== rendering 3-axis 3D plots ===\n');
    for k = 1:numel(scenarios)
        mfile = fullfile('results', ['results_v3_' scenarios{k} '.mat']);
        if isfile(mfile)
            S = load(mfile); out = S.out;
            plotting3(out.t, out.state, scenarios{k}, scenarios{k});
        end
    end

    fprintf('\nAll done. See plots/ for figures, results/ for .mat data.\n');
end
