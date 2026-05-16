function run_all()
% Immelmann test suite — runs 10 per-property tests in order.
    here = fileparts(mfilename('fullpath'));
    cd(fileparts(fileparts(here)));
    addpath(here);

    tests = {
        'test_01_pull_up'
        'test_02_inverted_at_apex'
        'test_03_apex_altitude_gain'
        'test_04_heading_reversal'
        'test_05_half_roll'
        'test_06_upright_recovery'
        'test_07_final_altitude'
        'test_08_flying_back'
        'test_09_no_excessive_oscillation'
        'test_10_camera_tracking'
    };

    traj = run_immelmann_sim();  % cached after first run

    results = cell(numel(tests), 3);
    for k = 1:numel(tests)
        try
            if strcmp(tests{k}, 'test_10_camera_tracking')
                [pass, msg] = feval(tests{k});   % needs the full drone+FW stack
            else
                [pass, msg] = feval(tests{k}, traj);
            end
        catch ME
            pass = false; msg = sprintf('threw: %s', ME.message);
        end
        results(k, :) = {tests{k}, pass, msg};
    end

    fprintf('\n=============== Immelmann test suite ===============\n');
    nP = 0;
    for k = 1:numel(tests)
        if results{k, 2}, tag = 'PASS'; nP = nP + 1; else, tag = 'FAIL'; end
        fprintf(' [%s]  %-34s  %s\n', tag, results{k,1}, results{k,3});
    end
    fprintf('----------------------------------------------------\n');
    fprintf('Total: %d/%d passing\n', nP, numel(tests));
end
