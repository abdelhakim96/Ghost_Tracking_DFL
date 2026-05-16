function run_all_tests()
% run_all_tests  Execute the FW maneuver test suite.
%   Each test:
%     - runs the FW dynamics alone on its config
%     - asserts geometric/kinematic criteria
%     - writes plots under plots/fw_maneuvers/
%   At the end prints PASS/FAIL summary.

    here = fileparts(mfilename('fullpath'));
    cd(fileparts(here));
    addpath('tests'); addpath('models'); addpath('utilities');
    addpath('fw_maneuvers'); addpath('trajectory_configs');

    tests = {'test_barrelroll', 'test_immelmann', 'test_splits', 'test_cuban8'};
    results = false(1, numel(tests));
    for k = 1:numel(tests)
        try
            results(k) = feval(tests{k});
        catch ME
            fprintf('\n%s threw: %s\n', tests{k}, ME.message);
            results(k) = false;
        end
    end

    fprintf('\n=========== FW maneuver test summary ===========\n');
    for k = 1:numel(tests)
        if results(k), tag = 'PASS'; else, tag = 'FAIL'; end
        fprintf('  %-22s : %s\n', tests{k}, tag);
    end
    fprintf('Total: %d/%d passing\n', sum(results), numel(results));
end
