function regen_v3_plots()
    addpath('utilities');
    for sc = {'roll', 'loop', 'rollsoft'}
        s = load(fullfile('results', ['results_v3_' sc{1} '.mat']));
        plotting3(s.out.t, s.out.state, sc{1}, sc{1});
    end
end
