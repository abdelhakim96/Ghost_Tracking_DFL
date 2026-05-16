function regen_all()
    addpath('utilities');
    manvs = {'barrelroll','immelmann','splits','cuban8'};
    for k = 1:numel(manvs)
        run_compare3(manvs{k}, ['v3_' manvs{k}]);
    end
    for k = 1:numel(manvs)
        s = load(fullfile('results', ['results_v3_' manvs{k} '.mat']));
        plotting3(s.out.t, s.out.state, manvs{k}, manvs{k});
    end
end
