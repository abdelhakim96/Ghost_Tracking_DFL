function extract_matlab_samples()
% Writes web_demo/tests/data/*.json with sampled (input, output) tuples
% from the MATLAB reference implementations. JS unit tests load these
% to verify the JS ports.

    here = fileparts(mfilename('fullpath'));
    repo = fileparts(here);
    cd(repo);
    addpath('models'); addpath('utilities'); addpath('DFL_controller');

    out_dir = fullfile('web_demo', 'tests', 'data');
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    rng(42, 'twister');

    % --- 1. quat samples (round-trip identities)
    q_samples = cell(30, 1);
    for k = 1:30
        a = randn(4,1); a = a / norm(a);
        b = randn(4,1); b = b / norm(b);
        q_samples{k} = struct(...
            'a', a(:).', ...
            'b', b(:).', ...
            'a_mul_b', quat_mul(a, b).', ...
            'a_conj', quat_conj(a).');
    end
    write_json(fullfile(out_dir, 'quat_samples.json'), q_samples);

    % --- 2. fw_dynamics samples
    fw_params = make_edge540_params();
    fw_samples = cell(30, 1);
    for k = 1:30
        s = random_fw_state();
        th = 100*rand(); el = 0.4*(2*rand()-1);
        ai = 0.6*(2*rand()-1); ru = 0.4*(2*rand()-1);
        xdot = fw_6dof_quat(0, s, th, el, ai, ru, fw_params);
        fw_samples{k} = struct('state', s(:).', 'thrust', th, ...
            'elevator', el, 'aileron', ai, 'rudder', ru, ...
            'xdot', xdot(:).');
    end
    write_json(fullfile(out_dir, 'fw_dynamics_samples.json'), ...
        struct('params', fw_params, 'samples', {fw_samples}));

    % --- 3. alpha_beta samples
    ab_samples = cell(30, 1);
    for k = 1:30
        s = random_drone_state();
        zeta = s(17); xi = s(18);
        a = alpha_gimbal_func3(s, 0, 0, 0, 0.01, 0.01, 0.01, ...
                               0.0023, 0.0023, 0.0046, ...
                               0.001, 0.001, 0.001, zeta, xi, 0.468);
        b = beta_gimbal_func3(s, 0, 0, 0, 0.01, 0.01, 0.01, ...
                              0.0023, 0.0023, 0.0046, ...
                              0.001, 0.001, 0.001, zeta, xi, 0.468);
        ab_samples{k} = struct('state', s(:).', ...
            'alpha', a(:).', 'beta_flat', b(:).');
    end
    write_json(fullfile(out_dir, 'alpha_beta_samples.json'), ab_samples);

    % --- 4. dfl_controller samples (need fw_state + drone_state)
    % dfl_controller3 reads m, Ix, Iy, Iz, g via globals; set them up front.
    global m Ix Iy Iz g
    m = 0.468; Ix = 0.0023; Iy = 0.0023; Iz = 0.0046; g = 9.81;
    dfl_samples = cell(15, 1);
    for k = 1:15
        ds = random_drone_state();
        fs = random_fw_state();
        gains = default_dfl_gains();
        xd = randn(3,1)*10;
        vd = randn(3,1)*5;
        ad = randn(3,1);
        jd = zeros(3,1); sd = zeros(3,1);
        psid = 0;
        u = dfl_controller3(0, ds, xd, vd, ad, jd, sd, psid, fs, fs(7:10), gains);
        dfl_samples{k} = struct('drone_state', ds(:).', 'fw_state', fs(:).', ...
            'xd', xd(:).', 'vd', vd(:).', 'ad', ad(:).', ...
            'jd', jd(:).', 'sd', sd(:).', 'psid', psid, ...
            'gains', gains, 'u', u(:).');
    end
    write_json(fullfile(out_dir, 'dfl_controller_samples.json'), dfl_samples);

    % --- 5. drone_dynamics samples (xdot from drone_state + u + fw_state)
    dy_samples = cell(15, 1);
    for k = 1:15
        ds = random_drone_state();
        fs = random_fw_state();
        gains = default_dfl_gains();
        xd = fs(1:3); vd = randn(3,1); ad = zeros(3,1);
        jd = zeros(3,1); sd = zeros(3,1);
        % use the realtime function which calls dfl internally
        global m Ix Iy Iz g
        m = 0.468; Ix = 0.0023; Iy = 0.0023; Iz = 0.0046; g = 9.81;
        sdot = quadrotor_dynamics_realtime3(0, ds, xd, vd, ad, jd, sd, ...
                                            0, fs, fs(7:10), gains);
        dy_samples{k} = struct('drone_state', ds(:).', 'fw_state', fs(:).', ...
            'xd', xd(:).', 'vd', vd(:).', 'ad', ad(:).', ...
            'jd', jd(:).', 'sd', sd(:).', ...
            'gains', gains, 'sdot', sdot(:).');
    end
    write_json(fullfile(out_dir, 'drone_dynamics_samples.json'), dy_samples);

    fprintf('Wrote 5 sample files to %s/\n', out_dir);
end

function s = random_fw_state()
    p = randn(3,1)*50 + [0;0;-100];
    v = [120 + 10*randn(); 5*randn(); 5*randn()];
    q = randn(4,1); q = q/norm(q);
    w = 0.3*randn(3,1);
    s = [p; v; q; w];
end

function s = random_drone_state()
    p = randn(3,1)*50 + [0;0;-100];
    q = randn(4,1); q = q/norm(q);
    v = [120;0;0] + 5*randn(3,1);
    w = 0.1*randn(3,1);
    gimbal = 0.3*randn(3,1);
    zeta = 0.468*9.81 + 0.5*randn();
    xi   = 0.1*randn();
    s = [p; q; v; w; gimbal; zeta; xi];
end

function g = default_dfl_gains()
    g.c0 = 51150; g.c1 = 51140; g.c2 = 1150; g.c3 = 150;
    g.c4 = 1; g.c5 = 1;
    g.c_phi = 50; g.c_theta = 50; g.c_psig = 50;
    g.c_q3 = 100; g.c_q3_dot = 20;
end

function p = make_edge540_params()
    p.m = 290; p.J = diag([550, 750, 1100]);
    p.S = 9.1; p.b = 7.44; p.c = 1.22;
    p.rho = 1.225; p.g = 9.81;
    p.CL0 = 0.4; p.CL_alpha = 5.7; p.CL_q = 7.0; p.CL_de = -0.8;
    p.CD0 = 0.04; p.k = 0.05; p.CDa = 0.1; p.CD_q = 0.0; p.CD_de = 0.0;
    p.CY_beta = -1.2; p.CY_p = -0.1; p.CY_r = 0.2; p.CY_da = 0.2; p.CY_dr = -0.2;
    p.Cl_beta = -0.15; p.Cl_p = -1.0; p.Cl_r = 0.25; p.Cl_da = 0.5; p.Cl_dr = 0.05;
    p.Cm0 = 0.0; p.Cm_alpha = -1.5; p.Cm_q = -15.0; p.Cm_de = -1.8;
    p.Cn_beta = 0.15; p.Cn_p = -0.1; p.Cn_r = -0.4; p.Cn_da = 0.04; p.Cn_dr = -0.1;
end

function write_json(filename, data)
    txt = jsonencode(data, 'PrettyPrint', true);
    fid = fopen(filename, 'w');
    fwrite(fid, txt);
    fclose(fid);
end
