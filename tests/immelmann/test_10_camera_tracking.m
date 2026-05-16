function [pass, msg] = test_10_camera_tracking()
% PASS: with the drone+gimbal stack tracking the Immelmann, the camera
% quaternion matches the FW quaternion to better than 0.5 deg MAE.
%
% This one runs the FULL stack (drone + FW) — not just the FW sim — so it
% doesn't share the cached FW-only trajectory.
    here = fileparts(mfilename('fullpath'));
    repo = fileparts(fileparts(here));
    cd(repo);
    addpath('DFL_controller'); addpath('models'); addpath('utilities');
    addpath('fw_maneuvers'); addpath('trajectory_configs');

    out = run_compare3('immelmann', 'v3_immelmann');
    mae = out.metrics.mae_orient_deg;
    pass = mae < 0.5;
    msg  = sprintf('camera-orientation MAE = %.4f deg  (want < 0.5)', mae);
end
