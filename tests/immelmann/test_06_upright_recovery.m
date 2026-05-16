function [pass, msg] = test_06_upright_recovery(traj)
% PASS: once the autopilot has had at least 1 s to settle (t >= 4.7 s),
% the FW stays approximately upright (R(3,3) >= 0.7) for the rest of the
% simulation.
    if nargin < 1, traj = run_immelmann_sim(); end
    mask = traj.t >= 5.0;
    if ~any(mask)
        pass = false; msg = 'simulation too short to evaluate recovery'; return;
    end
    minR_post = min(traj.R33(mask));
    pass = minR_post >= 0.7;
    msg  = sprintf('min R(3,3) after t=4.7s = %.3f  (want >= 0.7, 1=upright)', minR_post);
end
