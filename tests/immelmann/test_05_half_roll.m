function [pass, msg] = test_05_half_roll(traj)
% PASS: during the scripted half-roll window (t in [2.45, 3.65] s after the
% half-loop), the FW accumulates ~180 deg of body roll (within tolerance).
    if nargin < 1, traj = run_immelmann_sim(); end
    mask = traj.t >= 2.75 & traj.t <= 3.95;
    sweep = abs(traj.body_roll_int_deg(find(mask,1,'last')) - traj.body_roll_int_deg(find(mask,1,'first')));
    pass = sweep >= 140 && sweep <= 220;
    msg = sprintf('half-roll body-roll sweep = %.1f deg  (want 140..220)', sweep);
end
