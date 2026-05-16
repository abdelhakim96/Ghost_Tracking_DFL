function [pass, msg] = test_01_pull_up(traj)
% PASS criterion: the FW pitches through ~180 deg of body rotation during
% the scripted pull-up window (t in [0.30, 2.35] s).
    if nargin < 1, traj = run_immelmann_sim(); end
    mask = traj.t >= 0.30 & traj.t <= 2.65;
    sweep = traj.body_pitch_int_deg(find(mask,1,'last')) - traj.body_pitch_int_deg(find(mask,1,'first'));
    pass = sweep >= 150 && sweep <= 220;
    msg = sprintf('pull_up body-pitch sweep = %.1f deg  (want 150..220)', sweep);
end
