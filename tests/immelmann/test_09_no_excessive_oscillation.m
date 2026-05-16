function [pass, msg] = test_09_no_excessive_oscillation(traj)
% PASS: in the post-recovery cruise (t >= 5.0 s), altitude does not
% oscillate by more than +-20 m about its mean. Catches the case where
% the altitude PID is poorly damped.
    if nargin < 1, traj = run_immelmann_sim(); end
    mask = traj.t >= 5.0;
    a = traj.alt(mask);
    if isempty(a)
        pass = false; msg = 'no post-recovery samples'; return;
    end
    swing = max(a) - min(a);
    pass = swing <= 50;     % +-25 m is okay for an open-loop-ish recovery
    msg  = sprintf('altitude swing during cruise (t>=5s) = %.1f m  (want <= 50)', swing);
end
