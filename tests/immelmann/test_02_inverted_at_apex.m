function [pass, msg] = test_02_inverted_at_apex(traj)
% PASS: FW reaches a fully-inverted attitude (R(3,3) <= -0.7) at the top
% of the half-loop.
    if nargin < 1, traj = run_immelmann_sim(); end
    minR = min(traj.R33);
    pass = minR <= -0.7;
    msg  = sprintf('min R(3,3) = %.3f  (want <= -0.7,  -1 = fully inverted)', minR);
end
