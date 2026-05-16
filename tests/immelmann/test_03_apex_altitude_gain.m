function [pass, msg] = test_03_apex_altitude_gain(traj)
% PASS: the peak altitude during the maneuver exceeds the start altitude
% by at least 100 m (a half-loop on a 290 kg airframe with the configured
% elevator pull should easily clear that).
    if nargin < 1, traj = run_immelmann_sim(); end
    apex_gain = max(traj.alt) - traj.alt(1);
    pass = apex_gain >= 100;
    msg  = sprintf('apex altitude gain = %.1f m  (want >= 100)', apex_gain);
end
