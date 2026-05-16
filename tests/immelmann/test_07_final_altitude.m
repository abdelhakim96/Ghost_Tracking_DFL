function [pass, msg] = test_07_final_altitude(traj)
% PASS: at end of simulation, FW altitude is higher than start by at
% least 80 m and at most 280 m (Immelmann gains altitude but the autopilot
% should prevent runaway climb).
    if nargin < 1, traj = run_immelmann_sim(); end
    gain = traj.alt(end) - traj.alt(1);
    pass = gain >= 80 && gain <= 280;
    msg  = sprintf('final altitude gain = %.1f m  (want 80..280)', gain);
end
