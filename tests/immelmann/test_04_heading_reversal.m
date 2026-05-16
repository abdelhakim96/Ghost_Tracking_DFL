function [pass, msg] = test_04_heading_reversal(traj)
% PASS: heading change (via forward-axis projection onto the horizontal
% plane) is within +-20 deg of a full reversal (+-180 deg). Allow either
% rotation direction.
    if nargin < 1, traj = run_immelmann_sim(); end
    dh = wrap180(traj.yaw_proj_deg(end) - traj.yaw_proj_deg(1));
    pass = abs(dh) >= 160 && abs(dh) <= 200;
    msg  = sprintf('heading change = %+.1f deg  (want |.| in [160, 200])', dh);
end

function w = wrap180(d)
    w = mod(d + 180, 360) - 180;
end
