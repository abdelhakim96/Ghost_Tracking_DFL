function [pass, msg] = test_08_flying_back(traj)
% PASS: at end of simulation, FW world-frame velocity has a NEGATIVE x
% component (= "flying back" along -north direction, opposite of the
% initial heading). Magnitude check ensures it's actually moving fast,
% not just drifting near zero.
    if nargin < 1, traj = run_immelmann_sim(); end
    vx = traj.v_world(end, 1);
    vmag = norm(traj.v_world(end, :));
    pass = vx < -10 && vmag > 50;
    msg  = sprintf('final world v_x = %+.1f m/s,  |v| = %.1f m/s  (want v_x < -10, |v|>50)', vx, vmag);
end
