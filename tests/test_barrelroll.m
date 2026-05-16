function pass = test_barrelroll()
% test_barrelroll  Verify config_barrelroll.m produces a proper helical
% barrel roll: 360 deg of roll with small heading drift and bounded altitude.

    checks = struct();
    checks.label              = 'Barrel roll';
    checks.roll_change_deg    = [330,  390];   % ~1 full revolution
    checks.heading_change_deg = [-30,   30];   % minor heading drift
    checks.alt_range_max_m    = 25;            % stays within +-25 m of initial alt
    checks.pitch_sweep_deg    = [ 10, 120];    % some pitch action during the maneuver
    pass = test_fw_maneuver('barrelroll', checks, 'barrelroll');
end
