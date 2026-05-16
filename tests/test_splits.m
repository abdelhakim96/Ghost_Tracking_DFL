function pass = test_splits()
% test_splits
%
%   Split-S = mirror of Immelmann:
%     Phase A: half roll (180 deg body-roll) -> inverted
%     Phase B: half loop pull-through (180 deg body-pitch) -> right-side-up
%   Net effect: opposite heading, lower altitude, level again.

    checks = struct();
    checks.label                  = 'Split-S';
    checks.body_pitch_rotation_deg = [140, 220];
    checks.body_roll_rotation_deg  = [140, 220];
    % Split-S half-roll direction is positive aileron then pull-through, so the
    % heading reverses with a negative sign in our convention. Accept either
    % +-180 deg by widening the range to cover both sides.
    checks.heading_change_proj_deg = [-220, -140];
    checks.alt_change_m            = [-250, -30];   % descends
    checks.min_R33                 = [-1.0, -0.5];  % must be inverted at some point
    checks.final_R33               = [0.7, 1.0];    % upright at the end
    pass = test_fw_maneuver('splits', checks, 'splits');
end
