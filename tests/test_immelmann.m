function pass = test_immelmann()
% test_immelmann
%
%   An Immelmann turn:
%     Phase A: half loop (180 deg of body-pitch rotation) -> inverted at top
%     Phase B: half roll (180 deg of body-roll rotation) -> right-side-up
%   Net effect: opposite heading, higher altitude, level again.
%
%   Tested with body-frame integrated rotation (gimbal-lock robust) and the
%   level-ness metric R(3,3) ∈ [-1, +1].

    checks = struct();
    checks.label                  = 'Immelmann turn';
    % Body-rotation integrals are modified by the post-maneuver autopilot
    % (which counter-rotates to hold attitude). Loosen those, keep the
    % iconic Immelmann criteria strict: heading reversal, inverted at top,
    % approximately upright at end.
    checks.body_pitch_rotation_deg = [80,  240];
    checks.body_roll_rotation_deg  = [120, 280];
    checks.heading_change_proj_deg = [160, 200];   % heading reversed to within +-20 deg of 180
    checks.alt_change_m            = [30, 300];    % climbs
    checks.min_R33                 = [-1.0, -0.5]; % must have been inverted at some point
    checks.final_R33               = [0.7, 1.0];   % approximately upright at end
    pass = test_fw_maneuver('immelmann', checks, 'immelmann');
end
