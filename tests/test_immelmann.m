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
    checks.body_pitch_rotation_deg = [140, 220];   % half-loop in body-frame integration
    checks.body_roll_rotation_deg  = [140, 220];   % half-roll
    checks.heading_change_proj_deg = [140, 220];   % heading reversal
    checks.alt_change_m            = [30, 250];    % climbs
    checks.min_R33                 = [-1.0, -0.5]; % must have been inverted at some point
    checks.final_R33               = [0.7, 1.0];   % upright at the end
    pass = test_fw_maneuver('immelmann', checks, 'immelmann');
end
