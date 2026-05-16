function pass = test_cuban8()
% test_cuban8
%
%   Half Cuban-8 (5/8 loop + half-roll + 1/8 loop):
%     Phase A : pull through 5/8 of a loop (225 deg body-pitch)  -> 45 deg nose-down inverted
%     Phase B : half-roll (180 deg body-roll)                    -> 45 deg nose-down upright
%     Phase C : pull through 1/8 loop (45 deg body-pitch)        -> level
%   Net: heading reversed, altitude ~ initial.

    checks = struct();
    checks.label                  = 'Half Cuban-8';
    checks.body_pitch_rotation_deg = [220, 320];   % 5/8 + 1/8 of a loop
    checks.body_roll_rotation_deg  = [140, 220];   % half-roll
    % Half-Cuban-8 reverses heading; sign-agnostic magnitude check
    checks.abs_heading_change_proj_deg = [140, 220];
    checks.alt_change_m            = [-150, 200];
    checks.min_R33                 = [-1.0, -0.7];
    checks.final_R33               = [0.55, 1.0];
    pass = test_fw_maneuver('cuban8', checks, 'cuban8');
end
