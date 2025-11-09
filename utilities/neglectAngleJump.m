function corrected_angles = neglectAngleJump(angles)
    % neglectAngleJump: Corrects for large jumps in angle history by replacing
    % the jump with the previous value.
    %
    % INPUTS:
    %   angles - A vector of angles in radians
    %
    % OUTPUTS:
    %   corrected_angles - The corrected vector of angles in radians

    if isempty(angles)
        corrected_angles = [];
        return;
    end

    corrected_angles = angles;
    threshold = pi/2; % Threshold to detect a jump

    for i = 2:length(angles)
        diff = corrected_angles(i) - corrected_angles(i-1);
        if abs(diff) > threshold
            corrected_angles(i) = corrected_angles(i-1);
        end
    end
end
