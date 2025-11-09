function corrected_angle = correctAngleJump(current_angle, previous_angle)
    % correctAngleJump: Corrects for angle jumps of 360 degrees (2*pi)
    %
    % INPUTS:
    %   current_angle   - The current angle in radians
    %   previous_angle  - The angle from the previous time step in radians
    %
    % OUTPUTS:
    %   corrected_angle - The corrected angle in radians
    
    if isempty(previous_angle)
        corrected_angle = current_angle;
        return;
    end

    diff = current_angle - previous_angle;
    
    if abs(diff) > pi
        % Correct for a jump of 2*pi
        corrected_angle = current_angle - sign(diff) * 2 * pi;
    else
        corrected_angle = current_angle;
    end
end
