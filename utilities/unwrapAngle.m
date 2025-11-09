function unwrapped_angle = unwrapAngle(current_angle, last_angle)
% unwrapAngle Robustly unwraps an angle to prevent jumps.
% This function finds the multiple of 2*pi that brings the current_angle
% closest to the last_angle.

    k = round((last_angle - current_angle) / (2*pi));
    unwrapped_angle = current_angle + k*2*pi;
end
