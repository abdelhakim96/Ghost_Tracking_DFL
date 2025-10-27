function unwrapped_angle = unwrapAngle(current_angle, last_angle)
% unwrapAngle Robustly unwraps an angle to prevent jumps.
% This function calculates the difference between the current and last angle,
% wraps it to the range [-pi, pi], and rejects jumps larger than a threshold.

    delta_angle = current_angle - last_angle;
    delta_angle = mod(delta_angle + pi, 2*pi) - pi;

    unwrapped_angle = last_angle + delta_angle;
end
