function wrapped_angle = wrapToPi(angle)
% wrapToPi Wraps an angle to the interval [-pi, pi].
    wrapped_angle = mod(angle + pi, 2*pi) - pi;
end
