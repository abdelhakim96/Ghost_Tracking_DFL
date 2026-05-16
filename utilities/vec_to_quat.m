function q = vec_to_quat(a, b)
% vec_to_quat  Minimal-angle scalar-first quaternion rotating unit vector a onto unit vector b.
    a = a(:)/max(norm(a), 1e-12);
    b = b(:)/max(norm(b), 1e-12);
    d = dot(a, b);
    if d > 1 - 1e-9
        q = [1; 0; 0; 0];
        return;
    end
    if d < -1 + 1e-9
        % 180 deg rotation about any axis perpendicular to a
        if abs(a(1)) < 0.9
            ax = cross(a, [1;0;0]);
        else
            ax = cross(a, [0;1;0]);
        end
        ax = ax/norm(ax);
        q = [0; ax];
        return;
    end
    c = cross(a, b);
    s = sqrt((1 + d) * 2);
    q = [s/2; c/s];
    q = q/norm(q);
end
