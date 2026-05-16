function v_out = quatrotate_v(q, v)
% quatrotate_v Rotate a 3-vector v by a scalar-first quaternion q.
    q = q(:);
    q = q / max(norm(q), 1e-12);
    q0 = q(1); qv = q(2:4);
    v = v(:);
    v_out = v + 2*q0*cross(qv, v) + 2*cross(qv, cross(qv, v));
end
