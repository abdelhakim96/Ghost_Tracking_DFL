function qc = quat_conj(q)
% quat_conj Conjugate of scalar-first quaternion.
    qc = [q(1); -q(2); -q(3); -q(4)];
end
