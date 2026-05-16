function q = quat_mul(a, b)
% quat_mul Hamilton product of two scalar-first quaternions a (x) b.
    a0=a(1); a1=a(2); a2=a(3); a3=a(4);
    b0=b(1); b1=b(2); b2=b(3); b3=b(4);
    q = [a0*b0 - a1*b1 - a2*b2 - a3*b3;
         a0*b1 + a1*b0 + a2*b3 - a3*b2;
         a0*b2 - a1*b3 + a2*b0 + a3*b1;
         a0*b3 + a1*b2 - a2*b1 + a3*b0];
end
