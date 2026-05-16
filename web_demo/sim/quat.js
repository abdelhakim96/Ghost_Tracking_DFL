// web_demo/sim/quat.js
// Scalar-first quaternions: q = [w, x, y, z]

/**
 * Hamilton product of two scalar-first quaternions.
 * @param {number[]} a  4-vector [w, x, y, z]
 * @param {number[]} b  4-vector [w, x, y, z]
 * @returns {number[]}  4-vector a (x) b
 */
export function quatMul(a, b) {
  const [a0, a1, a2, a3] = a;
  const [b0, b1, b2, b3] = b;
  return [
    a0*b0 - a1*b1 - a2*b2 - a3*b3,
    a0*b1 + a1*b0 + a2*b3 - a3*b2,
    a0*b2 - a1*b3 + a2*b0 + a3*b1,
    a0*b3 + a1*b2 - a2*b1 + a3*b0,
  ];
}

/**
 * Conjugate of a scalar-first quaternion.
 * @param {number[]} q  4-vector [w, x, y, z]
 * @returns {number[]}  4-vector [w, -x, -y, -z]
 */
export function quatConj(q) {
  return [q[0], -q[1], -q[2], -q[3]];
}

/**
 * Euclidean norm of a quaternion.
 * @param {number[]} q  4-vector [w, x, y, z]
 * @returns {number}    sqrt(w^2 + x^2 + y^2 + z^2)
 */
export function quatNorm(q) {
  return Math.hypot(q[0], q[1], q[2], q[3]);
}

/**
 * Normalize a quaternion to unit length (epsilon-guarded).
 * @param {number[]} q  4-vector [w, x, y, z]
 * @returns {number[]}  unit-norm 4-vector
 */
export function quatNormalize(q) {
  const n = quatNorm(q) + 1e-12;
  return [q[0]/n, q[1]/n, q[2]/n, q[3]/n];
}

/**
 * Convert a quaternion to a 3x3 rotation matrix, returned as a row-major flat
 * 9-array [r00, r01, r02, r10, r11, r12, r20, r21, r22].
 * @param {number[]} q  4-vector [w, x, y, z]
 * @returns {number[]}  row-major flat 9-array
 */
export function quatToR(q) {
  const [w, x, y, z] = quatNormalize(q);
  return [
    w*w + x*x - y*y - z*z,  2*(x*y - w*z),          2*(x*z + w*y),
    2*(x*y + w*z),          w*w - x*x + y*y - z*z,  2*(y*z - w*x),
    2*(x*z - w*y),          2*(y*z + w*x),          w*w - x*x - y*y + z*z,
  ];
}

/**
 * Rotate a 3-vector by a quaternion.
 * @param {number[]} q  4-vector [w, x, y, z]
 * @param {number[]} v  3-vector [x, y, z]
 * @returns {number[]}  rotated 3-vector
 */
export function quatRot(q, v) {
  const R = quatToR(q);
  return [
    R[0]*v[0] + R[1]*v[1] + R[2]*v[2],
    R[3]*v[0] + R[4]*v[1] + R[5]*v[2],
    R[6]*v[0] + R[7]*v[1] + R[8]*v[2],
  ];
}

/**
 * Minimum-rotation quaternion that maps unit vector a onto unit vector b.
 * Handles the antiparallel case by choosing an arbitrary orthogonal axis.
 * @param {number[]} a  unit 3-vector
 * @param {number[]} b  unit 3-vector
 * @returns {number[]}  unit quaternion [w, x, y, z]
 */
export function vecToQuat(a, b) {
  const dot = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
  if (dot > 0.999999) return [1, 0, 0, 0];
  if (dot < -0.999999) {
    let ax = [1, 0, 0];
    if (Math.abs(a[0]) > 0.9) ax = [0, 1, 0];
    const cx = [a[1]*ax[2] - a[2]*ax[1], a[2]*ax[0] - a[0]*ax[2], a[0]*ax[1] - a[1]*ax[0]];
    const n = Math.hypot(...cx);
    return [0, cx[0]/n, cx[1]/n, cx[2]/n];
  }
  const c = [a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]];
  const s = Math.sqrt((1 + dot) * 2);
  return quatNormalize([s/2, c[0]/s, c[1]/s, c[2]/s]);
}
