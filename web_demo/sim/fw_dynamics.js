// web_demo/sim/fw_dynamics.js
// Port of models/fw_6dof_quat.m (Edge 540 6-DOF fixed-wing).
// state = [x, y, z, u, v, w, q0, q1, q2, q3, p, q, r]  (13)
//   - position in NED (m)
//   - velocity in body frame (m/s)
//   - quaternion body->NED, scalar-first (q0, q1, q2, q3)
//   - body angular rates (rad/s)
// controls: thrust (N), elevator/aileron/rudder (rad)

export function edge540Params() {
  return {
    m: 290,
    J: [550, 750, 1100],         // diagonal inertia [Jxx, Jyy, Jzz]
    S: 9.1, b: 7.44, c: 1.22,
    rho: 1.225, g: 9.81,
    CL0: 0.4, CL_alpha: 5.7, CL_q: 7.0, CL_de: -0.8,
    CD0: 0.04, k: 0.05, CDa: 0.1, CD_q: 0.0, CD_de: 0.0,
    CY_beta: -1.2, CY_p: -0.1, CY_r: 0.2, CY_da: 0.2, CY_dr: -0.2,
    Cl_beta: -0.15, Cl_p: -1.0, Cl_r: 0.25, Cl_da: 0.5, Cl_dr: 0.05,
    Cm0: 0.0, Cm_alpha: -1.5, Cm_q: -15.0, Cm_de: -1.8,
    Cn_beta: 0.15, Cn_p: -0.1, Cn_r: -0.4, Cn_da: 0.04, Cn_dr: -0.1,
  };
}

function quatToR_BN(q) {
  // Body-to-NED rotation matrix; scalar-first convention.
  const q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
  return [
    1 - 2*(q2*q2 + q3*q3), 2*(q1*q2 - q0*q3),     2*(q1*q3 + q0*q2),
    2*(q1*q2 + q0*q3),     1 - 2*(q1*q1 + q3*q3), 2*(q2*q3 - q0*q1),
    2*(q1*q3 - q0*q2),     2*(q2*q3 + q0*q1),     1 - 2*(q1*q1 + q2*q2),
  ];
}

function matVec3(M, v) {
  return [
    M[0]*v[0] + M[1]*v[1] + M[2]*v[2],
    M[3]*v[0] + M[4]*v[1] + M[5]*v[2],
    M[6]*v[0] + M[7]*v[1] + M[8]*v[2],
  ];
}

function matTransposeVec3(M, v) {
  return [
    M[0]*v[0] + M[3]*v[1] + M[6]*v[2],
    M[1]*v[0] + M[4]*v[1] + M[7]*v[2],
    M[2]*v[0] + M[5]*v[1] + M[8]*v[2],
  ];
}

function cross3(a, b) {
  return [
    a[1]*b[2] - a[2]*b[1],
    a[2]*b[0] - a[0]*b[2],
    a[0]*b[1] - a[1]*b[0],
  ];
}

// Accept J as either a 3-element diagonal array [Jxx, Jyy, Jzz]
// or a 3x3 matrix (row-major flat array of 9, or array-of-arrays).
function inertiaDiagonal(J) {
  if (Array.isArray(J) && J.length === 3 && typeof J[0] === 'number') {
    return [J[0], J[1], J[2]];
  }
  if (Array.isArray(J) && J.length === 3 && Array.isArray(J[0])) {
    return [J[0][0], J[1][1], J[2][2]];
  }
  if (Array.isArray(J) && J.length === 9) {
    return [J[0], J[4], J[8]];
  }
  throw new Error('Unsupported J inertia format');
}

export function fwDerivative(state, thrust, elevator, aileron, rudder, params) {
  const u = state[3], v = state[4], w = state[5];
  let q = [state[6], state[7], state[8], state[9]];
  const nq = Math.hypot(q[0], q[1], q[2], q[3]) + 1e-9;
  q = [q[0]/nq, q[1]/nq, q[2]/nq, q[3]/nq];
  const p = state[10], qr = state[11], r = state[12];

  const { m, S, b, c, rho, g } = params;
  const Jd = inertiaDiagonal(params.J);

  const R_BN = quatToR_BN(q);

  const v_b = [u, v, w];
  const v_ned = matVec3(R_BN, v_b);

  const Va = Math.max(1e-3, Math.hypot(u, v, w));
  const alpha = Math.atan2(w, u);
  const beta  = Math.asin(Math.max(-1, Math.min(1, v / Va)));
  const qbar  = 0.5 * rho * Va * Va;
  const p_hat = (b / (2 * Va)) * p;
  const q_hat = (c / (2 * Va)) * qr;
  const r_hat = (b / (2 * Va)) * r;

  const CL = params.CL0 + params.CL_alpha * alpha + params.CL_q * q_hat + params.CL_de * elevator;
  const CD = params.CD0 + params.k * CL * CL + params.CDa * alpha + params.CD_q * q_hat + params.CD_de * elevator;
  const CY = params.CY_beta * beta + params.CY_p * p_hat + params.CY_r * r_hat + params.CY_da * aileron + params.CY_dr * rudder;
  const Cl = params.Cl_beta * beta + params.Cl_p * p_hat + params.Cl_r * r_hat + params.Cl_da * aileron + params.Cl_dr * rudder;
  const Cm = params.Cm0 + params.Cm_alpha * alpha + params.Cm_q * q_hat + params.Cm_de * elevator;
  const Cn = params.Cn_beta * beta + params.Cn_p * p_hat + params.Cn_r * r_hat + params.Cn_da * aileron + params.Cn_dr * rudder;

  const Lift = qbar * S * CL;
  const Drag = qbar * S * CD;
  const Side = qbar * S * CY;
  const ca = Math.cos(alpha), sa = Math.sin(alpha);

  const F_aero_b = [-Drag*ca + Lift*sa, Side, -Drag*sa - Lift*ca];
  const M_aero_b = [qbar * S * b * Cl, qbar * S * c * Cm, qbar * S * b * Cn];

  const F_thrust_b = [thrust, 0, 0];
  // R_NB = R_BN^T; gravity in NED is [0, 0, g*m], rotated into body frame.
  const F_grav_b = matTransposeVec3(R_BN, [0, 0, g * m]);

  const F_b = [
    F_aero_b[0] + F_thrust_b[0] + F_grav_b[0],
    F_aero_b[1] + F_thrust_b[1] + F_grav_b[1],
    F_aero_b[2] + F_thrust_b[2] + F_grav_b[2],
  ];

  const omega = [p, qr, r];
  const cv = cross3(omega, v_b);
  const v_dot_b = [F_b[0]/m - cv[0], F_b[1]/m - cv[1], F_b[2]/m - cv[2]];

  const q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
  const q_dot = [
    -0.5 * (p*q1 + qr*q2 + r*q3),
     0.5 * (p*q0 + r*q2  - qr*q3),
     0.5 * (qr*q0 - r*q1 + p*q3),
     0.5 * (r*q0  + qr*q1 - p*q2),
  ];

  const Jw = [Jd[0]*p, Jd[1]*qr, Jd[2]*r];
  const omxJw = cross3(omega, Jw);
  const om_dot = [
    (M_aero_b[0] - omxJw[0]) / Jd[0],
    (M_aero_b[1] - omxJw[1]) / Jd[1],
    (M_aero_b[2] - omxJw[2]) / Jd[2],
  ];

  return [
    v_ned[0], v_ned[1], v_ned[2],
    v_dot_b[0], v_dot_b[1], v_dot_b[2],
    q_dot[0], q_dot[1], q_dot[2], q_dot[3],
    om_dot[0], om_dot[1], om_dot[2],
  ];
}
