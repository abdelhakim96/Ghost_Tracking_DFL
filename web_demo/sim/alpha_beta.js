// web_demo/sim/alpha_beta.js
// Auto-translated from DFL_controller/{alpha,beta}_gimbal_func3.m.
// state layout (0-based):
//   0..2   position
//   3..6   drone quaternion q0..q3
//   7..9   v_world
//   10..12 omega_body (p, q_body_pitch_rate, r)
//   13..15 gimbal (phi_g, theta_g, psi_g)
//   16,17  zeta, xi

/**
 * Alpha vector from the symbolic DFL controller (7-element).
 * @param {number[]} state  18-element drone state
 * @param {{Ap:number,Aq:number,Ar:number,Ix:number,Iy:number,Iz:number}} params
 * @param {number} zeta  total thrust
 * @param {number} xi    thrust derivative
 * @returns {number[]}   7-element alpha
 */
export function alphaGimbal(state, params, zeta, xi) {
  const { Ap, Aq, Ar, Ix, Iy, Iz } = params;
  const q0 = state[3], q1 = state[4], q2 = state[5], q3 = state[6];
  const p = state[10], qr = state[11], r = state[12];

  const t2 = p**2;
  const t3 = qr**2;
  const t4 = 1.0/zeta;

  return [
    zeta*(t2 + t3),
    -t4*(Ap*zeta + Ix*p*xi*2.0 - Ix*qr*r*zeta + Iy*qr*r*zeta - Iz*qr*r*zeta),
    -t4*(Aq*zeta + Iy*qr*xi*2.0 - Ix*p*r*zeta + Iy*p*r*zeta + Iz*p*r*zeta),
    (t4*(Ar*q0*zeta*-2.0 - Iz*p*q2*xi*4.0 + Iz*qr*q1*xi*4.0
         + Iz*q3*t2*zeta + Iz*q3*t3*zeta + Iz*q3*r**2*zeta
         - Ix*p*qr*q0*zeta*2.0 + Iy*p*qr*q0*zeta*2.0
         + Iz*p*q1*r*zeta*2.0 + Iz*qr*q2*r*zeta*2.0)) / (q0*2.0),
    0.0, 0.0, 0.0,
  ];
}

/**
 * Beta matrix from the symbolic DFL controller (7x7, row-major flat).
 * @param {number[]} state 18-element drone state
 * @param {{Ix:number,Iy:number,Iz:number,m:number}} params
 * @param {number} zeta  unused (state[16] is read directly); kept for parity
 * @param {number} xi    unused; kept for parity
 * @returns {number[]} 49-element row-major flat 7x7
 */
export function betaGimbal(state, params, zeta /* unused */, xi /* unused */) {
  const { Ix, Iy, Iz, m } = params;
  const q0 = state[3], q1 = state[4], q2 = state[5], q3 = state[6];
  const z = state[16];      // zeta from state

  const t2 = q0*q1;
  const t3 = q0*q2;
  const t4 = q0*q3;
  const t5 = q1*q2;
  const t6 = q1*q3;
  const t7 = q2*q3;
  const t8 = q0**2;
  const t9 = q1**2;
  const t10 = q2**2;
  const t11 = q3**2;
  const t12 = 1.0/q0;
  const t13 = 1.0/z;
  const t14 = -t9;
  const t15 = -t10;
  const t16 = -t11;
  const t17 = t8 + t9 + t10 + t11;
  const t18 = 1.0/(t17**2);

  // 49 entries, MATLAB column-major (matches reshape(..., [7,7]))
  const cm = [
    // column 1
    m*t18*(t3 + t6)*2.0,
    Ix*m*t13*t18*(t4 - t5)*2.0,
    Iy*m*t13*t18*(t8 + t9 + t15 + t16),
    -Iz*m*t12*t13*t18*(q0*t2 - q3*t3*2.0 + q2*t5 - q3*t6 + q1**3),
    0.0, 0.0, 0.0,
    // column 2
    m*t18*(t2 - t7)*-2.0,
    -Ix*m*t13*t18*(t8 + t10 + t14 + t16),
    Iy*m*t13*t18*(t4 + t5)*2.0,
    -Iz*m*t12*t13*t18*(q0*t3 + q3*t2*2.0 + q1*t5 - q3*t7 + q2**3),
    0.0, 0.0, 0.0,
    // column 3
    m*t18*(t8 + t11 + t14 + t15),
    Ix*m*t13*t18*(t2 + t7)*-2.0,
    Iy*m*t13*t18*(t3 - t6)*-2.0,
    Iz*m*q3*t12*t13*t18*(t9 + t10)*-2.0,
    0.0, 0.0, 0.0,
    // column 4
    0.0, 0.0, 0.0, Iz*t12*2.0, 0.0, 0.0, 0.0,
    // column 5
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    // column 6
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    // column 7
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
  ];

  // Transpose column-major -> row-major
  const rm = new Array(49);
  for (let r = 0; r < 7; r++) {
    for (let c = 0; c < 7; c++) {
      rm[r*7 + c] = cm[c*7 + r];
    }
  }
  return rm;
}
