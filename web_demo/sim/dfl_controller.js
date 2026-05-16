// web_demo/sim/dfl_controller.js
// Port of DFL_controller/dfl_controller3.m
import { quatMul, quatConj, quatToR } from './quat.js';
import { alphaGimbal, betaGimbal } from './alpha_beta.js';

const FIXED_PARAMS = {
  Ap: 0, Aq: 0, Ar: 0,
  Ag_p: 0.01, Ag_q: 0.01, Ag_r: 0.01,
  Ix: 0.0023, Iy: 0.0023, Iz: 0.0046,
  Ig_x: 0.001, Ig_y: 0.001, Ig_z: 0.001,
  m: 0.468,
};

const GRAVITY = 9.81;

// state idx (0-based)
// 0..2  position xyz
// 3..6  drone quat
// 7..9  v_world
// 10..12 omega_body
// 13..15 phi_g, theta_g, psi_g
// 16,17  zeta, xi

/**
 * Run one DFL controller step.
 * @param {object} args
 * @param {number[]} args.droneState  18-element drone state
 * @param {number[]} args.fwState     13-element FW state
 * @param {number[]} args.xd          3-element position setpoint
 * @param {number[]} args.vd          3-element velocity setpoint
 * @param {number[]} args.ad          3-element acceleration setpoint
 * @param {number[]} args.jd          3-element jerk setpoint
 * @param {number[]} args.sd          3-element snap setpoint
 * @param {number} args.psid          yaw setpoint (unused in v2 stack)
 * @param {object} args.gains         {c0..c3, c_phi, c_theta, c_psig, c_q3, c_q3_dot}
 * @returns {number[]} 7-element control input
 */
export function dflController({ droneState, fwState, xd, vd, ad, jd, sd, psid, gains }) {
  const x_w = droneState.slice(0, 3);
  let q_bw = droneState.slice(3, 7);
  const v_w = droneState.slice(7, 10);
  const omega_b = droneState.slice(10, 13);
  const phi_g = droneState[13], theta_g = droneState[14], psi_g = droneState[15];
  const zeta = droneState[16], xi = droneState[17];

  const n = Math.hypot(q_bw[0], q_bw[1], q_bw[2], q_bw[3]) + 1e-9;
  q_bw = q_bw.map(v => v / n);
  const [q0, q1, q2, q3] = q_bw;
  const R_bw = quatToR(q_bw);

  const m = FIXED_PARAMS.m;
  const F_thrust = [R_bw[2]*zeta, R_bw[5]*zeta, R_bw[8]*zeta];
  const a_ = [F_thrust[0]/m, F_thrust[1]/m, F_thrust[2]/m - GRAVITY];
  const Rc0 = [R_bw[0], R_bw[3], R_bw[6]];
  const Rc1 = [R_bw[1], R_bw[4], R_bw[7]];
  const Rc2 = [R_bw[2], R_bw[5], R_bw[8]];
  const jx = (zeta*(Rc0[0]*omega_b[1] - Rc1[0]*omega_b[0]) + Rc2[0]*xi) / m;
  const jy = (zeta*(Rc0[1]*omega_b[1] - Rc1[1]*omega_b[0]) + Rc2[1]*xi) / m;
  const jz = (zeta*(Rc0[2]*omega_b[1] - Rc1[2]*omega_b[0]) + Rc2[2]*xi) / m;
  const j = [jx, jy, jz];

  const q3dot = 0.5 * (-q2*omega_b[0] + q1*omega_b[1] + q0*omega_b[2]);

  const v_pos = [
    sd[0] - gains.c3*(j[0]-jd[0]) - gains.c2*(a_[0]-ad[0]) - gains.c1*(v_w[0]-vd[0]) - gains.c0*(x_w[0]-xd[0]),
    sd[1] - gains.c3*(j[1]-jd[1]) - gains.c2*(a_[1]-ad[1]) - gains.c1*(v_w[1]-vd[1]) - gains.c0*(x_w[1]-xd[1]),
    sd[2] - gains.c3*(j[2]-jd[2]) - gains.c2*(a_[2]-ad[2]) - gains.c1*(v_w[2]-vd[2]) - gains.c0*(x_w[2]-xd[2]),
  ];
  const v_q3 = -gains.c_q3_dot * q3dot - gains.c_q3 * (q3 - 0);

  let q_fw = fwState.slice(6, 10);
  const nf = Math.hypot(q_fw[0], q_fw[1], q_fw[2], q_fw[3]) + 1e-9;
  q_fw = q_fw.map(v => v / nf);
  const q_rel = quatMul(quatConj(q_bw), q_fw);
  const [phi_ref, theta_ref, psi_ref] = quatToXYZEuler(q_rel);

  const R_fw = quatToR(q_fw);
  const R_gb_des = matMul3x3T(R_bw, R_fw);
  const fw_omega_b = fwState.slice(10, 13);
  const omega_cam_b = matVec3(R_gb_des, fw_omega_b);
  const om_demand = [
    omega_cam_b[0] - omega_b[0],
    omega_cam_b[1] - omega_b[1],
    omega_cam_b[2] - omega_b[2],
  ];

  // Solve Jg * dTheta = om_demand, where
  //   Jg = [ 1, 0,                  sin(theta);
  //          0, cos(phi),           -sin(phi)*cos(theta);
  //          0, sin(phi),            cos(phi)*cos(theta) ]
  // Closed form (away from theta = +-pi/2):
  //   dTheta[1] =  cos(phi)*om[1] + sin(phi)*om[2]
  //   dTheta[2] = (-sin(phi)*om[1] + cos(phi)*om[2]) / cos(theta)
  //   dTheta[0] =  om[0] - sin(theta)*dTheta[2]
  const cosT = Math.cos(theta_g), sinT = Math.sin(theta_g);
  const cosP = Math.cos(phi_g),   sinP = Math.sin(phi_g);
  let dTheta;
  if (Math.abs(cosT) > 1e-3) {
    const r1 = cosP*om_demand[1] + sinP*om_demand[2];
    const r2 = (-sinP*om_demand[1] + cosP*om_demand[2]) / cosT;
    const r0 = om_demand[0] - sinT * r2;
    dTheta = [r0, r1, r2];
  } else {
    dTheta = om_demand.slice();
  }
  const phi_g_dot_ref   = dTheta[0];
  const theta_g_dot_ref = dTheta[1];
  const psi_g_dot_ref   = dTheta[2];

  const v_phi   = -gains.c_phi   * (phi_g - phi_ref)   + phi_g_dot_ref;
  const v_theta = -gains.c_theta * (theta_g - theta_ref) + theta_g_dot_ref;
  const v_psig  = -gains.c_psig  * (psi_g - psi_ref)   + psi_g_dot_ref;

  const v = [v_pos[0], v_pos[1], v_pos[2], v_q3, v_phi, v_theta, v_psig];

  const alpha = alphaGimbal(droneState, FIXED_PARAMS, zeta, xi);
  const beta  = betaGimbal(droneState, FIXED_PARAMS, zeta, xi);   // 7x7 row-major
  const u = alpha.slice();
  for (let r = 0; r < 7; r++) {
    for (let c = 0; c < 7; c++) {
      u[r] += beta[r*7 + c] * v[c];
    }
  }
  return u;
}

function matVec3(M, v) {
  return [
    M[0]*v[0]+M[1]*v[1]+M[2]*v[2],
    M[3]*v[0]+M[4]*v[1]+M[5]*v[2],
    M[6]*v[0]+M[7]*v[1]+M[8]*v[2],
  ];
}

function matMul3x3T(A, B) {
  // A^T * B
  const out = new Array(9);
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      out[i*3+j] = A[0*3+i]*B[0*3+j] + A[1*3+i]*B[1*3+j] + A[2*3+i]*B[2*3+j];
    }
  }
  return out;
}

function quatToXYZEuler(q) {
  const R = quatToR(q);
  const sth = Math.max(-1, Math.min(1, R[2]));
  const theta = Math.asin(sth);
  let psi, phi;
  if (Math.abs(Math.cos(theta)) > 1e-6) {
    psi = Math.atan2(-R[1], R[0]);
    phi = Math.atan2(-R[5], R[8]);
  } else {
    psi = 0;
    phi = Math.atan2(R[7], R[4]);
  }
  return [phi, theta, psi];
}
