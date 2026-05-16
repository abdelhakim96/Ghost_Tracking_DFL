// web_demo/sim/drone_dynamics.js
// Port of models/quadrotor_dynamics_realtime3.m
import { dflController } from './dfl_controller.js';
import { quatToR } from './quat.js';

const PARAMS = { m: 0.468, Ix: 0.0023, Iy: 0.0023, Iz: 0.0046, g: 9.81 };

/**
 * Drone+gimbal state derivative.
 * @param {object} args
 * @param {number[]} args.droneState  18-element drone state
 * @param {number[]} args.fwState     13-element FW state
 * @param {number[]} args.xd          3-element position setpoint
 * @param {number[]} args.vd          3-element velocity setpoint
 * @param {number[]} args.ad          3-element acceleration setpoint
 * @param {number[]} args.jd          3-element jerk setpoint
 * @param {number[]} args.sd          3-element snap setpoint
 * @param {object}   args.gains       controller gains
 * @returns {number[]} 18-element derivative
 */
export function droneDerivative({ droneState, fwState, xd, vd, ad, jd, sd, gains }) {
  const q_bw_raw = droneState.slice(3, 7);
  const n = Math.hypot(q_bw_raw[0], q_bw_raw[1], q_bw_raw[2], q_bw_raw[3]) + 1e-9;
  const q_bw = q_bw_raw.map(v => v / n);
  const [q0, q1, q2, q3] = q_bw;

  const v_w = droneState.slice(7, 10);
  const omega_b = droneState.slice(10, 13);
  const zeta = droneState[16], xi = droneState[17];

  const u = dflController({ droneState, fwState, xd, vd, ad, jd, sd, psid: 0, gains });

  const R_bw = quatToR(q_bw);
  const a_ = [
    R_bw[2] * zeta / PARAMS.m,
    R_bw[5] * zeta / PARAMS.m,
    R_bw[8] * zeta / PARAMS.m - PARAMS.g,
  ];

  const q_dot = [
    0.5 * (-q1*omega_b[0] - q2*omega_b[1] - q3*omega_b[2]),
    0.5 * ( q0*omega_b[0] - q3*omega_b[1] + q2*omega_b[2]),
    0.5 * ( q3*omega_b[0] + q0*omega_b[1] - q1*omega_b[2]),
    0.5 * (-q2*omega_b[0] + q1*omega_b[1] + q0*omega_b[2]),
  ];

  const om_dot = [
    u[1]/PARAMS.Ix + (omega_b[1]*omega_b[2]*(PARAMS.Iy - PARAMS.Iz))/PARAMS.Ix,
    u[2]/PARAMS.Iy - (omega_b[0]*omega_b[2]*(PARAMS.Ix - PARAMS.Iz))/PARAMS.Iy,
    u[3]/PARAMS.Iz + (omega_b[0]*omega_b[1]*(PARAMS.Ix - PARAMS.Iy))/PARAMS.Iz,
  ];

  return [
    v_w[0], v_w[1], v_w[2],
    q_dot[0], q_dot[1], q_dot[2], q_dot[3],
    a_[0], a_[1], a_[2],
    om_dot[0], om_dot[1], om_dot[2],
    u[4], u[5], u[6],
    xi,
    u[0],
  ];
}
