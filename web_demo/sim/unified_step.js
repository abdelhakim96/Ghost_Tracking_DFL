// web_demo/sim/unified_step.js
import { fwDerivative, edge540Params } from './fw_dynamics.js';
import { droneDerivative } from './drone_dynamics.js';
import { dflController } from './dfl_controller.js';
import { quatToR } from './quat.js';

const FW_PARAMS = edge540Params();

function addScaled(a, b, scale) {
  const out = new Array(a.length);
  for (let i = 0; i < a.length; i++) out[i] = a[i] + scale * b[i];
  return out;
}

function matVec3(M, v) {
  return [
    M[0]*v[0]+M[1]*v[1]+M[2]*v[2],
    M[3]*v[0]+M[4]*v[1]+M[5]*v[2],
    M[6]*v[0]+M[7]*v[1]+M[8]*v[2],
  ];
}

export function unifiedRK4Step({ fwState, droneState, controlInputs, gains, dt }) {
  const refsAt = (fw) => {
    const fwd = fwDerivative(fw, controlInputs.thrust, controlInputs.elevator,
                             controlInputs.aileron, controlInputs.rudder, FW_PARAMS);
    const q = fw.slice(6, 10);
    const R = quatToR(q);
    const v_dot_body = [fwd[3], fwd[4], fwd[5]];
    const ad_world = matVec3(R, v_dot_body);
    return { xd: fw.slice(0, 3), vd: fwd.slice(0, 3),
             ad: ad_world, jd: [0, 0, 0], sd: [0, 0, 0],
             fwd };
  };

  const k1_refs = refsAt(fwState);
  const k1_fw   = k1_refs.fwd;
  const k1_dr   = droneDerivative({ droneState, fwState,
                                    xd: k1_refs.xd, vd: k1_refs.vd, ad: k1_refs.ad,
                                    jd: k1_refs.jd, sd: k1_refs.sd, gains });

  const fw2 = addScaled(fwState, k1_fw, dt/2);
  const dr2 = addScaled(droneState, k1_dr, dt/2);
  const k2_refs = refsAt(fw2);
  const k2_fw   = k2_refs.fwd;
  const k2_dr   = droneDerivative({ droneState: dr2, fwState: fw2,
                                    xd: k2_refs.xd, vd: k2_refs.vd, ad: k2_refs.ad,
                                    jd: k2_refs.jd, sd: k2_refs.sd, gains });

  const fw3 = addScaled(fwState, k2_fw, dt/2);
  const dr3 = addScaled(droneState, k2_dr, dt/2);
  const k3_refs = refsAt(fw3);
  const k3_fw   = k3_refs.fwd;
  const k3_dr   = droneDerivative({ droneState: dr3, fwState: fw3,
                                    xd: k3_refs.xd, vd: k3_refs.vd, ad: k3_refs.ad,
                                    jd: k3_refs.jd, sd: k3_refs.sd, gains });

  const fw4 = addScaled(fwState, k3_fw, dt);
  const dr4 = addScaled(droneState, k3_dr, dt);
  const k4_refs = refsAt(fw4);
  const k4_fw   = k4_refs.fwd;
  const k4_dr   = droneDerivative({ droneState: dr4, fwState: fw4,
                                    xd: k4_refs.xd, vd: k4_refs.vd, ad: k4_refs.ad,
                                    jd: k4_refs.jd, sd: k4_refs.sd, gains });

  const newFw    = fwState.map((v, i) => v + dt/6 * (k1_fw[i] + 2*k2_fw[i] + 2*k3_fw[i] + k4_fw[i]));
  const newDrone = droneState.map((v, i) => v + dt/6 * (k1_dr[i] + 2*k2_dr[i] + 2*k3_dr[i] + k4_dr[i]));
  return { fwState: newFw, droneState: newDrone };
}

/**
 * Snapshot the DFL controller's u vector at the current (fwState, droneState)
 * and the given pilot controls. Independent of the integrator; for display only.
 * @param {object} args
 * @param {number[]} args.fwState
 * @param {number[]} args.droneState
 * @param {{ thrust:number, elevator:number, aileron:number, rudder:number }} args.controlInputs
 * @param {object} args.gains
 * @returns {number[]} 7-element u
 */
export function snapshotU({ fwState, droneState, controlInputs, gains }) {
  const fwd = fwDerivative(fwState, controlInputs.thrust, controlInputs.elevator,
                           controlInputs.aileron, controlInputs.rudder, FW_PARAMS);
  const q = fwState.slice(6, 10);
  const R = quatToR(q);
  const ad_world = matVec3(R, [fwd[3], fwd[4], fwd[5]]);
  return dflController({
    droneState, fwState,
    xd: fwState.slice(0, 3), vd: fwd.slice(0, 3), ad: ad_world,
    jd: [0, 0, 0], sd: [0, 0, 0],
    psid: 0, gains,
  });
}

export function defaultGains() {
  return {
    c0: 51150, c1: 51140, c2: 1150, c3: 150,
    c4: 1, c5: 1,
    c_phi: 50, c_theta: 50, c_psig: 50,
    c_q3: 100, c_q3_dot: 20,
  };
}

// Spawn altitude raised to 600 m so the new terrain is clearly visible below
// and the FW has room to manoeuvre without immediately clipping a hill.
export function initialFwState() {
  return [0, 0, -600,
          30, 0, 0,
          1, 0, 0, 0,
          0, 0, 0];
}

export function initialDroneState() {
  return [0, 0, -600,
          1, 0, 0, 0,
          30, 0, 0,
          0, 0, 0,
          0, 0, 0,
          0.468 * 9.81, 0];
}
