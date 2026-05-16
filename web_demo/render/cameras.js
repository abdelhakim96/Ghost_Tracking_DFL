// web_demo/render/cameras.js
import * as THREE from 'three';
import { quatMul } from '../sim/quat.js';

// q_cam_mount: rotates Three's camera-local frame to body frame.
//   Three(-Z) -> body(+X)
//   Three(+Y) -> body(-Z)
//   Three(+X) -> body(+Y)
// Derived rotation matrix has trace 0; standard conversion gives this quat:
export const Q_CAM_MOUNT = [0.5, -0.5, -0.5, 0.5];

// q_ned_to_three: rotates NED-world axes to Three-world axes.
//   NED(+X north) -> Three(+X)
//   NED(+Y east)  -> Three(+Z)
//   NED(+Z down)  -> Three(-Y)
// This is a 90 deg rotation about Three's +X axis.
export const Q_NED_TO_THREE = [Math.SQRT1_2, Math.SQRT1_2, 0, 0];

export function makeCamera(aspect) {
  return new THREE.PerspectiveCamera(75, aspect, 0.5, 10000);
}

// posNED -> Three position; quatBodyToNED -> Three camera quaternion.
export function setCameraPose(cam, qBodyToNED, posNED) {
  cam.position.set(posNED[0], -posNED[2], posNED[1]);
  const q = quatMul(Q_NED_TO_THREE, quatMul(qBodyToNED, Q_CAM_MOUNT));
  cam.quaternion.set(q[1], q[2], q[3], q[0]);
}

// gimbal_quat = q_x(phi) (x) q_y(theta) (x) q_z(psi)
export function gimbalQuat(phi, theta, psi) {
  const qx = [Math.cos(phi/2), Math.sin(phi/2), 0, 0];
  const qy = [Math.cos(theta/2), 0, Math.sin(theta/2), 0];
  const qz = [Math.cos(psi/2), 0, 0, Math.sin(psi/2)];
  return quatMul(quatMul(qx, qy), qz);
}
