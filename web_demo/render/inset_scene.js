// web_demo/render/inset_scene.js
// A small standalone Three.js scene used by the corner inset. Renders the
// FW model (semi-transparent) and the drone model (opaque) from an orbiting
// camera so the viewer can see how well they coincide.
import * as THREE from 'three';
import { makeFwModel, makeDroneModel } from './models.js';

const ORBIT_RADIUS = 30;
const ORBIT_HEIGHT = 10;
const ORBIT_RATE   = 10;

export function buildInsetScene(aspect) {
  const scene = new THREE.Scene();
  scene.background = null;

  const fwModel = makeFwModel();
  scene.add(fwModel);

  const { group: droneModel, rotorMeshes } = makeDroneModel();
  scene.add(droneModel);

  const disc = new THREE.Mesh(
    new THREE.RingGeometry(2, 2.5, 32),
    new THREE.MeshBasicMaterial({ color: 0x666666, side: THREE.DoubleSide, transparent: true, opacity: 0.4 }),
  );
  disc.rotation.x = -Math.PI / 2;
  disc.position.y = -2.5;
  scene.add(disc);

  scene.add(new THREE.AmbientLight(0xffffff, 0.6));
  const sun = new THREE.DirectionalLight(0xffffff, 0.8);
  sun.position.set(5, 8, 4);
  scene.add(sun);

  const camera = new THREE.PerspectiveCamera(40, aspect, 0.5, 500);
  camera.position.set(ORBIT_RADIUS, ORBIT_HEIGHT, 0);
  camera.lookAt(0, 0, 0);

  return { scene, fwModel, droneModel, rotorMeshes, camera };
}

/**
 * Update inset poses and orbit camera angle.
 * @param {object} ins                  result of buildInsetScene
 * @param {number[]} fwStateBodyToNED   13-element FW state
 * @param {number[]} droneState         18-element drone state
 * @param {number} timeNowMs            performance.now()
 */
export function updateInset(ins, fwStateBodyToNED, droneState, timeNowMs) {
  const fwPosNED    = [fwStateBodyToNED[0], fwStateBodyToNED[1], fwStateBodyToNED[2]];
  const dronePosNED = [droneState[0],       droneState[1],       droneState[2]];

  const fwPos3    = [0, 0, 0];
  const dronePos3 = [
    dronePosNED[0] - fwPosNED[0],
    -(dronePosNED[2] - fwPosNED[2]),
    dronePosNED[1] - fwPosNED[1],
  ];

  ins.fwModel.position.set(...fwPos3);
  ins.droneModel.position.set(...dronePos3);

  const qFw = fwStateBodyToNED.slice(6, 10);
  setNEDQuaternion(ins.fwModel, qFw);

  const qDrone = droneState.slice(3, 7);
  setNEDQuaternion(ins.droneModel, qDrone);

  const dt = 1 / 60;
  for (const r of ins.rotorMeshes) r.rotation.z += dt * 60;

  const angle = (timeNowMs / 1000) * (ORBIT_RATE * Math.PI / 180);
  ins.camera.position.set(
    ORBIT_RADIUS * Math.cos(angle),
    ORBIT_HEIGHT,
    ORBIT_RADIUS * Math.sin(angle),
  );
  ins.camera.lookAt(0, 0, 0);
}

function setNEDQuaternion(obj3D, qBodyToNED) {
  const qNedToThree = [Math.SQRT1_2, Math.SQRT1_2, 0, 0];
  const q = quatMul(qNedToThree, qBodyToNED);
  obj3D.quaternion.set(q[1], q[2], q[3], q[0]);
}

function quatMul(a, b) {
  return [
    a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
    a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
    a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
    a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0],
  ];
}
