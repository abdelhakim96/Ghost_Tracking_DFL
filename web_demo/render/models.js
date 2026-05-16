// web_demo/render/models.js
// Procedural low-poly meshes for the inset 3D plot.
import * as THREE from 'three';

/**
 * Returns a simple low-poly fixed-wing aircraft group.
 * Span ~12 m, length ~8 m. Material is shared on the group so opacity can
 * be tuned via group.traverse(...).
 * @returns {THREE.Group}
 */
export function makeFwModel() {
  const mat = new THREE.MeshLambertMaterial({
    color: 0xcccccc, transparent: true, opacity: 0.35,
    flatShading: true, side: THREE.DoubleSide,
  });
  const group = new THREE.Group();

  const fuselage = new THREE.Mesh(new THREE.BoxGeometry(8, 1.5, 1.5), mat);
  group.add(fuselage);

  const wings = new THREE.Mesh(new THREE.BoxGeometry(2, 0.3, 12), mat);
  wings.position.set(0, 0.1, 0);
  group.add(wings);

  const tailV = new THREE.Mesh(new THREE.BoxGeometry(1.5, 2.5, 0.2), mat);
  tailV.position.set(-3.5, 1.2, 0);
  group.add(tailV);

  const tailH = new THREE.Mesh(new THREE.BoxGeometry(2, 0.2, 4), mat);
  tailH.position.set(-3.5, 0.3, 0);
  group.add(tailH);

  const nose = new THREE.Mesh(new THREE.ConeGeometry(0.6, 1.5, 6), mat);
  nose.position.set(4.5, 0, 0);
  nose.rotation.z = -Math.PI / 2;
  group.add(nose);

  return group;
}

/**
 * Returns a quadrotor model and its rotor mesh array so the caller can spin them.
 * @returns {{ group: THREE.Group, rotorMeshes: THREE.Mesh[] }}
 */
export function makeDroneModel() {
  const bodyMat = new THREE.MeshLambertMaterial({ color: 0x222831, flatShading: true });
  const armMat  = new THREE.MeshLambertMaterial({ color: 0x393e46, flatShading: true });
  const rotorMat = new THREE.MeshBasicMaterial({
    color: 0x9aa1aa, transparent: true, opacity: 0.7, side: THREE.DoubleSide,
  });

  const group = new THREE.Group();

  const body = new THREE.Mesh(new THREE.BoxGeometry(0.6, 0.3, 0.6), bodyMat);
  group.add(body);

  const rotorMeshes = [];
  for (let i = 0; i < 4; i++) {
    const angle = i * Math.PI / 2 + Math.PI / 4;
    const arm = new THREE.Mesh(new THREE.BoxGeometry(1.6, 0.08, 0.08), armMat);
    arm.position.set(0, 0, 0);
    arm.rotation.y = -angle;
    group.add(arm);

    const rx = Math.cos(angle) * 0.8;
    const rz = Math.sin(angle) * 0.8;
    const rotor = new THREE.Mesh(new THREE.CircleGeometry(0.5, 16), rotorMat);
    rotor.position.set(rx, 0.1, rz);
    rotor.rotation.x = -Math.PI / 2;
    group.add(rotor);
    rotorMeshes.push(rotor);
  }

  return { group, rotorMeshes };
}
