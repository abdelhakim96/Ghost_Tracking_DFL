// web_demo/render/scene.js
import * as THREE from 'three';

export function buildScene() {
  const scene = new THREE.Scene();
  // Sky gradient via a large inverted sphere
  const skyGeom = new THREE.SphereGeometry(5000, 32, 16);
  const skyMat  = new THREE.ShaderMaterial({
    side: THREE.BackSide,
    uniforms: { topColor: { value: new THREE.Color(0x3a78c8) }, bottomColor: { value: new THREE.Color(0xa8c8ee) } },
    vertexShader: `varying vec3 vPos; void main(){ vPos = position; gl_Position = projectionMatrix * modelViewMatrix * vec4(position,1.0); }`,
    fragmentShader: `varying vec3 vPos; uniform vec3 topColor; uniform vec3 bottomColor;
      void main(){ float t = clamp(0.5 + 0.5 * normalize(vPos).y, 0.0, 1.0); gl_FragColor = vec4(mix(bottomColor, topColor, t), 1.0); }`,
  });
  scene.add(new THREE.Mesh(skyGeom, skyMat));

  // Ground: large textured plane with grid
  const groundMat = new THREE.MeshBasicMaterial({ color: 0x4a6a3a });
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(10000, 10000), groundMat);
  ground.rotation.x = -Math.PI / 2;
  scene.add(ground);
  const grid = new THREE.GridHelper(10000, 200, 0x223322, 0x223322);
  grid.position.y = 0.01;
  scene.add(grid);

  // Reference cubes/spheres scattered across the area
  const rng = mulberry32(123);
  const colors = [0xff6666, 0x66ccff, 0xffd866, 0xc098ff, 0x66ff99, 0xff99cc];
  for (let i = 0; i < 30; i++) {
    const x = (rng() - 0.5) * 5000;
    const z = (rng() - 0.5) * 5000;
    const y = 30 + rng() * 200;
    const size = 30 + rng() * 50;
    const isCube = rng() > 0.5;
    const geo = isCube ? new THREE.BoxGeometry(size, size, size) : new THREE.SphereGeometry(size/2, 16, 12);
    const mat = new THREE.MeshLambertMaterial({ color: colors[i % colors.length] });
    const m = new THREE.Mesh(geo, mat);
    m.position.set(x, y, z);
    scene.add(m);
  }

  // Lights
  const sun = new THREE.DirectionalLight(0xffffff, 1.0);
  sun.position.set(1, 1, 0.5);
  scene.add(sun);
  scene.add(new THREE.AmbientLight(0x6080aa, 0.6));

  return scene;
}

function mulberry32(seed) {
  return function() {
    seed |= 0; seed = (seed + 0x6D2B79F5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
