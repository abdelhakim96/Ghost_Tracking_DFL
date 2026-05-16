// web_demo/render/scene.js
import * as THREE from 'three';

const SEA_SIZE = 20000;
const MOUNT_COUNT = 60;
const MOUNT_RADIUS_KM = 5;
const PALETTE = [0x4a6a3a, 0x5a7a48, 0x6b6a4a, 0x7a6a3a]; // brown-greens

export function buildScene() {
  const scene = new THREE.Scene();
  scene.add(makeSky());
  scene.add(makeSea());
  for (const m of makeMountains()) scene.add(m);
  scene.add(makeSun());
  scene.add(new THREE.AmbientLight(0x6080aa, 0.6));
  return scene;
}

function makeSky() {
  const geom = new THREE.SphereGeometry(8000, 32, 16);
  const mat  = new THREE.ShaderMaterial({
    side: THREE.BackSide,
    uniforms: {
      topColor:    { value: new THREE.Color(0x3a78c8) },
      bottomColor: { value: new THREE.Color(0xa8c8ee) },
    },
    vertexShader: `varying vec3 vPos;
      void main(){ vPos = position;
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position,1.0); }`,
    fragmentShader: `varying vec3 vPos;
      uniform vec3 topColor; uniform vec3 bottomColor;
      void main(){ float t = clamp(0.5 + 0.5 * normalize(vPos).y, 0.0, 1.0);
        gl_FragColor = vec4(mix(bottomColor, topColor, t), 1.0); }`,
  });
  return new THREE.Mesh(geom, mat);
}

function makeSea() {
  const geom = new THREE.PlaneGeometry(SEA_SIZE, SEA_SIZE, 1, 1);
  const mat  = new THREE.MeshPhongMaterial({ color: 0x1a4f7a, shininess: 60 });
  const mesh = new THREE.Mesh(geom, mat);
  mesh.rotation.x = -Math.PI / 2;
  mesh.position.y = 0;
  return mesh;
}

function makeMountains() {
  const rng = mulberry32(123);
  const out = [];
  for (let i = 0; i < MOUNT_COUNT; i++) {
    const r = 200 + rng() * (MOUNT_RADIUS_KM * 1000 - 200);
    const theta = rng() * Math.PI * 2;
    const x = r * Math.cos(theta);
    const z = r * Math.sin(theta);
    const height = 100 + rng() * 400;
    const radius = 80 + rng() * 220;
    const color  = PALETTE[Math.floor(rng() * PALETTE.length)];
    const geom = new THREE.ConeGeometry(radius, height, 5);
    const mat  = new THREE.MeshLambertMaterial({ color, flatShading: true });
    const mesh = new THREE.Mesh(geom, mat);
    mesh.position.set(x, height / 2, z);
    out.push(mesh);
  }
  return out;
}

function makeSun() {
  const sun = new THREE.DirectionalLight(0xffffff, 1.0);
  sun.position.set(1, 1, 0.5);
  return sun;
}

function mulberry32(seed) {
  return function() {
    seed |= 0; seed = (seed + 0x6D2B79F5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
