// web_demo/render/scene.js
// Atmospheric Sky (Three.js Sky) + procedural heightmap terrain + sea + fog.
import * as THREE from 'three';
import { Sky } from 'https://unpkg.com/three@0.160.0/examples/jsm/objects/Sky.js';

const TERRAIN_SIZE = 24000;
const TERRAIN_SEG  = 240;
const SEA_LEVEL    = 0;
const SUN_ELEV_DEG = 28;
const SUN_AZ_DEG   = 90;

export function buildScene() {
  const scene = new THREE.Scene();
  // Mild fog so distant terrain is visible but still has atmospheric depth
  scene.fog = new THREE.Fog(0xcfe3ed, 2000, 18000);

  // Sky — Three.js atmospheric scattering (no external assets)
  const sky = new Sky();
  sky.scale.setScalar(45000);
  const u = sky.material.uniforms;
  u.turbidity.value      = 4;
  u.rayleigh.value       = 1.6;
  u.mieCoefficient.value = 0.004;
  u.mieDirectionalG.value = 0.7;
  const sunDir = sunDirection(SUN_ELEV_DEG, SUN_AZ_DEG);
  u.sunPosition.value.copy(sunDir);
  scene.add(sky);

  // Sun light aligned with sky
  const sun = new THREE.DirectionalLight(0xffeacb, 1.1);
  sun.position.copy(sunDir).multiplyScalar(1000);
  scene.add(sun);
  scene.add(new THREE.AmbientLight(0x7090b0, 0.65));

  // Sea — kept smaller than terrain so land dominates the view; brighter
  // teal-blue so it doesn't drown the scene
  const sea = new THREE.Mesh(
    new THREE.PlaneGeometry(40000, 40000, 1, 1),
    new THREE.MeshPhongMaterial({ color: 0x2c7396, shininess: 90, specular: 0xc8dfe8 }),
  );
  sea.rotation.x = -Math.PI / 2;
  sea.position.y = SEA_LEVEL - 5;     // sits just below terrain shore
  scene.add(sea);

  // Procedural noise-displaced terrain mesh
  scene.add(makeTerrain());

  return scene;
}

function makeTerrain() {
  const geom = new THREE.PlaneGeometry(TERRAIN_SIZE, TERRAIN_SIZE, TERRAIN_SEG, TERRAIN_SEG);
  geom.rotateX(-Math.PI / 2);

  const pos = geom.attributes.position;
  for (let i = 0; i < pos.count; i++) {
    const x = pos.getX(i);
    const z = pos.getZ(i);
    // Multi-octave noise — pushed up so most of the box is above sea level
    let h = 0;
    h += valueNoise(x * 0.00018, z * 0.00018) * 800;   // big rolling hills
    h += valueNoise(x * 0.00080, z * 0.00080) * 220;   // medium peaks
    h += valueNoise(x * 0.00350, z * 0.00350) * 60;    // small detail
    h += valueNoise(x * 0.01200, z * 0.01200) * 15;    // surface roughness
    // Bias: land sits between ~20 m and ~900 m, with occasional valleys
    // that dip to sea (allowing rivers/coast where noise is low).
    const land = Math.max(-30, h - 60);
    pos.setY(i, land);
  }
  pos.needsUpdate = true;
  geom.computeVertexNormals();

  // Vertex-colour gradient based on height: shore -> grass -> rock -> snow
  const colors = new Float32Array(pos.count * 3);
  const c0 = new THREE.Color(0x735542);
  const c1 = new THREE.Color(0x4a6a3a);
  const c2 = new THREE.Color(0x6a5d4a);
  const c3 = new THREE.Color(0xeef0f3);
  const tmp = new THREE.Color();
  for (let i = 0; i < pos.count; i++) {
    const y = pos.getY(i);
    let col;
    if      (y < 30)  col = tmp.copy(c0).lerp(c1, y / 30);
    else if (y < 200) col = tmp.copy(c1).lerp(c2, (y - 30) / 170);
    else if (y < 450) col = tmp.copy(c2).lerp(c3, (y - 200) / 250);
    else              col = tmp.copy(c3);
    colors[i*3 + 0] = col.r;
    colors[i*3 + 1] = col.g;
    colors[i*3 + 2] = col.b;
  }
  geom.setAttribute('color', new THREE.BufferAttribute(colors, 3));

  const mat = new THREE.MeshLambertMaterial({ vertexColors: true });
  return new THREE.Mesh(geom, mat);
}

function sunDirection(elevDeg, azDeg) {
  const e = elevDeg * Math.PI / 180;
  const a = azDeg   * Math.PI / 180;
  return new THREE.Vector3(
    Math.cos(e) * Math.cos(a),
    Math.sin(e),
    Math.cos(e) * Math.sin(a),
  );
}

// Smooth value noise via deterministic integer hash + bilinear interpolation.
function valueNoise(x, z) {
  const xi = Math.floor(x), zi = Math.floor(z);
  const xf = x - xi,        zf = z - zi;
  const a = hash2(xi,     zi);
  const b = hash2(xi + 1, zi);
  const c = hash2(xi,     zi + 1);
  const d = hash2(xi + 1, zi + 1);
  const u = smoothstep(xf);
  const v = smoothstep(zf);
  const ab = a * (1 - u) + b * u;
  const cd = c * (1 - u) + d * u;
  return (ab * (1 - v) + cd * v) * 2 - 1;
}

function smoothstep(t) { return t * t * (3 - 2 * t); }

function hash2(x, z) {
  let h = (x * 374761393) ^ (z * 668265263);
  h = (h ^ (h >>> 13)) * 1274126177;
  h = h ^ (h >>> 16);
  return ((h >>> 0) / 4294967295);
}
