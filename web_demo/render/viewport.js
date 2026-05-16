// web_demo/render/viewport.js
import * as THREE from 'three';

let renderer;

export function initRenderer(canvas) {
  renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setScissorTest(true);
  window.addEventListener('resize', () => renderer.setSize(window.innerWidth, window.innerHeight));
  return renderer;
}

export function renderDual(scene, camLeft, camRight) {
  const W = window.innerWidth, H = window.innerHeight;
  const halfW = Math.floor(W / 2);
  camLeft.aspect  = halfW / H;
  camRight.aspect = halfW / H;
  camLeft.updateProjectionMatrix();
  camRight.updateProjectionMatrix();

  renderer.setViewport(0, 0, halfW, H);
  renderer.setScissor (0, 0, halfW, H);
  renderer.render(scene, camLeft);

  renderer.setViewport(halfW, 0, W - halfW, H);
  renderer.setScissor (halfW, 0, W - halfW, H);
  renderer.render(scene, camRight);
}
