// web_demo/render/viewport.js
import * as THREE from 'three';

let renderer;
let insetRenderer;

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

/**
 * Initialise a SECOND WebGL renderer dedicated to the corner inset canvas.
 * @param {HTMLCanvasElement} canvas
 * @returns {THREE.WebGLRenderer | null}
 */
export function initInsetRenderer(canvas) {
  try {
    insetRenderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
    insetRenderer.setPixelRatio(window.devicePixelRatio);
    insetRenderer.setSize(canvas.clientWidth, canvas.clientHeight, false);
    insetRenderer.setClearColor(0x000000, 0);
    return insetRenderer;
  } catch (e) {
    console.warn('Inset renderer failed to init, hiding inset:', e);
    canvas.style.display = 'none';
    insetRenderer = null;
    return null;
  }
}

export function renderInset(scene, camera) {
  if (!insetRenderer) return;
  insetRenderer.render(scene, camera);
}
