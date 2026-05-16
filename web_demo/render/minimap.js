// web_demo/render/minimap.js
// Canvas2D top-down minimap showing trajectory traces for the FW and the
// drone. Auto-scales to fit the bounding box of recent history. North-up.

const HISTORY_LIMIT = 600;     // ~10 s at 60 Hz
const TRACE_STRIDE  = 1;       // sample every Nth render frame

const COLORS = {
  bg:        '#0a0e16',
  border:    'rgba(120,200,255,0.55)',
  grid:      'rgba(120,200,255,0.18)',
  fwTrace:   '#ff6666',
  droneTrace:'#66ccff',
  fwDot:     '#ff2222',
  droneDot:  '#00aaff',
  text:      '#d6f0ff',
};

let ctx, w, h;
let fwHistory   = [];
let droneHistory = [];
let frameCounter = 0;

/**
 * Bind to a Canvas2D context for the minimap.
 * @param {HTMLCanvasElement} canvas
 */
export function initMinimap(canvas) {
  const dpr = window.devicePixelRatio || 1;
  w = canvas.clientWidth;
  h = canvas.clientHeight;
  canvas.width  = w * dpr;
  canvas.height = h * dpr;
  ctx = canvas.getContext('2d');
  ctx.scale(dpr, dpr);
}

/**
 * Reset accumulated trajectory traces.
 */
export function resetMinimap() {
  fwHistory = [];
  droneHistory = [];
  frameCounter = 0;
}

/**
 * Push the latest FW + drone NED positions and redraw the minimap.
 * @param {number[]} fwPos    [N, E, D]
 * @param {number[]} dronePos [N, E, D]
 */
export function updateMinimap(fwPos, dronePos) {
  if (!ctx) return;
  if (frameCounter % TRACE_STRIDE === 0) {
    fwHistory.push([fwPos[0], fwPos[1]]);          // North, East only
    droneHistory.push([dronePos[0], dronePos[1]]);
    if (fwHistory.length    > HISTORY_LIMIT) fwHistory.shift();
    if (droneHistory.length > HISTORY_LIMIT) droneHistory.shift();
  }
  frameCounter++;
  draw();
}

function draw() {
  // Background + border
  ctx.fillStyle = COLORS.bg;
  ctx.fillRect(0, 0, w, h);

  // Compute bounding box of recent history; pad it to a min size so the
  // view doesn't snap when the trajectory is short.
  const all = fwHistory.concat(droneHistory);
  if (all.length === 0) { drawLabels(); return; }
  let xmin = Infinity, xmax = -Infinity, ymin = Infinity, ymax = -Infinity;
  for (const p of all) {
    if (p[0] < xmin) xmin = p[0];
    if (p[0] > xmax) xmax = p[0];
    if (p[1] < ymin) ymin = p[1];
    if (p[1] > ymax) ymax = p[1];
  }
  const padX = Math.max(50, (xmax - xmin) * 0.15);
  const padY = Math.max(50, (ymax - ymin) * 0.15);
  xmin -= padX; xmax += padX; ymin -= padY; ymax += padY;
  // Enforce a minimum span so it doesn't zoom in too tight
  const MIN_SPAN = 200;
  if (xmax - xmin < MIN_SPAN) { const c = (xmin + xmax)/2; xmin = c - MIN_SPAN/2; xmax = c + MIN_SPAN/2; }
  if (ymax - ymin < MIN_SPAN) { const c = (ymin + ymax)/2; ymin = c - MIN_SPAN/2; ymax = c + MIN_SPAN/2; }
  // Keep square aspect ratio
  const spanX = xmax - xmin, spanY = ymax - ymin;
  if (spanX > spanY) {
    const extra = (spanX - spanY) / 2;
    ymin -= extra; ymax += extra;
  } else {
    const extra = (spanY - spanX) / 2;
    xmin -= extra; xmax += extra;
  }

  // Convert world (N, E) -> minimap (x, y). North is up, East to the right.
  // World N goes UP visually -> screen y = h - (N - ymin) / span * h
  const margin = 20;
  const useW = w - margin * 2, useH = h - margin * 2;
  function tx(N) { return margin + (N - xmin) / (xmax - xmin) * useW; }
  // Wait — we want NORTH up. North is index 0 in [N,E,D]. Screen Y axis
  // points DOWN, so plotting North as screen-Y-up means: y = h-margin - frac*useH.
  // Plotting East as screen-X-right: x = margin + frac*useW.
  // But our tx() above uses the FIRST coord (N) for X. Recompute correctly.
  // Use this mapping instead:
  function toXY(p) {
    // p = [N, E]; minimap X = East scaled, minimap Y = North scaled (up)
    const fracE = (p[1] - ymin) / (ymax - ymin);
    const fracN = (p[0] - xmin) / (xmax - xmin);
    return [margin + fracE * useW, h - margin - fracN * useH];
  }
  // Actually the bbox above conflated coords; rebuild bbox separately for N and E.
  let nmin = Infinity, nmax = -Infinity, emin = Infinity, emax = -Infinity;
  for (const p of all) {
    if (p[0] < nmin) nmin = p[0];
    if (p[0] > nmax) nmax = p[0];
    if (p[1] < emin) emin = p[1];
    if (p[1] > emax) emax = p[1];
  }
  const padN = Math.max(50, (nmax - nmin) * 0.15);
  const padE = Math.max(50, (emax - emin) * 0.15);
  nmin -= padN; nmax += padN; emin -= padE; emax += padE;
  if (nmax - nmin < MIN_SPAN) { const c = (nmin + nmax)/2; nmin = c - MIN_SPAN/2; nmax = c + MIN_SPAN/2; }
  if (emax - emin < MIN_SPAN) { const c = (emin + emax)/2; emin = c - MIN_SPAN/2; emax = c + MIN_SPAN/2; }
  // Square aspect
  const sN = nmax - nmin, sE = emax - emin;
  if (sN > sE) { const ex = (sN - sE)/2; emin -= ex; emax += ex; }
  else         { const ex = (sE - sN)/2; nmin -= ex; nmax += ex; }
  function map(p) {
    const fracE = (p[1] - emin) / (emax - emin);
    const fracN = (p[0] - nmin) / (nmax - nmin);
    return [margin + fracE * useW, h - margin - fracN * useH];
  }

  // Grid
  ctx.strokeStyle = COLORS.grid;
  ctx.lineWidth   = 1;
  ctx.beginPath();
  for (let i = 1; i < 4; i++) {
    const gx = margin + (useW * i / 4);
    const gy = margin + (useH * i / 4);
    ctx.moveTo(gx, margin); ctx.lineTo(gx, h - margin);
    ctx.moveTo(margin, gy); ctx.lineTo(w - margin, gy);
  }
  ctx.stroke();

  // FW trace
  drawTrace(fwHistory, COLORS.fwTrace, 2, map);
  // Drone trace
  drawTrace(droneHistory, COLORS.droneTrace, 2, map);

  // Current-position dots
  if (fwHistory.length > 0) {
    const [px, py] = map(fwHistory[fwHistory.length - 1]);
    drawDot(px, py, 5, COLORS.fwDot);
  }
  if (droneHistory.length > 0) {
    const [px, py] = map(droneHistory[droneHistory.length - 1]);
    drawDot(px, py, 4, COLORS.droneDot);
  }

  drawLabels();
  drawScale(nmax - nmin);
}

function drawTrace(points, color, lineWidth, map) {
  if (points.length < 2) return;
  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.beginPath();
  const [x0, y0] = map(points[0]);
  ctx.moveTo(x0, y0);
  for (let i = 1; i < points.length; i++) {
    const [x, y] = map(points[i]);
    ctx.lineTo(x, y);
  }
  ctx.stroke();
}

function drawDot(x, y, r, color) {
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(x, y, r, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = 'white';
  ctx.lineWidth = 1;
  ctx.stroke();
}

function drawLabels() {
  ctx.fillStyle = COLORS.text;
  ctx.font = 'bold 13px monospace';
  ctx.textAlign = 'left';
  ctx.textBaseline = 'top';
  ctx.fillText('TRAJECTORY  N↑', 8, 6);
  // Legend
  ctx.font = '12px monospace';
  ctx.fillStyle = COLORS.fwTrace;     ctx.fillText('● FW',    8, h - 36);
  ctx.fillStyle = COLORS.droneTrace;  ctx.fillText('● Drone', 8, h - 20);
}

function drawScale(spanN) {
  ctx.fillStyle = COLORS.text;
  ctx.font = '11px monospace';
  ctx.textAlign = 'right';
  ctx.textBaseline = 'bottom';
  ctx.fillText(`${spanN.toFixed(0)} m wide`, w - 6, h - 6);
}
