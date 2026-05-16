# Ghost-Tracking DFL — Web Demo

Pilot a fixed-wing aircraft, watch the multicopter+gimbal camera POV match it in real time.

**Live:** https://abdelhakim96.github.io/Ghost_Tracking_DFL/

## Controls

| Key | Action |
| --- | --- |
| ↑ / ↓ | Pitch up / down |
| ← / → | Roll left / right |
| A / D | Rudder left / right |
| W / S | Increase / decrease thrust |
| R | Reset |
| P | Pause / resume |

## Run locally

```bash
cd web_demo
python3 -m http.server 8000     # or any static server
# open http://localhost:8000
```

## Tests

```bash
cd web_demo
npm install
npm test                # unit tests (Vitest), MATLAB-equivalence checks
npm run test:e2e        # Playwright browser smoke
```

## Architecture

See `../docs/superpowers/specs/2026-05-16-ghost-tracking-game-design.md`.

## Regenerating MATLAB ground-truth samples

```bash
matlab -batch "addpath('scripts'); extract_matlab_samples"
```

Writes JSON test fixtures into `tests/data/`. Re-run after any change to the
MATLAB reference implementation.
