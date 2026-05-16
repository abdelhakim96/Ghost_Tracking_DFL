# Plots

Generated comparison figures for the camera-mimicry controller. Three groups, ordered from least-invasive fix to paper-faithful.

```
plots/
├── 2axis_phase0+2+3/   ← baseline 2-axis stack vs. Phase 0+2+3 fixes
│   ├── roll_baseline_vs_fix.png
│   └── loop_baseline_vs_fix.png
├── 3axis_phase1+5/     ← Phase 1+5 (camera-pose outputs + 3-axis gimbal)
│   ├── roll_3d.png             3D trajectory + FW frame + camera frame
│   ├── roll_frames.png         mid-trajectory snapshot, frames side-by-side
│   ├── roll_orient_err.png     camera vs FW orientation error timeseries
│   ├── loop_3d.png
│   ├── loop_frames.png
│   └── loop_orient_err.png
└── 3way_comparison/    ← baseline vs Phase 0+2+3 vs Phase 1+5 (log scale)
    ├── roll_3way.png
    └── loop_3way.png
```

## Regenerate everything

```matlab
make_all_plots
```

(or step-by-step — see `make_all_plots.m`)

## What to look at first

1. `plots/3way_comparison/roll_3way.png` — log-scale orientation error: baseline + Phase 0+2+3 saturate near 100-180°; Phase 1+5 stays at ~10⁻³ deg for the full 360° roll.
2. `plots/3axis_phase1+5/roll_3d.png` — FW aircraft (red, semi-transparent) with its body-frame triad (solid R/G/B), and the camera frame at the drone position (dotted O/C/M). The two triads coincide along the entire trajectory.
3. `plots/3axis_phase1+5/roll_orient_err.png` — orientation error timeseries with log axis, showing the controller stays at the numerical-precision floor.
