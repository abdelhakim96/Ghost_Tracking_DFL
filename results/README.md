# Raw simulation outputs

Each `results_<tag>.mat` holds a struct `out` with:

| field         | type            | meaning                                          |
| ------------- | --------------- | ------------------------------------------------ |
| `tag`         | char            | run tag (`baseline_roll`, `v3_loop`, …)          |
| `config`      | char            | trajectory config name (`roll`, `loop`, `rollsoft`) |
| `t`           | Nx1 double      | simulation time samples                          |
| `state`       | Nx30 / Nx31     | combined `[quad_state; fw_state]` per timestep   |
| `diverged`    | logical         | `true` if ode45 bailed out                       |
| `metrics`     | struct          | scalar + per-timestep error metrics              |

State layout:
- `baseline_*`, `fix_*`, `noyaw_*` → 17 quad + 13 FW = 30 columns (2-axis).
- `v3_*` → 18 quad + 13 FW = 31 columns (3-axis).

## Files

```
results/
├── results_baseline_roll.mat        original controller, full 360° barrel roll
├── results_baseline_loop.mat        original controller, full 360° loop
├── results_baseline_rollsoft.mat    original controller, ~53° bank rolling pull-up
├── results_fix_roll.mat             Phase 0+2+3 patched 2-axis stack
├── results_fix_loop.mat
├── results_fix_rollsoft.mat
├── results_noyaw_*.mat              Phase 0+2+3 with yaw schedule disabled (bisect run)
├── results_v3_roll.mat              Phase 1+5  (3-axis, camera-pose outputs)
├── results_v3_loop.mat
└── results_v3_rollsoft.mat
```

`.mat` files are gitignored — regenerate with `make_all_plots.m` or the individual `run_compare*('scenario', 'tag')` calls.
