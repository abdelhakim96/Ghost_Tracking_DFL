# Control-signal feasibility analysis (3-axis DFL, ω=30)

Computed by recomputing the full DFL control vector `u = [T̈, τ_φ, τ_θ, τ_ψ, φ̇_g, θ̇_g, ψ̇_g]`
along each stored trajectory (exact, via the real `unified_dynamics3` code path), then
comparing thrust, body rate, gimbal rate, and body torque against realistic actuator limits.

## Assumed limits (small 0.468 kg quad — state explicitly in paper)
- thrust-to-weight `T/W ≤ 4` (aggressive multirotor)
- body rate `|ω| ≤ 1500 deg/s`
- gimbal slew rate `|θ̇_g| ≤ 400 deg/s`

## Results

| maneuver | peak T/W | % time T/W>4 | peak body rate (°/s) | peak gimbal rate (°/s) | % time gimbal>400 |
|---|---|---|---|---|---|
| barrelroll | 8.4 | 55 | 7,500 | 7,500 | 25 |
| cuban8 | 38.8 | 47 | 10,097 | 10,006 | 13 |
| immelmann | 21.1 | 36 | 12,145 | 12,472 | 6 |
| loop | 4.0 | 0 | 33,996 | 23,130 | 96 |
| roll | 8.4 | 50 | 2,424 | 2,407 | 34 |
| rollsoft | 8.3 | 68 | 2,402 | 2,402 | 17 |
| splits | 33.5 | 80 | 13,552 | 17,929 | 13 |

(The extreme **peak** rate/torque values are brief transients at the `1/q0` and `1/cosθ_g`
representation singularities; the **% time exceeded** columns are the trustworthy sustained
measure. Body-torque peaks were ~324 N·m and a 3740 N·m loop outlier — both singular-instant
artifacts, not sustained demand.)

## Interpretation
- The computed commands are **largely physically infeasible** for a real small quad: thrust
  demands of 8–39× weight (realistic ≈4×), sustained 36–80% of the time on the aggressive
  maneuvers; gimbal/body rates in the thousands of °/s.
- This is the **feasible-set `F`** evidence and directly answers Reviewer 19 ("input saturation
  is assumed away"): the unconstrained DFL achieves near-perfect tracking precisely by demanding
  unrealizable control. The tracking is exact; the *reference* is outside `F`.
- Two drivers: (1) **scale mismatch** — a 0.468 kg quad emulating a 290 kg / 120 m/s aerobatic
  aircraft (the paper text claims the 10.6 kg Alta-X, but the sims use the 0.468 kg quad — fix
  this inconsistency); (2) **singular transients** from `1/q0` (body past 90°) and `1/cosθ_g`.

## Implications for the paper
1. **Make `F` constructive:** define `F = { references whose flat-inverse inputs satisfy
   0 ≤ T ≤ T_max, |τ| ≤ τ_max, |θ̇_g| ≤ θ̇_max, cosθ_g ≠ 0 }`; the mimicry theorem holds on `F`.
   Replaces the circular Authority Assumption 2 (Reviewer 20).
2. **Report these numbers** as the empirical `F`-boundary; show which maneuvers/platforms are
   feasible (e.g., a thrust-to-weight sweep per maneuver) rather than claiming all are.
3. **Fix the platform inconsistency** (use a consistent, realistic quad; re-derive feasibility).
4. **Harden the singularities** (`1/q0`, `1/cosθ_g`) or restrict references to stay clear of them
   (the paper already remarks a planner could avoid the singular set).

Regenerate: `feas2.m`-style recomputation (calls `unified_dynamics3` per saved sample, extracts
`u` from `state_dot`: `T̈=sd(18)`, `θ̇_g=sd(14:16)`, torques from `sd(11:13)` + gyroscopic terms).
