# Journal Extension Plan — Fixed-Wing Dynamics Emulation on a Multicopter

**From:** CCTA 2026 submission #136 (rejected)
**To:** *Systems & Control Letters* — Special Issue *"Geometric, Structural, and Robust Methods in Systems and Control"* (Isidori 85th-birthday issue). **Deadline: 30 September 2026.**
**Decisions:** 3-axis gimbal primary (2-axis kept as structural analysis) · high-fidelity simulation only · soften stall claims to match evidence.

---

## 0. Strategic positioning

**Venue fit is excellent.** The method *is* Isidori's geometric framework: relative degree, the decoupling matrix `Δ(x)`, its structural singularity, and the **dynamic extension** that restores full relative degree (the paper already cites `Isidori1986CDC` and Isidori's textbook is the field's reference for exactly this). The CFP explicitly solicits *geometric methods*, *structural analysis of nonlinear systems*, *stability and feedback linearization*, and *applications in robotics/aerospace* — the paper hits all four. As an honor issue, an explicit, authentic connection to Isidori's geometric/structural program is a positive, not a stretch.

**Consequence for the plan:** the centre of gravity shifts from "experimental validation" to **geometric & structural depth**. Sim-only — the primary risk for T-RO — is *standard and accepted* for SCL. The bar is now: is the structural contribution rigorous, general, and clearly tied to the geometric approach. Robustness is also a named CFP theme, so we add a robustness/sensitivity thread to hit a second bullet.

The reviews remain recoverable: every concern is (a) solved by the 3-axis switch, (b) already computed in code but not surfaced, or (c) a narrative over-claim to scope back. The two harshest criticisms (R19 singularities, R20 circular assumption) become *structural contributions*.

**Sharpened contribution list (Isidori-framed; replaces the current 3-bullet list):**
1. **Structural analysis of cross-platform emulation as exact input–output decoupling.** Formalize physical-to-physical emulation as camera-pose mimicry on SE(3) and show the multicopter+gimbal plant has a *deficient* relative degree (`Σrᵢ = 9 ≠ 14`, `Δ(x)` singular ∀x) repaired by a **dynamic extension** (double-integrator on thrust) restoring `Σr̂ᵢ = 16`. This is a clean, novel instance of Isidori's geometric construction.
2. **Geometric/structural characterization of the feasible mimicry set `F`** — the set of reference trajectories over which the decoupling is well-posed — derived from the decoupling-matrix rank, the gimbal input-twist matrix `S` (`det S = cosφ_g cosθ_g`), and actuator limits. This *removes* the circular "Authority" assumption (R20 major) with a structural condition.
3. **Structural comparison of 2-axis vs 3-axis gimbals.** The input-twist/decoupling matrix loses rank on the codim-1 set `cosφ_g cosθ_g = 0` (2-axis) vs a single surface `cosθ_g = 0` (3-axis): the third axis makes the construction globally well-defined off one surface — *resolving* R19's singularity criticism as a structural result.
4. **Robustness of the linearizing feedback** under model mismatch, disturbance (wind) and measurement noise, quantified by Monte-Carlo over 7 aerobatic maneuvers with full metrics and control-input histories (hits the issue's *robust methods* theme).

**Residual risk:** now *theoretical*, not experimental — the structural results must be genuinely general/rigorous (not a worked example), and the application must not read as "DFL applied once more." Mitigation = lead with the structural/geometric generality (the deficient-relative-degree → dynamic-extension story and the `F` characterization), keep aerospace application as the motivating instance.

---

## 1. Reviewer-concern → fix traceability

| # | Reviewer | Concern | Fix | Where |
|---|---|---|---|---|
| 1 | AE | Vague requirements → subjective eval | Add **Requirements & Verification** subsection: quantitative success criteria stated up front, verified in results | §Results intro |
| 2 | AE / R19 | Novelty unclear | Sharpened 4-point contribution list; explicit positioning vs MPC tracking & similarity theory | §Intro |
| 3 | AE / R19 / R20 | Simulation only | Acceptable for SCL (theory journal); still strengthen with actuator dynamics, limits, noise, wind, Monte-Carlo robustness; brief "path to hardware" | §Method, §Results |
| 4 | R19 | Singularities assumed away (Ass. 1) | 3-axis: `det J_G = cosθ_g` only — single pitch singularity, no roll singularity, global elsewhere; bounded inside `F` | §III (proof) |
| 5 | R19 | Input saturation assumed away (Ass. 2) | Feasible-set `F` derived *from* the limits; control-input plots show limits respected | §III, §Results |
| 6 | R20 | Stall selling-point never demonstrated | **Soften**: reword motivation; move aero-fidelity/stall to Limitations; keep cost/safety/maneuverability value prop | §Intro, §Conclusion |
| 7 | R20 | Mimicry proof circular (Ass. 2) | Replace with constructive `F` characterization (contribution #2) | §III.B |
| 8 | R20 | Control inputs never shown | Add input-history figures (T, τ_φ, τ_θ, τ_ψ, gimbal rates) with saturation lines | §Results |
| 9 | R20 | No quantitative metrics | Metrics table (RMSE, max, steady-state, per-axis) + singularity margin — already in `compute_metrics3` | §Results |

---

## 2. Theory upgrades (the heart of the journal version)

### 2.1 Replace the circular Authority assumption with a constructive feasible set `F`  *(kills R20 major)*
Currently Assumption 2 says "limits are not violated along trajectories of interest" — circular. Replace with an explicit, checkable envelope on the **fixed-wing reference** `y_AC(t)`:

- **Thrust envelope:** the commanded multicopter acceleration `a_M = p̈_AC + (lever terms)` must satisfy `0 ≤ ||a_M + g e3|| ≤ T_max/m`. → bounds reference acceleration magnitude.
- **Tilt-rate / torque envelope:** reorienting the thrust vector to track curvature bounds the reference **jerk/snap**, mapped through the torque limits `τ_max` and `J_M`.
- **Gimbal envelope:** `q_rel = q̄_M ⊗ q_A` must stay clear of `θ_g = ±π/2` (det `J_G = cosθ_g`), and the gimbal-rate demand `ω_demand = R_gb·ω_A − ω_M` must satisfy `||θ̇_g|| ≤ θ̇_max`.

Define `F = { y_AC(t) : envelopes (a)+(b)+(c) hold ∀t }`. **Theorem (restated):** for all `y_AC ∈ F`, multicopter+3-axis-gimbal inputs exist realizing `y_MC(t) ≡ y_AC(t)`. This is constructive and verifiable — and the `min·cosθ_g` column we already compute is the empirical certificate that each maneuver lies in `F` (splits at 0.097 sits on the boundary — use it as the worked illustration of `F`'s edge).

### 2.2 3-axis global controllability  *(resolves R19)*
With the 3-axis gimbal, `S ≡ I₃` for the roll/pitch/yaw camera DOF and the only singular surface is `θ_g = ±π/2`. State the controllability result globally on SE(3) minus that surface; the 2-axis result (current paper) becomes a **Remark/limitation** that motivates the third axis.

### 2.3 Keep the DFL dynamic-extension derivation (it's solid)
The double-integrator thrust extension (`r̂ = [4 4 4 2 1 1]`, `N̂ = 16`) is good and unchallenged — keep, but extend the relative-degree bookkeeping to the 3-axis (7-input) plant to match `dfl_controller3.m`.

---

## 3. Results / experiment upgrades

### 3.1 Requirements & Verification subsection *(kills AE concern)*
State up front, e.g.: camera-pose orientation error `< 0.5°` RMS, position error `< 2 m` peak / `< 0.5 m` RMS, actuator commands within Alta-X limits, gimbal singularity margin `cosθ_g > 0.1`. Then the results table *verifies* against these — converting "subjective" to requirement-driven.

### 3.2 Master metrics table (replaces visual-only evaluation)
Core numbers already extracted from `results_v3_*.mat`:

| Maneuver | t (s) | RMSE pos (m) | max pos (m) | RMSE orient (°) | max orient (°) | min·cosθ_g |
|---|---|---|---|---|---|---|
| barrel roll | 2.5 | 0.31 | 0.49 | ~0.00 | 0.002 | 0.47 |
| cuban8 | 5.0 | 1.21 | 2.85 | ~0.00 | 0.003 | 0.24 |
| immelmann | 7.5 | 0.73 | 2.04 | ~0.00 | 0.003 | 0.62 |
| loop | 5.5 | 0.26 | 0.49 | ~0.00 | 0.006 | 0.36 |
| roll | 1.1 | 0.17 | 0.40 | 0.002 | 0.077 | 0.19 |
| rollsoft | 1.5 | 0.52 | 0.82 | ~0.00 | 0.003 | 0.84 |
| splits | 3.0 | 1.37 | 2.78 | 0.001 | 0.13 | 0.10 |

(Extend `compute_metrics3` to also emit RMSE, per-axis breakdown, steady-state error, and input-effort stats.)

### 3.3 HIL-grade simulation (addresses "sim-only")
Add to the plant (`unified_dynamics3.m` / quad dynamics):
- **Actuator limits:** thrust saturation `[0,T_max]`, body-torque limits, gimbal angle & **rate** limits.
- **Motor dynamics:** first-order lag on thrust/torque.
- **Sensor noise:** IMU/pose measurement noise on the feedback path.
- **Wind disturbance:** Dryden gust model as an external force/torque.
- **Monte-Carlo:** N runs per maneuver over randomized IC + wind + noise; report mean ± std of each metric (shaded bands in plots).

### 3.4 Control-input histories *(R20 minor)*
Log `u = [T̈→T, τ_φ, τ_θ, τ_ψ, φ̇_g, θ̇_g, ψ̇_g]` along each trajectory (re-evaluate `dfl_controller3` on the saved state, or re-run with logging). Plot with saturation limits as dashed lines — visually proving inputs stay feasible.

---

## 4. Visual / figure plan (your "nice visuals" ask)

Color convention (keep): **fixed-wing = red, multicopter = blue, camera/gimbal = green**.

1. **Hero figure (new):** composite of the multicopter "ghost-tracking" the fixed-wing through a loop — body frames, gimbal, camera frustum, and a rendered camera-viewpoint inset showing the two streams are visually indistinguishable. (Reuse `web_demo` 3D assets / CAD.)
2. **3D trajectory figures:** time-colored gradient path, attitude triads at intervals, shaded error tube around the camera path. Vector PDF, IEEE fonts.
3. **Per-maneuver small-multiples:** clean 2×2 (3D · position-vs-time · orientation-error-vs-time · control inputs) per maneuver.
4. **Feasibility-envelope figure (new, ties to theory):** required vs available thrust/accel and gimbal-angle trajectory vs the singular set — visual proof trajectories lie in `F`.
5. **Monte-Carlo bands:** error-vs-time with mean line + shaded ±σ.
6. **Summary chart:** grouped bar / radar comparing all 7 maneuvers across metrics — one-glance result.
7. **2-axis vs 3-axis contrast (small):** show the 2-axis controller saturating/diverging on a full roll vs 3-axis succeeding — motivates the third axis.

---

## 5. Work breakdown (mapped to code)

**Phase A — Theory & narrative (no new sims).** Reframe 3-axis primary; write feasible-set `F` characterization (§III.B rewrite); add Requirements subsection; soften stall language in `Introduction.tex` + abstract; sharpen contributions; 2-axis → motivating remark. *Files:* `root.tex`, `Introduction.tex`, `Results.tex`.

**Phase B — Results infrastructure.** Extend `compute_metrics3` (RMSE, per-axis, steady-state, input stats); add actuator limits + motor lag + wind + noise to plant; add input logging; write Monte-Carlo driver. *Files:* `DFL_controller/`, `models/`, `run_compare3.m`, new `run_montecarlo.m`.

**Phase C — Re-run.** All 7 maneuvers under HIL-grade sim; regenerate `results_v3_*.mat` + metrics table. Watch: splits (`cosθ_g=0.10`) and roll (`0.19`) may need feasible-set clamping or exclusion under actuator limits.

**Phase D — Figures.** New publication-quality plotting (`utilities/plotting_journal.m`): hero, 3D, inputs, feasibility envelope, MC bands, summary, 2v3 contrast.

**Phase E — Integrate & review.** Rewrite Results; compile full paper; internal code-review + claim-verification pass; consistency check.

---

## 6. Open risks (SCL)
- **Theoretical generality** — primary risk now. The structural results must read as general (deficient relative degree → dynamic extension; `Δ`/`S` rank conditions; `F` characterization), not as a single worked aerospace example. Mitigation: state results for the general class, present the multicopter/fixed-wing as the instantiation.
- **"DFL applied once more" perception** — counter by foregrounding contributions #1–#3 (structural) and the explicit, authentic geometric connection to Isidori's program.
- **Actuator limits may break aggressive maneuvers** (splits near-singular, `cosθ_g=0.10`) — use as the `F`-boundary worked example rather than hiding it.
- **Hero render effort** — `web_demo` assets exist but compositing takes time; degrade gracefully to a clean 3D MATLAB render.

## 7. Timeline
Deadline **30 Sep 2026**; today ~22 Jun 2026 → ~14 weeks. Phases A→E are comfortably within this; front-load Phase A (theory/structural rewrite) since it carries the venue-fit and the hardest reviewer concern (R20 circular assumption), then B/C (sim + robustness), then D/E.
