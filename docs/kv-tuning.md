# Motor constant tuning

## All measurements

| Trajectory | Command | Cruise | Planned | Measured | Scale |
|---|---|---|---|---|---|
| tune_vy_slow | vy = 0.10 m/s | 2.0 s | 7.9 in  | 15 in    | 1.9x |
| tune_vy_mid  | vy = 0.25 m/s | 1.0 s | 9.8 in  | 32.5 in  | 3.3x |
| tune_vx      | vx = 0.35 m/s | 1.0 s | 13.8 in | 15 in    | 1.1x |
| tune_rotate  | ω  = 1.0 rad/s| 1.0 s | 57 deg  | 540 deg  | **9.47x** |

## Why translation measurements are unreliable here

tune_vy_mid and tune_vx both drive the wheels at ~8 rad/s. If kV were the only issue,
they'd overshoot by the same factor. They don't (3.3x vs 1.1x) — so the tape-measure
linear tests are dominated by confounding factors:
- Braking chunks cause backward bounce proportional to actual (not planned) speed
- Robot mass/inertia isn't in the motor model at all — the model assumes instantaneous
  speed, but a real robot takes time to accelerate and decelerate
- Net measured position absorbs all of this noise

## Why rotation is reliable

- Degrees accumulate cleanly — no linear coasting
- ALPHA_WHEEL geometry doesn't affect rotation (all 4 wheels run at R_chassis × ω)
- Symmetric: all wheels doing exactly the same job
- 540° vs 57° is a large, precise signal

## Derived constant

```
kV_new = kV_current / scale_factor
       = 0.139 / 9.47
       ≈ 0.0147  →  use 0.015
```

kS stays at 0.119 for now — it's a voltage offset and harder to isolate without slower,
more controlled tests. kA stays near 0.

## Next step

Apply kV = 0.015, re-run tune_rotate, and check if rotation is now close to 57°.
Then re-run a translation test to see if translation also improved.
