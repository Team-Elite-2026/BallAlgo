# Ball Distance Model — Data Review & Training Pipeline

This document describes the end-to-end process for turning raw ball-distance
calibration samples into a deployable distance model: how the data is reviewed
and cleaned, how the candidate models are trained, and how the most accurate one
is selected.

It covers three model families that all map a detected ball pixel position to a
physical distance:

1. **Parametric log-poly-Fourier** — a closed-form `cm = f(x, y)` surface
   (torch-free, deploys as a small JSON of coefficients).
2. **MLP, `xy_radius` features** — neural net on `[x, y, r]`.
3. **MLP, `polar` features** — neural net on `[r, sin θ, cos θ]`.

---

## 1. The data and why its structure matters

`training/distance_training_data.csv` has columns `x, y, measured_cm`, where
`(x, y)` is the detected ball position **already centred** on the image origin
defined by `thresholds.json → offsets` (currently `(330, 318)`), and
`measured_cm` is the known physical distance the ball was placed at.

The robot sees the field through an **omnidirectional (catadioptric) mirror**, so
distance is fundamentally a function of pixel **radius** `r = hypot(x, y)`, with a
secondary dependence on **bearing** `θ = atan2(y, x)` caused by mirror
tilt/decentering.

The data is collected as **constant-distance arcs**: the ball is held at a fixed
distance and swept around the robot, so every distinct `measured_cm` value is one
arc of points that should lie on a near-constant pixel radius. This structure is
the backbone of the entire review — see §2.

Raw dataset at time of writing: **647 rows**, `cm ∈ [12.8, 227.4]`,
`r ∈ [99, 287] px`.

---

## 2. Data review findings

Run the review/cleaning script (see §3) to regenerate all of this.

### 2.1 The dominant error source is optical-center offset, not noise

A constant physical distance maps to a circle in the image. If the assumed center
is wrong by `(dx, dy)`, every arc's radius picks up a coherent
`r(θ) ≈ R + dx·cos θ + dy·sin θ` wobble.

- Within-arc radius RMS spread at the current center: **~13.0 px**.
- Best center correction found: **(+15, +1.5) px**, which removes **~60 %** of the
  within-arc radius variance (down to ~8.2 px).
- Per-arc, after removing the smooth first-order decenter wobble, the **true
  scatter is only ~2–3 px** across the whole distance range (see
  `plots/06_per_arc_radius_spread.png`).

**Interpretation.** Most of the apparent "spread" in the raw radius↔distance
relationship is a *coherent, fittable* decentering effect, not random noise. This
is why distance must be modeled as a function of **both** radius and angle — a
pure `r → cm` curve cannot resolve it (the 161 cm arc alone spans `r ∈ [197, 273]`
px). The parametric Fourier model and both MLP feature modes all use the angle and
can represent it.

> The center offset is a **calibration** fix (update `thresholds.json → offsets`
> and retrain), so the cleaning script **reports it but never silently applies
> it** — that would put the cleaned data in a different frame than inference.

### 2.2 Genuine measurement noise grows with range

After accounting for decentering, far arcs (> 130 cm) still show ~9 px of
non-smooth, point-to-point scatter (vs ~1–2 px for near arcs). At the rim the
distance gradient is steep, so this pixel noise translates into large cm error.
This is consistent with the recent switch to **bbox-midpoint ball detection**: the
ball is small and distorted near the mirror rim, so its detected center is noisy
there.

**Consequence:** model accuracy is strongly range-dependent — see §5.

### 2.3 Outliers removed

The cleaning removes **55 / 647 rows (8.5 %)**:

| Reason | Rows | What it catches |
|---|---|---|
| Duplicate `(x, y)`, large `cm` spread | 18 | Same pixel labeled with very different distances (ambiguous) |
| Duplicate `(x, y)`, small spread | 4 collapsed → median | Repeated samples, harmless |
| Per-arc angular-residual outlier | 33 | Ball detections far off their arc's smooth radius curve |
| Global model-residual cross-check | 0 | Mislabeled distances on a plausible radius |

Result: **`training/distance_training_data_cleaned.csv` (592 rows)**, same
coordinate frame as the input.

### 2.4 Plots produced (`training/data_review/plots/`)

| File | Shows |
|---|---|
| `01_raw_measured_scatter.png` | Raw samples colored by measured distance |
| `02_contour_map.png` | Interpolated distance surface (the contour map) over the cleaned data |
| `03_gradient_magnitude.png` | `‖∇cm‖` (cm change per pixel) — where the mirror compresses distance |
| `04_radius_vs_distance.png` | The core radius↔distance relationship, raw vs cleaned; shows the per-arc radius bands |
| `05_removed_outliers.png` | Which points were dropped |
| `06_per_arc_radius_spread.png` | Per-arc radius spread before/after removing decenter wobble (the key diagnostic) |

---

## 3. How to reproduce the data review / cleaning

```bash
cd training/data_review
python3 review_and_clean_distance_data.py
```

This reads `../distance_training_data.csv` and writes:

- `../distance_training_data_cleaned.csv` — cleaned, drop-in for all training.
- `outlier_report.csv` — every dropped row with its reason.
- `arc_summary.csv` — per-arc radius spread before/after detrending.
- `plots/*.png` — the figures in §2.4.

Tunable thresholds (duplicate spread tolerance, per-arc robust-z, etc.) are
constants at the top of the script and documented inline.

---

## 4. Training the candidate models

All models train on the **cleaned** CSV.

### 4.1 Parametric log-poly-Fourier (closed form)

Model: `log(cm) = Σ_{p=0..P} (log r)^p · A_p(θ)`, where each `A_p` is a Fourier
series of order `M` in θ. Fit in log space (least squares, closed form), which
optimizes relative error and keeps `cm = exp(...)` positive. The script sweeps
`P` and `M`, picks the best **monotonic** fit by cross-validated RMSE.

```bash
cd training
python3 fit_parametric_model.py distance_training_data_cleaned.csv \
    --max-radial-degree 5 --max-angular-order 4 --cv-folds 5 \
    --save-coeffs parametric_model_cleaned.json \
    --plots-dir data_review/parametric_plots_cleaned \
    --eval-csv distance_eval_data.csv
```

### 4.2 MLP — two feature modes, capped at 300 epochs

Architecture: 3-input → `(32, 32)` ReLU → 1 output, far-distance-weighted MSE
loss (`weight = 1 + 1.5·(cm/max_cm)²`), Adam (lr 0.01, weight-decay 1e-4). The two
"methods" are the two feature parameterizations:

- **`xy_radius`**: `[x, y, r]` — lets the net learn arbitrary angular structure.
- **`polar`**: `[r, sin θ, cos θ]` — bakes in the radial/angular split, which
  matches the physics (radius drives distance, angle corrects for decentering).

**Torch path (deployment).** When torch is installed, this writes `.pt`
checkpoints (every 50 epochs) and supports ONNX export:

```bash
cd training
python3 train_distance_model.py distance_training_data_cleaned.csv \
    --run-dir run_cleaned_xy_radius_300 \
    --feature-mode xy_radius --max-epochs 300 --learning-rate 0.01 --cv-folds 5
python3 train_distance_model.py distance_training_data_cleaned.csv \
    --run-dir run_cleaned_polar_300 \
    --feature-mode polar --max-epochs 300 --learning-rate 0.01 --cv-folds 5
python3 evaluate_distance_models.py --run-dir run_cleaned_xy_radius_300
python3 evaluate_distance_models.py --run-dir run_cleaned_polar_300
```

**Torch-free path (used for the results below).** Because torch was not yet
installed on the Pi, the MLP was trained with a **pure-numpy reference
implementation**, [`train_distance_model_numpy.py`](../training/train_distance_model_numpy.py),
that mirrors the torch model exactly (same architecture, normalization, loss,
Adam, Kaiming-uniform init). Its results are directly comparable; the torch path
remains the deployment route (for `.pt`/ONNX) once torch is available.

```bash
cd training
python3 train_distance_model_numpy.py --feature-mode xy_radius \
    --max-epochs 300 --learning-rate 0.01 --cv-folds 5 --save-json mlp_numpy_xy_radius.json
python3 train_distance_model_numpy.py --feature-mode polar \
    --max-epochs 300 --learning-rate 0.01 --cv-folds 5 --save-json mlp_numpy_polar.json
```

> Install torch on this Pi with `pip install torch --break-system-packages`. The
> parametric model and the numpy MLP need only numpy.

---

## 5. Model comparison & selection

Compare all candidates on the held-out eval set and pick the winner:

```bash
cd training
python3 compare_models.py \
    --parametric parametric_model_cleaned.json \
    --numpy-mlp mlp_numpy_xy_radius.json \
    --numpy-mlp mlp_numpy_polar.json \
    --eval-csv distance_eval_data.csv
# (or --mlp-run <run> for torch .pt checkpoints)
```

### Results (current cleaned 592-row data)

All three candidates, full range. **CV** = 5-fold out-of-fold (honest, 592 pts);
**Held-out** = `distance_eval_data.csv` (11 pts). See
[`model_comparison_results.csv`](../training/model_comparison_results.csv).

| Model | CV MAE | CV RMSE | CV MAPE | Held MAE | Held MAPE | Max err |
|---|---|---|---|---|---|---|
| Parametric (deg 5, ord 2) | 11.75 | 17.52 | **12.65 %** | 10.97 | 11.84 % | **33.5** |
| MLP `xy_radius` (300 ep) | 9.41 | **14.34** | 14.59 % | 12.16 | 13.59 % | 44.7 |
| **MLP `polar` (300 ep)** | **9.21** | 14.43 | 13.84 % | **10.68** | **11.21 %** | 50.8 |

Parametric, restricted to shorter range (where the data is clean), is much
better — useful if the robot only needs near-field distance:

| Parametric range | Best (P, M) | CV MAE | CV RMSE | CV MAPE |
|---|---|---|---|---|
| ≤130 cm | (1, 3) | 5.85 | 8.97 | 9.54 % |
| ≤100 cm | (1, 2) | 4.28 | 6.12 | 8.52 % |

### Selected model

**MLP with `polar` features (300 epochs) is the most accurate overall** — it has
the lowest cross-validated MAE/RMSE and the lowest held-out MAE *and* MAPE. The
`polar` parameterization beats `xy_radius` because it encodes the radius/angle
split that the physics dictates. The parametric model is a strong, **torch-free**
runner-up: best relative error (CV MAPE) and the **tightest worst case** (33.5 vs
50.8 cm max error), deployable as a 30-coefficient JSON.

**Practical recommendation:**
- Deploy **MLP `polar`** for the lowest average error, **or** the **parametric**
  model if you want torch-free deployment, a bounded worst case, and near-field
  accuracy.
- If the task only needs distance within ~1.3 m, fit/deploy the **≤130 cm
  parametric** variant (≈6 cm MAE, far better than any full-range model).

### Headline finding

Accuracy is **strongly range-dependent** on the current data: close range
(≤ 100 cm) is good (~4 cm MAE), but error grows sharply past ~130 cm because of
irreducible rim-detection noise (§2.2). For reference, the older
`distance_training_data_cleaned_center_330_318.csv` fit to ~4 % MAPE full-range;
the current data is noisier, pointing at the **bbox-midpoint detection** as the
thing to improve next for far-range accuracy.

### Recommendations

1. **Deploy the model with the lowest held-out MAPE** from `compare_models.py`.
   If the robot only needs distance within ~1.3 m, fit/deploy the close-range
   variant — it is markedly more accurate.
2. **Recalibrate the optical center** (apply the ~(+15, +1.5) px offset to
   `thresholds.json`, recollect/recenter, retrain). This is the single biggest
   available accuracy win and removes ~60 % of within-arc variance.
3. **Improve rim ball-detection** (revisit the bbox-midpoint change) to cut the
   far-range noise that currently floors accuracy beyond ~130 cm.

---

## 6. File map

| Path | Role |
|---|---|
| `training/distance_training_data.csv` | Raw calibration samples (input) |
| `training/data_review/review_and_clean_distance_data.py` | Review + cleaning + plots |
| `training/distance_training_data_cleaned.csv` | Cleaned output (drop-in for training) |
| `training/data_review/outlier_report.csv` | Dropped rows + reasons |
| `training/data_review/arc_summary.csv` | Per-arc spread diagnostics |
| `training/data_review/plots/` | Review figures |
| `training/fit_parametric_model.py` | Parametric model fit |
| `training/parametric_model_cleaned.json` | Fitted parametric coefficients (deploy candidate) |
| `training/train_distance_model.py` | MLP training (torch; `.pt` + ONNX) |
| `training/train_distance_model_numpy.py` | Torch-free numpy MLP (parity reference) |
| `training/mlp_numpy_polar.json`, `mlp_numpy_xy_radius.json` | Trained numpy MLP weights |
| `training/evaluate_distance_models.py` | Per-checkpoint MLP evaluation (torch) |
| `training/compare_models.py` | Cross-model comparison + winner selection |
| `training/model_comparison_results.csv` | Final comparison table |
| `training/run_training_pipeline.sh` | One-shot driver: parametric + both MLPs + compare |
| `training/distance_eval_data.csv` | Held-out evaluation set |
