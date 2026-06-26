#!/usr/bin/env bash
# End-to-end distance-model training on the CLEANED data.
#
# Trains both MLP feature modes (capped at 300 epochs), evaluates every
# checkpoint, (re)fits the parametric model, then compares all candidates and
# prints the most accurate one.
#
# Requires torch:  pip install torch --break-system-packages
# Run from the training/ directory:  bash run_training_pipeline.sh
set -euo pipefail
cd "$(dirname "$0")"

DATA=distance_training_data_cleaned.csv
EVAL=distance_eval_data.csv
LR=0.01
EPOCHS=300

if [[ ! -f "$DATA" ]]; then
  echo "Cleaned data missing. Run: (cd data_review && python3 review_and_clean_distance_data.py)" >&2
  exit 1
fi

echo "=== Parametric log-poly-Fourier ==="
python3 fit_parametric_model.py "$DATA" \
  --max-radial-degree 5 --max-angular-order 4 --cv-folds 5 \
  --save-coeffs parametric_model_cleaned.json \
  --plots-dir data_review/parametric_plots_cleaned \
  --eval-csv "$EVAL"

if python3 -c "import torch" 2>/dev/null; then
  echo; echo "=== MLP (torch): xy_radius + polar (max ${EPOCHS} epochs) ==="
  python3 train_distance_model.py "$DATA" --run-dir run_cleaned_xy_radius_300 \
    --feature-mode xy_radius --max-epochs "$EPOCHS" --learning-rate "$LR" --cv-folds 5
  python3 evaluate_distance_models.py --run-dir run_cleaned_xy_radius_300
  python3 train_distance_model.py "$DATA" --run-dir run_cleaned_polar_300 \
    --feature-mode polar --max-epochs "$EPOCHS" --learning-rate "$LR" --cv-folds 5
  python3 evaluate_distance_models.py --run-dir run_cleaned_polar_300
  MLP_ARGS=(--mlp-run run_cleaned_xy_radius_300 --mlp-run run_cleaned_polar_300)
else
  echo; echo "=== MLP (numpy, torch not installed): xy_radius + polar (max ${EPOCHS} epochs) ==="
  python3 train_distance_model_numpy.py "$DATA" --feature-mode xy_radius \
    --max-epochs "$EPOCHS" --learning-rate "$LR" --cv-folds 5 --save-json mlp_numpy_xy_radius.json
  python3 train_distance_model_numpy.py "$DATA" --feature-mode polar \
    --max-epochs "$EPOCHS" --learning-rate "$LR" --cv-folds 5 --save-json mlp_numpy_polar.json
  MLP_ARGS=(--numpy-mlp mlp_numpy_xy_radius.json --numpy-mlp mlp_numpy_polar.json)
fi

echo; echo "=== Model comparison ==="
python3 compare_models.py \
  --parametric parametric_model_cleaned.json \
  "${MLP_ARGS[@]}" \
  --eval-csv "$EVAL"
