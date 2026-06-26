"""Distance MLP definition and checkpoint load/predict helpers."""

from __future__ import annotations

from pathlib import Path

import numpy as np

try:
    import torch
    from torch import nn
except ModuleNotFoundError as exc:
    torch = None
    nn = None
    TORCH_IMPORT_ERROR = exc
else:
    TORCH_IMPORT_ERROR = None

NORMALIZATION_KEYS = ("feature_mean", "feature_std", "distance_mean", "distance_std")


def require_torch() -> None:
    if TORCH_IMPORT_ERROR is not None:
        raise SystemExit(
            "Missing dependency: torch. Install with:\n"
            "  python3 -m pip install torch\n"
        ) from TORCH_IMPORT_ERROR


if nn is not None:

    class DistanceMLP(nn.Module):
        """3 input features -> hidden layers -> scaled distance output."""

        def __init__(self, input_dim: int = 3, hidden_sizes: tuple[int, ...] = (16, 16)) -> None:
            super().__init__()
            self.input_dim = input_dim
            self.hidden_sizes = hidden_sizes

            layers: list[nn.Module] = []
            current_dim = input_dim
            for hidden_size in hidden_sizes:
                layers.append(nn.Linear(current_dim, hidden_size))
                layers.append(nn.ReLU())
                current_dim = hidden_size
            self.hidden = nn.Sequential(*layers)
            self.output = nn.Linear(current_dim, 1)

        def forward(self, coords: torch.Tensor) -> torch.Tensor:
            return self.output(self.hidden(coords))

    class NormalizedDistanceModel(nn.Module):
        """Apply feature and distance normalization around the MLP."""

        def __init__(
            self,
            model: DistanceMLP,
            feature_mean: torch.Tensor,
            feature_std: torch.Tensor,
            distance_mean: torch.Tensor,
            distance_std: torch.Tensor,
        ) -> None:
            super().__init__()
            self.model = model
            self.register_buffer("feature_mean", feature_mean)
            self.register_buffer("feature_std", feature_std)
            self.register_buffer("distance_mean", distance_mean)
            self.register_buffer("distance_std", distance_std)

        def forward(self, raw_features: torch.Tensor) -> torch.Tensor:
            features = (raw_features - self.feature_mean) / self.feature_std
            return self.model(features) * self.distance_std + self.distance_mean

else:

    class DistanceMLP:  # type: ignore[no-redef]
        pass

    class NormalizedDistanceModel:  # type: ignore[no-redef]
        pass


def build_input_features(coords: np.ndarray) -> np.ndarray:
    coords_array = np.asarray(coords, dtype=np.float32)
    if coords_array.ndim == 1:
        coords_array = coords_array.reshape(1, -1)

    if coords_array.shape[1] == 3:
        return coords_array.astype(np.float32)
    if coords_array.shape[1] != 2:
        raise ValueError(f"Expected 2 or 3 input columns, got shape {coords_array.shape}")

    radius = np.hypot(coords_array[:, 0], coords_array[:, 1]).reshape(-1, 1).astype(np.float32)
    return np.concatenate((coords_array, radius), axis=1)


def prepare_model_inputs(coords: np.ndarray, normalization: dict[str, torch.Tensor]) -> np.ndarray:
    expected_dim = int(normalization["feature_mean"].shape[-1])
    coords_array = np.asarray(coords, dtype=np.float32)
    if coords_array.ndim == 1:
        coords_array = coords_array.reshape(1, -1)

    if coords_array.shape[1] == expected_dim:
        return coords_array.astype(np.float32)
    if coords_array.shape[1] == 2 and expected_dim == 3:
        return build_input_features(coords_array)
    raise ValueError(f"Model expects {expected_dim} inputs, but received shape {coords_array.shape}")


def predict_model(
    model: DistanceMLP,
    normalization: dict[str, torch.Tensor],
    coords: np.ndarray,
) -> np.ndarray:
    require_torch()
    raw_features = prepare_model_inputs(coords, normalization)
    wrapped = NormalizedDistanceModel(model, **normalization)
    wrapped.eval()
    with torch.no_grad():
        prediction = wrapped(torch.from_numpy(raw_features.astype(np.float32))).numpy()
    return prediction.astype(np.float32)


def load_checkpoint_model(checkpoint_path: Path) -> tuple[DistanceMLP, dict[str, torch.Tensor]]:
    require_torch()
    checkpoint = torch.load(checkpoint_path, map_location="cpu")

    missing = [key for key in NORMALIZATION_KEYS if key not in checkpoint["normalization"]]
    if missing:
        raise KeyError(f"Checkpoint missing normalization keys: {missing}")

    model = DistanceMLP(
        input_dim=int(checkpoint["input_dim"]),
        hidden_sizes=tuple(int(size) for size in checkpoint["hidden_sizes"]),
    )
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()
    return model, checkpoint["normalization"]
