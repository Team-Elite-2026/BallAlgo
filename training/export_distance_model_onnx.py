#!/usr/bin/env python3
"""Export a trained ball-distance checkpoint to ONNX for the C++ runtime.

The exported ONNX graph bakes in the feature normalization and distance
denormalization, so the runtime contract stays simple:

input:
  [dx, dy, radius] pixel features

output:
  scalar distance in centimeters
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

try:
    import onnx
    from onnx import TensorProto, helper, numpy_helper
except ModuleNotFoundError as exc:
    raise SystemExit(
        "Missing dependency: onnx. Install it before exporting ONNX.\n"
        "  python3 -m pip install onnx"
    ) from exc

try:
    import torch
except ModuleNotFoundError as exc:
    raise SystemExit(
        "Missing dependency: torch. Install it before exporting ONNX.\n"
        "  python3 -m pip install torch"
    ) from exc

from distance_model import load_checkpoint_model


TRAINING_DIR = Path(__file__).resolve().parent
DEFAULT_CHECKPOINT = TRAINING_DIR / "run_11" / "checkpoints" / "epoch_0850.pt"
DEFAULT_ONNX_OUT = TRAINING_DIR / "exported" / "ball_distance_latest.onnx"
REQUIRED_INPUT_DIM = 3


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export the ball distance checkpoint to ONNX.")
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT,
                        help=f"Checkpoint to export (default: {DEFAULT_CHECKPOINT})")
    parser.add_argument("--onnx-out", type=Path, default=DEFAULT_ONNX_OUT,
                        help=f"Output ONNX path (default: {DEFAULT_ONNX_OUT})")
    parser.add_argument("--opset", type=int, default=18,
                        help="ONNX opset version (default: 18)")
    return parser.parse_args()


def to_numpy(tensor: torch.Tensor) -> np.ndarray:
    return tensor.detach().cpu().numpy().astype(np.float32, copy=False)


def make_initializer(name: str, values: np.ndarray) -> onnx.TensorProto:
    return numpy_helper.from_array(np.asarray(values, dtype=np.float32), name=name)


def main() -> None:
    args = parse_args()
    model, normalization = load_checkpoint_model(args.checkpoint)

    input_dim = int(normalization["feature_mean"].shape[-1])
    if input_dim != REQUIRED_INPUT_DIM:
        raise SystemExit(
            f"Unsupported checkpoint input dimension {input_dim}. "
            f"Expected exactly {REQUIRED_INPUT_DIM} features: dx, dy, radius."
        )

    if len(model.hidden) != 4:
        raise SystemExit("Unsupported architecture: expected exactly two hidden Linear+ReLU blocks.")

    feature_mean = to_numpy(normalization["feature_mean"])
    feature_std = to_numpy(normalization["feature_std"])
    distance_mean = to_numpy(normalization["distance_mean"])
    distance_std = to_numpy(normalization["distance_std"])

    w1 = to_numpy(model.hidden[0].weight).T
    b1 = to_numpy(model.hidden[0].bias).reshape(1, -1)
    w2 = to_numpy(model.hidden[2].weight).T
    b2 = to_numpy(model.hidden[2].bias).reshape(1, -1)
    w3 = to_numpy(model.output.weight).T
    b3 = to_numpy(model.output.bias).reshape(1, -1)

    input_tensor = helper.make_tensor_value_info(
        "features", TensorProto.FLOAT, ["batch", input_dim]
    )
    output_tensor = helper.make_tensor_value_info(
        "distance_cm", TensorProto.FLOAT, ["batch", 1]
    )

    initializers = [
        make_initializer("feature_mean", feature_mean),
        make_initializer("feature_std", feature_std),
        make_initializer("distance_mean", distance_mean),
        make_initializer("distance_std", distance_std),
        make_initializer("w1", w1),
        make_initializer("b1", b1),
        make_initializer("w2", w2),
        make_initializer("b2", b2),
        make_initializer("w3", w3),
        make_initializer("b3", b3),
    ]

    nodes = [
        helper.make_node("Sub", ["features", "feature_mean"], ["features_centered"], name="normalize_sub"),
        helper.make_node("Div", ["features_centered", "feature_std"], ["features_norm"], name="normalize_div"),
        helper.make_node("MatMul", ["features_norm", "w1"], ["hidden1_linear_mm"], name="hidden1_matmul"),
        helper.make_node("Add", ["hidden1_linear_mm", "b1"], ["hidden1_linear"], name="hidden1_add"),
        helper.make_node("Relu", ["hidden1_linear"], ["hidden1_relu"], name="hidden1_relu"),
        helper.make_node("MatMul", ["hidden1_relu", "w2"], ["hidden2_linear_mm"], name="hidden2_matmul"),
        helper.make_node("Add", ["hidden2_linear_mm", "b2"], ["hidden2_linear"], name="hidden2_add"),
        helper.make_node("Relu", ["hidden2_linear"], ["hidden2_relu"], name="hidden2_relu"),
        helper.make_node("MatMul", ["hidden2_relu", "w3"], ["distance_scaled_mm"], name="output_matmul"),
        helper.make_node("Add", ["distance_scaled_mm", "b3"], ["distance_scaled"], name="output_add"),
        helper.make_node("Mul", ["distance_scaled", "distance_std"], ["distance_denorm_mm"], name="denorm_mul"),
        helper.make_node("Add", ["distance_denorm_mm", "distance_mean"], ["distance_cm"], name="denorm_add"),
    ]

    graph = helper.make_graph(
        nodes=nodes,
        name="ball_distance_mlp",
        inputs=[input_tensor],
        outputs=[output_tensor],
        initializer=initializers,
    )
    onnx_model = helper.make_model(
        graph,
        producer_name="ballalgo_manual_export",
        opset_imports=[helper.make_operatorsetid("", args.opset)],
    )
    onnx.checker.check_model(onnx_model)

    args.onnx_out.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(onnx_model, args.onnx_out)

    print(f"Exported ONNX model: {args.onnx_out}")
    print(f"Input dimension: {input_dim}")
    print("Runtime output units: centimeters")


if __name__ == "__main__":
    main()
