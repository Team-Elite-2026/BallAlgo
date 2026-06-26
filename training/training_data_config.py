import json
from decimal import Decimal
from pathlib import Path

_thresholds = json.loads((Path(__file__).resolve().parents[1] / "thresholds.json").read_text())
_offsets = _thresholds["offsets"]
PIXEL_ORIGIN = (_offsets["x"], _offsets["y"])

DISTANCE_ADJUSTMENT_CM = Decimal("9")
