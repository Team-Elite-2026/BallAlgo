# BallAlgo Testing Compatibility Layer

The canonical offline simulator now lives in [sim/README.md](/Users/premshah/Desktop/Robotics/Soccer Robotics/Elite/main-2026/BallAlgo/sim/README.md).

This `testing/` folder remains as a compatibility layer so older commands still work:

- `build-test/ballalgo_planner_bench`
- `testing/visualize_plan.py`
- `testing/visualize_chunks.py`
- `testing/run_pose_sweep.py`

Compatibility behavior:

- `ballalgo_planner_bench` now forwards into the shared sim core
- if you do not pass `--mode`, the wrapper defaults to `pose_target`
- the Python scripts forward to the implementation under `BallAlgo/sim/`

For new production-route work, prefer:

```bash
cd BallAlgo/sim
python3 visualize_plan.py ../sim/artifacts/your_case.json
```

and:

```bash
cd BallAlgo
./build-test/ballalgo_sim --mode production_ball_plan ...
```
