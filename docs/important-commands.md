## Rebuild Pi code

```bash
cmake --build build --target ballalgo
```

## Run a trajectory replay

Kill any running instance first, then run the replay with the Foxglove sidecar:

```bash
pkill -f ballalgo
python3 tools/trajectory_debug/run_replay.py forward_only.traj --with-sidecar
```

Omit `--with-sidecar` if you started `foxglove_sim/sidecar.py` manually in a separate terminal (useful to keep Foxglove Studio connected across multiple runs).
