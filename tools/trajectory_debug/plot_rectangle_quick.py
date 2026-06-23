#!/usr/bin/env python3
"""Quick matplotlib visualization of rectangle_loop trajectory."""

import json
import struct
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).parent.parent.parent

def load_artifact(artifact_path):
    """Load a .traj artifact and parse position samples."""
    with open(artifact_path, "rb") as f:
        header_line = f.readline().decode().strip()
        header = json.loads(header_line)

        # Read sample data
        positions = []
        velocities = []
        times = []
        elapsed_ms = 0

        for chunk in header.get("chunks", []):
            start_delay = chunk.get("start_delay_ms", 0)
            dt_ms = chunk.get("dt_ms", 4)
            actions = chunk.get("actions", [])

            for action in actions:
                # Sample format: vx (float), vy (float), omega (float)
                vx = action.get("vx", 0.0)
                vy = action.get("vy", 0.0)

                # Integrate position (simple Euler integration)
                if len(positions) == 0:
                    x, y = header.get("start_pose", {}).get("x_mm", 0), header.get("start_pose", {}).get("y_mm", 0)
                else:
                    x, y = positions[-1]
                    # Add velocity contribution (vx is body +forward, vy is body +left in this model)
                    x += vx * (dt_ms / 1000.0) * 1000  # mm
                    y += vy * (dt_ms / 1000.0) * 1000  # mm

                positions.append((x, y))
                velocities.append((vx, vy))
                elapsed_ms += dt_ms
                times.append(elapsed_ms / 1000.0)  # convert to seconds

        return positions, velocities, times, header

artifact_path = REPO_ROOT / "tests/trajectory_cases/generated/rectangle_loop.traj"
positions, velocities, times, header = load_artifact(artifact_path)

positions = np.array(positions)
velocities = np.array(velocities)
times = np.array(times)

# Create figure with subplots
fig, axes = plt.subplots(2, 2, figsize=(12, 10))

# Plot 1: Position track
ax = axes[0, 0]
ax.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=2, label='Path')
ax.plot(positions[0, 0], positions[0, 1], 'go', markersize=10, label='Start')
ax.plot(positions[-1, 0], positions[-1, 1], 'rs', markersize=10, label='End')
ax.set_xlabel('X (mm)')
ax.set_ylabel('Y (mm)')
ax.set_title('Rectangle Loop Trajectory')
ax.legend()
ax.grid(True, alpha=0.3)
ax.axis('equal')

# Plot 2: VX over time
ax = axes[0, 1]
ax.plot(times, velocities[:, 0], 'b-', linewidth=2)
ax.set_xlabel('Time (s)')
ax.set_ylabel('VX (m/s)')
ax.set_title('Forward Velocity')
ax.grid(True, alpha=0.3)

# Plot 3: VY over time
ax = axes[1, 0]
ax.plot(times, velocities[:, 1], 'g-', linewidth=2)
ax.set_xlabel('Time (s)')
ax.set_ylabel('VY (m/s)')
ax.set_title('Sideways Velocity')
ax.grid(True, alpha=0.3)

# Plot 4: Speed magnitude
ax = axes[1, 1]
speed = np.sqrt(velocities[:, 0]**2 + velocities[:, 1]**2)
ax.plot(times, speed, 'r-', linewidth=2)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Speed (m/s)')
ax.set_title('Total Speed')
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(REPO_ROOT / "tests/trajectory_cases/generated/rectangle_loop_plot.png", dpi=150, bbox_inches='tight')
print(f"Saved plot to tests/trajectory_cases/generated/rectangle_loop_plot.png")
plt.close()
