#!/usr/bin/env python3
"""Refresh the offline replay data after run_simulation.py (standard library only)."""
import csv
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
SCENARIOS = {
    "swingup": "Downward start",
    "offset": "Offset start",
    "mirror": "Mirrored offset",
    "model_error": "Effective length +10%",
    "friction": "Added friction",
    "balance": "Upright with disturbance",
    "legacy_limits": "Original motor limits — timeout",
}
data = {}
for scenario, label in SCENARIOS.items():
    path = HERE / "results" / (scenario + ".csv")
    if not path.exists():
        continue
    with path.open() as file:
        rows = list(csv.DictReader(file))
    # Replay/plots at 20 Hz; assertions still ran on every 1 kHz integration step.
    sampled = rows[::5]
    if rows and sampled[-1] is not rows[-1]:
        sampled.append(rows[-1])
    data[scenario] = {
        "label": label,
        "rows": [[round(float(row[key]), 5) for key in ("time_s", "x_m", "v_m_s", "theta_rad", "omega_rad_s", "accel_m_s2", "mode")] for row in sampled],
    }
(HERE / "viewer-data.js").write_text("window.PENDULUM_REPLAY = " + json.dumps(data, separators=(",", ":")) + ";\n")
print("Refreshed " + str(HERE / "viewer-data.js"))
