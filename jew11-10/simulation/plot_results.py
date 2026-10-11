#!/usr/bin/env python3
"""Optional standard matplotlib plots from run_simulation.py CSV output.

Requires matplotlib (the simulation/test runner itself only uses the stdlib).
Usage: python3 simulation/plot_results.py simulation/results/swingup.csv
"""
import argparse
import csv
import math
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    args = parser.parse_args()
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        parser.error("Plotting needs matplotlib; CSV generation and all tests work without it.")
    with args.csv.open() as file:
        rows = list(csv.DictReader(file))
    t = [float(row["time_s"]) for row in rows]
    values = ([math.remainder(float(row["theta_rad"]) - math.pi, 2 * math.pi) for row in rows],
              [float(row["x_m"]) for row in rows],
              [float(row["v_m_s"]) for row in rows],
              [float(row["accel_m_s2"]) for row in rows])
    labels = ("Upright error [rad]", "Cart position [m]", "Cart speed [m/s]", "Acceleration [m/s²]")
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(11, 8), constrained_layout=True)
    for axis, value, label in zip(axes, values, labels):
        axis.plot(t, value, linewidth=1)
        axis.set_ylabel(label)
        axis.grid(alpha=.3)
    capture = next((float(row["time_s"]) for row in rows if row["mode"] == "1"), None)
    if capture is not None:
        for axis in axes:
            axis.axvline(capture, color="#2d9658", linestyle="--", linewidth=1)
    axes[-1].set_xlabel("Time [s]")
    fig.suptitle(args.csv.stem + " — software simulation, hardware capability unverified")
    output = args.csv.with_suffix(".png")
    fig.savefig(output, dpi=150)
    print(output)


if __name__ == "__main__":
    main()
