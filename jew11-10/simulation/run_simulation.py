#!/usr/bin/env python3
"""Compile the real C controller/encoder and test them against a nonlinear plant."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parent.parent
CASES = ("swingup", "offset", "mirror", "model_error", "friction", "balance", "legacy_limits")
SOURCES = ("simulation/plant_sim.c", "test/Core/Src/pendulum_control.c", "test/Core/Src/encoder.c")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=CASES, help="run one scenario instead of all")
    parser.add_argument("--output", type=Path, default=ROOT / "simulation/results", help="CSV and JSON output folder")
    args = parser.parse_args()
    cc = shutil.which(os.environ.get("CC", "gcc"))
    if not cc:
        parser.error("Install a host C compiler or set CC to gcc/clang.")
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    with tempfile.TemporaryDirectory(prefix="pendulum-sil-") as work:
        executable = Path(work) / "plant_sim"
        subprocess.run([cc, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-DPENDULUM_HOST_TEST",
                        "-I", str(ROOT / "test/Core/Inc"), *[str(ROOT / src) for src in SOURCES],
                        "-lm", "-o", str(executable)], check=True)
        for case in (args.case,) if args.case else CASES:
            result = subprocess.run([str(executable), case, str(args.output / (case + ".csv"))],
                                    capture_output=True, text=True, timeout=20)
            message = (result.stdout + result.stderr).strip()
            print(message, flush=True)
            results.append({"case": case, "passed": result.returncode == 0, "message": message,
                            "csv": case + ".csv"})
    report = {
        "scope": "Software in the loop; actual firmware controller and encoder estimator; ideal cart acceleration actuator.",
        "capable_profile": {"acceleration_m_s2": 3.0, "speed_m_s": 1.0, "rail_length_m": 0.9,
                            "status": "SIMULATION ASSUMPTION ONLY; not verified hardware capability"},
        "legacy_profile": {"acceleration_m_s2": 0.31415927, "speed_m_s": 0.15707963,
                           "expected_result": "bounded timeout, not successful swing-up"},
        "source_sha256": {src: hashlib.sha256((ROOT / src).read_bytes()).hexdigest() for src in SOURCES},
        "results": results,
    }
    (args.output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print("CSV trajectories and source-versioned summary: " + str(args.output))
    print("The capable profile does not establish that the motor can deliver these limits.")
    return 0 if all(result["passed"] for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
