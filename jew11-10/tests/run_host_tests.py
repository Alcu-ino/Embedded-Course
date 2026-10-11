#!/usr/bin/env python3
"""Compile and run the actual controller/motor C code on this computer."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parent.parent
CASES = (
    "middle", "at_a", "at_b", "reversed_mapping", "reversed_at_a", "reversed_at_b",
    "both_pressed", "a_stuck_pressed", "b_never_pressed", "timer_stalled",
    "active_low", "mixed_polarity", "polarity_mismatch", "wrong_mapping",
    "invalid_config", "cancel", "cancel_before_start", "delayed_poll", "switch_bounce",
    "tick_wrap", "command_nan", "command_bounds", "zero_acc_holds_velocity",
    "pending_pulse", "ug_not_a_step", "runtime_limit", "odd_track", "short_track",
    "long_track", "timer_start_failure", "direction_reversal", "latched_limit_bounce",
)
TARGETS = {
    "lqr": ("tests/host/lqr_test.c", "test/Core/Src/lqr.c", "test/Core/Src/pendulum_control.c"),
    "encoder": ("tests/host/encoder_test.c", "test/Core/Src/encoder.c"),
    "control": ("tests/host/control_test.c", "test/Core/Src/pendulum_control.c"),
    "application": ("tests/host/application_test.c", "test/Core/Src/application.c",
                    "test/Core/Src/encoder.c", "test/Core/Src/pendulum_control.c"),
    "homing": ("tests/host/homing_sim.c", "test/Core/Src/motor.c"),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--case", choices=CASES, help="run one homing scenario")
    selection.add_argument("--lqr-only", action="store_true", help="check balance adapter arithmetic")
    selection.add_argument("--suite", choices=TARGETS, help="run one module's suite")
    parser.add_argument("--sanitize", action="store_true", help="enable undefined-behaviour sanitizer")
    args = parser.parse_args()
    cc = shutil.which(os.environ.get("CC", "gcc"))
    if cc is None:
        parser.error("A host C compiler is required (gcc, or set CC=clang).")
    common = [
        cc, "-std=gnu11", "-O2", "-Wall", "-Wextra", "-Werror",
        "-include", str(ROOT / "tests/host/fake_hal.h"),
        "-I", str(ROOT / "test/Core/Inc"),
    ]
    if args.sanitize:
        common += ["-fsanitize=undefined", "-fno-sanitize-recover=all"]
    selected = ["homing"] if args.case else ["lqr"] if args.lqr_only else (
        [args.suite] if args.suite else list(TARGETS))
    records = []
    with tempfile.TemporaryDirectory(prefix="pendulum-host-tests-") as work:
        work = Path(work)
        for name in selected:
            # Encoder-only tests also exercise the actual TIM3 read wrapper.
            defines = ["-DPENDULUM_HOST_TEST"] if name == "application" else []
            command = common + defines + [str(ROOT / src) for src in TARGETS[name]] + [
                "-lm", "-o", str(work / name)]
            subprocess.run(command, check=True)
            cases = ((args.case,) if args.case else CASES) if name == "homing" else (None,)
            for case in cases:
                label = f"{name}/{case}" if case else name
                print("\nRunning " + label, flush=True)
                try:
                    result = subprocess.run([str(work / name)] + ([case] if case else []),
                                            timeout=10, capture_output=True, text=True)
                    output = result.stdout + result.stderr
                    passed = result.returncode == 0
                except subprocess.TimeoutExpired:
                    output, passed = "FAIL: host watchdog expired\n", False
                print(output, end="", flush=True)
                records.append({"case": label, "passed": passed, "output": output})
    source_paths = {ROOT / src for name in selected for src in TARGETS[name]}
    source_paths.update((ROOT / "test/Core/Inc").glob("*.h"))
    source_paths.add(ROOT / "tests/host/fake_hal.h")
    report = {"sanitizer": args.sanitize, "cases": records,
              "source_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in sorted(source_paths)}}
    out = ROOT / "build/host-tests"
    out.mkdir(parents=True, exist_ok=True)
    (out / ("report-ubsan.json" if args.sanitize else "report.json")).write_text(
        json.dumps(report, indent=2) + "\n")
    failed = [record["case"] for record in records if not record["passed"]]
    print(f"\n{len(records) - len(failed)}/{len(records)} test invocations passed.")
    if failed:
        print("Failed: " + ", ".join(failed))
    print("Host tests only; no USB, flashing, electrical signals, or real motor motion.")
    return bool(failed)


if __name__ == "__main__":
    raise SystemExit(main())
