#!/usr/bin/env python3
"""Compile current firmware control sources on the host and reproduce two issues.

Uses only the Python standard library and a host GCC compiler. The real main.c
callback and its angle thresholds are extracted on each run to avoid keeping a
second implementation of the switching logic. No firmware files are edited.
"""

from pathlib import Path
import re
import subprocess
import tempfile


HERE = Path(__file__).resolve().parent
CORE = HERE.parents[1] / "test" / "Core"


def extract_callback(source):
    signature = "void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)"
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 1
    cursor = opening + 1
    while depth:
        depth += (source[cursor] == "{") - (source[cursor] == "}")
        cursor += 1
    return source[start:cursor]


def main():
    source = (CORE / "Src" / "main.c").read_text()
    definitions = []
    for name in ("ANGOLO_MIN", "ANGOLO_MAX"):
        match = re.search(r"^#define\s+" + name + r"\s+(.+)$", source, re.M)
        if match is None:
            raise RuntimeError(f"Cannot locate {name} in main.c")
        definitions.append(f"#define {name} {match.group(1)}")

    with tempfile.TemporaryDirectory(prefix="pendulum-offline-audit-") as directory:
        build = Path(directory)
        callback = build / "callback_under_test.c"
        callback.write_text(
            '#include "harness.h"\n'
            + "\n".join(definitions)
            + "\n"
            + extract_callback(source)
            + "\n"
        )
        executable = build / "audit"
        subprocess.run(
            [
                "gcc", "-std=c11", "-O2", "-Wall", "-Wextra",
                "-I", str(HERE), "-I", str(CORE / "Inc"),
                str(HERE / "harness.c"), str(callback),
                str(CORE / "Src" / "lqr.c"),
                str(CORE / "Src" / "swingup.c"),
                "-lm", "-o", str(executable),
            ],
            check=True,
        )
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
