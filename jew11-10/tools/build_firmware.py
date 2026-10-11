#!/usr/bin/env python3
"""Reproducibly compile/link STM32 firmware without CubeIDE or flashing."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent


def find_compiler(explicit):
    if explicit:
        return Path(explicit).resolve()
    found = shutil.which("arm-none-eabi-gcc")
    if found:
        return Path(found)
    bundled = sorted(Path("/opt/st").glob("*/plugins/*gnu-tools-for-stm32*/tools/bin/arm-none-eabi-gcc"))
    if bundled:
        return bundled[-1]
    raise SystemExit("ARM GCC not found; provide --compiler /path/to/arm-none-eabi-gcc")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default=os.environ.get("ARM_GCC"))
    parser.add_argument("--output", type=Path, default=ROOT / "build/firmware")
    parser.add_argument("--optimization", choices=("0", "2", "s"), default="2")
    args = parser.parse_args()
    cc = find_compiler(args.compiler)
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    target = ["-mcpu=cortex-m4", "-mthumb", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"]
    flags = target + ["--specs=nano.specs", "-g3", "-O" + args.optimization,
                      "-ffunction-sections", "-fdata-sections", "-Wall", "-Wextra", "-Werror",
                      "-DDEBUG", "-DUSE_HAL_DRIVER", "-DSTM32F446xx"]
    includes = ["Core/Inc", "Drivers/STM32F4xx_HAL_Driver/Inc",
                "Drivers/STM32F4xx_HAL_Driver/Inc/Legacy",
                "Drivers/CMSIS/Device/ST/STM32F4xx/Include", "Drivers/CMSIS/Include"]
    sources = sorted((ROOT / "test/Core/Src").glob("*.c")) + \
        sorted((ROOT / "test/Drivers/STM32F4xx_HAL_Driver/Src").glob("*.c")) + \
        sorted((ROOT / "test/Core/Startup").glob("*.s"))

    def compile_one(source):
        obj = out / (source.stem + ".o")
        language = ["-x", "assembler-with-cpp"] if source.suffix == ".s" else ["-std=gnu11"]
        vendor_flags = ["-Wno-unused-parameter"] if "Drivers" in source.parts else []
        command = [str(cc), *flags, *vendor_flags, *language, *["-I" + str(ROOT / "test" / i) for i in includes],
                   "-c", str(source), "-o", str(obj)]
        result = subprocess.run(command, text=True, capture_output=True)
        return obj, result

    with ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(compile_one, sources))
    failed = False
    for _, result in results:
        if result.stdout or result.stderr:
            print(result.stdout + result.stderr, end="")
        failed |= result.returncode != 0
    if failed:
        return 1
    elf = out / "pendulum.elf"
    command = [str(cc), *target, "--specs=nano.specs", "--specs=nosys.specs",
               "-T" + str(ROOT / "test/STM32F446RETX_FLASH.ld"), "-Wl,--gc-sections",
               "-Wl,-Map=" + str(out / "pendulum.map"), *[str(obj) for obj, _ in results],
               "-Wl,--start-group", "-lc", "-lm", "-Wl,--end-group", "-o", str(elf)]
    subprocess.run(command, check=True)
    subprocess.run([str(cc.with_name("arm-none-eabi-objcopy")), "-O", "binary",
                    str(elf), str(out / "pendulum.bin")], check=True)
    size = subprocess.run([str(cc.with_name("arm-none-eabi-size")), str(elf)],
                          check=True, capture_output=True, text=True).stdout
    version = subprocess.run([str(cc), "--version"], check=True, capture_output=True, text=True).stdout.splitlines()[0]
    inputs = sources + sorted((ROOT / "test/Core/Inc").glob("*.h")) + [ROOT / "test/STM32F446RETX_FLASH.ld"]
    manifest = {"compiler": version, "optimization": args.optimization, "source_count": len(sources),
                "warnings": "-Wall -Wextra -Werror; vendor HAL only: -Wno-unused-parameter",
                "source_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},
                "size": size, "flashed": False}
    (out / "build.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(size, end="")
    print(f"PASS: compiled and linked {len(sources)} sources with warnings treated as errors.\n{elf}\nNo hardware accessed or flashed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
