# STM32 cart pendulum

Firmware for the STM32F446RE cart pendulum, with homing, encoder calibration, swing-up, upright balancing, and offline tests. The application follows a defined sequence: confirm configuration, press the start button with the pendulum hanging down, establish its reference, home the cart, wait for settling, then attempt swing-up and balance. Errors stop motion and remain latched until reset.

`APP_RUN_MODE` in [app_config.h](test/Core/Inc/app_config.h) also supports staged commissioning:

| Value | Behavior after calibration and homing |
| --- | --- |
| `1` | Homing only: remain stopped at the midpoint, with fault monitoring active. |
| `2` | Manual upright capture: remain stopped while the pendulum is raised by hand; engage balancing only after the capture conditions persist. Loss of capture stops and faults. |
| `0` | Automatic swing-up and balancing after the pendulum settles. This is the default run mode, subject to the configuration gate below. |

**The physical plant has not been tested during this work.** Motion is disabled by default through `APP_HARDWARE_CONFIRMED=0` in [app_config.h](test/Core/Inc/app_config.h). The original speed/acceleration limits remain the firmware defaults. They do not achieve swing-up in the current nonlinear regression. Successful simulation cases use a separately labelled, hypothetical stronger actuator: 3 m/s² acceleration, 1 m/s speed, and a 0.90 m track. These results do not establish that the real motor can provide that motion.

## Try it now, without USB

From this directory, using Python 3 and GCC (or `CC=clang`):

```sh
python3 tests/run_host_tests.py
python3 tests/run_host_tests.py --sanitize
python3 simulation/run_simulation.py
python3 simulation/viewer_build.py
```

The host suite contains 36 invocations covering motor/homing and four module suites. The nonlinear simulation has seven scenarios, including an expected timeout with the original motor limits. No board, flashing, network access, or additional Python packages are needed.

Open [the offline replay viewer](simulation/viewer.html) in a browser to play the simulated pendulum and inspect its angle and cart position. Reports and trajectories are written to `build/host-tests/` and `simulation/results/`.

For a focused homing check:

```sh
python3 tests/run_host_tests.py --case at_b
python3 tests/run_host_tests.py --suite application
```

The simulated switches are in [homing_sim.c](tests/host/homing_sim.c). A fault-test PASS means the intended failure was detected and motion stopped; it does not necessarily mean the cart avoided contact with an endpoint.

## Build the STM32 firmware

An ARM GCC toolchain is required for this step:

```sh
python3 tools/build_firmware.py
```

The script finds `arm-none-eabi-gcc` on PATH or in a supported local STM32CubeIDE installation. Otherwise, specify its path with `--compiler /path/to/arm-none-eabi-gcc`. The build produces `build/firmware/pendulum.elf`, `pendulum.bin`, a map, and a source-hash manifest. It does not flash anything. The CubeIDE project is in [test/](test/).

## Read next

- [Testing guide](tests/README.md): commands, simulated switch behavior, module coverage, and fault-result interpretation.
- [Nonlinear simulation guide](simulation/README.md): physical model, controller conventions, assumptions, and success criteria.
- [Commissioning guide](docs/COMMISSIONING.md): board-only checks and the measurements needed when the real plant returns.
- [Detailed changelog](CHANGELOG.md): firmware repairs, tests, behavior changes, and remaining limitations.

The older PDF guides describe earlier source snapshots. Use the current source and these Markdown guides for the implementation delivered here.
