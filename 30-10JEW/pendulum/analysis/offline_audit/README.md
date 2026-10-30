# Offline reproduction of current control issues

Run from any directory with Python 3 and GCC installed:

```sh
python3 /home/lorenzo/Downloads/27-10JEW/pendulum/analysis/offline_audit/run.py
```

This builds the project's actual `lqr.c` and `swingup.c`. It also extracts and
builds the actual `HAL_TIM_PeriodElapsedCallback` and angle thresholds from
`main.c` on every run. The executable and generated callback are temporary files.
The original firmware files are not changed.

The assertions **reproduce current behavior**, so a pass does not mean the
controller is fixed or safe to run:

- At 170 and 190 degrees, the actual mode logic and swing-up function send no
  motor command. A previously requested velocity is retained while the debug
  acceleration is zero.
- At exactly upright, seeding the LQR observer with one encoder count per 1 ms
  produces the maximum acceleration command in either direction. This proves
  sensitivity of observer entry to the raw derivative; it does not prove that
  this condition caused the earlier hardware wobble.

The harness supplies angle samples and substitutes HAL time and motor output.
It does not model motor torque, missed steps, timer waveforms, real encoder noise,
the pendulum, or interrupts. It cannot validate physical stability. A compiler
warning about discarding `volatile` comes from the actual callback passing the
global `acc` into `lqr_controller`; the extraction deliberately preserves it.

After correcting those behaviors, replace the reproduction assertions with the
desired behavior and rerun. Do not add this folder or its fake HAL header to the
CubeIDE firmware include paths.
