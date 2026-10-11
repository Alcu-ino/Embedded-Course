# Test the firmware without the plant

The simulated switches are C functions in [host/homing_sim.c](host/homing_sim.c). They run on your computer. They are not additional hardware, CubeIDE components, or a simulator package to install. The runner compiles the project's actual C modules with a small substitute hardware interface.

You need Python 3 and a host C compiler: `gcc`, or `clang` selected with `CC=clang`. No USB connection, STM32 board, MATLAB, Python packages, network access, flashing, or motor movement is involved.

## Start here

Run these commands from the project root:

```sh
python3 tests/run_host_tests.py
python3 tests/run_host_tests.py --sanitize
```

The complete suite has **36 test invocations**: 32 motor/homing scenarios and four module test executables. Each module executable contains multiple assertions. Both commands should finish with all invocations passing. `--sanitize` repeats the selected tests with UndefinedBehaviorSanitizer and stops on detected undefined behavior; it is not a hardware or electrical check.

To investigate one issue:

```sh
python3 tests/run_host_tests.py --case at_b
python3 tests/run_host_tests.py --case switch_bounce
python3 tests/run_host_tests.py --case latched_limit_bounce --sanitize
python3 tests/run_host_tests.py --suite application
python3 tests/run_host_tests.py --suite encoder
python3 tests/run_host_tests.py --suite control
python3 tests/run_host_tests.py --suite homing
python3 tests/run_host_tests.py --suite lqr
```

`--lqr-only` remains an alias for selecting the balance adapter checks. `--case`, `--suite`, and `--lqr-only` are mutually exclusive. Use `python3 tests/run_host_tests.py --help` to see the current selection list.

The runner creates temporary executables and removes them on exit. It writes results and source hashes to `build/host-tests/report.json`, or `report-ubsan.json` with `--sanitize`. A selected run replaces that report with results for only the selection; rerun without a selection to record the complete suite. Exit status zero means all selected assertions passed. A nonzero exit, compiler error, assertion failure, or host watchdog expiry needs investigation; failures are no longer expected during a normal full run.

## Where the simulated switches are

`pressed_switches()` defines the fictional cart track. In the normal 400-step fixture its basic behavior is:

```c
A_pressed = cart_position <= 0;
B_pressed = cart_position >= 400;
```

The test's `HAL_GPIO_ReadPin()` converts those pressed/released states to HIGH or LOW according to the scenario. The production `motor_homing()` and `motor_acc()` functions in [motor.c](../test/Core/Src/motor.c) read these values through their normal hardware interface. The fake PWM timer advances the imaginary cart as completed STEP pulses occur. Reaching a track end changes the next simulated GPIO reading.

The default fixture places A at position 0, B at position 400, and maps positive/CW motion toward B. Separate cases reverse that mapping or use active-LOW and mixed-polarity switches with corresponding configuration. These settings test configurable behavior; they do not identify the real plant's wiring. `active_low` now tests correctly configured active-LOW switches. `polarity_mismatch` deliberately supplies a wrong configuration.

The simulator advances virtual time immediately. It models a TIM2-like counter, fixed pulse widths, compare flags and interrupts, software update events, interrupt masking, stopped timers, and varying foreground delays. It includes switch bounce and a targeted interrupt/restart race. The 30-second virtual watchdog and the runner's 10-second wall-clock timeout protect the test process; they are separate from the production homing deadlines.

## Read PASS and fault results correctly

A normal homing PASS requires the measured span to match the fixture, the cart to return within one step of its midpoint, the homed position to become zero, PWM to stop, and counted pulses to match simulated emitted pulses. These cases also require zero attempted motion beyond the endpoints.

An injected-fault PASS means the firmware detects an error within the applicable bound, returns failure, stops PWM, invalidates homing, and refuses a subsequent acceleration command. It does **not** mean homing succeeded, and it does not necessarily mean the cart avoided the physical endpoint.

The `b_never_pressed` case demonstrates this limitation explicitly. With B's signal missing, the fictional cart reaches B and attempts **100 further steps into the end stop** before the configured travel bound faults. Its output includes `travel=0`, a nonzero `fault`, `into_endstop=100`, and `PWM=OFF`. This passes the bounded-failure assertion. Software counting commanded steps cannot discover an unreported obstruction or skipped motor steps; a limit signal that never arrives cannot provide normal endpoint protection. The `wrong_mapping` fault case also must not be interpreted as proof that incorrect wiring is harmless.

`into_endstop` counts attempted steps beyond the fixture, while the simulated physical cart stays clamped at its endpoint. This distinguishes emitted commands from actual cart displacement under an obstruction. Numerical fault values are defined by `motor_fault_t` in [motor.h](../test/Core/Inc/motor.h).

## Motor and homing scenarios

| Cases | What they exercise |
| --- | --- |
| `middle`, `at_a`, `at_b` | Acquire both ends, release and relatch contacts, measure travel, and return to center from each starting location. |
| `reversed_mapping`, `reversed_at_a`, `reversed_at_b` | The same behavior with CW pointing toward A. |
| `active_low`, `mixed_polarity` | Correctly configured input polarity for each switch. |
| `both_pressed`, `polarity_mismatch`, `invalid_config` | Reject invalid initial state or configuration without emitting steps. |
| `a_stuck_pressed`, `b_never_pressed`, `wrong_mapping` | Bound missing release, absent arrival, and incorrect direction configuration; latch failure. |
| `timer_stalled`, `timer_start_failure` | Stop on missing timer progress or HAL start failure. |
| `cancel`, `cancel_before_start` | Cancellation during homing and cancellation already latched before entry. |
| `delayed_poll`, `tick_wrap` | Foreground delay variation and unsigned millisecond-counter wraparound. |
| `switch_bounce` | Stop at contact and debounce while stopped, including contact release bounce. |
| `latched_limit_bounce` | Inject a contact interrupt at the command's critical-section boundary, release the simulated GPIO, and ensure the latched event prevents restart. |
| `odd_track`, `short_track`, `long_track` | Midpoint rounding and accepted travel bounds. |
| `command_nan`, `command_bounds` | Reject nonfinite acceleration and enforce configured speed/acceleration limits. |
| `zero_acc_holds_velocity` | Verify that zero acceleration maintains velocity; stopping requires `motor_stop()`. |
| `pending_pulse`, `ug_not_a_step` | Drain an in-progress/pending real pulse without double counting; do not count software UG as a STEP. |
| `runtime_limit`, `direction_reversal` | Stop and latch a runtime endpoint; reverse direction with consistent pulse accounting. |

## Other module tests

| Suite | Production code exercised | Main assertions |
| --- | --- | --- |
| `encoder` | `encoder.c`, including its TIM3 wrapper | Nonzero down reference, first-sample history, wrapping in both directions, direction calibration, 20 Hz velocity filtering, quantization, invalid parameters/counts, implausible jumps. |
| `control` | `pendulum_control.c` | Defined launch, angle/rate/command capture gates, dwell and hysteresis, preserved cart reference, stopping-distance bounds, fault latching, timeout, energy-injection sign. |
| `application` | `application.c`, `encoder.c`, `pendulum_control.c` | Truly uncalibrated startup, configuration gate, stationarity before calibration, homing-result publication, settling, control activation, cancellation, timing faults, end limits, preserved home on capture, stopped homing-only mode, and manual upright capture with a fault on loss of balance. |
| `lqr` | Compatibility `lqr.c` and shared balance calculation | Equilibrium, SI-to-step conversion, both physical lean directions, angular wrapping. The current balance gains use acceleration-input pole placement; the historical filename remains. |

The application suite substitutes the motor boundary so lifecycle transitions can be tested independently. The motor suite separately executes the actual driver and homing implementation. These are complementary tests, not a single full microcontroller emulation.

## Simulate the pendulum dynamics

The separate nonlinear simulation tests the actual controller and encoder estimator against an ideal acceleration-driven cart and nonlinear pendulum:

```sh
python3 simulation/run_simulation.py
python3 simulation/run_simulation.py --case swingup
python3 simulation/viewer_build.py
```

There are seven scenarios: `swingup`, `offset`, `mirror`, `model_error`, `friction`, `balance`, and `legacy_limits`. Six demonstrate upright capture and balancing under the explicitly hypothetical capable actuator profile. `legacy_limits` expects a latched timeout using the original restrictive motor limits; its PASS means the failure was handled.

Open [the replay viewer](../simulation/viewer.html) in a browser to play the generated trajectories or inspect plots. It works offline. After changing the controller or simulated plant, run all scenarios and refresh the viewer data. The replay is a visualization of saved results; assertions run in the C simulation at 1 kHz. See [the simulation guide](../simulation/README.md) for equations, parameters, success criteria, and limitations.

## Use the tests while changing the code

1. Reproduce the behavior with a specific case or suite.
2. Change the relevant production module, keeping physical wiring and actuator assumptions explicit.
3. Rerun that selection; then run the full suite and sanitizer suite before accepting the change.
4. For controller, estimator, units, or timing changes, rerun the nonlinear scenarios as well.
5. Build the real STM32 firmware with `python3 tools/build_firmware.py`. Host compilation alone does not check the target HAL, startup code, linker configuration, or instruction set.

The tests do not establish real switch levels, motor/encoder directions, electrical pulse shape, available motor torque, actual interrupt latency, missed-step detection, belt behavior, or successful physical balancing. The default firmware keeps motion disabled until configuration is confirmed. Follow [commissioning](../docs/COMMISSIONING.md) when the plant is available, and consult [the changelog](../CHANGELOG.md) for the implemented repairs.
