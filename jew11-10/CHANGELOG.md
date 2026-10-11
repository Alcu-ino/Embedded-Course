# Detailed changelog — 2026-10-11

## Scope and delivered status

Repaired and integrated the STM32F446RE pendulum firmware for a complete
calibration → homing → swing-up → upright-balance lifecycle, with separate
commissioning modes and reproducible tests without the real plant.

The software builds for the STM32 and passes the host tests and selected
nonlinear simulation scenarios listed below. **No physical pendulum, motor,
switch or STM32 board was connected, flashed or tested during this repair.**
Physical balancing remains unverified. Motion is gated by
`APP_HARDWARE_CONFIRMED=0` until wiring and mechanical configuration are known.

Original firmware speed/acceleration limits remain in place. They do not swing
up successfully in the supplied nonlinear regression. The successful cases use
a separately labelled hypothetical actuator profile. This distinction is part
of the delivered result, not a claim that changing code can supply more torque.

There was no usable Git history in the supplied workspace. Before the larger
repair, a snapshot of Core, tests, model and project configuration was saved at
`/tmp/pendulum-before-repair-ttfrkzt8`. This is a temporary comparison copy, not a
permanent version-control backup. Existing generated build outputs are not the
source of truth; rebuild from source.

## 1. Replaced the homing algorithm

Files: `test/Core/Inc/motor.h`, `test/Core/Src/motor.c`.

The old behavior depended on an assumed initial direction and did not robustly
handle starting on either endpoint, switch polarity, failed release, or a
stopped timer. The user's proposed replacement also could not implement a
reliable home:

- `hit` was read before initialization.
- Switches were read only after the initial movement, not continuously.
- `while(hit)` never reread switches or cleared `hit`.
- Reversing acceleration does not instantly reverse velocity.
- `motor_acc(0)` maintains velocity; it does not stop the motor.
- `steps++` counted outer loop iterations, not physical STEP pulses.
- `HAL_Delay(1)` was inconsistent with the comment claiming a 5 ms interval and
  with fixed integration assumptions.

The implemented home now:

1. Validates configuration and rejects contradictory switch states.
2. Determines the departure direction from the identity of a pressed switch,
   or seeks a configured first end when starting between the ends.
3. Stops on raw contact, debounces while stopped, backs away, verifies release,
   and slowly relatches the first endpoint.
4. Traverses to the other endpoint, repeats backoff and slow relatching, and
   measures actual commanded pulse travel between the repeatable references.
5. Rejects travel outside configured bounds, returns half the measured span,
   stops and establishes cart position zero at the midpoint.

Pressed HIGH/LOW is independently configurable for A and B. The configured CW
endpoint can be A or B; the code does not equate a label with physical left/right.
Both initial endpoints, reversed mapping and mixed switch polarities are tested.
Odd pulse spans produce a midpoint rounded within half a pulse.

Added global, per-phase, release-time and release-distance bounds. Homing has
latched fault reasons for invalid configuration, contradictory/unexpected
switches, release failure, travel, timeout, timer failure and cancellation.
Cancellation is respected before homing begins as well as while it runs. A
failed home stops PWM, clears velocity and never enables balance/swing-up.

Homing runs in the foreground while interrupts remain active. Integration uses
measured elapsed HAL ticks, capped at 20 ms per update, rather than pretending
each `HAL_Delay(1)` lasts exactly one millisecond. Unsigned tick wrap is tested.

## 2. Repaired STEP generation and position accounting

Files: `motor.h`, `motor.c`, `main.c`, `stm32f4xx_it.c/.h`.

- Replaced integrated-command/UIF-based position accounting with signed
  `int32_t` counts of completed STEP pulses. Software update events are not
  counted as steps.
- TIM2 PWM1 now uses a fixed nominal 2.5 microsecond high pulse. Its CC1 falling
  edge interrupt calls `motor_pulse_finished_isr()`, records the pulse, checks
  limits and enforces exact pulse-count targets.
- Stop handling drains an in-progress high pulse/pending compare once, stops
  with STEP low and clears stale events. The completion loop has a finite
  cycle budget if the timer clock stops.
- Direction changes finish the previous pulse before changing DIR and provide
  setup/hold delays. Pending pulses keep the old direction's sign.
- Timer period updates freeze CEN while changing CNT/ARR. Masking CPU interrupts
  alone would not freeze the hardware counter.
- A latched contact prevents a foreground command from restarting PWM after an
  interrupt stopped it, even if contact bounce has already released the raw
  GPIO. The targeted restart-race regression exercises this exact ordering.
- Added finite acceleration checks, explicit speed/acceleration limits, timer
  start failure handling, pulse-count overflow protection, and a 10 kHz pulse
  interrupt ceiling. Increasing that ceiling requires measured timing evidence.
- `motor_stop()` explicitly stops output and clears commanded speed;
  `motor_acc(0)` intentionally remains zero acceleration.

Counts represent **commanded emitted pulses**, not observed cart motion. There
is no independent cart encoder or stall detector. Lost steps, belt slip,
mechanical obstruction and inertia remain physical limitations.

## 3. Repaired and calibrated the encoder estimator

Files: `test/Core/Inc/encoder.h`, `test/Core/Src/encoder.c`.

- Defined angle zero as physical down and upright as pi radians. Positive angle
  near down moves the pendulum COM toward positive cart motion.
- Added explicit down-reference calibration and configurable encoder direction.
  An uncalibrated estimator rejects active control samples; startup checks raw
  count stationarity before establishing the reference.
- Removed the arbitrary startup angle offset. The first sample after calibration
  seeds history with zero velocity, preventing an invented derivative spike.
- Corrected circular differences in both directions and consistent degree/radian
  reporting. Changing direction invalidates the old reference.
- Added a first-order 20 Hz velocity filter. At 2,400 counts/revolution and 1 kHz,
  the unfiltered one-count derivative is approximately 2.618 rad/s; filtering
  makes that quantization less disruptive to feedback.
- Validate count range, resolution, sample time, direction and filter settings;
  reject jumps implying angular speed above the configured 50 rad/s default.
  Invalid samples do not overwrite the last accepted estimator history.
- Kept a thin TIM3 hardware wrapper and separated count processing so exactly the
  same estimator runs in host and nonlinear tests.

A stationary/disconnected encoder cannot be diagnosed from counts alone. A
stationary upright pendulum also cannot be distinguished from down without an
external reference; down positioning is a required startup condition.

## 4. Replaced inconsistent control laws with a shared C controller

Files: new `test/Core/Inc/pendulum_control.h`,
`test/Core/Src/pendulum_control.c`; compatibility adapters `lqr.h/.c`,
`swingup.h/.c`; updated `cartpole.m`.

The old code mixed a force-input model, acceleration commands and step units.
It also had an uninitialized feedback accumulator and an unparenthesized
metres-per-step expression. The new controller accepts metres, metres/second,
radians and radians/second and emits cart acceleration in metres/second².
Conversion to step acceleration occurs at the application boundary.

For the original assumed uniform rod with 0.35 m and 0.65 m arms:

```text
COM distance d = 0.15 m
pivot inertia J = 0.010583333 kg m², assuming mass 0.1 kg
L_eff = J/(m*d) = 0.70555556 m
energy from down to upright = 2*m*g*d = 0.2943 J
```

The effective dynamic length is not the COM distance. Geometry remains
unverified. The nonlinear equation and angle sign are now consistent:

```text
theta_ddot = -(g*sin(theta) + cart_acceleration*cos(theta))/L_eff
upright_error = remainder(theta-pi, 2*pi)
normalized_energy = 0.5*L_eff/g*omega² + 1 - cos(theta)
upright target energy = 2
```

Swing-up uses a bounded initial push to leave exact downward equilibrium,
energy feedback with the correct energy-derivative sign, cart centering, and
configured actuator limits. A stopping-distance speed envelope brakes before
endpoints; excessive position latches a travel fault. Invalid samples,
configuration and accumulated swing-up timeout also latch faults.

Balance uses acceleration-input state feedback with ideal continuous poles
`-2,-2,-3,-3 rad/s`, polynomial `(s+2)²(s+3)²`. The gains are derived from
`L_eff` in the shared implementation. The historical `lqr` filename is retained
as an adapter; this is now pole placement, not a newly optimized LQR cost.
Upright error wraps correctly on either side of pi. Physical correction-sign
checks are included in the tests.

Capture requires angle error below 0.15 rad, angular speed below 1 rad/s, and an
initially unsaturated balance command for 20 ms. A wider 0.55 rad / 3 rad/s exit
window avoids rapid mode chatter. The measured home coordinate is preserved
when entering balance. A fault means an actual motor stop, not simply passing
zero acceleration to a moving motor.

`cartpole.m` now documents the same acceleration-input linear model, geometry
and gain derivation, asserts the characteristic polynomial and plots an ideal
unsaturated response without requiring the Control System Toolbox. It no longer
presents the old force-to-acceleration conversion as the firmware design. The
MATLAB script was not executed here; C tests and nonlinear simulations provide
the executable validation.

## 5. Integrated the application lifecycle and commissioning configuration

Files: new `application.h/.c`, `app_config.h`; updated `main.c`, `main.h`,
`stm32f4xx_it.c/.h`.

Removed the disconnected/demo startup behavior, including the arbitrary
`motor_drv.position = 0.9` assignment. Main now initializes the driver, encoder,
explicit homing settings and shared controller, then services homing requests.
The 1 kHz TIM1 callback owns lifecycle transitions and active control.

Added:

- Configuration gate, debounced B1 start/stop and down-reference stationarity
  detection. Zeroing has a 10-second deadline.
- Foreground homing with interrupt-side encoder sampling and cancellation.
  The foreground posts results; the control tick consumes them, avoiding two
  independent writers of application state.
- Measured rail limits derived from successful homing and steps-to-metres
  calibration. No guessed initial cart position is used.
- Post-home down settling for one second, with a 30-second deadline, before
  automatic swing-up.
- `APP_RUN_MODE=1` for stopped homing-only checks, `2` for manual upright capture
  and balancing only, and `0` for automatic swing-up/balance. Balance-only mode
  stops on loss of capture instead of beginning an automatic swing-up.
- Latched input, endpoint, motor, controller, timing and user-stop faults. Board
  reset is required to rearm the supplied application.
- Debug expressions for raw switches, application/controller/motor faults,
  measured travel, pulse position, encoder state, acceleration and cycle time;
  LD2 indicates the BALANCE state.

TIM3 is sampled synchronously at the control rate. The old TIM4/DMA configuration
remains generated but is not started, avoiding concurrent estimator updates.
SysTick runs above TIM2's pulse interrupt, which runs above TIM1's control
interrupt. TIM1/TIM2 freeze during a debug halt; debug pause is not a substitute
for stopping the physical mechanics.

TIM1's reload is derived from `APP_CONTROL_HZ`; TIM3's reload from `CPR*RES`.
Startup verifies the timer clocks and STEP prescaler against configured values,
so edits cannot silently desynchronize timing and estimator assumptions.
DWT cycle measurements latch a fault when callback execution consumes a full
period or the interval exceeds 1.5 periods. CPU timing itself has not been
measured on hardware. Hard fault/error handlers disable STEP output directly.

All hardware assumptions are exposed in `app_config.h`: switch polarities and
pulls, direction mapping, step geometry, control/timer rates, motor envelope,
homing limits, dynamic length and rail margin. Defaults are marked unverified.

## 6. Added reproducible host tests and nonlinear simulation

### Host regression

Files: `tests/run_host_tests.py`, `tests/host/fake_hal.h`,
`homing_sim.c`, `encoder_test.c`, `control_test.c`, `application_test.c`,
`lqr_test.c`, and rewritten `tests/README.md`.

The runner compiles production C with `-Wall -Wextra -Werror`, optionally adding
UndefinedBehaviorSanitizer. It executes 32 motor scenarios plus four module
suites: **36 invocations**, each containing relevant assertions. It writes
source-hashed reports in `build/host-tests/`.

Coverage includes starting at either endpoint, reversed mapping, active-LOW and
mixed polarity, contradictions, stuck/missing switches, bounce and restart
races, stalled/failed timers, cancellation, tick wrap, delayed polling, odd and
out-of-range track spans, command bounds, pending pulses, real direction reversal,
no phantom software-UG steps, calibrated encoder wrapping/filtering, controller
capture and faults, real uncalibrated application startup, handoffs, staged modes,
state reference preservation and timing faults.

The application suite isolates the motor boundary; motor tests separately
execute the real driver against a modeled timer. This is not a complete CPU or
microcontroller emulator. The missing-B-switch test intentionally permits 100
attempted steps into a mechanical end before its bounded failure: PASS does not
mean a broken switch can always be handled without physical contact.

### Nonlinear software-in-the-loop regression

Files: `simulation/plant_sim.c`, `run_simulation.py`, `README.md`, optional
`plot_results.py`, replay `viewer.html`, `viewer_build.py`, `viewer-data.js`,
and CSV/JSON results.

Runs the actual `pendulum_control.c` and `encoder.c` against a nonlinear RK4
plant at 1 ms, with 2,400-count encoder quantization, the actual 20 Hz estimator
filter, quantized cart position, finite rail, actuator limits, damping and a
smooth friction scenario. No separate Python controller, MATLAB dependency or
external simulator installation is required.

Successful cases must remain in BALANCE throughout the last ten seconds, keep
peak upright error below 0.05 rad and cart RMS below 0.025 m. Results on the
hypothetical 3 m/s², 1 m/s, 0.90 m total-rail profile:

| Scenario | Capture time | Result |
|---|---:|---|
| Centered downward start | 12.005 s | Sustained balance; final angle RMS 0.00125 rad, cart RMS 0.00478 m |
| Offset start | 56.043 s | Sustained balance |
| Mirrored offset start | 58.930 s | Sustained balance |
| Effective length +10%, doubled damping | 12.043 s | Sustained balance |
| 0.001 N m smooth Coulomb friction | 12.268 s | Sustained balance |
| Near-upright start, angular disturbance at 8 s | 0.019 s | Captures and recovers |
| Original restrictive motor limits | No capture | Expected latched timeout at 90.037 s |

The last row is a handled-failure test. Original defaults are approximately
0.314 m/s² and 0.157 m/s. Successful stronger limits have **not** been applied
to real firmware configuration. An additional temporary search over 2,106 unique
energy/position/velocity gain triples found no successful nominal swing-up at
the original limits. That experiment did not change production defaults and
is not a proof that another controller or trajectory cannot work.

The offline viewer replays saved trajectories with scenario selection, play,
scrubbing, angle/cart plots and CSV loading. Assertions execute separately in
the C simulation. Browser rendering verification could not be completed because
the browser security check was unavailable; JavaScript/Python syntax checks
passed. No browser permission restriction was bypassed.

## 7. Build and documentation

- Added `tools/build_firmware.py`: discovers ARM GCC, builds all Core/HAL/startup
  sources for Cortex-M4 hard-float, links with the provided flash script, emits
  ELF/BIN/map and a source-hash manifest. Defaults to `-O2`. Warnings are errors;
  only vendor HAL unused-parameter warnings are suppressed.
- Added `test/makefile.defs` to include new Core sources when using the existing
  generated CubeIDE makefile. CubeIDE's Core source entry also discovers them
  after normal project regeneration. No generated makefile was hand-edited.
- Added `.gitignore` entries for standalone build and Python cache outputs.
- Added root `README.md`, `docs/COMMISSIONING.md` and this changelog; replaced
  the previous tests guide that described unrepaired homing and expected failures.
- Left bundled vendor HAL, startup assembly and linker script unchanged. The
  generated DMA/TIM4 setup remains dormant, as documented above.

### Validation performed

```sh
python3 tests/run_host_tests.py
python3 tests/run_host_tests.py --sanitize
python3 simulation/run_simulation.py
python3 simulation/viewer_build.py
python3 tools/build_firmware.py
```

The host suite has 36 passing invocations and the nonlinear suite has seven
passing expected outcomes. ARM GCC 14.3.1 compiles/links 29 source files; outputs
are in `build/firmware/`. The existing generated CubeIDE Debug make path was
also rebuilt to check inclusion of the new modules. Nothing was flashed.

### Earlier small fixes, superseded by the integrated repair

Before this broader request, three local edits fixed the `motor_drv->position`
member-access compiler error, initialized the old LQR accumulator, and
parenthesized the metres-per-step macro. Initial host tests then reproduced the
homing faults. Those edits alone did not repair homing or establish stability.
The current implementation replaces those partial paths and the corresponding
old test documentation; it should be assessed as the complete change above.

## Remaining physical work

Follow `docs/COMMISSIONING.md` in stages: inspect inputs, confirm directions and
units, test homing only, test manual upright capture, then test swing-up. Record
real travel, pendulum dynamics, switch behavior and loaded actuator capability.
The simulator omits torque/current limits, motor lag, stiction, missed steps,
belt elasticity, wiring faults and actual interrupt timing. No claim is made
that the unavailable plant is already physically stabilized.
