# Testing and commissioning

## What you can do now, without the plant

From the project root:

```sh
python3 tests/run_host_tests.py --sanitize
python3 simulation/run_simulation.py
python3 simulation/viewer_build.py
python3 tools/build_firmware.py
```

These commands use the installed Python, host GCC, and ARM compiler. They do not
connect to USB or flash a board. The first command exercises the actual firmware
C modules with simulated peripherals. The second drives the actual controller
and encoder estimator with a nonlinear simulated pendulum. Open
`simulation/viewer.html` in your browser to play the trajectories, select a
scenario, or load a CSV. See `simulation/README.md` for model assumptions.

The simulated switches are ordinary C functions in
`tests/host/homing_sim.c`, not a device to buy or an IDE component to install.
`HAL_GPIO_ReadPin()` is replaced in the host executable. Its output changes as
an imaginary cart reaches either end. Try:

```sh
python3 tests/run_host_tests.py --case at_b
python3 tests/run_host_tests.py --case active_low
python3 tests/run_host_tests.py --case b_never_pressed
```

The final case passes only because the failure is detected and movement stops
within a bound. It deliberately records 100 attempted steps into a mechanical
end before stopping. Software with no independent position sensor cannot know
that a missing switch concealed an endpoint. See `tests/README.md`.

## What the default firmware does

`test/Core/Inc/app_config.h` deliberately has `APP_HARDWARE_CONFIRMED=0`.
The board samples raw switches and encoder counts but does not produce motor
STEP motion. This is because the physical wiring, signs and motor capability
are unknown. Setting this flag does not itself verify them.

The driver reset/sleep pin is released during initialization, so a connected
powered driver can energize its windings even while STEP motion is disabled.
For board-only checks, leave the motor power/plant disconnected.

After verified settings and `APP_HARDWARE_CONFIRMED=1`, press and release the
NUCLEO B1 user button while the pendulum hangs **down** and is stationary. The
application observes a stationary count window for one second, establishes
that down reference, and runs homing. The encoder is incremental: it cannot
recognize a physically upright but motionless pendulum as the wrong reference.

Use `APP_RUN_MODE` to choose the next stage:

| Value | Mode | Behaviour after successful homing |
|---|---|---|
| `1` | Homing only | Holds stopped at the measured midpoint (`APP_HOMED_IDLE`). Start with this for commissioning. |
| `2` | Manual upright capture | Holds stopped (`APP_WAIT_BALANCE`) while you raise the pendulum. Engages balancing only inside the angle/rate/acceleration capture guard. Losing capture latches a stop. |
| `0` | Swing-up and balance | Waits for the pendulum to settle down, then pumps energy and automatically transitions to balancing. This is the selected run mode in the supplied source, behind the configuration gate. |

A new B1 press during an active session requests a latched stop, including during
homing. Reset the board to start a new session and recalibrate. B1 is a software
stop request, not an independent power disconnect. Physical endpoint contact
outside homing latches a stop; automatic reversal is specific to homing.

## Optional STM32-only checks

You can build without plugging in the board. When ready for board checks, use
the ST-LINK USB connector with STM32CubeIDE; build the `test` project and choose
a debug session. The standalone build produces `build/firmware/pendulum.elf`
and `.bin`; either can be selected in an appropriate ST-LINK workflow. No board
has been flashed as part of this repair.

Keep `APP_HARDWARE_CONFIRMED=0`. In the debugger's live expressions inspect:

| Expression | Meaning |
|---|---|
| `app.mode` | Should remain `APP_WAIT_CONFIGURATION` |
| `raw_limit_a`, `raw_limit_b` | Raw GPIO values, independent of configured pressed polarity |
| `encoder.raw_count` | Current TIM3 count, before calibration |
| `encoder.calibrated`, `encoder.valid` | Expected false until a down reference is established |
| `motor_drv.pwm_on` | Must remain zero |
| `control_max_cycles` | Maximum control callback duration in CPU cycles |

Without switches, pins may float: a changing/random GPIO is not a firmware
reversal failure. Do not enable motion to work around floating inputs. Board-only
GPIO tests with a known input bias can verify digital reads, but cannot identify
the missing plant's wiring. A USB cable does not simulate switches or mechanics.

The control callback uses TIM1 at 1 kHz. TIM2 generates STEP on PA0 and counts
completed pulses. TIM3 reads quadrature encoder inputs PA6/PA7. Generated TIM4
and its DMA setup remain present but are not started. TIM1/TIM2 freeze on debug
halt; TIM3 remains live. Halt/resume is unsuitable for a balancing pendulum:
the mechanics continue moving, and a resumed session can fault or have stale
state. Reset after debugging an interrupted run.

## Settings to measure on the plant

Edit `test/Core/Inc/app_config.h` after measurement. Record the results rather
than guessing from whether a name says CW or left.

| Setting | Required observation |
|---|---|
| `APP_LIMIT_A_ACTIVE`, `APP_LIMIT_B_ACTIVE` | Record `raw_limit_a/b` with each switch released and pressed. Choose `GPIO_PIN_SET` for pressed HIGH, or `GPIO_PIN_RESET` for pressed LOW, independently. |
| `APP_LIMIT_A_PULL`, `APP_LIMIT_B_PULL` | Match the electrical input circuit; current `GPIO_NOPULL` assumes an external bias. |
| `APP_LIMIT_CW_MASK` | Which switch lies in the positive command direction: `1` for A, `2` for B. Current driver CW means DIR pin LOW. |
| `APP_ENCODER_DIRECTION` | With pendulum near down, a positive reported angle must move its COM toward positive cart motion. Use +1 or -1. |
| `APP_STEPS_PER_REV`, `APP_MICROSTEPS`, `APP_PULLEY_RADIUS_M` | Actual motor step angle, driver mode, and effective pulley radius. Verify measured travel per counted pulse. |
| `APP_MAX_STEP_RATE_HZ`, `APP_MAX_STEP_ACCEL` | Sustainable loaded motion without missed steps, in pulse/s and pulse/s². Do not substitute a datasheet no-load speed. |
| `APP_HOMING_*` | Slow approach/backoff, release distance/time, plausible total travel and deadlines for the real rail. |
| `APP_EFFECTIVE_LENGTH_M` | Dynamic length J/(m*d), derived from the actual pendulum or measured small-angle period T: L_eff = g*(T/(2*pi))². |
| `APP_TRAVEL_MARGIN_M` | Clearance from the calibrated endpoints including real stopping and positioning error. |

Encoder resolution is configured by `CPR` and `RES` in `encoder.h`; currently
600 cycles/revolution times quadrature x4 = 2,400 counts/revolution. Confirm with
one complete mechanical revolution. TIM3 wrap is derived from these values at
startup. TIM1's period is derived from `APP_CONTROL_HZ`; the clock tree is checked
against `APP_TIMER_HZ` before motion. Keep 1 kHz unless you also retest the
controller, estimator, interrupt budget and all simulations at a new rate.

Useful geometry formulas:

```text
metres_per_step = 2*pi*pulley_radius / (full_steps_per_rev * microstep_factor)
measured_rail_metres = motor_drv.travel_steps * metres_per_step
cart_x_metres = motor_position_steps(&motor_drv) * metres_per_step
```

The current example uses 200 full steps/revolution, full stepping and a 5 mm
radius, giving 0.00015708 m/pulse. It has not been measured. A 0.90 m rail is a
simulation assumption, not a hard-coded measurement in the application: actual
successful homing sets the control rail bounds from counted travel.

## Staged plant test

1. **Inputs and direction first.** Keep STEP motion gated while checking each
   switch and encoder electrically. Identify DIR-to-cart direction through a
   controlled, low-speed bench jog with clear travel and the pendulum secured;
   the gate does not provide an automatic jog UI. Confirm the actual driver part,
   supply/current settings and microstep wiring against its own datasheet. The
   repository's driver PDF and source naming do not identify the fitted part
   consistently; the motor catalogue likewise contains multiple variants.
2. **Homing only.** Select `APP_RUN_MODE=1`, measured settings and the confirmed
   hardware flag. Begin down/stationary, press B1, then observe both approaches,
   release, slow relatching and return to center. Repeat from near each end. The
   normal switch should actuate before the physical hard stop. Compare counted
   travel to measured travel. Test B1 stop before increasing speed.
3. **Balance alone.** Select mode 2 and repeat down calibration/homing. Raise
   the pendulum carefully near upright; motion engages automatically once the
   capture guard has held for 20 ms. Have clear cart travel before entering that
   window. Verify the correction sign: a rightward lean requires rightward cart
   acceleration. Review angle, velocity, acceleration and fault values. Original
   motor limits allow only a very small capture region; increase limits only
   after loaded actuator tests.
4. **Swing-up.** Select mode 0 only after homing, signs and balance are checked.
   Repeat down calibration/homing. It settles, swings, captures and keeps the
   same center reference. Use measured parameters in the simulator before
   trying a revised profile. A 90-second accumulated swing-up timeout stops an
   unsuccessful attempt; it does not endlessly run into saturation.

Do not increase a travel budget merely to suppress a homing fault. A stuck or
missing switch, wrong mapping, lost steps, or an incorrect steps/metre setting
requires diagnosis. Stopping STEP output cannot guarantee instantaneous physical
stopping under inertia, nor can counted pulses detect a stall or belt slip.

## What success and failure look like

Inspect `app.mode`, `app.fault`, `motor_drv.fault`,
`app.controller.output.fault`, `app.commanded_acceleration`,
`motor_drv.fcurrent`, `motor_drv.step_position`, `motor_drv.travel_steps`,
`encoder.rad_angle`, and `encoder.w`. LD2 is on while the application is in
`APP_BALANCE`. No serial telemetry or remote start protocol is implemented.

- `APP_FAULT_NOT_STATIONARY`: down zeroing exceeded 10 seconds, or automatic
  post-home settling exceeded 30 seconds.
- `APP_FAULT_HOMING`: examine the motor fault for switch/release/travel/timeout
  detail. A failed home never enables the controller.
- `APP_FAULT_ENCODER`: invalid count, uncalibrated active estimator or
  implausible count jump. A motionless disconnected encoder cannot be detected
  from counts alone.
- `APP_FAULT_LIMIT` / `APP_FAULT_MOTOR`: endpoint contact or motor subsystem
  fault during control. PWM is stopped and velocity command cleared.
- `APP_FAULT_CONTROLLER`: inspect the controller reason: invalid input,
  configuration, excessive travel or swing-up timeout.
- `APP_FAULT_DEADLINE`: callback duration reaches one period, or an interval
  exceeds 1.5 periods. Check interrupt load and build optimization. The
  standalone firmware build uses `-O2`; CPU timing has not been measured here.
- `APP_FAULT_CAPTURE_LOST`: in balance-only mode, the pendulum left the balance
  region; the application stopped instead of starting a swing-up.

All these faults latch until reset. The library exposes a motor fault-clear API
for deliberate future recovery logic, but the application does not automatically
restart after a fault.

## Limits of the evidence

The supplied software passes host tests and nonlinear selected-scenario tests.
It has not balanced the real plant. Successful swing-up simulations use an
ideal acceleration actuator capped at **3 m/s² and 1 m/s**, on **0.90 m total
rail**. Real defaults retain **0.314 m/s² and 0.157 m/s** and fail to swing up in
the regression scenario, producing a timeout. This does not prove that every
possible controller fails at those limits. It demonstrates that motor authority,
friction and calibration must be resolved rather than hidden by a code change.
