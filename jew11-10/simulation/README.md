# Nonlinear software-in-the-loop simulation

Run from the project root, with Python 3 and gcc (or `CC=clang`):

```sh
python3 simulation/run_simulation.py
python3 simulation/run_simulation.py --case swingup
```

No STM32 board, USB connection, Python packages, MATLAB, or network access is needed. The script builds a temporary executable, runs the scenarios, and writes the trajectories to `simulation/results/*.csv` plus `summary.json`. A nonzero exit code means a failed assertion/scenario. Results include hashes of the C sources used, so old results can be distinguished from a later controller revision. To plot a CSV on a machine that has matplotlib, run `python3 simulation/plot_results.py simulation/results/swingup.csv`; plotting is optional and not part of the test pass criteria.

## Replay without a plant

Open `simulation/viewer.html` in a web browser. Choose a scenario, press Play, or scrub the time slider. The diagram shows the cart and assumed two-sided rod; the charts show upright error and cart displacement. The original-limit scenario is clearly labelled as a timeout, and every successful scenario is labelled with its unverified higher motor envelope. No network requests or server are needed.

After changing the C controller or simulation, run `python3 simulation/run_simulation.py` then `python3 simulation/viewer_build.py` to refresh the bundled replay. You can also select any generated trajectory CSV directly in the viewer. Replay is downsampled to 20 Hz; simulation assertions still check every 1 kHz integration step. JavaScript syntax was checked, but browser rendering could not be inspected because the browser tool security check was unavailable.

## What is real and what is simulated

`test/Core/Src/pendulum_control.c` and `test/Core/Src/encoder.c` are the same controller and encoder estimator used by the STM32 firmware. There is no separate Python controller. The small C file `plant_sim.c` supplies an ideal cart actuator, nonlinear pendulum dynamics, and quantized raw encoder counts. It integrates the nonlinear plant with fourth-order Runge–Kutta at 1 ms. It calls the actual encoder estimator at the same rate, including its 20 Hz velocity filter. Angle feedback is quantized to 2,400 counts/revolution (600 CPR × 4); cart feedback is quantized to the nominal 5 mm pulley / 200 full steps per revolution geometry.

The simulation is a software regression test, not proof of successful balancing on the unavailable plant. It cannot detect reversed physical wiring, switch electrical polarity, missed motor steps, inadequate drive current/torque, incorrect microstep wiring, backlash, belt elasticity, timing jitter, or physical rail length. Its ideal actuator follows the commanded acceleration immediately. Cart velocity is the commanded velocity, as in the firmware; a lost motor step cannot be observed without a cart sensor.

## Coordinates and physical model

- Cart `x=0` is the measured rail midpoint; positive x follows the configured positive motor direction.
- Pendulum angle `theta=0` is down. A small positive angle from down moves the bob toward positive cart x. This convention reverses the angle sign relative to the original `cartpole.m` formulation.
- Upright error is `remainder(theta - pi, 2*pi)`. A rightward bob lean about upright therefore has **negative** upright error.
- Equation: `theta_ddot = -(g*sin(theta) + a*cos(theta))/L_eff - b*omega - tau_f/J*tanh(omega/0.03)`.
- For the assumed uniform rod extending 0.35 m and 0.65 m either side of the pivot: COM distance `d=0.15 m`, mass `m=0.1 kg`, pivot inertia `J=0.010583333 kg m²`, and `L_eff=J/(m*d)=0.70555556 m`. Effective length is not the COM distance.
- Normalized energy is `E/(m*g*d)=0.5*L_eff/g*omega²+1-cos(theta)`; upright target is 2. Actual target energy under that rod assumption is 0.2943 J.

The friction case uses a smooth Coulomb approximation, not a static-friction/stiction solver. Parameters come from the repository model and explicitly stated assumptions; none have been measured on this plant during this task.

## Two deliberately distinct motor envelopes

**The firmware default preserves the old limits:** 2,000 full steps/s² and 1,000 full steps/s, equivalent to approximately **0.314 m/s² and 0.157 m/s** for the nominal pulley. The `legacy_limits` scenario is expected to fail to reach upright and to latch a timeout while staying within its position and speed limits. Its PASS means the failure is handled; it does not mean swing-up succeeded. At 0.314 m/s², even the instantaneous zero-angular-speed acceleration needed to oppose gravity exceeds the limit for roughly 1.83° of lean. A broad capture angle alone cannot fix insufficient authority.

**The successful swing-up scenarios use a hypothetical capable actuator:** maximum acceleration **3 m/s²**, speed **1 m/s**, and a **0.90 m rail** (0.45 m either side of center). Those correspond to about 19,099 full steps/s² and 6,366 full steps/s. They have not been enabled as the firmware defaults or certified for the real motor. The existing plant may require different gearing, microstepping, a different trajectory/controller, or a more capable drive. Never infer its capability from these simulation passes.

## Scenarios and assertions

| Scenario | Initial state / change | Expected result |
|---|---|---|
| `swingup` | Downward, centered, at rest; viscous damping 0.02/s | Capture upright, then maintain balance |
| `offset` | x=0.05 m, theta=0.05 rad, at rest | Capture upright from a perturbed start |
| `mirror` | x=-0.05 m, theta=-0.05 rad; mirrored departure | Capture with the opposite kickoff direction |
| `model_error` | Actual effective length +10%; damping doubled to 0.04/s | Capture with nominal controller parameters |
| `friction` | Added 0.001 N m smooth Coulomb friction | Capture and maintain bounded balance |
| `balance` | Manually placed 0.03 rad from upright; +0.2 rad/s angular impulse at 8 s | Capture and recover after perturbation |
| `legacy_limits` | Original restrictive motor envelope, down at rest | Latched timeout; no false success claim |

Every integration step checks physical rail, cart speed, and commanded acceleration limits. Successful cases must remain in BALANCE for the entire final 10 seconds, have peak upright error below 0.05 rad, and cart RMS displacement below 0.025 m. Successful swing-up runs last 80 seconds; manual balancing runs 25 seconds. This is a selected regression set, not a proof over arbitrary starts, rail lengths, friction, delays, or motor limits. Pumping near nonlinear capture boundaries can be sensitive to small changes; new measured parameters require rerunning the tests and further tuning.

## Controller design

The stateful controller uses a defined 0.2 s initial push near the stationary downward position, normalized-energy feedback, cart-position feedback, bounded acceleration, bounded speed, and a stopping-distance envelope before either endpoint. It latches invalid-input, travel, configuration, and swing-up-timeout faults. A zero-acceleration fault output must be interpreted as **stop the motor**, not passed as a request to coast.

Capture requires upright error below 0.15 rad, angular speed below 1 rad/s, and an initially unsaturated balance command continuously for 20 ms. BALANCE exits only beyond the larger 0.55 rad / 3 rad/s window, providing hysteresis. The homed cart coordinate is never reset during this transition.

Balancing uses acceleration-input state feedback derived for poles `-2,-2,-3,-3 rad/s`, replacing the old force-input LQR constants. Its characteristic polynomial is `(s+2)²(s+3)² = s⁴+10s³+37s²+60s+36`. The shared implementation derives gains from `L_eff` and gravity, uses the wrapped upright error from either side, and applies the configured motor/rail envelope afterward. Saturation, quantization, or insufficient actuator capability can invalidate the ideal linear stability prediction; the nonlinear tests exercise several such limits.
