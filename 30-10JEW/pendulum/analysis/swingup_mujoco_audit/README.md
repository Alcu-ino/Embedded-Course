# Investigation of the approximately 100° swing-up limit

Inspected on 2026-10-28. The simulation was found at:

`/home/lorenzo/Downloads/27-10/SIMULAZIONE/Embedded-Course-SIMULAZIONE`

It uses MuJoCo, with the STM32 running the controller and exchanging measurements/commands over serial (hardware in the loop, HIL). It does not use Simulink. The original firmware and simulation files were not edited.

**What was reproduced**

This diagnostic executes the simulation project's actual `swingup.c` against its actual MuJoCo XML model and `Pianta` class, in a deterministic 1 ms loop. It includes the 2400-count/revolution encoder quantization and angular-velocity filter. It substitutes ideal command-velocity integration and command-position counting for the board's motor/timer implementation.

It does **not** reproduce USB delays, dropped/repeated serial measurements, hardware STEP/DIR behavior, lost motor steps, TIM5 counting faults or the LQR controller. Consequently, it identifies mechanisms present even without those additional issues; it is not an exact reconstruction of the colleague's run or a hardware validation.

The baseline reads its constants from the compiled firmware headers and the `FMAXmotor` literal in `main.c`. `results/summary.json` records SHA-256 hashes of the inspected source files and the MuJoCo version, 3.17.0. If the source changes, rerunning can produce different results.

**Results**

Angles below mean angular displacement from hanging down, folded into 0–180° so that both swing directions are comparable. Down is 0°, horizontal is 90°, upright is 180°.

| Experiment | Largest angle | Simulated duration | Outcome |
|---|---:|---:|---|
| Current parameters | 107.42° | 24.344 s | Cart reached simulated rail switch |
| Only hinge damping changed from 0.01 to 0 | 160.08° | 6.390 s | Swing-up function reached its early-return threshold |
| Only hinge damping halved to 0.005 | about 160° | 12.118 s | Same threshold; very little rail margin |
| Only sign threshold EPS changed from 0.4 to 0.05 | 69.52° | 30 s | Still oscillating below upright |
| Only gain K changed from 100 to 4 | 107.42° | 24.344 s | Identical to baseline |
| Only command speed limit raised from 0.471 to 0.8 m/s | 17.20° | 0.825 s | Cart reached rail switch early |
| Contacts disabled, otherwise baseline | 107.42° | 24.344 s | Identical to baseline |
| Matched energy constants, reference 2.2 mgl, K=4, KV=2, original EPS=0.4 | 101.16° | 30 s | Stayed within ±0.310 m but did not reach upright |

The last row changes multiple parameters. It is an example of why matching parameters and restoring velocity feedback alone are not a proven complete fix. The additional `moderate_gain` scenario also changes multiple parameters and must not be interpreted as a gain-only experiment.

Reaching approximately 160° is **not** successful upright stabilization. This harness deliberately stops when `swingup_update()` returns `SWINGUP_CATTURA`. It does not call LQR. The inspected snapshot has swing-up returning inside ±20° of upright, while main selects LQR only inside ±10°. That separate handover gap still needs correcting in this snapshot, but cannot explain a failure near 100°. The user's requested assumption that gaps are removed does not change the diagnosis below.

Baseline peaks were still growing slowly: about 100.9°, 102.2°, 103.6°, 104.7°, 105.7°, 107.4° before the rail termination. Thus the result is a similar failure in the 100° region, not proof of a mathematically fixed 100° barrier or indefinite plateau.

**Why a large acceleration request does not guarantee a large swing**

The signal path is:

1. Swing-up calculates an acceleration from the estimated energy error and the swing phase.
2. It limits that acceleration to ±3 m/s².
3. `motor_acc()` integrates acceleration to obtain a velocity command and limits that velocity.
4. Python sends the velocity command to MuJoCo's velocity actuator. The acceleration field in the serial packet is logged, not directly applied to the plant.

The current conversion is `M_PER_STEP = (0.0314159 * 0.005) / 32 = 0.000004908734375 m/pulse`. With `FMAXmotor=96000`, the speed limit is approximately 0.4712385 m/s. From rest, an acceleration of 3 m/s² reaches that speed in 0.4712385/3 = 0.1571 s. Continuing to request +3 after reaching +0.471 does not increase the velocity command. The actuator then mainly maintains speed; the actual cart acceleration need not equal the requested acceleration.

At the peak of a 100° swing, using the firmware's own energy constants and nearly zero angular velocity:

```
MGL = 0.1 * 9.81 * 0.7056 = 0.6921936 J
E = MGL * (1 - cos(100°)) ≈ 0.81239 J
E_reference = 10 * MGL = 6.921936 J
raw pumping magnitude ≈ 100 * (6.921936 - 0.81239) ≈ 611 m/s²
after acceleration limiting: 3 m/s²
```

Even K=4 still requests about 24.4 m/s² there. Both requests become 3 m/s². This explains the identical gain-only test. The baseline command spent about 63% of controller samples at the speed limit as well.

With the simulation's sign convention, rotational energy changes approximately as

`dE/dt = m*d*a_cart*omega*cos(theta) - b*omega²`.

Here `d` is the center-of-mass distance perpendicular to the pivot, `omega` is angular velocity, and `b` is hinge damping. Cart acceleration can add or remove energy depending on its timing; damping always removes it. At exactly horizontal, the cart-to-pendulum energy-transfer term is momentarily zero. This does not create a 90° barrier: previously accumulated kinetic energy must carry the rod past horizontal.

The XML hinge damping is 0.01 N m s/rad. At 4 rad/s it dissipates 0.01*4² = 0.16 W. This is an illustration at that speed, not the average loss of a complete swing. The damping ablation demonstrates that this modeled loss strongly affects the result. It does not establish whether 0.01 accurately represents the physical mechanism.

The energy-controller and centering terms are added **before** limiting. Near the rail, the maximum position-centering term is only 2*0.462 = 0.924 m/s², compared with a raw pumping request of hundreds. Subtracting 0.924 from approximately 611 still produces +3 after clipping. Moreover, KV is currently zero. Consequently, the current centering correction has almost no authority while the pumping term is deeply saturated.

**Model mismatch to correct before interpreting gains**

The XML comments describe rod arms of 0.877 m and 0.100 m, but the actual `fromto` coordinates use 0.82 m and 0.16 m. MuJoCo uses the coordinates, not the comments. The rotating assembly includes more than the rod, including its shaft and hub. For the XML as inspected, MuJoCo gives:

| Quantity | Firmware energy model | Current MuJoCo model |
|---|---:|---:|
| Rotational inertia about pivot, J | 0.0497871 kg m² | 0.0285435 kg m² |
| Gravitational energy coefficient, m*g*d | 0.692194 J | 0.489000 J |
| Energy of stationary upright state, 2*m*g*d | 1.38439 J | 0.977999 J |
| Effective pendulum length, J/(m*d) | 0.7056 m | 0.572622 m |

The simulated rotating assembly mass is 0.395396 kg and its center of mass is 0.126069 m from the pivot axis. Effective length is not the physical center-of-mass distance. Substituting the assembly mass and effective length into `J=m*L²` would be incorrect.

For **this XML only**, a consistent energy calculation is:

```c
const float J_pivot = 0.0285435377f;
const float MGL = 0.4889996742f;
float E = 0.5f * J_pivot * omega * omega
        + MGL * (1.0f - cosf(theta));
float E_upright = 2.0f * MGL;
```

These are model-derived numbers, not measurements of the real plant. First settle the intended geometry/materials and pivot location, then regenerate these quantities. Changing the pivot changes both inertia and gravity torque. The LQR/observer model and gains should subsequently be recalculated for the same plant. Changing only one `L` macro does not automatically regenerate compiled LQR gains.

**Recommended correction sequence**

1. Verify which source, compiled firmware, XML and command-line options the colleague used. Save the serial CSV and determine whether the run ends at a rail limit or continues with similar peaks. The deterministic baseline cannot settle that distinction for their session.

2. Make the XML geometry and physical parameters agree with the intended plant. Replace conflicting comments and constants. Calibrate meters per motor pulse from pulley pitch circumference, full steps per revolution and microstep setting. The XML's visual pulley comment and firmware's 5 mm radius currently describe different transmissions; the visual pulley does not automatically change the firmware conversion or drive physics.

3. Identify hinge loss. Hold the real cart fixed, displace the pendulum moderately from down and release it. Record successive angles and times; reproduce that free-decay experiment with the simulated cart held fixed. Fit the damping to the measured decay, adding a suitable non-viscous friction model if needed. Zero damping is useful as a diagnostic, not a justified final setting unless supported by measurements. If the actual mechanism has substantial loss, the controller must work with it, or the mechanism must be improved.

4. Use a physically consistent energy estimate and an upright target near `2*MGL`. Any intentional excess must be tuned together with the capture speed and available cart travel; `10*MGL` means five times the controller model's stationary-upright energy. Do not expect changing this target alone to cure the limit. The matched-parameter experiment above did not.

5. Give centering and rail protection actual authority. Limit the pumping term before combining it with centering, allocate some acceleration to centering/braking, and enforce speed and rail constraints explicitly. A useful structure to develop and validate is:

```text
pump = limit(energy_gain * energy_error * pumping_phase, pumping_budget)
center = -position_gain * cart_position - velocity_gain * cart_velocity
acceleration = limit(pump + center, total_acceleration_limit)
acceleration = apply_speed_and_rail_constraints(acceleration, position, velocity)
```

This is an implementation structure, not a validated controller or ready-to-use set of gains. The pumping budget must leave useful correction authority. Close to an end, braking must take priority over adding swing energy. At 0.471 m/s and ideal braking of 3 m/s², the kinematic stopping distance alone is `v²/(2*a) ≈ 0.037 m`; actual actuator response, pendulum coupling and delay require additional margin. Start braking before reaching the limit switch, rather than using the switch as the normal turn-around command.

6. Tune the pumping phase with the velocity limit included. The classical acceleration-based energy rule does not automatically account for periods when its acceleration request cannot be executed. Examine actual cart acceleration, command speed, pendulum phase and energy change together. Arrange the cart motion so acceleration authority is available at useful swing phases. This may require changing the pumping policy or planning a constrained trajectory, not just increasing K. Validate the result at the measured damping and with realistic speed, acceleration and rail bounds.

7. Keep EPS changes separate from handover hysteresis. EPS applies to `omega*cos(theta)`, not an angle interval around upright. Within ±EPS the code retains the previous pushing sign. Lowering it changes push timing and noise sensitivity; the isolated reduction from 0.4 to 0.05 made this simulation worse. Do not treat it as an automatic improvement. Hysteresis between swing-up and balance does not fix a loss/saturation problem at 100°.

8. Compare deterministic results to board HIL results. The current board can execute controller ticks without a new angle packet, while Python advances once per received state packet. Serial delay, stale measurements and different effective clocks can further change the pumping phase. Log a true controller-tick counter and fresh-measurement flag; transmitted-frame sequence numbers alone cannot prove every tick was represented. A deterministic HIL mode should advance one controller step per fresh simulated sample, and use simulated-time-consistent cart position feedback as well. TIM5 continuing in wall-clock time must not silently remain the HIL position source in such a mode.

9. Once the swing reaches the upright neighborhood, validate handover and LQR separately, with angle, angular speed, cart speed and rail margin considered. Passing 160° in this diagnostic does not prove capture. Keep the already requested continuous controller coverage; an all-controllers-off angle gap does not address the present problem.

**Reproduction**

Requires Python, NumPy, MuJoCo, pyserial, and GCC on a Linux host. This directory is a host-side diagnostic; do not add `controller_bridge.c` to the STM32 CubeIDE build. It uses the minimal HAL declarations in the adjacent `../offline_audit` directory.

From this directory, with those dependencies available:

```sh
python3 run.py --source /home/lorenzo/Downloads/27-10/SIMULAZIONE/Embedded-Course-SIMULAZIONE --duration 30 --cases baseline no_damping half_damping small_eps moderate_gain no_contacts more_speed lower_gain_only matched_energy_same_eps
```

The session's dependencies were installed separately under `/tmp/pendulum-mujoco-deps`; they are not part of the project. The exact session invocation used:

```sh
PYTHONPATH=/tmp/pendulum-mujoco-deps PYTHONDONTWRITEBYTECODE=1 /home/lorenzo/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 run.py --duration 30 --cases baseline no_damping half_damping small_eps moderate_gain no_contacts more_speed lower_gain_only matched_energy_same_eps
```

`results/summary.json` contains aggregate results and source provenance. The CSV files contain samples every 5 ms, although control runs every 1 ms. `a_after_acc_limit` is already limited to ±ACC_MAX; it is not the unconstrained energy-controller request. `a_command_effective` is the change in command velocity divided by 1 ms after velocity limiting. `a_cart` is an estimate of actual cart acceleration from MuJoCo velocity differences.

`w_filter` and `E_est` describe the controller's pre-step sample; `w_true` and `E_true` describe the post-step plant, 1 ms later. The work/loss integrals are approximate diagnostic estimates, not exact solver energy accounting. Rail termination occurs at the Python switch criterion, approximately ±0.4615 m.

For the existing board simulation, the colleague can use the script's `--log`, `--durata` and optionally `--no-viewer` options, supplying their actual serial port. Its CSV fields `x`, `xd`, `v_cmd`, `p_scheda`, `acc_cmd`, `theta_deg`, `lqr`, and `ritardo_tick` help distinguish rail travel, speed saturation and HIL effects. Raw `theta_deg` wraps 0–360: for example, 260° represents a 100° displacement from down on the opposite side.

MuJoCo's velocity-actuator and damping semantics are documented in the [official XML reference](https://mujoco.readthedocs.io/en/latest/XMLreference.html). The quantitative findings above come from the local code/model and the included executions.
