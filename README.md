# Super Heavy pitch control (thrust-vector control)

Small-signal pitch attitude control for a reusable booster, through engine gimbal (thrust-vector control, TVC). The vehicle numbers match the companion simulator [matlab-rocket-sim](https://github.com/arussian00/matlab-rocket-sim) (`rocket_params.m`, `starshipStack.m`, `attitudeControl` inside `simulateBooster.m`). That repo is not modified.

The companion flies the stack as a point mass and, after separation, turns the booster with a rigid-rod PD law and an instantaneous gimbal. This project linearises that pitch axis, adds the pieces that law leaves out (aerodynamic moment, gimbal servo, rate and angle limits, sensor delay), and checks stability margins with Control System Toolbox. A Simulink model of the same loop is built by a script when you run it. The `.slx` is not in git: building it needs Simulink.

Designed for **MATLAB Online** with **Simulink** and **Control System Toolbox** (PID Tuner, Control System Designer, Linear System Analyzer are in that toolbox). It does not use the Aerospace Blockset or Simulink Control Design.

[![Open in MATLAB Online](https://www.mathworks.com/images/responsive/global/open-in-matlab-online.svg)](https://matlab.mathworks.com/open/github/v1?repo=arussian00/rocket-controls-simulink&file=run_pitch_tvc.m)

## 1. Run it

MATLAB Online (R2021a or newer; Online is current). In the Command Window:

```matlab
gitclone("https://github.com/arussian00/rocket-controls-simulink.git");
cd rocket-controls-simulink
run_pitch_tvc
```

`gitclone` needs R2023b or newer. On an older desktop install, clone the repo from the Files panel (right-click, Source Control, Clone Repository) or with git, then `cd` into the folder and run the same script.

What `run_pitch_tvc` does:

1. Prints the gain-scheduled PID, margins, step-response numbers, and three sweeps (sensor delay, actuator bandwidth, static margin).
2. Builds `boosterPitchTVC.slx` in the current folder and simulates it.
3. Opens **one** dashboard window, `Pitch TVC`, and prints a PASS/REVIEW checklist.

The dashboard is a browser figure with three tabs:

| Tab | What is on it |
|---|---|
| **Responses** | 5 deg pitch, linear (dashed) over Simulink (solid); gimbal angle and rate against the stops; gust response on its own scale; stats table |
| **Bode** | Loop gain for liftoff, max-q and landing, with phase margin, upward gain margin, gain-crossover circles and gain-margin squares |
| **Limits and disturbances** | 30 deg liftoff step on the stops, thrust-misalignment trim, open-loop drift at max-q |
| **Along the flight** | Phase and gain margins versus time for the 1/mu schedule and for a frozen liftoff gain. Black dots are the three design points. Red x marks samples under 6 dB or 30 deg |

| Command | What you get |
|---|---|
| `run_pitch_tvc` | Full design, model build, five simulations, one dashboard, checklist |
| `design_pitch_controller(vehicle_params())` | Margins and linear plots only (no Simulink) |
| `open_pitch_tuner("pid")` | PID Tuner on the liftoff plant |
| `open_pitch_tuner("pid", "max-q")` | Same, at max-q |
| `open_pitch_tuner("designer", "landing")` | Control System Designer |
| `open_system("boosterPitchTVC")` | The model, after `run_pitch_tvc` has built it |

The first run takes a few seconds of design work and a few seconds per simulation.

## 2. What is being controlled

One axis: pitch of the nose from the trim attitude, using one equivalent gimbal angle. The companion's sign is kept. Positive gimbal produces positive pitch acceleration (nose downrange). Torque is `F * arm * sin(delta)`, with the centre of mass at mid-length and `I = m*L^2/12`, the same rigid rod as `attitudeControl`.

Three frozen conditions, gain-scheduled. Liftoff and max-q are the full stack (Super Heavy + Starship). Landing is the booster alone on the three centre engines.

| | Liftoff | Max-q | Landing burn |
|---|---|---|---|
| What it is | 33 engines, vertical, on the pad | T+46 s, throttle bucket | 3 engines, ~1 km, ~55 t propellant left |
| Mass | 5260 t | 4224 t | 285 t |
| Length used for `I` and the gimbal arm | 121 m (71+50) | 121 m | 71 m |
| Throttle | 100% | 75% | 70% |
| Thrust / weight | 1.47 | 1.37 | 1.73 |
| Dynamic pressure | 0 | 22.37 kPa | 3.56 kPa |
| `mu = F*arm/I` | 0.716 /s² | 0.668 /s² | 1.432 /s² |
| Unstable pole | none (`q = 0`) | 0.050 rad/s | none (static margin 0) |

Max-q mass, altitude (5.88 km) and speed (259 m/s) are a snapshot of the companion's `starshipStack` ascent (33 Raptors, 75% throttle bucket, gravity turn). An RK4 integration of that model at 0.05 s reproduces the companion README: max-q about 22 kPa near T+46 s. Dynamic pressure in this repo is recomputed from that altitude and speed with the US76 troposphere, the same layer `earthModel.m` uses.

`mu` is the small-signal pitch acceleration per radian of gimbal. The schedule is `delta_cmd = a_cmd / mu`. One PID, in acceleration units, is used at every condition. Heavier vehicles and lower throttle ask for more gimbal for the same acceleration, and they hit the angle stop sooner. That is why the 30 deg liftoff case is the one that saturates.

## 3. Plant

Perturbations about a trim with zero angle of attack and zero gimbal. Pitch is measured from the trim attitude (the flight-path angle), so the flight-path angle does not appear as a state.

```
theta'' = lambda*theta + mu*sin(delta + delta_mis) + d_gust
delta   = Actuator(delta_cmd)
```

`lambda > 0` is statically unstable: the centre of pressure is ahead of the centre of mass, and a positive angle of attack produces a positive pitching moment. With the sign convention here, a positive crosswind `w` reduces angle of attack (`alpha = theta - w/V`). The gust enters as `d_gust = -(lambda/V)*w`. At zero airspeed the gain is zero.

**Aerodynamics (a design assumption, not in the companion).** Slender-body normal-force slope `C_Nα = 2` per radian on the frontal area. Static margin on ascent is **−0.5 caliber** (CP is 4.5 m ahead of the CG on the 9 m body). Landing margin is **0**: the companion has no body aero moment, and grid-fin stability is not in this model yet. At max-q that −0.5 caliber is a real instability, and it is a slow one. The time to double is about 14 s. TVC authority (`mu ≈ 0.67 /s²`) is about 270 times the aero stiffness (`lambda ≈ 0.0025 /s²`), so a 15 m/s gust (about 3.3 deg of angle of attack at 259 m/s) trims out with about **0.01 deg of gimbal**. The dedicated gust figure uses its own scale so that small motion is visible. Push `V.staticMarginAscent` more negative and re-run; the printed sweep goes out to −50 calibers, where the unstable pole is about 0.5 rad/s and the phase margin has only moved from 60 deg to about 59 deg. Aero is not what spends the margin on this vehicle. Actuator bandwidth and delay are.

**Actuator (a design assumption).** Second order, 6 Hz, damping 0.7, then hard limits: **±15 deg** (the companion's `gimbalMaxDeg` for Super Heavy) and **±20 deg/s**. The 6 Hz and 20 deg/s are sizing numbers for a large-engine servo, not identified hardware. In the linear analysis the limits are off. In Simulink they are integrator saturation on the rate state and the angle state, so the linear servo and the nonlinear servo are the same transfer function while the states are inside the stops.

**Sensor delay (a design assumption).** 20 ms of pure delay on pitch and pitch rate. The companion's gimbal and sensors are instantaneous.

**Command prefilter.** A 0.5 s first-order filter on the attitude command. It sits outside the feedback loop, so it does not change margins. A raw 5 deg step would ask the 6 Hz servo for well over 100 deg/s of gimbal rate. Guidance commands are smooth; with the prefilter a 5 deg step peaks near 14 deg/s at max-q, inside the 20 deg/s stop. The 30 deg liftoff step still hits both stops. That case is there to show the limits.

## 4. Controller

The companion law is

```
tau_cmd = I * (wn^2 * (theta_cmd - theta) - 2*zeta*wn * theta_dot)
delta   = asin(tau_cmd / (F*arm))
```

with `wn = 1 rad/s` and `zeta = 0.8`. For small gimbal that is exactly `a_cmd = wn^2*e - 2*zeta*wn*q`, then `delta = a_cmd/mu`. This project keeps that PD and adds three things the instantaneous law does not have:

| Term | Value | Role |
|---|---|---|
| `Kp` | `wn^2 = 1` /s² | attitude error to acceleration |
| `Kd` | `2*zeta*wn = 1.6` /s | rate damping, on the **measured** rate (no derivative kick) |
| `Ki` | `0.02*wn^3 = 0.02` /s³ | trims a constant moment (misalignment, aero) |
| `Tf` | 0.05 s | filter on the rate feedback |
| `Kaw` | 1 /s | back-calculation when `a_cmd` hits `±mu*delta_max` |

`Ki` is deliberately weak. The 5% settling time stays near 4 s, which is the rigid-body response the companion was tuned for. The last fraction of a degree takes longer (2% settling near half a minute) because the integrator is slow. Raising `V.attKiOverWn3` toward 0.1 trims faster and adds overshoot; PID Tuner is the place to make that trade.

`isstable` cannot judge a loop once `feedback` has turned the sensor delay into an internal delay. The `stable` column is therefore a 6th-order Padé model of that same loop (orders 4 and 6 are both checked). `margin`, `allmargin`, Bode and the step responses keep the exact delay. If a release refuses the exact delay in `step`, `bode` or the tuner apps, that call falls back to the Padé model and prints a warning.

Loop gain used for margins, with `C(s) = Kp + Ki/s + Kd*s/(Tf*s+1)`:

```
L(s) = C(s) * Actuator(s) * 1/(s^2 - lambda) * exp(-s*Td)
```

The command response in the plots is not this 1-DOF loop. The derivative is on measured rate, and the command goes through the prefilter. Margins match the 1-DOF PID. The step shape matches the Simulink diagram.

### Margins (independent NumPy check)

Same plant and PID, frequency response on a dense grid, closed-loop poles of a 8th-order Padé delay. `run_pitch_tvc` recomputes these with `margin` and `allmargin`. Expect agreement within about 0.3 deg and 0.3 dB.

| Condition | Phase margin | At | Upward gain margin | At | Extra delay margin | Stable |
|---|---|---|---|---|---|---|
| Liftoff | 60.0 deg | 1.73 rad/s | 21.2 dB | 15.0 rad/s | 0.60 s | yes |
| Max-q | 60.0 deg | 1.73 rad/s | 21.2 dB | 15.0 rad/s | 0.60 s | yes |
| Landing | 60.0 deg | 1.73 rad/s | 21.2 dB | 15.0 rad/s | 0.60 s | yes |

`margin` in MATLAB reports one gain margin, the one closest to 0 dB. That is the **upward** margin, about 21 dB. There is a second, downward margin near 0.11 rad/s (about 36 to 38 dB) from the integrator: the loop gain can fall that far before the low-frequency branch loses the Nyquist encirclement. `allmargin` lists both. The printed report labels them.

Command response to a 5 deg step, prefilter on, limits off (fixed-step check of the same nonlinear-free equations; `stepinfo` in the script is the number to keep):

| | Liftoff | Max-q | Landing |
|---|---|---|---|
| 10–90% rise | 2.6 s | 2.6 s | 2.6 s |
| Overshoot | 3.5% | 3.8% | 3.5% |
| 5% settling | 3.8 s | 3.8 s | 3.8 s |
| Peak pitch rate | 2.0 deg/s | 2.0 deg/s | 2.0 deg/s |
| Peak gimbal | 3.1 deg | 3.3 deg | 1.5 deg |
| Peak gimbal rate | 13 deg/s | 14 deg/s | 6 deg/s |

All three 5 deg cases stay inside ±15 deg and ±20 deg/s. The linear model and the Simulink model should overlie on that run. The checklist flags the RMS between them up to the gust.

A 0.5 deg thrust misalignment (a nozzle bias in the same direction as positive gimbal) is held by commanding about −0.5 deg of gimbal. Pitch returns to the command. That is the disturbance the integrator is there for. The wind gust is in the model as well; at these inertias it barely moves the gimbal.

### What actually spends the margin

At max-q, same gains, from the same NumPy check. The script prints the MATLAB version of each table.

Sensor delay:

| Delay | 0 | 10 ms | 20 ms (design) | 50 ms | 100 ms |
|---|---|---|---|---|---|
| Phase margin | 62 deg | 61 deg | 60 deg | 57 deg | 52 deg |
| Upward gain margin | 24.5 dB | 22.7 dB | 21.2 dB | 18.0 dB | 14.4 dB |

Actuator natural frequency (damping 0.7, delay 20 ms):

| Bandwidth | 3 Hz | 4 Hz | 6 Hz (design) | 8 Hz | 10 Hz |
|---|---|---|---|---|---|
| Phase margin | 56 deg | 58 deg | 60 deg | 61 deg | 61 deg |
| Upward gain margin | 17.3 dB | 19.0 dB | 21.2 dB | 22.6 dB | 23.7 dB |

Static margin at max-q from 0 to −50 calibers leaves the phase margin between 60 and 59 deg. The open-loop drift figure is still worth looking at: from 1 deg of initial pitch, the uncontrolled max-q pole walks off over a minute, and the loop holds it.

## 5. Schedule along the flight

`data/superheavy_booster_trajectory.csv` is a 1 Hz RK4 history of the companion Super Heavy mission: stack ascent to hot staging, then flip, boostback, coast, brake and landing. It carries mass, throttle, engine count, thrust, dynamic pressure, Mach, speed, pitch inertia, the mid-body centre of mass, and the centre of pressure from the static-margin assumption. Flip and coast are in the table with `tvc_active = 0` (engines off). There is no entry burn (`engEntry = 0`).

`make_booster_trajectory` rewrites that file with the companion's `ode45` once `matlab-rocket-sim` is on the path. Until then the RK4 snapshot is what `run_pitch_tvc` loads.

The normalised PID does not change. The Simulink lookups make `mu`, `lambda` and `1/mu` functions of flight time, so the gimbal gain tracks control effectiveness. With that schedule the linear margins stay near **60 deg and 21.2 dB** wherever the engines are lit (they only move with `lambda`, which stays near zero on this aero model). A liftoff gain frozen in gimbal units does not. A NumPy check of the same loop puts the worst fixed-gain point at **t = 185 s, boostback: about 16 deg and 2.7 dB** (`mu` about 6.1 /s², 8.5 times liftoff). Brake ignition is also under 6 dB. The Along the flight tab draws both curves, marks liftoff, max-q and the 1 km landing point, and puts a red x on fixed-gain samples under 6 dB or 30 deg. `run_pitch_tvc` prints the same worst cases from Control System Toolbox.

## 6. Simulink model

`build_pitch_model.m` creates `boosterPitchTVC` with `new_system`, `add_block`, `add_line` and `set_param`, using only the standard Simulink library, then saves `boosterPitchTVC.slx`.

```
theta_cmd (Step)
    -> Controller  -> delta_cmd -> Actuator -> delta -> Plant -> theta, q
           ^                                                   |
           +------------------------ theta, q -----------------+
```

* **Controller** (light blue): command prefilter, 20 ms delay on pitch and rate, PID into an acceleration command, saturation at `±mu*deltaMax` with the existing back-calculation, and `1/mu`. `mu` and the saturation limit come from 1-D lookup tables versus time.
* **Actuator** (orange): `wn^2` and `2*zeta*wn` around two integrators. The rate integrator saturates at `±gimbalRate`. The angle integrator saturates at `±deltaMax`.
* **Plant** (green): `mu(t)*sin(delta + misalign) + lambda(t)*theta + gustAccPerWind*wind`, then two integrators. `mu` and `lambda` are the same lookup breakpoints. Misalignment and wind are Step blocks inside the plant.
* Scopes on attitude, gimbal and gimbal rate. To Workspace logs `theta`, `q`, `delta`, `deltaCmd`, `gimbalRateMeas`, `thetaCmd` as timeseries.

Every number the diagram uses is a base-workspace variable (`Kp`, `mu`, `lambdaAero`, `tStop`, ...). `apply_pitch_condition` writes them. If you press Run in the model with an empty workspace, `pitch_model_init` loads the liftoff case.

The nominal script runs:

| Run | Command | Gust | What to look at |
|---|---|---|---|
| Liftoff, max-q, landing | 5 deg step at 1 s | 15 m/s at 14 s | Three curves on top of each other. Gimbal a few degrees. Rate under 20 deg/s. |
| Liftoff, large | 30 deg step | off | Nozzle on the ±15 deg stop, rate on the ±20 deg/s stop. Pitch arrives more slowly than the linear 5 deg case. |
| Max-q trim | hold 0 deg | off | 0.5 deg misalignment at 2 s. Gimbal settles near −0.5 deg. Pitch returns toward 0. |

Solver: `ode45`, relative tolerance `1e-6`, max step 5 ms, stop time from `tStop`.

## 7. Tuning in the apps

```matlab
open_pitch_tuner("pid", "max-q")
open_pitch_tuner("designer", "liftoff")
```

The plant handed to the app is `a_cmd` to `theta` (rigid body, actuator, sensor delay). The PID is `pid(Kp, Ki, Kd, Tf)`.

The app's step response is the 1-DOF loop: derivative of the error, no command prefilter. A step in that view kicks the derivative channel. Margins are the ones that match the model. Use the Bode and margin views to tune, and use `run_pitch_tvc` for the step the vehicle actually flies.

Gains in the tuner are acceleration gains. Gimbal in radians is `a_cmd/mu`. `mu` is printed when the tuner opens (0.716, 0.668 and 1.43 at the three conditions).

To keep a hand-tuned result, set any of `V.Kp`, `V.Ki`, `V.Kd`, `V.Tf` on the struct from `vehicle_params` before `design_pitch_controller`. Unset fields stay on the `wn` / `zeta` formulas. Example, in the Command Window, after you have numbers you like:

```matlab
V = vehicle_params();
V.Kp = 1.2;
V.Kd = 1.8;
D = design_pitch_controller(V);
```

`V.attWn`, `V.attZeta`, `V.attKiOverWn3`, `V.attTf`, `V.cmdTau`, `V.actHz`, `V.sensorDelay`, `V.staticMarginAscent` and `V.gimbalRateDeg` are the knobs if you would rather keep the formulas.

## 8. Files

| File | Role |
|---|---|
| `vehicle_params.m` | Companion vehicle numbers, design assumptions, three flight conditions |
| `pitch_plant.m` | Linear plant `a_cmd -> theta` |
| `design_pitch_controller.m` | PID, margins, sweeps, linear plots |
| `apply_pitch_condition.m` | Writes one condition into the base workspace |
| `build_pitch_model.m` | Builds and saves `boosterPitchTVC.slx` |
| `pitch_model_init.m` | Fills the workspace if you press Run on an empty session |
| `run_pitch_tvc.m` | Top-level script |
| `booster_trajectory.m` | Loads the Super Heavy time history |
| `make_booster_trajectory.m` | Regenerates that table with ode45 from matlab-rocket-sim |
| `flight_margin_schedule.m` | Margins along the flight (scheduled and fixed-gain) |
| `apply_flight_schedule.m` | Writes mu(t) and lambda(t) into the Simulink lookups |
| `loop_margins.m` | Exact-delay margins and the Padé stability test |
| `data/superheavy_booster_trajectory.csv` | RK4 trajectory snapshot (mass, thrust, q, mu, lambda, CG, CP) |
| `tools/build_superheavy_trajectory.py` | Script that wrote the snapshot |
| `pitch_dashboard.m` | The single Pitch TVC window (responses, Bode, limits) |
| `open_pitch_tuner.m` | PID Tuner or Control System Designer |

`V.library` also holds the hop, orbital-booster and Starship numbers from `rocket_params.m` (masses, thrust, diameter, length, gimbal limit, `attWn`, `attZeta`, engine counts). The flight-condition table uses Super Heavy. Point a new condition at `V.library.hop` or `V.library.booster` if you want those vehicles on the same plant; their ascent states are not in the snapshot table.

## 9. Assumptions

* Pitch only, in the plane. No yaw, roll, or lateral translation.
* Uniform rod, CG fixed at mid-length, one equivalent nozzle using the whole thrust (`V.gimbalThrustFraction = 1`, matching the companion). A real stack CG sits further aft, which shortens the gimbal arm and can move the static margin. Both are inputs: arm is `length/2`, margin is `staticMarginCal`.
* Frozen conditions, not a gain schedule along a trajectory. Mass, thrust and `q` are constant during each sim.
* No bending, no slosh, no engine-out, no RCS. The companion's RCS covers torque the gimbal cannot; this loop is gimbal only, and the acceleration command is limited to what ±15 deg can produce.
* No Simulink Control Design. The linear model is written with `tf` / `feedback`, not linearised out of the diagram. The diagram uses `sin`, so its small-signal gain at zero gimbal is the linear `mu`.
* Actuator bandwidth, rate limit, delay, `C_Nα` and static margin are assumptions. Thrust, mass, geometry, throttle limits and the 15 deg stop are companion numbers.

## 10. What to check in MATLAB Online

This repo was written and the algebra was checked with NumPy (frequency response, Padé closed-loop poles, and a fixed-step time simulation). MATLAB and Simulink were not executed here. On your side, `run_pitch_tvc` should finish without a red error, and then:

1. **Margins.** The printed table matches the table in section 4 within a few tenths of a dB and a degree. `MATLAB margin()` at max-q is about 21 dB and 60 deg. `DC` of the command response is 1 (within about 0.01). `stable` is true on all three rows.
2. **One window.** A single figure named **Pitch TVC** opens, with tabs Responses, Bode, Limits and disturbances, and Along the flight. No other plot windows open.
3. **5 deg overlay.** On the Responses tab, solid and dashed pitch curves for each condition lie on top of each other up to the gust at 14 s. The checklist RMS is well under 0.05 deg. Peak gimbal is a few degrees, peak rate is under 20 deg/s. The stats table matches the printed margins.
4. **30 deg liftoff.** On Limits and disturbances, gimbal and gimbal rate both sit on their limits for part of the run. Pitch still heads toward 30 deg, more slowly than the 5 deg case. The checklist says both limits were reached.
5. **Misalignment and drift.** On that same tab, gimbal ends near −0.5 deg and the dashed linear pitch is close to the solid one. The uncontrolled max-q trace walks away from 1 deg, and the closed-loop trace holds.
6. **Bode tab.** Three loop gains, circles at the gain crossover, squares at the upward gain-margin frequency, legend showing about 60 deg and 21 dB.
6b. **Along the flight.** The scheduled curve stays near 60 deg and 21 dB. The fixed liftoff-gain curve drops under the red floor in boostback (worst about 16 deg and 2.7 dB near t = 185 s) and is also under 6 dB at brake ignition. The printed worst-case lines should match that picture within a few tenths.
7. **Apps.** `open_pitch_tuner("pid", "max-q")` opens PID Tuner. `open_pitch_tuner("designer", "liftoff")` opens Control System Designer. The plant is SISO, stable in closed loop, phase margin near 60 deg.
8. **Model.** `open_system("boosterPitchTVC")` shows Controller, Actuator and Plant. Scopes exist. Pressing Run with a fresh MATLAB session (no variables) still simulates liftoff, because of `pitch_model_init`.

If the RMS check says REVIEW, the usual causes are a solver warning on the rate limit, or a MATLAB release whose `step` and `To Workspace` timeseries differ slightly. The curves will still show whether the loop is doing the right thing.

## 11. Next steps

Roughly in the order they build on this loop:

| Next | Why |
|---|---|
| Yaw, and roll by differential gimbal | Same plant on the other axes. Roll needs a second actuator path; the companion is planar and has no roll state. |
| Landing-burn guidance around this loop | The companion's `landingAccel` commands pitch (and throttle). Close it outside this attitude loop and keep the 15 deg / 30 deg tilt limits (`maxTiltDeg` is 15 on the hop and 30 on Super Heavy). |
| Schedule `mu` and `lambda` along the flight | Done for this pitch loop (`flight_margin_schedule`, Simulink lookups, Along the flight tab). |
| Bending modes | A couple of flexible modes between the actuator and the gyro, with notches. This is where the 6 Hz servo and the 20 ms delay start to fight the structure. |
| Slosh | A pendulum or a spring-mass in the pitch channel, CG and inertia moving as the tanks drain (`m*L^2/12` will not survive that). |
| Monte Carlo | The companion already disperses thrust, Isp, drag, dry mass and wind. Repeat that on the margins and on the 30 deg case: delay, `mu`, static margin, rate limit. |
| Multi-loop tuning | Control System Tuner (`systune`) can tune this PID together with the guidance loop. That needs **Simulink Control Design**, which this project does not require. The plant and the loop transfers here are the starting point. |
| Hop and the orbital booster | `V.library` has their masses, thrust and 6 deg gimbal stop. They need their own liftoff / max-q / landing snapshot; the dynamics functions are the same. |

## 12. Troubleshooting

| Symptom | What to do |
|---|---|
| `Undefined function 'vehicle_params'` | Current Folder must be this project. `cd` into it. |
| `pitch_plant:NeedsCST` or a missing `tf` / `pid` | Control System Toolbox is not on this license. The design scripts need it. |
| `build_pitch_model:NeedsSimulink` | Simulink is not on this license. `design_pitch_controller(vehicle_params())` still runs. |
| Algebraic loop or rate-limiter warning on the 30 deg run | Expected while the integrators sit on the stops. The 5 deg runs should be quiet. |
| `margin` shows about 21 dB, and you also see a large negative margin in `allmargin` | That negative one is the downward margin from the integrator. The report prints it as "Downward gain margin". The closed loop is stable when `stable` is true. |
| `isstable` / internal delays | Fixed on this branch. Stability is a Padé test. Re-run `run_pitch_tvc`. A warning that names Padé means that one call could not use the exact delay. |
| Tuner step looks worse than `run_pitch_tvc` | The tuner is 1-DOF, with derivative on the error and no prefilter. See section 6. |
