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

1. Prints the gain-scheduled cascade, margins, step-response numbers, and three sweeps (sensor delay, actuator bandwidth, static margin).
2. Builds `boosterPitchTVC.slx` in the current folder and simulates it.
3. Opens **one** dashboard window, `Pitch TVC`, and prints a PASS/REVIEW checklist.

The dashboard is a browser figure. The tab group sits in a grid that fills the window, and each tab is a tiled layout of at most four plots. Axes toolbars are hidden so they do not cover the curves.

| Tab | What is on it |
|---|---|
| **Responses** | 2×2: 5 deg pitch (solid Simulink, dashed linear), gust, gimbal angle, gimbal rate |
| **Stats** | The numbers table, including the 30 deg before/after rows, time on the stops, and the integrator |
| **Bode** | Loop gain and phase for liftoff, max-q and landing |
| **30 deg step** | 2×2: governor off (before) and on (after) for pitch, integrator, gimbal angle, and gimbal rate |
| **Disturbances** | Thrust-misalignment pitch and gimbal, and the max-q open-loop drift |
| **Along the flight** | Phase and gain margins versus time for the 1/mu schedule and for a frozen liftoff gain. Black dots are the three design points. Red x marks samples under 6 dB or 30 deg |

| Command | What you get |
|---|---|
| `run_pitch_tvc` | Full design, model build, six simulations, one dashboard, checklist |
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

`mu` is the small-signal pitch acceleration per radian of gimbal. The schedule is `delta_cmd = a_cmd / mu`. One cascade, in acceleration units, is used at every condition. Heavier vehicles and lower throttle ask for more gimbal for the same acceleration. The 30 deg liftoff step is the large-maneuver case: without the command governor the gimbal reaches its stops, and with the governor it does not.

## 3. Plant

Perturbations about a trim with zero angle of attack and zero gimbal. Pitch is measured from the trim attitude (the flight-path angle), so the flight-path angle does not appear as a state.

```
theta'' = lambda*theta + mu*sin(delta + delta_mis) + d_gust
delta   = Actuator(delta_cmd)
```

`lambda > 0` is statically unstable: the centre of pressure is ahead of the centre of mass, and a positive angle of attack produces a positive pitching moment. With the sign convention here, a positive crosswind `w` reduces angle of attack (`alpha = theta - w/V`). The gust enters as `d_gust = -(lambda/V)*w`. At zero airspeed the gain is zero.

**Aerodynamics (a design assumption, not in the companion).** Slender-body normal-force slope `C_Nα = 2` per radian on the frontal area. Static margin on ascent is **−0.5 caliber** (CP is 4.5 m ahead of the CG on the 9 m body). Landing margin is **0**: the companion has no body aero moment, and grid-fin stability is not in this model yet. At max-q that −0.5 caliber is a real instability, and it is a slow one. The time to double is about 14 s. TVC authority (`mu ≈ 0.67 /s²`) is about 270 times the aero stiffness (`lambda ≈ 0.0025 /s²`), so a 15 m/s gust (about 3.3 deg of angle of attack at 259 m/s) trims out with about **0.01 deg of gimbal**. The dedicated gust figure uses its own scale so that small motion is visible. Push `V.staticMarginAscent` more negative and re-run; the printed sweep goes out to −50 calibers, where the unstable pole is about 0.5 rad/s and the phase margin stays near 64 deg. Aero is not what spends the margin on this vehicle. Actuator bandwidth and delay are.

**Actuator (a design assumption).** Second order, 6 Hz, damping 0.7, then hard limits: **±15 deg** (the companion's `gimbalMaxDeg` for Super Heavy) and **±20 deg/s**. The 6 Hz and 20 deg/s are sizing numbers for a large-engine servo, not identified hardware. In the linear analysis the limits are off. In Simulink they are integrator saturation on the rate state and the angle state, so the linear servo and the nonlinear servo are the same transfer function while the states are inside the stops.

**Sensor delay (a design assumption).** 20 ms of pure delay on pitch and pitch rate. The companion's gimbal and sensors are instantaneous.

**Command prefilter.** A 0.5 s first-order filter on the attitude command. It sits outside the feedback loop, so it does not change margins. A raw 5 deg step would ask the 6 Hz servo for well over 100 deg/s of gimbal rate. Guidance commands are smooth; with the prefilter a 5 deg step stays inside the 20 deg/s stop. The command governor (section 4) then limits how far and how fast that command can move when a large step would otherwise drive the gimbal into its stops.

## 4. Controller

The companion law is

```
tau_cmd = I * (wn^2 * (theta_cmd - theta) - 2*zeta*wn * theta_dot)
delta   = asin(tau_cmd / (F*arm))
```

with `wn = 1 rad/s` and `zeta = 0.8`. For small gimbal that is exactly `a_cmd = wn^2*e - 2*zeta*wn*q`, then `delta = a_cmd/mu`. This project uses the same acceleration-command idea as a cascade, retuned so the scheduled margins stay above 6 dB and 30 deg:

| Term | Value | Role |
|---|---|---|
| `Kq` | 2 /s | inner rate loop. Equivalent `Kd` |
| `Kth` | `wn^2/Kq = 0.5` /s | outer attitude loop. `Kp = Kq*Kth = 1` /s² |
| `KiTh` | `0.01` /s² | attitude integrator. `Ki = Kq*KiTh = 0.02` /s³ |
| `Tf` | 0.05 s | filter on the measured rate |
| `Kaw` | 1 /s | back-calculation on the rate-command clip and on `a_cmd` |

Equivalent damping is `zeta = 1`, a bit heavier than the companion's 0.8. `V.attZeta` is still the companion number; it does not set `Kq`. The loop gain is the PID `Kp + Ki/s + Kd*s/(Tf*s+1)` with `Kd = Kq`.

`Ki` is deliberately weak. The 5% settling time stays near 5 s. The last fraction of a degree takes longer (2% settling near 40 s) because the integrator is slow. Raising `V.attKiOverWn3` toward 0.1 trims faster and adds overshoot; PID Tuner is the place to make that trade.

**Command governor.** After the 0.5 s prefilter, the pitch command may lead the measured pitch by at most `e_lim`, and the shaped command may move at at most `q_lim`. Both limits scale with the gimbal authority still left:

```
delta_avail = max(delta_max - |trim|, 0.25*delta_max)
a_avail     = mu * sin(delta_avail)
e_lim       = max(0.85 * a_avail / Kp,  1.2 * 5 deg)
q_lim       = max(0.85 * a_avail / Kq,  1.25 * (5 deg) / 0.5 s)
```

`trim` is the measured gimbal, filtered at 3 s. At liftoff the authority lead is about 9 deg, so that is `e_lim`. The authority rate is only about 4.5 deg/s, and the floor of 12.5 deg/s is what the governor uses, so the 5 deg design step does not bind. A 30 deg step does: the command cannot run away from the pitch the booster has already reached. `shapeOn = 0` bypasses the governor and is the "before" case. `shapeOn = 1` is the normal law.

Anti-windup is the backstop, not the strategy. Back-calculation runs on the rate-command clip (`Kaw`) and on the acceleration clip (`Kaw/Kq`). On the 30 deg step the integrator stays near zero either way.

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
| Liftoff | 64.0 deg | 2.09 rad/s | 19.4 dB | 15.1 rad/s | 0.53 s | yes (Padé, not run here) |
| Max-q | 64.0 deg | 2.09 rad/s | 19.4 dB | 15.1 rad/s | 0.53 s | yes (Padé, not run here) |
| Landing | 64.0 deg | 2.09 rad/s | 19.4 dB | 15.1 rad/s | 0.53 s | yes (Padé, not run here) |

`margin` in MATLAB reports one gain margin, the one closest to 0 dB. That is the **upward** margin, about 19.4 dB. There is a second, downward margin near 0.10 rad/s (about 38 to 40 dB) from the integrator: the loop gain can fall that far before the low-frequency branch loses the Nyquist encirclement. `allmargin` lists both. The printed report labels them.

Command response to a 5 deg step, prefilter on, limits off (fixed-step check of the same nonlinear-free equations; `stepinfo` in the script is the number to keep):

| | Liftoff | Max-q | Landing |
|---|---|---|---|
| 10–90% rise | 3.3 s | 3.3 s | 3.3 s |
| Overshoot | 3.5% | 3.7% | 3.5% |
| 5% settling | 4.9 s | 4.8 s | 4.9 s |
| Peak gimbal (governor on) | 2.8 deg | 3.0 deg | 1.4 deg |
| Peak gimbal rate | 10.7 deg/s | 11.5 deg/s | 5.4 deg/s |

All three 5 deg cases stay inside ±15 deg and ±20 deg/s. The governor does not bind on that step (the RMS against the ungoverned linear command, up to the gust, is about 0.034 deg). The checklist flags that RMS above 0.05 deg.

A 30 deg liftoff step, same plant, stop time 45 s (NumPy RK4, 0.5 ms). "Before" is `shapeOn = 0`. "After" is the governor. Time on a stop counts samples within 0.05 deg or 0.05 deg/s of the limit.

| | Before | After |
|---|---|---|
| Final pitch | 29.5 deg | 30.6 deg |
| Peak gimbal | 15.0 deg | 6.7 deg |
| Peak gimbal rate | 20 deg/s | 18.1 deg/s |
| Time on the angle stop | 0.22 s | 0 s |
| Time on the rate stop | 1.71 s | 0 s |
| Integrator at 45 s | −0.004 rad/s | +0.005 rad/s |

A 0.5 deg thrust misalignment (a nozzle bias in the same direction as positive gimbal) is held by commanding about −0.5 deg of gimbal. Pitch returns to the command. That is the disturbance the integrator is there for. The wind gust is in the model as well; at these inertias it barely moves the gimbal.

### What actually spends the margin

At max-q, same gains, from the same NumPy check. The script prints the MATLAB version of each table.

Sensor delay:

| Delay | 0 | 10 ms | 20 ms (design) | 50 ms | 100 ms |
|---|---|---|---|---|---|
| Phase margin | 66 deg | 65 deg | 64 deg | 60 deg | 54 deg |
| Upward gain margin | 22.7 dB | 20.9 dB | 19.4 dB | 16.2 dB | 12.7 dB |

Actuator natural frequency (damping 0.7, delay 20 ms):

| Bandwidth | 3 Hz | 4 Hz | 6 Hz (design) | 8 Hz | 10 Hz |
|---|---|---|---|---|---|
| Phase margin | 60 deg | 62 deg | 64 deg | 65 deg | 66 deg |
| Upward gain margin | 15.5 dB | 17.2 dB | 19.4 dB | 20.8 dB | 21.9 dB |

Static margin at max-q from 0 to −50 calibers leaves the phase margin at about 64 deg. The open-loop drift figure is still worth looking at: from 1 deg of initial pitch, the uncontrolled max-q pole walks off over a minute, and the loop holds it.

## 5. Schedule along the flight

`data/superheavy_booster_trajectory.csv` is a 1 Hz RK4 history of the companion Super Heavy mission: stack ascent to hot staging, then flip, boostback, coast, brake and landing. It carries mass, throttle, engine count, thrust, dynamic pressure, Mach, speed, pitch inertia, the mid-body centre of mass, and the centre of pressure from the static-margin assumption. Flip and coast are in the table with `tvc_active = 0` (engines off). There is no entry burn (`engEntry = 0`).

`make_booster_trajectory` rewrites that file with the companion's `ode45` once `matlab-rocket-sim` is on the path. Until then the RK4 snapshot is what `run_pitch_tvc` loads.

The normalised cascade does not change. The Simulink lookups make `mu`, `lambda` and `1/mu` functions of flight time, so the gimbal gain tracks control effectiveness. With that schedule the linear margins stay near **64 deg and 19.4 dB** wherever the engines are lit (they only move with `lambda`, which stays near zero on this aero model). A NumPy check puts the scheduled worst case at **t = 0 s, 64.0 deg and 19.4 dB, 0 samples under the floor**. A liftoff gain frozen in gimbal units does not. The same check puts the worst fixed-gain point at **t = 185 s, boostback: about 5.4 deg and 0.8 dB** (`mu` about 6.1 /s², 8.5 times liftoff), with **12 samples** under 6 dB or 30 deg. The Along the flight tab draws both curves, marks liftoff, max-q and the 1 km landing point, and puts a red x on fixed-gain samples under the floor. `run_pitch_tvc` prints the same worst cases from Control System Toolbox. The heavier rate gain (`Kd` 2 instead of 1.6) is why the fixed-gain worst case moved down from the previous 16 deg / 2.7 dB. The schedule is what holds the margin.

## 6. Simulink model

`build_pitch_model.m` creates `boosterPitchTVC` with `new_system`, `add_block`, `add_line` and `set_param`, using only the standard Simulink library, then saves `boosterPitchTVC.slx`.

```
theta_cmd (Step)
    -> Controller  -> delta_cmd -> Actuator -> delta -> Plant -> theta, q
           ^                                                   |
           +------------------------ theta, q -----------------+
```

* **Controller** (light blue): 0.5 s command prefilter, 20 ms delay on pitch and rate, command governor (lead limit and rate limit from `mu`, gimbal trim, and the angle stop), cascade `Kth` / `Kq`, saturation of `a_cmd` at `±mu*deltaMax` with back-calculation, and `1/mu`. `mu`, `1/mu` and the acceleration limit are 1-D lookup tables versus time. A fourth input is the measured gimbal, for the trim filter. A second output is the attitude integrator, logged as `integ`. `shapeOn` selects the governor.
* **Actuator** (orange): `wn^2` and `2*zeta*wn` around two integrators. The rate integrator saturates at `±gimbalRate`. The angle integrator saturates at `±deltaMax`.
* **Plant** (green): `mu(t)*sin(delta + misalign) + lambda(t)*theta + gustAccPerWind*wind`, then two integrators. `mu` and `lambda` are the same lookup breakpoints. Misalignment and wind are Step blocks inside the plant.
* Scopes on attitude, gimbal and gimbal rate. To Workspace logs `theta`, `q`, `delta`, `deltaCmd`, `gimbalRateMeas`, `thetaCmd` and `integ` as timeseries.

Every number the diagram uses is a base-workspace variable (`Kp`, `mu`, `lambdaAero`, `tStop`, ...). `apply_pitch_condition` writes them. If you press Run in the model with an empty workspace, `pitch_model_init` loads the liftoff case.

The nominal script runs:

| Run | Command | Gust | What to look at |
|---|---|---|---|
| Liftoff, max-q, landing | 5 deg step at 1 s | 15 m/s at 14 s | Three curves on top of each other. Gimbal a few degrees. Rate under 20 deg/s. |
| Liftoff, 30 deg, governor off | 30 deg step | off | The before case. Gimbal rate sits on ±20 deg/s for about 1.7 s and the angle reaches the ±15 deg stop. Pitch ends near 29.5 deg. |
| Liftoff, 30 deg, governor on | 30 deg step | off | The after case. Peak gimbal about 6.7 deg, peak rate about 18 deg/s, neither stop touched. Pitch ends near 30.6 deg. |
| Max-q trim | hold 0 deg | off | 0.5 deg misalignment at 2 s. Gimbal settles near −0.5 deg. Pitch returns toward 0. |

Solver: `ode45`, relative tolerance `1e-6`, max step 5 ms, stop time from `tStop`.

## 7. Tuning in the apps

```matlab
open_pitch_tuner("pid", "max-q")
open_pitch_tuner("designer", "liftoff")
```

The plant handed to the app is `a_cmd` to `theta` (rigid body, actuator, sensor delay). The PID is the cascade equivalent `pid(Kp, Ki, Kd, Tf)` with `Kd = Kq`.

The app's step response is the 1-DOF loop: derivative of the error, no command prefilter. A step in that view kicks the derivative channel. Margins are the ones that match the model. Use the Bode and margin views to tune, and use `run_pitch_tvc` for the step the vehicle actually flies.

Gains in the tuner are acceleration gains. Gimbal in radians is `a_cmd/mu`. `mu` is printed when the tuner opens (0.716, 0.668 and 1.43 at the three conditions).

To keep a hand-tuned result, set `V.Kq`, `V.Kth` or `V.KiTh` before `design_pitch_controller`. A pasted `V.Kp`, `V.Ki` or `V.Kd` still wins, and the cascade is rebuilt from that PID so the diagram matches the margins. Example:

```matlab
V = vehicle_params();
V.Kq = 2.2;
D = design_pitch_controller(V);
```

`V.attWn`, `V.attKiOverWn3`, `V.attTf`, `V.cmdTau`, `V.etaAuth`, `V.shapeKs`, `V.actHz`, `V.sensorDelay`, `V.staticMarginAscent` and `V.gimbalRateDeg` are the other knobs. `V.attZeta` is the companion's 0.8 and does not set `Kq`.

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
| `pitch_dashboard.m` | The single Pitch TVC window (responses, stats, Bode, 30 deg step, disturbances, flight) |
| `open_pitch_tuner.m` | PID Tuner or Control System Designer |

`V.library` also holds the hop, orbital-booster and Starship numbers from `rocket_params.m` (masses, thrust, diameter, length, gimbal limit, `attWn`, `attZeta`, engine counts). The flight-condition table uses Super Heavy. Point a new condition at `V.library.hop` or `V.library.booster` if you want those vehicles on the same plant; their ascent states are not in the snapshot table.

## 9. Assumptions

* Pitch only, in the plane. No yaw, roll, or lateral translation.
* Uniform rod, CG fixed at mid-length, one equivalent nozzle using the whole thrust (`V.gimbalThrustFraction = 1`, matching the companion). A real stack CG sits further aft, which shortens the gimbal arm and can move the static margin. Both are inputs: arm is `length/2`, margin is `staticMarginCal`.
* The three design sims hold mass, thrust and `q` constant. The margin sweep and the Simulink lookups follow `mu(t)` and `lambda(t)` from the trajectory table. The 5 deg and 30 deg runs use the frozen liftoff, max-q or landing row.
* No bending, no slosh, no engine-out, no RCS. The companion's RCS covers torque the gimbal cannot; this loop is gimbal only, and the acceleration command is limited to what ±15 deg can produce.
* No Simulink Control Design. The linear model is written with `tf` / `feedback`, not linearised out of the diagram. The diagram uses `sin`, so its small-signal gain at zero gimbal is the linear `mu`.
* Actuator bandwidth, rate limit, delay, `C_Nα` and static margin are assumptions. Thrust, mass, geometry, throttle limits and the 15 deg stop are companion numbers.

## 10. What to check in MATLAB Online

This repo was written and the algebra was checked with NumPy (frequency response, Padé closed-loop poles, and a fixed-step time simulation). MATLAB and Simulink were not executed here. On your side, `run_pitch_tvc` should finish without a red error, and then:

1. **Margins.** The printed table matches the table in section 4 within a few tenths of a dB and a degree. `MATLAB margin()` at max-q is about 19.4 dB and 64 deg. `DC` of the command response is 1 (within about 0.01). `stable` is true on all three rows. The command-governor line prints a rate floor of 12.5 deg/s and a lead floor of 6 deg.
2. **One window.** A single figure named **Pitch TVC** fills the browser figure. Tabs: Responses, Stats, Bode, 30 deg step, Disturbances, Along the flight. Resizing the window stretches the plots. No other plot windows open.
3. **5 deg overlay.** On the Responses tab, solid and dashed pitch curves for each condition lie on top of each other up to the gust at 14 s. The checklist RMS is about 0.03 deg, under 0.05. Peak gimbal is a few degrees, peak rate is under 20 deg/s. The Stats tab matches the printed margins, and the two 30 deg rows are at the bottom.
4. **30 deg liftoff, before and after.** On the 30 deg step tab, orange is the governor off and blue is the governor on. Before: gimbal rate on the ±20 deg/s stop for on the order of 1.7 s, angle on or at the ±15 deg stop, pitch ending near 29.5 deg. After: peak gimbal near 7 deg, peak rate near 18 deg/s, both stop-times at 0, pitch ending near 30.6 deg. The integrator traces stay near zero. Stop times are in the Stats tab. The checklist PASSes the before row when the rate-stop time is at least 0.2 s, and the after row when both stop-times are under 0.05 s and the peaks stay off the stops.
5. **Misalignment and drift.** On that same tab, gimbal ends near −0.5 deg and the dashed linear pitch is close to the solid one. The uncontrolled max-q trace walks away from 1 deg, and the closed-loop trace holds.
6. **Bode tab.** Three loop gains, circles at the gain crossover, squares at the upward gain-margin frequency, legend showing about 64 deg and 19 dB.
6b. **Along the flight.** The scheduled curve stays near 64 deg and 19.4 dB, with 0 samples under the floor. The fixed liftoff-gain curve drops under the red floor in boostback (worst about 5 deg and 0.8 dB near t = 185 s). The printed worst-case lines should match that picture within a few tenths. Expect more fixed-gain samples under the floor than the previous 11, because `Kd` is 2.
7. **Apps.** `open_pitch_tuner("pid", "max-q")` opens PID Tuner. `open_pitch_tuner("designer", "liftoff")` opens Control System Designer. The plant is SISO, stable in closed loop, phase margin near 64 deg.
8. **Model.** `open_system("boosterPitchTVC")` shows Controller, Actuator and Plant. Scopes exist. Pressing Run with a fresh MATLAB session (no variables) still simulates liftoff, because of `pitch_model_init`.

If the RMS check says REVIEW, the usual causes are a solver warning on the rate limit, or a MATLAB release whose `step` and `To Workspace` timeseries differ slightly. The curves will still show whether the loop is doing the right thing.

## 11. Next steps

Roughly in the order they build on this loop:

| Next | Why |
|---|---|
| Command governor on large steps | Done for this pitch loop. The 30 deg command is rate-limited and lead-limited from the gimbal authority that is left. Anti-windup remains the backstop. Load relief, slosh, bending and grid fins are not in this step. |
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
| `Invalid setting ... BackgroundColor` on Plant | `lightGreen` is not a Simulink colour. The plant now uses an `[r,g,b]` string, and a rejected colour or block position only warns. |
| `Skipped LimitDataPoints` on the To Workspace logs | The block parameter is `MaxDataPoints`, set to `inf`. A missing optional name is no longer warned about. |
| `uitable` ... `Values within a cell array must be numeric, logical, or char` | The stats table was handing `uitable` string scalars. Every cell is now a number or a char row. |
| Algebraic loop or rate-limiter warning on the 30 deg before run | Possible while the gimbal integrators sit on the stops. The governor-on run and the 5 deg runs should be quiet. |
| `margin` shows about 19 dB, and you also see a large negative margin in `allmargin` | That negative one is the downward margin from the integrator (about 38 dB at max-q). The report prints it as "Downward gain margin". The closed loop is stable when `stable` is true. |
| `isstable` / internal delays | Fixed on this branch. Stability is a Padé test. Re-run `run_pitch_tvc`. A warning that names Padé means that one call could not use the exact delay. |
| Tuner step looks worse than `run_pitch_tvc` | The tuner is 1-DOF, with derivative on the error and no prefilter. See section 7. |
| Build stops on `MinMax`, `Switch` or `Saturation Dynamic` | Those blocks are the governor. A rejected `Criteria`, `Function` or port connection is a real diagram error, not a colour. Cosmetic `set_param` failures only warn. |

## 13. Descent and grid fins

`run_pitch_tvc` now continues past staging through the booster return and adds one **Descent** tab to the same Pitch TVC window. The ascent checklist is unchanged. To run only this part:

```matlab
run_booster_descent
```

What it covers, using the committed trajectory (the RK4 twin of `simulateBooster` with the superheavy preset):

| Phase | Engines | Pitch effector |
|---|---|---|
| Flip | off | RCS, the companion `tauRCS` (30e6 N·m). Fins have no air up here |
| Boostback | 13 | TVC only |
| Coast | off | Grid fins, but dynamic pressure is negligible above about 40 km |
| Entry | off | Grid fins. This is the unpowered descent below `finAlt` (40 km). There is no entry burn (`engEntry = 0`) |
| Brake | 13 | TVC and grid fins blended by the acceleration each one can produce |
| Landing | 3 centre | Same blend, fading to TVC as `q` falls. Catch is at 65 m |

The cascade, the command governor and the back-calculation anti-windup are the ascent law. A mode weight splits the acceleration command. The weight and the `mu` in the divider use the same 0.5 s filter, and each surface command is then rate-limited, so the command and the integrator do not step when the engines light, cut off, or drop from 13 to 3.

Grid-fin numbers are design assumptions on top of the companion's `finCLA = 25 m^2`. Half of that force is the pitch pair, the stop is ±20 deg, the rate stop is ±30 deg/s, and the servo is 2 Hz. The alpha stiffness is a quarter of the control derivative (restoring, because engines-first the fins are the tail). The pitch command stands up from 400 m above the arms and is upright by 150 m: the companion's 20 m tilt fade is faster than this 1 rad/s loop. The 30 deg cap is the companion's `maxTiltDeg`.

A NumPy twin of the fixed-step loop (the Descent tab) gave, approximately: catch error **+1.3 deg**, peak fin **20 deg** with about **8.3 s** on the stop, peak gimbal under **2 deg** and never on the stop. Phase anchors from that twin, same plant and the 20 ms delay:

| Phase | PM | GM up |
|---|---|---|
| Flip (RCS) | 62.5 deg | 20.6 dB |
| Boostback (TVC) | 64.0 deg | 19.4 dB |
| Entry at peak q | 54.1 deg | 12.9 dB |
| Brake, blended | 60.9 deg | 18.9 dB |
| Landing just after 13 to 3 | 62.7 deg | 19.5 dB |
| Catch | 64.0 deg | 19.4 dB |

`run_booster_descent` reprints these with `loop_margins` (Padé stability on the anchors). Coast above the fins is not claimed: there is no pitch authority. Handoffs in the NumPy twin moved the commands by well under 0.1 deg in any 20 ms sample inside ±1 s of the switch, and the integrator step stayed near 1e-5 rad/s.

New files: `grid_fin_model.m`, `grid_fin_effectiveness.m`, `make_booster_return_trajectory.m`, `descent_targets.m`, `descent_openloop.m`, `descent_controller.m`, `build_descent_model.m`, `descent_model_init.m`, `apply_descent_workspace.m`, `run_booster_descent.m`, `descent_dashboard.m`. `run_pitch_tvc.m` only calls them. The Simulink model is `boosterDescentPitch.slx` (gitignored). If the diagram fails, the warning says so and the Descent tab still shows the fixed-step loop.
