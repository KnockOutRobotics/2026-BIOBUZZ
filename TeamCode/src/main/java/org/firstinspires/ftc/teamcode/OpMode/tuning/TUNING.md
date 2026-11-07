# PID tuning procedures

How to tune the three closed loops on the robot: the **drive** (swerve pod steering, then
Pedro's path follower), the **turret** (aim angle) and the **launcher** (flywheel speed).
Each has a tuner OpMode in the `Tuning` group that runs the tests and tells you what to change.

| Loop | Controller | Gains (Dashboard config) | Tuner OpMode | Run first |
|---|---|---|---|---|
| Swerve pod steering | `PIDFController`, error in **radians** | `Drive`: `TURN_P/I/D/F` | Drive Pod PID Tuner | Diffy Pod Check |
| Path follower | Pedro Foresight | `SubsystemConstants.Drive` (final) | AutoTune page | pod tuning |
| Turret aim | `PIDFController` + velocity feedforward, error in **degrees** | `Turret`: `P/I/D/F/KV` | Turret PID Tuner | Turret Check |
| Flywheel | `VelocityController`, **ticks/s**, voltage-compensated | `Launcher`: `KS/KV/KP/KI` | Launcher PID Tuner | config + encoder |

Recommended order: drive, then turret, then launcher. The turret's KV only matters once the
robot can drive and turn, and the launcher needs nothing else working.

---

## Before you start

**Dashboard.** Join the robot's Wi-Fi and open `http://192.168.43.1:8080/dash`.

- *Configuration* panel: gains (`Turret`, `Drive`, `Launcher`) and test settings
  (`Turret Tuner`, `Drive Pod Tuner`, `Launcher Tuner`). Edits apply on the next loop.
- *Graph* panel: plot `target`, `measured` and `error` (plus `power` / `velocity` in the
  open-loop steps). The graph tells you more than any single number.

**Gamepad 1 controls (all tuners).**

| Button | Action |
|---|---|
| dpad up / down | pick a step (only while nothing is running) |
| A | start the picked step |
| B | stop. The mechanism goes limp. |

Results stay on screen after a step finishes or is stopped. Pressing A again clears them.

**Dashboard edits are lost when the Robot Controller app restarts.** Write each tuned value
into `subsystems/SubsystemConstants.java` as soon as you're happy with it.

**Use a charged battery** (12.8 V or more at rest) for the feedforward steps. A sagging battery
makes the same power give less speed.

### Reading a step response

| You see | Change |
|---|---|
| Slow to get there, no overshoot | raise P |
| Overshoots, then settles | raise D (or lower P a little) |
| Oscillates or never settles | lower P; raise D |
| Stops just short of target (final err ≈ constant) | raise F (static friction) first; I only as a last resort |
| Buzzes / chatters at the target | lower F, lower D, or widen the tolerance a little |
| Good on big steps, sluggish on small ones | F too low |

A practical method: set I and D to 0, raise P until it overshoots by about 10–20%, then add D
until the overshoot is gone. Raise P again and repeat until D can no longer keep it clean, then
back both off about 20%. Leave I at 0 unless a steady error remains that F can't fix.

---

## 1. Drive

### 1a. Pod steering: `Drive Pod PID Tuner`

**Prerequisite:** `Diffy Pod Check` passes: encoder direction, `RIGHT/LEFT_POD_ANGLE_OFFSET_DEGREES`,
`TURN_REVERSED` and `DRIVE_REVERSED` are right. If a pod spins away instead of settling, stop
and fix `TURN_REVERSED`. Tuning won't help.

Units: `TURN_*` act on the angle error in **radians**, so `TURN_P = 0.29` is 0.29 power per
radian (about 0.005 per degree). The bumpers pick the pod (right/left) while no step is running.
The other pod stays limp. Both pods share one set of gains.

1. **Static friction → `TURN_F`.** Robot **on the floor** (tile scrub is the real load).
   Run step 1 on the right pod, then the left. It ramps steering power until the pod turns,
   both ways. Set `TURN_F` to the *lower* of the two suggestions. Too much F makes the pods
   buzz at rest.
2. **Step response → `TURN_P`, `TURN_D`.** Still on the floor. Run step 2. The pod flips
   between `STEP_LOW` and `STEP_HIGH` (±40°) every second. Tune with the method above.
   Aim for:
   - rise under ~0.1 s, overshoot under 5%, settled within ~0.2 s;
   - final error inside `TURN_TOLERANCE_RADIANS` (1°).

   Check the other pod with the same gains.
3. **Small steps.** Set `STEP_LOW = 0`, `STEP_HIGH = 5` in `Drive Pod Tuner`. If the pod lags
   on small steps or stops short, raise `TURN_F` slightly. Keep steps under 90°. Past 90° the
   pod flips direction and drives backward instead of steering, by design.
4. **Under drive load.** Robot **on a stand**. Re-run step 2 and hold the right trigger
   (wheel at up to `DRIVE_POWER`). Steering and driving share the same two motors, so at
   high drive power the steering has less authority. The step numbers shouldn't get much
   worse.
5. Copy `TURN_P/I/D/F` (and `TURN_TOLERANCE_RADIANS` if changed) into `SubsystemConstants.Drive`.

### 1b. Path follower: Pedro AutoTune

The follower's translational, heading and braking gains come from Pedro's own tuner. Don't
hand-tune them. With the pods tuned:

1. Measure `POD_X_INCHES` / `POD_Y_INCHES` (y is positive to the robot's **right**).
2. AutoTune page → **Pinpoint Tuner**, and check against the offsets in `pedroPathing/Constants`.
3. AutoTune page → **Foresight Tuner**, and copy all 17 values into `SubsystemConstants.Drive`.
4. AutoTune page → **Tests** (line, curve, hold). If the robot wobbles while holding, go back to
   1a: the pods are usually the cause.

---

## 2. Turret: `Turret PID Tuner`

**Prerequisite:** `Turret Check` passes: `ANALOG_MIN/MAX_VOLTAGE`, `ENCODER_INCREASES_CCW`,
`ANGLE_OFFSET_DEGREES`, `FOLLOWER_REVERSED`, `POWER_REVERSED` are right. The open-loop
steps (1 and 2) need room to spin the turret freely, so check nothing can snag.

Units: P/I/D/F act on the error in **degrees**, KV is power per degree/second of target motion.
The CR servos have a deadband (no motion at small powers), so F matters a lot here.

1. **Deadband → `F`.** Run step 1. It ramps power until the turret moves, both ways. Set
   `Turret.F` to the suggestion. If the turret later buzzes back and forth at the target,
   lower F by ~10%.
2. **Feedforward → `KV`.** Run step 2. It spins the turret at 5 powers each way (about 20 s).
   Set `Turret.KV` to the slope. Expect R² above 0.98. The intercept should be near F; if it's
   far off, the servos aren't linear in that range, so narrow `SWEEP_MIN/MAX_POWER` to the
   speeds aiming actually uses.
3. **Tracking with feedback off → trim `KV`.** Set `P`, `I`, `D` to 0 (keep F). Run step 3:
   the target sweeps ±90° every 3 s and its rate is fed forward. After the second cycle, read
   the **amplitude ratio** and set KV to the "try" value. Repeat until the ratio is within
   0.95–1.05.
4. **Step response → `P`, `D`.** Run step 4 (0° ↔ 90° every 2 s) and tune with the method
   above. Aim for overshoot under 10%, settled under ~0.4 s, final error inside
   `TOLERANCE_DEGREES` (2°). Then set `STEP_LOW = 0`, `STEP_HIGH = 10` and check small steps
   still land (F). Keep steps under 180°, since a 180° target could be reached either way round.
5. **Tracking with feedback on.** Re-run step 3 with your P/D. RMS error should be a few
   degrees at most. For a harder test, shorten `TRACK_PERIOD_SECONDS` to 1.5.
6. Copy `P/I/D/F/KV` (and `TOLERANCE_DEGREES`, `MAX_POWER`, `VELOCITY_FILTER` if changed) into
   `SubsystemConstants.Turret`.
7. **On the robot:** with `aimCommand` running, spin the robot in place. The turret should stay
   on the goal. If it trails while the robot spins, raise KV slightly. If it jitters, lower
   `VELOCITY_FILTER` (smoother target velocity).

---

## 3. Launcher: `Launcher PID Tuner`

**Prerequisite:** the robot config has `flywheel` (motor with its encoder plugged in),
`xWheelsLeader` / `xWheelsFollower` and `transferPivot` / `transferRoller`. Velocities are encoder
ticks/s from `getVelocity()`. Keep hands clear of the flywheel.

The controller multiplies its output by 12 V / battery voltage. The tuner reports power
scaled to 12 V, so its numbers go straight into the constants on any battery.

1. **Feedforward → `KS`, `KV`.** Run step 1. It runs the flywheel at 5 powers from 0.2 to
   1.0, 2 s to settle and 1 s to measure each (about 15 s). Set `Launcher.KS` to the intercept
   and `Launcher.KV` to the slope. Sanity checks: R² above 0.99, and KV ≈ 1 / top speed. If the
   top speed is far below what the motor should reach, check for friction or a rubbing belt.
   If the readout says it moved the wrong way, the motor or encoder direction is reversed.
2. **Spin-up → `KP`.** Run step 2. The target flips between `STEP_LOW` (idle) and `STEP_HIGH`
   (pollen speed) every 3 s.
   - First with `KP = 0`: up steps should finish close to the target on feedforward alone.
     If the final error is more than ~5% of the step, redo step 1.
   - Raise KP until up steps overshoot, then halve it. Watch `flywheel power` on the graph: if
     it chatters at speed, KP is amplifying encoder noise, so lower it.
   - Down steps coast (the flywheel is never driven backward), so their numbers only show how
     long it takes to slow down.
3. **Shot recovery → `KP`, `KI`.** Run step 3. It holds `RECOVERY_VELOCITY`. When the screen
   says **READY**, shoot one game piece (hold right bumper to run the x-wheels at
   `FEED_POWER`, and flip its sign if they feed backward). Each shot shows the dip and the
   recovery time. Raise KP for faster recovery. Add a small KI (and keep `I_LIMIT` small)
   only if it settles below the target. The average recovery time is the fastest you can
   fire repeatedly; shots should wait for `isFlywheelAtSpeed()`.
4. Copy `KS/KV/KP/KI` (and `I_LIMIT`, `TOLERANCE` if changed) into `SubsystemConstants.Launcher`.

---

## Troubleshooting

| Message / symptom | Likely cause |
|---|---|
| "Moved the wrong way for the power sign" | direction setting (`POWER_REVERSED`, `TURN_REVERSED`, motor/encoder direction) |
| "no motion by X power" | not configured / unplugged, mechanism jammed, or `MOVE_THRESHOLD` too high |
| Sweep R² low | battery sagging, binding, or `SWEEP_SETTLE_SECONDS` too short for the speed to settle |
| Step response "settle never" | tolerance tighter than the sensor noise, or oscillating (lower P) |
| Turret fine on the bench but laggy on the robot | KV too low (retune step 3) or `VELOCITY_FILTER` too low |
| Pods buzz while the robot sits still | `TURN_F` too high, or `TURN_TOLERANCE_RADIANS` tighter than encoder noise |
