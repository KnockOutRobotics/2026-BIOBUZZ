# TODO

## Drivetrain (differential swerve + Pedro)
- [ ] `Diffy Pod Check` on a stand: encoder direction, pod angle offsets, `TURN_REVERSED`, `DRIVE_REVERSED`
- [ ] Tune pod steering PID (`TURN_P/I/D/F` in `SubsystemConstants.Drive`)
- [ ] Measure `POD_X_INCHES` / `POD_Y_INCHES` (y is positive to the robot's **right**)
- [ ] AutoTune: Pinpoint Tuner, then check against the offsets in `pedroPathing/Constants`
- [ ] AutoTune: Foresight Tuner, copy the 17 values into `SubsystemConstants.Drive`
- [ ] AutoTune: Tests (line, curve, hold)
- [ ] Drive `Pedro Swerve TeleOp`; decide on heading lock (`ManualDrive.headingLock` / `driveOrHold`)
- [ ] Field-centric "forward" per alliance at TeleOp start
- [ ] Real autonomous routine in `OpMode/teleop/PedroSwerveAuto`
- [x] Q2 TeleOp drives through `PedroDrive`; old `SwerveDrive` / `SwerveTeleOp` removed

## Field pose
- [x] Pose from `PedroDrive.pose()` (pass `drive::pose` to `aimCommand`)
- [ ] Alliance selection in init loop (picks the goal pose)
- [ ] Check `Targeting` goal coordinates against the field drawings

## Turret
Homing (known start: turret centered on the robot at auto setup; TeleOp continues from the saved angle):
- [x] `new Turret(hm)` in auto, `Turret.continueFromLastAngle(hm)` in TeleOp
- [ ] Bind `zeroHereCommand()` to a driver button (recovery if the carried-over angle is wrong)

Hardware / config:
- [ ] `TICKS_PER_REV` = encoder counts per turret rev (placeholder until the encoder is set up)
- [ ] Motor direction: +power turns CCW viewed from above
- [ ] Confirm `MIN_ANGLE` / `MAX_ANGLE` against the cable wrap

Control:
- [x] `waitUntilOnTarget()` command
- [ ] Tune `TurretTuning` live on Dashboard, copy values back
- [x] Velocity feedforward while aiming (`TurretTuning.KV`)

Aiming:
- [x] `aimCommand(poseSupplier, goal)` via `Targeting`
- [ ] Offset if the turret pivot is not at the robot's center

OpMode:
- [ ] Register in Q2 TeleOp (`continueFromLastAngle`), add telemetry (target, angle, power)

## Robot / controller behavior (define first)
Drafts in `util/robotBehavior.txt` and `util/controllerMap.txt`. Finish these before building
the launcher or filling in Q2 TeleOp.
- [ ] Launcher behavior
- [ ] Turret auto/manual behavior (hive selection, "lock turret")
- [ ] Intake/outtake behavior
- [ ] Final controller map (driver + operator), pre-match alliance selection

## Launcher
Not started.

## Intake
Stays on fixed powers for now (`NECTAR_POWER` / `POLLEN_POWER`).

## Q2 TeleOp
Kept mostly blank (drive + intake) until behavior is defined.
- [ ] Register turret and launcher, bindings per the controller map

## Cleanup
- [ ] Remove `samples/` when no longer needed
