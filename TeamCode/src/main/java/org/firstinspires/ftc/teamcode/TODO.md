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
Infinite rotation, 2 CR servos, MelonBotics encoder (absolute analog, 1:1 with the turret), so
no homing or saved angle: `new Turret(hm)` everywhere.

Hardware / config (bench steps in the `Turret Check` OpMode):
- [ ] Robot config: `turretLeader` / `turretFollower` (CR servos), `turretEncoder` (analog input)
- [ ] Measure `ANALOG_MIN_VOLTAGE` / `ANALOG_MAX_VOLTAGE`
- [ ] `ENCODER_INCREASES_CCW`, `ANGLE_OFFSET_DEGREES`
- [ ] `FOLLOWER_REVERSED`, `POWER_REVERSED` (+power turns CCW viewed from above)

Control:
- [x] `waitUntilOnTarget()` command
- [ ] Tune "Turret" live on Dashboard (F for the servo deadband, then KV with P at 0, then P/D),
  copy values back into `SubsystemConstants.Turret`
- [x] Velocity feedforward while aiming (`SubsystemConstants.Turret.KV`)

Aiming:
- [x] `aimCommand(poseSupplier, goal)` via `Targeting`
- [ ] Offset if the turret pivot is not at the robot's center

OpMode:
- [ ] Register in Q2 TeleOp (`new Turret(hm)`), add telemetry (target, angle, power)

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
