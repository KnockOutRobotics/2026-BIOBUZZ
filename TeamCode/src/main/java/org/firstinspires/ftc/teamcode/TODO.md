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
- [ ] Real autonomous routine in `auto/PedroSwerveAuto`
- [ ] Retire the old `SwerveDrive` / `SwerveTeleOp` once the Pedro TeleOp drives well

## Turret
Homing (1:1 incremental encoder):
- [ ] Pick: carry encoder count across OpModes from a known start, limit switch, or hardstop stall detection
- [ ] Stop zeroing the encoder in the constructor; add `offsetTicks`, `zeroAt(knownAngle)` and a static `homed` flag
- [ ] `homeCommand()`: drive slowly until switch/stall, then `zeroAt()`
- [ ] Skip the PID while not homed; driver button to re-home or zero manually
- [ ] Limit switch only: re-zero whenever the turret passes it

Hardware / config:
- [ ] `TICKS_PER_REV` = encoder counts per rev
- [ ] Motor direction
- [ ] Confirm `MIN_ANGLE` / `MAX_ANGLE` against the cable wrap

Control:
- [ ] Command that waits for `isAtTarget()` (for shooting in auto)
- [ ] Tune P/I/D/F, `I_LIMIT`, `TOLERANCE_DEGREES`, `MAX_POWER`
- [ ] Maybe feedforward to cancel chassis rotation while aiming

Aiming:
- [ ] Goal coordinate per alliance (Pedro field coordinates)
- [ ] Target angle from `PedroDrive.pose()`: angle to goal minus robot heading
- [ ] Aim command that updates the target every loop

OpMode:
- [ ] Register in `PedroSwerveTeleOp` / `PedroSwerveAuto`, add telemetry (target, angle, power)
