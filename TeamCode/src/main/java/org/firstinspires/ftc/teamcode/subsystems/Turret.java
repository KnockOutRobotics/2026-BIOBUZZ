package org.firstinspires.ftc.teamcode.subsystems;

import java.util.function.Supplier;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.ivy.Subsystem;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.util.maths.Targeting;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

/**
 * Positional turret: the target angle comes from the robot's pose on the field
 * and the known goal pose, and a PIDF loop drives the azimuth there. Chosen
 * over a velocity loop that chases a turret-mounted Limelight's AprilTag offset
 * because it is less prone to oscillation and lets vision be mounted anywhere
 * (the tag sits almost directly above the robot).
 */
public class Turret implements Subsystem {

  private final PIDFController pidController = new PIDFController(
      TurretTuning.P, TurretTuning.I, TurretTuning.D, TurretTuning.F,
      TurretTuning.I_LIMIT, TurretTuning.TOLERANCE_DEGREES);
  private final Targeting targeting = new Targeting();

  private static final double MAX_TURRET_ANGLE = SubsystemConstants.Turret.MAX_ANGLE;
  private static final double MIN_TURRET_ANGLE = SubsystemConstants.Turret.MIN_ANGLE;
  // feedforward only applies while tracking, i.e. within this many degrees of
  // the target
  private static final double FEEDFORWARD_MAX_ERROR_DEGREES = 90;

  // "timestamp ms,angle degrees", written when an OpMode stops, so TeleOp
  // starts at auto's angle
  private static final String STATE_FILE = "turretState.txt";

  private final DcMotorEx azimuthMotor;
  // raw encoder count at 0 degrees; the encoder itself is never reset
  private double offsetTicks;
  private double targetAngleDegrees;
  // how fast the target is moving, for the velocity feedforward; 0 for a fixed
  // target
  private double targetVelocityDegPerSec = 0;
  // the in-limits copy of the target the PID is driving to; NaN until the first
  // loop
  private double wrappedTargetDegrees = Double.NaN;
  private double power;

  /**
   * Builds the turret at a known start: it must be centered on the robot (0
   * degrees) right now. Use this in auto, where the turret is centered by hand
   * during setup.
   */
  public Turret(HardwareMap hardwareMap) {
    this(hardwareMap, 0);
  }

  private Turret(HardwareMap hardwareMap, double startAngleDegrees) {
    azimuthMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
    azimuthMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    // TODO set the motor direction so +power moves toward +angle
    azimuthMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    setCurrentAngleDegrees(startAngleDegrees);

    // hold where it is until something aims it
    targetAngleDegrees = startAngleDegrees;
  }

  /**
   * Builds the turret at the angle the previous OpMode stopped at, so TeleOp
   * picks up where auto left the turret. Works across an app restart or hub
   * power loss as long as the turret was not moved in between (it brakes).
   * Falls back to centered (0 degrees) if nothing was saved in the last {@code
   * SAVED_ANGLE_MAX_AGE_MS} -- e.g. TeleOp run on its own for practice.
   */
  public static Turret continueFromLastAngle(HardwareMap hardwareMap) {
    return new Turret(hardwareMap, readSavedAngle());
  }

  private static double readSavedAngle() {
    try {
      String[] saved = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile(STATE_FILE))
          .trim().split(",");
      long ageMs = System.currentTimeMillis() - Long.parseLong(saved[0]);
      double angle = Double.parseDouble(saved[1]);
      if (ageMs >= 0 && ageMs <= SubsystemConstants.Turret.SAVED_ANGLE_MAX_AGE_MS && !Double.isNaN(angle)) {
        return angle;
      }
    } catch (RuntimeException e) {
      // no file yet, or garbled: fall back to centered
    }
    return 0;
  }

  /** Saves the current angle for {@link #continueFromLastAngle}. */
  @Override
  public void onStop() {
    ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile(STATE_FILE),
        System.currentTimeMillis() + "," + getCurrentAngleDegrees());
  }

  /** Declares the turret's current position to be {@code angle} degrees. */
  private void setCurrentAngleDegrees(double angle) {
    offsetTicks = azimuthMotor.getCurrentPosition() - angle / 360.0 * SubsystemConstants.Turret.TICKS_PER_REV;
    // the old wrap choice and PID history were measured against the old zero
    wrappedTargetDegrees = Double.NaN;
    pidController.reset();
  }

  /// Set a fixed target angle. Any angle is accepted; periodic() wraps it into
  /// the turret's limits.
  /// @param targetAngle target angle in degrees
  public void setTargetAngleDegrees(double targetAngle) {
    setTargetAngleDegrees(targetAngle, 0);
  }

  /// Set a moving target: the angle now and how fast it is changing, which
  /// feeds the velocity feedforward so the turret keeps up instead of lagging
  /// behind on P alone.
  /// @param targetAngle target angle in degrees
  /// @param targetVelocity target angle's rate of change in degrees per second
  public void setTargetAngleDegrees(double targetAngle, double targetVelocity) {
    targetAngleDegrees = targetAngle;
    targetVelocityDegPerSec = targetVelocity;
  }

  public double getTargetAngleDegrees() {
    return targetAngleDegrees;
  }

  public double getCurrentAngleDegrees() {
    return (azimuthMotor.getCurrentPosition() - offsetTicks) / SubsystemConstants.Turret.TICKS_PER_REV * 360.0;
  }

  public double getPowerSetpoint() {
    return power;
  }

  /**
   * True once the turret is within TurretTuning.TOLERANCE_DEGREES of its
   * target. Updated each periodic().
   */
  public boolean isAtTarget() {
    return pidController.atSetPoint();
  }

  @Override
  public void periodic() {
    // re-read every loop so Dashboard edits take effect live
    pidController.setGains(TurretTuning.P, TurretTuning.I, TurretTuning.D, TurretTuning.F, TurretTuning.I_LIMIT);
    pidController.setTolerance(TurretTuning.TOLERANCE_DEGREES);

    double currentAngle = getCurrentAngleDegrees();
    double previousWrappedTarget = wrappedTargetDegrees;
    wrappedTargetDegrees = wrapAround(targetAngleDegrees, currentAngle);
    // switching to another copy of the target flips the error by up to 360
    // degrees; integral built up chasing the old copy would push the wrong way
    if (Math.abs(wrappedTargetDegrees - previousWrappedTarget) > 180) {
      pidController.reset();
    }
    // Far from the target means unwinding to another copy of it, against the
    // target's motion; feeding that motion forward would push the turret into
    // its cable-wrap limit.
    double feedforward = Math.abs(wrappedTargetDegrees - currentAngle) < FEEDFORWARD_MAX_ERROR_DEGREES
        ? TurretTuning.KV * targetVelocityDegPerSec
        : 0;
    power = pidController.calculate(currentAngle, wrappedTargetDegrees) + feedforward;
    // PID output is unbounded; cap it before it reaches the motor
    power = Range.clip(power, -TurretTuning.MAX_POWER, TurretTuning.MAX_POWER);
    azimuthMotor.setPower(power);
  }

  /**
   * Picks the copy of {@code desiredAngle} (desiredAngle + 360 * k) that lies
   * inside [MIN_TURRET_ANGLE, MAX_TURRET_ANGLE] and is closest to {@code
   * currentAngle}. If no copy fits (limits narrower than 360), the angle is
   * clamped to the limits instead.
   *
   * <p>
   * To stop it flipping between two copies that are about equally far away, it
   * keeps the copy it was already driving to unless another one is closer by
   * WRAP_HYSTERESIS_DEGREES.
   */
  private double wrapAround(double desiredAngle, double currentAngle) {
    double closestToCurrent = Double.NaN;
    double closestToCurrentDist = Double.POSITIVE_INFINITY;
    double closestToPrevious = Double.NaN; // the copy that continues the previous target
    double closestToPreviousDist = Double.POSITIVE_INFINITY;

    // lowest number of full turns that could put a copy inside limits
    int firstTurnCount = (int) Math.ceil((MIN_TURRET_ANGLE - desiredAngle) / 360.0);
    for (int turnCount = firstTurnCount; desiredAngle + 360 * turnCount <= MAX_TURRET_ANGLE; turnCount++) {
      double candidate = desiredAngle + 360 * turnCount;

      double distanceToTravel = Math.abs(candidate - currentAngle);
      if (distanceToTravel < closestToCurrentDist) {
        closestToCurrentDist = distanceToTravel;
        closestToCurrent = candidate;
      }

      // NaN distance (no previous target yet) never wins, so this stays NaN on
      // the first loop
      double distanceFromPrevious = Math.abs(candidate - wrappedTargetDegrees);
      if (distanceFromPrevious < closestToPreviousDist) {
        closestToPreviousDist = distanceFromPrevious;
        closestToPrevious = candidate;
      }
    }

    if (Double.isNaN(closestToCurrent)) {
      return Range.clip(desiredAngle, MIN_TURRET_ANGLE, MAX_TURRET_ANGLE);
    }
    if (!Double.isNaN(closestToPrevious)
        && Math.abs(closestToPrevious - currentAngle) <= closestToCurrentDist
            + SubsystemConstants.Turret.WRAP_HYSTERESIS_DEGREES) {
      return closestToPrevious;
    }
    return closestToCurrent;
  }

  // =========================================================
  // Ivy commands
  //
  // The PID lives in periodic(), which IvyOpMode runs every loop regardless of
  // what is scheduled, so a command only has to move the target. Requiring this
  // subsystem is what stops two commands aiming the azimuth at once.
  // =========================================================

  /** Ivy counterpart of {@link #setTargetAngleDegrees(double)}. */
  public CommandBuilder goToAngle(double targetAngle) {
    return Commands.instant(() -> setTargetAngleDegrees(targetAngle)).requiring(this);
  }

  /**
   * Keeps the turret pointed at {@code goal} (Pedro field coordinates) from
   * wherever the robot is, updating the target every loop until cancelled.
   * Angles are CCW-positive from the robot's front, so +turret power must turn
   * the turret CCW (viewed from above). Assumes the turret pivot is at the
   * robot's center.
   *
   * <p>
   * The target's rate of change (the robot turning or driving past the goal) is
   * measured loop to loop and fed forward, so the turret counter-rotates with
   * the chassis.
   */
  public CommandBuilder aimCommand(Supplier<Pose> robotPose, Pose goal) {
    return Command.build()
        .requiring(this)
        .setStart(() -> lastAimAngle = Double.NaN)
        .setExecute(() -> {
          double angle = targeting.getRobotAngleDegrees(robotPose.get(), goal);
          double dt = aimTimer.seconds();
          aimTimer.reset();
          if (Double.isNaN(lastAimAngle)) {
            aimVelocity = 0;
          } else if (dt > 1e-3) {
            // Targeting returns 0..360, so take the short way across the 0/360
            // seam
            double rawVelocity = normalizeDegrees(angle - lastAimAngle) / dt;
            aimVelocity += TurretTuning.VELOCITY_FILTER * (rawVelocity - aimVelocity);
          }
          lastAimAngle = angle;
          setTargetAngleDegrees(angle, aimVelocity);
        })
        .setDone(() -> false)
        .setEnd(endCondition -> targetVelocityDegPerSec = 0);
  }

  // aimCommand state: last target and its smoothed rate of change
  private final ElapsedTime aimTimer = new ElapsedTime();
  private double lastAimAngle = Double.NaN;
  private double aimVelocity = 0;

  private static double normalizeDegrees(double angle) {
    return ((angle + 180) % 360 + 360) % 360 - 180;
  }

  /** Finishes once the turret is on target; for waiting before a shot. */
  public CommandBuilder waitUntilOnTarget() {
    return Commands.waitUntil(this::isAtTarget);
  }

  /**
   * Re-zeroes with the turret centered on the robot by hand -- the recovery
   * when the carried-over angle is wrong (turret moved between OpModes, or a
   * stale save). Bind to a driver button. Also re-targets 0, so the turret does
   * not swing back to an angle aimed under the old zero.
   */
  public CommandBuilder zeroHereCommand() {
    return Commands.instant(() -> {
      setCurrentAngleDegrees(0);
      targetAngleDegrees = 0;
    }).requiring(this);
  }

  // TODO (also tracked in TODO.md)
  // [ ] TICKS_PER_REV = encoder counts per turret rev
  // [ ] motor direction (constructor): +power turns CCW viewed from above
  // [ ] confirm MIN_ANGLE / MAX_ANGLE against the cable wrap
  // [ ] tune TurretTuning on Dashboard (KV first, with P at 0), then copy the
  //   values back
}
