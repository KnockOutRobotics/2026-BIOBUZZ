package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ivy.Subsystem;
import org.firstinspires.ftc.teamcode.util.PIDFController;

/**
 * Positional turret: the target angle comes from the robot's pose on the field and the known
 * goal pose, and a PIDF loop drives the azimuth there. Chosen over a velocity loop that chases a
 * turret-mounted Limelight's AprilTag offset because it is less prone to oscillation and lets
 * vision be mounted anywhere (the tag sits almost directly above the robot).
 */
public class Turret implements Subsystem {

  private final PIDFController pidController = new PIDFController(
      SubsystemConstants.Turret.P,
      SubsystemConstants.Turret.I,
      SubsystemConstants.Turret.D,
      SubsystemConstants.Turret.F,
      SubsystemConstants.Turret.I_LIMIT,
      SubsystemConstants.Turret.TOLERANCE_DEGREES);

  private static final double MAX_TURRET_ANGLE = SubsystemConstants.Turret.MAX_ANGLE;
  private static final double MIN_TURRET_ANGLE = SubsystemConstants.Turret.MIN_ANGLE;

  private final DcMotorEx azimuthMotor;
  private double targetAngleDegrees;
  // the in-limits copy of the target the PID is driving to; NaN until the first loop
  private double wrappedTargetDegrees = Double.NaN;
  private double power;

  public Turret(HardwareMap hardwareMap) {
    azimuthMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
    azimuthMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    // TODO set the motor direction so +power moves toward +angle
    // TODO stop zeroing here; zero only when homed (see the TODO list at the bottom)
    resetEncoder();

    targetAngleDegrees = 0;
  }

  private void resetEncoder() {
    azimuthMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    azimuthMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  /// Set the target angle. Any angle is accepted; periodic() wraps it into the turret's limits.
  /// @param targetAngle target angle in degrees
  public void setTargetAngleDegrees(double targetAngle) {
    targetAngleDegrees = targetAngle;
  }

  private double normalize(double angle) {
    return ((angle + 180) % 360 + 360) % 360 - 180;
  }

  public double getTargetAngleDegrees() {
    return targetAngleDegrees;
  }

  public double getCurrentAngleDegrees() {
    return azimuthMotor.getCurrentPosition() / SubsystemConstants.Turret.TICKS_PER_REV * 360.0;
  }

  public double getPowerSetpoint() {
    return power;
  }

  /** True once the turret is within TOLERANCE_DEGREES of its target. Updated each periodic(). */
  public boolean isAtTarget() {
    return pidController.atSetPoint();
  }

  @Override
  public void periodic() {
    double currentAngle = getCurrentAngleDegrees();
    wrappedTargetDegrees = wrapAround(targetAngleDegrees, currentAngle);
    power = pidController.calculate(currentAngle, wrappedTargetDegrees);
    // PID output is unbounded; cap it before it reaches the motor
    power = Range.clip(power, -SubsystemConstants.Turret.MAX_POWER, SubsystemConstants.Turret.MAX_POWER);
    azimuthMotor.setPower(power);
  }

  /**
   * Picks the copy of {@code desiredAngle} (desiredAngle + 360 * k) that lies inside
   * [MIN_TURRET_ANGLE, MAX_TURRET_ANGLE] and is closest to {@code currentAngle}. If no copy fits
   * (limits narrower than 360), the angle is clamped to the limits instead.
   *
   * <p>To stop it flipping between two copies that are about equally far away, it keeps the copy
   * it was already driving to unless another one is closer by WRAP_HYSTERESIS_DEGREES.
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

      // NaN distance (no previous target yet) never wins, so this stays NaN on the first loop
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
        && Math.abs(closestToPrevious - currentAngle)
            <= closestToCurrentDist + SubsystemConstants.Turret.WRAP_HYSTERESIS_DEGREES) {
      return closestToPrevious;
    }
    return closestToCurrent;
  }

  // =========================================================
  // Ivy commands
  //
  // The PID lives in periodic(), which IvyOpMode runs every loop regardless of what is
  // scheduled, so a command only has to move the target. Requiring this subsystem is what stops
  // two commands aiming the azimuth at once.
  // =========================================================

  /** Ivy counterpart of {@link #setTargetAngleDegrees(double)}. */
  public CommandBuilder goToAngle(double targetAngle) {
    return Commands.instant(() -> setTargetAngleDegrees(targetAngle)).requiring(this);
  }

  // TODO (also tracked in TODO.md)
  // Homing -- 1:1 incremental encoder, so the zero has to come from somewhere:
  //   [ ] pick: carry the encoder count across OpModes from a known start position,
  //       a limit switch, or driving into a hardstop and detecting the stall (velocity ~0
  //       and current high for ~100 ms)
  //   [ ] add offsetTicks + zeroAt(knownAngle) and a static `homed` flag
  //   [ ] homeCommand(): drive slowly until switch/stall, then zeroAt(); swappable detection
  //   [ ] skip the PID in periodic() while not homed (except during homing)
  //   [ ] driver button to re-home or manually zeroAt(0)
  //   [ ] limit switch only: re-zero whenever the turret passes it
  // Hardware / config:
  //   [ ] TICKS_PER_REV = encoder counts per rev (1:1 with the turret)
  //   [ ] motor direction (constructor)
  //   [ ] confirm MIN_ANGLE / MAX_ANGLE against the cable wrap
  // Control:
  //   [ ] command that waits for isAtTarget(), for shooting in auto
  //   [ ] tune P/I/D/F, I_LIMIT, TOLERANCE_DEGREES, MAX_POWER
  //   [ ] maybe feedforward to cancel chassis rotation while aiming
  // Aiming:
  //   [ ] goal coordinate per alliance (Pedro field coordinates)
  //   [ ] target angle = atan2(goal - robot position) - robot heading, from PedroDrive.pose()
  //   [ ] aim command that updates the target every loop
  // OpMode:
  //   [ ] register in PedroSwerveTeleOp / PedroSwerveAuto, add telemetry (target, angle, power)

  // Unused for now: a running total (no wrap at +/-180) for the wrap-around math, if the
  // encoder ends up being read as an angle rather than a raw count.
  private double totalPosition = 0;
  private double lastPosition = 0;

  private void updateTotalPosition() {

  }
}
