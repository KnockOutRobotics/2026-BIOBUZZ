package org.firstinspires.ftc.teamcode.subsystems;

import java.util.function.Supplier;

import org.firstinspires.ftc.teamcode.ivy.Subsystem;
import org.firstinspires.ftc.teamcode.util.CRServoPair;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.util.maths.Targeting;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Turret implements Subsystem {

  private final PIDFController pidController = new PIDFController(
      SubsystemConstants.Turret.P, SubsystemConstants.Turret.I, SubsystemConstants.Turret.D,
      SubsystemConstants.Turret.F, SubsystemConstants.Turret.I_LIMIT,
      SubsystemConstants.Turret.TOLERANCE_DEGREES);
  private final Targeting targeting = new Targeting();

  private final CRServoPair servos;
  private final AnalogInput encoder;

  private double targetAngleDegrees;
  private double targetVelocityDegPerSec = 0;
  // unwrapped angle and setpoint fed to the PID, so its D term doesn't spike
  // when the encoder crosses +/-180
  private double continuousAngle = Double.NaN;
  private double lastWrappedAngle;
  private double setpointDegrees = Double.NaN;
  private double power;

  public Turret(HardwareMap hardwareMap) {
    servos = new CRServoPair(hardwareMap, SubsystemConstants.Turret.SERVO_LEADER,
        SubsystemConstants.Turret.SERVO_FOLLOWER, SubsystemConstants.Turret.FOLLOWER_REVERSED);
    encoder = hardwareMap.get(AnalogInput.class, SubsystemConstants.Turret.ENCODER);

    targetAngleDegrees = getCurrentAngleDegrees();
  }

  public void setTargetAngleDegrees(double targetAngle) {
    setTargetAngleDegrees(targetAngle, 0);
  }

  public void setTargetAngleDegrees(double targetAngle, double targetVelocity) {
    targetAngleDegrees = targetAngle;
    targetVelocityDegPerSec = targetVelocity;
  }

  public double getTargetAngleDegrees() {
    return targetAngleDegrees;
  }

  public double getEncoderVoltage() {
    return encoder.getVoltage();
  }

  /** Encoder angle before offset and direction, 0..360; the value for ANGLE_OFFSET_DEGREES. */
  public double getRawEncoderDegrees() {
    double range = SubsystemConstants.Turret.ANALOG_MAX_VOLTAGE - SubsystemConstants.Turret.ANALOG_MIN_VOLTAGE;
    return Range.clip((encoder.getVoltage() - SubsystemConstants.Turret.ANALOG_MIN_VOLTAGE) / range, 0, 1) * 360.0;
  }

  /** Degrees CCW from the robot's front, in [-180, 180). */
  public double getCurrentAngleDegrees() {
    double angle = getRawEncoderDegrees() - SubsystemConstants.Turret.ANGLE_OFFSET_DEGREES;
    return normalizeDegrees(SubsystemConstants.Turret.ENCODER_INCREASES_CCW ? angle : -angle);
  }

  public double getPowerSetpoint() {
    return power;
  }

  public boolean isAtTarget() {
    return pidController.atSetPoint();
  }

  @Override
  public void periodic() {
    pidController.setGains(SubsystemConstants.Turret.P, SubsystemConstants.Turret.I, SubsystemConstants.Turret.D,
        SubsystemConstants.Turret.F, SubsystemConstants.Turret.I_LIMIT);
    pidController.setTolerance(SubsystemConstants.Turret.TOLERANCE_DEGREES);

    double currentAngle = getCurrentAngleDegrees();
    updateContinuousAngle(currentAngle);

    double previousSetpoint = setpointDegrees;
    setpointDegrees = chooseSetpoint(currentAngle);

    if (Math.abs(setpointDegrees - previousSetpoint) > 180) {
      pidController.reset();
    }

    power = pidController.calculate(continuousAngle, setpointDegrees)
        + SubsystemConstants.Turret.KV * targetVelocityDegPerSec;
    power = Range.clip(power, -SubsystemConstants.Turret.MAX_POWER, SubsystemConstants.Turret.MAX_POWER);
    servos.setPower(SubsystemConstants.Turret.POWER_REVERSED ? -power : power);
  }

  private void updateContinuousAngle(double wrappedAngle) {
    if (Double.isNaN(continuousAngle)) {
      continuousAngle = wrappedAngle;
    } else {
      continuousAngle += normalizeDegrees(wrappedAngle - lastWrappedAngle);
    }
    lastWrappedAngle = wrappedAngle;
  }

  /**
   * The target in continuous-angle space: the short way round, unless the long
   * way continues the previous setpoint and is at most WRAP_HYSTERESIS_DEGREES
   * longer.
   */
  private double chooseSetpoint(double currentAngle) {
    double error = normalizeDegrees(targetAngleDegrees - currentAngle);
    double shortest = continuousAngle + error;
    if (Double.isNaN(setpointDegrees)) {
      return shortest;
    }

    double other = shortest - 360 * Math.signum(error);
    boolean otherContinuesPrevious = Math.abs(other - setpointDegrees) < Math.abs(shortest - setpointDegrees);
    boolean otherCloseEnough = Math.abs(other - continuousAngle)
        <= Math.abs(error) + SubsystemConstants.Turret.WRAP_HYSTERESIS_DEGREES;
    return otherContinuesPrevious && otherCloseEnough ? other : shortest;
  }

  public CommandBuilder goToAngle(double targetAngle) {
    return Commands.instant(() -> setTargetAngleDegrees(targetAngle)).requiring(this);
  }

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
            aimVelocity += SubsystemConstants.Turret.VELOCITY_FILTER * (rawVelocity - aimVelocity);
          }
          lastAimAngle = angle;
          setTargetAngleDegrees(angle, aimVelocity);
        })
        .setDone(() -> false)
        .setEnd(endCondition -> targetVelocityDegPerSec = 0);
  }

  private final ElapsedTime aimTimer = new ElapsedTime();
  private double lastAimAngle = Double.NaN;
  private double aimVelocity = 0;

  private static double normalizeDegrees(double angle) {
    return ((angle + 180) % 360 + 360) % 360 - 180;
  }

  public CommandBuilder waitUntilOnTarget() {
    return Commands.waitUntil(this::isAtTarget);
  }

  // TODO (also tracked in TODO.md)
  // [ ] robot config: turretLeader / turretFollower (CR servos), turretEncoder
  //   (analog input)
  // [ ] Turret Check: ENCODER_INCREASES_CCW, ANALOG_MIN/MAX_VOLTAGE,
  //   ANGLE_OFFSET_DEGREES, FOLLOWER_REVERSED, POWER_REVERSED
  // [ ] tune "Turret" on Dashboard (F for the servo deadband, then KV with P at
  //   0, then P/D), then copy the values back into SubsystemConstants.Turret
}
