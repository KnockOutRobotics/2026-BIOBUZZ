package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.ivy.Subsystem;
import org.firstinspires.ftc.teamcode.util.CRServoPair;
import org.firstinspires.ftc.teamcode.util.VelocityController;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Flywheel launcher. The flywheel holds a target speed with a {@link VelocityController} run in
 * {@link #periodic()}; gains are in {@link SubsystemConstants.Launcher}, live on Dashboard. Tune
 * them with the "Launcher PID Tuner" OpMode (OpMode/tuning/TUNING.md).
 *
 * <p>The x-wheels and transfer servos are mapped; their behavior is still to be defined.
 */
public class Launcher implements Subsystem {

  // the battery voltage is a separate bus read, so it is only refreshed this often
  private static final double VOLTAGE_REFRESH_SECONDS = 0.25;

  private final DcMotorEx flywheel;
  private final CRServoPair xWheels;
  private final CRServo transferPivot;
  private final CRServo transferRoller;
  private final VoltageSensor battery;

  private final VelocityController flywheelController = new VelocityController(
      SubsystemConstants.Launcher.KV, SubsystemConstants.Launcher.KS, SubsystemConstants.Launcher.KP,
      SubsystemConstants.Launcher.KI, SubsystemConstants.Launcher.I_LIMIT, SubsystemConstants.Launcher.TOLERANCE);

  private double targetVelocity = 0;
  private boolean openLoop = false;
  private double flywheelPower = 0;

  private double batteryVoltage = Double.NaN;
  private final ElapsedTime voltageTimer = new ElapsedTime();

  public Launcher(HardwareMap hwm) {
    flywheel = hwm.get(DcMotorEx.class, SubsystemConstants.Launcher.FLYWHEEL);
    flywheel.setMode(RunMode.RUN_WITHOUT_ENCODER);
    flywheel.setDirection(Direction.FORWARD);
    // coast down rather than braking the flywheel against its gearbox
    flywheel.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);

    xWheels = new CRServoPair(hwm, SubsystemConstants.Launcher.X_WHEELS_LEADER,
        SubsystemConstants.Launcher.X_WHEELS_FOLLOWER, false);

    transferPivot = hwm.get(CRServo.class, SubsystemConstants.Launcher.TRANSFER_PIVOT);
    transferRoller = hwm.get(CRServo.class, SubsystemConstants.Launcher.TRANSFER_ROLLER);

    battery = hwm.voltageSensor.iterator().next();
  }

  /** Flywheel target in encoder ticks/s; 0 lets it coast down. */
  public void setFlywheelVelocity(double ticksPerSecond) {
    if (openLoop) {
      openLoop = false;
      flywheelController.reset();
    }
    targetVelocity = ticksPerSecond;
  }

  /**
   * Drives the flywheel at a fixed power (clipped to 0..1, it never runs backward), bypassing the
   * controller until the next {@link #setFlywheelVelocity}. For the tuner's open-loop steps.
   */
  public void setFlywheelOpenLoop(double power) {
    openLoop = true;
    targetVelocity = 0;
    flywheelPower = Range.clip(power, 0, 1);
    flywheel.setPower(flywheelPower);
  }

  public double getFlywheelVelocity() {
    return flywheel.getVelocity();
  }

  public double getFlywheelTarget() {
    return targetVelocity;
  }

  public double getFlywheelPower() {
    return flywheelPower;
  }

  /** True once the flywheel is within TOLERANCE of a nonzero target. */
  public boolean isFlywheelAtSpeed() {
    return !openLoop && flywheelController.atSetPoint();
  }

  /** Battery voltage, refreshed at most every VOLTAGE_REFRESH_SECONDS. */
  public double getBatteryVoltage() {
    if (Double.isNaN(batteryVoltage) || voltageTimer.seconds() >= VOLTAGE_REFRESH_SECONDS) {
      batteryVoltage = battery.getVoltage();
      voltageTimer.reset();
    }
    return batteryVoltage;
  }

  public void setXWheelPower(double power) {
    xWheels.setPower(power);
  }

  @Override
  public void periodic() {
    if (openLoop) {
      return;
    }
    // re-read every loop so Dashboard edits take effect live
    flywheelController.setGains(SubsystemConstants.Launcher.KV, SubsystemConstants.Launcher.KS,
        SubsystemConstants.Launcher.KP, SubsystemConstants.Launcher.KI, SubsystemConstants.Launcher.I_LIMIT);
    flywheelController.setTolerance(SubsystemConstants.Launcher.TOLERANCE);

    flywheelPower = Range.clip(
        flywheelController.calculate(getFlywheelVelocity(), targetVelocity, getBatteryVoltage()), 0, 1);
    flywheel.setPower(flywheelPower);
  }
}
