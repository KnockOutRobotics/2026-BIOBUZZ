package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

/**
 * Every subsystem's constants. Fields that are {@code public static} but not
 * {@code final} are
 * live-tunable in FTC Dashboard (the {@link Config} classes, shown as "Turret", "Drive"
 * and "Launcher")
 * and re-read by the subsystem every loop; copy tuned values back here so they
 * survive a
 * redeploy.
 */
public class SubsystemConstants {

  @Config("Turret")
  public static class Turret {
    // ---- Hardware names. Two CR servos drive the turret (infinite rotation); a
    // MelonBotics encoder geared 1:1 with the turret reports its absolute angle
    // on an analog port.
    public static final String SERVO_LEADER = "turretLeader";
    public static final String SERVO_FOLLOWER = "turretFollower";
    public static final String ENCODER = "turretEncoder";

    // ---- Directions.
    public static final boolean FOLLOWER_REVERSED = false; // true: the follower servo is mounted mirrored
    public static final boolean POWER_REVERSED = false; // true: +power turns the turret CW (viewed from above)
    public static final boolean ENCODER_INCREASES_CCW = true; // viewed from above

    // ---- Analog encoder. TODO placeholders: measure the voltage range and the
    // offset (Turret Check).
    public static final double ANALOG_MIN_VOLTAGE = 0.0;
    public static final double ANALOG_MAX_VOLTAGE = 3.3;
    // raw encoder angle when the turret faces the robot's front
    public static final double ANGLE_OFFSET_DEGREES = 0;

    // with the target about 180 degrees away, the turret keeps turning the way
    // it was going unless the other way is shorter by more than this
    public static final double WRAP_HYSTERESIS_DEGREES = 20;

    // ---- Live-tunable PIDF (util.PIDFController), on angle error in degrees.
    public static double P = 0.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 0.0; // static friction / servo deadband: F * sign(error), off inside tolerance
    public static double I_LIMIT = 0.2; // most power the I term may add
    public static double TOLERANCE_DEGREES = 2;
    public static double MAX_POWER = 1.0; // PID + feedforward is clipped to +/- this

    // Velocity feedforward: power per degree/second of target motion. Tune with P
    // at 0: spin
    // the robot in place while aiming and raise KV until the turret holds on the
    // goal.
    public static double KV = 0.0;
    // Smoothing on the measured target velocity, 0..1: 1 = raw, lower = smoother
    // but laggier.
    public static double VELOCITY_FILTER = 0.5;
  }

  public static class Intake {
    public static final String POLLEN_MOTOR = "pollenMotor";
    public static final String NECTAR_MOTOR = "nectarMotor";

    public static final double NECTAR_POWER = 0.2;
    public static final double POLLEN_POWER = 0.2;
  }

  /**
   * Flywheel controller (util.VelocityController) and launcher hardware.
   * Velocities are encoder
   * ticks/s from {@code DcMotorEx.getVelocity()}.
   *
   * <p>
   * PLACEHOLDERS: rough guesses for a 6000 rpm motor (28 ticks/rev, ~2800 ticks/s
   * free
   * speed), not measured. Tune in order: KS, then KV with KP at 0, then KP, then
   * KI only if
   * needed.
   */
  @Config("Launcher")
  public static class Launcher {
    public static final String FLYWHEEL = "flywheel";
    public static final String X_WHEELS_LEADER = "xWheelsLeader";
    public static final String X_WHEELS_FOLLOWER = "xWheelsFollower";
    public static final String TRANSFER_PIVOT = "transferPivot";
    public static final String TRANSFER_ROLLER = "transferRoller";

    // ---- Live-tunable flywheel controller.
    public static double KV = 0.00035; // power per tick/s; ~1 / free speed
    public static double KS = 0.05; // power that just starts the wheel turning
    public static double KP = 0.0005; // power per tick/s of error
    public static double KI = 0.0;
    public static double I_LIMIT = 0.1; // most power the I term may add
    public static double TOLERANCE = 50; // ticks/s; within this counts as at speed

    public static double POLLEN_VELOCITY = 1800; // ticks/s
    public static double NECTAR_VELOCITY = 1500; // ticks/s

    public static double IDLE_VELOCITY = 500; // ticks/s
  }

  /**
   * Differential swerve + Pedro Pathing. Anything set to {@code Double.NaN} has
   * not been measured
   * or tuned yet; {@code pedroPathing.Constants} refuses to build the drivetrain
   * or follower
   * (and names the missing values) until it is filled in.
   *
   * <p>
   * Only the pod turn gains are live-tunable ("Drive" on Dashboard); everything
   * else is final.
   */
  @Config("Drive")
  public static class Drive {
    // ---- Hardware names. Module 1 is the right pod, module 2 the left.
    public static final String RIGHT_POD_MOTOR_1 = "mod1m1";
    public static final String RIGHT_POD_MOTOR_2 = "mod1m2";
    public static final String RIGHT_POD_ENCODER = "mod1E";
    public static final String LEFT_POD_MOTOR_1 = "mod2m1";
    public static final String LEFT_POD_MOTOR_2 = "mod2m2";
    public static final String LEFT_POD_ENCODER = "mod2E";

    // ---- Pod positions, inches from the robot's center of rotation.
    // Pedro's Swerve expects x = FORWARD and y = RIGHT (checked against its
    // rotation math; with
    // y = left the robot spins the wrong way). The right pod is placed at (POD_X,
    // +POD_Y) and
    // the left pod at (POD_X, -POD_Y).
    public static final double POD_X_INCHES = Double.NaN; // forward offset; 0 if pods are on the center line
    public static final double POD_Y_INCHES = Double.NaN; // half the distance between the pods

    // ---- Analog pod encoders. Voltage range measured on the old SwerveDrive.
    public static final double ANALOG_MIN_VOLTAGE = 0.043;
    public static final double ANALOG_MAX_VOLTAGE = 3.143;
    // Raw encoder angle when the wheel points forward (carried over from the old
    // SwerveDrive).
    public static final double RIGHT_POD_ANGLE_OFFSET_DEGREES = 35.0;
    public static final double LEFT_POD_ANGLE_OFFSET_DEGREES = 65.0;

    // ---- Directions. Defaults match the signs the old SwerveDrive used.
    public static final boolean ENCODER_INCREASES_CCW = true; // viewed from above
    public static final boolean TURN_REVERSED = true; // true: +common-mode power turns the pod CW
    public static final boolean DRIVE_REVERSED = false; // true: +differential power drives the wheel backward

    // ---- Live-tunable pod turn PIDF, on angle error in radians
    // (util.PIDFController), re-read by DifferentialPod every loop. Tune with the
    // "Drive Pod PID Tuner" OpMode (OpMode/tuning/TUNING.md).
    // P = the old SwerveDrive gain (0.05/deg, divided by 10 inside its PID) in
    // radians.
    public static double TURN_P = 0.29;
    public static double TURN_I = 0.0;
    public static double TURN_D = 0.0;
    public static double TURN_F = 0.0; // static friction, F * sign(error)
    public static double TURN_I_LIMIT = 0.2;
    public static double TURN_TOLERANCE_RADIANS = Math.toRadians(1);
    // Skip a motor write unless the power moved by at least this much.
    public static final double MOTOR_CACHING_THRESHOLD = 0.01;

    // ---- Pedro Swerve options.
    public static final boolean VOLTAGE_COMPENSATION = false;

    // ---- Foresight path follower. Copy these from AutoTune > Foresight Tuner.
    public static final double FORWARD_TRANSLATIONAL_PRIMARY_P = Double.NaN;
    public static final double FORWARD_TRANSLATIONAL_SECONDARY_P = Double.NaN;
    public static final double STRAFE_TRANSLATIONAL_PRIMARY_P = Double.NaN;
    public static final double STRAFE_TRANSLATIONAL_SECONDARY_P = Double.NaN;
    public static final double COAST_KV = Double.NaN;
    public static final double BRAKE_KV = Double.NaN;
    public static final double HEADING_P = Double.NaN;
    public static final double HEADING_BRAKING_LINEAR = Double.NaN;
    public static final double HEADING_BRAKING_QUADRATIC = Double.NaN;
    public static final double FORWARD_BRAKING_LINEAR = Double.NaN;
    public static final double FORWARD_BRAKING_QUADRATIC = Double.NaN;
    public static final double STRAFE_BRAKING_LINEAR = Double.NaN;
    public static final double STRAFE_BRAKING_QUADRATIC = Double.NaN;
    public static final double MAX_ACHIEVABLE_FORWARD_VELOCITY = Double.NaN;
    public static final double MAX_ACHIEVABLE_STRAFE_VELOCITY = Double.NaN;
    public static final double NATURAL_FORWARD_DECELERATION = Double.NaN;
    public static final double NATURAL_STRAFE_DECELERATION = Double.NaN;
  }
}
