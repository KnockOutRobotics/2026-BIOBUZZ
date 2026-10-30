package org.firstinspires.ftc.teamcode.subsystems;

public class SubsystemConstants {
    public class Turret {
        public final static double P = 0.0;
        public final static double I = 0.0;
        public final static double D = 0.0;
        public final static double F = 0.0; // static friction: F * sign(error), off inside tolerance
        public final static double I_LIMIT = 0.2; // most power the I term may add
        public final static double TOLERANCE_DEGREES = 10;
        public final static double TICKS_PER_REV = 20.0;
        public final static double MIN_ANGLE = -270;
        public final static double MAX_ANGLE = 270;
        public final static double MAX_POWER = 1.0; // PID output is clipped to +/- this
        // another copy of the target must be this much closer before the turret switches to it
        public final static double WRAP_HYSTERESIS_DEGREES = 20;
    }

    public static class Intake {
        public static final double NECTAR_POWER = 0.2;
        public static final double POLLEN_POWER = 0.2;
    }

    /**
     * Differential swerve + Pedro Pathing. Anything set to {@code Double.NaN} has not been measured
     * or tuned yet; {@code pedroPathing.Constants} refuses to build the drivetrain or follower
     * (and names the missing values) until it is filled in.
     */
    public static class Drive {
        // ---- Hardware names (same as SwerveDrive). Module 1 is the right pod, module 2 the left.
        public static final String RIGHT_POD_MOTOR_1 = "mod1m1";
        public static final String RIGHT_POD_MOTOR_2 = "mod1m2";
        public static final String RIGHT_POD_ENCODER = "mod1E";
        public static final String LEFT_POD_MOTOR_1 = "mod2m1";
        public static final String LEFT_POD_MOTOR_2 = "mod2m2";
        public static final String LEFT_POD_ENCODER = "mod2E";

        // ---- Pod positions, inches from the robot's center of rotation.
        // Pedro's Swerve expects x = FORWARD and y = RIGHT (checked against its rotation math; with
        // y = left the robot spins the wrong way). The right pod is placed at (POD_X, +POD_Y) and
        // the left pod at (POD_X, -POD_Y).
        public static final double POD_X_INCHES = Double.NaN; // forward offset; 0 if pods are on the center line
        public static final double POD_Y_INCHES = Double.NaN; // half the distance between the pods

        // ---- Analog pod encoders. Voltage range from SwerveDrive.readEncoder().
        public static final double ANALOG_MIN_VOLTAGE = 0.043;
        public static final double ANALOG_MAX_VOLTAGE = 3.143;
        // Raw encoder angle when the wheel points forward (SwerveTeleOpConfig.module1Adjust/2Adjust).
        public static final double RIGHT_POD_ANGLE_OFFSET_DEGREES = 35.0;
        public static final double LEFT_POD_ANGLE_OFFSET_DEGREES = 65.0;

        // ---- Directions. Defaults match the signs SwerveDrive.drive() uses.
        public static final boolean ENCODER_INCREASES_CCW = true; // viewed from above
        public static final boolean TURN_REVERSED = true;  // true: +common-mode power turns the pod CW
        public static final boolean DRIVE_REVERSED = false; // true: +differential power drives the wheel backward

        // ---- Pod turn PIDF, on angle error in radians (util.PIDFController).
        // P = SwerveTeleOpConfig.Kp (0.05/deg, divided by 10 inside PIDcontroller) in radians.
        public static final double TURN_P = 0.29;
        public static final double TURN_I = 0.0;
        public static final double TURN_D = 0.0;
        public static final double TURN_F = 0.0; // static friction, F * sign(error)
        public static final double TURN_I_LIMIT = 0.2;
        public static final double TURN_TOLERANCE_RADIANS = Math.toRadians(1);
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
