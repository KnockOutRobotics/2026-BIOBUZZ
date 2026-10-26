package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.SwervePod;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.util.maths.mathsOperations;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemConstants.Drive;

import java.util.HashMap;
import java.util.Map;

/**
 * A differential swerve pod for Pedro's {@code Swerve} drivetrain: two motors whose common-mode
 * power steers the pod and whose differential power drives the wheel, plus an analog absolute
 * encoder on the steering.
 *
 * <p>Angle frames:
 * <ul>
 * <li><b>Wheel space</b> (what {@code Swerve} passes to {@link #move}): radians CCW from robot
 *     right, so pi/2 is forward.</li>
 * <li><b>Encoder space</b> (what {@link #getAngle()} returns): radians CCW from forward, wrapped to
 *     [-pi, pi). {@link #adjustThetaForEncoder} converts between them.</li>
 * </ul>
 */
public class DifferentialPod implements SwervePod {

    private final String name;
    private final DcMotorEx motor1;
    private final DcMotorEx motor2;
    private final AnalogInput encoder;
    private final Vector2D offset;
    private final double angleOffsetRad;

    private final PIDFController turnController = new PIDFController(
            Drive.TURN_P, Drive.TURN_I, Drive.TURN_D, Drive.TURN_F,
            Drive.TURN_I_LIMIT, Drive.TURN_TOLERANCE_RADIANS);

    // Unwrapped pod angle, so the PID's derivative doesn't spike when the encoder wraps at +/-pi.
    private double continuousAngle = Double.NaN;
    private double lastWrappedAngle = 0;

    private double lastMotor1Power = 0;
    private double lastMotor2Power = 0;
    private double lastTurnPower = 0;
    private double lastDrivePower = 0;
    private double lastTargetRad = 0;

    /**
     * @param offset         pod position from robot center, x forward / y RIGHT, inches
     * @param angleOffsetRad raw encoder angle when the wheel points forward
     */
    public DifferentialPod(HardwareMap hardwareMap, String name, String motor1Name, String motor2Name,
                           String encoderName, Vector2D offset, double angleOffsetRad) {
        this.name = name;
        this.offset = offset;
        this.angleOffsetRad = angleOffsetRad;

        motor1 = hardwareMap.get(DcMotorEx.class, motor1Name);
        motor2 = hardwareMap.get(DcMotorEx.class, motor2Name);
        encoder = hardwareMap.get(AnalogInput.class, encoderName);

        for (DcMotorEx motor : new DcMotorEx[]{motor1, motor2}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setPower(0);
        }
    }

    @Override
    public String name() {
        return name;
    }

    @Override
    public Vector2D getOffset() {
        return offset;
    }

    /** Pod angle in encoder space: radians CCW from forward, in [-pi, pi). */
    @Override
    public double getAngle() {
        double range = Drive.ANALOG_MAX_VOLTAGE - Drive.ANALOG_MIN_VOLTAGE;
        double normalized = Range.clip((encoder.getVoltage() - Drive.ANALOG_MIN_VOLTAGE) / range, 0, 1);
        double raw = normalized * 2 * Math.PI;
        if (!Drive.ENCODER_INCREASES_CCW) {
            raw = -raw;
        }
        return wrap(raw - angleOffsetRad);
    }

    @Override
    public double adjustThetaForEncoder(double wheelTheta) {
        return wrap(wheelTheta - Math.PI / 2);
    }

    @Override
    public void move(double targetAngleRad, double drivePower, boolean ignoreAngleChanges) {
        double current = getAngle();
        updateContinuousAngle(current);

        double target = adjustThetaForEncoder(targetAngleRad);
        double error = wrap(target - current);

        // Never steer more than 90 degrees: point the other way and drive backwards instead.
        if (Math.abs(error) > Math.PI / 2) {
            target = wrap(target + Math.PI);
            error = wrap(target - current);
            drivePower = -drivePower;
        }

        double turnPower = 0;
        if (!ignoreAngleChanges) {
            turnPower = Range.clip(
                    turnController.calculate(continuousAngle, continuousAngle + error), -1, 1);
        }

        double rotate = Drive.TURN_REVERSED ? -turnPower : turnPower;
        double translate = Drive.DRIVE_REVERSED ? -drivePower : drivePower;
        double[] powers = mathsOperations.diffyConvert(rotate, translate);

        lastTargetRad = target;
        lastTurnPower = turnPower;
        lastDrivePower = drivePower;
        setMotorPowers(powers[0], powers[1]);
    }

    @Override
    public void setToFloat() {
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void setToBreak() {
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public Map<String, Object> debug() {
        Map<String, Object> map = new HashMap<>();
        map.put("angleDeg", Math.toDegrees(getAngle()));
        map.put("targetDeg", Math.toDegrees(lastTargetRad));
        map.put("turnPower", lastTurnPower);
        map.put("drivePower", lastDrivePower);
        map.put("motor1Power", lastMotor1Power);
        map.put("motor2Power", lastMotor2Power);
        return map;
    }

    private void updateContinuousAngle(double wrappedAngle) {
        if (Double.isNaN(continuousAngle)) {
            continuousAngle = wrappedAngle;
        } else {
            continuousAngle += wrap(wrappedAngle - lastWrappedAngle);
        }
        lastWrappedAngle = wrappedAngle;
    }

    private void setMotorPowers(double power1, double power2) {
        if (shouldWrite(power1, lastMotor1Power)) {
            motor1.setPower(power1);
            lastMotor1Power = power1;
        }
        if (shouldWrite(power2, lastMotor2Power)) {
            motor2.setPower(power2);
            lastMotor2Power = power2;
        }
    }

    /** Skips tiny changes to save bus time, but always lets the motor reach exactly 0. */
    private static boolean shouldWrite(double power, double lastPower) {
        return Math.abs(power - lastPower) >= Drive.MOTOR_CACHING_THRESHOLD
                || (power == 0 && lastPower != 0);
    }

    /** Wraps to [-pi, pi). */
    private static double wrap(double angle) {
        double wrapped = (angle + Math.PI) % (2 * Math.PI);
        if (wrapped < 0) {
            wrapped += 2 * Math.PI;
        }
        return wrapped - Math.PI;
    }
}
