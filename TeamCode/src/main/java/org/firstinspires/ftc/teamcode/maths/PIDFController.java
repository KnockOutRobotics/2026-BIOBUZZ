package org.firstinspires.ftc.teamcode.maths;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Position PIDF controller.
 *
 * <ul>
 * <li>P, I and D act on error = setpoint - measurement.</li>
 * <li>D is taken on the measurement rather than the error, so a jump in the setpoint (a new
 *     target, or a wrap-around picking a different copy of the angle) doesn't kick the output.</li>
 * <li>F is a static-friction term, kF * sign(error), applied only outside the tolerance so the
 *     output doesn't chatter while holding at the target.</li>
 * <li>The I term's contribution is limited to +/- iLimit (anti-windup).</li>
 * </ul>
 *
 * <p>The output is not clamped; the caller limits it to what the motor should see.
 */
public class PIDFController {

    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double iLimit;
    private double tolerance;

    private double setpoint = 0;
    private double error = Double.POSITIVE_INFINITY; // not at setpoint until the first calculate()
    private double integralSum = 0;
    private double lastMeasurement = Double.NaN;

    private final ElapsedTime timer = new ElapsedTime();

    /**
     * @param iLimit    largest output the I term may contribute
     * @param tolerance |error| at or below which {@link #atSetPoint()} is true
     */
    public PIDFController(double kP, double kI, double kD, double kF, double iLimit, double tolerance) {
        setGains(kP, kI, kD, kF, iLimit);
        this.tolerance = tolerance;
        timer.reset();
    }

    public double calculate(double measurement, double setpoint) {
        this.setpoint = setpoint;
        error = setpoint - measurement;

        double dt = timer.seconds();
        timer.reset();

        // the first call has no previous measurement to differentiate against
        double derivative = 0;
        if (!Double.isNaN(lastMeasurement) && dt > 1e-6) {
            derivative = -(measurement - lastMeasurement) / dt;
            integralSum += error * dt;
            if (kI != 0) {
                double maxSum = Math.abs(iLimit / kI);
                integralSum = Range.clip(integralSum, -maxSum, maxSum);
            }
        }
        lastMeasurement = measurement;

        double staticFriction = atSetPoint() ? 0 : kF * Math.signum(error);
        return kP * error + kI * integralSum + kD * derivative + staticFriction;
    }

    public boolean atSetPoint() {
        return Math.abs(error) <= tolerance;
    }

    public double getError() {
        return error;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setGains(double kP, double kI, double kD, double kF, double iLimit) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.iLimit = iLimit;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    /** Clears the integral and derivative history, e.g. after the mechanism was disabled. */
    public void reset() {
        integralSum = 0;
        lastMeasurement = Double.NaN;
        error = Double.POSITIVE_INFINITY;
        timer.reset();
    }
}
