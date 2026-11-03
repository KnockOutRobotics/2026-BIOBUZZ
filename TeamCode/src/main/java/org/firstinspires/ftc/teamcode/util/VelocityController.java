package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Velocity controller for flywheels and other spinning mechanisms: feedforward does most of the
 * work, feedback cleans up the rest.
 *
 * <ul>
 * <li>Feedforward: kV * target + kS * sign(target). kS is the power that just overcomes
 *     friction; kV is the power per unit of velocity on top of that.</li>
 * <li>Feedback: P and I on error = target - measured. No D -- measured velocity is already
 *     noisy, and differentiating it makes that worse.</li>
 * <li>The I term's contribution is limited to +/- iLimit (anti-windup), and cleared whenever the
 *     target is 0.</li>
 * <li>Voltage compensation: the whole output is scaled by NOMINAL_VOLTAGE / battery voltage, so
 *     gains tuned on a full battery hold as it sags.</li>
 * <li>A target of 0 gives 0 output, so the mechanism coasts down instead of being braked or
 *     pushed by kS.</li>
 * </ul>
 *
 * <p>Velocity units are whatever the caller uses (ticks/s from {@code DcMotorEx.getVelocity()},
 * typically) as long as the gains and tolerance match. The output is not clamped; the caller
 * limits it to what the motor should see (for a flywheel, 0..1 so it never drives backward).
 */
public class VelocityController {

    /** Battery voltage the gains are tuned at. */
    public static final double NOMINAL_VOLTAGE = 12.0;

    private double kV;
    private double kS;
    private double kP;
    private double kI;
    private double iLimit;
    private double tolerance;

    private double target = 0;
    private double error = Double.POSITIVE_INFINITY; // not at speed until the first calculate()
    private double integralSum = 0;
    private boolean hasLastCall = false;

    private final ElapsedTime timer = new ElapsedTime();

    /**
     * @param iLimit    largest output the I term may contribute
     * @param tolerance |error| at or below which {@link #atSetPoint()} is true
     */
    public VelocityController(double kV, double kS, double kP, double kI, double iLimit, double tolerance) {
        setGains(kV, kS, kP, kI, iLimit);
        this.tolerance = tolerance;
        timer.reset();
    }

    /**
     * @param measured current velocity
     * @param target   desired velocity
     * @param voltage  battery voltage, for compensation; NaN or non-positive skips compensation
     * @return motor power, unclamped
     */
    public double calculate(double measured, double target, double voltage) {
        this.target = target;
        error = target - measured;

        double dt = timer.seconds();
        timer.reset();

        if (target == 0) {
            integralSum = 0;
            hasLastCall = true;
            return 0;
        }

        // the first call has no interval to integrate over
        if (hasLastCall && dt > 1e-6) {
            integralSum += error * dt;
            if (kI != 0) {
                double maxSum = Math.abs(iLimit / kI);
                integralSum = Range.clip(integralSum, -maxSum, maxSum);
            }
        }
        hasLastCall = true;

        double feedforward = kV * target + kS * Math.signum(target);
        double feedback = kP * error + kI * integralSum;
        double output = feedforward + feedback;

        if (voltage > 0) {
            output *= NOMINAL_VOLTAGE / voltage;
        }
        return output;
    }

    /** {@link #calculate(double, double, double)} without voltage compensation. */
    public double calculate(double measured, double target) {
        return calculate(measured, target, Double.NaN);
    }

    /** True once the measured velocity is within tolerance of a nonzero target. */
    public boolean atSetPoint() {
        return target != 0 && Math.abs(error) <= tolerance;
    }

    public double getError() {
        return error;
    }

    public double getTarget() {
        return target;
    }

    public void setGains(double kV, double kS, double kP, double kI, double iLimit) {
        this.kV = kV;
        this.kS = kS;
        this.kP = kP;
        this.kI = kI;
        this.iLimit = iLimit;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    /** Clears the integral history, e.g. after the mechanism was disabled. */
    public void reset() {
        integralSum = 0;
        hasLastCall = false;
        error = Double.POSITIVE_INFINITY;
        timer.reset();
    }
}
