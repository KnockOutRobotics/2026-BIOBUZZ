package org.firstinspires.ftc.teamcode.util.tuning;

/**
 * Turns a wrapped angle (one that jumps from +180 to -180) into a continuous one, by assuming it
 * never moves more than half a period between two updates.
 */
public class AngleUnwrapper {

    private final double period;
    private double continuous = Double.NaN;
    private double lastWrapped;

    /** @param period the wrap period, e.g. 360 for degrees */
    public AngleUnwrapper(double period) {
        this.period = period;
    }

    public double update(double wrapped) {
        if (Double.isNaN(wrapped)) {
            return continuous;
        }
        if (Double.isNaN(continuous)) {
            continuous = wrapped;
        } else {
            continuous += wrap(wrapped - lastWrapped, period);
        }
        lastWrapped = wrapped;
        return continuous;
    }

    public void reset() {
        continuous = Double.NaN;
    }

    /** Wraps {@code value} into [-period/2, period/2). */
    public static double wrap(double value, double period) {
        double wrapped = (value + period / 2) % period;
        if (wrapped < 0) {
            wrapped += period;
        }
        return wrapped - period / 2;
    }
}
