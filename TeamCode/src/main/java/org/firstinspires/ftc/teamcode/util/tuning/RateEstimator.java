package org.firstinspires.ftc.teamcode.util.tuning;

import java.util.ArrayDeque;

/**
 * Rate of change of a sampled value, taken across a short time window rather than between two
 * consecutive loops, so encoder noise and loop-time jitter don't swamp it. Feed it a continuous
 * value (unwrap angles first with {@link AngleUnwrapper}).
 */
public class RateEstimator {

    private final double windowSeconds;
    private final ArrayDeque<double[]> samples = new ArrayDeque<>(); // {time, value}

    public RateEstimator(double windowSeconds) {
        this.windowSeconds = windowSeconds;
    }

    public void add(double time, double value) {
        if (Double.isNaN(value)) {
            return;
        }
        samples.addLast(new double[]{time, value});
        while (samples.size() > 2 && time - samples.peekFirst()[0] > windowSeconds) {
            samples.removeFirst();
        }
    }

    /** Units per second; NaN until two samples at different times. */
    public double rate() {
        if (samples.size() < 2) {
            return Double.NaN;
        }
        double[] first = samples.peekFirst();
        double[] last = samples.peekLast();
        double dt = last[0] - first[0];
        return dt > 1e-6 ? (last[1] - first[1]) / dt : Double.NaN;
    }

    public void reset() {
        samples.clear();
    }
}
