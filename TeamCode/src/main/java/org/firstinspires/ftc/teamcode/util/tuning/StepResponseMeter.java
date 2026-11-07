package org.firstinspires.ftc.teamcode.util.tuning;

/**
 * Measures one step response, live: rise time, overshoot, settling time and the error left over.
 * Call {@link #start} when the target jumps, then {@link #update} every loop. Times are seconds
 * since the step.
 *
 * <ul>
 * <li>Rise time: from 10% to 90% of the way to the new target.</li>
 * <li>Overshoot: how far past the target it went, as a percentage of the step.</li>
 * <li>Settling time: when it last entered the tolerance band; NaN while outside it.</li>
 * <li>Final error: measured - target, smoothed over about 0.2 s, so it reads the steady-state
 *     error once settled.</li>
 * </ul>
 */
public class StepResponseMeter {

    private static final double FINAL_ERROR_TIME_CONSTANT = 0.2;

    private double startTime;
    private double from;
    private double to;
    private double tolerance;
    private double lastTime;

    private double t10 = Double.NaN;
    private double t90 = Double.NaN;
    private double peakProgress = Double.NaN;
    private double enteredBandTime = Double.NaN;
    private double finalError = Double.NaN;

    public void start(double time, double from, double to, double tolerance) {
        this.startTime = time;
        this.from = from;
        this.to = to;
        this.tolerance = Math.abs(tolerance);
        lastTime = time;
        t10 = t90 = peakProgress = enteredBandTime = finalError = Double.NaN;
    }

    public void update(double time, double measured) {
        if (Double.isNaN(measured)) {
            return;
        }
        double step = to - from;
        double error = measured - to;
        double elapsed = time - startTime;

        if (step != 0) {
            double progress = (measured - from) / step;
            peakProgress = Double.isNaN(peakProgress) ? progress : Math.max(peakProgress, progress);
            if (Double.isNaN(t10) && progress >= 0.1) {
                t10 = elapsed;
            }
            if (Double.isNaN(t90) && progress >= 0.9) {
                t90 = elapsed;
            }
        }

        if (Math.abs(error) <= tolerance) {
            if (Double.isNaN(enteredBandTime)) {
                enteredBandTime = elapsed;
            }
        } else {
            enteredBandTime = Double.NaN;
        }

        double dt = time - lastTime;
        lastTime = time;
        if (Double.isNaN(finalError)) {
            finalError = error;
        } else {
            finalError += dt / (FINAL_ERROR_TIME_CONSTANT + dt) * (error - finalError);
        }
    }

    public double stepSize() {
        return to - from;
    }

    /** 10% to 90% rise time, s; NaN until it has got 90% of the way. */
    public double riseTime() {
        return t90 - t10;
    }

    /** Percent of the step it went past the target; 0 if it never did. NaN before any update. */
    public double overshootPercent() {
        return Double.isNaN(peakProgress) ? Double.NaN : Math.max(0, peakProgress - 1) * 100;
    }

    /** Time since the step at which it entered the band and has stayed there; NaN while outside. */
    public double settlingTime() {
        return enteredBandTime;
    }

    public boolean isSettled() {
        return !Double.isNaN(enteredBandTime);
    }

    /** measured - target, smoothed. */
    public double finalError() {
        return finalError;
    }
}
