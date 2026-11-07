package org.firstinspires.ftc.teamcode.util.tuning;

import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Square-wave test of the closed loop: the target flips between a low and a high value every
 * period, and each flip is measured with a {@link StepResponseMeter}. Change the gains live on
 * Dashboard while it runs and watch the numbers (and the target/measured graph) respond.
 */
public class StepResponseStep extends TuningStep {

    private final DoubleSupplier measured;
    private final ClosedLoop closedLoop;

    private DoubleSupplier low = () -> 0;
    private DoubleSupplier high = () -> 90;
    private DoubleSupplier periodSeconds = () -> 2;
    private DoubleSupplier tolerance = () -> 1;
    private double wrapPeriod = 0;

    private final StepResponseMeter meter = new StepResponseMeter();
    private boolean atHigh;
    private double target;
    private double stepStart;
    private int stepCount;

    private final Result lastUp = new Result();
    private final Result lastDown = new Result();

    /**
     * @param measured   the controlled quantity, same units as the targets
     * @param closedLoop runs the mechanism's real controller toward a target
     */
    public StepResponseStep(String name, DoubleSupplier measured, ClosedLoop closedLoop, String... instructions) {
        super(name, instructions);
        this.measured = measured;
        this.closedLoop = closedLoop;
    }

    public StepResponseStep targets(DoubleSupplier low, DoubleSupplier high) {
        this.low = low;
        this.high = high;
        return this;
    }

    /** Seconds at each target before flipping; long enough to settle. */
    public StepResponseStep period(DoubleSupplier periodSeconds) {
        this.periodSeconds = periodSeconds;
        return this;
    }

    /** The band that counts as settled, usually the controller's own tolerance. */
    public StepResponseStep tolerance(DoubleSupplier tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    /**
     * For angles: measured is compared to the target modulo this (360 for a turret, 180 for a
     * swerve pod that may point either way). 0 (the default) compares directly.
     */
    public StepResponseStep wrapPeriod(double wrapPeriod) {
        this.wrapPeriod = wrapPeriod;
        return this;
    }

    @Override
    public void start(double now) {
        lastUp.clear();
        lastDown.clear();
        stepCount = 0;
        atHigh = false;
        flip(now);
    }

    private void flip(double now) {
        atHigh = !atHigh;
        target = atHigh ? high.getAsDouble() : low.getAsDouble();
        stepStart = now;
        stepCount++;
        meter.start(now, relative(measured.getAsDouble(), target), target, tolerance.getAsDouble());
    }

    /** measured, moved by whole wrap periods to sit within half a period of the target. */
    private double relative(double value, double toward) {
        return wrapPeriod > 0 ? toward + AngleUnwrapper.wrap(value - toward, wrapPeriod) : value;
    }

    @Override
    public void loop(double now) {
        if (now - stepStart >= periodSeconds.getAsDouble()) {
            (atHigh ? lastUp : lastDown).capture(meter);
            flip(now);
        }
        closedLoop.run(target, 0);
        meter.update(now, relative(measured.getAsDouble(), target));
    }

    @Override
    public void report(Telemetry telemetry, boolean running) {
        if (running) {
            double m = relative(measured.getAsDouble(), target);
            telemetry.addData("target", target);
            telemetry.addData("measured", m);
            telemetry.addData("error", target - m);
            telemetry.addLine(String.format("Step %d (%s): rise %s s, overshoot %s %%, settled %s",
                    stepCount, atHigh ? "up" : "down", fmt("%.2f", meter.riseTime()),
                    fmt("%.1f", meter.overshootPercent()),
                    meter.isSettled() ? fmt("%.2f", meter.settlingTime()) + " s" : "no"));
        }
        telemetry.addLine("Last up:   " + lastUp);
        telemetry.addLine("Last down: " + lastDown);
    }

    /** A finished step's numbers. */
    private static final class Result {
        double rise = Double.NaN;
        double overshoot = Double.NaN;
        double settle = Double.NaN;
        double finalError = Double.NaN;
        boolean valid;

        void capture(StepResponseMeter meter) {
            rise = meter.riseTime();
            overshoot = meter.overshootPercent();
            settle = meter.settlingTime();
            finalError = meter.finalError();
            valid = true;
        }

        void clear() {
            valid = false;
        }

        @Override
        public String toString() {
            if (!valid) {
                return "--";
            }
            return String.format("rise %s s, overshoot %s %%, settle %s, final err %s",
                    fmt("%.2f", rise), fmt("%.1f", overshoot),
                    Double.isNaN(settle) ? "never" : fmt("%.2f", settle) + " s", fmt("%.2f", finalError));
        }
    }
}
