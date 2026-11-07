package org.firstinspires.ftc.teamcode.util.tuning;

import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Checks velocity feedforward on a moving target: the target sweeps back and forth as a sine wave,
 * its rate is passed to the controller, and each full cycle is measured.
 *
 * <ul>
 * <li>Amplitude ratio: how far the mechanism swung compared to the target. With feedback off
 *     (P = 0) this is set by the feedforward alone: below 1 means KV is too low, above 1 too high,
 *     and KV / ratio is the suggested value.</li>
 * <li>RMS / max error: with feedback back on, how well the whole controller tracks.</li>
 * </ul>
 *
 * <p>The first cycle includes the start-up transient, so wait for the second.
 */
public class TrackingStep extends TuningStep {

    private static final double RATIO_OK_BAND = 0.05;

    private final DoubleSupplier measured;
    private final ClosedLoop closedLoop;

    private DoubleSupplier center = () -> 0;
    private DoubleSupplier amplitude = () -> 90;
    private DoubleSupplier periodSeconds = () -> 3;
    private double wrapPeriod = 0;
    private String gainName;
    private DoubleSupplier gain;

    private double startTime;
    private double cycleStart;
    private int cycle;
    private double target;

    private double sumSquares;
    private double maxAbs;
    private double lowest;
    private double highest;
    private int samples;

    private double lastRms = Double.NaN;
    private double lastMax = Double.NaN;
    private double lastRatio = Double.NaN;
    private int lastCycle;

    public TrackingStep(String name, DoubleSupplier measured, ClosedLoop closedLoop, String... instructions) {
        super(name, instructions);
        this.measured = measured;
        this.closedLoop = closedLoop;
    }

    /** target = center + amplitude * sin(2 pi t / period) */
    public TrackingStep wave(DoubleSupplier center, DoubleSupplier amplitude, DoubleSupplier periodSeconds) {
        this.center = center;
        this.amplitude = amplitude;
        this.periodSeconds = periodSeconds;
        return this;
    }

    /** See {@link StepResponseStep#wrapPeriod}. */
    public TrackingStep wrapPeriod(double wrapPeriod) {
        this.wrapPeriod = wrapPeriod;
        return this;
    }

    /** The feedforward gain, so the readout can suggest gain / amplitude ratio. */
    public TrackingStep feedforwardGain(String gainName, DoubleSupplier gain) {
        this.gainName = gainName;
        this.gain = gain;
        return this;
    }

    @Override
    public void start(double now) {
        startTime = cycleStart = now;
        cycle = 1;
        lastRms = lastMax = lastRatio = Double.NaN;
        lastCycle = 0;
        clearCycle();
    }

    private void clearCycle() {
        sumSquares = maxAbs = 0;
        lowest = Double.POSITIVE_INFINITY;
        highest = Double.NEGATIVE_INFINITY;
        samples = 0;
    }

    @Override
    public void loop(double now) {
        double period = periodSeconds.getAsDouble();
        double a = amplitude.getAsDouble();
        if (now - cycleStart >= period) {
            if (samples > 0) {
                lastRms = Math.sqrt(sumSquares / samples);
                lastMax = maxAbs;
                lastRatio = a != 0 ? (highest - lowest) / (2 * Math.abs(a)) : Double.NaN;
                lastCycle = cycle;
            }
            cycle++;
            cycleStart = now;
            clearCycle();
        }

        double omega = 2 * Math.PI / period;
        double phase = omega * (now - startTime);
        target = center.getAsDouble() + a * Math.sin(phase);
        closedLoop.run(target, a * omega * Math.cos(phase));

        double m = measured.getAsDouble();
        if (Double.isNaN(m)) {
            return;
        }
        double error = wrapPeriod > 0 ? AngleUnwrapper.wrap(target - m, wrapPeriod) : target - m;
        double position = target - error; // measured, unwrapped next to the target
        sumSquares += error * error;
        maxAbs = Math.max(maxAbs, Math.abs(error));
        lowest = Math.min(lowest, position);
        highest = Math.max(highest, position);
        samples++;
    }

    @Override
    public void report(Telemetry telemetry, boolean running) {
        if (running) {
            double m = measured.getAsDouble();
            telemetry.addData("target", target);
            telemetry.addData("measured", wrapPeriod > 0 ? target - AngleUnwrapper.wrap(target - m, wrapPeriod) : m);
            telemetry.addLine("Cycle " + cycle + (cycle == 1 ? " (start-up, not counted yet)" : ""));
        }
        if (lastCycle == 0) {
            telemetry.addLine("No full cycle yet");
            return;
        }
        telemetry.addLine(String.format("Cycle %d%s: RMS err %s, max err %s, amplitude ratio %s", lastCycle,
                lastCycle == 1 ? " (start-up)" : "", fmt("%.2f", lastRms), fmt("%.2f", lastMax), fmt("%.3f", lastRatio)));
        if (gain != null && !Double.isNaN(lastRatio) && lastRatio > 0) {
            String verdict = Math.abs(lastRatio - 1) <= RATIO_OK_BAND ? "close enough"
                    : lastRatio < 1 ? "swinging short, raise" : "swinging wide, lower";
            telemetry.addLine(String.format("With P = 0: %s %s; try %s", verdict, gainName,
                    fmt("%.6f", gain.getAsDouble() / lastRatio)));
        }
    }
}
