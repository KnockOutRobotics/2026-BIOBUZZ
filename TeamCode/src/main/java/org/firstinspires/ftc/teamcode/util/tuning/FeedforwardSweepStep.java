package org.firstinspires.ftc.teamcode.util.tuning;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Measures the velocity feedforward: runs the mechanism open loop at a series of powers, waits for
 * the speed to settle at each, averages it, then fits power = KS + KV * velocity through the
 * points. The slope is KV; the intercept is the friction while moving (a cross-check for KS / F).
 */
public class FeedforwardSweepStep extends TuningStep {

    private enum Phase { SETTLE, MEASURE, DONE }

    private final String gainName;
    private String interceptName;
    private final DoubleSupplier velocity;
    private final DoubleConsumer output;

    private DoubleSupplier minPower = () -> 0.2;
    private DoubleSupplier maxPower = () -> 1.0;
    private IntSupplier points = () -> 5;
    private DoubleSupplier settleSeconds = () -> 1.0;
    private DoubleSupplier measureSeconds = () -> 1.0;
    private DoubleSupplier moveThreshold = () -> 0;
    private DoubleSupplier powerScale = () -> 1;
    private boolean bothDirections = false;

    private Phase phase = Phase.DONE;
    private double[] powers = new double[0];
    private int index;
    private double phaseStart;
    private double velocitySum;
    private double scaleSum;
    private int samples;
    private boolean wrongWay;
    private double topSpeed = Double.NaN;
    private final LinearFit fit = new LinearFit();
    private final List<String> pointLines = new ArrayList<>();

    /**
     * @param gainName which constant the slope is for, for the readout
     * @param velocity measured velocity; positive the way positive power should move it
     * @param output   sets open-loop power
     */
    public FeedforwardSweepStep(String name, String gainName, DoubleSupplier velocity, DoubleConsumer output,
                                String... instructions) {
        super(name, instructions);
        this.gainName = gainName;
        this.velocity = velocity;
        this.output = output;
    }

    /**
     * Names the constant the intercept goes into. Use it when the controller's static term is a
     * feedforward added whenever the target is nonzero (VelocityController's KS): friction while
     * moving is then exactly the right value. Leave it off when the static term is for breaking
     * away from rest (PIDFController's F); the intercept is then only a cross-check.
     */
    public FeedforwardSweepStep interceptGain(String interceptName) {
        this.interceptName = interceptName;
        return this;
    }

    /** Evenly spaced powers from min to max, inclusive. */
    public FeedforwardSweepStep powers(DoubleSupplier min, DoubleSupplier max, IntSupplier count) {
        minPower = min;
        maxPower = max;
        points = count;
        return this;
    }

    /** Seconds to wait for the speed to settle at each power, then seconds to average over. */
    public FeedforwardSweepStep timing(DoubleSupplier settle, DoubleSupplier measure) {
        settleSeconds = settle;
        measureSeconds = measure;
        return this;
    }

    /** Points slower than this are left out of the fit (the mechanism didn't really move). */
    public FeedforwardSweepStep moveThreshold(DoubleSupplier moveThreshold) {
        this.moveThreshold = moveThreshold;
        return this;
    }

    /** Multiplier from raw power to the gain's units, e.g. batteryVoltage / 12. */
    public FeedforwardSweepStep powerScale(DoubleSupplier powerScale) {
        this.powerScale = powerScale;
        return this;
    }

    /** Also sweeps the negative powers, fitting both directions together. */
    public FeedforwardSweepStep bothDirections(boolean bothDirections) {
        this.bothDirections = bothDirections;
        return this;
    }

    @Override
    public void start(double now) {
        int count = Math.max(1, points.getAsInt());
        double min = minPower.getAsDouble();
        double max = maxPower.getAsDouble();
        powers = new double[bothDirections ? 2 * count : count];
        for (int i = 0; i < count; i++) {
            double p = count == 1 ? max : min + (max - min) * i / (count - 1);
            powers[i] = p;
            if (bothDirections) {
                powers[count + i] = -p;
            }
        }
        fit.clear();
        pointLines.clear();
        wrongWay = false;
        topSpeed = Double.NaN;
        index = 0;
        beginPoint(now);
    }

    private void beginPoint(double now) {
        phase = Phase.SETTLE;
        phaseStart = now;
    }

    @Override
    public void loop(double now) {
        if (phase == Phase.DONE) {
            output.accept(0);
            return;
        }
        double power = powers[index];
        output.accept(power);

        if (phase == Phase.SETTLE) {
            if (now - phaseStart >= settleSeconds.getAsDouble()) {
                phase = Phase.MEASURE;
                phaseStart = now;
                velocitySum = scaleSum = 0;
                samples = 0;
            }
            return;
        }

        double v = velocity.getAsDouble();
        if (!Double.isNaN(v)) {
            velocitySum += v;
            scaleSum += powerScale.getAsDouble();
            samples++;
        }
        if (now - phaseStart >= measureSeconds.getAsDouble()) {
            recordPoint(power);
            index++;
            if (index >= powers.length) {
                phase = Phase.DONE;
                output.accept(0);
            } else {
                beginPoint(now);
            }
        }
    }

    private void recordPoint(double power) {
        if (samples == 0) {
            pointLines.add(String.format("  %+.2f -> no readings", power));
            return;
        }
        double averageVelocity = velocitySum / samples;
        double scaledPower = Math.abs(power) * scaleSum / samples;
        boolean moved = Math.abs(averageVelocity) > moveThreshold.getAsDouble();
        if (moved) {
            fit.add(Math.abs(averageVelocity), scaledPower);
            topSpeed = Double.isNaN(topSpeed) ? Math.abs(averageVelocity) : Math.max(topSpeed, Math.abs(averageVelocity));
            if (Math.signum(averageVelocity) != Math.signum(power)) {
                wrongWay = true;
            }
        }
        pointLines.add(String.format("  %+.2f -> %.1f%s", power, averageVelocity, moved ? "" : " (didn't move, left out)"));
    }

    @Override
    public boolean isDone() {
        return phase == Phase.DONE;
    }

    public double slope() {
        return fit.slope();
    }

    public double intercept() {
        return fit.intercept();
    }

    @Override
    public void report(Telemetry telemetry, boolean running) {
        if (running && phase != Phase.DONE) {
            telemetry.addData("power", powers[index]);
            telemetry.addData("velocity", velocity.getAsDouble());
            telemetry.addLine(String.format("Point %d of %d: %s", index + 1, powers.length,
                    phase == Phase.SETTLE ? "settling" : "measuring"));
        }
        telemetry.addLine("Suggested " + gainName + " (slope): " + fmt("%.6f", fit.slope()));
        telemetry.addLine((interceptName != null ? "Suggested " + interceptName + " (intercept): "
                : "Friction while moving (intercept): ") + fmt("%.4f", fit.intercept()));
        telemetry.addLine("Fit R^2 (1 = straight line): " + fmt("%.4f", fit.rSquared()));
        telemetry.addLine("Top speed: " + fmt("%.1f", topSpeed));
        if (wrongWay) {
            telemetry.addLine("!! Moved the wrong way for the power sign: fix the direction setting first");
        }
        for (String line : pointLines) {
            telemetry.addLine(line);
        }
    }
}
