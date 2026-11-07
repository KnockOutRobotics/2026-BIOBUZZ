package org.firstinspires.ftc.teamcode.util.tuning;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Finds the static friction ("breakaway") power: ramps open-loop power up slowly from 0 until the
 * mechanism starts to move, then, if both directions are wanted, does the same the other way. That
 * power is the F / KS term: the least power that gets the mechanism unstuck.
 */
public class StaticFrictionStep extends TuningStep {

    private static final double SETTLE_SECONDS = 1.0;

    private enum Phase { RAMP, SETTLE, DONE }

    private final String gainName;
    private final DoubleSupplier velocity;
    private final DoubleConsumer output;

    private DoubleSupplier rampRate = () -> 0.05;
    private DoubleSupplier moveThreshold = () -> 5;
    private DoubleSupplier powerScale = () -> 1;
    private double maxPower = 0.5;
    private boolean bothDirections = true;

    private Phase phase = Phase.DONE;
    private int direction;
    private double phaseStart;
    private double power;
    // index 0: positive direction, 1: negative
    private final double[] breakaway = {Double.NaN, Double.NaN};
    private final boolean[] movedWrongWay = new boolean[2];

    /**
     * @param gainName which constant the result is for, for the readout
     * @param velocity measured velocity, any units; positive the way positive power should move it
     * @param output   sets open-loop power
     */
    public StaticFrictionStep(String name, String gainName, DoubleSupplier velocity, DoubleConsumer output,
                              String... instructions) {
        super(name, instructions);
        this.gainName = gainName;
        this.velocity = velocity;
        this.output = output;
    }

    /** How fast the power ramps, power per second. Slower is more accurate. */
    public StaticFrictionStep rampRate(DoubleSupplier rampRate) {
        this.rampRate = rampRate;
        return this;
    }

    /** Speed (velocity units) above which the mechanism counts as moving. */
    public StaticFrictionStep moveThreshold(DoubleSupplier moveThreshold) {
        this.moveThreshold = moveThreshold;
        return this;
    }

    /**
     * Multiplier from raw power to the units the gain is in, e.g. batteryVoltage / 12 for a
     * voltage-compensated controller.
     */
    public StaticFrictionStep powerScale(DoubleSupplier powerScale) {
        this.powerScale = powerScale;
        return this;
    }

    /** Gives up on a direction at this power. */
    public StaticFrictionStep maxPower(double maxPower) {
        this.maxPower = maxPower;
        return this;
    }

    /** False for mechanisms that must only turn one way (a flywheel). */
    public StaticFrictionStep bothDirections(boolean bothDirections) {
        this.bothDirections = bothDirections;
        return this;
    }

    @Override
    public void start(double now) {
        breakaway[0] = breakaway[1] = Double.NaN;
        movedWrongWay[0] = movedWrongWay[1] = false;
        direction = 1;
        beginRamp(now);
    }

    private void beginRamp(double now) {
        phase = Phase.RAMP;
        phaseStart = now;
        power = 0;
    }

    @Override
    public void loop(double now) {
        double v = velocity.getAsDouble();
        switch (phase) {
            case RAMP:
                power = direction * Math.min(rampRate.getAsDouble() * (now - phaseStart), maxPower);
                boolean moving = Math.abs(v) > moveThreshold.getAsDouble(); // false for NaN
                if (moving || Math.abs(power) >= maxPower) {
                    int i = direction > 0 ? 0 : 1;
                    breakaway[i] = moving ? Math.abs(power) * powerScale.getAsDouble() : Double.POSITIVE_INFINITY;
                    movedWrongWay[i] = moving && Math.signum(v) != direction;
                    power = 0;
                    phase = Phase.SETTLE;
                    phaseStart = now;
                }
                break;
            case SETTLE:
                power = 0;
                if (now - phaseStart >= SETTLE_SECONDS) {
                    if (direction > 0 && bothDirections) {
                        direction = -1;
                        beginRamp(now);
                    } else {
                        phase = Phase.DONE;
                    }
                }
                break;
            default:
                power = 0;
        }
        output.accept(power);
    }

    @Override
    public boolean isDone() {
        return phase == Phase.DONE;
    }

    /** Mean of the directions that moved; NaN if none did. */
    public double suggestedGain() {
        double sum = 0;
        int count = 0;
        for (double b : breakaway) {
            if (!Double.isNaN(b) && !Double.isInfinite(b)) {
                sum += b;
                count++;
            }
        }
        return count == 0 ? Double.NaN : sum / count;
    }

    @Override
    public void report(Telemetry telemetry, boolean running) {
        if (running) {
            telemetry.addData("power", power);
            telemetry.addData("velocity", velocity.getAsDouble());
            telemetry.addLine(phase == Phase.RAMP
                    ? (direction > 0 ? "Ramping + ..." : "Ramping - ...")
                    : "Settling ...");
        }
        telemetry.addLine("Breakaway +: " + describe(0));
        if (bothDirections) {
            telemetry.addLine("Breakaway -: " + describe(1));
        }
        telemetry.addLine("Suggested " + gainName + ": " + fmt("%.4f", suggestedGain()));
        if (movedWrongWay[0] || movedWrongWay[1]) {
            telemetry.addLine("!! Moved the wrong way for the power sign: fix the direction setting first");
        }
    }

    private String describe(int i) {
        if (Double.isInfinite(breakaway[i])) {
            return String.format("no motion by %.2f power (check wiring / threshold)", maxPower);
        }
        return fmt("%.4f", breakaway[i]);
    }
}
