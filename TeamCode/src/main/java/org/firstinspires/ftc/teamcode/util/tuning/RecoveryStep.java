package org.firstinspires.ftc.teamcode.util.tuning;

import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Disturbance rejection for a velocity loop (a flywheel losing speed to a shot): holds one target,
 * and each time the speed drops more than a threshold below it, measures how far it dipped and how
 * long it took to get back inside the tolerance band.
 */
public class RecoveryStep extends TuningStep {

    private enum State { SPINNING_UP, READY, OUT_OF_BAND, DIPPED }

    private final DoubleSupplier measured;
    private final ClosedLoop closedLoop;

    private DoubleSupplier target = () -> 0;
    private DoubleSupplier tolerance = () -> 50;
    private DoubleSupplier dipThreshold = () -> 100;

    private State state = State.SPINNING_UP;
    private double leftBandTime;
    private double lowest;
    private double lastTarget;

    private int dips;
    private double lastDip = Double.NaN;
    private double lastRecovery = Double.NaN;
    private double recoverySum;

    public RecoveryStep(String name, DoubleSupplier measured, ClosedLoop closedLoop, String... instructions) {
        super(name, instructions);
        this.measured = measured;
        this.closedLoop = closedLoop;
    }

    public RecoveryStep target(DoubleSupplier target) {
        this.target = target;
        return this;
    }

    public RecoveryStep tolerance(DoubleSupplier tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    /** How far below the target counts as a shot (anything smaller is noise). */
    public RecoveryStep dipThreshold(DoubleSupplier dipThreshold) {
        this.dipThreshold = dipThreshold;
        return this;
    }

    @Override
    public void start(double now) {
        state = State.SPINNING_UP;
        dips = 0;
        lastDip = lastRecovery = Double.NaN;
        recoverySum = 0;
        lastTarget = target.getAsDouble();
    }

    /** True once at speed and waiting for a shot. */
    public boolean isReady() {
        return state == State.READY;
    }

    @Override
    public void loop(double now) {
        double goal = target.getAsDouble();
        if (goal != lastTarget) { // changed on Dashboard: wait to get there again
            state = State.SPINNING_UP;
            lastTarget = goal;
        }
        closedLoop.run(goal, 0);

        double m = measured.getAsDouble();
        if (Double.isNaN(m)) {
            return;
        }
        boolean inBand = Math.abs(m - goal) <= tolerance.getAsDouble();

        switch (state) {
            case SPINNING_UP:
                if (inBand) {
                    state = State.READY;
                }
                break;
            case READY:
                if (!inBand) {
                    state = State.OUT_OF_BAND;
                    leftBandTime = now;
                    lowest = m;
                }
                break;
            case OUT_OF_BAND:
            case DIPPED:
                lowest = Math.min(lowest, m);
                if (goal - lowest > dipThreshold.getAsDouble()) {
                    state = State.DIPPED;
                }
                if (inBand) {
                    if (state == State.DIPPED) {
                        dips++;
                        lastDip = goal - lowest;
                        lastRecovery = now - leftBandTime;
                        recoverySum += lastRecovery;
                    }
                    state = State.READY;
                }
                break;
        }
    }

    @Override
    public void report(Telemetry telemetry, boolean running) {
        if (running) {
            telemetry.addData("target", target.getAsDouble());
            telemetry.addData("measured", measured.getAsDouble());
            telemetry.addLine(state == State.READY ? "READY: shoot now"
                    : state == State.SPINNING_UP ? "Spinning up ..." : "Recovering ...");
        }
        telemetry.addLine("Shots measured: " + dips);
        telemetry.addLine("Last dip: " + fmt("%.0f", lastDip) + ", recovered in " + fmt("%.2f", lastRecovery) + " s");
        telemetry.addLine("Average recovery: " + fmt("%.2f", dips > 0 ? recoverySum / dips : Double.NaN) + " s");
    }
}
