package org.firstinspires.ftc.teamcode.util.tuning;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * One step of a tuning procedure, run by {@code OpMode.tuning.PidTuningOpMode}: started with A,
 * looped every cycle until it finishes or B stops it. Results stay up after it stops, until it is
 * started again.
 */
public abstract class TuningStep {

    private final String name;
    private final String[] instructions;

    protected TuningStep(String name, String... instructions) {
        this.name = name;
        this.instructions = instructions;
    }

    public String name() {
        return name;
    }

    /** What to do, and which gain to change, shown while the step is selected. */
    public String[] instructions() {
        return instructions;
    }

    /** Clears old results and begins. */
    public abstract void start(double now);

    /** Drives the mechanism for one cycle. */
    public abstract void loop(double now);

    /** True when the step has finished on its own; the OpMode then turns the mechanism off. */
    public boolean isDone() {
        return false;
    }

    /** Called once when the step finishes or is stopped. */
    public void end() {
    }

    /** Adds results (and live values while running) to telemetry. */
    public abstract void report(Telemetry telemetry, boolean running);

    /** Formats a number, showing "--" for NaN. */
    protected static String fmt(String format, double value) {
        return Double.isNaN(value) ? "--" : String.format(format, value);
    }
}
