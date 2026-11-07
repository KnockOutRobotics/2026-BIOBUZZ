package org.firstinspires.ftc.teamcode.OpMode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemConstants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.tuning.AngleUnwrapper;
import org.firstinspires.ftc.teamcode.util.tuning.FeedforwardSweepStep;
import org.firstinspires.ftc.teamcode.util.tuning.RateEstimator;
import org.firstinspires.ftc.teamcode.util.tuning.StaticFrictionStep;
import org.firstinspires.ftc.teamcode.util.tuning.StepResponseStep;
import org.firstinspires.ftc.teamcode.util.tuning.TrackingStep;

/**
 * Tunes the turret's PIDF + velocity feedforward ("Turret" on Dashboard). Run "Turret Check" first
 * so the encoder and directions are right. Procedure: TUNING.md, section Turret. Test settings are
 * "Turret Tuner" on Dashboard.
 */
@Config("Turret Tuner")
@TeleOp(name = "Turret PID Tuner", group = "Tuning")
public class TurretTuner extends PidTuningOpMode {

    // Step 1: deadband (F)
    public static double RAMP_RATE = 0.05; // power per second
    public static double MOVE_THRESHOLD = 5; // deg/s that counts as moving
    // Step 2: feedforward sweep (KV)
    public static double SWEEP_MIN_POWER = 0.15;
    public static double SWEEP_MAX_POWER = 0.8;
    public static int SWEEP_POINTS = 5;
    public static double SWEEP_SETTLE_SECONDS = 1.0;
    public static double SWEEP_MEASURE_SECONDS = 1.0;
    // Step 3: tracking (KV check with P at 0; later, everything together)
    public static double TRACK_CENTER = 0;
    public static double TRACK_AMPLITUDE = 90;
    public static double TRACK_PERIOD_SECONDS = 3.0;
    // Step 4: step response (P, D). Keep the step under 180 degrees (180 is ambiguous).
    public static double STEP_LOW = 0;
    public static double STEP_HIGH = 90;
    public static double STEP_PERIOD_SECONDS = 2.0;

    private Turret turret;
    private final AngleUnwrapper unwrapper = new AngleUnwrapper(360);
    private final RateEstimator rate = new RateEstimator(0.15);
    private double angle;
    private double velocity;

    @Override
    protected String title() {
        return "Turret PID Tuner";
    }

    @Override
    protected void initHardware() {
        turret = new Turret(hardwareMap);

        addStep(new StaticFrictionStep("1. Deadband (F)", "F", () -> velocity, turret::setOpenLoopPower,
                "Ramps power until the turret moves, both ways.",
                "Set Turret.F to the suggestion (a bit under if it",
                "buzzes at the target).")
                .rampRate(() -> RAMP_RATE)
                .moveThreshold(() -> MOVE_THRESHOLD)
                .maxPower(0.5));

        addStep(new FeedforwardSweepStep("2. Feedforward (KV)", "KV", () -> velocity, turret::setOpenLoopPower,
                "Spins the turret at several powers, both ways.",
                "Needs clear rotation. Set Turret.KV to the slope.")
                .powers(() -> SWEEP_MIN_POWER, () -> SWEEP_MAX_POWER, () -> SWEEP_POINTS)
                .timing(() -> SWEEP_SETTLE_SECONDS, () -> SWEEP_MEASURE_SECONDS)
                .moveThreshold(() -> MOVE_THRESHOLD)
                .bothDirections(true));

        addStep(new TrackingStep("3. Tracking (KV check)", () -> angle,
                (target, targetRate) -> {
                    turret.setTargetAngleDegrees(target, targetRate);
                    turret.periodic();
                },
                "Target sweeps back and forth with feedforward.",
                "Set P/I/D to 0, trim KV until amplitude ratio",
                "is ~1; then restore P/D, check RMS error.")
                .wave(() -> TRACK_CENTER, () -> TRACK_AMPLITUDE, () -> TRACK_PERIOD_SECONDS)
                .feedforwardGain("KV", () -> SubsystemConstants.Turret.KV)
                .wrapPeriod(360));
        addStep(new StepResponseStep("4. Step response (P, D)", () -> angle,
                (target, targetRate) -> {
                    turret.setTargetAngleDegrees(target, targetRate);
                    turret.periodic();
                },
                "Flips between STEP_LOW and STEP_HIGH.",
                "Raise P until it overshoots, add D to stop it,",
                "repeat. I only for a steady error F can't fix.")
                .targets(() -> STEP_LOW, () -> STEP_HIGH)
                .period(() -> STEP_PERIOD_SECONDS)
                .tolerance(() -> SubsystemConstants.Turret.TOLERANCE_DEGREES)
                .wrapPeriod(360));

    }

    @Override
    protected void updateSensors(double now) {
        angle = turret.getCurrentAngleDegrees();
        rate.add(now, unwrapper.update(angle));
        velocity = rate.rate();
    }

    @Override
    protected void stopOutputs() {
        turret.setOpenLoopPower(0);
    }

    @Override
    protected void addStatus(Telemetry telemetry) {
        telemetry.addData("angle", angle);
        telemetry.addData("velocity (deg/s)", velocity);
        telemetry.addData("turret power", turret.getPowerSetpoint());
        telemetry.addLine(String.format("Gains: P %.4f  I %.4f  D %.4f  F %.3f  KV %.5f",
                SubsystemConstants.Turret.P, SubsystemConstants.Turret.I, SubsystemConstants.Turret.D,
                SubsystemConstants.Turret.F, SubsystemConstants.Turret.KV));
    }
}
