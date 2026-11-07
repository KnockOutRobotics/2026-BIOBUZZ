package org.firstinspires.ftc.teamcode.OpMode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemConstants;
import org.firstinspires.ftc.teamcode.util.VelocityController;
import org.firstinspires.ftc.teamcode.util.tuning.FeedforwardSweepStep;
import org.firstinspires.ftc.teamcode.util.tuning.RecoveryStep;
import org.firstinspires.ftc.teamcode.util.tuning.StepResponseStep;

/**
 * Tunes the flywheel's velocity controller ("Launcher" on Dashboard). Procedure: TUNING.md,
 * section Launcher. Test settings are "Launcher Tuner" on Dashboard.
 *
 * <p>The controller is voltage-compensated, so the feedforward sweep reports power scaled to 12 V:
 * the numbers go straight into KS / KV whatever the battery is at. KS comes from the sweep's
 * intercept (friction while spinning), not a breakaway ramp: VelocityController adds KS whenever
 * the target is nonzero, i.e. while the wheel is already turning.
 *
 * <p>Hold right bumper to run the x-wheels at FEED_POWER (to feed shots for step 3).
 */
@Config("Launcher Tuner")
@TeleOp(name = "Launcher PID Tuner", group = "Tuning")
public class LauncherTuner extends PidTuningOpMode {

    // Step 1: feedforward sweep (KS, KV)
    public static double MOVE_THRESHOLD = 30; // ticks/s; slower sweep points are left out
    public static double SWEEP_MIN_POWER = 0.2;
    public static double SWEEP_MAX_POWER = 1.0;
    public static int SWEEP_POINTS = 5;
    public static double SWEEP_SETTLE_SECONDS = 2.0;
    public static double SWEEP_MEASURE_SECONDS = 1.0;
    // Step 2: spin-up (KP)
    public static double STEP_LOW = SubsystemConstants.Launcher.IDLE_VELOCITY;
    public static double STEP_HIGH = SubsystemConstants.Launcher.POLLEN_VELOCITY;
    public static double STEP_PERIOD_SECONDS = 3.0;
    // Step 3: shot recovery (KP, KI)
    public static double RECOVERY_VELOCITY = SubsystemConstants.Launcher.POLLEN_VELOCITY;
    public static double DIP_THRESHOLD = 100; // ticks/s below target that counts as a shot
    public static double FEED_POWER = 1.0; // flip the sign if the x-wheels feed backward

    private Launcher launcher;
    private double velocity;

    @Override
    protected String title() {
        return "Launcher PID Tuner";
    }

    @Override
    protected void initHardware() {
        launcher = new Launcher(hardwareMap);

        addStep(new FeedforwardSweepStep("1. Feedforward (KS, KV)", "KV", () -> velocity, launcher::setFlywheelOpenLoop,
                "Runs the flywheel at several powers.",
                "Set Launcher.KS to the intercept and KV to the",
                "slope. Full battery; R^2 should be near 1.")
                .interceptGain("KS")
                .powers(() -> SWEEP_MIN_POWER, () -> SWEEP_MAX_POWER, () -> SWEEP_POINTS)
                .timing(() -> SWEEP_SETTLE_SECONDS, () -> SWEEP_MEASURE_SECONDS)
                .moveThreshold(() -> MOVE_THRESHOLD)
                .powerScale(this::voltageScale));

        addStep(new StepResponseStep("2. Spin-up (KP)", () -> velocity,
                (target, targetRate) -> {
                    launcher.setFlywheelVelocity(target);
                    launcher.periodic();
                },
                "Flips between STEP_LOW and STEP_HIGH ticks/s.",
                "Up steps: raise KP until it overshoots, then",
                "back off. Down steps coast; ignore their speed.")
                .targets(() -> STEP_LOW, () -> STEP_HIGH)
                .period(() -> STEP_PERIOD_SECONDS)
                .tolerance(() -> SubsystemConstants.Launcher.TOLERANCE));

        addStep(new RecoveryStep("3. Shot recovery (KP, KI)", () -> velocity,
                (target, targetRate) -> {
                    launcher.setFlywheelVelocity(target);
                    launcher.periodic();
                },
                "Holds RECOVERY_VELOCITY. Shoot (hold RB to feed)",
                "when READY. Raise KP for faster recovery; add",
                "KI only if it settles short of the target.")
                .target(() -> RECOVERY_VELOCITY)
                .tolerance(() -> SubsystemConstants.Launcher.TOLERANCE)
                .dipThreshold(() -> DIP_THRESHOLD));
    }

    /** Raw power to 12 V-equivalent power, matching VelocityController's compensation. */
    private double voltageScale() {
        return launcher.getBatteryVoltage() / VelocityController.NOMINAL_VOLTAGE;
    }

    @Override
    protected void updateSensors(double now) {
        velocity = launcher.getFlywheelVelocity();
    }

    @Override
    protected void stopOutputs() {
        launcher.setFlywheelOpenLoop(0);
    }

    @Override
    protected void extraControls(double now) {
        launcher.setXWheelPower(gamepad1.right_bumper ? FEED_POWER : 0);
    }

    @Override
    protected void addStatus(Telemetry telemetry) {
        telemetry.addData("flywheel velocity", velocity);
        telemetry.addData("flywheel power", launcher.getFlywheelPower());
        telemetry.addData("battery (V)", launcher.getBatteryVoltage());
        telemetry.addLine("At speed: " + launcher.isFlywheelAtSpeed());
        telemetry.addLine(String.format("Gains: KS %.4f  KV %.6f  KP %.5f  KI %.5f",
                SubsystemConstants.Launcher.KS, SubsystemConstants.Launcher.KV,
                SubsystemConstants.Launcher.KP, SubsystemConstants.Launcher.KI));
    }
}
