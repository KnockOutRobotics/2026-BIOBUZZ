package org.firstinspires.ftc.teamcode.OpMode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.DifferentialPod;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemConstants.Drive;
import org.firstinspires.ftc.teamcode.util.tuning.AngleUnwrapper;
import org.firstinspires.ftc.teamcode.util.tuning.RateEstimator;
import org.firstinspires.ftc.teamcode.util.tuning.StaticFrictionStep;
import org.firstinspires.ftc.teamcode.util.tuning.StepResponseStep;

/**
 * Tunes the swerve pods' steering PIDF (TURN_* in "Drive" on Dashboard), one pod at a time; both
 * pods share the gains. Run "Diffy Pod Check" first so the offsets and directions are right.
 * Procedure: TUNING.md, section Drive. Test settings are "Drive Pod Tuner" on Dashboard.
 *
 * <p>Left/right bumper picks the pod (while no step runs); the other pod stays limp.
 */
@Config("Drive Pod Tuner")
@TeleOp(name = "Drive Pod PID Tuner", group = "Tuning")
public class DrivePodTuner extends PidTuningOpMode {

    // Step 1: static friction (TURN_F)
    public static double RAMP_RATE = 0.1; // power per second
    public static double MOVE_THRESHOLD = 10; // deg/s that counts as moving
    // Step 2: step response (TURN_P, TURN_D), encoder degrees CCW from forward. Keep the step
    // under 90 degrees: past that the pod flips and drives backward instead of steering.
    public static double STEP_LOW = -40;
    public static double STEP_HIGH = 40;
    public static double STEP_PERIOD_SECONDS = 1.0;
    // Right trigger drives the wheel at up to this power during the step response (robot on a stand)
    public static double DRIVE_POWER = 0.5;

    private final DifferentialPod[] pods = new DifferentialPod[2];
    private final AngleUnwrapper[] unwrappers = {new AngleUnwrapper(360), new AngleUnwrapper(360)};
    private final RateEstimator[] rates = {new RateEstimator(0.1), new RateEstimator(0.1)};
    private final double[] angles = new double[2];
    private final double[] velocities = new double[2];
    private int pod = 0; // 0 = right, 1 = left

    @Override
    protected String title() {
        return "Drive Pod PID Tuner";
    }

    @Override
    protected void initHardware() {
        // Pod positions at zero, so the tuner works before POD_X/Y_INCHES are measured.
        pods[0] = new DifferentialPod(hardwareMap, "right",
                Drive.RIGHT_POD_MOTOR_1, Drive.RIGHT_POD_MOTOR_2, Drive.RIGHT_POD_ENCODER, Vector2D.zero(),
                Math.toRadians(Drive.RIGHT_POD_ANGLE_OFFSET_DEGREES));
        pods[1] = new DifferentialPod(hardwareMap, "left",
                Drive.LEFT_POD_MOTOR_1, Drive.LEFT_POD_MOTOR_2, Drive.LEFT_POD_ENCODER, Vector2D.zero(),
                Math.toRadians(Drive.LEFT_POD_ANGLE_OFFSET_DEGREES));

        addStep(new StaticFrictionStep("1. Static friction (TURN_F)", "TURN_F",
                () -> velocities[pod], power -> pods[pod].setOpenLoop(power, 0),
                "Ramps steering power until the pod turns, both",
                "ways. Robot on the floor (real scrub). Do both",
                "pods; set Drive.TURN_F to the lower result.")
                .rampRate(() -> RAMP_RATE)
                .moveThreshold(() -> MOVE_THRESHOLD)
                .maxPower(0.6));

        addStep(new StepResponseStep("2. Step response (TURN_P, TURN_D)", () -> angles[pod],
                (target, targetRate) -> pods[pod].move(Math.toRadians(target) + Math.PI / 2,
                        DRIVE_POWER * gamepad1.right_trigger, false),
                "Flips the pod between STEP_LOW and STEP_HIGH.",
                "Raise TURN_P until it overshoots, add TURN_D,",
                "repeat. Floor first; then on a stand, hold RT to",
                "check it still steers while driving.")
                .targets(() -> STEP_LOW, () -> STEP_HIGH)
                .period(() -> STEP_PERIOD_SECONDS)
                .tolerance(() -> Math.toDegrees(Drive.TURN_TOLERANCE_RADIANS))
                .wrapPeriod(180)); // the pod may point either way and drive backward
    }

    @Override
    protected void updateSensors(double now) {
        for (int i = 0; i < 2; i++) {
            angles[i] = Math.toDegrees(pods[i].getAngle());
            rates[i].add(now, unwrappers[i].update(angles[i]));
            velocities[i] = rates[i].rate();
        }
    }

    @Override
    protected void stopOutputs() {
        pods[0].setOpenLoop(0, 0);
        pods[1].setOpenLoop(0, 0);
    }

    @Override
    protected void extraControls(double now) {
        if (!isStepRunning()) {
            if (gamepad1.right_bumper) {
                pod = 0;
            } else if (gamepad1.left_bumper) {
                pod = 1;
            }
        } else {
            pods[1 - pod].setOpenLoop(0, 0); // the pod not being tuned stays limp
        }
    }

    @Override
    protected void addStatus(Telemetry telemetry) {
        telemetry.addLine("Pod: " + (pod == 0 ? "RIGHT" : "LEFT") + " (bumpers to change)");
        telemetry.addData("pod angle", angles[pod]);
        telemetry.addData("pod velocity (deg/s)", velocities[pod]);
        telemetry.addData("right", pods[0].debug());
        telemetry.addData("left", pods[1].debug());
        telemetry.addLine(String.format("Gains: TURN_P %.3f  I %.3f  D %.4f  F %.3f",
                Drive.TURN_P, Drive.TURN_I, Drive.TURN_D, Drive.TURN_F));
    }
}
