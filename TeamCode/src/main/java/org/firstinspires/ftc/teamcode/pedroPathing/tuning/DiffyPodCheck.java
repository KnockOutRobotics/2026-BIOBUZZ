package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.DifferentialPod;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemConstants.Drive;

/**
 * Bench check for the differential pods, before any Pedro tuning. Robot on a stand.
 *
 * <ul>
 * <li>Direction first: turn a wheel CCW by hand (viewed from above). "Right/Left angle" should
 *     increase; if not, flip ENCODER_INCREASES_CCW.</li>
 * <li>Offsets: point both wheels forward by hand; "raw" is the value for
 *     RIGHT/LEFT_POD_ANGLE_OFFSET_DEGREES.</li>
 * <li>Steering: dpad up/left/down/right sends both pods to 0/90/180/-90 degrees (forward, left,
 *     back, right; 180 shows as 0 with the wheel driving backwards). If a pod spins away instead
 *     of settling, flip TURN_REVERSED. Then tune TURN_P/I/D/F.</li>
 * <li>Drive: hold right trigger. Both wheels should roll the way the pod points; if not, flip
 *     DRIVE_REVERSED.</li>
 * </ul>
 */
@TeleOp(name = "Diffy Pod Check", group = "Tuning")
public class DiffyPodCheck extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Offset 0 so getAngle() shows the raw angle to copy into the constants.
        DifferentialPod rightRaw = new DifferentialPod(hardwareMap, "rightRaw",
                Drive.RIGHT_POD_MOTOR_1, Drive.RIGHT_POD_MOTOR_2, Drive.RIGHT_POD_ENCODER, Vector2D.zero(), 0);
        DifferentialPod leftRaw = new DifferentialPod(hardwareMap, "leftRaw",
                Drive.LEFT_POD_MOTOR_1, Drive.LEFT_POD_MOTOR_2, Drive.LEFT_POD_ENCODER, Vector2D.zero(), 0);

        DifferentialPod right = new DifferentialPod(hardwareMap, "right",
                Drive.RIGHT_POD_MOTOR_1, Drive.RIGHT_POD_MOTOR_2, Drive.RIGHT_POD_ENCODER, Vector2D.zero(),
                Math.toRadians(Drive.RIGHT_POD_ANGLE_OFFSET_DEGREES));
        DifferentialPod left = new DifferentialPod(hardwareMap, "left",
                Drive.LEFT_POD_MOTOR_1, Drive.LEFT_POD_MOTOR_2, Drive.LEFT_POD_ENCODER, Vector2D.zero(),
                Math.toRadians(Drive.LEFT_POD_ANGLE_OFFSET_DEGREES));

        double targetDegrees = 0; // encoder space: CCW from forward
        boolean active = false;

        while (opModeInInit()) {
            showAngles(rightRaw, leftRaw, right, left, targetDegrees, false);
        }

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) { targetDegrees = 0; active = true; }
            if (gamepad1.dpad_left) { targetDegrees = 90; active = true; }
            if (gamepad1.dpad_down) { targetDegrees = 180; active = true; }
            if (gamepad1.dpad_right) { targetDegrees = -90; active = true; }

            if (active) {
                // move() takes wheel space (CCW from robot right), so add 90 degrees.
                double wheelTheta = Math.toRadians(targetDegrees) + Math.PI / 2;
                double drive = 0.3 * gamepad1.right_trigger;
                right.move(wheelTheta, drive, false);
                left.move(wheelTheta, drive, false);
            }

            showAngles(rightRaw, leftRaw, right, left, targetDegrees, active);
        }
    }

    private void showAngles(DifferentialPod rightRaw, DifferentialPod leftRaw,
                            DifferentialPod right, DifferentialPod left, double target, boolean active) {
        telemetry.addData("Right raw (offset)", "%.1f", Math.toDegrees(rightRaw.getAngle()));
        telemetry.addData("Left raw (offset)", "%.1f", Math.toDegrees(leftRaw.getAngle()));
        telemetry.addData("Right angle", "%.1f", Math.toDegrees(right.getAngle()));
        telemetry.addData("Left angle", "%.1f", Math.toDegrees(left.getAngle()));
        telemetry.addData("Target", active ? String.format("%.0f", target) : "press a dpad direction");
        telemetry.addData("Right", right.debug());
        telemetry.addData("Left", left.debug());
        telemetry.update();
    }
}
