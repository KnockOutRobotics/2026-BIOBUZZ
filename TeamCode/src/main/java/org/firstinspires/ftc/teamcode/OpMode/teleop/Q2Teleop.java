package org.firstinspires.ftc.teamcode.OpMode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ivy.IvyOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PedroDrive;

/**
 * Qualifier 2 competition TeleOp. Subsystems are added here as they are finished.
 *
 * <p>Drives through {@link PedroDrive}, so it starts at the pose auto ended on (or the field
 * center facing +x if there is none) and the pose is available for aiming.
 *
 * <p>Controls (gamepad1):
 * <ul>
 * <li>Left stick: drive. Right stick x: turn.</li>
 * <li>Options/start: make the robot's current facing field-centric "forward".</li>
 * <li>Right bumper (hold): nectar intake.</li>
 * <li>Left bumper (hold): pollen intake.</li>
 * </ul>
 */
@TeleOp(name = "Q2 TeleOp", group = "Main")
public class Q2Teleop extends IvyOpMode {

    private static final Pose FALLBACK_START = new Pose(72, 72, 0);

    private PedroDrive drive;
    private Intake intake;

    @Override
    protected void onInit() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = register(PedroDrive.continueFromLastPose(hardwareMap, FALLBACK_START));
        intake = register(new Intake(hardwareMap));

        // Pedro's convention: forward, strafe left and turn CCW are positive.
        setDefaultCommand(drive.driveCommand(
                () -> -gamepad1.left_stick_y * DriveConfig.DRIVE_SPEED_SCALAR,
                () -> -gamepad1.left_stick_x * DriveConfig.DRIVE_SPEED_SCALAR,
                () -> -gamepad1.right_stick_x * DriveConfig.ROTATION_SPEED_SCALAR,
                () -> DriveConfig.FIELD_CENTRIC));
    }

    @Override
    protected void onStart() {
        bindings.whenPressed(
                        () -> gamepad1.options || gamepad1.start,
                        drive.resetDriverForwardCommand().then(Commands.instant(() -> gamepad1.rumble(500))))
                .whileHeld(() -> gamepad1.right_bumper, intake.runNectar())
                .whileHeld(() -> gamepad1.left_bumper, intake.runPollen());
    }

    @Override
    protected void onLoop() {
        Pose pose = drive.pose();
        telemetry.addData("X", pose.x());
        telemetry.addData("Y", pose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
    }
}
