package org.firstinspires.ftc.teamcode.OpMode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ivy.IvyOpMode;
import org.firstinspires.ftc.teamcode.subsystems.PedroDrive;

/**
 * Driver-controlled swerve through Pedro's follower. Starts at the pose auto ended on, so odometry
 * carries over; if there is none (the app restarted), it starts at the field center facing +x.
 *
 * <p>Field-centric "forward" starts along field +x. Press options/start with the robot facing
 * away from you to make that direction forward instead.
 */
@TeleOp(name = "Pedro Swerve TeleOp", group = "Main")
public class PedroSwerveTeleOp extends IvyOpMode {

    private static final Pose FALLBACK_START = new Pose(72, 72, 0);

    private PedroDrive drive;

    @Override
    protected void onInit() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = register(PedroDrive.continueFromLastPose(hardwareMap, FALLBACK_START));

        // Pedro's convention: forward, strafe left and turn CCW are positive.
        setDefaultCommand(drive.driveCommand(
                () -> -gamepad1.left_stick_y * SwerveTeleOpConfig.DRIVE_SPEED_SCALAR,
                () -> -gamepad1.left_stick_x * SwerveTeleOpConfig.DRIVE_SPEED_SCALAR,
                () -> -gamepad1.right_stick_x * SwerveTeleOpConfig.ROTATION_SPEED_SCALAR,
                () -> SwerveTeleOpConfig.FIELD_CENTRIC));
    }

    @Override
    protected void onStart() {
        bindings.whenPressed(
                () -> gamepad1.options || gamepad1.start,
                drive.resetDriverForwardCommand().then(Commands.instant(() -> gamepad1.rumble(500))));
    }

    @Override
    protected void onLoop() {
        Pose pose = drive.pose();
        telemetry.addData("X", pose.x());
        telemetry.addData("Y", pose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
    }
}
