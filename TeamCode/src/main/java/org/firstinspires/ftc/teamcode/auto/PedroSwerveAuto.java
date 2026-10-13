package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ivy.IvyOpMode;
import org.firstinspires.ftc.teamcode.subsystems.PedroDrive;

/**
 * Starting point for autonomous on the differential swerve: drives out 24 inches, comes back,
 * and holds the start pose. Replace the poses and routine with the real one; see
 * samples/PedroAutoSample for a fuller example. The final pose carries over into
 * {@code PedroSwerveTeleOp}.
 */
@Autonomous(name = "Pedro Swerve Auto", group = "Main")
public class PedroSwerveAuto extends IvyOpMode {

    private final Pose startPose = new Pose(72, 72, 0);
    private final Pose outPose = new Pose(96, 72, 0);

    private PedroDrive drive;
    private Path out;
    private Path back;

    @Override
    protected void onInit() {
        drive = register(new PedroDrive(hardwareMap, startPose));

        out = Paths.line(startPose, outPose).linear(startPose.heading(), outPose.heading());
        back = Paths.line(outPose, startPose).linear(outPose.heading(), startPose.heading());
    }

    @Override
    protected void onStart() {
        schedule(sequential(
                drive.followCommand(out),
                waitMs(500),
                drive.followCommand(back),
                drive.holdCommand(startPose)
        ));
    }

    @Override
    protected void onLoop() {
        Pose pose = drive.pose();
        telemetry.addData("X", pose.x());
        telemetry.addData("Y", pose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
    }
}
