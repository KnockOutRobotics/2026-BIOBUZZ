package org.firstinspires.ftc.teamcode.samples;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ivy.IvyOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/** Reference follower-driven teleop, on Ivy. Disabled until Constants can build a follower. */
@Disabled
@TeleOp
public class PedroTeleOpSample extends IvyOpMode {

    private Follower follower;

    @Override
    protected void onInit() {
        follower = Constants.createFollower(hardwareMap);

        // Ivy requirements are plain objects, so the follower itself is the requirement here --
        // it does not have to be a Subsystem. Any command that follows a path should require it
        // too, and then it cannot be fighting the sticks.
        setDefaultCommand(Command.build()
                .requiring(follower)
                .setExecute(() -> follower.manual(
                        // Field-centric drive. For robot-centric, use the three-argument
                        // follower.manual(-left_stick_y, -left_stick_x, -right_stick_x) instead.
                        ManualDrive.fieldCentric(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x,
                                follower.pose().heading()
                        )))
                .setDone(() -> false));

        // Pedro 3 has no startTeleopDrive(): the follower enters manual mode on the first
        // manual() call and leaves it when a path is followed.
    }

    @Override
    protected void onStart() {
        // The follower has to be updated every loop, whether it is following a path or being
        // driven. Requirement-free, and scheduled first, so it runs ahead of everything else.
        schedule(Commands.infinite(follower::update));
    }

    @Override
    protected void onLoop() {
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", follower.pose().heading());
    }
}
