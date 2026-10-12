package org.firstinspires.ftc.teamcode.samples;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.ivy.IvyOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/** Reference autonomous structure, on Ivy. Disabled until Constants can build a follower. */
@Disabled
@Autonomous
public class PedroAutoSample extends IvyOpMode {

    private Follower follower;

    // Poses
    private final Pose startPose = new Pose(9, 111, Math.toRadians(-90));
    private final Pose scorePose = new Pose(16, 128, Math.toRadians(-45));
    private final Pose pickup1Pose = new Pose(30, 121, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(30, 131, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(45, 128, Math.toRadians(90));
    private final Pose parkPose = new Pose(68, 96, Math.toRadians(-90));

    // Paths
    private Path scorePreload;
    private Path grabPickup1;
    private Path grabPickup2;
    private Path grabPickup3;
    private Path scorePickup1;
    private Path scorePickup2;
    private Path scorePickup3;
    private Path park;

    public void buildPaths() {
        scorePreload = Paths.line(startPose, scorePose)
                .linear(startPose.heading(), scorePose.heading());

        grabPickup1 = Paths.line(scorePose, pickup1Pose)
                .linear(scorePose.heading(), pickup1Pose.heading());

        scorePickup1 = Paths.line(pickup1Pose, scorePose)
                .linear(pickup1Pose.heading(), scorePose.heading());

        grabPickup2 = Paths.line(scorePose, pickup2Pose)
                .linear(scorePose.heading(), pickup2Pose.heading());

        scorePickup2 = Paths.line(pickup2Pose, scorePose)
                .linear(pickup2Pose.heading(), scorePose.heading());

        grabPickup3 = Paths.line(scorePose, pickup3Pose)
                .linear(scorePose.heading(), pickup3Pose.heading());

        scorePickup3 = Paths.line(pickup3Pose, scorePose)
                .linear(pickup3Pose.heading(), scorePose.heading());

        park = Paths.curve(
                scorePose,
                new Pose(68, 110), // Control point
                parkPose
        ).linear(scorePose.heading(), parkPose.heading());
    }

    // Mechanism commands - replace these with your actual subsystem commands.
    // A real one is a factory on the subsystem, e.g. turret.goToAngle(90), so that it carries its
    // requirements and two routines can never drive the same hardware at once.
    private Command openOuttakeClaw() {
        return instant(() -> {
            // Example: outtakeSubsystem.openClaw();
        });
    }

    private Command grabSample() {
        return instant(() -> {
            // Example: intakeSubsystem.grabSample();
        });
    }

    private Command scoreSample() {
        return instant(() -> {
            // Example: outtakeSubsystem.scoreSample();
        });
    }

    private Command level1Ascent() {
        return instant(() -> {
            // Example: hangSubsystem.level1Ascent();
        });
    }

    @Override
    protected void onInit() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        buildPaths();
    }

    @Override
    protected void onStart() {
        // The follower has to be updated every loop. Requirement-free, and scheduled before the
        // routine, so it runs ahead of the path commands each loop.
        schedule(Commands.infinite(follower::update));

        // sequential() runs one command at a time, in order, and finishes with the last one.
        // Swap in parallel()/race()/deadline() where a mechanism should move while driving --
        // e.g. follow(follower, grabPickup1).with(intake.run()).
        schedule(sequential(
                // Score preload
                follow(follower, scorePreload),
                openOuttakeClaw(),
                waitMs(1000), // Wait 1 second

                // First pickup cycle
                follow(follower, grabPickup1),
                grabSample(),
                follow(follower, scorePickup1),
                scoreSample(),

                // Second pickup cycle
                follow(follower, grabPickup2),
                grabSample(),
                follow(follower, scorePickup2),
                scoreSample(),

                // Third pickup cycle
                follow(follower, grabPickup3),
                grabSample(),
                follow(follower, scorePickup3),
                scoreSample(),

                // Park, then sit on the end pose instead of coasting off it
                follow(follower, park),
                hold(follower, parkPose),
                level1Ascent()
        ));

        // Pedro 2's per-path maxPower/holdEnd arguments are gone: speed limits live in the
        // Algorithm config in Constants, and follower.holdEnd.set(...) decides whether a finished
        // path is held. hold(follower, pose) above is the explicit version of holding a point.
    }

    @Override
    protected void onLoop() {
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", follower.pose().heading());
    }
}
