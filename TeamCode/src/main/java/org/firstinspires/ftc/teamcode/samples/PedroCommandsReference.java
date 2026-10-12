package org.firstinspires.ftc.teamcode.samples;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ivy.IvyOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Reference for the Ivy Pedro commands and the composition operators, replacing the SolversLib
 * {@code FollowPathCommand}/{@code HoldPointCommand}/{@code TurnCommand} reference this file used
 * to hold. Disabled until Constants can build a follower.
 *
 * <p>{@code com.pedropathing.ivy.pedro.PedroCommands} is deliberately small -- {@code follow} and
 * {@code hold} -- because everything else is composition: Ivy's groups and decorators cover what
 * the per-command flags used to do.
 */
@Disabled
@Autonomous
public class PedroCommandsReference extends IvyOpMode {

    private Follower follower;

    private final Pose pose = new Pose(72, 72, Math.toRadians(90));

    private Path path;

    @Override
    protected void onInit() {
        follower = Constants.createFollower(hardwareMap);

        path = Paths.line(
                new Pose(0, 0, Math.toRadians(0)),
                new Pose(16, 28, Math.toRadians(90))
        ).linear(Math.toRadians(0), Math.toRadians(90));
    }

    @Override
    protected void onStart() {
        // Every Pedro command needs the follower updated each loop. Requirement-free, so it keeps
        // running underneath whatever is driving.
        schedule(Commands.infinite(follower::update));

        schedule(sequential(
                // follow(): drives the path, finishes at the parametric end of it
                follow(follower, path),

                // hold(): pins the robot to a pose. Instant -- it only puts the follower in
                // hold mode -- so pair it with a wait or a condition to sit there for a while.
                hold(follower, pose).then(waitMs(500)),

                // hold(follower) with no pose reads follower.pose() when the command is *built*,
                // which in a routine like this one is at START. lazy() defers the build to the
                // moment the command runs, which is what "hold wherever I am now" means.
                Commands.lazy(() -> hold(follower)).then(waitMs(500)),

                // Turning in place: there is no TurnCommand in Pedro 3, a turn is a hold at the
                // same point with a new heading.
                turnTo(Math.PI / 2),
                turnTo(Math.toRadians(90)),

                // ---- composition, instead of per-command flags ----

                // Run a mechanism while driving (what a parallel/deadline group is for)
                follow(follower, path).with(instant(() -> {
                    // Example: intake.run()
                })),

                // Give a path a deadline: whichever finishes first ends the race
                follow(follower, path).raceWith(waitMs(3000)),

                // Or an abort condition
                follow(follower, path).until(() -> gamepad1.b),

                // Or skip it entirely. Written out with conditional() rather than the
                // decorator .unless(): in Ivy 1.1.1 that decorator falls back to Command.NOOP,
                // which is an empty builder whose done() is always false, so it would hang the
                // sequence instead of skipping a step.
                Commands.conditional(
                        () -> gamepad1.x,
                        instant(() -> {
                            // skipped
                        }),
                        follow(follower, path))
        ));

        // Requirements and priorities are what keep two routines off the same hardware:
        //   Command.build().requiring(follower).setPriority(1)...
        // A higher-priority command interrupts a lower one; equal priorities override by default.
    }

    /**
     * Holds the current position at a new heading, finishing once the robot is within 2 degrees.
     * lazy() again: the pose to hold is the one we are at when the turn starts, not when the
     * routine was built.
     */
    private Command turnTo(double headingRadians) {
        return Commands.lazy(() -> hold(follower, new Pose(
                        follower.pose().x(),
                        follower.pose().y(),
                        headingRadians)))
                .then(waitUntil(() -> Math.abs(AngleUnit.normalizeRadians(
                        headingRadians - follower.pose().heading())) < Math.toRadians(2)));
    }

    @Override
    protected void onLoop() {
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", follower.pose().heading());
    }
}
