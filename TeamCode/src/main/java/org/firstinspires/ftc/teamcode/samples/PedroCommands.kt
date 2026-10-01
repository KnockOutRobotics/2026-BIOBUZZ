package org.firstinspires.ftc.teamcode.samples

import com.pedropathing.api.Paths
import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand
import com.seattlesolvers.solverslib.pedroCommand.HoldPointCommand
import com.seattlesolvers.solverslib.pedroCommand.TurnCommand
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

/** Reference for the SolversLib Pedro commands. Disabled until Constants can build a follower. */
@Disabled
@Autonomous
class PedroCommands : CommandOpMode() {

    lateinit var follower: Follower

    var pose = Pose(72.0, 72.0, 90.0)

    lateinit var path: Path

    override fun initialize() {
        super.reset()

        follower = Constants.createFollower(hardwareMap)

        path = Paths.line(
            Pose(0.0, 0.0, Math.toRadians(0.0)),
            Pose(16.0, 28.0, Math.toRadians(90.0))
        ).linear(Math.toRadians(0.0), Math.toRadians(90.0))

        schedule(
            // Updates follower to follow path
            RunCommand(Runnable { follower.update() }),

            // HoldPointCommand
            HoldPointCommand(follower, Pose(0.0, 4.0, 0.0), false),
            HoldPointCommand(follower, pose, true),

            // TurnCommand
            TurnCommand(follower, Math.PI / 2, false),
            TurnCommand(follower, 90.0, true, AngleUnit.DEGREES),

            // TurnToCommand
            TurnToCommand(follower, Math.PI / 2),
            TurnToCommand(follower, 90.0, AngleUnit.DEGREES),

            // FollowPathCommand
            FollowPathCommand(follower, path),
            FollowPathCommand(follower, path, true),
            FollowPathCommand(follower, path, true, 1.0),
            FollowPathCommand(follower, path, true, 1.0).setGlobalMaxPower(1.0)
        )
    }

    override fun run() {
        super.run()
    }
}
