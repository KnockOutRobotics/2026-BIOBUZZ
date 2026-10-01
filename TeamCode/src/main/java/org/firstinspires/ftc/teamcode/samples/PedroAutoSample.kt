package org.firstinspires.ftc.teamcode.samples

import com.pedropathing.api.Paths
import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.WaitCommand
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand
import com.seattlesolvers.solverslib.util.TelemetryData

import org.firstinspires.ftc.teamcode.pedroPathing.Constants

/** Reference autonomous structure. Disabled until Constants can build a follower. */
@Disabled
@Autonomous
class PedroAutoSample : CommandOpMode() {

    private lateinit var follower: Follower
    var telemetryData = TelemetryData(telemetry)

    // Poses
    private val startPose = Pose(9.0, 111.0, Math.toRadians(-90.0))
    private val scorePose = Pose(16.0, 128.0, Math.toRadians(-45.0))
    private val pickup1Pose = Pose(30.0, 121.0, Math.toRadians(0.0))
    private val pickup2Pose = Pose(30.0, 131.0, Math.toRadians(0.0))
    private val pickup3Pose = Pose(45.0, 128.0, Math.toRadians(90.0))
    private val parkPose = Pose(68.0, 96.0, Math.toRadians(-90.0))

    // Paths
    private lateinit var scorePreload: Path
    private lateinit var grabPickup1: Path
    private lateinit var grabPickup2: Path
    private lateinit var grabPickup3: Path
    private lateinit var scorePickup1: Path
    private lateinit var scorePickup2: Path
    private lateinit var scorePickup3: Path
    private lateinit var park: Path

    fun buildPaths() {
        scorePreload = Paths.line(startPose, scorePose)
            .linear(startPose.heading(), scorePose.heading())

        grabPickup1 = Paths.line(scorePose, pickup1Pose)
            .linear(scorePose.heading(), pickup1Pose.heading())

        scorePickup1 = Paths.line(pickup1Pose, scorePose)
            .linear(pickup1Pose.heading(), scorePose.heading())

        grabPickup2 = Paths.line(scorePose, pickup2Pose)
            .linear(scorePose.heading(), pickup2Pose.heading())

        scorePickup2 = Paths.line(pickup2Pose, scorePose)
            .linear(pickup2Pose.heading(), scorePose.heading())

        grabPickup3 = Paths.line(scorePose, pickup3Pose)
            .linear(scorePose.heading(), pickup3Pose.heading())

        scorePickup3 = Paths.line(pickup3Pose, scorePose)
            .linear(pickup3Pose.heading(), scorePose.heading())

        park = Paths.curve(
            scorePose,
            Pose(68.0, 110.0), // Control point
            parkPose
        ).linear(scorePose.heading(), parkPose.heading())
    }

    // Mechanism commands - replace these with your actual subsystem commands
    private fun openOuttakeClaw() = InstantCommand(Runnable {
        // Example: outtakeSubsystem.openClaw();
    })

    private fun grabSample() = InstantCommand(Runnable {
        // Example: intakeSubsystem.grabSample();
    })

    private fun scoreSample() = InstantCommand(Runnable {
        // Example: outtakeSubsystem.scoreSample();
    })

    private fun level1Ascent() = InstantCommand(Runnable {
        // Example: hangSubsystem.level1Ascent();
    })

    override fun initialize() {
        super.reset()

        // Initialize follower
        follower = Constants.createFollower(hardwareMap)
        follower.setPose(startPose)
        buildPaths()

        // Schedule the autonomous sequence
        schedule(
            // Score preload
            FollowPathCommand(follower, scorePreload),
            openOuttakeClaw(),
            WaitCommand(1000), // Wait 1 second

            // First pickup cycle
            // Sets globalMaxPower to 50% for all future paths
            // (unless a custom maxPower is given)
            FollowPathCommand(follower, grabPickup1).setGlobalMaxPower(0.5),
            grabSample(),
            FollowPathCommand(follower, scorePickup1),
            scoreSample(),

            // Second pickup cycle
            FollowPathCommand(follower, grabPickup2),
            grabSample(),
            FollowPathCommand(follower, scorePickup2, 1.0), // Overrides maxPower to 100% for this path only
            scoreSample(),

            // Third pickup cycle
            FollowPathCommand(follower, grabPickup3),
            grabSample(),
            FollowPathCommand(follower, scorePickup3),
            scoreSample(),

            // Park
            FollowPathCommand(follower, park, false), // park with holdEnd false
            level1Ascent()
        )
    }

    override fun run() {
        super.run()
        follower.update()

        telemetryData.addData("X", follower.pose().x())
        telemetryData.addData("Y", follower.pose().y())
        telemetryData.addData("Heading", follower.pose().heading())
        telemetryData.update()
    }
}
