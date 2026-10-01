package org.firstinspires.ftc.teamcode.samples

import com.pedropathing.follower.Follower
import com.pedropathing.follower.ManualDrive
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.util.TelemetryData

import org.firstinspires.ftc.teamcode.pedroPathing.Constants

/** Reference follower-driven teleop. Disabled until Constants can build a follower. */
@Disabled
@TeleOp
class PedroTeleOpSample : CommandOpMode() {

    lateinit var follower: Follower
    var telemetryData = TelemetryData(telemetry)

    override fun initialize() {
        follower = Constants.createFollower(hardwareMap)
        super.reset()

        // Pedro 3 has no startTeleopDrive(): the follower enters manual mode on the first
        // manual() call and leaves it when a path is followed.
    }

    override fun run() {
        super.run()

        /* Robot-Centric Drive
        follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        */

        // Field-Centric Drive
        follower.manual(
            ManualDrive.fieldCentric(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble(),
                follower.pose().heading()
            )
        )
        follower.update()

        telemetryData.addData("X", follower.pose().x())
        telemetryData.addData("Y", follower.pose().y())
        telemetryData.addData("Heading", follower.pose().heading())
        telemetryData.update()
    }
}
