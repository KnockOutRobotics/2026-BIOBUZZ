package org.firstinspires.ftc.teamcode.pedroPathing

import com.pedropathing.algorithm.Algorithm
import com.pedropathing.drivetrain.Drivetrain
import com.pedropathing.follower.Follower
import com.pedropathing.localization.Localizer
import com.pedropathing.revhub.localizers.PinpointConfig
import com.pedropathing.revhub.localizers.PinpointLocalizer
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.HardwareMap

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

/**
 * Pedro Pathing configuration.
 *
 * Pedro 3 builds a [Follower] from three pieces -- a [Localizer], a [Drivetrain] and an
 * [Algorithm] -- instead of the 2.x `FollowerBuilder`/`FollowerConstants` pair this file
 * used to hold.
 *
 * The localizer below is complete: the values match the Pinpoint setup in `SwerveTeleOp`.
 * The other two pieces are robot-specific and still outstanding:
 *
 * - **Drivetrain** -- `com.pedropathing.revhub.drivetrains.Swerve` drives coaxial pods, so our
 *   differential modules need a `SwervePod` implementation (the diffy mixing already lives in
 *   `mathsOperations.diffyConvert`) or a [Drivetrain] wrapper around
 *   [org.firstinspires.ftc.teamcode.subsystems.SwerveDrive]. This also needs the module
 *   positions on the chassis, which aren't recorded anywhere yet.
 * - **Algorithm** -- `ForesightConfig` has 12 required values (controllers, max velocities,
 *   accelerations) with no defaults. Those come out of the Pedro tuning OpModes that the
 *   `com.pedropathing:tuning` dependency registers; they are deliberately not guessed here.
 */
object Constants {

    /** Pinpoint odometry computer, configured as it is in `SwerveTeleOp`. */
    @JvmStatic
    fun createLocalizer(hardwareMap: HardwareMap): Localizer {
        return PinpointLocalizer(hardwareMap, PinpointConfig { config ->
            config.name.set("odo")
            config.xPodOffset.set(-170.5)
            config.yPodOffset.set(42.023)
            config.offsetUnits.set(DistanceUnit.MM)
            config.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            config.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            config.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED)
        })
    }

    /** Builds a follower on our Pinpoint localizer with a caller-supplied drivetrain and algorithm. */
    @JvmStatic
    fun createFollower(hardwareMap: HardwareMap, drivetrain: Drivetrain, algorithm: Algorithm): Follower {
        return Follower(createLocalizer(hardwareMap), drivetrain, algorithm)
    }

    /**
     * @throws UnsupportedOperationException always -- see the class comment. Kept so the reference
     *         OpModes in `samples` still show the intended call shape; fail loudly on INIT
     *         rather than NPE partway through a path.
     */
    @JvmStatic
    fun createFollower(hardwareMap: HardwareMap): Follower {
        throw UnsupportedOperationException(
            "Pedro follower is not configured yet: supply a Drivetrain for the differential " +
                    "swerve and a tuned Foresight algorithm, then call " +
                    "createFollower(hardwareMap, drivetrain, algorithm)."
        )
    }
}
