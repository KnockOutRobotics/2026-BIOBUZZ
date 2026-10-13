package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Swerve;
import com.pedropathing.revhub.drivetrains.SwerveConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemConstants.Drive;

import java.util.ArrayList;
import java.util.List;

/**
 * Pedro Pathing configuration. Pedro 3 builds a {@link Follower} from a {@link Localizer}, a
 * {@link Drivetrain} and an {@link Algorithm}:
 *
 * <ul>
 * <li>Localizer -- the Pinpoint, configured as in {@code SwerveTeleOp}.</li>
 * <li>Drivetrain -- Pedro's {@link Swerve} with two {@link DifferentialPod}s (right and left).</li>
 * <li>Algorithm -- {@link Foresight}, with values from the AutoTune Foresight Tuner.</li>
 * </ul>
 *
 * <p>Robot-specific numbers live in {@link Drive}. Values still {@code NaN} there make the
 * builders below throw with the list of what is missing, instead of driving on garbage.
 */
public class Constants {

    private Constants() {
    }

    public static Localizer createLocalizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, new PinpointConfig(config -> {
            config.name.set("odo");
            config.xPodOffset.set(-170.5);
            config.yPodOffset.set(42.023);
            config.offsetUnits.set(DistanceUnit.MM);
            config.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            config.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            config.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        }));
    }

    public static Drivetrain createDrivetrain(HardwareMap hardwareMap) {
        requireSet("pod positions",
                "POD_X_INCHES", Drive.POD_X_INCHES,
                "POD_Y_INCHES", Drive.POD_Y_INCHES);

        SwerveConfig swerveConfig = new SwerveConfig(config -> {
            config.voltageCompensation.set(Drive.VOLTAGE_COMPENSATION);
            // X_LOCK points the pods at the robot center; with both pods on one axis that
            // doesn't lock anything, so just leave the pods where they are when stopped.
            config.zeroPowerBehavior.set(SwerveConfig.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES);
        });

        DifferentialPod rightPod = new DifferentialPod(hardwareMap, "rightPod",
                Drive.RIGHT_POD_MOTOR_1, Drive.RIGHT_POD_MOTOR_2, Drive.RIGHT_POD_ENCODER,
                Vector2D.cartesian(Drive.POD_X_INCHES, Drive.POD_Y_INCHES),
                Math.toRadians(Drive.RIGHT_POD_ANGLE_OFFSET_DEGREES));
        DifferentialPod leftPod = new DifferentialPod(hardwareMap, "leftPod",
                Drive.LEFT_POD_MOTOR_1, Drive.LEFT_POD_MOTOR_2, Drive.LEFT_POD_ENCODER,
                Vector2D.cartesian(Drive.POD_X_INCHES, -Drive.POD_Y_INCHES),
                Math.toRadians(Drive.LEFT_POD_ANGLE_OFFSET_DEGREES));

        return new Swerve(hardwareMap, swerveConfig, rightPod, leftPod);
    }

    /** Foresight in the shape the AutoTune Foresight Tuner prints, fed from {@link Drive}. */
    public static Algorithm createAlgorithm() {
        requireSet("Foresight values (run AutoTune > Foresight Tuner)",
                "FORWARD_TRANSLATIONAL_PRIMARY_P", Drive.FORWARD_TRANSLATIONAL_PRIMARY_P,
                "FORWARD_TRANSLATIONAL_SECONDARY_P", Drive.FORWARD_TRANSLATIONAL_SECONDARY_P,
                "STRAFE_TRANSLATIONAL_PRIMARY_P", Drive.STRAFE_TRANSLATIONAL_PRIMARY_P,
                "STRAFE_TRANSLATIONAL_SECONDARY_P", Drive.STRAFE_TRANSLATIONAL_SECONDARY_P,
                "COAST_KV", Drive.COAST_KV,
                "BRAKE_KV", Drive.BRAKE_KV,
                "HEADING_P", Drive.HEADING_P,
                "HEADING_BRAKING_LINEAR", Drive.HEADING_BRAKING_LINEAR,
                "HEADING_BRAKING_QUADRATIC", Drive.HEADING_BRAKING_QUADRATIC,
                "FORWARD_BRAKING_LINEAR", Drive.FORWARD_BRAKING_LINEAR,
                "FORWARD_BRAKING_QUADRATIC", Drive.FORWARD_BRAKING_QUADRATIC,
                "STRAFE_BRAKING_LINEAR", Drive.STRAFE_BRAKING_LINEAR,
                "STRAFE_BRAKING_QUADRATIC", Drive.STRAFE_BRAKING_QUADRATIC,
                "MAX_ACHIEVABLE_FORWARD_VELOCITY", Drive.MAX_ACHIEVABLE_FORWARD_VELOCITY,
                "MAX_ACHIEVABLE_STRAFE_VELOCITY", Drive.MAX_ACHIEVABLE_STRAFE_VELOCITY,
                "NATURAL_FORWARD_DECELERATION", Drive.NATURAL_FORWARD_DECELERATION,
                "NATURAL_STRAFE_DECELERATION", Drive.NATURAL_STRAFE_DECELERATION);

        return new Foresight(new ForesightConfig(c -> {
            // Below 2.5 in of error the secondary gain is used, above it the primary (tuner default).
            c.forwardTranslational.set(Controller.piecewise(
                    Controller.proportional(Drive.FORWARD_TRANSLATIONAL_SECONDARY_P))
                    .put(2.5, Controller.proportional(Drive.FORWARD_TRANSLATIONAL_PRIMARY_P)));
            c.strafeTranslational.set(Controller.piecewise(
                    Controller.proportional(Drive.STRAFE_TRANSLATIONAL_SECONDARY_P))
                    .put(2.5, Controller.proportional(Drive.STRAFE_TRANSLATIONAL_PRIMARY_P)));

            c.coast.set(Controller.proportionalFeedforward(Drive.COAST_KV));
            c.brake.set(Controller.proportionalFeedforward(Drive.BRAKE_KV));

            c.headingFeedback.set(Controller.proportional(Drive.HEADING_P));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(
                    Drive.HEADING_BRAKING_LINEAR, Drive.HEADING_BRAKING_QUADRATIC));

            c.linearBrakeCoefficients.set(Matrix.diag(
                    Drive.FORWARD_BRAKING_LINEAR, Drive.STRAFE_BRAKING_LINEAR));
            c.quadraticBrakeCoefficients.set(Matrix.diag(
                    Drive.FORWARD_BRAKING_QUADRATIC, Drive.STRAFE_BRAKING_QUADRATIC));

            c.maxAchievableForwardVelocity.set(Drive.MAX_ACHIEVABLE_FORWARD_VELOCITY);
            c.maxAchievableStrafeVelocity.set(Drive.MAX_ACHIEVABLE_STRAFE_VELOCITY);
            c.naturalForwardDeceleration.set(Drive.NATURAL_FORWARD_DECELERATION);
            c.naturalStrafeDeceleration.set(Drive.NATURAL_STRAFE_DECELERATION);
        }));
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(createLocalizer(hardwareMap), createDrivetrain(hardwareMap), createAlgorithm());
    }

    /**
     * Throws if any of the (name, value) pairs is still NaN, naming all of them at once.
     *
     * @param namesAndValues alternating String name, Double value
     */
    private static void requireSet(String what, Object... namesAndValues) {
        List<String> missing = new ArrayList<>();
        for (int i = 0; i < namesAndValues.length; i += 2) {
            if (Double.isNaN((Double) namesAndValues[i + 1])) {
                missing.add((String) namesAndValues[i]);
            }
        }
        if (!missing.isEmpty()) {
            throw new IllegalStateException("Fill in " + what + " in SubsystemConstants.Drive: "
                    + String.join(", ", missing));
        }
    }
}
