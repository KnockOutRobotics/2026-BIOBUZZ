package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Registers the AutoTune procedures (the robot-hosted tuning page). The tuning library finds the
 * static {@code @Tuner} methods here and lists each one on the page.
 *
 * <p>Order on a new robot: check the pods with the "Diffy Pod Check" OpMode, then Pinpoint Tuner,
 * then Foresight Tuner (copy its output into SubsystemConstants.Drive), then Tests.
 */
public class Tuning {

    private Tuning() {
    }

    @Tuner(name = "Pinpoint Tuner")
    static Procedure pinpoint() {
        return new PinpointTuner();
    }

    @Tuner(name = "Foresight Tuner")
    static Procedure foresight() {
        return new ForesightTuner(Constants::createLocalizer, Constants::createDrivetrain);
    }

    @Tuner(name = "Tests")
    static Procedure tests() {
        return new Tests(Constants::createDrivetrain, Constants::createLocalizer, Constants::createAlgorithm);
    }
}
