package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ivy.Subsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * The differential swerve, driven through Pedro's {@link Follower} in both autonomous and TeleOp,
 * so path following, holding and stick driving all go through the same drivetrain code and the
 * same odometry.
 *
 * <p>{@link #periodic()} updates the follower every loop. Every command that moves the robot
 * requires this subsystem, so a path and the sticks can never drive at once: in TeleOp a
 * {@link #followCommand} (an auto-align, say) interrupts stick drive, and stick drive comes back
 * as the default command when it finishes.
 */
public class PedroDrive implements Subsystem {

    /** Pose at the end of the last OpMode, so TeleOp starts where auto left off. */
    private static Pose lastPose = null;

    private final Follower follower;

    // Field heading the driver's "forward" points along, for field-centric drive.
    private double driverForwardHeading = 0;

    public PedroDrive(HardwareMap hardwareMap, Pose startPose) {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        lastPose = startPose;
    }

    /**
     * Builds the drive at the pose the previous OpMode ended on, or at {@code fallback} if there
     * is none (first OpMode since the Robot Controller app started).
     */
    public static PedroDrive continueFromLastPose(HardwareMap hardwareMap, Pose fallback) {
        return new PedroDrive(hardwareMap, lastPose != null ? lastPose : fallback);
    }

    public Follower follower() {
        return follower;
    }

    public Pose pose() {
        return follower.pose();
    }

    @Override
    public void periodic() {
        follower.update();
        lastPose = follower.pose();
    }

    // =========================================================
    // Ivy commands
    // =========================================================

    /**
     * Stick drive for as long as it is scheduled. Inputs are Pedro's convention: forward,
     * strafe left, and turn CCW are positive.
     */
    public CommandBuilder driveCommand(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn,
                                       BooleanSupplier fieldCentric) {
        return Command.build()
                .requiring(this)
                .setExecute(() -> {
                    double f = forward.getAsDouble();
                    double s = strafe.getAsDouble();
                    double t = turn.getAsDouble();
                    follower.manual(fieldCentric.getAsBoolean()
                            ? ManualDrive.fieldCentric(f, s, t, follower.pose().heading() - driverForwardHeading)
                            : new DrivePowers(f, s, t));
                })
                .setDone(() -> false)
                .setEnd(endCondition -> follower.manual(DrivePowers.zero()));
    }

    /** Follows {@code path}, finishing at its end. */
    public CommandBuilder followCommand(Path path) {
        return PedroCommands.follow(follower, path).requiring(this);
    }

    /** Holds {@code pose} until another drive command takes over. Finishes immediately. */
    public CommandBuilder holdCommand(Pose pose) {
        return Commands.instant(() -> follower.hold(pose)).requiring(this);
    }

    /** Holds wherever the robot is when the command starts. Finishes immediately. */
    public CommandBuilder holdHereCommand() {
        return Commands.instant(() -> follower.hold(follower.pose())).requiring(this);
    }

    /**
     * Makes the robot's current facing the driver's "forward" for field-centric drive. Leaves the
     * pose alone, so odometry (and anything aiming off it) is unaffected. Requirement-free, so it
     * never interrupts driving.
     */
    public CommandBuilder resetDriverForwardCommand() {
        return Commands.instant(() -> driverForwardHeading = follower.pose().heading());
    }

    /** Sets the field heading (radians) that field-centric "forward" points along. */
    public void setDriverForwardHeading(double heading) {
        driverForwardHeading = heading;
    }
}
