package org.firstinspires.ftc.teamcode.OpMode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ivy.IvyOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;

/**
 * Qualifier 2 competition TeleOp. Subsystems are added here as they are finished.
 *
 * <p>Controls (gamepad1):
 * <ul>
 * <li>Left stick: drive. Right stick x: turn.</li>
 * <li>Options/start: re-zero the heading.</li>
 * <li>Right bumper (hold): nectar intake.</li>
 * <li>Left bumper (hold): pollen intake.</li>
 * </ul>
 *
 * <p>Drives through {@link SwerveDrive} until the Pedro constants are tuned; then this can switch
 * to {@code PedroDrive} the way {@link PedroSwerveTeleOp} does.
 */
@TeleOp(name = "Q2 TeleOp", group = "Main")
public class Q2Teleop extends IvyOpMode {

    private SwerveDrive swerve;
    private Intake intake;

    @Override
    protected void onInit() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        swerve = register(new SwerveDrive(telemetry, hardwareMap));
        intake = register(new Intake(hardwareMap));

        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-170.5, 42.023, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        swerve.updateOdo(odo);

        // Same stick signs as SwerveTeleOp; SwerveDrive applies the speed scalars.
        setDefaultCommand(swerve.driveCommand(
                () -> gamepad1.left_stick_x,
                () -> gamepad1.left_stick_y,
                () -> -gamepad1.right_stick_x
        ));

        telemetry.addData("Status", "Ready. Run 'Module Zeroing' if wheels are not aligned.");
    }

    @Override
    protected void onStart() {
        bindings.whenPressed(
                        () -> gamepad1.options || gamepad1.start,
                        swerve.resetIMUCommand().then(Commands.instant(() -> gamepad1.rumble(500))))
                .whileHeld(() -> gamepad1.right_bumper, intake.runNectar())
                .whileHeld(() -> gamepad1.left_bumper, intake.runPollen());
    }
}
