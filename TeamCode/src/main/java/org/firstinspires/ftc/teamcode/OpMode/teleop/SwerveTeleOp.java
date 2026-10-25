package org.firstinspires.ftc.teamcode.OpMode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ivy.IvyOpMode;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;

/**
 * Driver-controlled swerve.
 *
 * <p>The loop itself lives in {@link IvyOpMode}: the drive's {@code periodic()} runs the module
 * PIDs every loop, stick drive is the drive's default command, and the IMU reset is a button
 * binding. Nothing here calls the hardware directly, so a command (an aim routine, an auto-align)
 * can take the drive for a moment and the sticks pick it back up by themselves.
 */
@TeleOp(name = "Swerve TeleOp (Final)", group = "Main")
public class SwerveTeleOp extends IvyOpMode {

    private SwerveDrive swerve;
    private GoBildaPinpointDriver odo;

    private long lastLoopTime = 0;

    @Override
    protected void onInit() {
        // Telemetry to the phone and the Dashboard.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        swerve = register(new SwerveDrive(telemetry, hardwareMap));

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-170.5, 42.023, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        swerve.updateOdo(odo);

        // Stick drive whenever no other command wants the drive. SwerveDrive applies the
        // Dashboard speed scalars, so only the raw stick values and their signs belong here.
        setDefaultCommand(swerve.driveCommand(
                () -> gamepad1.left_stick_x,
                () -> gamepad1.left_stick_y,
                () -> -gamepad1.right_stick_x
        ));

        telemetry.addData("Status", "Ready. Run 'Module Zeroing' if wheels are not aligned.");
    }

    @Override
    protected void onStart() {
        // resetIMUCommand() takes no requirements, so re-zeroing never interrupts the drive.
        bindings.whenPressed(
                () -> gamepad1.options || gamepad1.start,
                swerve.resetIMUCommand().then(Commands.instant(() -> gamepad1.rumble(500)))
        );
    }

    @Override
    protected void onLoop() {
        // The drive reads the Pinpoint heading itself inside drive(), so there is no odo.update()
        // here any more -- a second read per loop only cost loop time.

        long currentTime = System.currentTimeMillis();
        long loopTime = currentTime - lastLoopTime;
        lastLoopTime = currentTime;

        telemetry.addData("Loop Time (ms)", loopTime);
        telemetry.addData("Frequency (Hz)", 1000.0 / loopTime);
    }
}
