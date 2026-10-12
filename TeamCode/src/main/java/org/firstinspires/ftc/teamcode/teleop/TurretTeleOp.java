package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ivy.IvyOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Turret bring-up OpMode, and the worked example of the three ways a subsystem gets driven under
 * Ivy:
 *
 * <ul>
 * <li>{@code periodic()} -- the azimuth PID, running every loop no matter what else is scheduled.</li>
 * <li>a default command -- stick jogging, which comes back on its own after anything interrupts it.</li>
 * <li>bound commands -- the D-pad presets, which take the turret, finish once they are on target,
 *     and hand it back.</li>
 * </ul>
 *
 * <p>Needs a {@code turretMotor} in the robot configuration.
 */
@TeleOp(name = "Turret Test", group = "Test")
public class TurretTeleOp extends IvyOpMode {

    /** How fast the right stick walks the turret target, in degrees per second at full deflection. */
    public static final double MANUAL_RATE_DEG_PER_SEC = 90.0;

    private Turret turret;

    @Override
    protected void onInit() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = register(new Turret(hardwareMap));

        setDefaultCommand(turret.jog(() -> gamepad1.right_stick_x * MANUAL_RATE_DEG_PER_SEC));

        telemetry.addData("Status", "Ready. D-pad aims, right stick jogs, A holds.");
    }

    @Override
    protected void onStart() {
        bindings.whenPressed(() -> gamepad1.dpad_up, turret.goToAngle(0))
                .whenPressed(() -> gamepad1.dpad_right, turret.goToAngle(90))
                .whenPressed(() -> gamepad1.dpad_down, turret.goToAngle(180))
                .whenPressed(() -> gamepad1.dpad_left, turret.goToAngle(-90))
                .whenPressed(() -> gamepad1.a, turret.hold());
    }

    @Override
    protected void onLoop() {
        telemetry.addData("Target (deg)", turret.getTargetAngleDegrees());
        telemetry.addData("Current (deg)", turret.getCurrentAngleDegrees());
        telemetry.addData("Power", turret.getPowerSetpoint());
        telemetry.addData("At Target", turret.atTarget());
    }
}
