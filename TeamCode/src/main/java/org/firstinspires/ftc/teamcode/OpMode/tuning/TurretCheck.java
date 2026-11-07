package org.firstinspires.ftc.teamcode.OpMode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Bench check for the turret, in this order (constants in SubsystemConstants.Turret):
 *
 * <ul>
 * <li>Voltage range: in init, turn the turret a full turn by hand. "Lowest/highest voltage" are
 *     ANALOG_MIN/MAX_VOLTAGE.</li>
 * <li>Direction: turn the turret CCW by hand (viewed from above). "Angle" should increase; if
 *     not, flip ENCODER_INCREASES_CCW.</li>
 * <li>Offset: point the turret at the robot's front by hand; "Raw" is ANGLE_OFFSET_DEGREES.</li>
 * <li>Servos: dpad up/left/down/right sends the turret to 0/90/180/-90 degrees (front, left, back,
 *     right). If the servos fight each other (buzz, little movement), flip FOLLOWER_REVERSED. If
 *     the turret spins away instead of settling, flip POWER_REVERSED.</li>
 * <li>Tuning: the "Turret PID Tuner" OpMode (procedure in TUNING.md): F (servo deadband) first,
 *     then KV with P at 0, then P/D.</li>
 * </ul>
 */
@TeleOp(name = "Turret Check", group = "Tuning")
public class TurretCheck extends LinearOpMode {

    @Override
    public void runOpMode() {
        Telemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Turret turret = new Turret(hardwareMap);

        double lowestVoltage = Double.POSITIVE_INFINITY;
        double highestVoltage = Double.NEGATIVE_INFINITY;
        boolean active = false;

        while (opModeInInit() || opModeIsActive()) {
            double voltage = turret.getEncoderVoltage();
            lowestVoltage = Math.min(lowestVoltage, voltage);
            highestVoltage = Math.max(highestVoltage, voltage);

            if (opModeIsActive()) {
                if (gamepad1.dpad_up) { turret.setTargetAngleDegrees(0); active = true; }
                if (gamepad1.dpad_left) { turret.setTargetAngleDegrees(90); active = true; }
                if (gamepad1.dpad_down) { turret.setTargetAngleDegrees(180); active = true; }
                if (gamepad1.dpad_right) { turret.setTargetAngleDegrees(-90); active = true; }

                // servos stay off until a target is picked, so the turret can be turned by hand
                if (active) {
                    turret.periodic();
                }
            }

            tele.addData("Voltage", "%.3f", voltage);
            tele.addData("Lowest voltage", "%.3f", lowestVoltage);
            tele.addData("Highest voltage", "%.3f", highestVoltage);
            tele.addData("Raw", "%.1f", turret.getRawEncoderDegrees());
            tele.addData("Angle", "%.1f", turret.getCurrentAngleDegrees());
            tele.addData("Target", active
                    ? String.format("%.0f", turret.getTargetAngleDegrees())
                    : "press a dpad direction");
            tele.addData("Power", "%.2f", turret.getPowerSetpoint());
            tele.addData("At target", turret.isAtTarget());
            tele.update();
        }
    }
}
