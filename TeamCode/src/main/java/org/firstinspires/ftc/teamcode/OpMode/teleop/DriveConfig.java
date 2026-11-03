package org.firstinspires.ftc.teamcode.OpMode.teleop;

import com.acmerobotics.dashboard.config.Config;

/**
 * Live-tunable driver preferences for the TeleOps. {@link Config} reflects over the
 * {@code public static} fields below, so every value here shows up in FTC Dashboard.
 */
@Config
public class DriveConfig {

    private DriveConfig() {
    }

    public static double DRIVE_SPEED_SCALAR = 0.8;
    public static double ROTATION_SPEED_SCALAR = 0.8;
    public static boolean FIELD_CENTRIC = true;
}
