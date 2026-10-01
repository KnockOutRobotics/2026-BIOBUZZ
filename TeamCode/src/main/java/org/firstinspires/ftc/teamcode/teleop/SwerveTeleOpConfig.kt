package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.config.Config

/**
 * Live-tunable constants. [Config] reflects over the `public static` fields this object's
 * `@JvmField` properties compile down to, so every value here shows up in FTC Dashboard.
 */
@Config
object SwerveTeleOpConfig {

    // =========================================================
    // 1. PID COEFFICIENTS (Edit these live to tune holding power)
    // =========================================================
    @JvmField var Kp = 0.05
    @JvmField var Kd = 0.0
    @JvmField var Ki = 0.0 // Usually 0 for drive modules
    @JvmField var Kf = 0.0
    @JvmField var Kl = 0.0 // Integral limit

    // =========================================================
    // 2. MODULE OFFSETS (Values from your Zeroing OpMode)
    // =========================================================
    // Enter the raw angles you read when wheels are pointing FORWARD
    @JvmField var module1Adjust = 35.0
    @JvmField var module2Adjust = 65.0
    @JvmField var module3Adjust = -140.0

    // =========================================================
    // 3. DRIVER PREFERENCES
    // =========================================================
    @JvmField var DRIVE_SPEED_SCALAR = 0.8
    @JvmField var ROTATION_SPEED_SCALAR = 0.8
    @JvmField var FIELD_CENTRIC = true
    @JvmField var USE_IMU = true

    // Set to -1 if rotating the robot clockwise makes heading DECREASE
    // Defaulting to -1 to match the GoBilda Pinpoint yaw direction used in this bot.
    @JvmField var IMU_POLARITY = -1.0

    // =========================================================
    // 4. KINEMATICS / GEOMETRY
    // =========================================================
    // Effective radius of rotation. Increase this if the robot spins too fast/slow relative to drive.
    // In your old code this was implicitly "1.0".
    @JvmField var ROBOT_RADIUS = 1.0

    // Example: if facing driver and translation is mirrored, try +90 or -90.
    @JvmField var HEADING_FRAME_OFFSET_DEG = 0.0

    @JvmField var HEADING_LOCK_KP = 0.017
    @JvmField var HEADING_LOCK_KI = 0.00000
    @JvmField var HEADING_LOCK_KD = 0.0001
    @JvmField var HEADING_LOCK_KF = 0.00000
    @JvmField var HEADING_LOCK_KL = 0.0
    @JvmField var HEADING_LOCK_DEADBAND = 0.2
    @JvmField var P = 0.0008
}
