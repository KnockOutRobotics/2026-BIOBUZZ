package org.firstinspires.ftc.teamcode.maths

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin
import kotlin.math.sqrt

class swerveKinematics {

    fun calculate(
        forward: Double,
        strafe: Double,
        rotate: Double,
        imu: Double,
        fieldcentric: Boolean,
        radius: Double
    ): DoubleArray {

        // FIX: Removed "fieldcentric = true;" here so the user can actually toggle it.

        // 1. Field Centric Adjustment
        var strafe1 = strafe
        var forward1 = forward

        if (fieldcentric) {
            // Optimization: Convert once
            val rad = Math.toRadians(imu)
            val sin = sin(rad)
            val cos = cos(rad)

            strafe1 = cos * strafe - sin * forward
            forward1 = sin * strafe + cos * forward
        }

        // 2. Kinematics (Wheel Specific Vectors)
        // Vx = strafe - rot * Ry
        // Vy = forward + rot * Rx

        // Module 1: Right (X=1, Y=-0)
        val mod1strafe = strafe1 - (rotate * radius * -0.0)
        val mod1forward = forward1 + (rotate * radius * 1.0)

        // Module 2: Left (X=-1, Y=-0)
        val mod2strafe = strafe1 - (rotate * radius * -0.0)
        val mod2forward = forward1 + (rotate * radius * -1.0)

        // 3. Extract Speed (Magnitude)
        var mod1speed = sqrt((mod1strafe * mod1strafe) + (mod1forward * mod1forward))
        var mod2speed = sqrt((mod2strafe * mod2strafe) + (mod2forward * mod2forward))

        // 4. Normalize Speeds (Don't exceed 1.0)
        val max1 = max(abs(mod2speed), abs(mod1speed))
        if (abs(max1) > 1) {
            mod1speed /= abs(max1)
            mod2speed /= abs(max1)
        }

        // 5. Extract Angle (Atan2)
        val mod1angle = atan2(mod1strafe, mod1forward) * 180 / Math.PI
        val mod2angle = atan2(mod2strafe, mod2forward) * 180 / Math.PI

        return doubleArrayOf(mod1speed, mod2speed, mod1angle, mod2angle)
    }
}
