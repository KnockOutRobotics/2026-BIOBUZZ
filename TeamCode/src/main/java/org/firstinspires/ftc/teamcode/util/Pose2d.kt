package org.firstinspires.ftc.teamcode.util

import android.annotation.SuppressLint
import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.sqrt

class Pose2d @JvmOverloads constructor(
    var x: Double,
    var y: Double,
    var heading: Double = 0.0
) : Cloneable {

    fun add(p1: Pose2d) {
        this.x += p1.x
        this.y += p1.y
        this.heading += p1.heading
    }

    fun isNaN(): Boolean = x.isNaN() || y.isNaN() || heading.isNaN()

    fun getDistanceFromPoint(newPoint: Pose2d): Double { // distance equation
        val dx = x - newPoint.x
        val dy = y - newPoint.y
        return sqrt(dx * dx + dy * dy)
    }

    fun getErrorInX(newPoint: Pose2d): Double { // distance equation
        return abs(x - newPoint.x)
    }

    fun getErrorInY(newPoint: Pose2d): Double { // distance equation
        return abs(y - newPoint.y)
    }

    fun clipAngle() {
        heading = clipAngle(heading)
    }

    // fun convertPose2D(pose: Pose2D): Pose2d {
    //     return Pose2d(pose.getX(), pose.getY(), pose.getHeading())
    // }

    public override fun clone(): Pose2d = Pose2d(x, y, heading)

    @SuppressLint("DefaultLocale")
    override fun toString(): String = String.format("(%.3f, %.3f, %.3f", x, y, heading)

    companion object {
        @JvmStatic
        fun clipAngle(angle: Double): Double {
            var a = angle
            while (abs(a) > Math.PI) {
                a -= Math.PI * 2.0 * sign(a)
            }
            return a
        }
    }
}
