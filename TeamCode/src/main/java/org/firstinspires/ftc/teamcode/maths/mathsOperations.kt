package org.firstinspires.ftc.teamcode.maths

import kotlin.math.abs
import kotlin.math.max

object mathsOperations {

    //normalizes the angle given
    @JvmStatic
    fun angleWrap(wrap: Double): Double {
        var angle = wrap

        while (angle <= -180) {
            angle += 360.0
        }
        while (angle > 180) {
            angle -= 360.0
        }
        return angle
    }

    //replaces turning a module by 180 degrees with reversing motor power.
    @JvmStatic
    fun efficientTurn(reference: Double, state: Double, power: Double): DoubleArray {
        var ref = reference
        var pow = power
        var error = ref - state

        while (error > 90) {
            pow *= -1
            ref -= 180.0
            error = ref - state
        }
        while (error < -90) {
            pow *= -1
            ref += 180.0
            error = ref - state
        }

        return doubleArrayOf(ref, pow)
    }

    @JvmStatic
    fun dynamicTurn(error: Double): Boolean = abs(error) > 90

    //converts two degrees of freedom into a differential
    @JvmStatic
    fun diffyConvert(rotate: Double, translate: Double): DoubleArray {
        var m1 = rotate + translate
        var m2 = rotate - translate
        val maxi = max(abs(m1), abs(m2))
        if (maxi > 1) {
            m1 /= abs(maxi)
            m2 /= abs(maxi)
        }
        return doubleArrayOf(m1, m2)
    }

    //math for detecting when an absolute encoder has wrapped around
    @JvmStatic
    fun modWrap(state: Double, wrap: Double, last: Double, ratio: Double): Double {
        var turns = wrap
        val delta = state - last

        if (delta > 180) turns += 1
        if (delta < -180) turns += 1
        if (turns > ratio - 1) turns = 0.0
        if (turns == 0.0) return state / ratio
        return 360 / (turns + 1) + state / ratio
    }

    @JvmStatic
    fun equals(state: Double, equals: Double, thresh: Double): Boolean {
        return abs(state - equals) < thresh
    }

    @JvmStatic
    fun power(x: Double, pow: Double): Double {
        var result = x
        var i = 0
        while (i < pow) {
            result *= result
            i++
        }
        return result
    }
}
