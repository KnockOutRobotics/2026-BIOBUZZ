package org.firstinspires.ftc.teamcode.maths

import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import kotlin.math.abs
import kotlin.math.sign

class PIDcontroller(
    private var Kp: Double,
    private var Kd: Double,
    private var Ki: Double,
    private var Kf: Double,
    private var Kl: Double
) {
    //PID controller class

    private var integralSum = 0.0
    private var out = 0.0
    private var lastError = 0.0

    private var Kv = 0.0
    private var Ka = 0.0
    private var Kstatic = 0.0

    private val KpS = Kp
    private val KdS = Kd
    private val KiS = Ki
    private val KfS = Kf
    private val KlS = Kl

    private val timer = ElapsedTime()

    init {
        timer.reset()
    }

    //calculate
    fun pidOut(error: Double): Double {
        if (abs(error) > 0) {
            var dt = timer.seconds()
            if (dt <= 0) {
                dt = 1e-3
            }
            //integral and derivative values
            val derivative = (error - lastError) / dt
            integralSum += error * dt
            integralSum = Range.clip(integralSum, -Kl, Kl)
            //weight each term so that tuning makes a difference
            out = (Kp * error) + (Kd * derivative) + (Ki * integralSum) + (Kf * sign(error))
            out /= 10
            lastError = error
            timer.reset()
        }
        return out
    }

    fun ffOut(error: Double, velocityTarget: Double, accelerationTarget: Double): Double {
        return pidOut(error) + Kv * velocityTarget + Ka * accelerationTarget + Kstatic
    }

    fun setPIDgains(Kp: Double, Kd: Double, Ki: Double, Kf: Double, Kl: Double) {
        this.Kp = Kp
        this.Kd = Kd
        this.Ki = Ki
        this.Kf = Kf
        this.Kl = Kl
    }

    fun setFFgains(Kv: Double, Ka: Double, Kstatic: Double) {
        this.Kv = Kv
        this.Ka = Ka
        this.Kstatic = Kstatic
    }

    fun toDefault() {
        this.Kp = KpS
        this.Kd = KdS
        this.Ki = KiS
        this.Kf = KfS
        this.Kl = KlS
    }

    fun reset() {
        integralSum = 0.0
        lastError = 0.0
        out = 0.0
        timer.reset()
    }
}
