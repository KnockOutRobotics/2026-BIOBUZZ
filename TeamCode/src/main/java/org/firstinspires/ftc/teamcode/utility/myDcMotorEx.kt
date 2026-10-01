package org.firstinspires.ftc.teamcode.utility

import com.qualcomm.robotcore.hardware.DcMotorEx
import kotlin.math.abs

/**
 * A [DcMotorEx] wrapper that filters [setPower] writes: powers inside a deadband are
 * forced to zero, and powers that differ from the last written value by less than
 * [powerStep] are dropped entirely (saving bus traffic on tight control loops).
 *
 * Every other member is forwarded to the wrapped motor unchanged.
 */
class myDcMotorEx(private val motor: DcMotorEx) : DcMotorEx by motor {

    private var lastPower = 0.0
    private var powerStep = 0.0
    private var minimum_power = 0.0

    override fun setPower(power: Double) {
        if (abs(power) <= abs(minimum_power)) {
            motor.setPower(0.0)
        } else if (abs(power - lastPower) >= powerStep) {
            motor.setPower(power)
            lastPower = power
        }
    }

    fun setPowerThresholds(minimum_power: Double, powerStep: Double) {
        this.powerStep = powerStep
        this.minimum_power = minimum_power
    }
}
