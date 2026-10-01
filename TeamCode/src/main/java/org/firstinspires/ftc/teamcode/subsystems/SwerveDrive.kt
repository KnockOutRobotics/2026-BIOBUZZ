package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.maths.PIDcontroller
import org.firstinspires.ftc.teamcode.maths.mathsOperations
import org.firstinspires.ftc.teamcode.maths.swerveKinematics
import org.firstinspires.ftc.teamcode.teleop.SwerveTeleOpConfig
import org.firstinspires.ftc.teamcode.utility.myDcMotorEx

import kotlin.math.abs

class SwerveDrive(private var telemetry: Telemetry, hardwareMap: HardwareMap) {

    private var odo: GoBildaPinpointDriver? = null

    private val mod1m1: myDcMotorEx
    private val mod1m2: myDcMotorEx
    private val mod2m1: myDcMotorEx
    private val mod2m2: myDcMotorEx
    private val mod1E: AnalogInput
    private val mod2E: AnalogInput

    // PIDs
    private val mod1PID = PIDcontroller(0.0, 0.0, 0.0, 0.0, 0.0)
    private val mod2PID = PIDcontroller(0.0, 0.0, 0.0, 0.0, 0.0)

    private val kinematics = swerveKinematics()

    private var imuOffset = 0.0
    private var imuZeroed = false

    // Heading correction (ported closely to user's previous implementation)
    private val headingDt = ElapsedTime()
    private val headingReset = ElapsedTime()
    private var headingTarget = 0.0
    private var lastHeadingError = 0.0
    private var headingTargetSet = false
    private var HEADING_LOCK_DEADBAND = SwerveTeleOpConfig.HEADING_LOCK_DEADBAND

    // Safety Variables
    private var initialized = false
    private var lastGoodHeading = 180.0 // Failsafe for IMU singularity

    init {
        // --- Hardware Mapping ---
        mod1m1 = myDcMotorEx(hardwareMap.get(DcMotorEx::class.java, "mod1m1"))
        mod1m2 = myDcMotorEx(hardwareMap.get(DcMotorEx::class.java, "mod1m2"))
        mod2m1 = myDcMotorEx(hardwareMap.get(DcMotorEx::class.java, "mod2m1"))
        mod2m2 = myDcMotorEx(hardwareMap.get(DcMotorEx::class.java, "mod2m2"))

        mod1E = hardwareMap.get(AnalogInput::class.java, "mod1E")
        mod2E = hardwareMap.get(AnalogInput::class.java, "mod2E")

        // --- Motor Defaults ---
        val allMotors = arrayOf(mod1m1, mod1m2, mod2m1, mod2m2)
        for (motor in allMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
            motor.setDirection(DcMotorSimple.Direction.FORWARD)
            motor.setPowerThresholds(0.05, 0.0)
        }

        // --- Motor Reversals ---
        // Only bottom motors are reversed (Coaxial standard)
        mod1m2.setDirection(DcMotorSimple.Direction.FORWARD)
        mod2m2.setDirection(DcMotorSimple.Direction.FORWARD)
    }

    fun driveWithConfig(strafe: Double, forward: Double, rot: Double) {
        drive(
            strafe, forward, rot,
            SwerveTeleOpConfig.module1Adjust,
            SwerveTeleOpConfig.module2Adjust,
            SwerveTeleOpConfig.module3Adjust,
            SwerveTeleOpConfig.Kp,
            SwerveTeleOpConfig.Kd,
            SwerveTeleOpConfig.Ki,
            SwerveTeleOpConfig.Kf,
            SwerveTeleOpConfig.Kl,
            SwerveTeleOpConfig.FIELD_CENTRIC,
            SwerveTeleOpConfig.IMU_POLARITY,
            SwerveTeleOpConfig.ROBOT_RADIUS
        )
    }

    fun drive(
        strafe: Double, forward: Double, rot: Double,
        m1Offset: Double, m2Offset: Double, m3Offset: Double,
        Kp: Double, Kd: Double, Ki: Double, Kf: Double, Kl: Double,
        fieldCentric: Boolean, imuPolarity: Double, robotRadius: Double
    ) {
        var strafeIn = strafe
        var forwardIn = forward
        var rotIn = rot

        HEADING_LOCK_DEADBAND = SwerveTeleOpConfig.HEADING_LOCK_DEADBAND

        val odo = this.odo
        if (odo != null) {
            odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING)
            // Auto-zero IMU once at startup using the same math as getHeading(), so heading reads 0 at startup.
            if (!imuZeroed) {
                val rawDeg = Math.toDegrees(odo.getHeading(AngleUnit.RADIANS))
                imuOffset = rawDeg * SwerveTeleOpConfig.IMU_POLARITY - SwerveTeleOpConfig.HEADING_FRAME_OFFSET_DEG
                imuZeroed = true
            }
        }

        if (forwardIn.isNaN() || strafeIn.isNaN() || rotIn.isNaN()) {
            forwardIn = 0.0; strafeIn = 0.0; rotIn = 0.0
        }

        // 1. Update PID Coefficients
        mod1PID.setPIDgains(Kp, Kd, Ki, Kf, Kl)
        mod2PID.setPIDgains(Kp, Kd, Ki, Kf, Kl)

        // 2. Read Sensors (Volts -> Degrees)
        var mod1P = readEncoder(mod1E, m1Offset)
        var mod2P = readEncoder(mod2E, m2Offset)

        var heading = 0.0
        val needsHeading = fieldCentric || (initialized && abs(rotIn) < HEADING_LOCK_DEADBAND)
        if (needsHeading) {
            if (odo == null) {
                heading = lastGoodHeading
                telemetry.addData("WARNING", "IMU UNAVAILABLE - USING FALLBACK")
            } else {
                val rawHeading = getHeading(imuPolarity)

                if (rawHeading.isNaN()) {
                    heading = lastGoodHeading // Use last known safe angle
                    telemetry.addData("WARNING", "IMU NAN DETECTED - USING FALLBACK")
                } else {
                    heading = rawHeading
                    lastGoodHeading = heading // Update history
                }
            }
        }

        var headingCorrection = 0.0
        if (needsHeading && odo != null) {
            // First-time capture after init/reset so we don't drive toward 0 by default
            if (!headingTargetSet) {
                headingTarget = heading
                headingTargetSet = true
                headingReset.reset()
                headingDt.reset()
            }
            if (abs(rotIn) > 0.05) {
                headingReset.reset()
            }
            if (headingReset.milliseconds() < 200) {
                headingTarget = heading
                lastHeadingError = 0.0
            } else {
                val headingError = AngleUnit.normalizeDegrees(headingTarget - heading)
                var dt = headingDt.seconds()
                if (dt <= 0) dt = 1e-3

                headingCorrection = SwerveTeleOpConfig.HEADING_LOCK_KP * headingError +
                        SwerveTeleOpConfig.HEADING_LOCK_KD * (headingError - lastHeadingError) / dt

                lastHeadingError = headingError
            }
            headingDt.reset()
        }

        rotIn -= headingCorrection
        rotIn = rotIn.coerceIn(-1.0, 1.0)

        // 5. Calculate Kinematics (Vectors)
        val output = kinematics.calculate(forwardIn, -strafeIn, -rotIn, heading, fieldCentric, robotRadius)

        val mod1power = output[0]
        val mod2power = output[1]

        // References
        var mod1reference = output[2]
        var mod2reference = output[3]

        // 6. Locking Logic
        if (forwardIn != 0.0 || strafeIn != 0.0 || rotIn != 0.0 || !initialized) {
            initialized = true
        }

        // 7. Efficient Turn & PID
        // Angle Wrap
        mod1P = mathsOperations.angleWrap(mod1P)
        mod2P = mathsOperations.angleWrap(mod2P)

        mod1reference = mathsOperations.angleWrap(mod1reference)
        mod2reference = mathsOperations.angleWrap(mod2reference)

        // Efficient Turn
        val m1Eff = mathsOperations.efficientTurn(mod1reference, mod1P, mod1power)
        val m2Eff = mathsOperations.efficientTurn(mod2reference, mod2P, mod2power)

        // PID Calculation
        val m1PID = mod1PID.pidOut(AngleUnit.normalizeDegrees(m1Eff[0] - mod1P))
        val m2PID = mod2PID.pidOut(AngleUnit.normalizeDegrees(m2Eff[0] - mod2P))

        // 8. Differential Mixing (PID + Drive Power)
        val m1Out = mathsOperations.diffyConvert(-m1PID, m1Eff[1])
        val m2Out = mathsOperations.diffyConvert(-m2PID, m2Eff[1])

        // 9. Output
        mod1m1.setPower(m1Out[0]); mod1m2.setPower(m1Out[1])
        mod2m1.setPower(m2Out[0]); mod2m2.setPower(m2Out[1])

        // 10. Telemetry
        telemetry.addData("Heading", heading)
        telemetry.addData("M1 Angle/Ref", "%.1f / %.1f", mod1P, m1Eff[0])
        telemetry.addData("M2 Angle/Ref", "%.1f / %.1f", mod2P, m2Eff[0])
    }

    private fun readEncoder(enc: AnalogInput, offset: Double): Double {
        // Parentheses here are critical! Do not remove.
        val raw = (enc.voltage - 0.043) / 3.1 * 360.0
        return AngleUnit.normalizeDegrees(raw - offset)
    }

    private fun getHeading(polarity: Double): Double {
        val headingDeg = Math.toDegrees(odo!!.getHeading(AngleUnit.RADIANS))
        return AngleUnit.normalizeDegrees(
            headingDeg * polarity - imuOffset - SwerveTeleOpConfig.HEADING_FRAME_OFFSET_DEG
        )
    }

    fun resetIMU() {
        val odo = this.odo
        if (odo != null) {
            val rawDeg = Math.toDegrees(odo.getHeading(AngleUnit.RADIANS))
            imuOffset = rawDeg * SwerveTeleOpConfig.IMU_POLARITY - SwerveTeleOpConfig.HEADING_FRAME_OFFSET_DEG
        }
        headingTarget = Math.PI
        lastHeadingError = 0.0
        headingDt.reset()
        headingReset.reset()
        headingTargetSet = false
        lastGoodHeading = 0.0
        initialized = false // re-arm lock on next driver input
        imuZeroed = true
    }

    fun tuneModules(
        targetAngle: Double, m1Offset: Double, m2Offset: Double, m3Offset: Double,
        Kp: Double, Kd: Double, Ki: Double, Kf: Double, Kl: Double
    ) {
        // Update PIDs
        mod1PID.setPIDgains(Kp, Kd, Ki, Kf, Kl)
        mod2PID.setPIDgains(Kp, Kd, Ki, Kf, Kl)

        // Read Current Angles
        var mod1P = readEncoder(mod1E, m1Offset)
        var mod2P = readEncoder(mod2E, m2Offset)

        // Wrap/Optimize
        mod1P = mathsOperations.angleWrap(mod1P)
        mod2P = mathsOperations.angleWrap(mod2P)

        val m1Eff = mathsOperations.efficientTurn(targetAngle, mod1P, 0.0)
        val m2Eff = mathsOperations.efficientTurn(targetAngle, mod2P, 0.0)

        val m1PID = mod1PID.pidOut(AngleUnit.normalizeDegrees(m1Eff[0] - mod1P))
        val m2PID = mod2PID.pidOut(AngleUnit.normalizeDegrees(m2Eff[0] - mod2P))

        // Apply ONLY rotation power (no drive)
        val m1Out = mathsOperations.diffyConvert(m1PID, 0.0)
        val m2Out = mathsOperations.diffyConvert(-m2PID, 0.0)

        mod1m1.setPower(m1Out[0]); mod1m2.setPower(m1Out[1])
        mod2m1.setPower(m2Out[0]); mod2m2.setPower(m2Out[1])

        telemetry.addData("Target", targetAngle)
        telemetry.addData("M1 Pos", mod1P)
        telemetry.addData("M2 Pos", mod2P)
    }

    fun updateTelemetry(telem: Telemetry) {
        telemetry = telem
    }

    fun updateOdo(odom: GoBildaPinpointDriver) {
        odo = odom
    }
}
