package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.ivy.Subsystem;
import org.firstinspires.ftc.teamcode.maths.PIDcontroller;
import org.firstinspires.ftc.teamcode.utility.myDcMotorEx;

public class Turret implements Subsystem {
    //do we want positional PID turret or Velocity PID Turret???
    //Positional determine desired position for us to be inline with target
    //Velocity: adjust velocity to stay in line with turret
    //Problems
    //Positional: harder to keep track of robot position and goal position
    //Velocity: easier for oscillations to occur

    //Velocity: uses a limelight mounted to the turret to determine where it should move.
    //  Whether it needs to move more left or right is determined by offset of
    //  limelight from center of apriltag on the goal
    //  this ends in us setting up a velocity PID with the velocity factor
    //  changing based on position

    //Positional: uses the robots pose on the field and the known pose of the goal to determine
    //  the angle that the turret should be at. this gives us a target pos (angle) for our turret
    //  to be at. with a positional PID to do this. This also lets us mount vision elsewhere.
    //  making it easier hardware wise since the April tag is almost directly above the
    //  robot by 3 feet.

    //We are using a positional Controller YAY

    //I guess we should use a PIDF controller
    private double targetAngleDegrees; //degreesdfCoefficients.)
    private double currentAngleDegrees;
    private PIDFController pidController = new PIDFController(
            SubsystemConstants.Turret.P,
            SubsystemConstants.Turret.I,
            SubsystemConstants.Turret.D,
            SubsystemConstants.Turret.F
    );

    //motors be important
    private DcMotorEx azimuthMotor;
    private double power;

    public Turret(HardwareMap hardwareMap){
        azimuthMotor = new myDcMotorEx(hardwareMap.get(DcMotorEx.class, "turretMotor"));
        azimuthMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoder();

        currentAngleDegrees = getCurrentAngleDegrees();
        targetAngleDegrees = 0;

        pidController.setTolerance(10);
    }

    private void resetEncoder(){
        azimuthMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        azimuthMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    ///set target position
    /// @param targetAngle target angle in degrees
    public void setTargetAngleDegrees(double targetAngle) {
        targetAngleDegrees = targetAngle;
    }

    public double getTargetAngleDegrees(){
        return targetAngleDegrees;
    }

    public double getCurrentAngleDegrees() {
        return azimuthMotor.getCurrentPosition() / SubsystemConstants.Turret.TICKS_PER_REV * 360.0;
    }

    @Override
    public void periodic(){
        power = pidController.calculate(getCurrentAngleDegrees(), targetAngleDegrees);
        azimuthMotor.setPower(power);
    }

    public double getPowerSetpoint() {
        return power;
    }

    // =========================================================
    // Ivy commands
    //
    // The PID lives in periodic(), which IvyOpMode runs every loop regardless of what is
    // scheduled, so a command only has to move the target. Requiring this subsystem is what stops
    // two commands aiming the azimuth at once.
    // =========================================================

    /** Ivy counterpart of {@link #setTargetAngleDegrees(double)}. */
    public CommandBuilder goToAngle(double targetAngle) {
        return Commands.instant(() -> setTargetAngleDegrees(targetAngle)).requiring(this);
    }

    /**wrap around funciotn
     * @param target angle
     * takes shortest path to angle and sees if it goes outside of bounds
     * if it does send turret to complementry angle of current position
     * then tries again.
     * @retruns a registered command to either continue to target or a new commadn to go
     * to complementry angle and then targeted angle
     */


    //TODO
    //1. encoder for current position
    //2. wrap around logic
    //3. aiming logic. how to set target atngle
    //4. goal coordinate for aiming
    //5. tuning
    //6. opmode api


    //absolute encoder
    //need to have a total encoder reading too
    //this be helpful for wrap around
    //absolute encoder help with angle according to robot
    //and zeroed angle
    //total angle help with wrap around
    //-# doesn't exist for wraparoudn and compliment angle math
    //when using a aboslute encoder
    private double totalPostition = 0;
    private double lastPosition = 0;
    private void updateTotalPosition(){
        
    }
}
