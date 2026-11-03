package org.firstinspires.ftc.teamcode.subsystems;

import android.content.pm.LauncherApps;

import org.firstinspires.ftc.teamcode.ivy.Subsystem;
import org.firstinspires.ftc.teamcode.util.CRServoPair;
import org.firstinspires.ftc.teamcode.util.VelocityController;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Launcher implements Subsystem {
  DcMotorEx flywheel;
  CRServoPair xWheels;
  CRServo transferPivot;
  CRServo transferRoller;

  Launcher(HardwareMap hwm) {
    flywheel = hwm.get(DcMotorEx.class, "flywheel");
    flywheel.setMode(RunMode.RUN_WITHOUT_ENCODER);
    flywheel.setDirection(Direction.FORWARD);

    xWheels = new CRServoPair(hwm, "xWheelsLeader", "xWheelsFollower", false);

    transferPivot = hwm.get(CRServo.class, "transferPivot");
    transferRoller = hwm.get(CRServo.class, "transferRoller");
  }


   VelocityController flyWheelControler = new VelocityController(
      SubsystemConstants.Launcher.KV, SubsystemConstants.Launcher.KS, SubsystemConstants.Launcher.KP, SubsystemConstants.Launcher.KI, SubsystemConstants.Launcher.I_LIMIT, SubsystemConstants.Launcher.TOLERANCE);

}