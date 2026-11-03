package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.ivy.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher implements Subsystem {
  DcMotorEx flywheel;
  Servo xWheelLeader;
  Servo xWheelFollow;

  // Constants [move to sub-constants later]

  Launcher(HardwareMap hwm) {
    flywheel = hwm.get(DcMotorEx.class, "flywheel");
    xWheelLeader = hwm.get(Servo.class, "xWheelLeader");
    xWheelFollow = hwm.get(Servo.class, "xWheelFollow");
  }
}
