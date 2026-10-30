package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ivy.Subsystem;

public class Launcher implements Subsystem {
    DcMotorEx flywheel;

    //Constants [move to sub-constants later]

    Launcher(HardwareMap hwm) {
        flywheel = hwm.get(DcMotorEx.class, "flywheel");
    }
}
