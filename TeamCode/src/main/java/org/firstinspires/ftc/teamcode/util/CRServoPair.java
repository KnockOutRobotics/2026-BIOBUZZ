package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Two continuous servos driven as one; the follower mirrors the leader. */
public class CRServoPair {

  private final CRServo leader;
  private final CRServo follower;

  /** @param followerReversed true when the follower is mounted facing the leader (mirrored) */
  public CRServoPair(HardwareMap hwm, String leaderName, String followerName, boolean followerReversed) {
    leader = hwm.get(CRServo.class, leaderName);
    follower = hwm.get(CRServo.class, followerName);

    follower.setDirection(followerReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
  }

  public void setPower(double power) {
    leader.setPower(power);
    follower.setPower(power);
  }

  public double getPower() {
    return leader.getPower();
  }
}
