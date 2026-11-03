package org.firstinspires.ftc.teamcode.util.maths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

public class Targeting {

  // need pose and field.
  // I just want this to have a simple function inside that says
  // what angle we should be at for our target.
  // pass in a pose to the targeting object
  // either have it update a set var or constantly call to return angle
  //
  // optionally keep in turret mechanism. meas passign pose from drive tho

  //use pedro coordinate system
  //0-144 up an right with originon the bottom left corner of field.
  public Targeting() {
  }

  PoseFactory poseFactory = PoseFactory.degrees();

  //label hardcoded value for bottom red goal and generate others via mirror
  public Pose bottomRed = poseFactory.of(58, 55, 0);
  public Pose topRed = poseFactory.of(58, mirror(55), 0);
  public Pose bottomBlue = poseFactory.of(mirror(58), 55, 0);
  public Pose topBlue = poseFactory.of(mirror(58), mirror(55), 0);

  //returns field angle from robot to goal, in degrees (0 = +x, CCW positive)
  public double getGlobalAngleDegrees(Pose robot, Pose goal) {
    double x = goal.x() - robot.x();
    double y = goal.y() - robot.y();

    // atan2 keeps the quadrant, so no special case is needed for either half of the field
    return Math.toDegrees(Math.atan2(y, x));
  }

  //returns angle turret should be at, relative to the robot's heading
  //normalized to [0, 360)
  public double getRobotAngleDegrees(Pose robot, Pose goal){
    // Pedro headings are radians
    double angle = getGlobalAngleDegrees(robot, goal) - Math.toDegrees(robot.heading());

    //normalize
    while (angle >= 360) {
      angle -= 360;
    }
    while (angle < 0) {
      angle += 360;
    }
    return angle;
  }


  double mirror(double n) {
    return 72 + (72 - n);
  }
}
