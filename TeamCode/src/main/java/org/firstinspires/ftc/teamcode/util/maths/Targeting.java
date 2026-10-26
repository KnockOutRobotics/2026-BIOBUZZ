package org.firstinspires.ftc.teamcode.util.maths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import org.jetbrains.annotations.TestOnly;

import java.util.ArrayList;

class Targeting {

  // need pose and field.
  // I just want this to have a simple function inside that says
  // what angle we should be at for our target.
  // pass in a pose to the targeting object
  // either have it update a set var or constantly call to return angle
  //
  // optionally keep in turret mechanism. meas passign pose from drive tho

  //use pedro coordinate system
  //0-144 up an right with originon the bottom left corner of field.
  Targeting(Pose robotPose) {
  }

  PoseFactory poseFactory = PoseFactory.degrees();

  //label hardcoded value for bottom red goal and generate others via mirror
  Pose bottomRed = poseFactory.of(58, 55, 0);
  Pose topRed = poseFactory.of(58, mirror(55), 0);
  Pose bottomBlue = poseFactory.of(mirror(58), 55, 0);
  Pose topBlue = poseFactory.of(mirror(58), mirror(55), 0);

  //returns angle from robot to goal
  double getGlobalAngleDegrees(Pose robot, Pose goal) {
    double x = robot.x() - goal.x();
    double y = robot.y() - goal.y();

    double angle = Math.atan(y / x);
    if (robot.y() > 72) {
      angle = +180;
    }

    return angle;
  }

  //returns angle turret should be at
  //normalized
  double getRobotAngleDegrees(Pose robot, Pose goal){
    double angle = getRobotAngleDegrees(robot, goal) - robot.heading();

    //normalize
    while(!(angle >= 0 && angle <= 360)){
      if (angle > 360) {
        angle =- 360;
      }
      else {
        angle =+ 360;
      }
    }
    return angle;
  }


  double mirror(double n) {
    return 72 + (72 - n);
  }

  //test fucntion because mine won't work
  boolean test(double function, double expected) {
    return function == expected ? true : false;
  }

  //run test function | takes test in an array
  //iterates to see if one fails,
  //if fail outputs to console
  void runTest(boolean[] tests) {
    for (int i = 0; i < tests.length; i++) {
      if (tests[i] == false) {
        System.out.println("test failed on: " + tests[i]);
      }
    }
    System.out.println("all test passed");
  }
}

  //==================

