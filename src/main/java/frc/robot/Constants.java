// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  private static final RobotType robot = RobotType.ROBOT_2024;
  public static final double loopPeriodSecs = 0.02;
  public static final boolean tuningMode = true;

  public static boolean invalidRobotAlertSent = false;

  //This method is to know if the robot code is in real/simulator mode or replay mode
  public static RobotType getRobot() {
    if (!disableHAL && RobotBase.isReal()) {
      if (robot == RobotType.ROBOT_SIM) { // Invalid robot selected
        if (!invalidRobotAlertSent) {
          invalidRobotAlertSent = true;
          System.out.println("Invalid Robot Selected");
        }
        return RobotType.ROBOT_2024;
      } else {
        return robot;
      }
    } else {
      return robot;
    }
  }

  //To get the mode
  public static Mode getMode() {
    switch (getRobot()) {
      case ROBOT_2024:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIM:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  //Folder root in the usb stick
  public static final Map<RobotType, String> logFolders =
      Map.of(RobotType.ROBOT_2024, "/U/logs");

  //All of the robot types we ever get
  public static enum RobotType {
    ROBOT_2024,
    ROBOT_SIM
  }

  //Modes of the code
  public static enum Mode {
    REAL,
    REPLAY,
    SIM
  }

  // Function to disable HAL interaction when running without native libs
  public static boolean disableHAL = false;
  public static void disableHAL() {
    disableHAL = true;
  }

  /** Checks whether the robot the correct robot is selected when deploying. */
  public static void main(String... args) {
    if (robot == RobotType.ROBOT_SIM) {
      System.err.println("Cannot deploy, invalid robot selected: " + robot.toString());
      System.exit(1);
    }
  }
}

