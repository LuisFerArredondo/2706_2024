// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Util.Team6328.Alert;
import frc.robot.Util.Team6328.Alert.AlertType;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private static final double canErrorTimeThreshold = 0.5; // Seconds to disable alert
  private Command autoCommand;

  private boolean autoMessagePrinted;
  private double autoStart;

  private RobotContainer m_robotContainer;
  private final Timer canErrorTimer = new Timer();
  private final Timer canErrorTimerInitial = new Timer();
  private final Timer disabledTimer = new Timer();

  private final Alert logNoFileAlert = new Alert("No log path set for current robot. Data will NOT be logged.",
      AlertType.WARNING);
  private final Alert logReceiverQueueAlert = new Alert("Logging queue exceeded capacity, data will NOT be logged.",
      AlertType.ERROR);
  private final Alert canErrorAlert = new Alert("CAN errors detected, robot may not be controllable.", AlertType.ERROR);

  // Set the refresh rate of the robot
  public Robot() {
    super(Constants.loopPeriodSecs);
  }

  @Override
  public void robotInit() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    switch (Constants.getMode()) {
      case REAL:
      // Running on a real robot, log to a USB stick ("/U/logs")
        String folder = Constants.logFolders.get(Constants.getRobot());
        if (folder != null) {
          Logger.addDataReceiver(new WPILOGWriter());
        } else {
          logNoFileAlert.set(true);
        }
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.disableDeterministicTimestamps()

    // Start AdvantageKit logger
    Logger.start();


    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
                  "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize(
            (Command command) -> {
              logCommandFunction.accept(command, true);
            });
    CommandScheduler.getInstance()
        .onCommandFinish(
            (Command command) -> {
              logCommandFunction.accept(command, false);
            });
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (Command command) -> {
              logCommandFunction.accept(command, false);
            });

    canErrorTimer.reset();
    canErrorTimer.start();
    canErrorTimerInitial.reset();
    canErrorTimerInitial.start();
    disabledTimer.reset();
    disabledTimer.start();

    System.out.println("[Init] Instantiating RobotContainer");
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    //Check if this improves something
    //Threads.setCurrentThreadPriority(true, 99);

    // Update CAN error alert
    var canStatus = RobotController.getCANStatus();
    if (canStatus.receiveErrorCount > 0 || canStatus.transmitErrorCount > 0) {
      canErrorTimer.reset();
    }
    canErrorAlert.set(
        !canErrorTimer.hasElapsed(canErrorTimeThreshold)
            && canErrorTimerInitial.hasElapsed(canErrorTimeThreshold));

    // Check logging fault
    logReceiverQueueAlert.set(Logger.getReceiverQueueFault());


    // Log list of NT clients
    List<String> clientNames = new ArrayList<>();
    List<String> clientAddresses = new ArrayList<>();
    for (var client : NetworkTableInstance.getDefault().getConnections()) {
      clientNames.add(client.remote_id);
      clientAddresses.add(client.remote_ip);
    }
    Logger.recordOutput("NTClients/Names", clientNames.toArray(new String[clientNames.size()]));
    Logger.recordOutput("NTClients/Addresses", clientAddresses.toArray(new String[clientAddresses.size()]));


    // Print auto duration
    if (autoCommand != null) {
      if (!autoCommand.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.println(
              String.format(
                  "*** Auto finished in %.2f secs ***", Timer.getFPGATimestamp() - autoStart));
        } else {
          System.out.println(
              String.format(
                  "*** Auto cancelled in %.2f secs ***", Timer.getFPGATimestamp() - autoStart));
        }
        autoMessagePrinted = true;
      }
    }

    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    autoStart = Timer.getFPGATimestamp();
    autoMessagePrinted = false;
    autoCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {

    if (autoCommand != null) {
      autoCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
